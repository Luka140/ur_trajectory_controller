import rclpy 
from rclpy.node import Node
# from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, qos_profile_system_default, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String

import numpy as np


class UrController(Node):

    def __init__(self) -> None:
        super().__init__('ur_controller')

        # Declare all parameters
        self.declare_parameter("controller_name", "ur_script")
        self.declare_parameter("goal_names", [""])
        self.declare_parameter("autonomous_execution", True)
        self.declare_parameter("joints", [""])
        self.declare_parameter("check_starting_point", False)

        # Read parameters
        self.controller_name        = self.get_parameter("controller_name").value
        goal_names                  = self.get_parameter("goal_names").value
        self.joints                 = self.get_parameter("joints").value
        self.check_starting_point   = self.get_parameter("check_starting_point").value
        self.autonomous_execution   = self.get_parameter('autonomous_execution').value

        self.starting_point = {}

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        if goal_names is None or len(goal_names) == 0:
            raise Exception('"goal_names" parameter is not set! - There are no waypoints available.')
        
        # If controlled in urscript 
        if 'ur_script' in self.controller_name:
            publish_topic = '/urscript_interface/script_command'
            self.ur_script_interface = self.create_publisher(String,publish_topic, 1)
            self.move_types = []
        
        # Setup trajectory publisher and the trigger subscriber 
        if self.controller_name in ('position_trajectory_controller', 'joint_trajectory_controller', 'scaled_joint_trajectory_controller'):
            publish_topic = f"/{self.controller_name}/joint_trajectory"
            self.publisher_traj = self.create_publisher(JointTrajectory, publish_topic, 1)

        # If desired check whether the starting point is valid/safe position
        if self.check_starting_point:
            # Reads the start limits from the config
            self.check_start_position()

            # Subscribe to joint_states to check whether the actual state is within the start limits only for the first callback
            self.joint_state_msg_received = False
            self.joint_state_sub = self.create_subscription(JointState, 
                                                            "joint_states", 
                                                            self.joint_state_callback, 
                                                            10)

        # initialize starting point status
        self.starting_point_ok = not self.check_starting_point

        # Read all positions from parameters
        self.goals = self.read_waypoint_params(goal_names)

        trigger_next_move_topic = "~/trigger_move"
        self.subscriber_execute_next_move = self.create_subscription(Empty, trigger_next_move_topic, self.execute_next_move, 1)
        
        # Counter to keep track of which goal within the list is next. Goals will loop infinitely
        self.goal_indexer = 0
        
        self.get_logger().info(
            f"Publishing {len(goal_names)} goals on topic '{publish_topic}'"
            f"Waiting for a message on  topic {trigger_next_move_topic} to execute the next move"
        )

    def read_waypoint_params(self, goal_names) -> list[JointTrajectoryPoint]:
        goals = []  # List of JointTrajectoryPoint
        for name in goal_names:
            self.declare_parameter(name, descriptor=ParameterDescriptor(dynamic_typing=True))

            point = JointTrajectoryPoint()

            def get_sub_param(sub_param):
                param_name = name + "." + sub_param
                self.declare_parameter(param_name, [float()])
                param_value = self.get_parameter(param_name).value

                float_values = []

                if len(param_value) != len(self.joints):
                    return [False, float_values]

                float_values = [float(value) for value in param_value]

                return [True, float_values]

            one_ok = False

            [ok, values] = get_sub_param("positions")
            if ok:
                point.positions = values
                one_ok = True

            [ok, values] = get_sub_param("velocities")
            if ok:
                point.velocities = values
                one_ok = True

            [ok, values] = get_sub_param("accelerations")
            if ok:
                point.accelerations = values
                one_ok = True

            [ok, values] = get_sub_param("effort")
            if ok:
                point.effort = values
                one_ok = True

    
            # Specific to URscript commands
            self.declare_parameter(f"{name}.move_type", "")
            self.declare_parameter(f"{name}.velocity", 0.)
            self.declare_parameter(f"{name}.acceleration", 0.)
            self.declare_parameter(f"{name}.movement_duration", [0, 0])
            move_type           = self.get_parameter(f"{name}.move_type").value
            velocity_ur         = self.get_parameter(f"{name}.velocity").value
            acceleration_ur     = self.get_parameter(f"{name}.acceleration").value
            movement_duration   = self.get_parameter(f"{name}.movement_duration").value

            if one_ok:
                if 'ur_script' in self.controller_name:
                    if move_type.strip() in ('movej', 'movel', 'movep'):
                        self.move_types.append(move_type.strip())
                    else:
                        self.get_logger().warn('Move type not valid for URScript')

                    if velocity_ur > 0.:
                        point.velocities = [velocity_ur] * len(self.joints)
                    if acceleration_ur > 0.:
                        point.accelerations = [acceleration_ur] * len(self.joints)

                if movement_duration[0] > 0 or movement_duration[1] > 0:
                    point.time_from_start = Duration(sec=movement_duration[0], nanosec=movement_duration[1])
                goals.append(point)

                self.get_logger().info(f'Goal "{name}" has definition {point}')

            else:
                self.get_logger().warn(
                    f'Goal "{name}" definition is wrong. This goal will not be used. '
                    "Use the following structure: \n<goal_name>:\n  "
                    "positions: [joint1, joint2, joint3, ...]\n  "
                    "velocities: [v_joint1, v_joint2, ...]\n  "
                    "accelerations: [a_joint1, a_joint2, ...]\n  "
                    "effort: [eff_joint1, eff_joint2, ...]"
                )

        if len(goals) < 1:
            self.get_logger().error("No valid goal found. Exiting...")
            exit(1)

        return goals

    def execute_next_move(self, msg):
        if self.check_starting_point and not self.joint_state_msg_received:
            self.get_logger().warn(
                'Start configuration could not be checked! Check "joint_state" topic!'
            )
            return 
        
        if not self.starting_point_ok:
            self.get_logger().warn("Start configuration is not within configured limits!")
            return 

        self.get_logger().info(f"Sending goal {self.goals[self.goal_indexer]}.")
        if not 'ur_script' in self.controller_name:
            traj = JointTrajectory()
            traj.joint_names = self.joints
            traj.points.append(self.goals[self.goal_indexer])
            self.publisher_traj.publish(traj)

        else: 
            # If UR script commands are used
            if self.autonomous_execution:  
                command_str = ''
                for i in range(len(self.goals)):          
                    command_str += f'  {self.create_ur_script_str(i)}\n'
            else:
                command_str = f'  {self.create_ur_script_str(self.goal_indexer)}\n'
            command = String(data=f'def move():\n{command_str}\nend')
            self.ur_script_interface.publish(command)

        self.goal_indexer += 1
        self.goal_indexer %= len(self.goals)
        
        return 

    def create_ur_script_str(self, index):
        move = self.move_types[index]
        goal = self.goals[index]
        # The URScript interface uses a different joint order than joint_states so the positions need to be reordered
        position = [goal.positions[-1], *goal.positions[:-1]]
        cmd = f'{move}('
        pos = np.array2string(np.array(position), separator=", ", max_line_width=10**3, formatter={'float_kind':lambda x: "%.8f" % x})
        if 'movej' in move:
            cmd += f'{pos}, '
        if 'movel' in move or 'movep' in move:
            cmd += f'{pos}, '
        
        if len(goal.accelerations) > 0:
            cmd += f'a={np.mean(goal.accelerations)}, '
        if len(goal.velocities) > 0:
            cmd += f'v={np.mean(goal.velocities)}, '
        
        if (('movej' in move) or ('movel' in move)) and (goal.time_from_start.sec > 0 or goal.time_from_start.nanosec > 0):
            cmd += f't={goal.time_from_start.sec}, '
            # cmd += f't=0, '

        # TODO implement blend radius 
        cmd += 'r=0'
        cmd += ')'
        return cmd 

    def joint_state_callback(self, msg):

        if not self.joint_state_msg_received or not self.starting_point_ok:

            # check start state
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):
                    self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
                    limit_exceeded[idx] = True

            if any(limit_exceeded):
                self.starting_point_ok = False
            else:
                self.starting_point_ok = True

            self.joint_state_msg_received = True
        else:
            return
        
    def check_start_position(self) -> None:
        """
        Reading the limits on the start position.
        The actual position will be checked once a joint state message is first received. 
        """
        # declare nested params
        for name in self.joints:
            param_name_tmp = "starting_point_limits" + "." + name
            self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
            self.starting_point[name] = self.get_parameter(param_name_tmp).value

        for name in self.joints:
            if len(self.starting_point[name]) != 2:
                raise Exception('"starting_point" parameter is not set correctly!')

    
def main(args=None):
    rclpy.init(args=args)

    ur_controller = UrController()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(ur_controller, executor=executor)
    except KeyboardInterrupt:
        pass 
    ur_controller.destroy_node()

    # Avoid stack trace 
    try:
        rclpy.shutdown()
    except rclpy._rclpy_pybind11.RCLError:
        pass 


if __name__ == '__main__':
    main()
    