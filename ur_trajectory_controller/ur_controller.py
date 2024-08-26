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

import numpy as np


class UrController(Node):

    def __init__(self) -> None:
        super().__init__('ur_controller')

        # Declare all parameters
        self.declare_parameter("controller_name", "position_trajectory_controller")
        self.declare_parameter("goal_names", [""])
        self.declare_parameter('movement_duration', [6, 0])
        self.declare_parameter("joints", [""])
        self.declare_parameter("check_starting_point", False)

        # Read parameters
        controller_name             = self.get_parameter("controller_name").value
        goal_names                  = self.get_parameter("goal_names").value
        self.movement_duration      = self.get_parameter("movement_duration").value
        self.joints                 = self.get_parameter("joints").value
        self.check_starting_point   = self.get_parameter("check_starting_point").value
        self.starting_point = {}

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        if goal_names is None or len(goal_names) == 0:
            raise Exception('"goal_names" parameter is not set! - There are no waypoints available.')

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

        # Setup trajectory publisher and the trigger subscriber 
        publish_topic = f"/{controller_name}/joint_trajectory"
        subscriber_topic = f"/{controller_name}/trigger_move"
        self.publisher_traj                 = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.subscriber_execute_next_move   = self.create_subscription(Empty, subscriber_topic, self.execute_next_move, 1)
        
        # Counter to keep track of which goal within the list is next. Goals will loop infinitely
        self.goal_indexer = 0
        
        self.get_logger().info(
            f"Publishing {len(goal_names)} goals on topic '{publish_topic}'"
            f"Waiting for a message on  topic {subscriber_topic} to execute the next move"
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

            if one_ok:
                point.time_from_start = Duration(sec=self.movement_duration[0], nanosec=self.movement_duration[1])
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

        if self.starting_point_ok:
            self.get_logger().info(f"Sending goal {self.goals[self.goal_indexer]}.")

            traj = JointTrajectory()
            traj.joint_names = self.joints
            traj.points.append(self.goals[self.goal_indexer])
            self.publisher_traj.publish(traj)

            self.goal_indexer += 1
            self.goal_indexer %= len(self.goals)

        elif self.check_starting_point and not self.joint_state_msg_received:
            self.get_logger().warn(
                'Start configuration could not be checked! Check "joint_state" topic!'
            )
        else:
            self.get_logger().warn("Start configuration is not within configured limits!")

    def joint_state_callback(self, msg):

        if not self.joint_state_msg_received:

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

    rclpy.spin(ur_controller, executor=executor)
    ur_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    