import rclpy 
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from data_gathering_msgs.srv import RequestPCL


class MeasurementCoordinator(Node):

    def __init__(self):
        super().__init__('measurement_coordinator')

        self.declare_parameter('trigger_topic', '/trigger_move')
        self.declare_parameter('laser_on_topic', '/laseron')
        self.declare_parameter('laser_off_topic', '/laseroff')
        self.declare_parameter('combine_pcl_topic', '/combine_pointclouds')
        
        # Auto loop infinitely loops the trajectory
        self.declare_parameter('auto_loop', False)
        # loop_on_service allows the start of a loop to be triggered from the 'execute_loop' topic 
        self.declare_parameter('loop_on_service', False)

        self.declare_parameter("goal_names", [""])

        trigger_topic       = self.get_parameter('trigger_topic').get_parameter_value().string_value
        laser_on_topic      = self.get_parameter('laser_on_topic').get_parameter_value().string_value
        laser_off_topic     = self.get_parameter('laser_off_topic').get_parameter_value().string_value
        combine_cloud_topic = self.get_parameter('combine_pcl_topic').get_parameter_value().string_value

        self.goal_names     = self.get_parameter("goal_names").value
        self.loop           = self.get_parameter('auto_loop').value
        self.loop_on_service  = self.get_parameter('loop_on_service').value

        if self.loop_on_service and self.loop:
            raise ValueError("'auto_loop' and 'loop_on_service' cannot be True at the same time. ")

        self.move_index = 0
        self.move_buffer_seconds = 0.3

        # Obtain the duration and laser setting of each move from the config file
        self.durations, self.laser_setting = self.load_move_settings(self.goal_names)

        self.move_trigger       = self.create_publisher(Empty, trigger_topic, 1)        # trigger the next move
        self.laser_on_pub       = self.create_publisher(Empty, laser_on_topic, 1)       # turn on the LLS
        self.laser_off_pub      = self.create_publisher(Empty, laser_off_topic, 1)      # turn off the LLS
        self.combine_cloud      = self.create_client(RequestPCL, combine_cloud_topic)      # make mesh_constructor combine and store all pointclouds
        
        # Turn off laser initially
        self.laser_off_pub.publish(Empty())

        # Setting this to true allows the 'execute_loop' topic to start a new trajectory loop
        # Is then turned to false for the duration of a loop
        # Variable is only used if loop_on_service is True 
        self.ready_for_next_loop = True 
        self.latest_cloud = None 
        self.done_rate = self.create_rate(0.5)

        self.startup_delay = 5
        if self.loop_on_service:
            self.create_service(RequestPCL, 'execute_loop', self.trigger_loop, callback_group=MutuallyExclusiveCallbackGroup())

        else:
            self.get_logger().info(f"Measurement coordinator initialized, starting in {self.startup_delay} seconds")
            self.next_move_timer = self.create_timer(self.startup_delay, self.next_move)
    
    def load_move_settings(self, goal_names):
        """
        Parses the trajectory config file and stores the duration and laser on/off setting of each move.
        """
        duration_dict = {}
        laser_setting_dict = {}

        def get_sub_param(param):
            param = f'{name}.{param}'
            self.declare_parameter(param, None, descriptor=ParameterDescriptor(dynamic_typing=True))
            param_val = self.get_parameter(param).value
            return param_val

        for name in goal_names:
            self.declare_parameter(name, descriptor=ParameterDescriptor(dynamic_typing=True))

            duration_param = get_sub_param('movement_duration')
            if duration_param is None:
                self.get_logger().warn(f"No duration set for move {name} - Setting the duration to 20s")
                duration_param = [30, 0]            
            duration_dict[name] = duration_param[0] + duration_param[1] / 10**9

            laser_param = get_sub_param('laser_on')
            if laser_param is None:
                self.get_logger().warn(f"No laser setting for move {name} - Turning laser off for this move")
                laser_param = False
            # laser_on = False if (laser_param != 'true') or (laser_param != 'True') else True 
            laser_setting_dict[name] = laser_param

        return duration_dict, laser_setting_dict

    def next_move(self):
        """
        Triggers the ur_trajectory_controller to send the next move command.
        """
        self.next_move_timer.cancel()
        move = self.goal_names[self.move_index]
        laser_on = self.laser_setting[move]
        move_duration = self.durations[move]

        self.get_logger().info(f"Starting move: {move}")
        if laser_on:
            self.laser_on_pub.publish(Empty())
        else:
            self.laser_off_pub.publish(Empty())
        
        self.move_trigger.publish(Empty())

        # Queue the next move call after the current move duration is exceeded 
        self.next_move_timer = self.create_timer(move_duration + self.move_buffer_seconds, self.next_move)
        
        self.move_index +=1 
        if self.move_index >= len(self.goal_names):
            if self.loop:
                self.move_index %= len(self.goal_names)
            else:
                self.next_move_timer.cancel()
                self.get_logger().info(f"Finished trajectory.")
                cloud_call = self.combine_cloud.call_async(RequestPCL.Request())
                cloud_call.add_done_callback(self.combine_done)
                
                self.move_index %= len(self.goal_names)
                self.done_timer = self.create_timer(move_duration + self.move_buffer_seconds, self.loop_done)

    def loop_done(self):
        self.done_timer.cancel()
        self.ready_for_next_loop = True
    
    def combine_done(self, result):
        success = result.result().success
        # if success:
        self.latest_cloud = result.result().pointcloud
        # else:
            # raise ValueError("Could failed to combine pointcloud")
        

    def trigger_loop(self, request, response):
        """_
        Callback to trigger the start of a trajectory loop from a topic. 
        """
        if self.ready_for_next_loop:
            self.get_logger().info(f"'execute_loop' received, starting in {self.startup_delay} seconds")
            self.next_move_timer = self.create_timer(self.startup_delay, self.next_move)
            self.ready_for_next_loop = False

            # Check every x seconds whether done 
            while not self.ready_for_next_loop or self.latest_cloud is None:
                self.done_rate.sleep()
                        
            response.success = True
            response.pointcloud = self.latest_cloud
            self.latest_cloud = None # Reset for next loop
            self.get_logger().info("Loop finished")
        else:
            self.get_logger().info("Surface scan request received, but the node is not ready to execute request. Discarding request...")
            response.success = False 
            response.message = 'The surface measurement coordinator was not ready to perform the scan, as another was already running.'
        return response 
            

def main(args=None):
    rclpy.init(args=args)

    coordinator = MeasurementCoordinator()
    executor = MultiThreadedExecutor()

    rclpy.spin(coordinator, executor=executor)
    coordinator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    