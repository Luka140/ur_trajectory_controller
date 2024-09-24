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


class MeasurementCoordinator(Node):

    def __init__(self):
        super().__init__('measurement_coordinator')

        self.declare_parameter('trigger_topic', '/trigger_move')
        self.declare_parameter('laser_on_topic', '/laseron')
        self.declare_parameter('laser_off_topic', '/laseroff')
        self.declare_parameter('combine_pcl_topic', '/combine_pointclouds')

        self.declare_parameter("goal_names", [""])

        trigger_topic       = self.get_parameter('trigger_topic').get_parameter_value().string_value
        laser_on_topic      = self.get_parameter('laser_on_topic').get_parameter_value().string_value
        laser_off_topic     = self.get_parameter('laser_off_topic').get_parameter_value().string_value
        combine_cloud_topic = self.get_parameter('combine_pcl_topic').get_parameter_value().string_value

        self.goal_names           = self.get_parameter("goal_names").value
        self.move_index = 0
        self.move_buffer_seconds = 0.3
        self.loop = False

        self.durations, self.laser_setting = self.load_move_settings(self.goal_names)

        self.move_trigger       = self.create_publisher(Empty, trigger_topic, 1)
        self.laser_on_pub       = self.create_publisher(Empty, laser_on_topic, 1)
        self.laser_off_pub      = self.create_publisher(Empty, laser_off_topic, 1)
        self.combine_cloud_pub  = self.create_publisher(Empty, combine_cloud_topic, 1)
        
        self.laser_off_pub.publish(Empty())

        startup_delay = 10
        self.get_logger().info(f"Measurement coordinator initialized, starting in {startup_delay} seconds")
        self.next_move_timer = self.create_timer(startup_delay, self.next_move)
    
    def load_move_settings(self, goal_names):
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
        self.next_move_timer = self.create_timer(move_duration + self.move_buffer_seconds, self.next_move)
        
        self.move_index +=1 
        if self.move_index >= len(self.goal_names):
            if self.loop:
                self.move_index %= len(self.goal_names)
            else:
                self.next_move_timer.cancel()
                self.get_logger().info(f"Finished trajectory.")
                self.combine_cloud_pub.publish(Empty())

    
def main(args=None):
    rclpy.init(args=args)

    coordinator = MeasurementCoordinator()
    executor = MultiThreadedExecutor()

    rclpy.spin(coordinator, executor=executor)
    coordinator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    