import rclpy
from rclpy.node import Node
from keyboard_msgs.msg import Key
from sensor_msgs.msg import JointState
from datetime import datetime

import numpy as np


class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        
        # TODO parameterize later
        self.create_subscription(JointState, 'joint_states', self.update_state, 1)
        self.create_subscription(Key, 'keydown', self.key_callback, 1)

        self.trigger_key = Key.KEY_P
        self.reset_key   = Key.KEY_DELETE   
        self.finish_key  = Key.KEY_RETURN

        self.controller  = 'scaled_joint_trajectory_controller'
        self.movement_duration = 8
        self.filepath = 'src/ur_trajectory_controller/config/trajectories'
        
        # Retain an array of past states - keep it at a max length (nr of rows) of 10 msgs 
        self.states = np.empty((10, 6))
        # index of the row to be replaced
        self.oldest_index = 0 

        self.joint_names = None 
        self.starting_limits = {'shoulder_pan_joint': [-0.2,0.2],
                                'shoulder_lift_joint': [-1.7,-1.4],
                                'elbow_joint': [-1.7,-1.4],
                                'wrist_1_joint': [-1.7,-1.4],
                                'wrist_2_joint': [1.4,1.7],
                                'wrist_3_joint': [-0.2,0.2]}

        # List storing all recorded points 
        self.trajectory = []
    

    def key_callback(self, msg):
        if msg.code == self.trigger_key:
            self.register_position()
        if msg.code == self.reset_key:
            self.reset_trajectory()
        if msg.code == self.finish_key:
            self.write_to_file()

    def update_state(self, msg):
        positions = msg.position
        self.states[self.oldest_index, :] = positions
        self.oldest_index += 1 
        self.oldest_index %= 10
        
        if self.joint_names is None:
            self.joint_names = msg.name

    def register_position(self):
        self.get_logger().info('Registering position')
        average_position = np.mean(self.states, axis=0)
        # self.get_logger().info(f'avg_pos: {average_position}')
        self.trajectory.append(average_position)

    def reset_trajectory(self):
        self.trajectory = []
        self.get_logger().info('The stored trajectory has been reset')

    def write_to_file(self):
        filename = f'{self.filepath}/trajectory_{datetime.now()}.yaml'
        self.get_logger().info(f'Creating config file at {filename}')
        indent = '  '
        with open(filename, 'w') as f:
            # Header
            f.write('publisher_scaled_joint_trajectory_controller:\n')
            f.write(f'{indent}ros__parameters:\n\n')
            f.write(f'{indent*2}controller_name: "{self.controller}"\n\n')

            # Waypoints
            nr_of_waypoints = len(self.trajectory)
            f.write(f'{indent*2}goal_names: {[f"pose{i}" for i in range(nr_of_waypoints)]}\n')

            for i in range(nr_of_waypoints):
                f.write(f'{indent*2}pose{i}:\n')
                f.write(f'{indent*3}positions: {np.array2string(self.trajectory[i], precision=10, separator=",", max_line_width=10**3)}\n')

            # Movement duration
            f.write(f'\n{indent*2}movement_duration: [{int(self.movement_duration)}, {int((self.movement_duration - int(self.movement_duration))/10**9)}]\n\n')

            # Joint names
            f.write(f'{indent*2}joints:\n')
            for joint_name in self.joint_names:
                f.write(f'{indent*3}- {joint_name.strip()}\n')

            # Safe starting positions
            f.write(f'\n{indent*2}check_starting_point: true\n')
            f.write(f'{indent*2}starting_point_limits:\n')
            for joint_name in self.joint_names:
                lim = self.starting_limits.get(joint_name, [-0.2, 0.2])
                f.write(f'{indent*3}{joint_name}: {lim}\n')

        self.get_logger().info(f'Created {filename} with {nr_of_waypoints} trajectory points.\n Resetting trajectory list...')
        self.trajectory = []

def main(args=None):
    rclpy.init(args=args)
    recorder = PathRecorder()

    rclpy.spin(recorder)
    recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
