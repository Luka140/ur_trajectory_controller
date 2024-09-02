import rclpy
from rclpy.node import Node
from keyboard_msgs.msg import Key
from sensor_msgs.msg import JointState
from datetime import datetime

import numpy as np


class PathRecorder(Node):
    MOVE_STC_INDEX = 0
    MOVEJ_INDEX    = 1
    MOVEL_INDEX    = 2
    MOVEP_INDEX    = 3

    def __init__(self):
        super().__init__('path_recorder')
        
        # TODO parameterize later
        self.create_subscription(JointState, 'joint_states', self.update_state, 1)
        self.create_subscription(Key, 'keydown', self.key_callback, 1)

        self.declare_parameter("record_urscript", True)
        record_urscript = self.get_parameter("record_urscript").value 

        self.move_p_trigger = Key.KEY_P
        self.move_l_trigger = Key.KEY_L
        self.move_j_trigger = Key.KEY_J
        self.scaled_trajectory_trigger = Key.KEY_S

        self.reset_key   = Key.KEY_DELETE   
        self.finish_key  = Key.KEY_RETURN

        if record_urscript:
            self.controller = 'ur_script'
        else:
            self.controller  = 'scaled_joint_trajectory_controller'

        # Velocities and accelerations in rad/s 
        self.default_velocity = {'movej':0.15,
                                 'movel':0.05,
                                 'movep':0.05
                                 }
        self.default_acceleration = {'movej':0.1,
                                     'movel':0.1,
                                     'movep':0.1
                                    }
        
        self.default_movement_duration = 10

        self.filepath = 'src/ur_trajectory_controller/config/trajectories'
        
        # Retain an array of past states - keep it at a max length (nr of rows) of 10 msgs to average over later
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
        if msg.code == self.scaled_trajectory_trigger:
            self.register_position('stc')
        if msg.code == self.move_j_trigger:
            self.register_position('movej')
        if msg.code == self.move_l_trigger:
            self.register_position('movel')
        if msg.code == self.move_p_trigger:
            self.register_position('movep')

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

    def register_position(self, move_type):
        self.get_logger().info('Registering position')
        average_position = np.mean(self.states, axis=0)
        self.trajectory.append((average_position, move_type))

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
                f.write(f'{indent*3}positions: {np.array2string(self.trajectory[i][0], precision=10, separator=",", max_line_width=10**3)}\n')
                f.write(f'{indent*3}move_type: "{self.trajectory[i][1]}"\n')

                if self.controller == 'ur_script':
                    f.write(f'{indent*3}velocity: {self.default_velocity[self.trajectory[i][1]]}\n')
                    f.write(f'{indent*3}acceleration: {self.default_acceleration[self.trajectory[i][1]]}\n')

                # Movement duration
                f.write(f'{indent*3}movement_duration: [{int(self.default_movement_duration)}, {int((self.default_movement_duration - int(self.default_movement_duration))/10**9)}]\n\n')

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
