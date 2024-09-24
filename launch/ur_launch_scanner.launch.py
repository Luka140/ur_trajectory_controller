import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)
    

def generate_launch_description():

    pkg = 'ur_trajectory_controller'
    tf_config = 'lls_calibration_tf.yaml'

    lls_driver = Node(
            package='micro_epsilon_scancontrol_driver',
            executable='driver_node',
            name='scancontrol_driver',
            output='screen',
            on_exit=Shutdown(),
        )


     # Obtained from the CAD model 
    # offset =[28.5, 42.01, 32.01]
    # offset = [0.0, 0.0, 0.0]
    # rotation = [0.0, 3.141592654, -1.570796327]
    # rotation_quat = [0, 0.5*2**0.5, 0.5*2**0.5, 0]

    # offset = [0.0287, 0.0419, 0.0297]
    # rotation_quat = [0.0024, -0.7032, -0.7109, 0.0028]

    config_file_path = os.path.join(
        get_package_share_directory(pkg),
        'config',
        tf_config
    )

    # Load the YAML file
    config = load_yaml(config_file_path)

    # Extract the values for the static transform
    tf = config['static_transform']

    flange_tool_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                str(tf['x']),    
                str(tf['y']),    
                str(tf['z']),    
                str(tf['qx']),   
                str(tf['qy']),   
                str(tf['qz']),   
                str(tf['qw']),   
                tf['frame-id'],  
                tf['child-frame-id']
            ]
        )
    
    ld = LaunchDescription()
    ld.add_action(lls_driver)
    ld.add_action(flange_tool_tf)
    return ld