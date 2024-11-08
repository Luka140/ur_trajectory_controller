import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    pkg = 'ur_trajectory_controller'
    # Include the driver launch file.
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(pkg),'launch','ur_driver.launch.py')])
    )

    key_logger = Node(
        package="keyboard",
        executable="keyboard"
    )

    recorder = Node(
        package=pkg,
        executable='ur_trajectory_recorder',
        parameters=[{'record_urscript': True
                     }]
    )

        # RViz Configuration
    rviz_config_path = os.path.join(
        get_package_share_directory(pkg),
        'config',
        'rviz_config.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output={'both': 'log'},
        arguments=['-d', rviz_config_path],
    )

    ld = LaunchDescription([driver, recorder, key_logger, rviz])
    return ld
