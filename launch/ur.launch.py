import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Include the driver launch file.
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ur_trajectory_controller'),'launch','ur_driver.launch.py')])
    )

    # Include the trajectory controller launch file
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ur_trajectory_controller'),'launch', 'ur_trajectory_control.launch.py')])
    )

    lls_pcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('lls_processing'),'launch', 'lls_processing.launch.py')])
    )
    
    return LaunchDescription([
        driver,
        controller,
        lls_pcl
    ]) 