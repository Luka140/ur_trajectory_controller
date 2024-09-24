import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


def generate_launch_description():

    pkg = "ur_trajectory_controller"
    
    #   ==================== Remember to build any new trajectory that was recorded before trying to launch it =====================
    path_config = 'trajectory_scan_turbine_blade2.yaml'

    position_goals = PathJoinSubstitution(
        [FindPackageShare(f"{pkg}"), "config/trajectories", path_config]
    )
    
    controller = Node(
        package=pkg,
        executable="ur_controller",
        name='publisher_scaled_joint_trajectory_controller',    # This is the first line in the config file 
        parameters=[position_goals, {
            'autonomous_execution': False
        }],
    )

    coordinator = Node(
        package=pkg,
        executable='coordinator',
        name='publisher_scaled_joint_trajectory_controller',
        parameters=[position_goals, {
            'autonomous_execution': False
        }],
    )


    # UR ROS2 Driver
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(pkg),'launch','ur_driver.launch.py')])
    )

    lls_pcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('lls_processing'),'launch', 'lls_processing.launch.py')])
    )

    # Scancontrol driver & calibration
    scanner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(pkg),'launch','ur_launch_scanner.launch.py')])
    )
   
    
    rviz_config_path = os.path.join(get_package_share_directory(pkg),
        'config',
        'rviz_config.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output={'both': 'log'},
        arguments=['-d', rviz_config_path,],
    )

    ld = LaunchDescription()
    ld.add_action(driver)
    ld.add_action(coordinator)
    ld.add_action(controller)
    ld.add_action(lls_pcl)
    ld.add_action(rviz)
    ld.add_action(scanner)
    return ld