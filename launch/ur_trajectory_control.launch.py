from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = "ur_trajectory_controller"
    # path_config = "test_grinding_measurement_config.yaml"
    path_config = "trajectory_2024-08-28 09:53:26.650496.yaml"

    position_goals = PathJoinSubstitution(
        [FindPackageShare(f"{pkg}"), "config/trajectories", path_config]
    )
    
    controller = Node(
        package=pkg,
        executable="ur_controller",
        name='publisher_scaled_joint_trajectory_controller',    # This is the first line in the config file 
        parameters=[position_goals],
        )
    
    tester = Node(
        package=pkg,
        executable="ur_trigger",
        name="publisher_scaled_joint_trajectory_controller",
        parameters=[position_goals]
    )

    keyboard_listener = Node(
        package="keyboard",
        executable="keyboard"
    )

    flange_tool_tf = Node(package="tf2_ros",
                          executable='static_transform_publisher',
                          arguments= ["0", "0", "0", "0", "0", "-1.570796", "flange", "scancontrol"])

    ld = LaunchDescription([controller])
    ld.add_action(tester)
    ld.add_action(keyboard_listener)
    ld.add_action(flange_tool_tf)
    return ld