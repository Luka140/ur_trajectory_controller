from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = "ur_trajectory_controller"


    position_goals = PathJoinSubstitution(
        [FindPackageShare(f"{pkg}"), "config", "test_grinding_measurement_config.yaml"]
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

    ld = LaunchDescription([controller])
    ld.add_action(tester)
    ld.add_action(keyboard_listener)
    return ld