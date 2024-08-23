from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = "ur_trajectory_controller"

    position_goals = PathJoinSubstitution(
        [FindPackageShare(f"{pkg}"), "config", "test_goal_publishers_config.yaml"]
    )
    
    controller = Node(
        package=pkg,
        executable="ur_controller",
        name='publisher_scaled_joint_trajectory_controller',
        parameters=[position_goals],
        )
        
    ld = LaunchDescription([controller])
    return ld