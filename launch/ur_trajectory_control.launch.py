from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = "ur_trajectory_controller"
    
    #   ==================== Remember to build any new trajectory that was recorded before trying to launch it =====================
    # path_config = 'trajectory_find_rad.yaml'
    # path_config = 'trajectory_calibration.yaml'
    # path_config = 'trajectory_calibration_extended.yaml'
    # path_config = 'trajectory_calibration_inversion.yaml'
    # path_config = 'trajectory_calibration_3.yaml'
    # path_config = 'trajectory_calibration_fussball.yaml'
    path_config = "trajectory_test_plate_vertical2.yaml"
    # path_config = 'trajectory_repeatability_line.yaml'

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