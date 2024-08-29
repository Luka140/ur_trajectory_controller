from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    rviz_file = LaunchConfiguration("rviz_file")
    launch_rviz = LaunchConfiguration("launch_rviz")

    pkg = "ur_trajectory_controller"

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur16e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.0.1",  # put your robot's IP address here
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    # ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.1 \
    # kinematics_params_file:="${HOME}/my_robot_calibration.yaml"

    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value = PathJoinSubstitution([
                FindPackageShare(pkg), "config", "my_robot_calibration.yaml",
                        ]
                    ),
            description="Path to the calibration file",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ur_robot_driver"),
                                "launch",
                                "ur_control.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "kinematics_params_file": kinematics_params_file,
                    # "kinematics_params_file": kinematics_params_file
                    # "tf_prefix": [LaunchConfiguration("ur_type"), "_"],
                    "launch_rviz": 'false',
                    # "description_launchfile": PathJoinSubstitution(
                    #     [
                    #         FindPackageShare("my_robot_cell_control"),
                    #         "launch",
                    #         "rsp.launch.py",
                        # ]
                    # ),
                }.items(),
            ),
        ]
    )
