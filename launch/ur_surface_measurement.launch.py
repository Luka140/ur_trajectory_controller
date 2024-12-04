import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg = "ur_trajectory_controller"

    # Declare Launch Arguments for this package
    path_config_arg = DeclareLaunchArgument(
        'path_config',
        default_value='trajectory_moving_grinder_dual_robot.yaml',
        description='Path to the trajectory config file'
    )

    autonomous_execution_arg = DeclareLaunchArgument(
        'autonomous_execution',
        default_value='False',
        description='Enable autonomous execution'
    )

    loop_on_service_arg = DeclareLaunchArgument(
        'loop_on_service',
        default_value='True',
        description='Enable looping on service'
    )

    auto_loop_arg = DeclareLaunchArgument(
        'auto_loop',
        default_value='False',
        description='Enable automatic looping'
    )

    # Declare Launch Arguments for lls_processing.launch.py
    global_frame_arg = DeclareLaunchArgument(
        'global_frame_id', default_value='base_link',
        description='Global frame ID for TF transformations'
    )
    
    alpha_arg = DeclareLaunchArgument(
        'alpha', default_value='0.005',
        description='Alpha value for mesh construction'
    )
    
    bbox_max_arg = DeclareLaunchArgument(
        'bbox_max', default_value='1.0',
        description='Bounding box max size for cropping point clouds'
    )

    local_bbox_max_arg = DeclareLaunchArgument(
        'local_bbox_max', default_value='0.0',
        description='Bounding box max size for cropping point clouds in the local coordinate system'
    )

    save_mesh_arg = DeclareLaunchArgument(
        'save_mesh', default_value='false',
        description='Flag to save the constructed mesh'
    )

    # Get Launch Configurations for this package
    path_config = LaunchConfiguration('path_config')
    autonomous_execution = LaunchConfiguration('autonomous_execution')
    loop_on_service = LaunchConfiguration('loop_on_service')
    auto_loop = LaunchConfiguration('auto_loop')

    # Get Launch Configurations for lls_processing
    global_frame = LaunchConfiguration('global_frame_id')
    alpha = LaunchConfiguration('alpha')
    bbox_max = LaunchConfiguration('bbox_max')
    local_bbox_max = LaunchConfiguration('local_bbox_max')
    save_mesh = LaunchConfiguration('save_mesh')

    # Define Path to Position Goals
    position_goals = PathJoinSubstitution(
        [FindPackageShare(pkg), "config", "trajectories", path_config]
    )

    # Define Nodes for this package
    controller = Node(
        package=pkg,
        executable="ur_controller",
        parameters=[
            position_goals,  # Load parameters from the YAML file
            {'autonomous_execution': autonomous_execution}
        ],
    )

    movement_coordinator = Node(
        package=pkg,
        executable='coordinator',
        parameters=[
            position_goals,  # Load parameters from the YAML file
            {
                'loop_on_service': loop_on_service,
                'auto_loop': auto_loop
            }
        ],
    )

    # Include Other Launch Files for this package
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', 'ur_driver.launch.py')
        ),
        launch_arguments={
            "robot_ip": "192.168.0.1"
        }.items()
    )

#     driver_w_namespace = GroupAction(
#         actions=[
#             PushRosNamespace('UR'),
#             driver,
#       ]
#    )

    scanner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', 'ur_launch_scanner.launch.py')
        )
    )

    # Include lls_processing.launch.py with arguments
    lls_pcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lls_processing'), 'launch', 'lls_processing.launch.py')
        ),
        launch_arguments={
            'global_frame_id': global_frame,
            'alpha': alpha,
            'bbox_max': bbox_max,
            'local_bbox_max': local_bbox_max,
            'save_mesh': save_mesh
        }.items()
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

    # Create Launch Description and Populate
    ld = LaunchDescription()

    # Add Launch Arguments for this package
    ld.add_action(path_config_arg)
    ld.add_action(autonomous_execution_arg)
    ld.add_action(loop_on_service_arg)
    ld.add_action(auto_loop_arg)

    # Add Launch Arguments for lls_processing
    ld.add_action(global_frame_arg)
    ld.add_action(alpha_arg)
    ld.add_action(bbox_max_arg)
    ld.add_action(local_bbox_max_arg)
    ld.add_action(save_mesh_arg)

    # Add Actions
    ld.add_action(driver)
    # ld.add_action(driver_w_namespace)
    ld.add_action(movement_coordinator)
    ld.add_action(controller)
    ld.add_action(lls_pcl)
    ld.add_action(scanner)
    ld.add_action(rviz)

    return ld
