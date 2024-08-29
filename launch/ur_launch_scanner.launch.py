import launch

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='micro_epsilon_scancontrol_driver',
            executable='driver_node',
            name='scancontrol_driver',
            output='screen',
            on_exit=launch.actions.Shutdown(),
        )
    ])
    return ld