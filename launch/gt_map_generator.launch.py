from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    ld = LaunchDescription()

    # Setup project paths
    world_path = '/home/anaomscosta/maps_and_bags/ambiente_0.sdf'
    map_output_path = '/home/anaomscosta/maps_and_bags/gt_map'

    # Output map params
    corners = '(-8.0,-4.0,0.05)(10.0,8.5,10.0)'
    resolution = '0.05'

    start_gazebo_plugin = ExecuteProcess(
        cmd=[
            'gazebo',
            '-s', 'libgazebo_map_creator.so', 
            world_path
        ],
        output='screen'
    )
    ld.add_action(start_gazebo_plugin)

    start_map_generator = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_map_creator', 'request_map.py',
            '-c', corners,
            '-r', resolution,
            '-f', map_output_path,
            '--skip-vertical-scan'
        ],
        output='screen'
    )
    ld.add_action(start_map_generator)

    gazebo_shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_map_generator,
            on_exit=[
                LogInfo(msg='Mapeamento pronto! Fechando simulação...'),
                ExecuteProcess(cmd=['pkill', '-f', 'gazebo'], output='screen')
            ],
        )
    )
    ld.add_action(gazebo_shutdown_handler)

    # Launch!
    return ld
