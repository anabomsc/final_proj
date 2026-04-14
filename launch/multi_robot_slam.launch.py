import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def multiple_slam_instances(context, *args, **kwargs):
    slam_instances = []

    # Resolve runtime values
    use_sim_time = LaunchConfiguration('use_sim_time')
    num_robots = int(LaunchConfiguration('num_robots').perform(context))

    # Setup project paths
    pkg_project = get_package_share_directory('final_proj')
    slam_params_file = os.path.join(pkg_project,'config','mapper_params_online_async.yaml')

    for i in range(num_robots):
        name = "artbot" + str(i+1)
        namespace = "/" + name
        frame_prefix = name + "_"

        # SLAM Toolbox Mapping
        start_async_slam_toolbox_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            namespace=namespace,
            parameters=[
                slam_params_file,
                {'odom_frame': f'{frame_prefix}odom',
                'map_frame': f'{frame_prefix}map',
                'base_frame': f'{frame_prefix}base_footprint',
                'scan_topic': f'{namespace}/scan',
                'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('/map', f'{namespace}/map'),
                ('/map_metadata', f'{namespace}/map_metadata'),
            ],
        )

        slam_instances.append(start_async_slam_toolbox_node)
    
    return slam_instances

def generate_launch_description():
    ld = LaunchDescription()

    # Configuration
    declare_num_robots = DeclareLaunchArgument(
        'num_robots',
        default_value='2',
        description='Número de robôs na simulação'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )

    opaque_function = OpaqueFunction(function=multiple_slam_instances)

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_num_robots)
    ld.add_action(opaque_function)

    # Launch!
    return ld