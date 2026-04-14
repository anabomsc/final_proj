import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ld.add_action(declare_use_sim_time)

    # Setup project paths
    pkg_project = get_package_share_directory('final_proj')

    # Setup for namespaces and frames
    name = "artbot1"
    namespace = "/" + name
    frame_prefix = name + "_"

    # SLAM Toolbox Mapping
    slam_params_file = os.path.join(pkg_project,'config','mapper_params_online_async.yaml')
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
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
    ld.add_action(start_async_slam_toolbox_node)

    # Launch!
    return ld