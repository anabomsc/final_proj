import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
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
    topic_prefix = name + "/"

    # Process the URDF file
    xacro_file = os.path.join(pkg_project,'description','robot','robot.urdf.xacro')
    robot_description_config = Command([
        'xacro ', xacro_file,
        ' frame_prefix:=', frame_prefix,
        ' topic_prefix:=', topic_prefix,
    ])

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[params]
    )
    ld.add_action(state_publisher)

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', f'{namespace}/robot_description',
                                   '-entity', f"{name}",
                                   '-z', '0.1'],
                        output='screen')
    ld.add_action(spawn_entity)

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge_params = os.path.join(pkg_project,'config','artbot_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        parameters=[{
            'config_file': bridge_params,
            'expand_gz_topic_names':True
        }],
        output='screen'
    )
    ld.add_action(bridge)

    # Launch!
    return ld