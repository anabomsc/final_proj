import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()
    xy_offsets = [(0.0,0.0),(5.0,0.0)] # robot positions

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

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    num_robots = LaunchConfiguration('num_robots')

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_num_robots)

    # Setup project paths
    pkg_project = get_package_share_directory('final_proj')

    xacro_file = os.path.join(pkg_project,'description','robot','robot.urdf.xacro')

    # Spawn robots
    last_action = None
    max_robots = len(xy_offsets)

    for i in range(max_robots):
        name = "artbot" + str(i+1)
        namespace = "/" + name
        frame_prefix = name + "_"
        topic_prefix = name + "/"

        condition = IfCondition(PythonExpression([num_robots, f" > {i}"]))

        robot_description_config = Command([
            'xacro ', xacro_file,
            ' frame_prefix:=', frame_prefix,
            ' topic_prefix:=', topic_prefix
        ])

        #State publisher
        params = {'robot_description':robot_description_config, 'use_sim_time': use_sim_time}
        state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[params],
            output='both',
            condition=condition
        )

        # Spawn robot in Gazebo world
        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', f'{namespace}/robot_description',
                '-name', f'{name}',
                '-x', str(xy_offsets[i][0]),
                '-y', str(xy_offsets[i][1]),
                '-z', '0.1'
            ],
            output='screen',
            condition=condition
        )

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

        # Sequential spawning handling
        if last_action is None:
            ld.add_action(state_publisher)
            ld.add_action(spawn_robot)
            ld.add_action(bridge)
        else:
            spawn_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[state_publisher, spawn_robot, bridge]
                ),
                condition=condition
            )
            ld.add_action(spawn_event)

        last_action = spawn_robot

    return ld