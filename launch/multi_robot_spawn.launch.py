import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit

def parse_robot_positions(robots_positions_string):
    robots_positions_string = robots_positions_string.strip()
    if not robots_positions_string:
        return []

    positions = []
    for robot in robots_positions_string.split('),'):
        robot = robot.strip().strip('()')
        try:
            x, y, yaw = robot.split(',')
            positions.append((float(x), float(y), float(yaw)))

        except ValueError:
            raise ValueError(
                f'Posição inválida: "{robot}". '
                'Use o formato (x,y,yaw),(x,y,yaw),...'
            )
    return positions

def setup_robots(context, *args, **kwargs):
    # Launch configurations
    robots_positions_string = LaunchConfiguration('robots_positions').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_noise_stddev = LaunchConfiguration('lidar_noise_stddev')
    odom_noise_stddev = LaunchConfiguration('odom_noise_stddev')

    robot_positions = parse_robot_positions(robots_positions_string)
    if len(robot_positions) == 0:
        raise ValueError('Nenhuma posição de robô foi fornecida.')

    # Setup paths
    pkg_project = get_package_share_directory('final_proj')
    xacro_file = os.path.join(pkg_project,'description','robot','robot.urdf.xacro')
    bridge_params = os.path.join(pkg_project,'config','artbot_bridge.yaml')

    # Spawn robots
    z_offset = 0.1
    actions = []
    last_action = None

    for i, (x,y,yaw) in enumerate(robot_positions):
        name = f'artbot{i + 1}'
        namespace = f'/{name}'
        frame_prefix = f'{name}_'
        topic_prefix = f'{name}/'

        # Robot state publisher
        robot_description_config = Command([
            'xacro ', xacro_file,
            ' frame_prefix:=', frame_prefix,
            ' topic_prefix:=', topic_prefix,
            ' lidar_noise_stddev:=', lidar_noise_stddev,
            ' odom_noise_stddev:=', odom_noise_stddev,
        ])
        params = {'robot_description': robot_description_config,'use_sim_time': use_sim_time}
        state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[params],
            output='both'
        )

        # Robot gazebo spawn
        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic',f'{namespace}/robot_description',
                '-name',name,
                '-x',str(x),
                '-y',str(y),
                '-z',str(z_offset),
                '-Y',str(yaw)
            ],
            output='screen'
        )

        # ROS-Gazebo bridge
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',

            namespace=namespace,

            parameters=[{
                'config_file': bridge_params,
                'expand_gz_topic_names': True
            }],

            output='screen'
        )

        # Sequantial spawn handling
        if last_action is None:
            actions.append(state_publisher)
            actions.append(spawn_robot)
            actions.append(bridge)
        else:
            actions.append(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=last_action,
                        on_exit=[state_publisher, spawn_robot, bridge]
                    )
                )
            )
        last_action = spawn_robot

    return actions

def generate_launch_description():
    ld = LaunchDescription()

    # Configuration
    declare_robots_positions = DeclareLaunchArgument(
        'robots_positions',
        default_value='(0.0,0.0,0.0),(2.0,0.0,0.0)',
        description='Robot positions in the format (x,y,yaw),(x,y,yaw),...'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )
    declare_lidar_noise_stddev = DeclareLaunchArgument(
        'lidar_noise_stddev',
        default_value='0.0',
        description='Standard deviation of LiDAR gaussian noise'
    )
    declare_odom_noise_stddev = DeclareLaunchArgument(
        'odom_noise_stddev',
        default_value='0.0',
        description='Standard deviation of OdometryPublisher plugin gaussian noise'
    )

    ld.add_action(declare_robots_positions)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_lidar_noise_stddev)
    ld.add_action(declare_odom_noise_stddev)
    ld.add_action(OpaqueFunction(function=setup_robots))

    # Launch!
    return ld