import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Setup gazebo simulation
    gz_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project, 'launch', 'ambiente_0.launch.py')),
    )
    ld.add_action(gz_simulation)

    # Setup single robot spawn and details
    single_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project, 'launch', 'single_robot_spawn.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    ld.add_action(single_robot)

    # Setup single robot slam
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project, 'launch', 'single_robot_slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    ld.add_action(slam)

    # Setup rviz2 
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project, 'config', 'sim_single_robot.rviz')],
    )
    ld.add_action(rviz)

    # Launch!
    return ld