import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    # Setup project paths
    pkg_project = get_package_share_directory('final_proj')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                PathJoinSubstitution([
                    pkg_project,
                    'worlds',
                    'ambiente_0.sdf'
                ]),
                TextSubstitution(text=' -r')
            ]
        }.items(),
    )
    ld.add_action(gz_sim)

    return ld