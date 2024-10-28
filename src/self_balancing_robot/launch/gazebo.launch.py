import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Path to the Gazebo world file
    world_file = os.path.join(get_package_share_directory('self_balancing_robot'), 'worlds', 'self_balancing_world.world')

    # Gazebo launch file to launch the world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items(),
    )

    return LaunchDescription([
        gazebo_launch
    ])
