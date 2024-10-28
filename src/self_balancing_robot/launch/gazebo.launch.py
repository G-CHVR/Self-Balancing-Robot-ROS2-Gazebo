import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Path to the Gazebo world file
    world_file = os.path.join(get_package_share_directory('self_balancing_robot'), 'worlds', 'self_balancing_world.world')

    # Gazebo launch file to launch the world
    gazebo_launch = ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        )

    return LaunchDescription([
        gazebo_launch
    ])
