import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Path to the Gazebo world file
    world_file = os.path.join(get_package_share_directory('self_balancing_robot'), 'worlds', 'self_balancing_world.world')

    # Gazebo launch file to launch the world
    gazebo_launch = ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        )
    
    # Robot State Publisher to display the URDF in Gazebo
    urdf_file = os.path.join(get_package_share_directory('self_balancing_robot'), 'urdf', 'self_balancing_robot.urdf')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )
    

    return LaunchDescription([
        gazebo_launch, 
        robot_state_publisher
    ])
