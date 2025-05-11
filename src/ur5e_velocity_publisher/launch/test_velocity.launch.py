from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5e_velocity_publisher',
            executable='test_velocity_publisher',
            name='test_velocity_publisher',
            output='screen'
        )
    ])