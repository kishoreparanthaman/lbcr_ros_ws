from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('ur5e_velocity_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'ur5e_velocity.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'ur5e_velocity.rviz')

    # ✅ Add missing xacro arguments
    xacro_args = {
        'name': 'ur5e',
        'ur_type': 'ur5e',  # Must match one of the valid robot types
        'use_fake_hardware': 'true',
        'fake_sensor_commands': 'false'
    }

    # Process xacro with args
    doc = xacro.process_file(xacro_file, mappings=xacro_args)
    robot_description_config = doc.toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description_config}],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen',
            shell=True
        )
    ])
