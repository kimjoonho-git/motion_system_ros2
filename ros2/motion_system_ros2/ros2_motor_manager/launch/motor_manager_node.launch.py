import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_motor_manager')
    default_config = os.path.join(pkg_share, 'config', 'example_socketcan_cubemars.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Absolute path to motor_manager YAML (masters / drivers).',
    )
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='false',
        description='Automatically run the CiA402 servo enable sequence.',
    )

    return LaunchDescription([
        config_file_arg,
        auto_enable_arg,
        Node(
            package='ros2_motor_manager',
            executable='motor_manager_node',
            name='motor_manager_node',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'auto_enable': LaunchConfiguration('auto_enable'),
            }],
        ),
    ])
