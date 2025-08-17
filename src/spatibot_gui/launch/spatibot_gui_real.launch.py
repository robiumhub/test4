from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('spatibot_gui'),
        'config',
        'gui_config.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'display',
            default_value=':0',
            description='Display for GUI (e.g., :0 for local display)'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
        ),
        Node(
            package='spatibot_gui',
            executable='spatibot_gui',
            name='spatibot_gui_node',
            output='screen',
            parameters=[config_path],
        )
    ]) 