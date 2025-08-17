#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'auto_start_localization',
            default_value='false',
            description='Whether to automatically start localization on startup'
        ),
        DeclareLaunchArgument(
            'auto_start_navigation',
            default_value='false',
            description='Whether to automatically start navigation on startup'
        ),
        DeclareLaunchArgument(
            'default_map',
            default_value='',
            description='Default map file to load for localization'
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value='waypoints.yaml',
            description='Waypoints file to load on startup'
        ),
        
        # Robot API Node
        Node(
            package='spatibot_gui',
            executable='robot_api_node',
            name='robot_api_node',
            output='screen',
            parameters=[{
                'auto_start_localization': LaunchConfiguration('auto_start_localization'),
                'auto_start_navigation': LaunchConfiguration('auto_start_navigation'),
                'default_map': LaunchConfiguration('default_map'),
                'waypoints_file': LaunchConfiguration('waypoints_file'),
            }]
        )
    ]) 