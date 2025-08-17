#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Declare launch arguments
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='Enable camera display in GUI'
    )
    
    auto_start_localization_arg = DeclareLaunchArgument(
        'auto_start_localization',
        default_value='false',
        description='Automatically start localization on startup'
    )
    
    auto_start_navigation_arg = DeclareLaunchArgument(
        'auto_start_navigation',
        default_value='false',
        description='Automatically start navigation on startup'
    )
    
    default_map_arg = DeclareLaunchArgument(
        'default_map',
        default_value='',
        description='Default map file to load'
    )
    
    # Robot API Service Node
    robot_api_node = Node(
        package='spatibot_gui',
        executable='robot_api_node.py',
        name='robot_api_node',
        output='screen',
        parameters=[{
            'auto_start_localization': LaunchConfiguration('auto_start_localization'),
            'auto_start_navigation': LaunchConfiguration('auto_start_navigation'),
            'default_map': LaunchConfiguration('default_map'),
            'waypoints_file': 'waypoints.yaml'
        }]
    )
    
    # Service-Based GUI Node
    gui_node = Node(
        package='spatibot_gui',
        executable='spatibot_gui_service.py',
        name='spatibot_gui_service_client',
        output='screen',
        parameters=[{
            'enable_camera': LaunchConfiguration('enable_camera')
        }]
    )
    
    return LaunchDescription([
        enable_camera_arg,
        auto_start_localization_arg,
        auto_start_navigation_arg,
        default_map_arg,
        
        LogInfo(msg="Starting Service-Based SpatiBot GUI..."),
        LogInfo(msg="Robot API Service will handle all robot operations"),
        LogInfo(msg="GUI is a thin client that communicates via services"),
        
        # Start robot API service first
        robot_api_node,
        
        # Start GUI client
        gui_node
    ]) 