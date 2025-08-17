from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Include the Turtlebot4 OAK-D launch file
    turtlebot4_oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_bringup'),
                'launch',
                'oakd.launch.py'
            ])
        ])
    )

    # Launch our custom OAK-D node
    oakd_node = Node(
        package='spatibot_oakd',
        executable='oakd_node',
        name='oakd_node',
        output='screen'
    )

    # Launch RViz2 with a custom configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('spatibot_oakd'),
            'config',
            'oakd_view.rviz'
        ])]
    )

    return LaunchDescription([
        turtlebot4_oakd_launch,
        oakd_node,
        rviz_node
    ]) 