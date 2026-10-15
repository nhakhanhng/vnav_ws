"""Start TESSE visualization and the Foxglove WebSocket bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/current_state'),
        DeclareLaunchArgument('desired_topic', default_value='/desired_state'),
        DeclareLaunchArgument('visuals_topic', default_value='/tesse/visuals'),
        Node(
            package='tesse_foxglove_pkg',
            executable='tesse_viz_publisher',
            name='tesse_viz_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'desired_topic': LaunchConfiguration('desired_topic'),
                'visuals_topic': LaunchConfiguration('visuals_topic'),
            }],
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
        ),
    ])
