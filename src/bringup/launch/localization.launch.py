import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Extended Kalman Filter Node
        Node(
            package='localization',
            executable='ekf',
            name='ekf_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Graph SLAM Node
        Node(
            package='localization',
            executable='slam',
            name='slam_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Visualization node for debugging
        Node(
            package='localization',
            executable='viz',
            name='slam_viz',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])