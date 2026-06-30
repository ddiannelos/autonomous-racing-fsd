import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(get_package_share_directory('bringup'), 'config', 'params.yaml')

    return LaunchDescription([
        # Extended Kalman Filter Node
        Node(
            package='localization',
            executable='ekf',
            name='ekf_node',
            output='screen',
            parameters=[{'use_sim_time': True}, config_file]
        ),
        # Graph SLAM Node
        Node(
            package='localization',
            executable='slam',
            name='slam_node',
            output='screen',
            parameters=[{'use_sim_time': True}, config_file]
        ),
        # Visualization node for debugging
        Node(
            package='localization',
            executable='viz',
            name='slam_viz',
            output='screen',
            parameters=[{'use_sim_time': True}, config_file]
        )
    ])