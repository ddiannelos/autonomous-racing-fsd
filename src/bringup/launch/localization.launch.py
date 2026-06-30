import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('bringup'), 'config')
    ekf_params = os.path.join(config_dir, 'ekf_params.yaml')
    graph_slam_params = os.path.join(config_dir, 'graph_slam_params.yaml')
    viz_params = os.path.join(config_dir, 'slam_viz_params.yaml')

    return LaunchDescription([
        # Extended Kalman Filter Node
        Node(
            package='localization',
            executable='ekf',
            name='ekf_node',
            output='screen',
            parameters=[{'use_sim_time': True}, ekf_params]
        ),
        # Graph SLAM Node
        Node(
            package='localization',
            executable='slam',
            name='slam_node',
            output='screen',
            parameters=[{'use_sim_time': True}, graph_slam_params]
        ),
        # Visualization node for debugging
        Node(
            package='localization',
            executable='viz',
            name='slam_viz',
            output='screen',
            parameters=[{'use_sim_time': True}, viz_params]
        )
    ])