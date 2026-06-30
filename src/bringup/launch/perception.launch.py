import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('bringup'), 'config')
    lidar_params = os.path.join(config_dir, 'lidar_params.yaml')

    return LaunchDescription([
        # Camera Node
        Node(
            package='camera_detection',
            executable='camera',
            name='camera_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # LiDAR Node
        Node(
            package='lidar_perception',
            executable='lidar',
            name='lidar_node',
            output='screen',
            parameters=[{'use_sim_time': True}, lidar_params]
        ),
        # Sensor Fusion Node
        Node(
            package='sensor_fusion',
            executable='sensor_fusion',
            name='sensor_fusion_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])