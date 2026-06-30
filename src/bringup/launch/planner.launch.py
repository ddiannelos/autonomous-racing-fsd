import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('bringup'), 'config')
    params = os.path.join(config_dir, 'state_machine_params.yaml')

    return LaunchDescription([
        Node(
            package='path_planning',
            executable='state_machine_node',
            name='planner',
            output='screen',
            parameters=[{'use_sim_time': True}, params]
        )
    ])