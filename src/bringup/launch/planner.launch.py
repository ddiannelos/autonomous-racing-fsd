import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planning',
            executable='state_machine_node',
            name='planner',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])