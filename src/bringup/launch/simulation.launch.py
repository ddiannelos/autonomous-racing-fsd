import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('bringup')

    perception_launch = IncludeLaunchDescription(
        os.path.join(pkg_bringup, 'launch', 'perception.launch.py')
    )

    localization_launch = IncludeLaunchDescription(
        os.path.join(pkg_bringup, 'launch', 'localization.launch.py')
    )

    planner_launch = IncludeLaunchDescription(
        os.path.join(pkg_bringup, 'launch', 'planner.launch.py')
    )

    return LaunchDescription([
        perception_launch,
        localization_launch,
        planner_launch
    ])