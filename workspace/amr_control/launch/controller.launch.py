import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_control',
            executable='controller',
            name='controller',
            parameters=[os.path.join(
                get_package_share_directory('amr_control'), "config", "waypoints.yaml")],
            output='screen'),
    ])
