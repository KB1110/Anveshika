from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    anveshika_slam_pkg_dir = get_package_share_directory('anveshika_slam')
    config_file = os.path.join(anveshika_slam_pkg_dir, 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package='anveshika_slam',
            executable='anveshika_slam',
            output='screen',
            parameters=[config_file]
        )
    ])
