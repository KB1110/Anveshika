from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to your configurations and parameter files
    slam_lidar_config_dir = os.path.join(get_package_share_directory('slam_lidar'), 'configuration')
    realsense_param_file = os.path.join(get_package_share_directory('anveshika_detection'), 'params', 'realsense.yaml')

    return LaunchDescription([
        # Include the YDLIDAR launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')
            ])
        ),

        # Cartographer node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                # '--ros-args', '--',
                '-configuration_directory', slam_lidar_config_dir,
                '-configuration_basename', 'config.lua'
            ]
        ),

        # Cartographer occupancy grid node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=[
                # '--ros-args', '--',
                '-resolution', '0.05',
                '-publish_period_sec', '1.0'
            ]
        ),

        # SLAM Lidar navigation node
        Node(
            package='slam_lidar',
            executable='navigation_node',
            name='navigation_node',
            output='screen'
        ),

        # Node(
        #     package='slam_lidar',
        #     executable='imu_node',
        #     name='imu_node',
        #     # Launch the node with root access (GPIO) in a shell
        #     prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
        #     shell=True,
        # ),

        # SLAM Lidar driving node
        Node(
            package='slam_lidar',
            executable='driving_node',
            name='driving_node',
            output='screen'
        ),

        Node(
            package='anveshika_detection',
            executable='mode_node',
            name='mode_node',
            output='screen'
        ),

        # Node(
        #     package='anveshika_arm',
        #     executable='arm_node',
        #     name='arm_node',
        #     output='screen'
        # ),

        # RealSense camera node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            parameters=[realsense_param_file]
        ),

        # Anveshika detection node
        Node(
            package='anveshika_detection',
            executable='detection_node',
            name='detection_node',
            output='screen'
        ),

        # Node(
        #     package='anveshika_telemetry',
        #     executable='telemetry_node',
        #     name='telemetry_node',
        #     output='screen'
        # )
    ])
