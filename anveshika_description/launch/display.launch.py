from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare paths
    pkg_name = 'anveshika_description'
    urdf_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'urdf',
        'anveshika.urdf.xacro'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'rviz',
        'anveshika_rviz_config.rviz'
    ])

    # Get URDF via xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_path
    ])

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content
            }]
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])