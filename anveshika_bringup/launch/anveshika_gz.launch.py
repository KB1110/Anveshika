import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    description_pkg = 'anveshika_description'
    bringup_pkg = 'anveshika_bringup'
    
    world = "default.sdf"
    camera_name = "realsense_d435i"
    
    os.environ['GZ_SIM_RESOURCE_PATH'] = FindPackageShare(package=description_pkg).find(description_pkg)
    
    urdf_file_path = os.path.join(
        FindPackageShare(package=description_pkg).find(description_pkg),
        'urdf',
        'anveshika.urdf'
    )
    
    sdf_file_path = os.path.join(
        FindPackageShare(package=description_pkg).find(description_pkg),
        'urdf',
        'model.sdf'
    )

    gazebo_launch_file_path = os.path.join(
        FindPackageShare(package='ros_gz_sim').find('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    rviz_config_file_path = os.path.join(
        FindPackageShare(package=bringup_pkg).find(bringup_pkg),
        'rviz',
        'gz_config.rviz'
    )

    bridge_params = os.path.join(
        FindPackageShare(package=bringup_pkg).find(bringup_pkg),
        'params',
        'anveshika_gz_bridge.yaml'
    )

    robot_description_config = Command(['xacro ', urdf_file_path])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_launch_file_path
        ),
        launch_arguments={'gz_args': ['-r -v4 ', world, " --physics-engine gz-physics-bullet-featherstone-plugin"], 
        'on_exit_shutdown': 'true'}.items()
    )

    gazebo_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file',
            sdf_file_path,
            '-x', "0.0",
            '-y', "0.0",
            '-z', '0.0'
        ],
        output='screen',
    )

    start_gazebo_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    realsense_color_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[f'{camera_name}/color/image', f'{camera_name}/color/image'],
        output='screen',
    )
    
    realsense_depth_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[f'{camera_name}/depth/depth_image', f'{camera_name}/depth/depth_image'],
        output='screen',
    )
    
    realsense_infra_1_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[f'{camera_name}/infra_1/image', f'{camera_name}/infra_1/image'],
        output='screen',
    )
    
    realsense_infra_2_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[f'{camera_name}/infra_2/image', f'{camera_name}/infra_2/image'],
        output='screen',
    )

    robot_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_config, 'use_sim_time': use_sim_time}],
        output="screen"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_file_path],
        condition = IfCondition(use_rviz)
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(name = 'use_sim_time',
                                        default_value='true',
                                        description='Use sim time if true'
                                        ))  
    ld.add_action(DeclareLaunchArgument(name = 'use_rviz',
                                        default_value='true',
                                        description='Uses rviz2 if true'
                                        ))
    
    ld.add_action(robot_publisher_node)
    ld.add_action(gazebo_launch)
    ld.add_action(gazebo_spawn_entity)
    ld.add_action(start_gazebo_ros_bridge)
    ld.add_action(realsense_color_image_bridge)
    ld.add_action(realsense_depth_image_bridge)
    ld.add_action(realsense_infra_1_image_bridge)
    ld.add_action(realsense_infra_2_image_bridge)
    ld.add_action(rviz2_node)
        
    return ld