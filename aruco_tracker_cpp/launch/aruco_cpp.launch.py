import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'aruco_tracker_cpp'
    
    # Path to parameters
    config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'parameters.yaml'
    )

    # Realsense Launch (Optimized for Infrared Global Shutter)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': 'head_camera',
            'enable_infra1': 'true',
            'enable_color': 'false',
            'enable_depth': 'false',
            'depth_module.infra_profile': '848x480x90',
            'depth_module.emitter_enabled': '0', # Disable laser dots for ArUco
        }.items()
    )

    # Tracker Node
    tracker_node = Node(
        package=pkg_name,
        executable='aruco_node_cpp',
        name='aruco_node_cpp',
        output='screen',
        parameters=[config_path],
        remappings=[
            ('image_raw', '/camera/head_camera/infra1/image_rect_raw'),
            ('camera_info', '/camera/head_camera/infra1/camera_info')
        ]
    )

    return LaunchDescription([
        realsense_launch,
        tracker_node
    ])