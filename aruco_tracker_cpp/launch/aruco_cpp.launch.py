from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'aruco_tracker_cpp'
    realsense_dir = get_package_share_directory('realsense2_camera')
    
    config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'parameters.yaml'
    )

    # RealSense Launch - 90 FPS!
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_dir, 'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_name': 'head_camera',
            'enable_color': 'false',
            'enable_infra1': 'true',
            'enable_depth': 'false',
            'depth_module.infra_profile': '848x480x90',
            'depth_module.auto_exposure_priority': 'false',
        }.items()
    )

    # ArUco Node with config file
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

    # Kill laser emitter after tracker starts
    kill_laser = RegisterEventHandler(
        OnProcessStart(
            target_action=tracker_node,
            on_start=[
                LogInfo(msg='Stopping RealSense Emitter...'),
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/camera/head_camera', 'depth_module.emitter_enabled', '0'],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/camera/head_camera', 'depth_module.exposure', '2000'],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/camera/head_camera', 'depth_module.gain', '70'],
                    output='screen'
                ),
            ]
        )
    )

    return LaunchDescription([
        rs_launch,
        tracker_node,
        kill_laser
    ])