from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'aruco_tracker_cpp'
    realsense_dir = get_package_share_directory('realsense2_camera')
    
    # 1. Declare the GUI argument (Default is false to save CPU)
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Enable ArUco debug image publishing (adds CPU overhead)'
    )

    config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'parameters.yaml'
    )

    # RealSense Launch - Target 90 FPS
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_dir, 'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_name': 'head_camera',
            'enable_color': 'false',
            'enable_infra1': 'true',
            'enable_depth': 'false',
            'depth_module.infra_profile': '848x480x90',
            # This must be false to allow manual exposure at 90Hz
            'depth_module.auto_exposure_priority': 'false', 
        }.items()
    )

    # ArUco Node with config file and GUI toggle
    tracker_node = Node(
        package=pkg_name,
        executable='aruco_node_cpp',
        name='aruco_node_cpp',
        output='screen',
        parameters=[
            config_path, 
            {'publish_debug': LaunchConfiguration('gui')}
        ],
        remappings=[
            ('image_raw', '/camera/head_camera/infra1/image_rect_raw'),
            ('camera_info', '/camera/head_camera/infra1/camera_info')
        ]
    )

    # Hardware Tuning for the D435if Head Camera
    # These values compensate for the IR-pass filter while maintaining 90Hz
    tune_camera = RegisterEventHandler(
        OnProcessStart(
            target_action=tracker_node,
            on_start=[
                LogInfo(msg='Applying 90Hz D435if Hardware Tuning...'),
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/camera/head_camera', 'depth_module.emitter_enabled', '0'],
                    output='screen'
                ),
                # Disable auto-exposure first so manual values take effect
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/camera/head_camera', 'depth_module.enable_auto_exposure', 'false'],
                    output='screen'
                ),
                # 10ms exposure is the limit for 90Hz (1/90 = 11.1ms)
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/camera/head_camera', 'depth_module.exposure', '10000'],
                    output='screen'
                ),
                # High gain to compensate for the dark IR filter
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/camera/head_camera', 'depth_module.gain', '248'],
                    output='screen'
                ),
            ]
        )
    )

    return LaunchDescription([
        gui_arg,
        rs_launch,
        tracker_node,
        tune_camera
    ])