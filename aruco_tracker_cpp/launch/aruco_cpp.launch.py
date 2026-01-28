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

    # RealSense Launch
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_dir, 'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_name': 'head_camera',
            'enable_color': 'false',
            'enable_infra1': 'true',
            'enable_depth': 'false',
            
            # --- THE 90FPS FIX ---
            # 'depth_module.profile' is for depth, but 'infra_profile' 
            # specifically controls the OV9282 Global Shutter sensor.
            'depth_module.infra_profile': '848x480x90',
            
            # This prevents the sensor from slowing down to "see" better in the dark
            'depth_module.auto_exposure_priority': 'false', 
            
            'infra1.exposure': '2000',
            'infra1.gain': '90',
        }.items()
    )

    # Luanch ArUco Node
    tracker_node = Node(
        package=pkg_name,
        executable='aruco_node_cpp',
        name='aruco_node_cpp',
        output='screen',
        parameters=[{
            'marker_size': 0.05,
            'camera_frame': 'head_camera_infra1_optical_frame',
            'filter_min_cutoff': 1.0,
            'filter_beta': 0.05
        }],
        remappings=[
            ('image_raw', '/camera/head_camera/infra1/image_rect_raw'),
            ('camera_info', '/camera/head_camera/infra1/camera_info')
        ]
    )

    # Wait for rs_launch to trigger camera node
    kill_laser = RegisterEventHandler(
        OnProcessStart(
            target_action=tracker_node, # Wait for the tracker to start
            on_start=[
                LogInfo(msg='Stopping RealSense Emitter...'),
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/camera/head_camera', 'depth_module.emitter_enabled', '0'],
                    output='screen'
                )
            ]
        )
    )

    return LaunchDescription([
        rs_launch,
        tracker_node,
        kill_laser
    ])