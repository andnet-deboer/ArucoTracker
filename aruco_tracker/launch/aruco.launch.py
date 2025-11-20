from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ArUco parameters
    aruco_params = os.path.join(
        get_package_share_directory('aruco_tracker'),
        'config',
        'aruco_parameters.yaml'
    )

    aruco_node = Node(
        package='aruco_tracker',
        executable='aruco_node',
        parameters=[aruco_params]
    )

    return LaunchDescription([
        aruco_node,
    ])