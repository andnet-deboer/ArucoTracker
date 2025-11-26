# ArucoTracker
Tracks aruco markers in ROS 2 and publishes the frame(s) to the tf2 tree

A ROS2 node for detecting and tracking ArUco markers using OpenCV. This node detects ArUco markers in camera images, estimates their 6D poses, and publishes the results as both poses and transform frames.

This repo is a modification of the following repo: https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco

## Features

- Real-time ArUco marker detection
- TF2 transform broadcasting for detected markers
- Configurable marker dictionary and size
- Marker ID filtering


## Usage

Run the node with:

```bash
ros2 run aruco_tracker aruco_node
```

Or with custom parameters:

```bash
ros2 run aruco_tracker aruco_node --ros-args -p marker_size:=0.05 -p marker_ids:=[0,1,2]
```

## Configuration Parameters

All parameters should be set in your node's launch file or YAML configuration. Below are the key parameters you need to customize:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `marker_size` | double | 0.1 | **Physical size of your marker in meters** (required for accurate pose estimation) |
| `aruco_dictionary_id` | string | DICT_4X4_1000 | ArUco dictionary to use (e.g., DICT_4X4_50, DICT_5X5_100, DICT_6X6_250, DICT_7X7_1000) |
| `image_topic` | string | /camera/camera/color/image_raw | Camera image topic to subscribe to |
| `camera_info_topic` | string | /camera/camera/color/camera_info | Camera calibration info topic |
| `camera_frame` | string | camera_color_optical_frame | TF frame ID of the camera |
| `publish_tf` | bool | true | Whether to broadcast marker poses as TF transforms |
| `marker_ids` | int array | [-1] | List of marker IDs to detect ([-1] detects all markers) |

## Generating ArUco Markers

Generate your ArUco markers using the online tool:

**https://chev.me/arucogen/**

1. Select the appropriate dictionary (must match `aruco_dictionary_id`)
2. Enter your desired marker ID (must be within the dictionary range)
3. Specify the desired physical size when printing
4. Generate and print the marker

## Published Topics

- **aruco_poses** (PoseArray): Poses of all detected markers
- **aruco_markers** (ArucoMarkers): Marker information with IDs
- **tf**: Transform frames for each detected marker (if `publish_tf: true`)
