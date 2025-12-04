"""
ArUco detection node for ROS2.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from aruco_tracker_msgs.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from tf2_ros import TransformBroadcaster


class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Parameters
        self.declare_parameter(
            "marker_size", 0.1, ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            "aruco_dictionary_id", "DICT_4X4_1000", ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            "image_topic",
            "/camera/camera/color/image_raw", ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera/color/camera_info", ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            "camera_frame", "camera_color_optical_frame", ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            "publish_tf", True, ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter(
            "marker_ids", [-1], ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER_ARRAY))
        self.id_list = self.get_parameter(
            'marker_ids').get_parameter_value().integer_array_value
        self.marker_size = self.get_parameter(
            "marker_size").get_parameter_value().double_value
        self.camera_frame = self.get_parameter(
            "camera_frame").get_parameter_value().string_value
        self.publish_tf = self.get_parameter(
            "publish_tf").get_parameter_value().bool_value

        dict_name = self.get_parameter(
            "aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter(
            "image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter(
            "camera_info_topic").get_parameter_value().string_value
        
        try:
            dictionary_id = cv2.aruco.__getattribute__(dict_name)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
            self.aruco_params = cv2.aruco.DetectorParameters()
            
            # MOVE ALL TUNING HERE - do it once, not every frame
            self.aruco_params.adaptiveThreshWinSizeMin = 3
            self.aruco_params.adaptiveThreshWinSizeMax = 23
            self.aruco_params.adaptiveThreshWinSizeStep = 10
            
            # Corner refinement (MOST IMPORTANT FOR STABILITY)
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.aruco_params.cornerRefinementWinSize = 5
            self.aruco_params.cornerRefinementMaxIterations = 100
            self.aruco_params.cornerRefinementMinAccuracy = 0.01
            
            # Reduce false positives
            self.aruco_params.minMarkerPerimeterRate = 0.05
            self.aruco_params.maxMarkerPerimeterRate = 4.0
            self.aruco_params.polygonalApproxAccuracyRate = 0.03
            
        except Exception as e:
            self.get_logger().error(f"Failed to load dictionary {dict_name}: {e}")
            raise


        # self.get_logger().info(f"Marker size: {self.marker_size}")
        # self.get_logger().info(f"Dictionary: {dict_name}")
        # self.get_logger().info(f"Image topic: {image_topic}")
        # self.get_logger().info(f"ID list is :\n {self.id_list}")
        # Subscriptions and publishers
        self.create_subscription(
            CameraInfo,
            info_topic,
            self.info_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, image_topic, self.image_callback, 100)

        self.poses_pub = self.create_publisher(
            PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(
            ArucoMarkers, "aruco_markers", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Camera and ArUco
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.bridge = CvBridge()
        try:
            dictionary_id = cv2.aruco.__getattribute__(dict_name)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
            self.aruco_params = cv2.aruco.DetectorParameters()
        except Exception as e:
            self.get_logger().error(
                f"Failed to load dictionary {dict_name}: {e}")
            raise

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Simple preprocessing
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict,
                                                         parameters=self.aruco_params)
        if ids is None or len(ids) == 0:
            return
        for i, marker_id in enumerate(ids):
            if marker_id in self.id_list:

                # Estimate poses
                # Publish results
                markers = ArucoMarkers()
                pose_array = PoseArray()
                markers.header.frame_id = self.camera_frame
                markers.header.stamp = img_msg.header.stamp

                pose_array.header.frame_id = self.camera_frame
                pose_array.header.stamp = img_msg.header.stamp

                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                self.get_logger().info(f'corners are {corners}')

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion)
                self.get_logger().info(f'rvev is {rvecs}')
                self.get_logger().info(f'tvev is {tvecs}')
                
                corners_3d = np.array([[-self.marker_size/2.0, self.marker_size/2.0, 0], [self.marker_size/2.0, self.marker_size/2.0, 0], [self.marker_size/2.0, -self.marker_size/2.0, 0], [-self.marker_size/2.0, -self.marker_size/2.0, 0]])

                success, rvecPNP, tvecPNP = cv2.solvePnP(corners_3d, corners[i], self.intrinsic_mat, self.distortion, flags=0)
                self.get_logger().info(f'rvevPNP is {rvecPNP}')
                self.get_logger().info(f'tvevPNP is {tvecPNP}')
                if rvecs is not None:
                    cv2.drawFrameAxes(cv_image,
                                    self.intrinsic_mat,
                                    self.distortion,
                                    rvecPNP,
                                    tvecPNP,
                                    self.marker_size * 0.5)
        
        # # self.get_logger().info(f"Ids are: \n{ids}")
        # for id in ids:
        #     if id not in self.id_list:
        #         return
                cv2.imshow('camera', cv_image)
                cv2.waitKey(1)

                for i, marker_id in enumerate(ids):
                    pose = Pose()
                    pose.position.x = float(tvecs[i][0][0])
                    pose.position.y = float(tvecs[i][0][1])
                    pose.position.z = float(tvecs[i][0][2])

                    # Convert rotation vector to quaternion
                    rot_matrix = np.eye(4)
                    rot_matrix[0:3, 0:3] = cv2.Rodrigues(rvecs[i][0])[0]
                    quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                    pose.orientation.x = float(quat[0])
                    pose.orientation.y = float(quat[1])
                    pose.orientation.z = float(quat[2])
                    pose.orientation.w = float(quat[3])

                    pose_array.poses.append(pose)
                    markers.poses.append(pose)
                    markers.marker_ids.append(int(marker_id[0]))
                    # self.get_logger().info(
                    #     f"Detected {len(ids)} markers: {ids.flatten().tolist()}")

                    if self.publish_tf:
                        self._publish_transform(
                            pose, marker_id[0],
                            markers.header.frame_id,
                            img_msg.header.stamp)

                self.poses_pub.publish(pose_array)
                self.markers_pub.publish(markers)


    def _publish_transform(self, pose, marker_id, frame_id, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = frame_id
        transform.child_frame_id = f"aruco_marker_{int(marker_id)}"
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation.x = pose.orientation.x
        transform.transform.rotation.y = pose.orientation.y
        transform.transform.rotation.z = pose.orientation.z
        transform.transform.rotation.w = pose.orientation.w
        self.tf_broadcaster.sendTransform(transform)
    
    def estimate_pose_ippe(self, corners, marker_size, K, dist):
        """
        IPPE-based pose estimation for a single ArUco marker.
        corners: list of length 1, each is (4,1,2) or (1,4,2)
        marker_size: length of marker side (meters)
        K: camera matrix
        dist: distortion coefficients
        Returns:
            rvec, tvec
        """
        # Marker coordinate frame: square in Z=0 plane
        half = marker_size / 2.0
        obj_points = np.array([
            [-half,  half, 0.0],  # top-left
            [ half,  half, 0.0],  # top-right
            [ half, -half, 0.0],  # bottom-right
            [-half, -half, 0.0],  # bottom-left
        ], dtype=np.float32)

        # OpenCV returns corners shape (1,4,2)
        img_points = corners.reshape(4, 2).astype(np.float32)

        # SolvePnP using the IPPE-specific solver
        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            K, dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if not success:
            return None, None

        return rvec, tvec



def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
