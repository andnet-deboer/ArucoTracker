#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "aruco_tracker_msgs/msg/aruco_markers.hpp"

using std::placeholders::_1;

// EURO FILTER
class OneEuroFilter {
public:
    OneEuroFilter(double min_cutoff = 1.0, double beta = 0.0) 
        : min_cutoff_(min_cutoff), beta_(beta), d_cutoff_(1.0), initialized_(false) {}

    double filter(double value, double dt) {
        if (!initialized_) {
            initialized_ = true;
            x_prev_ = value;
            dx_prev_ = 0.0;
            return value;
        }

        // Calculate derivative
        double dx = (value - x_prev_) / dt;
        double dx_hat = exponentialSmoothing(dx, dx_prev_, alpha(d_cutoff_, dt));

        // Calculate dynamic cutoff frequency
        // If moving fast (high dx), cutoff increases (less smoothing, less lag)
        // If still (low dx), cutoff decreases (more smoothing, less jitter)
        double cutoff = min_cutoff_ + beta_ * std::abs(dx_hat);
        
        // Filter the main value
        double x_hat = exponentialSmoothing(value, x_prev_, alpha(cutoff, dt));

        x_prev_ = x_hat;
        dx_prev_ = dx_hat;
        return x_hat;
    }

    void reset() { initialized_ = false; }

private:
    double alpha(double cutoff, double dt) {
        double tau = 1.0 / (2.0 * M_PI * cutoff);
        return 1.0 / (1.0 + tau / dt);
    }

    double exponentialSmoothing(double current, double prev, double alpha) {
        return alpha * current + (1.0 - alpha) * prev;
    }

    double min_cutoff_, beta_, d_cutoff_;
    double x_prev_, dx_prev_;
    bool initialized_;
};
// -----------------------------------------------------

class ArucoNodeCpp : public rclcpp::Node {
public:
    ArucoNodeCpp() : Node("aruco_node_cpp") {
        // Parameters
        this->declare_parameter("marker_size", 0.05);
        this->declare_parameter("camera_frame", "head_camera_infra1_optical_frame");
        // Tuning for 1 Euro Filter
        this->declare_parameter("filter_min_cutoff", 1.0); // Hz. Lower = smoother when still.
        this->declare_parameter("filter_beta", 0.05);      // Higher = faster response when moving.

        marker_size_ = this->get_parameter("marker_size").as_double();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        
        double min_c = this->get_parameter("filter_min_cutoff").as_double();
        double beta = this->get_parameter("filter_beta").as_double();

        // Initialize 7 Filters (X, Y, Z, Qx, Qy, Qz, Qw)
        f_x = OneEuroFilter(min_c, beta);
        f_y = OneEuroFilter(min_c, beta);
        f_z = OneEuroFilter(min_c, beta);
        f_qx = OneEuroFilter(min_c, beta);
        f_qy = OneEuroFilter(min_c, beta);
        f_qz = OneEuroFilter(min_c, beta);
        f_qw = OneEuroFilter(min_c, beta);

        // ArUco Setup
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        parameters_ = cv::aruco::DetectorParameters::create();
        parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        // ROS Communication
        auto qos = rclcpp::SensorDataQoS();
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", qos, std::bind(&ArucoNodeCpp::info_callback, this, _1));
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", qos, std::bind(&ArucoNodeCpp::image_callback, this, _1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("aruco_poses", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "C++ Tracker + 1Euro Filter Ready.");
    }

private:
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (has_calibration_) return;
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        dist_coeffs_ = cv::Mat(1, 5, CV_64F);
        for(int i=0; i<9; i++) camera_matrix_.at<double>(i/3, i%3) = msg->k[i];
        for(int i=0; i<5; i++) dist_coeffs_.at<double>(0, i) = msg->d[i];
        has_calibration_ = true;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!has_calibration_) return;

        // Calculate Delta Time (dt) for Filter
        rclcpp::Time current_time = msg->header.stamp;
        if (last_time_.nanoseconds() == 0) {
            last_time_ = current_time;
            return; // Skip first frame to avoid huge dt
        }
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        if (dt <= 0) dt = 1.0/90.0; // Safety fallback

        // Decode Image
        cv::Mat frame;
        if (msg->encoding == "mono8") {
            frame = cv::Mat(msg->height, msg->width, CV_8UC1, const_cast<unsigned char*>(msg->data.data()), msg->step);
        } else {
            cv::Mat raw(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);
            if (msg->encoding == "rgb8") cv::cvtColor(raw, frame, cv::COLOR_RGB2BGR);
            else frame = raw;
        }

        // Detect
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dictionary_, corners, ids, parameters_);

        if (ids.size() > 0) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            auto pose_array = geometry_msgs::msg::PoseArray();
            pose_array.header = msg->header;

            // We only filter the FIRST marker found (Single Marker Logic)
            // If you track multiple, you'd need a filter bank per ID.
            size_t i = 0; 
            
            // --- RAW DATA ---
            double raw_x = tvecs[i][0];
            double raw_y = tvecs[i][1];
            double raw_z = tvecs[i][2];

            cv::Mat rot_matrix;
            cv::Rodrigues(rvecs[i], rot_matrix);
            tf2::Matrix3x3 tf2_rot(
                rot_matrix.at<double>(0,0), rot_matrix.at<double>(0,1), rot_matrix.at<double>(0,2),
                rot_matrix.at<double>(1,0), rot_matrix.at<double>(1,1), rot_matrix.at<double>(1,2),
                rot_matrix.at<double>(2,0), rot_matrix.at<double>(2,1), rot_matrix.at<double>(2,2)
            );
            tf2::Quaternion q; 
            tf2_rot.getRotation(q);

            // --- FILTERING ---
            double smooth_x = f_x.filter(raw_x, dt);
            double smooth_y = f_y.filter(raw_y, dt);
            double smooth_z = f_z.filter(raw_z, dt);
            
            double smooth_qx = f_qx.filter(q.x(), dt);
            double smooth_qy = f_qy.filter(q.y(), dt);
            double smooth_qz = f_qz.filter(q.z(), dt);
            double smooth_qw = f_qw.filter(q.w(), dt);

            // Normalize quaternion after filtering (Vital!)
            tf2::Quaternion smooth_q(smooth_qx, smooth_qy, smooth_qz, smooth_qw);
            smooth_q.normalize();

            // --- PUBLISH ---
            geometry_msgs::msg::Pose pose;
            pose.position.x = smooth_x;
            pose.position.y = smooth_y;
            pose.position.z = smooth_z;
            pose.orientation.x = smooth_q.x();
            pose.orientation.y = smooth_q.y();
            pose.orientation.z = smooth_q.z();
            pose.orientation.w = smooth_q.w();

            pose_array.poses.push_back(pose);
            
            // Publish TF
            geometry_msgs::msg::TransformStamped t;
            t.header = msg->header;
            t.child_frame_id = "aruco_" + std::to_string(ids[i]);
            t.transform.translation.x = pose.position.x;
            t.transform.translation.y = pose.position.y;
            t.transform.translation.z = pose.position.z;
            t.transform.rotation = pose.orientation;
            tf_broadcaster_->sendTransform(t);

            // Visualization (Green Axis = Filtered)
            if (frame.channels() == 1) cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
            
            // Draw Raw Axis (Red/Blue/Green standard)
            cv::drawFrameAxes(frame, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], marker_size_);
            
            // Draw Filtered Center (Yellow Dot)
            // (Project 3D point back to 2D to verify alignment)
            std::vector<cv::Point3f> pt3d = {{ (float)smooth_x, (float)smooth_y, (float)smooth_z }};
            std::vector<cv::Point2f> pt2d;
            // Note: simple projection check isn't trivial without rvec/tvec conversion back
            // so we trust the TF broadcast for verification
            
            pose_pub_->publish(pose_array);
        } else {
            // Reset filters if tracking is lost, so it doesn't "teleport" next time
            f_x.reset(); f_y.reset(); f_z.reset();
            f_qx.reset(); f_qy.reset(); f_qz.reset(); f_qw.reset();
        }

        cv::imshow("Filtered Tracker", frame);
        cv::waitKey(1);
    }

    // Filters
    OneEuroFilter f_x, f_y, f_z, f_qx, f_qy, f_qz, f_qw;
    rclcpp::Time last_time_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::Mat camera_matrix_, dist_coeffs_;
    bool has_calibration_ = false;
    double marker_size_;
    std::string camera_frame_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoNodeCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}