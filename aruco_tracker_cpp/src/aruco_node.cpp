#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <map>

using std::placeholders::_1;

// --- ONE EURO FILTER CLASS ---
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
        double dx = (value - x_prev_) / dt;
        double alpha_d = alpha(d_cutoff_, dt);
        double dx_hat = alpha_d * dx + (1.0 - alpha_d) * dx_prev_;
        double cutoff = min_cutoff_ + beta_ * std::abs(dx_hat);
        double a = alpha(cutoff, dt);
        double x_hat = a * value + (1.0 - a) * x_prev_;
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
    double min_cutoff_, beta_, d_cutoff_, x_prev_, dx_prev_;
    bool initialized_;
};

class ArucoNodeCpp : public rclcpp::Node {
public:
    ArucoNodeCpp() : Node("aruco_node_cpp") {
        // 1. Declare and Get Parameters
        this->declare_parameter("marker_size", 0.05);
        this->declare_parameter("publish_debug", false);
        this->declare_parameter("camera_frame", "head_camera_infra1_optical_frame");
        this->declare_parameter("aruco_dictionary_id", "DICT_APRILTAG_36h11");
        this->declare_parameter("filter_min_cutoff", 1.0);
        this->declare_parameter("filter_beta", 0.05);

        marker_size_ = this->get_parameter("marker_size").as_double();
        publish_debug_ = this->get_parameter("publish_debug").as_bool();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        std::string dict_id = this->get_parameter("aruco_dictionary_id").as_string();
        double min_c = this->get_parameter("filter_min_cutoff").as_double();
        double beta = this->get_parameter("filter_beta").as_double();

        // 2. Map Dictionary ID string to OpenCV enum
        std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dict_map = {
            {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
            {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
            {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11},
            {"DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5}
        };

        if (dict_map.count(dict_id)) {
            dictionary_ = cv::aruco::getPredefinedDictionary(dict_map[dict_id]);
            RCLCPP_INFO(this->get_logger(), "Using Dictionary: %s", dict_id.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid Dictionary %s! Defaulting to 36h11", dict_id.c_str());
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        }

        parameters_ = cv::aruco::DetectorParameters::create();
        
        // 3. Initialize Filters
        f_x = OneEuroFilter(min_c, beta); f_y = OneEuroFilter(min_c, beta); f_z = OneEuroFilter(min_c, beta);
        f_qx = OneEuroFilter(min_c, beta); f_qy = OneEuroFilter(min_c, beta); f_qz = OneEuroFilter(min_c, beta); f_qw = OneEuroFilter(min_c, beta);

        // 4. ROS Comms
        auto qos = rclcpp::SensorDataQoS();
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", qos, std::bind(&ArucoNodeCpp::info_callback, this, _1));
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", qos, std::bind(&ArucoNodeCpp::image_callback, this, _1));
        
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("aruco_poses", 10);
        debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aruco_debug", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "Aruco Node Ready. Frame: %s", camera_frame_.c_str());
    }

private:
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (has_calibration_) return;
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        dist_coeffs_ = cv::Mat(1, 5, CV_64F);
        for(int i=0; i<9; i++) camera_matrix_.at<double>(i/3, i%3) = msg->k[i];
        for(int i=0; i<5; i++) dist_coeffs_.at<double>(0, i) = msg->d[i];
        has_calibration_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera Calibration Received.");
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!has_calibration_) return;

        double dt = 1.0/90.0;
        rclcpp::Time now = msg->header.stamp;
        if (last_time_.nanoseconds() != 0) dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0) dt = 0.011;

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        cv::Mat frame = cv_ptr->image;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dictionary_, corners, ids, parameters_);

        if (!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            // Filter first marker
            double sx = f_x.filter(tvecs[0][0], dt);
            double sy = f_y.filter(tvecs[0][1], dt);
            double sz = f_z.filter(tvecs[0][2], dt);

            cv::Mat rot; cv::Rodrigues(rvecs[0], rot);
            tf2::Matrix3x3 tf2_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                                   rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                                   rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));
            tf2::Quaternion q; tf2_rot.getRotation(q);
            
            tf2::Quaternion sq(f_qx.filter(q.x(), dt), f_qy.filter(q.y(), dt), f_qz.filter(q.z(), dt), f_qw.filter(q.w(), dt));
            sq.normalize();

            // Broadcast TF
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = camera_frame_; // Using the param from YAML
            t.child_frame_id = "aruco_" + std::to_string(ids[0]);
            t.transform.translation.x = sx; t.transform.translation.y = sy; t.transform.translation.z = sz;
            t.transform.rotation.x = sq.x(); t.transform.rotation.y = sq.y(); t.transform.rotation.z = sq.z(); t.transform.rotation.w = sq.w();
            tf_broadcaster_->sendTransform(t);
        }

        if (publish_debug_ && debug_pub_->get_subscription_count() > 0) {
            cv::Mat debug_frame;
            if(frame.channels() == 1) cv::cvtColor(frame, debug_frame, cv::COLOR_GRAY2BGR);
            else debug_frame = frame.clone();

            cv::aruco::drawDetectedMarkers(debug_frame, corners, ids);
            auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_frame).toImageMsg();
            debug_pub_->publish(*debug_msg);
        }
    }

    OneEuroFilter f_x, f_y, f_z, f_qx, f_qy, f_qz, f_qw;
    rclcpp::Time last_time_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::Mat camera_matrix_, dist_coeffs_;
    bool has_calibration_ = false, publish_debug_;
    double marker_size_;
    std::string camera_frame_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoNodeCpp>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}