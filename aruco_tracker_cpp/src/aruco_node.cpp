#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <map>

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
        double dx = (value - x_prev_) / dt;
        double dx_hat = exponentialSmoothing(dx, dx_prev_, alpha(d_cutoff_, dt));
        double cutoff = min_cutoff_ + beta_ * std::abs(dx_hat);
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

class ArucoNodeCpp : public rclcpp::Node {
public:
    ArucoNodeCpp() : Node("aruco_node_cpp") {
        // Parameters
        this->declare_parameter("marker_size", 0.05);
        this->declare_parameter("camera_frame", "head_camera_infra1_optical_frame");
        this->declare_parameter("aruco_dictionary_id", "DICT_APRILTAG_36h11");
        this->declare_parameter("filter_min_cutoff", 1.0);
        this->declare_parameter("filter_beta", 0.05);
        this->declare_parameter("publish_debug_image", true);  // NEW

        marker_size_ = this->get_parameter("marker_size").as_double();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        std::string dict_name = this->get_parameter("aruco_dictionary_id").as_string();
        publish_debug_ = this->get_parameter("publish_debug_image").as_bool();
        
        double min_c = this->get_parameter("filter_min_cutoff").as_double();
        double beta = this->get_parameter("filter_beta").as_double();

        // Initialize Filters
        f_x = OneEuroFilter(min_c, beta);
        f_y = OneEuroFilter(min_c, beta);
        f_z = OneEuroFilter(min_c, beta);
        f_qx = OneEuroFilter(min_c, beta);
        f_qy = OneEuroFilter(min_c, beta);
        f_qz = OneEuroFilter(min_c, beta);
        f_qw = OneEuroFilter(min_c, beta);

        // Dictionary Selection
        std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dict_map = {
            {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
            {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
            {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
            {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
            {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
            {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
            {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
            {"DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5},
            {"DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9},
            {"DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10},
            {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11}
        };

        if (dict_map.count(dict_name)) {
            dictionary_ = cv::aruco::getPredefinedDictionary(dict_map[dict_name]);
            RCLCPP_INFO(this->get_logger(), "Initialized with Dictionary: %s", dict_name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid Dictionary: %s. Defaulting to APRILTAG_36h11", dict_name.c_str());
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        }

        parameters_ = cv::aruco::DetectorParameters::create();
        parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;

        // ROS Communication
        auto qos = rclcpp::SensorDataQoS();
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", qos, std::bind(&ArucoNodeCpp::info_callback, this, _1));
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", qos, std::bind(&ArucoNodeCpp::image_callback, this, _1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("aruco_poses", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Debug image publisher
        if (publish_debug_) {
            debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aruco_debug_image", 10);
            RCLCPP_INFO(this->get_logger(), "Debug image publishing ENABLED on 'aruco_debug_image'");
        }

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

        // Calculate Delta Time
        rclcpp::Time current_time = msg->header.stamp;
        if (last_time_.nanoseconds() == 0) {
            last_time_ = current_time;
            return;
        }
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        if (dt <= 0) dt = 1.0/90.0;

        // Decode Image
        cv::Mat frame;
        bool is_mono = (msg->encoding == "mono8");
        
        if (is_mono) {
            frame = cv::Mat(msg->height, msg->width, CV_8UC1, 
                const_cast<unsigned char*>(msg->data.data()), msg->step);
        } else {
            cv::Mat raw(msg->height, msg->width, CV_8UC3, 
                const_cast<unsigned char*>(msg->data.data()), msg->step);
            if (msg->encoding == "rgb8") cv::cvtColor(raw, frame, cv::COLOR_RGB2BGR);
            else frame = raw;
        }

        // Detect
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dictionary_, corners, ids, parameters_);

        // Prepare debug image only if needed and has subscribers
        cv::Mat debug_frame;
        bool should_publish_debug = publish_debug_ && 
            (debug_image_pub_->get_subscription_count() > 0);
        
        if (should_publish_debug) {
            // Convert mono to BGR for colored overlay
            if (is_mono) {
                cv::cvtColor(frame, debug_frame, cv::COLOR_GRAY2BGR);
            } else {
                debug_frame = frame.clone();
            }
        }

        if (ids.size() > 0) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            auto pose_array = geometry_msgs::msg::PoseArray();
            pose_array.header = msg->header;

            // Process first marker
            size_t i = 0;

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

            // Filter
            double smooth_x = f_x.filter(raw_x, dt);
            double smooth_y = f_y.filter(raw_y, dt);
            double smooth_z = f_z.filter(raw_z, dt);
            double smooth_qx = f_qx.filter(q.x(), dt);
            double smooth_qy = f_qy.filter(q.y(), dt);
            double smooth_qz = f_qz.filter(q.z(), dt);
            double smooth_qw = f_qw.filter(q.w(), dt);

            tf2::Quaternion smooth_q(smooth_qx, smooth_qy, smooth_qz, smooth_qw);
            smooth_q.normalize();

            // Publish PoseArray
            geometry_msgs::msg::Pose pose;
            pose.position.x = smooth_x;
            pose.position.y = smooth_y;
            pose.position.z = smooth_z;
            pose.orientation.x = smooth_q.x();
            pose.orientation.y = smooth_q.y();
            pose.orientation.z = smooth_q.z();
            pose.orientation.w = smooth_q.w();
            pose_array.poses.push_back(pose);
            pose_pub_->publish(pose_array);

            // Publish TF
            geometry_msgs::msg::TransformStamped t;
            t.header = msg->header;
            t.child_frame_id = "aruco_" + std::to_string(ids[i]);
            t.transform.translation.x = smooth_x;
            t.transform.translation.y = smooth_y;
            t.transform.translation.z = smooth_z;
            t.transform.rotation = pose.orientation;
            tf_broadcaster_->sendTransform(t);

            // Draw debug overlay
            if (should_publish_debug) {
                // Draw detected markers (green corners + ID)
                cv::aruco::drawDetectedMarkers(debug_frame, corners, ids);
                
                // Draw axes for each marker
                for (size_t j = 0; j < ids.size(); j++) {
                    cv::drawFrameAxes(debug_frame, camera_matrix_, dist_coeffs_, 
                                      rvecs[j], tvecs[j], marker_size_ * 0.5, 2);
                }
                
                // Add pose text overlay
                std::stringstream ss;
                ss << std::fixed << std::setprecision(3);
                ss << "ID:" << ids[i] << " X:" << smooth_x << " Y:" << smooth_y << " Z:" << smooth_z;
                cv::putText(debug_frame, ss.str(), cv::Point(10, 30), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            }

        } else {
            // Reset filters if tracking lost
            f_x.reset(); f_y.reset(); f_z.reset();
            f_qx.reset(); f_qy.reset(); f_qz.reset(); f_qw.reset();
            
            // Show "No marker" text
            if (should_publish_debug) {
                cv::putText(debug_frame, "No marker detected", cv::Point(10, 30), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }
        }

        // Publish debug image
        if (should_publish_debug) {
            auto debug_msg = std::make_unique<sensor_msgs::msg::Image>();
            debug_msg->header = msg->header;
            debug_msg->height = debug_frame.rows;
            debug_msg->width = debug_frame.cols;
            debug_msg->encoding = "bgr8";
            debug_msg->is_bigendian = false;
            debug_msg->step = debug_frame.cols * 3;
            debug_msg->data.assign(debug_frame.data, 
                                   debug_frame.data + debug_frame.total() * 3);
            debug_image_pub_->publish(std::move(debug_msg));
        }
    }

    // Filters
    OneEuroFilter f_x, f_y, f_z, f_qx, f_qy, f_qz, f_qw;
    rclcpp::Time last_time_;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;  // NEW
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // OpenCV
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::Mat camera_matrix_, dist_coeffs_;
    bool has_calibration_ = false;
    bool publish_debug_ = true;  // NEW
    double marker_size_;
    std::string camera_frame_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoNodeCpp>());
    rclcpp::shutdown();
    return 0;
}