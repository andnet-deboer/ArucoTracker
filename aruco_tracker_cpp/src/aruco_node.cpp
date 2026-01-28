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
#include <vector>
#include <algorithm>

using std::placeholders::_1;

// --- 1 EURO FILTER CLASS ---
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
    double min_cutoff_, beta_, d_cutoff_, x_prev_, dx_prev_;
    bool initialized_;
};

// --- FILTER BANK (One per Marker ID) ---
struct MarkerFilter {
    OneEuroFilter x, y, z, qx, qy, qz, qw;
    MarkerFilter(double mc, double b) : x(mc,b), y(mc,b), z(mc,b), qx(mc,b), qy(mc,b), qz(mc,b), qw(mc,b) {}
};

class ArucoNodeCpp : public rclcpp::Node {
public:
    ArucoNodeCpp() : Node("aruco_node_cpp") {
        // 1. Declare Parameters
        this->declare_parameter("marker_size", 0.05);
        this->declare_parameter("target_ids", std::vector<long int>{135}); 
        this->declare_parameter("filter_min_cutoff", 1.0);
        this->declare_parameter("filter_beta", 0.05);
        this->declare_parameter("show_gui", true);
        this->declare_parameter("camera_frame", "head_camera_infra1_optical_frame");

        // 2. Fetch Parameters
        marker_size_ = this->get_parameter("marker_size").as_double();
        target_ids_ = this->get_parameter("target_ids").as_integer_array();
        min_c_ = this->get_parameter("filter_min_cutoff").as_double();
        beta_ = this->get_parameter("filter_beta").as_double();
        show_gui_ = this->get_parameter("show_gui").as_bool();
        camera_frame_ = this->get_parameter("camera_frame").as_string();

        // 3. ArUco/OpenCV Setup
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        parameters_ = cv::aruco::DetectorParameters::create();
        parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        if (show_gui_) {
            cv::namedWindow("Filtered Tracker", cv::WINDOW_NORMAL);
            cv::startWindowThread();
        }

        // 4. ROS Communication
        auto qos = rclcpp::SensorDataQoS();
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", qos, std::bind(&ArucoNodeCpp::info_callback, this, _1));
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", qos, std::bind(&ArucoNodeCpp::image_callback, this, _1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("aruco_poses", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "Aruco Multi-Tracker Ready. GUI: %s", show_gui_ ? "ON" : "OFF");
    }

    ~ArucoNodeCpp() {
        if (show_gui_) cv::destroyAllWindows();
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

        rclcpp::Time current_time = msg->header.stamp;
        if (last_time_.nanoseconds() == 0) { last_time_ = current_time; return; }
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        if (dt <= 0) dt = 1.0/90.0;

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

        auto pose_array = geometry_msgs::msg::PoseArray();
        pose_array.header = msg->header;

        for (size_t k = 0; k < ids.size(); ++k) {
            int id = ids[k];
            // Whitelist Check
            if (std::find(target_ids_.begin(), target_ids_.end(), id) != target_ids_.end()) {
                
                if (filter_bank_.find(id) == filter_bank_.end()) {
                    filter_bank_.emplace(id, std::make_unique<MarkerFilter>(min_c_, beta_));
                }
                auto& f = filter_bank_.at(id);

                std::vector<cv::Vec3d> rvecs, tvecs;
                std::vector<std::vector<cv::Point2f>> single_set = {corners[k]};
                cv::aruco::estimatePoseSingleMarkers(single_set, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

                cv::Mat rot_matrix;
                cv::Rodrigues(rvecs[0], rot_matrix);
                tf2::Matrix3x3 tf2_rot(
                    rot_matrix.at<double>(0,0), rot_matrix.at<double>(0,1), rot_matrix.at<double>(0,2),
                    rot_matrix.at<double>(1,0), rot_matrix.at<double>(1,1), rot_matrix.at<double>(1,2),
                    rot_matrix.at<double>(2,0), rot_matrix.at<double>(2,1), rot_matrix.at<double>(2,2)
                );
                tf2::Quaternion q; tf2_rot.getRotation(q);

                // Filter Pos and Ori
                double sx = f->x.filter(tvecs[0][0], dt);
                double sy = f->y.filter(tvecs[0][1], dt);
                double sz = f->z.filter(tvecs[0][2], dt);
                double sqx = f->qx.filter(q.x(), dt);
                double sqy = f->qy.filter(q.y(), dt);
                double sqz = f->qz.filter(q.z(), dt);
                double sqw = f->qw.filter(q.w(), dt);
                tf2::Quaternion sq(sqx, sqy, sqz, sqw); sq.normalize();

                geometry_msgs::msg::Pose p;
                p.position.x = sx; p.position.y = sy; p.position.z = sz;
                p.orientation.x = sq.x(); p.orientation.y = sq.y(); p.orientation.z = sq.z(); p.orientation.w = sq.w();
                pose_array.poses.push_back(p);

                geometry_msgs::msg::TransformStamped t;
                t.header = msg->header;
                t.child_frame_id = "aruco_" + std::to_string(id);
                t.transform.translation.x = sx; t.transform.translation.y = sy; t.transform.translation.z = sz;
                t.transform.rotation = p.orientation;
                tf_broadcaster_->sendTransform(t);

                if (show_gui_) {
                    if (frame.channels() == 1) cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
                    cv::drawFrameAxes(frame, camera_matrix_, dist_coeffs_, rvecs[0], tvecs[0], marker_size_);
                }
            }
        }

                // DEBUG - print ALL detected IDs before whitelist filtering
        for (int id : ids) {
            RCLCPP_INFO(this->get_logger(), "Detected marker ID: %d", id);
        }

        if (!pose_array.poses.empty()) pose_pub_->publish(pose_array);

        if (show_gui_) {
            cv::Mat dst_image;
            cv::rotate(frame, dst_image, cv::ROTATE_90_CLOCKWISE);
            cv::imshow("Filtered Tracker", dst_image);
            cv::waitKey(1);
        }
    }

    // Members
    std::vector<long int> target_ids_;
    double marker_size_, min_c_, beta_;
    bool show_gui_;
    std::string camera_frame_;
    std::map<int, std::unique_ptr<MarkerFilter>> filter_bank_;
    rclcpp::Time last_time_;
    bool has_calibration_ = false;
    cv::Mat camera_matrix_, dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoNodeCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}