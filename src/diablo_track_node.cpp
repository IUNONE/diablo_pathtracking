#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>
#include <vector>
#include <array>
#include <mutex>
#include <string>
#include <algorithm>
#include <chrono>
#include <signal.h>
#include <atomic>
#include <thread>

// Global variables for signal handling
std::atomic<bool> g_emergency_stop{false};

// Signal handler for Ctrl+C
void signal_handler(int signum) {
    if (signum == SIGINT) {
        g_emergency_stop.store(true);
        RCLCPP_WARN(rclcpp::get_logger("signal_handler"), "Emergency stop triggered!");
        
        // Give some time for the emergency stop command to be sent
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        rclcpp::shutdown();
    }
}

double normalize_angle(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
{
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

class DiabloTrackNode : public rclcpp::Node
{
public:
    DiabloTrackNode() : Node("diablo_tracking_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        path_dt_ = declare_parameter<double>("path_dt", 0.1);
        control_hz_ = declare_parameter<int>("control_hz", 10);
        strategy_ = declare_parameter<std::string>("strategy", "open_loop");
        if (strategy_ != "pd" && strategy_ != "open_loop") {
            RCLCPP_FATAL(get_logger(), "Invalid strategy: %s", strategy_.c_str());
            rclcpp::shutdown();
        }

        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        robot_odom_frame_ = declare_parameter<std::string>("robot_odom_frame", "odom");
        robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");

        kp_linear_ = declare_parameter<double>("kp_linear", 1.0);
        kd_linear_ = declare_parameter<double>("kd_linear", 0.1);

        kp_angular_ = declare_parameter<double>("kp_angular", 2.0);
        kd_angular_ = declare_parameter<double>("kd_angular", 0.2);
        
        max_linear_vel_ = declare_parameter<double>("max_linear_vel", 2.0);
        max_angular_vel_ = declare_parameter<double>("max_angular_vel", 3.8);

        max_pos_error_ = declare_parameter<double>("max_pos_error", 0.3);
        feedforward_ramp_time_ = declare_parameter<double>("feedforward_ramp_time", 0.2);
        
        path_topic_ = declare_parameter<std::string>("path_topic", "/planned_path");

        // --------------------------------------------------------------------------------
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            path_topic_, 
            5,
            std::bind(&DiabloTrackNode::PathCallback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<motion_msgs::msg::MotionCtrl>("diablo/MotionCmd", 2);

        control_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / control_hz_),
            std::bind(&DiabloTrackNode::ControlLoop, this));

        RCLCPP_INFO(get_logger(), "Diablo Trajectory Tracking Node Initialized. Control strategy: %s. Waiting Path msg from topic: %s", 
            strategy_.c_str(), path_topic_.c_str());
    }

    void emergency_stop() {
        RCLCPP_WARN(get_logger(), "Emergency stop activated - sending zero velocity");
        _publish_cmd(0.0, 0.0);
        emergency_stop_active_.store(true);
    }

private:
    
    void _publish_cmd(double lin, double ang){
        motion_msgs::msg::MotionCtrl cmd;
        cmd.value.forward = lin;
        cmd.value.left = ang;
        cmd_pub_->publish(cmd);
        
        double elapsed = path_updated_ ? (this->now() - path_start_time_).seconds() : 0.0;
        RCLCPP_INFO(get_logger(), "[publish MotionCtrl] t=%.3fs, vel=(%.5f m/s, %.3f°/s)", 
                    elapsed, lin, ang * 180.0 / M_PI);
    }

    bool _get_tf(){
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                robot_odom_frame_, robot_base_frame_, 
                tf2::TimePointZero, 
                tf2::durationFromSec(0.1)
            );
            current_x_ = transform.transform.translation.x;
            current_y_ = transform.transform.translation.y;
            current_yaw_ = get_yaw_from_quaternion(transform.transform.rotation);
            
            return true;
        } 
        catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(get_logger(), "TF lookup failed for %s", ex.what());
            return false;
        }
    }

    /** 
        - extract x, y, yaw, vx, vy, vyaw from ros Path msg 
        - vx, vy, vyaw is reference velocity for open-loop
        - reset Error
    **/ 
    void PathCallback(const nav_msgs::msg::Path::SharedPtr msg){

        std::lock_guard<std::mutex> lock(path_mutex_);
        if (msg->poses.size() < 2) {
            RCLCPP_WARN(get_logger(), "Received path with less than 2 points, ignoring");
            return;
        }

        // 1. get x, y, yaw with init state 0,0,0
        path_points_.clear();
        path_points_.reserve(msg->poses.size() + 1);
        path_points_.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            double x = msg->poses[i].pose.position.x;
            double y = msg->poses[i].pose.position.y;
            double yaw = get_yaw_from_quaternion(msg->poses[i].pose.orientation);
            path_points_.push_back({x, y, yaw, 0.0, 0.0, 0.0});
        }

        // 2. compute (vx, vy, vyaw) from [i+1] - [i]
        // and the last is zero
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            double vx   = (path_points_[i+1][0] - path_points_[i][0]) / path_dt_;
            double vy   = (path_points_[i+1][1] - path_points_[i][1]) / path_dt_;
            double vyaw = normalize_angle(path_points_[i+1][2] - path_points_[i][2]) / path_dt_;

            path_points_[i][3] = vx;
            path_points_[i][4] = vy;
            path_points_[i][5] = vyaw;
        }
        
        // 3. Reset all control states
        if (strategy_ == "pd") {
            prev_pos_error_ = {0.0, 0.0, 0.0};
            prev_time_valid_ = false;
        }
        path_version_.fetch_add(1);
        path_updated_ = true;
        RCLCPP_INFO(get_logger(), "#########################################");
        RCLCPP_INFO(get_logger(), "New path received with %zu points, version: %lu, strategy: %s", 
                   msg->poses.size(), path_version_.load(), strategy_.c_str());
        path_start_time_ = now();
    }


    std::pair<double, double> calculate_pd_control(double elapsed_time){
        
        size_t subgoal_idx = static_cast<size_t>(elapsed_time / path_dt_) + 1;
        
        // error
        std::array<double, 3> pos_error = {
            path_points_[subgoal_idx][0] - current_x_, 
            path_points_[subgoal_idx][1] - current_y_, 
            normalize_angle(path_points_[subgoal_idx][2] - current_yaw_)
        };
        double pos_err_mag = std::hypot(pos_error[0], pos_error[1]);
        if (pos_err_mag > max_pos_error_) {
            pos_error[0] *= (max_pos_error_ / pos_err_mag);
            pos_error[1] *= (max_pos_error_ / pos_err_mag);
        }

        // pd
        std::array<double, 3> desired_vel_pd;
        auto now_time = now();
        if (prev_time_valid_) {
            double dt = (now_time - prev_time_).seconds();
            desired_vel_pd = {
                kp_linear_  * pos_error[0] + kd_linear_  * (pos_error[0] - prev_pos_error_[0]) / dt,
                kp_linear_  * pos_error[1] + kd_linear_  * (pos_error[1] - prev_pos_error_[1]) / dt,
                kp_angular_ * pos_error[2] + kd_angular_ * (pos_error[2] - prev_pos_error_[2]) / dt
            };
        } else {
            desired_vel_pd = {
                kp_linear_  * pos_error[0], 
                kp_linear_  * pos_error[1],
                kp_angular_ * pos_error[2]
            };
            prev_time_valid_ = true;
        }
        
        // feedward
        std::array<double, 3> ref_vel = {path_points_[subgoal_idx-1][3], path_points_[subgoal_idx-1][4], path_points_[subgoal_idx-1][5]};
        double feedforward_weight = std::min(1.0, elapsed_time / feedforward_ramp_time_);
        std::array<double, 3> total_desired_vel = {
            feedforward_weight * ref_vel[0] + desired_vel_pd[0],
            feedforward_weight * ref_vel[1] + desired_vel_pd[1],
            feedforward_weight * ref_vel[2] + desired_vel_pd[2]
        };
        
        prev_time_ = now_time;
        prev_pos_error_ = pos_error;

        double linear_vel = std::hypot(total_desired_vel[0], total_desired_vel[1]);
        double angular_vel = total_desired_vel[2];

        return {linear_vel, angular_vel};
    }


    std::pair<double, double> calculate_openloop_control(double elapsed_time){
        
        double subgoal_idx = elapsed_time / path_dt_;
        size_t cur_idx = static_cast<size_t>(subgoal_idx);

        double linear_vel = std::hypot(path_points_[cur_idx][3], path_points_[cur_idx][4]);
        double angular_vel = path_points_[cur_idx][5];

        return {linear_vel, angular_vel};
    }


    void ControlLoop(){
        
        // stop when emergency or tf is not available for close-loop
        if (emergency_stop_active_.load() || g_emergency_stop.load()) {
            _publish_cmd(0.0, 0.0);
            return;
        }
        if (strategy_ == "pd") {
            bool tf_available = false;
            tf_available = _get_tf();
            if (!tf_available) {
                _publish_cmd(0.0, 0.0);
                return;
            }
        }

        // check path
        std::lock_guard<std::mutex> lock(path_mutex_);
        uint64_t latest_version = path_version_.load();
        if (current_path_version_ != latest_version) {
            current_path_version_ = latest_version;
            RCLCPP_INFO(get_logger(), "Switched to path version: %lu", current_path_version_.load());
        }
        if (!path_updated_ || path_points_.empty()) {
            _publish_cmd(0.0, 0.0);
            return;
        }
        auto current_time = now();
        double elapsed_time = (current_time - path_start_time_).seconds();
        bool at_path_end = (elapsed_time / path_dt_) >= (path_points_.size() - 1);
        if (at_path_end) {
            RCLCPP_INFO(get_logger(), "Path execution completed (open-loop mode)");
            path_updated_ = false;
            _publish_cmd(0.0, 0.0);
            return;
        }

        // start control strategy
        double linear_vel = 0.0;
        double angular_vel = 0.0;
        
        if (strategy_ == "open_loop") {
            std::tie(linear_vel, angular_vel) = calculate_openloop_control(elapsed_time);
        } 
        else if (strategy_ == "pd") {
            std::tie(linear_vel, angular_vel) = calculate_pd_control(elapsed_time);
        }

        // Apply velocity limits
        linear_vel = std::clamp(linear_vel, -max_linear_vel_, max_linear_vel_);
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
        _publish_cmd(linear_vel, angular_vel);
    }

    // Emergency stop state
    std::atomic<bool> emergency_stop_active_{false};

    // ros msg Path
    std::string path_topic_;
    double path_dt_;
    std::mutex path_mutex_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    std::vector<std::array<double,6>> path_points_; // x, y, yaw, vx, vy, vyaw
    std::atomic<uint64_t> path_version_{0};
    std::atomic<uint64_t> current_path_version_{0};
    bool path_updated_ = false;
    rclcpp::Time path_start_time_;

    // ros msg MotionCtrl
    rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr cmd_pub_;
    double max_linear_vel_, max_angular_vel_;

    // tf for close-loop control
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string map_frame_, robot_odom_frame_, robot_base_frame_;
    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;

    // control strategy
    std::string strategy_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    int control_hz_;

    // pd + feedforward
    std::array<double,3> prev_pos_error_ = {0.0,0.0, 0.0};
    double max_pos_error_;
    double kp_linear_, kd_linear_;
    double kp_angular_, kd_angular_;
    double feedforward_ramp_time_;
    
    rclcpp::Time prev_time_;
    bool prev_time_valid_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiabloTrackNode>();
    
    // Register signal handler for Ctrl+C
    signal(SIGINT, signal_handler);
    
    RCLCPP_INFO(node->get_logger(), "Emergency stop enabled - press Ctrl+C to stop robot");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
