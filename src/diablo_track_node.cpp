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
std::shared_ptr<class DiabloTrackNode> g_node = nullptr;
std::atomic<bool> g_emergency_stop{false};

// Signal handler for Ctrl+C
void signal_handler(int signum) {
    if (signum == SIGINT) {
        g_emergency_stop.store(true);
        RCLCPP_WARN(rclcpp::get_logger("signal_handler"), "Emergency stop triggered! Sending zero velocity...");
        
        if (g_node) {
            g_node->emergency_stop();
        }
        
        // Give some time for the emergency stop command to be sent
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        rclcpp::shutdown();
    }
}

class DiabloTrackNode : public rclcpp::Node
{
public:
    DiabloTrackNode() : Node("diablo_tracking_node"),
        tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // Parameters
        path_dt_ = declare_parameter("path_dt", 0.1);
        control_hz_ = declare_parameter("control_hz", 50);
        strategy_ = declare_parameter("strategy", std::string("pd"));

        map_frame_ = declare_parameter("map_frame", std::string("map"));
        robot_odom_frame_ = declare_parameter("robot_odom_frame", std::string("odom"));
        robot_base_frame_ = declare_parameter("robot_base_frame", std::string("base_link"));
        goal_frame_ = declare_parameter("goal_frame", std::string("goal"));

        kp_linear_ = declare_parameter("kp_linear", 1.0);
        kd_linear_ = declare_parameter("kd_linear", 0.1);
        kp_angular_ = declare_parameter("kp_angular", 2.0);
        kd_angular_ = declare_parameter("kd_angular", 0.2);
        max_linear_vel_ = declare_parameter("max_linear_vel", 2.0);
        max_angular_vel_ = declare_parameter("max_angular_vel", 0.2);
        goal_tolerance_ = declare_parameter("goal_tolerance", 0.1);
        lookahead_distance_ = declare_parameter("lookahead_distance", 0.5);

        soft_start_time_ = declare_parameter("soft_start_time", 2.0);
        max_pos_error_ = declare_parameter("max_pos_error", 1.0);
        feedforward_ramp_time_ = declare_parameter("feedforward_ramp_time", 1.5);

        precise_goal_tolerance_ = declare_parameter("precise_goal_tolerance", 0.02);
        velocity_tolerance_ = declare_parameter("velocity_tolerance", 0.05);
        decel_distance_ = declare_parameter("decel_distance", 0.2);
        min_decel_factor_ = declare_parameter("min_decel_factor", 0.1);
        ki_linear_ = declare_parameter("ki_linear", 0.05);
        enable_curveadapt_ = declare_parameter("enable_curveadapt", true);

        if (strategy_ != "pd" && strategy_ != "pid" && strategy_ != "open_loop") {
            RCLCPP_FATAL(get_logger(), "Invalid strategy: %s", strategy_.c_str());
            rclcpp::shutdown();
        }

        // Parameter validation
        if (control_hz_ <= 0 || control_hz_ > 1000) {
            RCLCPP_FATAL(get_logger(), "Invalid control_hz: %d (must be 1-1000)", control_hz_);
            rclcpp::shutdown();
        }
        if (path_dt_ <= 0.0 || path_dt_ > 1.0) {
            RCLCPP_FATAL(get_logger(), "Invalid path_dt: %.3f (must be 0-1.0)", path_dt_);
            rclcpp::shutdown();
        }
        if (max_linear_vel_ <= 0.0) {
            RCLCPP_FATAL(get_logger(), "Invalid max_linear_vel: %.3f (must be > 0)", max_linear_vel_);
            rclcpp::shutdown();
        }
        if (max_angular_vel_ <= 0.0) {
            RCLCPP_FATAL(get_logger(), "Invalid max_angular_vel: %.3f (must be > 0)", max_angular_vel_);
            rclcpp::shutdown();
        }

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "planned_path", 5,
            std::bind(&DiabloTrackNode::path_callback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<motion_msgs::msg::MotionCtrl>("diablo/MotionCmd", 2);

        control_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / control_hz_),
            std::bind(&DiabloTrackNode::control_loop, this));

        current_linear_vel_ = 0.0;
        prev_time_valid_ = false;
        RCLCPP_INFO(get_logger(), "Diablo Tracking Node Initialized");
    }

    // Emergency stop function - public so signal handler can access it
    void emergency_stop() {
        RCLCPP_WARN(get_logger(), "Emergency stop activated - sending zero velocity");
        publish_cmd(0.0, 0.0);
        emergency_stop_active_.store(true);
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (msg->poses.size() < 2) {
            RCLCPP_WARN(get_logger(), "Received path with less than 2 points, ignoring");
            return;
        }
        
        // Atomic path update and state reset
        path_points_.clear();
        path_points_.reserve(msg->poses.size());
        for (auto &pose : msg->poses) {
            double yaw = get_yaw_from_quaternion(pose.pose.orientation);
            path_points_.push_back({pose.pose.position.x, pose.pose.position.y, yaw});
        }
        
        // Reset all control states
        path_start_time_ = now();
        current_time_index_ = 0.0;
        prev_pos_error_ = {0.0, 0.0};
        integral_error_ = {0.0, 0.0};
        prev_time_valid_ = false;
        prev_angle_error_ = 0.0;
        prev_angle_time_valid_ = false;
        current_linear_vel_ = 0.0;
        tf_failure_count_ = 0;
        fallback_to_openloop_ = false;
        
        // Update path version to ensure control loop uses latest path
        path_version_.fetch_add(1);
        path_updated_ = true;
        
        RCLCPP_INFO(get_logger(), "New path received with %zu points, version: %lu, strategy: %s", 
                   msg->poses.size(), path_version_.load(), strategy_.c_str());
    }

    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
    {
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    bool get_robot_pose()
    {
        try {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_.lookupTransform(robot_odom_frame_, robot_base_frame_, 
                                     tf2::TimePointZero, tf2::durationFromSec(0.15));
            current_x_ = transform.transform.translation.x;
            current_y_ = transform.transform.translation.y;
            current_yaw_ = get_yaw_from_quaternion(transform.transform.rotation);
            
            // Reset TF failure count on successful lookup
            if (tf_failure_count_ > 0) {
                tf_failure_count_ = 0;
                if (fallback_to_openloop_) {
                    RCLCPP_INFO(get_logger(), "TF recovered, resuming normal operation");
                    fallback_to_openloop_ = false;
                }
            }
            return true;
        } catch (tf2::TransformException &ex) {
            tf_failure_count_++;
            
            static rclcpp::Time last_warn_time = this->now();
            auto current_time = this->now();
            if ((current_time - last_warn_time).seconds() > 1.0) {
                RCLCPP_WARN(get_logger(), "TF lookup failed (%d/%d): %s", 
                           tf_failure_count_, MAX_TF_FAILURES, ex.what());
                last_warn_time = current_time;
            }
            
            // Switch to open-loop mode after consecutive failures
            if (tf_failure_count_ >= MAX_TF_FAILURES && !fallback_to_openloop_) {
                fallback_to_openloop_ = true;
                RCLCPP_WARN(get_logger(), "TF lookup failed %d times, switching to open-loop mode", 
                           MAX_TF_FAILURES);
            }
            
            return false;
        }
    }

    std::pair<std::array<double, 2>, std::array<double, 2>> get_desired_state(double time_index)
    {
        if (path_points_.empty()) 
            return {{0.0, 0.0}, {0.0, 0.0}};
        if (time_index < 0) 
            return {{path_points_[0][0], path_points_[0][1]}, {0.0, 0.0}};
        if (time_index >= (double)(path_points_.size() - 1))
            return {{path_points_.back()[0], path_points_.back()[1]}, {0.0, 0.0}};
        size_t idx = static_cast<size_t>(time_index);
        double frac = time_index - idx;
        std::array<double, 2> pos_current = {path_points_[idx][0], path_points_[idx][1]};
        std::array<double, 2> pos_next = {path_points_[idx + 1][0], path_points_[idx + 1][1]};
        std::array<double, 2> desired_pos = {
            pos_current[0] + frac * (pos_next[0] - pos_current[0]),
            pos_current[1] + frac * (pos_next[1] - pos_current[1])
        };
        std::array<double, 2> desired_vel = {
            (pos_next[0] - pos_current[0]) / path_dt_,
            (pos_next[1] - pos_current[1]) / path_dt_
        };
        return {desired_pos, desired_vel};
    }

    std::pair<double, double> calculate_pd_control(const std::array<double,2> &desired_pos, const std::array<double,2> &desired_vel, bool at_path_end, double elapsed_time, double distance_error)
    {
        // Debug output for target and error
        static rclcpp::Time last_debug_time = this->now();
        auto now_dbg = this->now();
        if ((now_dbg - last_debug_time).seconds() > 0.5) {
            RCLCPP_DEBUG(this->get_logger(), "Current pose: (%.3f, %.3f, %.3f), Target: (%.3f, %.3f), Error: (%.3f, %.3f)",
                         current_x_, current_y_, current_yaw_,
                         desired_pos[0], desired_pos[1],
                         desired_pos[0] - current_x_, desired_pos[1] - current_y_);
            last_debug_time = now_dbg;
        }

        std::array<double, 2> pos_error = {desired_pos[0] - current_x_, desired_pos[1] - current_y_};
        double soft_start_factor = std::min(1.0, elapsed_time / soft_start_time_);
        double current_max_linear = max_linear_vel_ * soft_start_factor;
        double current_max_angular = max_angular_vel_ * soft_start_factor;
        if (at_path_end && distance_error < decel_distance_) {
            double decel_factor = std::max(min_decel_factor_, distance_error / decel_distance_);
            current_max_linear *= decel_factor;
            current_max_angular *= decel_factor;
        }
        double pos_err_mag = std::hypot(pos_error[0], pos_error[1]);
        if (pos_err_mag > max_pos_error_) {
            pos_error[0] *= (max_pos_error_ / pos_err_mag);
            pos_error[1] *= (max_pos_error_ / pos_err_mag);
        }
        std::array<double, 2> desired_vel_pid;
        auto now_time = now();
        if (prev_time_valid_) {
            double dt = (now_time - prev_time_).seconds();
            if (dt > 0) {
                std::array<double, 2> error_rate = {
                    (pos_error[0] - prev_pos_error_[0]) / dt,
                    (pos_error[1] - prev_pos_error_[1]) / dt
                };
                
                // PID control with conditional integration
                if (strategy_ == "pid") {
                    double temp_vel_x = kp_linear_ * pos_error[0] + ki_linear_ * integral_error_[0];
                    double temp_vel_y = kp_linear_ * pos_error[1] + ki_linear_ * integral_error_[1];
                    double temp_vel_mag = std::hypot(temp_vel_x, temp_vel_y);
                    
                    if (temp_vel_mag < current_max_linear * 0.95) {
                        integral_error_[0] += pos_error[0] * dt;
                        integral_error_[1] += pos_error[1] * dt;
                    }
                    
                    double max_integral = ki_linear_ > 0 ? (1.0 / ki_linear_) : 1.0;
                    integral_error_[0] = std::clamp(integral_error_[0], -max_integral, max_integral);
                    integral_error_[1] = std::clamp(integral_error_[1], -max_integral, max_integral);
                    
                    desired_vel_pid = {
                        kp_linear_ * pos_error[0] + ki_linear_ * integral_error_[0] + kd_linear_ * error_rate[0],
                        kp_linear_ * pos_error[1] + ki_linear_ * integral_error_[1] + kd_linear_ * error_rate[1]
                    };
                } else {
                    // PD control only
                    desired_vel_pid = {
                        kp_linear_ * pos_error[0] + kd_linear_ * error_rate[0],
                        kp_linear_ * pos_error[1] + kd_linear_ * error_rate[1]
                    };
                }
                prev_pos_error_ = pos_error;
            }
        } else {
            desired_vel_pid = {kp_linear_ * pos_error[0], kp_linear_ * pos_error[1]};
            prev_pos_error_ = pos_error;
            prev_time_valid_ = true;
        }
        prev_time_ = now_time;
        double feedforward_weight = std::min(1.0, elapsed_time / feedforward_ramp_time_);
        if (at_path_end) feedforward_weight *= 0.5;
        // Curvature adaptive feedforward control
        double curvature_factor = 1.0;
        if (enable_curveadapt_) {
            if (path_points_.size() < 2) {
                RCLCPP_ERROR(get_logger(), "Path too short for curvature calculation");
                return {0.0, 0.0};
            }
            size_t max_idx = path_points_.size() - 2;
            size_t idx = static_cast<size_t>(std::min(current_time_index_, (double)max_idx));
            double dyaw = path_points_[idx+1][2] - path_points_[idx][2];
            double ds = std::hypot(path_points_[idx+1][0] - path_points_[idx][0],
                                  path_points_[idx+1][1] - path_points_[idx][1]);
            double curvature = ds > 1e-6 ? std::abs(dyaw / ds) : 0.0;
            curvature_factor = 1.0 / (1.0 + 2.0 * curvature);
        }
        feedforward_weight *= curvature_factor;
        std::array<double, 2> total_desired_vel = {
            feedforward_weight * desired_vel[0] + desired_vel_pid[0],
            feedforward_weight * desired_vel[1] + desired_vel_pid[1]
        };
        double linear_vel = std::hypot(total_desired_vel[0], total_desired_vel[1]);
        double angular_vel = 0.0;
        if (linear_vel > 1e-6) {
            double target_angle = std::atan2(total_desired_vel[1], total_desired_vel[0]);
            double angle_error = normalize_angle(target_angle - current_yaw_);
            
            // Maintain separate time history for angular control
            if (prev_angle_time_valid_) {
                double dt_ang = (now_time - prev_angle_time_).seconds();
                if (dt_ang > 0) {
                    double angle_error_rate = (angle_error - prev_angle_error_) / dt_ang;
                    angular_vel = kp_angular_ * angle_error + kd_angular_ * angle_error_rate;
                } else {
                    angular_vel = kp_angular_ * angle_error;
                }
            } else {
                angular_vel = kp_angular_ * angle_error;
                prev_angle_time_valid_ = true;
            }
            
            prev_angle_error_ = angle_error;
            prev_angle_time_ = now_time;
        }
        linear_vel = std::clamp(linear_vel, -current_max_linear, current_max_linear);
        angular_vel = std::clamp(angular_vel, -current_max_angular, current_max_angular);
        if (at_path_end && distance_error < precise_goal_tolerance_ && std::abs(linear_vel) < velocity_tolerance_) {
            linear_vel = 0.0;
            angular_vel = 0.0;
            RCLCPP_INFO(get_logger(), "Precise goal reached! Distance: %.3f", distance_error);
        }
        current_linear_vel_ = linear_vel;
        return {linear_vel, angular_vel};
    }

    void publish_cmd(double lin, double ang) {
        motion_msgs::msg::MotionCtrl cmd;
        cmd.value.forward = lin;
        cmd.value.left = ang;
        cmd_pub_->publish(cmd);
    }

    void control_loop()
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        
        // Check for emergency stop
        if (emergency_stop_active_.load() || g_emergency_stop.load()) {
            publish_cmd(0.0, 0.0);
            return;
        }
        
        // Check path version to ensure using latest path
        uint64_t latest_version = path_version_.load();
        if (current_path_version_ != latest_version) {
            current_path_version_ = latest_version;
            RCLCPP_DEBUG(get_logger(), "Switched to path version: %lu", current_path_version_.load());
        }
        
        if (!path_updated_ || path_points_.empty()) {
            publish_cmd(0.0, 0.0);
            return;
        }
        
        // Only get robot pose for strategies that need position feedback
        bool tf_available = false;
        if (strategy_ != "open_loop") {
            tf_available = get_robot_pose();
            if (!tf_available && !fallback_to_openloop_) {
                publish_cmd(0.0, 0.0);
                return;
            }
        }
        
        auto current_time = now();
        double elapsed_time = (current_time - path_start_time_).seconds();
        current_time_index_ = std::min(elapsed_time / path_dt_, (double)(path_points_.size() - 1));
        auto [desired_pos, desired_vel] = get_desired_state(current_time_index_);
        
        double linear_vel = 0.0;
        double angular_vel = 0.0;
        bool at_path_end = current_time_index_ >= path_points_.size() - 1;
        
        // Determine control strategy: use fallback mode if TF unavailable
        std::string effective_strategy = (fallback_to_openloop_ || strategy_ == "open_loop") ? "open_loop" : strategy_;
        
        if ((effective_strategy == "pd" || effective_strategy == "pid") && tf_available) {
            double goal_distance = std::hypot(path_points_.back()[0] - current_x_, path_points_.back()[1] - current_y_);
            std::tie(linear_vel, angular_vel) = calculate_pd_control(desired_pos, desired_vel, at_path_end, elapsed_time, goal_distance);
            
            // Check goal reached for PD control
            if (at_path_end && goal_distance < precise_goal_tolerance_ && std::abs(current_linear_vel_) < velocity_tolerance_) {
                RCLCPP_INFO(get_logger(), "Precise goal reached! Final distance: %.3f", goal_distance);
                path_updated_ = false;
            }
        } else {
            // Open-loop control (either by design or fallback)
            linear_vel = std::hypot(desired_vel[0], desired_vel[1]);
            if (linear_vel > 1e-6) {
                double target_angle = std::atan2(desired_vel[1], desired_vel[0]);
                if (tf_available) {
                    double angle_error = normalize_angle(target_angle - current_yaw_);
                    angular_vel = kp_angular_ * angle_error;
                } else {
                    // Use path orientation when TF unavailable
                    size_t idx = static_cast<size_t>(std::min(current_time_index_, (double)path_points_.size() - 1));
                    double path_yaw = path_points_[idx][2];
                    double angle_error = normalize_angle(target_angle - path_yaw);
                    angular_vel = kp_angular_ * angle_error * 0.5; // Reduced gain for safety
                }
            }
            linear_vel = std::clamp(linear_vel, -max_linear_vel_, max_linear_vel_);
            angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
            
            // Check path completion for open-loop control
            if (at_path_end) {
                RCLCPP_INFO(get_logger(), "Path execution completed (open-loop mode)");
                path_updated_ = false;
            }
        }
        
        publish_cmd(linear_vel, angular_vel);
        
        // Periodic status info
        static rclcpp::Time last_info_time = this->now();
        auto now_info = this->now();
        if ((now_info - last_info_time).seconds() > 2.0) {
            RCLCPP_INFO(get_logger(), "Control status: strategy=%s, time_idx=%.2f/%zu, vel=(%.2f,%.2f)", 
                       effective_strategy.c_str(), current_time_index_, path_points_.size()-1, linear_vel, angular_vel);
            last_info_time = now_info;
        }
    }

    double normalize_angle(double a) {
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }

    // ROS params
    double path_dt_, kp_linear_, kd_linear_, kp_angular_, kd_angular_, max_linear_vel_, max_angular_vel_, goal_tolerance_, lookahead_distance_, soft_start_time_, max_pos_error_, feedforward_ramp_time_, precise_goal_tolerance_, velocity_tolerance_, decel_distance_, min_decel_factor_, ki_linear_;
    int control_hz_;
    bool enable_curveadapt_;
    std::string strategy_, map_frame_, robot_odom_frame_, robot_base_frame_, goal_frame_;
    
    // State
    double prev_angle_error_ = 0.0;
    rclcpp::Time prev_angle_time_;
    bool prev_angle_time_valid_ = false;
    std::vector<std::array<double,3>> path_points_;
    std::atomic<uint64_t> path_version_{0};
    std::atomic<uint64_t> current_path_version_{0};
    bool path_updated_ = false;
    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
    double current_linear_vel_ = 0.0;
    double current_time_index_ = 0.0;
    std::array<double,2> prev_pos_error_ = {0.0,0.0};
    std::array<double,2> integral_error_ = {0.0,0.0};
    rclcpp::Time path_start_time_;
    rclcpp::Time prev_time_;
    bool prev_time_valid_;
    std::mutex path_mutex_;
    
    // Emergency stop state
    std::atomic<bool> emergency_stop_active_{false};
    
    // TF fallback mechanism
    int tf_failure_count_ = 0;
    bool fallback_to_openloop_ = false;
    static constexpr int MAX_TF_FAILURES = 5;
    
    // ROS
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiabloTrackNode>();
    
    // Set global node pointer for signal handler
    g_node = node;
    
    // Register signal handler for Ctrl+C
    signal(SIGINT, signal_handler);
    
    RCLCPP_INFO(node->get_logger(), "Emergency stop enabled - press Ctrl+C to stop robot");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
