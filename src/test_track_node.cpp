#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <string>
#include <vector>

class TestTrackNode : public rclcpp::Node
{
public:
    TestTrackNode() : Node("test_track_node")
    {
        path_type_ = declare_parameter<std::string>("path_type", "line");
        path_length_ = declare_parameter<double>("path_length", 1.5);
        point_spacing_ = declare_parameter<double>("point_spacing", 0.1);
        publish_rate_ = declare_parameter<double>("publish_rate", 1.0);
        loop_publish_ = declare_parameter<bool>("loop_publish", false);
        frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
        turn_time_s_ = declare_parameter<double>("turn_time_s", 1.0);

        path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", 5);

        test_path_ = generate_path();

        if (loop_publish_) {
            timer_ = create_wall_timer(
                std::chrono::duration<double>(1.0 / publish_rate_),
                std::bind(&TestTrackNode::publish_path, this));
        } else {
            timer_ = create_wall_timer(
                std::chrono::duration<double>(10.0),
                std::bind(&TestTrackNode::publish_once, this));
        }

        RCLCPP_INFO(get_logger(), "Test Track Node initialized with path type: %s", path_type_.c_str());
        RCLCPP_INFO(get_logger(), "Generated path with %zu points", test_path_.poses.size());
    }

private:
    nav_msgs::msg::Path generate_path()
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = frame_id_;

        if (path_type_ == "line") {
            path.poses = generate_line_path();
        } else if (path_type_ == "rectangle") {
            path.poses = generate_rectangle_path();
        } else if (path_type_ == "circle") {
            path.poses = generate_circle_path();
        } else if (path_type_ == "s_curve") {
            path.poses = generate_s_curve_path();
        } else {
            RCLCPP_ERROR(get_logger(), "Unknown path type: %s", path_type_.c_str());
            path.poses = generate_line_path();
        }

        // Debug: output start and end points with angles, and all trajectory points
        if (!path.poses.empty()) {
            // Extract yaw from quaternion for start and end points
            auto extract_yaw = [](const geometry_msgs::msg::Quaternion& q) {
                return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            };
            
            double start_yaw = extract_yaw(path.poses.front().pose.orientation);
            double end_yaw = extract_yaw(path.poses.back().pose.orientation);
            
            RCLCPP_INFO(get_logger(), "Path start: (%.3f, %.3f, %.1f°), end: (%.3f, %.3f, %.1f°)",
                        path.poses.front().pose.position.x, path.poses.front().pose.position.y,
                        start_yaw * 180.0 / M_PI,
                        path.poses.back().pose.position.x, path.poses.back().pose.position.y,
                        end_yaw * 180.0 / M_PI);
            
            // Output all trajectory points
            for (size_t i = 0; i < path.poses.size(); ++i) {
                double yaw = extract_yaw(path.poses[i].pose.orientation);
                RCLCPP_INFO(get_logger(), "Idx %zu: [%.3f, %.3f, %.1f°]", 
                           i, path.poses[i].pose.position.x, path.poses[i].pose.position.y,
                           yaw * 180.0 / M_PI);
            }
        }
        if (point_spacing_ > path_length_ / 5.0) {
            RCLCPP_WARN(get_logger(), "Point spacing is large relative to path length, may reduce tracking precision.");
        }
        return path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> generate_line_path()
    {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        int num_points = static_cast<int>(path_length_ / point_spacing_) + 1;
        for (int i = 0; i < num_points; ++i) {
            poses.push_back(create_pose_stamped(i * point_spacing_, 0.0, 0.0));
        }
        if (!poses.empty()) {
            poses.erase(poses.begin());
        }
        return poses;
    }

    std::vector<geometry_msgs::msg::PoseStamped> generate_arc_turn(
        double center_x, double center_y, double radius, 
        double start_angle, double end_angle)
    {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        double arc_length = radius * std::abs(end_angle - start_angle);
        int num_points = static_cast<int>(arc_length / point_spacing_) + 1;
        
        for (int i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / (num_points - 1);
            double angle = start_angle + t * (end_angle - start_angle);
            double x = center_x + radius * std::cos(angle);
            double y = center_y + radius * std::sin(angle);
            double yaw = angle + M_PI / 2;
            poses.push_back(create_pose_stamped(x, y, yaw));
        }
        return poses;
    }

    std::vector<geometry_msgs::msg::PoseStamped> generate_rectangle_path()
    {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        double side_length = path_length_; // path_length就是正方形边长
        
        int straight_points = static_cast<int>(side_length / point_spacing_) + 1;
        
        // 直线段 (向右)
        for (int i = 0; i < straight_points; ++i) {
            poses.push_back(create_pose_stamped(i * point_spacing_, 0.0, 0.0));
        }
        
        // 原地左转90度 (0° -> 90°)
        double time_interval = 0.1; // 固定时间间隔 0.1s
        int turn_points = static_cast<int>(turn_time_s_ / time_interval);
        for (int i = 0; i < turn_points; ++i) {
            double t = static_cast<double>(i) / (turn_points - 1);
            double yaw = t * M_PI / 2; // 从0度转到90度
            poses.push_back(create_pose_stamped(side_length, 0.0, yaw));
        }

        if (!poses.empty()) {
            poses.erase(poses.begin());
        }
        return poses;
    }

    std::vector<geometry_msgs::msg::PoseStamped> generate_circle_path()
    {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        double radius = path_length_; // path_length就是半径
        double arc_length = M_PI * radius / 2; // 1/4圆弧长度
        int num_points = static_cast<int>(arc_length / point_spacing_) + 1;

        for (int i = 0; i < num_points; ++i) {
            double angle = M_PI / 2 * i / (num_points - 1); // 从0到π/2 (90度)
            // 圆心在 (0, radius)，从 (0, 0) 开始向左拐
            double x = radius * std::sin(angle); // x 增大
            double y = radius * (1 - std::cos(angle)); // y 增大
            double yaw = angle; // yaw 从 0 增大到 π/2
            poses.push_back(create_pose_stamped(x, y, yaw));
        }
        if (!poses.empty()) {
            poses.erase(poses.begin());
        }
        return poses;
    }

    std::vector<geometry_msgs::msg::PoseStamped> generate_s_curve_path()
    {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        int num_points = static_cast<int>(path_length_ / point_spacing_) + 1;

        for (int i = 0; i < num_points; ++i) {
            double x = i * point_spacing_;
            double y = std::sin(2 * M_PI * x / path_length_) * (path_length_ / 8);
            double yaw;

            if (i < num_points - 1) {
                double x_next = (i + 1) * point_spacing_;
                double y_next = std::sin(2 * M_PI * x_next / path_length_) * (path_length_ / 8);
                yaw = std::atan2(y_next - y, x_next - x);
            } else if (!poses.empty()) {
                // Last point: keep yaw same as previous
                yaw = std::atan2(
                    2 * (poses.back().pose.orientation.w * poses.back().pose.orientation.z +
                         poses.back().pose.orientation.x * poses.back().pose.orientation.y),
                    1 - 2 * (poses.back().pose.orientation.y * poses.back().pose.orientation.y +
                             poses.back().pose.orientation.z * poses.back().pose.orientation.z)
                );
            } else {
                yaw = 0.0;
            }

            poses.push_back(create_pose_stamped(x, y, yaw));
        }
        if (!poses.empty()) {
            poses.erase(poses.begin());
        }
        return poses;
    }

    geometry_msgs::msg::PoseStamped create_pose_stamped(double x, double y, double yaw)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id_;
        pose.header.stamp = now();

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        return pose;
    }

    void publish_path()
    {
        test_path_.header.stamp = now();
        path_pub_->publish(test_path_);
        RCLCPP_INFO(get_logger(), "Published /planned_path topic with %s path (%zu points)", path_type_.c_str(), test_path_.poses.size());
    }

    void publish_once()
    {
        test_path_.header.stamp = now();
        path_pub_->publish(test_path_);
        RCLCPP_INFO(get_logger(), "Published /planned_path topic once with %s path (%zu points)", path_type_.c_str(), test_path_.poses.size());
    }

    // Parameters
    std::string path_type_, frame_id_;
    double path_length_, point_spacing_, publish_rate_, turn_time_s_;
    bool loop_publish_;

    // ROS
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    nav_msgs::msg::Path test_path_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestTrackNode>());
    rclcpp::shutdown();
    return 0;
}
