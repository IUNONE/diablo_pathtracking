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
        path_type_ = declare_parameter("path_type", std::string("line"));
        path_length_ = declare_parameter("path_length", 1.5);
        point_spacing_ = declare_parameter("point_spacing", 0.1);
        publish_rate_ = declare_parameter("publish_rate", 1.0);
        loop_publish_ = declare_parameter("loop_publish", false);
        frame_id_ = declare_parameter("frame_id", std::string("base_link"));

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

        // Debug: output start and end points, path length approximation
        if (!path.poses.empty()) {
            double length = 0.0;
            for (size_t i = 1; i < path.poses.size(); ++i) {
                double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
                double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
                length += std::hypot(dx, dy);
            }
            RCLCPP_INFO(get_logger(), "Path start: (%.3f, %.3f), end: (%.3f, %.3f), length: %.3f",
                        path.poses.front().pose.position.x, path.poses.front().pose.position.y,
                        path.poses.back().pose.position.x, path.poses.back().pose.position.y,
                        length);
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

    std::vector<geometry_msgs::msg::PoseStamped> generate_rectangle_path()
    {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        double side_length = path_length_ / 2.0;
        int num_points = static_cast<int>(side_length / point_spacing_) + 1;

        for (int i = 0; i < num_points; ++i)
            poses.push_back(create_pose_stamped(i * point_spacing_, 0.0, 0.0));
        for (int i = 1; i < num_points; ++i)
            poses.push_back(create_pose_stamped(side_length, i * point_spacing_, M_PI / 2));
        for (int i = 1; i < num_points; ++i)
            poses.push_back(create_pose_stamped(side_length - i * point_spacing_, side_length, M_PI));
        for (int i = 1; i < num_points; ++i)
            poses.push_back(create_pose_stamped(0.0, side_length - i * point_spacing_, -M_PI / 2));

        if (!poses.empty()) {
            poses.erase(poses.begin());
        }
        return poses;
    }

    std::vector<geometry_msgs::msg::PoseStamped> generate_circle_path()
    {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        double radius = path_length_ / (2 * M_PI);
        double circumference = 2 * M_PI * radius;
        int num_points = static_cast<int>(circumference / point_spacing_) + 1;

        for (int i = 0; i < num_points; ++i) {
            double angle = 2 * M_PI * i / (num_points - 1);
            double x = radius * std::cos(angle) + radius;
            double y = radius * std::sin(angle);
            double yaw = angle + M_PI / 2;
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
        RCLCPP_DEBUG(get_logger(), "Published %s path with %zu points", path_type_.c_str(), test_path_.poses.size());
    }

    void publish_once()
    {
        test_path_.header.stamp = now();
        path_pub_->publish(test_path_);
        RCLCPP_INFO(get_logger(), "Published %s path once with %zu points", path_type_.c_str(), test_path_.poses.size());
    }

    // Parameters
    std::string path_type_, frame_id_;
    double path_length_, point_spacing_, publish_rate_;
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
