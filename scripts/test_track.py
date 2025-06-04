#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

class TestTrackNode(Node):
    def __init__(self):
        super().__init__('test_track_node')
        
        # Declare parameters
        self.declare_parameter('path_type', 'line')  # line, rectangle, circle, s_curve
        self.declare_parameter('path_length', 1.5)
        self.declare_parameter('point_spacing', 0.1)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('loop_publish', False)
        self.declare_parameter('frame_id', 'base_link')
        
        # Get parameters
        self.path_type_ = self.get_parameter('path_type').get_parameter_value().string_value
        self.path_length_ = self.get_parameter('path_length').get_parameter_value().double_value
        self.point_spacing_ = self.get_parameter('point_spacing').get_parameter_value().double_value
        self.publish_rate_ = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.loop_publish_ = self.get_parameter('loop_publish').get_parameter_value().bool_value
        self.frame_id_ = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Publisher
        self.path_publisher_ = self.create_publisher(Path, '/planned_path', 5)
        
        # Generate test path
        self.test_path_ = self._generate_path()
        
        # Timer for publishing
        if self.loop_publish_:
            self.create_timer(1.0 / self.publish_rate_, self._publish_path)
        else:
            # Publish once after a short delay
            self.create_timer(10.0, self._publish_once)
        
        self.get_logger().info(f'Test Track Node initialized with path type: {self.path_type_}')
        self.get_logger().info(f'Generated path with {len(self.test_path_.poses)} points')

    def _generate_path(self):
        """Generate test path based on path_type parameter"""
        path = Path()
        path.header.frame_id = self.frame_id_
        
        if self.path_type_ == 'line':
            poses = self._generate_line_path()
        elif self.path_type_ == 'rectangle':
            poses = self._generate_rectangle_path()
        elif self.path_type_ == 'circle':
            poses = self._generate_circle_path()
        elif self.path_type_ == 's_curve':
            poses = self._generate_s_curve_path()
        else:
            self.get_logger().error(f"Unknown path type: {self.path_type_}")
            poses = self._generate_line_path()
        
        path.poses = poses
        return path

    def _generate_line_path(self):
        """Generate straight line path"""
        poses = []
        num_points = int(self.path_length_ / self.point_spacing_) + 1
        
        for i in range(num_points):
            x = i * self.point_spacing_
            y = 0.0
            yaw = 0.0
            
            pose = self._create_pose_stamped(x, y, yaw)
            poses.append(pose)
        
        return poses

    def _generate_rectangle_path(self):
        """Generate rectangular path"""
        poses = []
        side_length = self.path_length_ / 2.0  # Half of total length per side
        
        # Side 1: Bottom (left to right)
        num_points = int(side_length / self.point_spacing_) + 1
        for i in range(num_points):
            x = i * self.point_spacing_
            y = 0.0
            yaw = 0.0
            poses.append(self._create_pose_stamped(x, y, yaw))
        
        # Side 2: Right (bottom to top)
        for i in range(1, num_points):
            x = side_length
            y = i * self.point_spacing_
            yaw = math.pi / 2
            poses.append(self._create_pose_stamped(x, y, yaw))
        
        # Side 3: Top (right to left)
        for i in range(1, num_points):
            x = side_length - i * self.point_spacing_
            y = side_length
            yaw = math.pi
            poses.append(self._create_pose_stamped(x, y, yaw))
        
        # Side 4: Left (top to bottom)
        for i in range(1, num_points):
            x = 0.0
            y = side_length - i * self.point_spacing_
            yaw = -math.pi / 2
            poses.append(self._create_pose_stamped(x, y, yaw))
        
        return poses

    def _generate_circle_path(self):
        """Generate circular path"""
        poses = []
        radius = self.path_length_ / (2 * math.pi)  # Calculate radius from desired path length
        circumference = 2 * math.pi * radius
        num_points = int(circumference / self.point_spacing_) + 1
        
        for i in range(num_points):
            angle = 2 * math.pi * i / (num_points - 1)
            x = radius * math.cos(angle) + radius  # Offset to start at origin
            y = radius * math.sin(angle)
            yaw = angle + math.pi / 2  # Tangent direction
            
            poses.append(self._create_pose_stamped(x, y, yaw))
        
        return poses

    def _generate_s_curve_path(self):
        """Generate S-curve path"""
        poses = []
        num_points = int(self.path_length_ / self.point_spacing_) + 1
        
        for i in range(num_points):
            x = i * self.point_spacing_
            # S-curve using sine function
            y = math.sin(2 * math.pi * x / self.path_length_) * (self.path_length_ / 8)
            
            # Calculate tangent angle
            if i < num_points - 1:
                x_next = (i + 1) * self.point_spacing_
                y_next = math.sin(2 * math.pi * x_next / self.path_length_) * (self.path_length_ / 8)
                yaw = math.atan2(y_next - y, x_next - x)
            else:
                # Use previous point's direction for last point
                if poses:
                    prev_quat = poses[-1].pose.orientation
                    yaw = math.atan2(2 * (prev_quat.w * prev_quat.z + prev_quat.x * prev_quat.y),
                                   1 - 2 * (prev_quat.y * prev_quat.y + prev_quat.z * prev_quat.z))
                else:
                    yaw = 0.0
            
            poses.append(self._create_pose_stamped(x, y, yaw))
        
        return poses

    def _create_pose_stamped(self, x, y, yaw):
        """Create a PoseStamped message from x, y, yaw"""
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id_
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

    def _publish_path(self):
        """Publish the test path periodically"""
        self.test_path_.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher_.publish(self.test_path_)
        self.get_logger().debug(f'Published {self.path_type_} path with {len(self.test_path_.poses)} points')

    def _publish_once(self):
        """Publish the test path once and destroy timer"""
        self.test_path_.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher_.publish(self.test_path_)
        self.get_logger().info(f'Published {self.path_type_} path once with {len(self.test_path_.poses)} points')

def main(args=None):
    rclpy.init(args=args)
    node = TestTrackNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
