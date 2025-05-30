#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Vector3
from motion_msgs.msg import LegMotors

class WheelOdometryNode(Node):
    
    def __init__(self):
        super().__init__('wheel_odometry_node')
        
        # Parameters
        self.declare_parameter('wheel_radius', 0.18)
        self.declare_parameter('wheel_base', 0.485)

        self.declare_parameter('robot_odom_frame', 'odom')
        self.declare_parameter('robot_base_frame', 'base_link')

        self.wheel_radius_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base_ = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.robot_odom_frame_ = self.get_parameter('robot_odom_frame').get_parameter_value().string_value
        self.robot_base_frame_ = self.get_parameter('robot_base_frame').get_parameter_value().string_value

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = None
        
        # Publishers and Subscribers
        self.odom_pub = self.create_publisher(Odometry, 'diablo/wheel_odom', 10)
        self.motors_sub = self.create_subscription(
            LegMotors,
            'diablo/sensor/Motors',
            self.motors_callback,
            10
        )
        
        self.get_logger().info(f'Wheel odometry node started with radius={self.wheel_radius_}, wheelbase={self.wheel_base_}')
    
    def motors_callback(self, msg):
        current_time = msg.header.stamp
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Calculate time difference
        dt = (current_time.sec - self.last_time.sec) + (current_time.nanosec - self.last_time.nanosec) / 1e9
        
        if dt <= 0:
            return
        
        # Get wheel velocities (rad/s)
        left_vel = msg.left_wheel_vel
        right_vel = msg.right_wheel_vel
        
        # Calculate linear and angular velocities
        v_linear = (left_vel + right_vel) / 2.0 * self.wheel_radius_
        v_angular = (right_vel - left_vel) / self.wheel_base_ * self.wheel_radius_
        
        # Update position
        self.x += v_linear * math.cos(self.yaw) * dt
        self.y += v_linear * math.sin(self.yaw) * dt
        self.yaw += v_angular * dt
        
        # Normalize yaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.robot_odom_frame_
        odom.child_frame_id = self.robot_base_frame_
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)
        
        # Velocity
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = v_angular
        
        # Covariance matrices
        # Position covariance
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[35] = 0.05  # yaw
        
        # Velocity covariance
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[35] = 0.05  # vyaw
        
        self.odom_pub.publish(odom)
        self.last_time = current_time
        self.get_logger().info(
            f"Published odometry: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}, "
            f"v_linear={v_linear:.2f}, v_angular={v_angular:.2f}")

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception in spin: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
