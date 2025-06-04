#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
from filterpy.kalman import ExtendedKalmanFilter
from scipy.spatial.transform import Rotation


class EKFOdomNode(Node):
    
    def __init__(self):
        super().__init__('ekf_odom_node')
        
        # Parameters
        self.declare_parameter('imu_topic_name', '/diablo/sensor/Imu')
        self.declare_parameter('wheelodom_topic_name', '/diablo/wheel_odom')
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 30.0)
        
        self.imu_topic = self.get_parameter('imu_topic_name').get_parameter_value().string_value
        self.wheelodom_topic = self.get_parameter('wheelodom_topic_name').get_parameter_value().string_value
        self.output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # EKF initialization
        self.ekf = ExtendedKalmanFilter(dim_x=6, dim_z=3)
        self.setup_ekf()
        
        # State: [x, y, theta, vx, vy, omega]
        self.last_predict_time = None
        self.initialized = False
        
        # Debug counters
        self.imu_count = 0
        self.odom_count = 0
        self.tf_count = 0
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 100)
        self.odom_sub = self.create_subscription(Odometry, self.wheelodom_topic, self.odom_callback, 100)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Timer for prediction step
        self.timer = self.create_timer(1.0 / self.publish_rate, self.predict_and_publish)
        
        self.get_logger().info(f'EKF node started, subscribing to IMU: {self.imu_topic}, Odom: {self.wheelodom_topic}')
    
    def setup_ekf(self):
        # Initial state [x, y, theta, vx, vy, omega]
        self.ekf.x = np.array([0., 0., 0., 0., 0., 0.])
        
        # Initial covariance
        self.ekf.P = np.eye(6) * 0.1
        
        # Process noise covariance Q
        self.ekf.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        
        # Measurement noise covariance R (will be set per sensor)
        self.ekf.R = np.eye(3) * 0.1
    
    def predict_step(self, dt):
        """Prediction step using motion model"""
        old_state = self.ekf.x.copy()
        
        def f(x, dt):
            # State transition function
            # x = [x, y, theta, vx, vy, omega]
            x_new = np.zeros(6)
            theta = x[2]
            vx, vy, omega = x[3], x[4], x[5]
            
            # Position update
            x_new[0] = x[0] + (vx * np.cos(theta) - vy * np.sin(theta)) * dt
            x_new[1] = x[1] + (vx * np.sin(theta) + vy * np.cos(theta)) * dt
            x_new[2] = x[2] + omega * dt
            
            # Velocity assumed to change slowly
            x_new[3] = vx
            x_new[4] = vy 
            x_new[5] = omega
            
            return x_new
        
        def F_jacobian(x, dt):
            # Jacobian of state transition function
            theta = x[2]
            vx, vy = x[3], x[4]
            
            F = np.eye(6)
            F[0, 2] = (-vx * np.sin(theta) - vy * np.cos(theta)) * dt
            F[0, 3] = np.cos(theta) * dt
            F[0, 4] = -np.sin(theta) * dt
            F[1, 2] = (vx * np.cos(theta) - vy * np.sin(theta)) * dt
            F[1, 3] = np.sin(theta) * dt
            F[1, 4] = np.cos(theta) * dt
            F[2, 5] = dt
            
            return F
        
        # Set F matrix and predict
        self.ekf.F = F_jacobian(self.ekf.x, dt)
        self.ekf.predict()
        
        # Manual state update with nonlinear function
        self.ekf.x = f(self.ekf.x, dt)
        
        # Normalize angle
        self.ekf.x[2] = self.normalize_angle(self.ekf.x[2])
        
        self.get_logger().info(f'EKF Predict: dt={dt:.4f}s, state=[{self.ekf.x[0]:.3f}, {self.ekf.x[1]:.3f}, {self.ekf.x[2]:.3f}, {self.ekf.x[3]:.3f}, {self.ekf.x[4]:.3f}, {self.ekf.x[5]:.3f}]')
    
    def imu_callback(self, msg):
        """Handle IMU measurements"""
        if not self.initialized:
            self.get_logger().warn("IMU received but EKF not initialized yet")
            return
            
        self.imu_count += 1
        
        # Extract angular velocity
        omega_z = msg.angular_velocity.z
        old_state = self.ekf.x.copy()
        
        # Update with IMU data (only angular velocity for simplicity)
        z = np.array([omega_z])
        
        def h_imu(x):
            return np.array([x[5]])  # omega measurement
        
        def H_imu(x):
            H = np.zeros((1, 6))
            H[0, 5] = 1.0
            return H
        
        # Set measurement noise and dimensions
        R_imu = np.array([[0.01]])
        
        # Update EKF
        self.ekf.dim_z = 1
        self.ekf.R = R_imu
        self.ekf.update(z, HJacobian=H_imu, Hx=h_imu)
        self.ekf.dim_z = 3  # Reset
        
        if self.imu_count % 20 == 0:  # Log every 20 messages
            self.get_logger().info(f'IMU #{self.imu_count}: omega_z={omega_z:.4f}, state_change=[{self.ekf.x[5]-old_state[5]:.4f}]')
    
    def odom_callback(self, msg):
        """Handle wheel odometry measurements"""
        if not self.initialized:
            # Initialize EKF with first odometry reading
            self.initialize_with_odom(msg)
            return
        
        self.odom_count += 1
        old_state = self.ekf.x.copy()
        
        # Extract pose and twist
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to yaw
        r = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        yaw = r.as_euler('xyz')[2]
        
        vx = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        
        # Measurement vector: [x, y, theta, vx, omega]
        z = np.array([x, y, yaw, vx, omega])
        
        def h_odom(x):
            return np.array([x[0], x[1], x[2], x[3], x[5]])
        
        def H_odom(x):
            H = np.zeros((5, 6))
            H[0, 0] = 1.0  # x
            H[1, 1] = 1.0  # y
            H[2, 2] = 1.0  # theta
            H[3, 3] = 1.0  # vx
            H[4, 5] = 1.0  # omega
            return H
        
        # Set measurement noise based on odometry covariance
        R_odom = np.diag([0.01, 0.01, 0.05, 0.01, 0.05])
        
        # Update EKF
        self.ekf.dim_z = 5
        self.ekf.R = R_odom
        self.ekf.update(z, HJacobian=H_odom, Hx=h_odom)
        self.ekf.dim_z = 3  # Reset
        
        # Normalize angle after update
        self.ekf.x[2] = self.normalize_angle(self.ekf.x[2])
        
        self.get_logger().info(f'Odom #{self.odom_count}: received=[{x:.3f}, {y:.3f}, {yaw:.3f}, {vx:.3f}, {omega:.3f}]')
        self.get_logger().info(f'EKF Update: state=[{self.ekf.x[0]:.3f}, {self.ekf.x[1]:.3f}, {self.ekf.x[2]:.3f}, {self.ekf.x[3]:.3f}, {self.ekf.x[4]:.3f}, {self.ekf.x[5]:.3f}]')
    
    def initialize_with_odom(self, msg):
        """Initialize EKF state with first odometry reading"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        
        r = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        yaw = r.as_euler('xyz')[2]
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        self.ekf.x = np.array([x, y, yaw, vx, vy, omega])
        self.initialized = True
        self.last_predict_time = self.get_clock().now()
        
        self.get_logger().info(f'EKF initialized with pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
    
    def predict_and_publish(self):
        """Timer callback for prediction and publishing"""
        if not self.initialized:
            return
        
        current_time = self.get_clock().now()
        if self.last_predict_time is not None:
            dt = (current_time - self.last_predict_time).nanoseconds / 1e9
            if dt > 0:
                self.predict_step(dt)
        
        self.last_predict_time = current_time
        self.publish_tf(current_time)
    
    def publish_tf(self, stamp):
        """Publish TF transform from odom to base_link"""
        self.tf_count += 1
        
        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.output_frame
        transform.child_frame_id = self.base_frame
        
        # Extract pose from EKF state
        x, y, theta = self.ekf.x[0], self.ekf.x[1], self.ekf.x[2]
        
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = 0.0
        
        # Convert yaw to quaternion
        q = self.yaw_to_quaternion(theta)
        transform.transform.rotation = q
        
        try:
            self.tf_broadcaster.sendTransform(transform)
            
            if self.tf_count % 50 == 0:  # Log every 50 TF publications
                self.get_logger().info(f'TF #{self.tf_count}: {self.output_frame}->{self.base_frame} [x={x:.3f}, y={y:.3f}, θ={theta:.3f}]')
                self.get_logger().info(f'TF Quaternion: [x={q.x:.3f}, y={q.y:.3f}, z={q.z:.3f}, w={q.w:.3f}]')
                self.get_logger().info(f'Stats: IMU msgs={self.imu_count}, Odom msgs={self.odom_count}, TF pubs={self.tf_count}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to publish TF: {e}')
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = EKFOdomNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
