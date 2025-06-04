#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Path
from motion_msgs.msg import MotionCtrl
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math
import time
from threading import Lock

class DiabloTrackNode(Node):

    def __init__(self):
        super().__init__('diablo_tracking_node')

        # Declare parameters with default values
        self.declare_parameter('path_dt', 0.1)
        self.declare_parameter('control_hz', 100)
        self.declare_parameter('strategy', 'pd')
        
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_odom_frame', 'odom')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('goal_frame', 'goal')
        
        # PD controller parameters
        self.declare_parameter('kp_linear', 1.0)
        self.declare_parameter('kd_linear', 0.1)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('kd_angular', 0.2)
        self.declare_parameter('max_linear_vel', 2.0)
        self.declare_parameter('max_angular_vel', 0.2)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('lookahead_distance', 0.5)
        
        # Anti-overshoot parameters
        self.declare_parameter('soft_start_time', 2.0)
        self.declare_parameter('max_pos_error', 1.0)
        self.declare_parameter('feedforward_ramp_time', 1.5)
        
        # Precise control parameters
        self.declare_parameter('precise_goal_tolerance', 0.02)
        self.declare_parameter('velocity_tolerance', 0.05)
        self.declare_parameter('decel_distance', 0.2)
        self.declare_parameter('min_decel_factor', 0.1)
        self.declare_parameter('ki_linear', 0.05)

        self.path_dt_ = self.get_parameter('path_dt').get_parameter_value().double_value
        self.control_hz_ = self.get_parameter('control_hz').get_parameter_value().integer_value
        self.strategy_ = self.get_parameter('strategy').get_parameter_value().string_value

        self.map_frame_ = self.get_parameter('map_frame').get_parameter_value().string_value
        self.robot_base_frame_ = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.goal_frame_ = self.get_parameter('goal_frame').get_parameter_value().string_value
        self.robot_odom_frame_ = self.get_parameter('robot_odom_frame').get_parameter_value().string_value

        # PD parameters
        self.kp_linear_ = self.get_parameter('kp_linear').get_parameter_value().double_value
        self.kd_linear_ = self.get_parameter('kd_linear').get_parameter_value().double_value
        self.kp_angular_ = self.get_parameter('kp_angular').get_parameter_value().double_value
        self.kd_angular_ = self.get_parameter('kd_angular').get_parameter_value().double_value
        self.max_linear_vel_ = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel_ = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.goal_tolerance_ = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.lookahead_distance_ = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        
        # Anti-overshoot parameters
        self.soft_start_time_ = self.get_parameter('soft_start_time').get_parameter_value().double_value
        self.max_pos_error_ = self.get_parameter('max_pos_error').get_parameter_value().double_value
        self.feedforward_ramp_time_ = self.get_parameter('feedforward_ramp_time').get_parameter_value().double_value
        
        # Precise control parameters
        self.precise_goal_tolerance_ = self.get_parameter('precise_goal_tolerance').get_parameter_value().double_value
        self.velocity_tolerance_ = self.get_parameter('velocity_tolerance').get_parameter_value().double_value
        self.decel_distance_ = self.get_parameter('decel_distance').get_parameter_value().double_value
        self.min_decel_factor_ = self.get_parameter('min_decel_factor').get_parameter_value().double_value
        self.ki_linear_ = self.get_parameter('ki_linear').get_parameter_value().double_value

        assert self.strategy_ in ['pd'], f"Invalid strategy: {self.strategy_}"

        # Path tracking variables
        self.current_path_ = None
        self.path_points_ = None
        self.path_updated_ = False
        self.path_start_time_ = None
        self.current_time_index_ = 0.0
        self.path_lock_ = Lock()
        
        # Error tracking for PID control
        self.prev_pos_error_ = np.array([0.0, 0.0])
        self.integral_error_ = np.array([0.0, 0.0])
        self.prev_time_ = None
        self.current_linear_vel_ = 0.0

        # TF buffer and listener
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        # Robot current pose
        self.current_x_ = 0.0
        self.current_y_ = 0.0
        self.current_yaw_ = 0.0

        # Publishers and subscribers
        self.path_suber = self.create_subscription(Path, 'planned_path', self._pathcallback, 5)
        self.cmd_puber = self.create_publisher(MotionCtrl, 'diablo/MotionCmd', 2)
        
        # High frequency control timer
        self.create_timer(1.0 / self.control_hz_, self.control_loop)
        
        self.get_logger().info('Diablo Tracking Node Initialized')

    def _pathcallback(self, msg: Path):
        """Process new path messages, always use the latest one"""
        with self.path_lock_:
            if len(msg.poses) < 2:
                self.get_logger().warn("Received path with less than 2 points, ignoring")
                return
                
            # Convert path to numpy arrays for efficient processing
            self.path_points_ = np.array([[pose.pose.position.x, 
                                          pose.pose.position.y,
                                          self._get_yaw_from_quaternion(pose.pose.orientation)] 
                                         for pose in msg.poses])
            
            self.current_path_ = msg
            self.path_updated_ = True
            self.path_start_time_ = self.get_clock().now()
            self.current_time_index_ = 0.0
            
            # Reset PID error tracking
            self.prev_pos_error_ = np.array([0.0, 0.0])
            self.integral_error_ = np.array([0.0, 0.0])
            self.prev_time_ = None
            self.current_linear_vel_ = 0.0
            
            self.get_logger().info(f"New path received with {len(msg.poses)} points")

    def _get_yaw_from_quaternion(self, quat):
        """Extract yaw angle from quaternion"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _get_robot_pose(self):
        """Get robot current pose from TF"""
        try:
            transform = self.tf_buffer_.lookup_transform(
                self.robot_odom_frame_, 
                self.robot_base_frame_, 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            self.current_x_ = transform.transform.translation.x
            self.current_y_ = transform.transform.translation.y
            self.current_yaw_ = self._get_yaw_from_quaternion(transform.transform.rotation)
            return True
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f"TF lookup failed: {e}")
            return False

    def _get_desired_state(self, time_index):
        """Calculate desired position and velocity based on time index"""
        if self.path_points_ is None or len(self.path_points_) == 0:
            return None, None
            
        # Handle boundary cases
        if time_index < 0:
            desired_pos = self.path_points_[0][:2]
            desired_vel = np.array([0.0, 0.0])
            return desired_pos, desired_vel
            
        if time_index >= len(self.path_points_) - 1:
            desired_pos = self.path_points_[-1][:2]
            desired_vel = np.array([0.0, 0.0])
            return desired_pos, desired_vel
        
        # Interpolate position between path points
        idx = int(time_index)
        frac = time_index - idx
        pos_current = self.path_points_[idx][:2]
        pos_next = self.path_points_[idx + 1][:2]
        desired_pos = pos_current + frac * (pos_next - pos_current)
        
        # Calculate desired velocity using forward difference
        desired_vel = (pos_next - pos_current) / self.path_dt_
        
        return desired_pos, desired_vel

    def _calculate_control_commands(self, desired_pos, desired_vel):
        """Calculate linear and angular velocities using PID control with precise stopping"""
        if desired_pos is None or desired_vel is None:
            return 0.0, 0.0
            
        current_pos = np.array([self.current_x_, self.current_y_])
        pos_error = desired_pos - current_pos
        distance_error = np.linalg.norm(pos_error)
        
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.path_start_time_).nanoseconds * 1e-9
        
        # Check if at end of path for precise control
        at_path_end = self.current_time_index_ >= len(self.path_points_) - 1
        
        # Strategy 1: Soft start - gradually increase max velocity
        soft_start_factor = min(1.0, elapsed_time / self.soft_start_time_)
        current_max_linear = self.max_linear_vel_ * soft_start_factor
        current_max_angular = self.max_angular_vel_ * soft_start_factor
        
        # Strategy 2: Progressive deceleration near goal
        if at_path_end and distance_error < self.decel_distance_:
            decel_factor = max(self.min_decel_factor_, distance_error / self.decel_distance_)
            current_max_linear *= decel_factor
            current_max_angular *= decel_factor
        
        # Strategy 3: Limit position error to prevent large control outputs
        pos_error_magnitude = np.linalg.norm(pos_error)
        if pos_error_magnitude > self.max_pos_error_:
            pos_error = pos_error * (self.max_pos_error_ / pos_error_magnitude)
        
        # PID control for position error
        if self.prev_time_ is not None:
            dt = (current_time - self.prev_time_).nanoseconds * 1e-9
            if dt > 0:
                # Add integral term
                self.integral_error_ += pos_error * dt
                # Prevent integral windup
                max_integral = 1.0 / self.ki_linear_ if self.ki_linear_ > 0 else 1.0
                self.integral_error_ = np.clip(self.integral_error_, -max_integral, max_integral)
                
                # Calculate derivative
                error_rate = (pos_error - self.prev_pos_error_) / dt
                
                # PID controller
                desired_vel_pid = (self.kp_linear_ * pos_error + 
                                  self.ki_linear_ * self.integral_error_ + 
                                  self.kd_linear_ * error_rate)
                self.prev_pos_error_ = pos_error
            else:
                desired_vel_pid = self.kp_linear_ * pos_error
        else:
            desired_vel_pid = self.kp_linear_ * pos_error
            self.prev_pos_error_ = pos_error
            
        self.prev_time_ = current_time
        
        # Strategy 4: Progressive feedforward with velocity compensation
        feedforward_weight = min(1.0, elapsed_time / self.feedforward_ramp_time_)
        if at_path_end:
            feedforward_weight *= 0.5  # Reduce feedforward near goal
        
        # Combine feedforward and feedback
        total_desired_vel = feedforward_weight * desired_vel + desired_vel_pid
        
        # Convert to robot frame commands
        linear_vel = np.linalg.norm(total_desired_vel)
        if linear_vel > 1e-6:
            target_angle = math.atan2(total_desired_vel[1], total_desired_vel[0])
            angle_error = target_angle - self.current_yaw_
            
            # Normalize angle error to [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi
                
            angular_vel = self.kp_angular_ * angle_error
        else:
            angular_vel = 0.0
        
        # Apply dynamic velocity limits
        linear_vel = np.clip(linear_vel, -current_max_linear, current_max_linear)
        angular_vel = np.clip(angular_vel, -current_max_angular, current_max_angular)
        
        # Store current velocity for stopping judgment
        self.current_linear_vel_ = linear_vel
        
        # Precise stopping condition
        if at_path_end:
            if (distance_error < self.precise_goal_tolerance_ and 
                abs(linear_vel) < self.velocity_tolerance_):
                linear_vel = 0.0
                angular_vel = 0.0
                self.get_logger().info(f"Precise goal reached! Distance: {distance_error:.3f}m")
        
        return linear_vel, angular_vel

    def _pub_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0):
        """Publish motion command"""
        cmd = MotionCtrl()
        cmd.value.forward = linear_x
        cmd.value.left = angular_z
        self.cmd_puber.publish(cmd)

    def control_loop(self):
        """High frequency control loop"""
        with self.path_lock_:
            # Check if we have a valid path
            if not self.path_updated_ or self.path_points_ is None:
                self._pub_cmd(0.0, 0.0)
                return
                
            # Get robot current pose
            if not self._get_robot_pose():
                self.get_logger().warn("Failed to get robot pose, stopping")
                self._pub_cmd(0.0, 0.0)
                return
                
            # Calculate current time index
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.path_start_time_).nanoseconds * 1e-9
            self.current_time_index_ = elapsed_time / self.path_dt_
            
            # Get desired state based on time
            desired_pos, desired_vel = self._get_desired_state(self.current_time_index_)
            if desired_pos is None:
                self._pub_cmd(0.0, 0.0)
                return
                
            # Calculate control commands
            linear_vel, angular_vel = self._calculate_control_commands(desired_pos, desired_vel)
            
            # Publish command
            self._pub_cmd(linear_vel, angular_vel)
            
            # Check if reached goal with precise tolerance
            if self.current_time_index_ >= len(self.path_points_) - 1:
                goal_distance = math.sqrt((self.path_points_[-1, 0] - self.current_x_)**2 + 
                                        (self.path_points_[-1, 1] - self.current_y_)**2)
                if (goal_distance < self.precise_goal_tolerance_ and 
                    abs(self.current_linear_vel_) < self.velocity_tolerance_):
                    self.get_logger().info(f"Precise goal reached! Final distance: {goal_distance:.3f}m")
                    self.path_updated_ = False

def main(args=None):
    rclpy.init(args=args)
    node = DiabloTrackNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, stopping robot...")
        node._pub_cmd(0.0, 0.0)
        time.sleep(0.1)
        node.get_logger().info("Robot stopped, shutting down.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception in spin: {e}", exc_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
