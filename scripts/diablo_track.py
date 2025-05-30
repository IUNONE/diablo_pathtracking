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

        assert self.strategy_ in ['pd'], f"Invalid strategy: {self.strategy_}"

        # Path tracking variables
        self.current_path_ = None
        self.path_points_ = None
        self.path_updated_ = False
        self.path_start_time_ = None
        self.target_index_ = 0
        self.path_lock_ = Lock()
        
        # Error tracking for PD control
        self.prev_linear_error_ = 0.0
        self.prev_angular_error_ = 0.0
        self.prev_time_ = None

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
            self.target_index_ = 0
            
            # Reset PD error tracking
            self.prev_linear_error_ = 0.0
            self.prev_angular_error_ = 0.0
            self.prev_time_ = None
            
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

    def _find_target_point(self):
        """Find target point on path using lookahead distance"""
        if self.path_points_ is None or len(self.path_points_) == 0:
            return None
            
        # Find closest point on path
        distances = np.sqrt((self.path_points_[:, 0] - self.current_x_)**2 + 
                           (self.path_points_[:, 1] - self.current_y_)**2)
        closest_idx = np.argmin(distances)
        
        # Look for target point at lookahead distance
        for i in range(closest_idx, len(self.path_points_)):
            dist = np.sqrt((self.path_points_[i, 0] - self.current_x_)**2 + 
                          (self.path_points_[i, 1] - self.current_y_)**2)
            if dist >= self.lookahead_distance_:
                self.target_index_ = i
                return self.path_points_[i]
        
        # If no point at lookahead distance, use the last point
        self.target_index_ = len(self.path_points_) - 1
        return self.path_points_[-1]

    def _calculate_control_commands(self, target_point):
        """Calculate linear and angular velocities using PD control"""
        if target_point is None:
            return 0.0, 0.0
            
        # Calculate position error
        dx = target_point[0] - self.current_x_
        dy = target_point[1] - self.current_y_
        distance_error = math.sqrt(dx**2 + dy**2)
        
        # Calculate angle error
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.current_yaw_
        
        # Normalize angle error to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
            
        # PD control
        current_time = self.get_clock().now()
        if self.prev_time_ is not None:
            dt = (current_time - self.prev_time_).nanoseconds * 1e-9
            if dt > 0:
                # Linear velocity PD control
                linear_error_rate = (distance_error - self.prev_linear_error_) / dt
                linear_vel = self.kp_linear_ * distance_error + self.kd_linear_ * linear_error_rate
                
                # Angular velocity PD control
                angular_error_rate = (angle_error - self.prev_angular_error_) / dt
                angular_vel = self.kp_angular_ * angle_error + self.kd_angular_ * angular_error_rate
                
                # Update previous values
                self.prev_linear_error_ = distance_error
                self.prev_angular_error_ = angle_error
            else:
                linear_vel = self.kp_linear_ * distance_error
                angular_vel = self.kp_angular_ * angle_error
        else:
            linear_vel = self.kp_linear_ * distance_error
            angular_vel = self.kp_angular_ * angle_error
            self.prev_linear_error_ = distance_error
            self.prev_angular_error_ = angle_error
            
        self.prev_time_ = current_time
        
        # Apply velocity limits
        linear_vel = np.clip(linear_vel, -self.max_linear_vel_, self.max_linear_vel_)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel_, self.max_angular_vel_)
        
        # Stop if close to goal
        if distance_error < self.goal_tolerance_:
            linear_vel = 0.0
            angular_vel = 0.0
            
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
                
            # Find target point
            target_point = self._find_target_point()
            if target_point is None:
                self._pub_cmd(0.0, 0.0)
                return
                
            # Calculate control commands
            linear_vel, angular_vel = self._calculate_control_commands(target_point)
            
            # Publish command
            self._pub_cmd(linear_vel, angular_vel)
            
            # Check if reached goal
            if self.target_index_ >= len(self.path_points_) - 1:
                goal_distance = math.sqrt((self.path_points_[-1, 0] - self.current_x_)**2 + 
                                        (self.path_points_[-1, 1] - self.current_y_)**2)
                if goal_distance < self.goal_tolerance_:
                    self.get_logger().info("Goal reached!")
                    self.path_updated_ = False

def main(args=None):
    rclpy.init(args=args)
    node = DiabloTrackNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception in spin: {e}", exc_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
