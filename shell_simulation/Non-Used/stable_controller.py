#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2

class StablePurePursuitController(Node):
    def __init__(self):
        super().__init__('stable_controller')
        
        # Competition waypoints
        self.waypoints = [
            [334.949799, -161.106171, 0.001736],  # 63.17m from start
            [339.100037, -258.568939, 0.001679],  # 141.98m from start
            [396.295319, -183.195740, 0.001678],  # 127.84m from start
            [267.657074, -1.983160, 0.001678],    # 127.96m from start
            [153.868896, -26.115866, 0.001678],   # 163.25m from start
            [290.515564, -56.175072, 0.001677],   # 73.83m from start
            [92.325722, -86.063644, 0.001677],    # 192.95m from start
            [88.384346, -287.468567, 0.001728],   # 248.74m from start
            [177.594101, -326.386902, 0.001677],  # 222.27m from start
            [-1.646942, -197.501282, 0.001555],   # 290.14m from start
            [59.701321, -1.970804, 0.001467],     # 254.77m from start
            [122.100121, -55.142044, 0.001596],   # 174.78m from start
            [161.030975, -129.313187, 0.001679],  # 119.33m from start
            [184.758713, -199.424271, 0.001680]   # 118.56m from start
        ]
        
        # Vehicle state
        self.current_position = None
        self.current_orientation = None
        self.current_velocity = 0.0
        self.obstacle_detected = False
        
        # Navigation state
        self.current_waypoint_idx = 0
        self.last_log_time = 0.0
        
        # Pure Pursuit parameters
        self.lookahead_distance = 10.0  # meters (increased for more stable path)
        self.waypoint_threshold = 5.0  # meters
        self.max_throttle = 0.3  # reduced for better control
        self.min_throttle = 0.1
        
        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/carla/ego_vehicle/odometry', 
            self.odom_callback, 
            10)
        
        # Create publishers
        self.throttle_pub = self.create_publisher(Float64, 'throttle_command', 10)
        self.brake_pub = self.create_publisher(Float64, 'brake_command', 10)
        self.steering_pub = self.create_publisher(Float64, 'steering_command', 10)
        self.gear_pub = self.create_publisher(String, 'gear_command', 10)
        
        # Set initial gear
        gear_msg = String()
        gear_msg.data = 'forward'
        self.gear_pub.publish(gear_msg)
        
        # Create timer for control
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Stable Pure Pursuit Controller initialized")
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def log_throttle(self, period, msg):
        """Log message at throttled rate"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_log_time > period:
            self.get_logger().info(msg)
            self.last_log_time = current_time
    
    def odom_callback(self, msg):
        """Update vehicle state from odometry"""
        if not self.current_position:
            self.current_position = Point()
        
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
        
        # Extract orientation
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.current_orientation = yaw
        
        # Extract velocity
        self.current_velocity = math.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2)
    
    def control_loop(self):
        """Main control loop using Pure Pursuit algorithm"""
        if not self.current_position or self.current_orientation is None:
            return
        
        # Check if we've completed all waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            self.stop_vehicle()
            self.log_throttle(5.0, "All waypoints reached! Mission complete.")
            return
        
        # Get current target waypoint
        target_waypoint = self.waypoints[self.current_waypoint_idx]
        
        # Calculate distance to waypoint
        distance_to_waypoint = self.calculate_distance(target_waypoint)
        
        # Check if we've reached the waypoint
        if distance_to_waypoint < self.waypoint_threshold:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx + 1} of {len(self.waypoints)}")
            self.current_waypoint_idx += 1
            
            # Slow down briefly
            self.apply_control(throttle=0.0, brake=0.3, steering=0.0)
            return
        
        # Find lookahead point using Pure Pursuit
        lookahead_point = self.find_lookahead_point()
        
        # Calculate steering using Pure Pursuit
        steering = self.calculate_pure_pursuit_steering(lookahead_point)
        
        # Calculate throttle based on steering angle
        throttle = self.max_throttle * (1.0 - 0.5 * abs(steering))
        throttle = max(self.min_throttle, min(self.max_throttle, throttle))
        
        # Apply control
        self.apply_control(throttle=throttle, brake=0.0, steering=steering)
        
        # Log status
        self.log_throttle(1.0, 
                         f"Navigating to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}, "
                         f"Distance: {distance_to_waypoint:.1f}m, Steering: {steering:.2f}, "
                         f"Throttle: {throttle:.2f}")
    
    def calculate_distance(self, target):
        """Calculate distance to target"""
        return math.sqrt(
            (self.current_position.x - target[0])**2 + 
            (self.current_position.y - target[1])**2)
    
    def find_lookahead_point(self):
        """Find lookahead point for Pure Pursuit algorithm"""
        if self.current_waypoint_idx >= len(self.waypoints):
            return self.waypoints[-1]
        
        # Start with current target waypoint
        target_waypoint = self.waypoints[self.current_waypoint_idx]
        
        # If we're far from the waypoint, use it directly
        distance = self.calculate_distance(target_waypoint)
        if distance > self.lookahead_distance:
            return target_waypoint
        
        # Otherwise, interpolate between current and next waypoint
        if self.current_waypoint_idx < len(self.waypoints) - 1:
            next_waypoint = self.waypoints[self.current_waypoint_idx + 1]
            
            # Calculate direction vector from current to next waypoint
            dir_x = next_waypoint[0] - target_waypoint[0]
            dir_y = next_waypoint[1] - target_waypoint[1]
            dir_len = math.sqrt(dir_x**2 + dir_y**2)
            
            if dir_len > 0:
                # Normalize direction vector
                dir_x /= dir_len
                dir_y /= dir_len
                
                # Calculate distance needed to reach lookahead distance
                distance_needed = self.lookahead_distance - distance
                
                # Calculate lookahead point
                lookahead_x = target_waypoint[0] + dir_x * distance_needed
                lookahead_y = target_waypoint[1] + dir_y * distance_needed
                
                return [lookahead_x, lookahead_y, target_waypoint[2]]
        
        return target_waypoint
    
    def calculate_pure_pursuit_steering(self, lookahead_point):
        """Calculate steering angle using Pure Pursuit algorithm"""
        # Transform lookahead point to vehicle coordinates
        dx = lookahead_point[0] - self.current_position.x
        dy = lookahead_point[1] - self.current_position.y
        
        # Rotate to vehicle reference frame
        cos_yaw = math.cos(-self.current_orientation)
        sin_yaw = math.sin(-self.current_orientation)
        
        x_local = dx * cos_yaw - dy * sin_yaw
        y_local = dx * sin_yaw + dy * cos_yaw
        
        # Prevent division by zero
        if abs(x_local) < 0.001:
            x_local = 0.001
        
        # Calculate curvature (1/R)
        curvature = 2.0 * y_local / (x_local**2 + y_local**2)
        
        # Convert curvature to steering angle [-1, 1]
        # This factor can be tuned based on the vehicle's steering characteristics
        steering = 0.5 * curvature  
        
        # Limit steering value
        return max(-1.0, min(1.0, steering))
    
    def apply_control(self, throttle, brake, steering):
        """Apply control to vehicle"""
        throttle_msg = Float64()
        brake_msg = Float64()
        steering_msg = Float64()
        
        throttle_msg.data = max(0.0, min(1.0, throttle))
        brake_msg.data = max(0.0, min(1.0, brake))
        steering_msg.data = max(-1.0, min(1.0, steering))
        
        self.throttle_pub.publish(throttle_msg)
        self.brake_pub.publish(brake_msg)
        self.steering_pub.publish(steering_msg)
    
    def stop_vehicle(self):
        """Stop the vehicle"""
        self.apply_control(throttle=0.0, brake=1.0, steering=0.0)

def main(args=None):
    rclpy.init(args=args)
    controller = StablePurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()