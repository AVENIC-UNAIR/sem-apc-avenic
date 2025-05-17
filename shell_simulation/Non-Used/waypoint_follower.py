#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import pickle
import os
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Point

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Competition waypoints
        self.competition_waypoints = [
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
        self.first_position_received = False
        
        # Navigation state
        self.current_waypoint_idx = 0
        self.reached_waypoints = []
        self.last_log_time = 0.0
        self.navigation_started = False
        
        # Control parameters
        self.waypoint_threshold = 3.0  # meters
        self.max_throttle = 0.4
        self.steering_gain = 0.7
        
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
        
        # Wait for first position before starting navigation
        self.navigation_timer = None
        self.init_timer = self.create_timer(0.5, self.check_init)
        
        self.get_logger().info("Waypoint Follower initialized")
    
    def check_init(self):
        """Check if we have position data and then start navigation"""
        if self.first_position_received:
            self.get_logger().info("Position data received, starting navigation")
            self.init_timer.cancel()
            
            # Print initial distances to waypoints
            for i, waypoint in enumerate(self.competition_waypoints):
                distance = self.calculate_distance(waypoint)
                self.get_logger().info(f"Initial distance to waypoint {i+1}: {distance:.2f} meters")
            
            # Start navigation timer
            self.navigation_timer = self.create_timer(0.1, self.control_loop)
            self.navigation_started = True
    
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
        
        # Extract orientation using our custom function
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.current_orientation = yaw
        
        # Extract velocity
        self.current_velocity = math.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2)
        
        # Mark that we've received position data
        if not self.first_position_received:
            self.first_position_received = True
    
    def control_loop(self):
        """Main control loop"""
        if not self.current_position or self.current_orientation is None:
            self.get_logger().warn("Waiting for odometry data...")
            return
        
        # Check if we've completed all waypoints
        if self.current_waypoint_idx >= len(self.competition_waypoints):
            self.stop_vehicle()
            self.log_throttle(5.0, "All waypoints reached! Mission complete.")
            return
        
        # Get current target waypoint
        target = self.competition_waypoints[self.current_waypoint_idx]
        
        # Calculate distance to target
        distance = self.calculate_distance(target)
        
        # Check if we've reached the target
        if distance < self.waypoint_threshold:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx + 1} of {len(self.competition_waypoints)}, "
                                 f"distance: {distance:.2f}m")
            self.reached_waypoints.append(self.current_waypoint_idx)
            self.current_waypoint_idx += 1
            
            # Slow down when reaching a waypoint
            self.apply_control(throttle=0.0, brake=0.3, steering=0.0)
            
            # Wait a bit before continuing
            self.get_logger().info("Slowing down for waypoint transition...")
            return
        
        # Calculate steering to target
        steering = self.calculate_steering(target)
        
        # Calculate throttle based on steering and distance
        throttle = self.calculate_throttle(steering, distance)
        
        # Apply control
        self.apply_control(throttle=throttle, brake=0.0, steering=steering)
        
        # Log navigation status
        self.log_throttle(1.0, 
            f"Navigating to waypoint {self.current_waypoint_idx + 1}/{len(self.competition_waypoints)}, "
            f"Distance: {distance:.1f}m, Steering: {steering:.2f}, Throttle: {throttle:.2f}")
    
    def calculate_distance(self, target):
        """Calculate distance to target"""
        return math.sqrt(
            (self.current_position.x - target[0])**2 + 
            (self.current_position.y - target[1])**2)
    
    def calculate_steering(self, target):
        """Calculate steering to target"""
        # Get target angle to target
        target_angle = math.atan2(
            target[1] - self.current_position.y,
            target[0] - self.current_position.x)
        
        # Calculate angle difference
        angle_diff = target_angle - self.current_orientation
        
        # Normalize to [-pi, pi]
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Apply steering gain and convert to [-1, 1]
        steering = self.steering_gain * angle_diff / math.pi
        
        # Limit steering value
        return max(-1.0, min(1.0, steering))
    
    def calculate_throttle(self, steering, distance):
        """Calculate throttle based on steering and distance"""
        # Base throttle
        base_throttle = self.max_throttle
        
        # Reduce throttle in sharp turns
        steering_factor = 1.0 - 0.7 * abs(steering)
        
        # Reduce throttle when approaching waypoint
        distance_factor = min(1.0, distance / 10.0)
        
        # Combine factors
        throttle = base_throttle * steering_factor * distance_factor
        
        # Ensure minimum throttle for movement
        return max(0.1, min(self.max_throttle, throttle))
    
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
        
        # Log the control commands
        self.get_logger().debug(f"Control applied: throttle={throttle:.2f}, brake={brake:.2f}, steering={steering:.2f}")
    
    def stop_vehicle(self):
        """Stop the vehicle"""
        self.apply_control(throttle=0.0, brake=1.0, steering=0.0)

def main(args=None):
    rclpy.init(args=args)
    follower = WaypointFollower()
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()