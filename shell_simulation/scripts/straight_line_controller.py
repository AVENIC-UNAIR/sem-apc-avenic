#!/usr/bin/env python3

"""
Straight Line Controller Node untuk mengendalikan kendaraan bergerak lurus
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
import math
import numpy as np

class StraightLineController(Node):
    def __init__(self):
        super().__init__('straight_line_controller')
        
        # Declare parameters
        self.declare_parameter('target_speed', 10.0)
        self.declare_parameter('waypoint_tolerance', 3.0)
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
        
        # Get parameters
        self.target_speed = self.get_parameter('target_speed').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        
        # Publishers
        self.throttle_pub = self.create_publisher(Float64, '/throttle_command', 10)
        self.brake_pub = self.create_publisher(Float64, '/brake_command', 10)
        self.raw_steering_pub = self.create_publisher(Float64, '/control/raw_steering', 10)
        self.gear_pub = self.create_publisher(String, '/gear_command', 10)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vehicle/filtered_pose',
            self.pose_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odometry_callback,
            10
        )
        
        self.waypoints_sub = self.create_subscription(
            PoseArray,
            '/waypoints/simplified',
            self.waypoints_callback,
            10
        )
        
        # Variables
        self.current_pose = None
        self.current_speed = 0.0
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.initial_heading = None
        self.straight_line_counter = 0  # Counter for consecutive frames with small lateral error
        
        # PID controller variables
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.last_time = None
        
        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Set gear to forward
        self.set_gear_forward()
        
        self.get_logger().info('Straight Line Controller initialized')
    
    def set_gear_forward(self):
        """Set vehicle gear to forward"""
        gear_msg = String()
        gear_msg.data = "forward"
        self.gear_pub.publish(gear_msg)
        self.get_logger().info('Set gear to forward')
    
    def pose_callback(self, msg):
        """Process filtered vehicle pose data"""
        self.current_pose = msg.pose
        
        # Store initial heading if not set
        if self.initial_heading is None and self.current_pose is not None:
            # Extract yaw from quaternion - simplified for this example
            # In a real implementation, you'd use proper quaternion to euler conversion
            self.initial_heading = 0.0  # Assume initial heading is straight ahead
            self.get_logger().info(f'Initial heading set to {self.initial_heading}')
    
    def odometry_callback(self, msg):
        """Process vehicle odometry data for speed"""
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx**2 + vy**2)
    
    def waypoints_callback(self, msg):
        """Process waypoints data"""
        self.waypoints = msg.poses
        self.get_logger().info(f'Received {len(self.waypoints)} waypoints')
    
    def control_loop(self):
        """Main control loop for vehicle control"""
        if self.current_pose is None or not self.waypoints:
            return
        
        # Get current target waypoint
        if self.current_waypoint_idx >= len(self.waypoints):
            self.stop_vehicle()
            return
        
        target_waypoint = self.waypoints[self.current_waypoint_idx]
        
        # Calculate distance to target
        dx = target_waypoint.position.x - self.current_pose.position.x
        dy = target_waypoint.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if waypoint reached
        if distance < self.waypoint_tolerance:
            self.get_logger().info(f'Waypoint {self.current_waypoint_idx} reached!')
            self.current_waypoint_idx += 1
            return
        
        # Calculate lateral error (y-axis deviation) for straight line
        lateral_error = dy
        
        # Check if we're close to the straight line
        if abs(lateral_error) < 0.1:  # Very small lateral error
            self.straight_line_counter += 1
            
            # If we've been straight for a while, force steering to exactly zero
            if self.straight_line_counter > 10:  # 10 consecutive frames with small error
                self.get_logger().info('Vehicle is straight - forcing zero steering')
                raw_steering_msg = Float64()
                raw_steering_msg.data = 0.0
                self.raw_steering_pub.publish(raw_steering_msg)
                
                # Reset integral error to prevent buildup
                self.integral_error = 0.0
                
                # Apply throttle for constant speed
                throttle = 0.3  # Constant throttle when straight
                throttle_msg = Float64()
                throttle_msg.data = throttle
                self.throttle_pub.publish(throttle_msg)
                
                return
        else:
            # Reset counter if we're not straight
            self.straight_line_counter = 0
        
        # PID control for steering
        now = self.get_clock().now()
        
        if self.last_time is None:
            self.last_time = now
            dt = 0.05  # Default dt
        else:
            dt = (now - self.last_time).nanoseconds / 1e9
            self.last_time = now
        
        # Avoid division by zero
        if dt < 0.001:
            dt = 0.001
        
        # PID calculation
        proportional = self.kp * lateral_error
        self.integral_error += lateral_error * dt
        
        # Limit integral term to prevent windup
        self.integral_error = max(-1.0, min(1.0, self.integral_error))
        
        derivative = self.kd * (lateral_error - self.prev_error) / dt
        
        # Calculate raw steering - negative for right, positive for left
        raw_steering = -(proportional + self.ki * self.integral_error + derivative)
        
        # Limit steering to valid range
        raw_steering = max(-1.0, min(1.0, raw_steering))
        
        # Publish raw steering for stability filter
        raw_steering_msg = Float64()
        raw_steering_msg.data = raw_steering
        self.raw_steering_pub.publish(raw_steering_msg)
        
        # Speed control (P controller)
        speed_error = self.target_speed - self.current_speed
        throttle = 0.5 * speed_error
        throttle = max(0.0, min(1.0, throttle))
        
        # Apply throttle directly (steering will be handled by stability filter)
        throttle_msg = Float64()
        throttle_msg.data = throttle
        self.throttle_pub.publish(throttle_msg)
        
        # Update previous error
        self.prev_error = lateral_error
    
    def stop_vehicle(self):
        """Stop the vehicle"""
        # Apply brake
        brake_msg = Float64()
        brake_msg.data = 1.0
        self.brake_pub.publish(brake_msg)
        
        # Zero throttle
        throttle_msg = Float64()
        throttle_msg.data = 0.0
        self.throttle_pub.publish(throttle_msg)
        
        # Center steering
        raw_steering_msg = Float64()
        raw_steering_msg.data = 0.0
        self.raw_steering_pub.publish(raw_steering_msg)
        
        self.get_logger().info('All waypoints reached. Vehicle stopped.')

def main(args=None):
    rclpy.init(args=args)
    controller = StraightLineController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 