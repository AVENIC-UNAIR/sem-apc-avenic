#!/usr/bin/env python3

"""
Vehicle controller using Pure Pursuit algorithm adapted from CARLA example
- Uses the same core logic from the provided CARLA code
- Integrated with ROS2 framework and sensor data
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Float32
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
import math
import numpy as np
import time

class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        
        # Publishers for vehicle control
        self.brake_publisher = self.create_publisher(Float64, 'brake_command', 1)
        self.gear_publisher = self.create_publisher(String, 'gear_command', 1)
        self.steering_publisher = self.create_publisher(Float64, 'steering_command', 1)
        self.throttle_publisher = self.create_publisher(Float64, 'throttle_command', 1)
        
        # Subscribers
        self.path_subscription = self.create_subscription(
            PoseArray, 'planned_path', self.path_callback, 10)
            
        self.odometry_subscription = self.create_subscription(
            Odometry, '/carla/ego_vehicle/odometry', self.odometry_callback, 10)
            
        self.speedometer_subscription = self.create_subscription(
            Float32, '/carla/ego_vehicle/speedometer', self.speedometer_callback, 10)
        
        # Timer for control loop (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # State variables
        self.path = []
        self.current_pose = None
        self.current_speed = 0.0
        self.waypoint_list = []  # (x, y) coordinates of waypoints
        self.min_index = 0
        
        # Pure Pursuit parameters from the example code
        self.L = 2.875  # Wheelbase length (m)
        self.Kdd = 3.0  # Lookahead distance gain
        self.alpha_prev = 0  # Previous alpha angle
        self.delta_prev = 0  # Previous steering angle
        self.lookahead_dist = 5.0  # Default lookahead distance
        
        # Control parameters
        self.base_throttle = 0.4  # Base throttle value
        self.control_initialized = False
        
        self.get_logger().info('Pure Pursuit Controller initialized')
    
    def path_callback(self, msg):
        """Process new path"""
        if len(msg.poses) > 0:
            self.get_logger().info(f'Received new path with {len(msg.poses)} waypoints')
            self.path = msg.poses
            
            # Convert path to waypoint list format used in the example code
            self.waypoint_list = []
            for wp in self.path:
                self.waypoint_list.append((wp.position.x, wp.position.y))
            
            self.min_index = 0
            self.control_initialized = True
    
    def odometry_callback(self, msg):
        """Process vehicle odometry"""
        self.current_pose = msg.pose.pose
    
    def speedometer_callback(self, msg):
        """Process speedometer data"""
        self.current_speed = msg.data
    
    def control_loop(self):
        """Main control loop implementing Pure Pursuit algorithm"""
        if not self.control_initialized or not self.current_pose or not self.waypoint_list:
            return
        
        # Extract vehicle position and orientation
        veh_location = self.current_pose.position
        
        # Extract vehicle yaw from quaternion
        x = self.current_pose.orientation.x
        y = self.current_pose.orientation.y
        z = self.current_pose.orientation.z
        w = self.current_pose.orientation.w
        
        # Calculate yaw (heading) from quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Get forward velocity (speed)
        vf = self.current_speed
        vf = np.fmax(np.fmin(vf, 15.0), 0.1)  # Limit speed range
        
        # Get target waypoint index
        min_index, tx, ty, dist = self.get_target_wp_index(veh_location)
        
        # Calculate lookahead distance based on speed
        ld = self.get_lookahead_dist(vf)
        
        # Calculate alpha angle (angle between vehicle heading and target point)
        alpha = math.atan2(ty - veh_location.y, tx - veh_location.x) - yaw
        
        # Handle NaN values
        if math.isnan(alpha):
            alpha = self.alpha_prev
        else:
            self.alpha_prev = alpha
        
        # Calculate cross-track error
        e = np.sin(alpha) * ld
        
        # Calculate steering angle using Pure Pursuit
        steer_angle = self.calc_steering_angle(alpha, ld)
        
        # Calculate throttle based on conditions
        throttle = self.calculate_throttle(steer_angle, vf)
        
        # Apply controls
        self.apply_controls(throttle, steer_angle)
        
        # Log information occasionally
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
            if self.log_counter % 20 == 0:  # Log every second
                self.get_logger().info(
                    f'Control: speed={vf:.2f} m/s, lookahead={ld:.2f} m, ' +
                    f'alpha={alpha:.4f} rad, steer={steer_angle:.4f}, error={e:.2f} m'
                )
        else:
            self.log_counter = 0
    
    def get_target_wp_index(self, veh_location):
        """Find target waypoint index based on vehicle location"""
        dxl, dyl = [], []
        
        for i in range(len(self.waypoint_list)):
            dx = abs(veh_location.x - self.waypoint_list[i][0])
            dxl.append(dx)
            dy = abs(veh_location.y - self.waypoint_list[i][1])
            dyl.append(dy)
        
        dist = np.hypot(dxl, dyl)
        idx = np.argmin(dist) + 4  # Look a few points ahead for stability
        
        # Take closest waypoint, else last wp
        if idx < len(self.waypoint_list):
            tx = self.waypoint_list[idx][0]
            ty = self.waypoint_list[idx][1]
        else:
            tx = self.waypoint_list[-1][0]
            ty = self.waypoint_list[-1][1]
        
        return idx, tx, ty, dist
    
    def get_lookahead_dist(self, vf):
        """Calculate lookahead distance based on speed"""
        # Dynamic lookahead distance based on vehicle speed
        ld = self.Kdd * vf
        
        # Ensure minimum lookahead distance for stability
        return max(3.0, ld)
    
    def calc_steering_angle(self, alpha, ld):
        """Calculate steering angle using Pure Pursuit formula"""
        # Pure Pursuit controller formula
        delta = math.atan2(2 * self.L * np.sin(alpha), ld)
        
        # Limit steering angle
        delta = np.fmax(np.fmin(delta, 1.0), -1.0)
        
        # Handle NaN values
        if math.isnan(delta):
            delta = self.delta_prev
        else:
            self.delta_prev = delta
        
        return delta
    
    def calculate_throttle(self, steering_angle, current_speed):
        """Calculate appropriate throttle based on steering and speed"""
        # Base throttle
        throttle = self.base_throttle
        
        # Reduce throttle when turning sharply
        steering_abs = abs(steering_angle)
        if steering_abs > 0.7:
            throttle *= 0.7  # Significant reduction for sharp turns
        elif steering_abs > 0.4:
            throttle *= 0.85  # Moderate reduction for medium turns
        
        # Reduce throttle at higher speeds for safety
        if current_speed > 10.0:
            throttle *= 0.9
        
        # Limit maximum throttle
        throttle = min(0.6, throttle)
        
        return throttle
    
    def apply_controls(self, throttle, steering):
        """Apply calculated controls to the vehicle"""
        # Create control messages
        brake_msg = Float64()
        gear_msg = String()
        steering_msg = Float64()
        throttle_msg = Float64()
        
        # Set control values
        brake_msg.data = 0.0
        gear_msg.data = "forward"
        steering_msg.data = steering
        throttle_msg.data = throttle
        
        # Publish control messages
        self.brake_publisher.publish(brake_msg)
        self.gear_publisher.publish(gear_msg)
        self.steering_publisher.publish(steering_msg)
        self.throttle_publisher.publish(throttle_msg)
    
    def stop(self):
        """Stop the vehicle"""
        brake_msg = Float64()
        throttle_msg = Float64()
        
        brake_msg.data = 1.0  # Full brake
        throttle_msg.data = 0.0
        
        self.brake_publisher.publish(brake_msg)
        self.throttle_publisher.publish(throttle_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()