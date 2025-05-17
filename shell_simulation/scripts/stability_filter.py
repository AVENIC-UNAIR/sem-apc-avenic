#!/usr/bin/env python3

"""
Stability Filter Node untuk menstabilkan kontrol steering
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class StabilityFilter(Node):
    def __init__(self):
        super().__init__('stability_filter')
        
        # Declare parameters
        self.declare_parameter('deadband_threshold', 0.05)
        self.declare_parameter('lowpass_alpha', 0.2)
        self.declare_parameter('max_steering_change', 0.1)
        
        # Get parameters
        self.deadband_threshold = self.get_parameter('deadband_threshold').value
        self.lowpass_alpha = self.get_parameter('lowpass_alpha').value
        self.max_steering_change = self.get_parameter('max_steering_change').value
        
        # Publishers
        self.steering_pub = self.create_publisher(
            Float64,
            '/steering_command',
            10
        )
        
        # Subscribers
        self.raw_steering_sub = self.create_subscription(
            Float64,
            '/control/raw_steering',
            self.raw_steering_callback,
            10
        )
        
        # Variables
        self.prev_steering = 0.0
        self.filtered_steering = 0.0
        self.steering_history = [0.0] * 5  # Keep last 5 steering values for outlier detection
        
        self.get_logger().info('Stability Filter initialized')
        self.get_logger().info(f'Parameters: deadband={self.deadband_threshold}, ' +
                             f'lowpass={self.lowpass_alpha}, ' +
                             f'max_change={self.max_steering_change}')
    
    def raw_steering_callback(self, msg):
        """Process raw steering commands and apply filters"""
        raw_steering = msg.data
        
        # Detect and reject outliers
        if self.is_outlier(raw_steering):
            self.get_logger().debug(f'Outlier detected: {raw_steering:.3f}, ignoring')
            return
        
        # Apply deadband filter
        steering_after_deadband = self.apply_deadband(raw_steering)
        
        # Apply rate limiting
        steering_after_rate_limiting = self.limit_steering_change(steering_after_deadband)
        
        # Apply low-pass filter
        self.filtered_steering = self.apply_lowpass(steering_after_rate_limiting)
        
        # Update steering history
        self.steering_history.pop(0)
        self.steering_history.append(self.filtered_steering)
        
        # Publish filtered steering
        self.publish_filtered_steering()
    
    def is_outlier(self, steering):
        """Detect if a steering value is an outlier"""
        # If steering is very different from the average of recent values
        if len(self.steering_history) < 3:
            return False
        
        avg_steering = sum(self.steering_history) / len(self.steering_history)
        return abs(steering - avg_steering) > 0.5  # Threshold for outlier detection
    
    def apply_deadband(self, steering):
        """Apply deadband filter to ignore small steering values"""
        if abs(steering) < self.deadband_threshold:
            return 0.0
        else:
            # Preserve sign but subtract threshold to avoid jumps
            sign = 1.0 if steering > 0 else -1.0
            return sign * (abs(steering) - self.deadband_threshold)
    
    def limit_steering_change(self, desired_steering):
        """Limit the rate of change of steering"""
        change = desired_steering - self.prev_steering
        
        if abs(change) > self.max_steering_change:
            change = self.max_steering_change if change > 0 else -self.max_steering_change
        
        return self.prev_steering + change
    
    def apply_lowpass(self, steering):
        """Apply low-pass filter to smooth steering changes"""
        return self.lowpass_alpha * steering + (1 - self.lowpass_alpha) * self.prev_steering
    
    def publish_filtered_steering(self):
        """Publish the filtered steering command"""
        # Update previous steering
        self.prev_steering = self.filtered_steering
        
        # Ensure steering is within valid range
        self.filtered_steering = max(-1.0, min(1.0, self.filtered_steering))
        
        # Add extra dampening for near-zero values to help maintain straight line
        if abs(self.filtered_steering) < 0.2:
            self.filtered_steering *= 0.8  # Reduce small steering commands by 20%
        
        # Publish steering command
        steering_msg = Float64()
        steering_msg.data = self.filtered_steering
        self.steering_pub.publish(steering_msg)
        
        # Log significant steering changes
        if abs(self.filtered_steering) > 0.3:
            self.get_logger().debug(f'Significant steering: {self.filtered_steering:.3f}')

def main(args=None):
    rclpy.init(args=args)
    filter_node = StabilityFilter()
    rclpy.spin(filter_node)
    filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 