#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import pickle
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

class SimpleMapper(Node):
    def __init__(self):
        super().__init__('simple_mapper')
        
        # Storage for position data
        self.positions = []
        
        # Current position
        self.current_position = None
        
        # Create subscriber
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/carla/ego_vehicle/odometry', 
            self.odom_callback, 
            10)
        
        # Create timer to save data
        self.save_timer = self.create_timer(10.0, self.save_data)  # Save every 10 seconds
        
        # Create directory for data
        os.makedirs('map_data', exist_ok=True)
        
        self.get_logger().info("Simple Mapper started. Drive around to record positions.")
    
    def odom_callback(self, msg):
        # Store current position
        if not self.current_position:
            self.current_position = Point()
        
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
        
        # Record position (only if we've moved at least 1 meter)
        if not self.positions or self.calculate_distance(self.positions[-1]) > 1.0:
            self.positions.append([
                self.current_position.x,
                self.current_position.y,
                self.current_position.z
            ])
            self.get_logger().info(f"Recorded position: {len(self.positions)} points so far")
    
    def calculate_distance(self, point):
        """Calculate distance between current position and a point"""
        return math.sqrt(
            (self.current_position.x - point[0])**2 + 
            (self.current_position.y - point[1])**2)
    
    def save_data(self):
        """Save the recorded positions to file"""
        if not self.positions:
            self.get_logger().info("No position data to save yet")
            return
        
        try:
            with open('map_data/positions.pkl', 'wb') as f:
                pickle.dump(self.positions, f)
            
            self.get_logger().info(f"Saved {len(self.positions)} positions to positions.pkl")
        except Exception as e:
            self.get_logger().error(f"Error saving data: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    mapper = SimpleMapper()
    rclpy.spin(mapper)
    mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
