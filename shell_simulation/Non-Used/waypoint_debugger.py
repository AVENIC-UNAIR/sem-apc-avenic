#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import pickle
import os
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Point

class WaypointDebugger(Node):
    def __init__(self):
        super().__init__('waypoint_debugger')
        
        # Competition waypoints
        self.competition_waypoints = [
            [334.949799, -161.106171, 0.001736],
            [339.100037, -258.568939, 0.001679],
            [396.295319, -183.195740, 0.001678],
            [267.657074, -1.983160, 0.001678],
            [153.868896, -26.115866, 0.001678],
            [290.515564, -56.175072, 0.001677],
            [92.325722, -86.063644, 0.001677],
            [88.384346, -287.468567, 0.001728],
            [177.594101, -326.386902, 0.001677],
            [-1.646942, -197.501282, 0.001555],
            [59.701321, -1.970804, 0.001467],
            [122.100121, -55.142044, 0.001596],
            [161.030975, -129.313187, 0.001679],
            [184.758713, -199.424271, 0.001680]
        ]
        
        # Vehicle position
        self.current_position = None
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/carla/ego_vehicle/odometry', 
            self.odom_callback, 
            10)
        
        # Timer for calculating distances
        self.timer = self.create_timer(1.0, self.calculate_distances)
        
        self.get_logger().info("Waypoint Debugger started")
    
    def odom_callback(self, msg):
        """Store current position"""
        if not self.current_position:
            self.current_position = Point()
            
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
    
    def calculate_distance(self, target):
        """Calculate distance to target"""
        return math.sqrt(
            (self.current_position.x - target[0])**2 + 
            (self.current_position.y - target[1])**2)
    
    def calculate_distances(self):
        """Calculate and log distances to all waypoints"""
        if not self.current_position:
            self.get_logger().info("Waiting for position data...")
            return
            
        self.get_logger().info(f"Current position: ({self.current_position.x:.2f}, {self.current_position.y:.2f}, {self.current_position.z:.2f})")
        
        for i, waypoint in enumerate(self.competition_waypoints):
            distance = self.calculate_distance(waypoint)
            self.get_logger().info(f"Waypoint {i+1}: distance = {distance:.2f} meters")

def main(args=None):
    rclpy.init(args=args)
    debugger = WaypointDebugger()
    rclpy.spin(debugger)
    debugger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()