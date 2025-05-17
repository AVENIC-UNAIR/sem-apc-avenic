#!/usr/bin/env python3

"""
Path navigator using TSP to find optimal path between waypoints
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
import math
import numpy as np

class PathNavigator(Node):
    def __init__(self):
        super().__init__('path_navigator')
        
        # Publisher for the planned path
        self.path_publisher = self.create_publisher(PoseArray, 'planned_path', 10)
        
        # Subscriber untuk mendapatkan posisi dari odometry Carla
        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odometry_callback,
            10)
        
        # Initialize variables
        self.current_pose = None
        self.waypoints = []
        
        # Buat waypoint dari data README
        self.create_waypoints()
        
        # Timer untuk mempublikasikan rute setiap 1 detik
        self.timer = self.create_timer(1.0, self.publish_path)
        
        self.get_logger().info('Path Navigator initialized')
    
    def create_waypoints(self):
        """Create waypoints from the waypoint data in README"""
        waypoint_data = [
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
        
        for wp_data in waypoint_data:
            pose = Pose()
            pose.position.x = wp_data[0]
            pose.position.y = wp_data[1]
            pose.position.z = wp_data[2]
            pose.orientation.w = 1.0  # Default orientation (no rotation)
            self.waypoints.append(pose)
        
        self.get_logger().info(f'Created {len(self.waypoints)} waypoints')
    
    def odometry_callback(self, msg):
        """Process vehicle odometry"""
        self.current_pose = msg.pose.pose
    
    def calculate_path(self):
        """Calculate optimal path using greedy nearest neighbor TSP"""
        if not self.current_pose or not self.waypoints:
            return []
        
        # Start from current position
        current_position = self.current_pose.position
        
        # Make a copy of waypoints to modify
        remaining_waypoints = self.waypoints.copy()
        ordered_waypoints = []
        
        # Loop until all waypoints are visited
        while remaining_waypoints:
            # Find closest waypoint
            closest_idx = -1
            min_distance = float('inf')
            
            for i, waypoint in enumerate(remaining_waypoints):
                distance = self.calculate_distance(current_position, waypoint.position)
                if distance < min_distance:
                    min_distance = distance
                    closest_idx = i
            
            # Add closest waypoint to ordered list
            if closest_idx != -1:
                closest_waypoint = remaining_waypoints.pop(closest_idx)
                ordered_waypoints.append(closest_waypoint)
                current_position = closest_waypoint.position
        
        return ordered_waypoints
    
    def publish_path(self):
        """Calculate and publish the optimal path"""
        # Only publish if we have current position
        if not self.current_pose:
            self.get_logger().warn('No current position available, waiting...')
            return
        
        # Calculate path
        path = self.calculate_path()
        
        if not path:
            self.get_logger().warn('No path calculated, cannot publish')
            return
        
        # Create PoseArray message
        path_msg = PoseArray()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        path_msg.poses = path
        
        # Publish the path
        self.path_publisher.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path)} waypoints')
    
    def calculate_distance(self, point1, point2):
        """Calculate Euclidean distance between two points"""
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        dz = point1.z - point2.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

def main(args=None):
    rclpy.init(args=args)
    node = PathNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()