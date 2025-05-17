#!/usr/bin/env python3

"""
Waypoint Manager Node untuk menghasilkan waypoint jalur lurus
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point
from nav_msgs.msg import Odometry
import math
import numpy as np

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        # Declare parameters
        self.declare_parameter('waypoint_spacing', 10.0)
        self.declare_parameter('lookahead_distance', 15.0)
        
        # Get parameters
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        
        # Publishers
        self.waypoints_pub = self.create_publisher(
            PoseArray,
            '/waypoints/simplified',
            10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odometry_callback,
            10
        )
        
        # Variables
        self.current_position = None
        self.waypoints = []
        self.has_generated_waypoints = False
        
        # Timer
        self.timer = self.create_timer(1.0, self.publish_waypoints)
        
        self.get_logger().info('Waypoint Manager initialized')
    
    def odometry_callback(self, msg):
        """Process vehicle odometry data"""
        self.current_position = msg.pose.pose.position
        
        # Generate waypoints if not done yet
        if not self.has_generated_waypoints and self.current_position is not None:
            self.generate_straight_line_waypoints()
            self.has_generated_waypoints = True
    
    def generate_straight_line_waypoints(self):
        """Generate waypoints in a straight line ahead of the vehicle"""
        if self.current_position is None:
            return
        
        # Get current position and orientation
        x = self.current_position.x
        y = self.current_position.y
        z = self.current_position.z
        
        # Generate waypoints in a straight line (100m ahead)
        num_waypoints = int(100.0 / self.waypoint_spacing)
        self.waypoints = []
        
        for i in range(1, num_waypoints + 1):
            pose = Pose()
            pose.position.x = x + i * self.waypoint_spacing
            pose.position.y = y  # Keep y constant for straight line
            pose.position.z = z
            pose.orientation.w = 1.0  # Default quaternion
            
            self.waypoints.append(pose)
        
        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints in a straight line')
    
    def publish_waypoints(self):
        """Publish waypoints for other nodes to use"""
        if not self.waypoints:
            return
        
        # Create PoseArray message
        waypoints_msg = PoseArray()
        waypoints_msg.header.frame_id = "map"
        waypoints_msg.header.stamp = self.get_clock().now().to_msg()
        waypoints_msg.poses = self.waypoints
        
        # Publish waypoints
        self.waypoints_pub.publish(waypoints_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 