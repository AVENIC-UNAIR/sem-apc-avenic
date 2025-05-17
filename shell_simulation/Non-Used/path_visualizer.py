#!/usr/bin/env python3

"""
Visualizes the planned path and waypoints using RViz markers
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        
        # Publishers for visualization markers
        self.path_marker_publisher = self.create_publisher(Marker, 'path_marker', 10)
        self.waypoint_marker_publisher = self.create_publisher(MarkerArray, 'waypoint_markers', 10)
        
        # Subscribers
        self.path_subscription = self.create_subscription(
            PoseArray,
            'planned_path',
            self.path_callback,
            10)
        
        self.waypoint_subscription = self.create_subscription(
            PoseArray,
            'waypoints',
            self.waypoint_callback,
            10)
        
        self.get_logger().info('Path Visualizer initialized')
    
    def path_callback(self, msg):
        """Visualize the planned path as a line strip"""
        self.get_logger().info(f'Visualizing path with {len(msg.poses)} points')
        
        if len(msg.poses) < 2:
            self.get_logger().warn('Not enough points to visualize path')
            return
        
        # Create line strip marker for the path
        marker = Marker()
        marker.header = msg.header
        marker.ns = "planned_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set marker properties
        marker.scale.x = 0.5  # Line width
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue
        
        # Add points to the line strip
        marker.points = [pose.position for pose in msg.poses]
        
        # Set line strip to last from current time + 10 seconds
        marker.lifetime.sec = 10
        
        # Publish the marker
        self.path_marker_publisher.publish(marker)
    
    def waypoint_callback(self, msg):
        """Visualize waypoints as spheres"""
        self.get_logger().info(f'Visualizing {len(msg.poses)} waypoints')
        
        marker_array = MarkerArray()
        
        for i, pose in enumerate(msg.poses):
            # Create sphere marker for each waypoint
            marker = Marker()
            marker.header = msg.header
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set marker properties
            marker.pose = pose
            marker.scale.x = 1.0  # Sphere diameter
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
            
            # Set marker to last from current time + 10 seconds
            marker.lifetime.sec = 10
            
            marker_array.markers.append(marker)
        
        # Publish the marker array
        self.waypoint_marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()