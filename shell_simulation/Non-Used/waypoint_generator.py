#!/usr/bin/env python3

"""
Waypoint generator node that creates waypoints for the navigator
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import yaml
import os
import math
import numpy as np

class WaypointGenerator(Node):
    def __init__(self):
        super().__init__('waypoint_generator')
        
        # Publisher for waypoints
        self.waypoint_publisher = self.create_publisher(PoseArray, 'waypoints', 10)
        
        # Subscriber for current vehicle position (to publish waypoints relative to current position)
        self.position_subscription = self.create_subscription(
            Pose,
            'vehicle_position',
            self.position_callback,
            10)
        
        # Parameter for waypoint file path
        self.declare_parameter('waypoint_file', '')
        
        # Parameter for automatic waypoint generation
        self.declare_parameter('auto_generate', True)
        
        # Parameter for auto-publishing interval (in seconds)
        self.declare_parameter('publish_interval', 5.0)
        
        # Initialize variables
        self.waypoints = []
        self.current_position = None
        
        # Check parameter values
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.auto_generate = self.get_parameter('auto_generate').value
        
        # Debug parameter values
        self.get_logger().info(f'Waypoint file parameter: {self.waypoint_file}')
        self.get_logger().info(f'Auto generate parameter: {self.auto_generate}')
        
        # Try to load waypoints immediately
        self.load_waypoints()
        
        # Timer for periodic publishing
        publish_interval = self.get_parameter('publish_interval').value
        self.timer = self.create_timer(publish_interval, self.publish_waypoints)
        
        self.get_logger().info('Waypoint Generator initialized')
    
    def load_waypoints(self):
        """Load waypoints from file or generate test waypoints"""
        # Check if waypoint file is specified and exists
        if self.waypoint_file and os.path.exists(self.waypoint_file):
            # Load waypoints from YAML file
            self.get_logger().info(f'Loading waypoints from file: {self.waypoint_file}')
            try:
                with open(self.waypoint_file, 'r') as f:
                    data = yaml.safe_load(f)
                
                # Parse waypoints from YAML
                self.waypoints = []
                if data and 'waypoints' in data:
                    for wp_data in data['waypoints']:
                        pose = Pose()
                        pose.position.x = float(wp_data.get('x', 0.0))
                        pose.position.y = float(wp_data.get('y', 0.0))
                        pose.position.z = float(wp_data.get('z', 0.0))
                        
                        # Optional orientation
                        if 'orientation' in wp_data:
                            pose.orientation.x = float(wp_data['orientation'].get('x', 0.0))
                            pose.orientation.y = float(wp_data['orientation'].get('y', 0.0))
                            pose.orientation.z = float(wp_data['orientation'].get('z', 0.0))
                            pose.orientation.w = float(wp_data['orientation'].get('w', 1.0))
                        else:
                            pose.orientation.w = 1.0
                        
                        self.waypoints.append(pose)
                    
                    self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from file')
                else:
                    self.get_logger().error(f'Invalid YAML format in {self.waypoint_file}')
            except Exception as e:
                self.get_logger().error(f'Error loading waypoints from file: {e}')
                # Fall back to auto-generate
                if self.auto_generate:
                    self.generate_test_waypoints()
        
        # If no file or file loading failed, and auto_generate is enabled
        elif self.auto_generate:
            # Auto-generate test waypoints
            self.get_logger().info('Auto-generating test waypoints')
            self.generate_test_waypoints()
        else:
            self.get_logger().error(f'No waypoint file found at {self.waypoint_file} and auto_generate is disabled')
    
    def generate_test_waypoints(self):
        """Generate test waypoints in a pattern for testing"""
        # Clear existing waypoints
        self.waypoints = []
        
        # Waypoints from the provided list
        points = [
            (334.949799, -161.106171, 0.001736),
            (339.100037, -258.568939, 0.001679),
            (396.295319, -183.195740, 0.001678),
            (267.657074, -1.983160, 0.001678),
            (153.868896, -26.115866, 0.001678),
            (290.515564, -56.175072, 0.001677),
            (92.325722, -86.063644, 0.001677),
            (88.384346, -287.468567, 0.001728),
            (177.594101, -326.386902, 0.001677),
            (-1.646942, -197.501282, 0.001555),
            (59.701321, -1.970804, 0.001467),
            (122.100121, -55.142044, 0.001596),
            (161.030975, -129.313187, 0.001679),
            (184.758713, -199.424271, 0.001680)
        ]
        
        for x, y, z in points:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0
            self.waypoints.append(pose)
        
        self.get_logger().info(f'Generated {len(self.waypoints)} test waypoints')
    
    def position_callback(self, msg):
        """Handle current vehicle position updates"""
        self.current_position = msg
    
    def publish_waypoints(self):
        """Publish waypoints relative to current vehicle position"""
        if not self.waypoints:
            self.get_logger().warn('No waypoints to publish, generating defaults')
            self.generate_test_waypoints()
            
        if not self.waypoints:
            self.get_logger().warn('No waypoints to publish')
            return
        
        # Create PoseArray message
        waypoint_msg = PoseArray()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = "map"
        
        # Add waypoints as absolute coordinates (not relative)
        waypoint_msg.poses = self.waypoints
        
        # Publish waypoints
        self.waypoint_publisher.publish(waypoint_msg)
        self.get_logger().info(f'Published {len(waypoint_msg.poses)} waypoints')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()