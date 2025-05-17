#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pickle
import os
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray

class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')
        
        # Load recorded positions
        self.positions = []
        self.load_positions()
        
        # Create publishers for visualization
        self.path_pub = self.create_publisher(
            PoseArray,
            'recorded_path',
            10)
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'waypoint_markers',
            10)
        
        # Create timer for visualization
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)
        
        self.get_logger().info(f"Map Visualizer initialized with {len(self.positions)} points")
    
    def load_positions(self):
        """Load recorded positions from file"""
        try:
            if os.path.exists('map_data/positions.pkl'):
                with open('map_data/positions.pkl', 'rb') as f:
                    self.positions = pickle.load(f)
                self.get_logger().info(f"Loaded {len(self.positions)} positions from file")
            else:
                self.get_logger().error("No positions file found. Run simple_mapper first.")
        except Exception as e:
            self.get_logger().error(f"Error loading positions: {str(e)}")
    
    def publish_visualization(self):
        """Publish visualization of recorded path"""
        if not self.positions:
            self.get_logger().warn("No positions to visualize")
            return
        
        # Create path message
        path_msg = PoseArray()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for point in self.positions:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = point[2]
            path_msg.poses.append(pose)
        
        # Publish path
        self.path_pub.publish(path_msg)
        
        # Create marker for every 10th point
        marker_array = MarkerArray()
        
        for i in range(0, len(self.positions), 10):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "recorded_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.positions[i][0]
            marker.pose.position.y = self.positions[i][1]
            marker.pose.position.z = self.positions[i][2]
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
        
        # Publish markers
        self.marker_pub.publish(marker_array)
        
        self.get_logger().info("Published visualization of recorded path")

def main(args=None):
    rclpy.init(args=args)
    visualizer = MapVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()