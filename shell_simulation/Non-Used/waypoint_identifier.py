#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import pickle
import os
from visualization_msgs.msg import Marker, MarkerArray

class WaypointIdentifier(Node):
    def __init__(self):
        super().__init__('waypoint_identifier')
        
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
        
        # Check multiple possible data locations
        self.positions = []
        self.data_locations = [
            'map_data/positions.pkl',
            os.path.join(os.path.expanduser('~'), 'map_data/positions.pkl'),
            os.path.join(os.getcwd(), 'map_data/positions.pkl')
        ]
        
        # Try to load from any of the possible locations
        self.load_positions()
        
        # Create test data if nothing was loaded
        if not self.positions:
            self.create_test_data()
        
        # Nearest indices
        self.nearest_indices = []
        
        # Create publisher for visualization
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'waypoint_matches',
            10)
        
        # Identify nearest points
        self.identify_nearest_points()
        
        # Create timer for visualization
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)
        
        self.get_logger().info(f"Waypoint Identifier initialized with {len(self.positions)} positions")
    
    def load_positions(self):
        """Try to load positions from different possible locations"""
        for location in self.data_locations:
            try:
                if os.path.exists(location):
                    with open(location, 'rb') as f:
                        self.positions = pickle.load(f)
                    self.get_logger().info(f"Loaded {len(self.positions)} positions from {location}")
                    return
            except Exception as e:
                self.get_logger().error(f"Error loading from {location}: {str(e)}")
        
        # If we reach here, we couldn't load from any location
        self.get_logger().error(f"Could not load positions from any of these locations: {self.data_locations}")
    
    def create_test_data(self):
        """Create some test position data since we couldn't load real data"""
        self.get_logger().warn("Creating test data since no position data was found")
        
        # Create a grid of test positions around the waypoints
        for waypoint in self.competition_waypoints:
            # Add the waypoint itself
            self.positions.append([waypoint[0], waypoint[1], waypoint[2]])
            
            # Add some points around it
            for dx in [-5, 0, 5]:
                for dy in [-5, 0, 5]:
                    if dx == 0 and dy == 0:
                        continue  # Skip the center point (already added)
                    
                    self.positions.append([
                        waypoint[0] + dx,
                        waypoint[1] + dy,
                        waypoint[2]
                    ])
        
        self.get_logger().info(f"Created {len(self.positions)} test positions")
        
        # Save this test data
        try:
            os.makedirs('map_data', exist_ok=True)
            with open('map_data/positions.pkl', 'wb') as f:
                pickle.dump(self.positions, f)
            self.get_logger().info("Saved test data to map_data/positions.pkl")
        except Exception as e:
            self.get_logger().error(f"Error saving test data: {str(e)}")
    
    def identify_nearest_points(self):
        """Identify nearest recorded points to competition waypoints"""
        if not self.positions:
            self.get_logger().error("No positions available, cannot identify nearest points")
            return
        
        self.nearest_indices = []
        
        for waypoint in self.competition_waypoints:
            min_distance = float('inf')
            nearest_idx = 0
            
            for i, point in enumerate(self.positions):
                dist = math.sqrt(
                    (waypoint[0] - point[0])**2 + 
                    (waypoint[1] - point[1])**2)
                
                if dist < min_distance:
                    min_distance = dist
                    nearest_idx = i
            
            self.nearest_indices.append(nearest_idx)
            self.get_logger().info(f"Waypoint {waypoint} -> nearest point: {self.positions[nearest_idx]} (idx {nearest_idx}, distance: {min_distance:.2f}m)")
        
        # Save nearest indices
        try:
            os.makedirs('map_data', exist_ok=True)
            with open('map_data/nearest_indices.pkl', 'wb') as f:
                pickle.dump(self.nearest_indices, f)
            self.get_logger().info(f"Saved {len(self.nearest_indices)} nearest indices to file")
        except Exception as e:
            self.get_logger().error(f"Error saving nearest indices: {str(e)}")
    
    def publish_visualization(self):
        """Publish visualization of waypoints and nearest points"""
        if not self.positions or not self.nearest_indices:
            return
        
        marker_array = MarkerArray()
        
        # Create markers for competition waypoints
        for i, waypoint in enumerate(self.competition_waypoints):
            # Waypoint marker
            wp_marker = Marker()
            wp_marker.header.frame_id = "map"
            wp_marker.header.stamp = self.get_clock().now().to_msg()
            wp_marker.ns = "competition_waypoints"
            wp_marker.id = i
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.pose.position.x = waypoint[0]
            wp_marker.pose.position.y = waypoint[1]
            wp_marker.pose.position.z = waypoint[2]
            wp_marker.scale.x = 2.0
            wp_marker.scale.y = 2.0
            wp_marker.scale.z = 2.0
            wp_marker.color.r = 1.0
            wp_marker.color.g = 0.0
            wp_marker.color.b = 0.0
            wp_marker.color.a = 1.0
            
            marker_array.markers.append(wp_marker)
            
            # Nearest point marker
            nearest_idx = self.nearest_indices[i]
            nearest_point = self.positions[nearest_idx]
            
            np_marker = Marker()
            np_marker.header.frame_id = "map"
            np_marker.header.stamp = self.get_clock().now().to_msg()
            np_marker.ns = "nearest_points"
            np_marker.id = i
            np_marker.type = Marker.SPHERE
            np_marker.action = Marker.ADD
            np_marker.pose.position.x = nearest_point[0]
            np_marker.pose.position.y = nearest_point[1]
            np_marker.pose.position.z = nearest_point[2]
            np_marker.scale.x = 1.5
            np_marker.scale.y = 1.5
            np_marker.scale.z = 1.5
            np_marker.color.r = 0.0
            np_marker.color.g = 0.0
            np_marker.color.b = 1.0
            np_marker.color.a = 1.0
            
            marker_array.markers.append(np_marker)
            
            # Line between waypoint and nearest point
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "connection_lines"
            line_marker.id = i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            # Add waypoint position
            p1 = Marker().pose.position
            p1.x = waypoint[0]
            p1.y = waypoint[1]
            p1.z = waypoint[2]
            
            # Add nearest point position
            p2 = Marker().pose.position
            p2.x = nearest_point[0]
            p2.y = nearest_point[1]
            p2.z = nearest_point[2]
            
            line_marker.points = [p1, p2]
            
            line_marker.scale.x = 0.3  # Line width
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 0.8
            
            marker_array.markers.append(line_marker)
        
        # Publish all markers
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    identifier = WaypointIdentifier()
    rclpy.spin(identifier)
    identifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()