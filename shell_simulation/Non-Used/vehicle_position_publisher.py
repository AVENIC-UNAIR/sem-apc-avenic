#!/usr/bin/env python3

"""
Publishes vehicle position data from CARLA simulator or other sources
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import time

class VehiclePositionPublisher(Node):
    def __init__(self):
        super().__init__('vehicle_position_publisher')
        
        # Publisher for vehicle position
        self.position_publisher = self.create_publisher(Pose, 'vehicle_position', 10)
        
        # Parameter for publishing simulated position data
        self.declare_parameter('use_simulated_data', True)
        
        # Timer for periodic publishing (10Hz)
        self.timer = self.create_timer(0.1, self.publish_position)
        
        # For simulated data
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        self.get_logger().info('Vehicle Position Publisher initialized')
        
        if self.get_parameter('use_simulated_data').value:
            self.get_logger().info('Using simulated position data')
        else:
            self.get_logger().info('Will attempt to get real position data from CARLA')
            # Here you would connect to CARLA or another source for real data
    
    def publish_position(self):
        """Publish vehicle position data"""
        pose_msg = Pose()
        
        if self.get_parameter('use_simulated_data').value:
            # Use simulated data for testing
            pose_msg.position.x = self.current_x
            pose_msg.position.y = self.current_y
            pose_msg.position.z = self.current_z
            
            # Default orientation (facing forward)
            pose_msg.orientation.w = 1.0
            
            # Move the simulated vehicle a bit for testing
            self.current_x += 0.1
        else:
            # Here you would get real position data from CARLA
            # This is a placeholder - replace with actual code to get position
            pass
        
        # Publish the position
        self.position_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VehiclePositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()