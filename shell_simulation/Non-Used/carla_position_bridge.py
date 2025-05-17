#!/usr/bin/env python3

"""
Bridge node to forward vehicle position from Carla to navigation system
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

class CarlaPositionBridge(Node):
    def __init__(self):
        super().__init__('carla_position_bridge')
        
        # Publisher for navigation system
        self.position_publisher = self.create_publisher(Pose, 'vehicle_position', 10)
        
        # Subscriber to Carla odometry
        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',  # Carla odometry topic
            self.odometry_callback,
            10)
        
        self.get_logger().info('Carla Position Bridge initialized')
        self.get_logger().info('Waiting for odometry data from Carla...')
    
    def odometry_callback(self, msg):
        """Forward position from Carla odometry to vehicle_position topic"""
        pose_msg = Pose()
        
        # Copy position and orientation from odometry
        pose_msg.position = msg.pose.pose.position
        pose_msg.orientation = msg.pose.pose.orientation
        
        # Publish the position
        self.position_publisher.publish(pose_msg)
        
        # Log occasionally (every 10 callbacks) to avoid flooding
        if hasattr(self, 'callback_count'):
            self.callback_count += 1
            if self.callback_count % 10 == 0:
                self.get_logger().info(
                    f'Forwarded position: x={pose_msg.position.x:.2f}, y={pose_msg.position.y:.2f}, z={pose_msg.position.z:.2f}'
                )
        else:
            self.callback_count = 1
            self.get_logger().info('First odometry data received from Carla')
            self.get_logger().info(
                f'Initial position: x={pose_msg.position.x:.2f}, y={pose_msg.position.y:.2f}, z={pose_msg.position.z:.2f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = CarlaPositionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
