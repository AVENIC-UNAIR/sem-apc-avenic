#!/usr/bin/env python3

"""
Sensor Fusion Node untuk menggabungkan data odometri dan LiDAR
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Declare parameters
        self.declare_parameter('odom_weight', 0.8)
        
        # Get parameters
        self.odom_weight = self.get_parameter('odom_weight').value
        self.lidar_weight = 1.0 - self.odom_weight  # Adjust lidar weight accordingly
        
        # Publishers
        self.filtered_pose_pub = self.create_publisher(
            PoseStamped,
            '/vehicle/filtered_pose',
            10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odometry_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/carla/ego_vehicle/vlp16_1',
            self.lidar_callback,
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Variables
        self.odom_pose = None
        self.lidar_data_received = False
        self.filter_initialized = False
        
        # Simple Kalman filter state
        self.position_estimate = np.zeros(3)  # [x, y, z]
        self.velocity_estimate = np.zeros(3)  # [vx, vy, vz]
        
        # Timer for publishing filtered pose
        self.timer = self.create_timer(0.05, self.publish_filtered_pose)
        
        self.get_logger().info(f'Sensor Fusion initialized with odom_weight={self.odom_weight}')
    
    def odometry_callback(self, msg):
        """Process vehicle odometry data"""
        self.odom_pose = msg.pose.pose
        
        # Extract position and velocity
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # Initialize filter if not done yet
        if not self.filter_initialized:
            self.position_estimate = position
            self.velocity_estimate = velocity
            self.filter_initialized = True
            return
        
        # Update state with odometry
        self.position_estimate = self.position_estimate * (1 - self.odom_weight) + position * self.odom_weight
        self.velocity_estimate = self.velocity_estimate * (1 - self.odom_weight) + velocity * self.odom_weight
    
    def lidar_callback(self, msg):
        """Process LiDAR data"""
        self.lidar_data_received = True
        
        # In a real implementation, we would use LiDAR data to correct position
        # For this example, we're just marking that we received LiDAR data
        # A full implementation would extract ground plane, landmarks, etc.
        
        # Note: LiDAR processing is complex and beyond the scope of this example
        # For now, we'll rely primarily on odometry data
    
    def publish_filtered_pose(self):
        """Publish the filtered pose estimate"""
        if not self.filter_initialized:
            return
        
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        pose_msg.pose.position.x = self.position_estimate[0]
        pose_msg.pose.position.y = self.position_estimate[1]
        pose_msg.pose.position.z = self.position_estimate[2]
        
        # Copy orientation from odometry (if available)
        if self.odom_pose is not None:
            pose_msg.pose.orientation = self.odom_pose.orientation
        else:
            pose_msg.pose.orientation.w = 1.0  # Default quaternion
        
        # Publish filtered pose
        self.filtered_pose_pub.publish(pose_msg)
        
        # Broadcast transform
        self.broadcast_transform(pose_msg)
    
    def broadcast_transform(self, pose_msg):
        """Broadcast transform from map to vehicle"""
        t = TransformStamped()
        t.header = pose_msg.header
        t.child_frame_id = "vehicle"
        
        # Set translation
        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z
        
        # Set rotation
        t.transform.rotation = pose_msg.pose.orientation
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 