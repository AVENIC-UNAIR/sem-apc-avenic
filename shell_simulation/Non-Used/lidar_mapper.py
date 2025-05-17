#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os
from datetime import datetime

class LidarMapper(Node):
    def __init__(self):
        super().__init__('lidar_mapper')

        self.create_subscription(PointCloud2, '/carla/ego_vehicle/vlp16_1', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/carla/ego_vehicle/odometry', self.odom_callback, 10)

        self.pcd_data = []
        self.pose_data = []
        self.frame_count = 0

        self.get_logger().info("📡 LiDAR Mapper started...")

    def lidar_callback(self, msg):
        pc = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pc_np = np.array([[p[0], p[1], p[2]] for p in pc], dtype=np.float32)

        self.pcd_data.append(pc_np)

        self.frame_count += 1
        if self.frame_count % 10 == 0:
            self.get_logger().info(f"[{self.frame_count}] LiDAR frame saved. Points: {pc_np.shape[0]}")

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.pose_data.append([
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w
        ])

    def save_data(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        folder = os.path.expanduser('~/shell_ws/lidar_map')
        os.makedirs(folder, exist_ok=True)

        save_path = os.path.join(folder, f'map_{timestamp}.npz')
        np.savez_compressed(save_path, pointcloud=self.pcd_data, pose=self.pose_data)
        self.get_logger().info(f"💾 Map saved to {save_path}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_data()
        node.get_logger().info("🛑 Mapping stopped. Data saved.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
