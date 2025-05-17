#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import csv
import os

class LidarLogger(Node):
    def __init__(self):
        super().__init__('lidar_logger')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/carla/ego_vehicle/vlp16_1',
            self.listener_callback,
            10)

        self.counter = 0

        # Tentukan folder simpan
        self.output_dir = os.path.expanduser('~/shell_ws/lidar_logs')
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"Saving LiDAR logs to: {self.output_dir}")

    def listener_callback(self, msg):
        self.counter += 1
        if self.counter % 10 != 0:
            return  # Log setiap 10 frame

        pc = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        filename = os.path.join(self.output_dir, f'lidar_frame_{self.counter}.csv')

        with open(filename, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z'])
            for point in pc:
                writer.writerow(point)

        self.get_logger().info(f"✅ Saved: {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
