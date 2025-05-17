#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32

class SensorDebugNode(Node):
    def __init__(self):
        super().__init__('sensor_debug')

        self.create_subscription(Odometry, '/carla/ego_vehicle/odometry', self.odom_callback, 10)
        self.create_subscription(Imu, '/carla/ego_vehicle/imu', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/carla/ego_vehicle/gnss', self.gps_callback, 10)
        self.create_subscription(Float32, '/carla/ego_vehicle/speedometer', self.speed_callback, 10)

    def odom_callback(self, msg):
        self.get_logger().info(f'Odometry: x={msg.pose.pose.position.x:.2f}')

    def imu_callback(self, msg):
        self.get_logger().info(f'IMU Linear Accel: x={msg.linear_acceleration.x:.2f}')

    def gps_callback(self, msg):
        self.get_logger().info(f'GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}')

    def speed_callback(self, msg):
        self.get_logger().info(f'Speed: {msg.data:.2f} m/s')


def main(args=None):
    rclpy.init(args=args)
    node = SensorDebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
