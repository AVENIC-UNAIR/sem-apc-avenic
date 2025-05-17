#!/usr/bin/env python3

"""
Bridge to connect our control commands to Carla control format
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from carla_msgs.msg import CarlaEgoVehicleControl

class CarlaControlBridge(Node):
    def __init__(self):
        super().__init__('carla_control_bridge')
        
        # Publisher for Carla control
        self.carla_control_publisher = self.create_publisher(
            CarlaEgoVehicleControl,
            '/carla/ego_vehicle/vehicle_control_cmd',
            1
        )
        
        # Subscribers for our control commands
        self.throttle_subscription = self.create_subscription(
            Float64,
            'throttle_command',
            self.throttle_callback,
            1
        )
        
        self.brake_subscription = self.create_subscription(
            Float64,
            'brake_command',
            self.brake_callback,
            1
        )
        
        self.steering_subscription = self.create_subscription(
            Float64,
            'steering_command',
            self.steering_callback,
            1
        )
        
        self.gear_subscription = self.create_subscription(
            String,
            'gear_command',
            self.gear_callback,
            1
        )
        
        # Initialize control values
        self.throttle = 0.0
        self.brake = 0.0
        self.steering = 0.0
        self.reverse = False
        
        # Timer to publish combined control at 20Hz
        self.timer = self.create_timer(0.05, self.publish_control)
        
        self.get_logger().info('Carla Control Bridge initialized')
    
    def throttle_callback(self, msg):
        self.throttle = float(msg.data)
    
    def brake_callback(self, msg):
        self.brake = float(msg.data)
    
    def steering_callback(self, msg):
        self.steering = float(msg.data)
    
    def gear_callback(self, msg):
        self.reverse = (msg.data.lower() == "reverse")
    
    def publish_control(self):
        """Publish control command to Carla"""
        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = self.throttle
        control_msg.steer = self.steering
        control_msg.brake = self.brake
        control_msg.hand_brake = False
        control_msg.reverse = self.reverse
        control_msg.gear = -1 if self.reverse else 1
        control_msg.manual_gear_shift = False
        
        self.carla_control_publisher.publish(control_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CarlaControlBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()