#!/usr/bin/env python3

"""
CARLA Autopilot Waypoint Navigator - Uses CARLA's built-in autopilot
to navigate between waypoints without stopping
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import PoseArray, Pose, Point
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
import math
import numpy as np
import time
from enum import Enum

class NavigationState(Enum):
    INITIALIZING = 0    # Collecting initial data
    PLANNING = 1        # Planning optimal path
    NAVIGATING = 2      # Actively navigating to waypoints
    COMPLETE = 4        # Reached final waypoint

class CarlaAutopilotNavigator(Node):
    def __init__(self):
        super().__init__('carla_autopilot_navigator')
        
        # Load waypoints
        self.waypoints_data = [
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
        
        # Initialize waypoints and state
        self.waypoints = []
        self.waypoint_status = []  # False = not visited
        self.optimal_path = []
        self.current_waypoint_index = 0
        self.state = NavigationState.INITIALIZING
        self.current_position = None
        self.target_waypoint = None
        self.current_speed = 0.0
        
        # Navigation parameters
        self.arrival_tolerance = 8.0  # Distance in meters to consider waypoint reached
        
        # Publishers - Use both autopilot and direct control
        self.autopilot_publisher = self.create_publisher(
            Bool, 
            '/carla/ego_vehicle/enable_autopilot', 
            10)
            
        # Direct control for manual overrides when needed
        self.control_publisher = self.create_publisher(
            CarlaEgoVehicleControl, 
            '/carla/ego_vehicle/vehicle_control_cmd', 
            10)
            
        # Manual override control
        self.manual_override_publisher = self.create_publisher(
            Bool,
            '/carla/ego_vehicle/vehicle_control_manual_override',
            10
        )
        
        # Individual control publishers
        self.throttle_publisher = self.create_publisher(
            Float32,
            '/throttle_command',
            10
        )
        
        self.brake_publisher = self.create_publisher(
            Float32,
            '/brake_command',
            10
        )
        
        self.steering_publisher = self.create_publisher(
            Float32,
            '/steering_command',
            10
        )
        
        self.gear_publisher = self.create_publisher(
            String,
            '/gear_command',
            10
        )
        
        # Waypoint visualization
        self.path_publisher = self.create_publisher(
            PoseArray,
            '/planned_path',
            10
        )
        
        # Subscribers
        self.odometry_subscriber = self.create_subscription(
            Odometry, 
            '/carla/ego_vehicle/odometry', 
            self.odometry_callback, 
            10)
        
        self.speedometer_subscriber = self.create_subscription(
            Float32, 
            '/carla/ego_vehicle/speedometer', 
            self.speedometer_callback, 
            10)
        
        # Main timer
        self.timer = self.create_timer(0.5, self.navigation_loop)
        
        # Fast timer for control commands to keep vehicle moving
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Initialize waypoints
        self.initialize_waypoints()
        
        # Set gear to forward
        self.set_gear_forward()
        
        # Set initial state for control
        self.use_autopilot = True
        self.is_switching = False
        self.switch_timer = None
        
        self.get_logger().info('CARLA Autopilot Navigator initialized')
    
    def set_gear_forward(self):
        """Set vehicle gear to forward"""
        gear_msg = String()
        gear_msg.data = "forward"
        self.gear_publisher.publish(gear_msg)
    
    def initialize_waypoints(self):
        """Create waypoint objects from loaded data"""
        self.waypoints = []
        self.waypoint_status = []
        
        for wp_data in self.waypoints_data:
            pose = Pose()
            pose.position.x = wp_data[0]
            pose.position.y = wp_data[1]
            pose.position.z = wp_data[2]
            pose.orientation.w = 1.0
            
            self.waypoints.append(pose)
            self.waypoint_status.append(False)  # Not visited
            
        self.get_logger().info(f'Initialized {len(self.waypoints)} waypoints')
        
        # Move to planning state once waypoints are loaded
        self.state = NavigationState.PLANNING
    
    def plan_optimal_path(self):
        """Plan the optimal path through all waypoints using TSP approach"""
        if not self.current_position:
            self.get_logger().warn('Waiting for vehicle position data...')
            return False
            
        self.get_logger().info('Planning optimal path through waypoints...')
        
        # Simple implementation of Nearest Neighbor TSP algorithm
        # Start from current position
        current_pos = self.current_position
        unvisited = list(range(len(self.waypoints)))
        path = []
        
        while unvisited:
            # Find nearest unvisited waypoint
            min_dist = float('inf')
            nearest_idx = -1
            
            for idx in unvisited:
                wp = self.waypoints[idx]
                dist = self.calculate_distance(current_pos, wp.position)
                
                if dist < min_dist:
                    min_dist = dist
                    nearest_idx = idx
            
            if nearest_idx != -1:
                path.append(nearest_idx)
                unvisited.remove(nearest_idx)
                current_pos = self.waypoints[nearest_idx].position
            else:
                break
        
        self.optimal_path = path
        self.current_waypoint_index = 0
        
        # Output the optimized path
        path_str = ' -> '.join([str(idx) for idx in self.optimal_path])
        self.get_logger().info(f'Optimized path: {path_str}')
        
        # Publish path for visualization
        self.publish_path()
        
        return True
    
    def publish_path(self):
        """Publish the planned path for visualization"""
        if not self.optimal_path:
            return
            
        path_msg = PoseArray()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for idx in self.optimal_path:
            path_msg.poses.append(self.waypoints[idx])
            
        self.path_publisher.publish(path_msg)
    
    def odometry_callback(self, msg):
        """Process vehicle odometry data"""
        self.current_position = msg.pose.pose.position
        
        # If we're in navigating state, check if we've reached the current waypoint
        if self.state == NavigationState.NAVIGATING and self.current_waypoint_index < len(self.optimal_path):
            target_wp = self.waypoints[self.optimal_path[self.current_waypoint_index]]
            dist = self.calculate_distance(self.current_position, target_wp.position)
            
            if dist < self.arrival_tolerance:
                self.waypoint_status[self.optimal_path[self.current_waypoint_index]] = True
                
                self.get_logger().info(f'Reached waypoint {self.current_waypoint_index} ' + 
                                     f'(ID: {self.optimal_path[self.current_waypoint_index]})')
                
                # Temporarily switch to manual control to ensure continuous movement
                self.is_switching = True
                self.use_autopilot = False
                
                # Send throttle command immediately to prevent stopping
                self.apply_throttle(0.5)
                
                # Create a one-shot timer to switch back to autopilot after 0.5 seconds
                if self.switch_timer:
                    self.destroy_timer(self.switch_timer)
                self.switch_timer = self.create_timer(0.5, self.switch_back_to_autopilot)
                
                # Update current waypoint index
                self.current_waypoint_index += 1
                
                if self.current_waypoint_index >= len(self.optimal_path):
                    self.get_logger().info('Navigation complete - all waypoints reached!')
                    self.state = NavigationState.COMPLETE
                else:
                    # Update the target waypoint
                    self.update_target_waypoint()
    
    def switch_back_to_autopilot(self):
        """Switch back to autopilot after brief manual control"""
        if self.switch_timer:
            self.destroy_timer(self.switch_timer)
            self.switch_timer = None
        
        self.use_autopilot = True
        self.is_switching = False
        self.enable_autopilot()
        self.get_logger().info('Switched back to autopilot control')
    
    def speedometer_callback(self, msg):
        """Process vehicle speed data"""
        self.current_speed = msg.data
        
        # If speed drops too low during navigation, apply throttle
        if self.state == NavigationState.NAVIGATING and self.current_speed < 2.0:
            self.apply_throttle(0.5)
    
    def navigation_loop(self):
        """Main navigation control loop"""
        if self.state == NavigationState.PLANNING:
            # Try to plan the path
            if self.plan_optimal_path():
                # If planning succeeds, update target and start navigating
                self.update_target_waypoint()
                self.state = NavigationState.NAVIGATING
                self.enable_autopilot()
                
        elif self.state == NavigationState.NAVIGATING:
            # Periodically republish the path for visualization
            self.publish_path()
            
            # Check if we need to re-enable autopilot
            if self.use_autopilot and not self.is_switching:
                self.enable_autopilot()
    
    def control_loop(self):
        """Fast control loop for vehicle management"""
        if self.state == NavigationState.NAVIGATING:
            if not self.use_autopilot or self.is_switching:
                # Apply throttle during transitions or when not using autopilot
                self.apply_throttle(0.5)
            elif self.current_speed < 2.0 and self.use_autopilot:
                # If speed is too low even with autopilot, apply manual throttle briefly
                self.apply_throttle(0.5)
                
        elif self.state == NavigationState.COMPLETE:
            # Navigation completed, ensure vehicle is stopped
            self.stop_vehicle()
    
    def update_target_waypoint(self):
        """Update the current target waypoint"""
        if self.current_waypoint_index < len(self.optimal_path):
            wp_idx = self.optimal_path[self.current_waypoint_index]
            self.target_waypoint = self.waypoints[wp_idx]
            
            self.get_logger().info(f'Target waypoint {self.current_waypoint_index} ' +
                                f'(ID: {wp_idx}) at ' +
                                f'({self.target_waypoint.position.x:.1f}, {self.target_waypoint.position.y:.1f})')
    
    def enable_autopilot(self):
        """Enable CARLA's autopilot"""
        # Enable autopilot
        autopilot_msg = Bool()
        autopilot_msg.data = True
        self.autopilot_publisher.publish(autopilot_msg)
        
        # Disable manual override
        override_msg = Bool()
        override_msg.data = False
        self.manual_override_publisher.publish(override_msg)
    
    def apply_throttle(self, throttle_value):
        """Apply throttle directly to the vehicle"""
        # Publish to main control topic
        control = CarlaEgoVehicleControl()
        control.throttle = float(throttle_value)
        control.steer = 0.0
        control.brake = 0.0
        control.hand_brake = False
        control.reverse = False
        control.manual_gear_shift = False
        control.gear = 1
        self.control_publisher.publish(control)
        
        # Also publish to throttle command topic
        throttle_msg = Float32()
        throttle_msg.data = float(throttle_value)
        self.throttle_publisher.publish(throttle_msg)
        
        # Enable manual override temporarily
        override_msg = Bool()
        override_msg.data = True
        self.manual_override_publisher.publish(override_msg)
    
    def stop_vehicle(self):
        """Stop the vehicle"""
        # Disable autopilot
        autopilot_msg = Bool()
        autopilot_msg.data = False
        self.autopilot_publisher.publish(autopilot_msg)
        
        # Enable manual override
        override_msg = Bool()
        override_msg.data = True
        self.manual_override_publisher.publish(override_msg)
        
        # Apply brake
        control = CarlaEgoVehicleControl()
        control.throttle = 0.0
        control.steer = 0.0
        control.brake = 1.0
        control.hand_brake = True
        control.reverse = False
        control.manual_gear_shift = False
        control.gear = 1
        self.control_publisher.publish(control)
        
        # Also publish to individual command topics
        throttle_msg = Float32()
        throttle_msg.data = 0.0
        self.throttle_publisher.publish(throttle_msg)
        
        brake_msg = Float32()
        brake_msg.data = 1.0
        self.brake_publisher.publish(brake_msg)
    
    def calculate_distance(self, point1, point2):
        """Calculate Euclidean distance between two points"""
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        return math.sqrt(dx*dx + dy*dy)

def main(args=None):
    rclpy.init(args=args)
    node = CarlaAutopilotNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()