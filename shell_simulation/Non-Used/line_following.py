#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class LaneFollowingNavigator(Node):
    def __init__(self):
        super().__init__('lane_following_navigator')
        
        # Competition waypoints (as reference points for major turns)
        self.waypoints = [
            [334.949799, -161.106171, 0.001736],  # 63.17m from start
            [339.100037, -258.568939, 0.001679],  # 141.98m from start
            [396.295319, -183.195740, 0.001678],  # 127.84m from start
            [267.657074, -1.983160, 0.001678],    # 127.96m from start
            [153.868896, -26.115866, 0.001678],   # 163.25m from start
            [290.515564, -56.175072, 0.001677],   # 73.83m from start
            [92.325722, -86.063644, 0.001677],    # 192.95m from start
            [88.384346, -287.468567, 0.001728],   # 248.74m from start
            [177.594101, -326.386902, 0.001677],  # 222.27m from start
            [-1.646942, -197.501282, 0.001555],   # 290.14m from start
            [59.701321, -1.970804, 0.001467],     # 254.77m from start
            [122.100121, -55.142044, 0.001596],   # 174.78m from start
            [161.030975, -129.313187, 0.001679],  # 119.33m from start
            [184.758713, -199.424271, 0.001680]   # 118.56m from start
        ]
        
        # Vehicle state
        self.current_position = None
        self.current_orientation = None
        self.current_velocity = 0.0
        self.lane_offset = 0.0  # Lateral offset from lane center
        self.lane_detected = False
        
        # Navigation state
        self.current_waypoint_idx = 0
        self.last_log_time = 0.0
        self.target_speed = 15.0  # km/h
        
        # PID controller parameters
        # Steering PID
        self.steering_kp = 0.5     # Proportional gain
        self.steering_ki = 0.01    # Integral gain
        self.steering_kd = 0.1     # Derivative gain
        self.steering_error_sum = 0.0
        self.steering_last_error = 0.0
        
        # Speed PID
        self.speed_kp = 0.1
        self.speed_ki = 0.01
        self.speed_kd = 0.05
        self.speed_error_sum = 0.0
        self.speed_last_error = 0.0
        
        # Safety parameters
        self.max_steering = 0.8    # Max steering value
        self.max_throttle = 0.3    # Max throttle
        self.min_throttle = 0.1    # Min throttle to keep moving
        self.max_brake = 0.5       # Max brake
        self.waypoint_threshold = 20.0  # Distance to consider waypoint reached (m)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/carla/ego_vehicle/odometry', 
            self.odom_callback, 
            10)
        
        self.camera_sub = self.create_subscription(
            Image,
            '/carla/ego_vehicle/rgb_front/image',
            self.camera_callback,
            10)
        
        # Create publishers
        self.throttle_pub = self.create_publisher(Float64, 'throttle_command', 10)
        self.brake_pub = self.create_publisher(Float64, 'brake_command', 10)
        self.steering_pub = self.create_publisher(Float64, 'steering_command', 10)
        self.gear_pub = self.create_publisher(String, 'gear_command', 10)
        
        # Set initial gear
        gear_msg = String()
        gear_msg.data = 'forward'
        self.gear_pub.publish(gear_msg)
        
        # Create control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz control loop
        
        self.get_logger().info("Lane Following Navigator initialized")
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def log_throttle(self, period, msg):
        """Log message at throttled rate"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_log_time > period:
            self.get_logger().info(msg)
            self.last_log_time = current_time
    
    def odom_callback(self, msg):
        """Update vehicle state from odometry"""
        if not self.current_position:
            self.current_position = Point()
        
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
        
        # Extract orientation
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.current_orientation = yaw
        
        # Extract velocity (convert from m/s to km/h)
        self.current_velocity = math.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2) * 3.6
    
    def camera_callback(self, msg):
        """Process camera image for lane detection"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect lanes and calculate offset from center
            lane_offset = self.detect_lanes(cv_image)
            
            if lane_offset is not None:
                self.lane_offset = lane_offset
                self.lane_detected = True
            else:
                self.lane_detected = False
            
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {str(e)}")
    
    def detect_lanes(self, image):
        """Detect lane markings and calculate offset from center"""
        try:
            # Get image dimensions
            height, width, _ = image.shape
            
            # Create ROI (bottom half of the image)
            roi_height = height // 2
            roi = image[height - roi_height:height, 0:width]
            
            # Convert to grayscale
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Apply Canny edge detection
            edges = cv2.Canny(blur, 50, 150)
            
            # Apply mask to focus on the road area
            mask = np.zeros_like(edges)
            polygon = np.array([
                [(0, roi_height), (width//3, roi_height//2), (2*width//3, roi_height//2), (width, roi_height)]
            ], np.int32)
            cv2.fillPoly(mask, polygon, 255)
            masked_edges = cv2.bitwise_and(edges, mask)
            
            # Use Hough Transform to detect lines
            lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, threshold=20, minLineLength=20, maxLineGap=300)
            
            if lines is None or len(lines) == 0:
                return None
            
            # Separate left and right lane lines
            left_lines = []
            right_lines = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 == x1:  # Avoid division by zero
                    continue
                
                slope = (y2 - y1) / (x2 - x1)
                
                # Filter lines by slope
                if abs(slope) < 0.1:  # Nearly horizontal lines
                    continue
                
                if slope < 0:  # Negative slope = left lane
                    left_lines.append(line)
                else:  # Positive slope = right lane
                    right_lines.append(line)
            
            # Calculate left and right lane positions
            left_x = width // 4  # Default if no left lane
            right_x = 3 * width // 4  # Default if no right lane
            
            if left_lines:
                left_points = np.concatenate([line for line in left_lines])
                left_x = np.mean(left_points[:, [0, 2]])
            
            if right_lines:
                right_points = np.concatenate([line for line in right_lines])
                right_x = np.mean(right_points[:, [0, 2]])
            
            # Calculate center of the lane
            lane_center = (left_x + right_x) / 2
            
            # Calculate offset from the center of the vehicle
            center_offset = lane_center - width / 2
            
            # Normalize offset to [-1, 1] range
            normalized_offset = center_offset / (width / 2)
            
            return normalized_offset
            
        except Exception as e:
            self.get_logger().error(f"Lane detection error: {str(e)}")
            return None
    
    def control_loop(self):
        """Main control loop using PID controllers"""
        if not self.current_position or self.current_orientation is None:
            if self.get_clock().now().nanoseconds / 1e9 - self.last_log_time > 5.0:
                self.get_logger().warn("Waiting for vehicle state data...")
                self.last_log_time = self.get_clock().now().nanoseconds / 1e9
            return
        
        # Check if we've completed all waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            self.stop_vehicle()
            self.log_throttle(5.0, "All waypoints reached! Mission complete.")
            return
        
        # Get current target waypoint
        target_waypoint = self.waypoints[self.current_waypoint_idx]
        
        # Calculate distance to current waypoint
        distance_to_waypoint = self.calculate_distance(target_waypoint)
        
        # Check if we've reached the waypoint
        if distance_to_waypoint < self.waypoint_threshold:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx + 1} of {len(self.waypoints)}")
            self.current_waypoint_idx += 1
            
            # Slow down briefly when reaching waypoint
            self.apply_control(throttle=0.0, brake=0.3, steering=0.0)
            return
        
        # Calculate steering using PID
        if self.lane_detected:
            # Use lane position for steering (lane following)
            steering = self.calculate_steering_pid(self.lane_offset)
            self.log_throttle(1.0, f"Following lane, offset: {self.lane_offset:.2f}, steering: {steering:.2f}")
        else:
            # If lane not detected, steer towards waypoint
            target_angle = math.atan2(
                target_waypoint[1] - self.current_position.y,
                target_waypoint[0] - self.current_position.x)
            
            heading_error = target_angle - self.current_orientation
            # Normalize to [-pi, pi]
            if heading_error > math.pi:
                heading_error -= 2 * math.pi
            elif heading_error < -math.pi:
                heading_error += 2 * math.pi
                
            steering = self.steering_kp * heading_error / math.pi
            self.log_throttle(1.0, f"Lane not detected, using waypoint guidance. Steering: {steering:.2f}")
        
        # Calculate throttle and brake using PID
        speed_error = self.target_speed - self.current_velocity
        throttle, brake = self.calculate_throttle_brake_pid(speed_error, abs(steering))
        
        # Apply control
        self.apply_control(throttle=throttle, brake=brake, steering=steering)
        
        # Log status
        self.log_throttle(1.0, 
                         f"Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}, "
                         f"Distance: {distance_to_waypoint:.1f}m, Speed: {self.current_velocity:.1f} km/h, "
                         f"Steering: {steering:.2f}, Throttle: {throttle:.2f}")
    
    def calculate_steering_pid(self, lane_offset):
        """Calculate steering using PID controller based on lane offset"""
        # Proportional term
        p_term = self.steering_kp * lane_offset
        
        # Integral term
        self.steering_error_sum += lane_offset
        self.steering_error_sum = max(-10.0, min(10.0, self.steering_error_sum))  # Limit integral windup
        i_term = self.steering_ki * self.steering_error_sum
        
        # Derivative term
        d_term = self.steering_kd * (lane_offset - self.steering_last_error)
        self.steering_last_error = lane_offset
        
        # Combine terms
        steering = p_term + i_term + d_term
        
        # Limit steering
        steering = max(-self.max_steering, min(self.max_steering, steering))
        
        return steering
    
    def calculate_throttle_brake_pid(self, speed_error, steering_magnitude):
        """Calculate throttle and brake using PID controller based on speed error"""
        # Proportional term
        p_term = self.speed_kp * speed_error
        
        # Integral term
        self.speed_error_sum += speed_error
        self.speed_error_sum = max(-10.0, min(10.0, self.speed_error_sum))  # Limit integral windup
        i_term = self.speed_ki * self.speed_error_sum
        
        # Derivative term
        d_term = self.speed_kd * (speed_error - self.speed_last_error)
        self.speed_last_error = speed_error
        
        # Combine terms
        throttle_control = p_term + i_term + d_term
        
        # Reduce throttle in sharp turns
        turn_factor = 1.0 - steering_magnitude * 0.7
        throttle_control *= turn_factor
        
        # Determine throttle and brake
        if throttle_control > 0:
            throttle = min(self.max_throttle, throttle_control)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(self.max_brake, -throttle_control)
        
        # Ensure minimum throttle to keep moving if not braking
        if brake < 0.01 and throttle < self.min_throttle:
            throttle = self.min_throttle
        
        return throttle, brake
    
    def calculate_distance(self, target):
        """Calculate Euclidean distance to target"""
        return math.sqrt(
            (self.current_position.x - target[0])**2 + 
            (self.current_position.y - target[1])**2)
    
    def apply_control(self, throttle, brake, steering):
        """Apply control to vehicle"""
        throttle_msg = Float64()
        brake_msg = Float64()
        steering_msg = Float64()
        
        throttle_msg.data = max(0.0, min(1.0, throttle))
        brake_msg.data = max(0.0, min(1.0, brake))
        steering_msg.data = max(-1.0, min(1.0, steering))
        
        self.throttle_pub.publish(throttle_msg)
        self.brake_pub.publish(brake_msg)
        self.steering_pub.publish(steering_msg)
    
    def stop_vehicle(self):
        """Stop the vehicle"""
        self.apply_control(throttle=0.0, brake=1.0, steering=0.0)

def main(args=None):
    rclpy.init(args=args)
    
    # Initialize node
    try:
        navigator = LaneFollowingNavigator()
        rclpy.spin(navigator)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
