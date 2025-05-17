#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3, PoseStamped
from std_msgs.msg import Float32, String, Bool
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
import pickle
import os
from collections import deque

class RoadLearner(Node):
    def __init__(self):
        super().__init__('road_learner')
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/carla/ego_vehicle/vlp16_1',
            self.lidar_callback,
            10)
            
        self.odom_sub = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odom_callback,
            10)
            
        self.current_wp_sub = self.create_subscription(
            PoseStamped,
            '/current_waypoint',
            self.waypoint_callback,
            10)
        
        # Publishers - untuk output hasil analisis
        self.road_type_pub = self.create_publisher(
            String, 
            '/road_type', 
            10)
            
        self.recommended_steering_pub = self.create_publisher(
            Float32, 
            '/recommended_steering', 
            10)
            
        self.recommended_throttle_pub = self.create_publisher(
            Float32, 
            '/recommended_throttle', 
            10)
            
        self.road_direction_pub = self.create_publisher(
            Vector3,
            '/road_direction',
            10)
            
        # State variables
        self.vehicle_pose = None
        self.vehicle_speed = 0.0
        self.vehicle_yaw = 0.0
        self.current_waypoint = None
        self.processed_lidar = None
        
        # Riwayat data untuk analisis
        self.pose_history = deque(maxlen=20)  # Riwayat 2 detik (10Hz)
        self.direction_history = deque(maxlen=10)
        
        # Road pattern detection
        self.road_features = {}
        self.road_profiles = {
            'STRAIGHT': {'var_angle': 0.1, 'lidar_pattern': 'parallel'},
            'CURVE_LEFT': {'var_angle': 0.5, 'lidar_pattern': 'left_open'},
            'CURVE_RIGHT': {'var_angle': 0.5, 'lidar_pattern': 'right_open'},
            'JUNCTION': {'var_angle': 0.3, 'lidar_pattern': 'open'}
        }
        
        # Path untuk menyimpan data pelatihan
        self.model_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'training_data')
        os.makedirs(self.model_dir, exist_ok=True)
        
        # Mode pelatihan
        self.training_mode = True
        self.training_samples = []
        self.max_samples = 5000
        
        # Timer untuk analisis dan pemrosesan
        self.create_timer(0.1, self.analyze_road)  # 10 Hz
        self.create_timer(5.0, self.save_training_data)  # Simpan setiap 5 detik
        
        self.get_logger().info("Road Learner initialized")
        
    def odom_callback(self, msg):
        """Update state kendaraan dari odometry"""
        self.vehicle_pose = msg.pose.pose
        
        # Hitung kecepatan
        self.vehicle_speed = math.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2
        )
        
        # Extract yaw dari quaternion
        q = self.vehicle_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.vehicle_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Simpan untuk analisis
        if self.vehicle_pose:
            self.pose_history.append({
                'x': self.vehicle_pose.position.x,
                'y': self.vehicle_pose.position.y,
                'yaw': self.vehicle_yaw,
                'speed': self.vehicle_speed
            })
            
    def waypoint_callback(self, msg):
        """Simpan waypoint saat ini"""
        self.current_waypoint = msg.pose
        
    def lidar_callback(self, msg):
        """Proses data LiDAR untuk analisis jalan"""
        try:
            # Konversi PointCloud2 ke array numpy dengan format yang benar
            points = []
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                points.append([p[0], p[1], p[2]])  # Konversi explicit ke list 3D
            
            if not points:
                return
                
            # Konversi ke numpy array
            points_array = np.array(points)
            
            # Filter untuk mendapatkan titik-titik di level permukaan jalan
            # Pastikan array memiliki dimensi yang benar
            if points_array.ndim != 2 or points_array.shape[1] < 3:
                self.get_logger().warn(f"Unexpected point cloud format: shape {points_array.shape}")
                return
                
            road_points = points_array[(points_array[:, 2] > -0.5) & (points_array[:, 2] < 0.3)]
            
            if len(road_points) < 10:
                return
                
            # Bagi LiDAR menjadi beberapa sektor
            sectors = 16
            sector_distances = np.ones(sectors) * 50.0  # Default max range
            
            # Hitung jarak minimum per sektor
            for point in road_points:
                x, y = point[0], point[1]
                distance = math.sqrt(x*x + y*y)
                angle = math.atan2(y, x)  # -pi to pi
                
                # Konversi ke indeks sektor (0 sampai sectors-1)
                sector_idx = int((angle + math.pi) * sectors / (2 * math.pi)) % sectors
                
                if distance < sector_distances[sector_idx]:
                    sector_distances[sector_idx] = distance
            
            # Analisis pola jalan dari profil LiDAR
            left_distances = sector_distances[:sectors//4]
            right_distances = sector_distances[-sectors//4:]
            front_distances = sector_distances[3*sectors//8:5*sectors//8]
            
            # Identifikasi pola
            left_avg = np.mean(left_distances)
            right_avg = np.mean(right_distances)
            front_avg = np.mean(front_distances)
            
            lidar_pattern = 'unknown'
            if front_avg > 20.0 and left_avg < 10.0 and right_avg < 10.0:
                lidar_pattern = 'parallel'  # Jalan lurus, dinding di kiri dan kanan
            elif left_avg > right_avg * 1.5:
                lidar_pattern = 'left_open'  # Terbuka di kiri (belok kiri)
            elif right_avg > left_avg * 1.5:
                lidar_pattern = 'right_open'  # Terbuka di kanan (belok kanan)
            elif front_avg > 15.0 and left_avg > 10.0 and right_avg > 10.0:
                lidar_pattern = 'open'  # Persimpangan
                
            # Simpan hasil
            self.processed_lidar = {
                'sectors': sector_distances.tolist(),
                'left_avg': left_avg,
                'right_avg': right_avg,
                'front_avg': front_avg,
                'pattern': lidar_pattern
            }
            
        except Exception as e:
            self.get_logger().error(f"Error saat memproses LiDAR: {str(e)}")
    
    def calculate_path_direction(self):
        """Hitung arah jalur kendaraan dari riwayat posisi"""
        if len(self.pose_history) < 5:
            return [1.0, 0.0]  # Default arah ke depan
            
        # Gunakan beberapa posisi terakhir untuk menentukan arah jalur
        recent_poses = list(self.pose_history)
        
        # Hitung vektor arah
        if len(recent_poses) >= 2:
            start = recent_poses[0]
            end = recent_poses[-1]
            
            dx = end['x'] - start['x']
            dy = end['y'] - start['y']
            
            # Normalisasi
            mag = math.sqrt(dx*dx + dy*dy)
            if mag > 0.1:  # Pastikan mobil bergerak cukup jauh
                dx /= mag
                dy /= mag
                return [dx, dy]
        
        # Fallback: gunakan orientasi kendaraan saat ini
        return [math.cos(self.vehicle_yaw), math.sin(self.vehicle_yaw)]
    
    def detect_road_type(self):
        """Deteksi tipe jalan dari pola LiDAR dan riwayat gerakan"""
        if not self.processed_lidar or len(self.pose_history) < 5:
            return "UNKNOWN"
            
        # Cek variabilitas arah gerakan
        yaws = [pose['yaw'] for pose in self.pose_history]
        yaw_diffs = [abs((yaws[i+1] - yaws[i] + math.pi) % (2*math.pi) - math.pi) 
                    for i in range(len(yaws)-1)]
        var_angle = np.var(yaw_diffs) if yaw_diffs else 0
        
        # Ambil pola LiDAR
        lidar_pattern = self.processed_lidar['pattern']
        
        # Gabungkan variabilitas sudut dan pola LiDAR untuk menentukan tipe jalan
        road_type = "UNKNOWN"
        
        if var_angle < 0.01 and lidar_pattern == 'parallel':
            road_type = "STRAIGHT"
        elif var_angle > 0.01 and lidar_pattern == 'left_open':
            road_type = "CURVE_LEFT"
        elif var_angle > 0.01 and lidar_pattern == 'right_open':
            road_type = "CURVE_RIGHT"
        elif lidar_pattern == 'open':
            road_type = "JUNCTION"
            
        return road_type
    
    def recommend_control(self, road_type):
        """Buat rekomendasi kontrol berdasarkan tipe jalan"""
        # Default values
        steering = 0.0
        throttle = 0.3
        
        if road_type == "STRAIGHT":
            # Jalan lurus: kemudi minimal, gas normal
            steering = 0.0
            throttle = 0.4
        elif road_type == "CURVE_LEFT":
            # Belok kiri: kemudi negatif, gas rendah
            steering = -0.3
            throttle = 0.25
        elif road_type == "CURVE_RIGHT":
            # Belok kanan: kemudi positif, gas rendah
            steering = 0.3
            throttle = 0.25
        elif road_type == "JUNCTION":
            # Persimpangan: gas rendah, steering tergantung target waypoint
            throttle = 0.2
            
            # Gunakan waypoint jika tersedia untuk menentukan arah di persimpangan
            if self.current_waypoint and self.vehicle_pose:
                vehicle_x = self.vehicle_pose.position.x
                vehicle_y = self.vehicle_pose.position.y
                target_x = self.current_waypoint.position.x
                target_y = self.current_waypoint.position.y
                
                # Hitung arah ke waypoint dalam koordinat kendaraan
                dx = target_x - vehicle_x
                dy = target_y - vehicle_y
                
                # Transformasi ke frame kendaraan
                cos_yaw = math.cos(-self.vehicle_yaw)
                sin_yaw = math.sin(-self.vehicle_yaw)
                target_x_local = dx * cos_yaw - dy * sin_yaw
                target_y_local = dx * sin_yaw + dy * cos_yaw
                
                # Hitung sudut kemudi
                steering = math.atan2(target_y_local, target_x_local) * 0.5
                
        # Batas nilai
        steering = max(-0.5, min(0.5, steering))
        throttle = max(0.1, min(0.5, throttle))
        
        return steering, throttle
        
    def analyze_road(self):
        """Analisis jalan dan publikasikan rekomendasi"""
        if self.vehicle_pose is None or self.processed_lidar is None:
            return
            
        # Hitung arah jalur dari riwayat
        path_direction = self.calculate_path_direction()
        self.direction_history.append(path_direction)
        
        # Publish arah jalur
        dir_msg = Vector3()
        dir_msg.x = float(path_direction[0])
        dir_msg.y = float(path_direction[1])
        dir_msg.z = 0.0
        self.road_direction_pub.publish(dir_msg)
        
        # Deteksi tipe jalan
        road_type = self.detect_road_type()
        
        # Publish tipe jalan
        type_msg = String()
        type_msg.data = road_type
        self.road_type_pub.publish(type_msg)
        
        # Buat rekomendasi kontrol
        steering, throttle = self.recommend_control(road_type)
        
        # Publish rekomendasi
        steer_msg = Float32()
        steer_msg.data = float(steering)
        self.recommended_steering_pub.publish(steer_msg)
        
        throttle_msg = Float32()
        throttle_msg.data = float(throttle)
        self.recommended_throttle_pub.publish(throttle_msg)
        
        # Logging
        self.get_logger().info(
            f"Road type: {road_type}, "
            f"Recommended steering: {steering:.2f}, "
            f"Recommended throttle: {throttle:.2f}, "
            f"Current speed: {self.vehicle_speed:.1f} m/s"
        )
        
        # Simpan data pelatihan
        if self.training_mode and self.processed_lidar:
            sample = {
                'lidar': self.processed_lidar,
                'pose': {
                    'x': self.vehicle_pose.position.x,
                    'y': self.vehicle_pose.position.y,
                    'yaw': self.vehicle_yaw,
                    'speed': self.vehicle_speed
                },
                'path_direction': path_direction,
                'road_type': road_type,
                'steering': steering,
                'throttle': throttle,
                'timestamp': self.get_clock().now().to_msg().sec
            }
            
            self.training_samples.append(sample)
            
            # Batasi jumlah sampel
            if len(self.training_samples) > self.max_samples:
                self.training_samples = self.training_samples[-self.max_samples:]
    
    def save_training_data(self):
        """Simpan data pelatihan ke file"""
        if not self.training_samples:
            return
            
        try:
            filename = os.path.join(self.model_dir, f'road_data_{len(self.training_samples)}.pkl')
            with open(filename, 'wb') as f:
                pickle.dump(self.training_samples, f)
                
            self.get_logger().info(f"Saved {len(self.training_samples)} training samples to {filename}")
        except Exception as e:
            self.get_logger().error(f"Error saving training data: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = RoadLearner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()