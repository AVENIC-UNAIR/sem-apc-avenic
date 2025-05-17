#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Vector3
from std_msgs.msg import String, Int32, Bool, Float32
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np
from collections import deque

class WaypointProcessor(Node):
    def __init__(self):
        super().__init__('waypoint_processor')
        
        # Subscriber untuk mendapatkan posisi kendaraan
        self.odom_sub = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odom_callback,
            10)
            
        # Subscriber untuk LiDAR data
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/carla/ego_vehicle/vlp16_1',
            self.lidar_callback,
            10)
            
        # Publisher untuk waypoint dan path
        self.current_waypoint_pub = self.create_publisher(
            PoseStamped, 
            '/current_waypoint', 
            10)
            
        self.next_waypoint_pub = self.create_publisher(
            PoseStamped, 
            '/next_waypoint', 
            10)
            
        self.path_pub = self.create_publisher(
            Path, 
            '/planned_path', 
            10)
            
        self.waypoint_index_pub = self.create_publisher(
            Int32, 
            '/current_waypoint_index', 
            10)
            
        self.status_pub = self.create_publisher(
            String, 
            '/waypoint_status', 
            10)
            
        # TAMBAHAN: Publisher untuk info navigasi
        self.navigation_info_pub = self.create_publisher(
            String,
            '/navigation_info',
            10)
            
        self.road_direction_pub = self.create_publisher(
            Vector3,
            '/road_direction',
            10)

        self.is_junction_pub = self.create_publisher(
            Bool,
            '/is_junction',
            10)
            
        self.recommended_speed_pub = self.create_publisher(
            Float32,
            '/recommended_speed',
            10)
        
        # Posisi kendaraan dan state
        self.vehicle_pose = None
        self.vehicle_speed = 0.0
        self.vehicle_yaw = 0.0
        
        # Data LiDAR terbaru
        self.latest_lidar_points = None
        
        # Informasi peta Town01 (persimpangan utama)
        self.junction_points = [
            [88.384346, -287.468567],    # Persimpangan di waypoint 7
            [177.594101, -326.386902],   # Persimpangan di waypoint 8
            [-1.646942, -197.501282],    # Persimpangan di waypoint 9
            [161.030975, -129.313187]    # Persimpangan di waypoint 12
        ]
        self.junction_radius = 20.0  # radius area persimpangan (meter)
        
        # Arah jalan yang direkam
        self.road_direction_history = deque(maxlen=5)
        
        # Daftar waypoint berdasarkan input
        self.waypoints = [
            [334.949799, -161.106171, 0.001736],  # 0
            [339.100037, -258.568939, 0.001679],  # 1
            [396.295319, -183.195740, 0.001678],  # 2
            [267.657074, -1.983160, 0.001678],    # 3
            [153.868896, -26.115866, 0.001678],   # 4
            [290.515564, -56.175072, 0.001677],   # 5
            [92.325722, -86.063644, 0.001677],    # 6
            [88.384346, -287.468567, 0.001728],   # 7 - persimpangan
            [177.594101, -326.386902, 0.001677],  # 8 - persimpangan
            [-1.646942, -197.501282, 0.001555],   # 9 - persimpangan
            [59.701321, -1.970804, 0.001467],     # 10
            [122.100121, -55.142044, 0.001596],   # 11
            [161.030975, -129.313187, 0.001679],  # 12 - persimpangan
            [184.758713, -199.424271, 0.001680]   # 13
        ]
        
        # Arah jalan untuk setiap segmen
        self.road_directions = self.calculate_road_directions()
        
        # Kecepatan yang direkomendasikan untuk setiap waypoint
        self.recommended_speeds = self.calculate_recommended_speeds()
        
        # Indeks waypoint saat ini
        self.current_waypoint_idx = 0
        
        # Parameter untuk mengetahui kapan waypoint telah dicapai
        self.waypoint_threshold = 10.0  # meter
        
        # Timer untuk update waypoint
        self.create_timer(0.1, self.update_waypoint)  # 10 Hz
        
        # Timer untuk publish path (lebih jarang, path tidak sering berubah)
        self.create_timer(1.0, self.publish_path)  # 1 Hz
        
        # Timer baru untuk analisis environment
        self.create_timer(0.2, self.process_environment)  # 5 Hz
        
        # Status inisialisasi
        self.get_logger().info("Advanced Waypoint Processor initialized for Town01")
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints with environment understanding")
        
        # Publikasikan path awal
        self.publish_path()
    
    def odom_callback(self, msg):
        """Update posisi kendaraan berdasarkan odometry"""
        self.vehicle_pose = msg.pose.pose
        
        # Update kecepatan
        self.vehicle_speed = math.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2
        )
        
        # Update orientasi
        q = self.vehicle_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.vehicle_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def lidar_callback(self, msg):
        """Process data LiDAR untuk environment sensing"""
        try:
            # Konversi PointCloud2 ke array numpy
            points = []
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                points.append(p)
            
            if points:
                self.latest_lidar_points = np.array(points)
        except Exception as e:
            self.get_logger().error(f"Error saat memproses LiDAR: {str(e)}")
    
    def process_environment(self):
        """Analisis lingkungan dan navigasi berdasarkan posisi dan data LiDAR"""
        if self.vehicle_pose is None:
            return
            
        # Dapatkan posisi kendaraan
        vx = self.vehicle_pose.position.x
        vy = self.vehicle_pose.position.y
        
        # Periksa apakah kendaraan berada di persimpangan
        is_junction = False
        for junction in self.junction_points:
            distance = math.sqrt((vx - junction[0])**2 + (vy - junction[1])**2)
            if distance < self.junction_radius:
                is_junction = True
                break
                
        # Publish status persimpangan
        junction_msg = Bool()
        junction_msg.data = is_junction
        self.is_junction_pub.publish(junction_msg)
        
        # Dapatkan arah jalan saat ini
        if self.current_waypoint_idx < len(self.road_directions):
            current_direction = self.road_directions[self.current_waypoint_idx]
            
            # Simpan ke history untuk smoothing
            self.road_direction_history.append(current_direction)
            
            # Hitung rata-rata arah
            avg_dir = [0.0, 0.0]
            for dir in self.road_direction_history:
                avg_dir[0] += dir[0]
                avg_dir[1] += dir[1]
                
            if len(self.road_direction_history) > 0:
                avg_dir[0] /= len(self.road_direction_history)
                avg_dir[1] /= len(self.road_direction_history)
                
                # Normalisasi
                mag = math.sqrt(avg_dir[0]**2 + avg_dir[1]**2)
                if mag > 0:
                    avg_dir[0] /= mag
                    avg_dir[1] /= mag
            
            # Publish arah jalan
            dir_msg = Vector3()
            dir_msg.x = float(avg_dir[0])
            dir_msg.y = float(avg_dir[1])
            dir_msg.z = 0.0
            self.road_direction_pub.publish(dir_msg)
        
        # Publish kecepatan yang direkomendasikan
        if self.current_waypoint_idx < len(self.recommended_speeds):
            speed_msg = Float32()
            speed_msg.data = float(self.recommended_speeds[self.current_waypoint_idx])
            self.recommended_speed_pub.publish(speed_msg)
        
        # Analisis kondisi navigasi
        nav_info = "STRAIGHT_ROAD"
        
        if is_junction:
            # Di persimpangan, analisis arah belok
            if self.current_waypoint_idx < len(self.waypoints) - 1:
                curr_wp = self.waypoints[self.current_waypoint_idx]
                next_wp = self.waypoints[self.current_waypoint_idx + 1]
                
                # Hitung arah jalan saat ini
                road_heading = math.atan2(avg_dir[1], avg_dir[0])
                
                # Hitung arah ke waypoint berikutnya
                next_heading = math.atan2(
                    next_wp[1] - curr_wp[1],
                    next_wp[0] - curr_wp[0]
                )
                
                # Hitung perbedaan sudut
                angle_diff = next_heading - road_heading
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                angle_diff_deg = math.degrees(angle_diff)
                
                # Tentukan arah belok
                if abs(angle_diff_deg) < 30:
                    nav_info = "JUNCTION:STRAIGHT"
                elif angle_diff_deg > 30:
                    if angle_diff_deg > 150:
                        nav_info = "JUNCTION:U_TURN_RIGHT"
                    elif angle_diff_deg > 75:
                        nav_info = "JUNCTION:RIGHT_SHARP"
                    else:
                        nav_info = "JUNCTION:RIGHT"
                else:  # angle_diff_deg < -30
                    if angle_diff_deg < -150:
                        nav_info = "JUNCTION:U_TURN_LEFT"
                    elif angle_diff_deg < -75:
                        nav_info = "JUNCTION:LEFT_SHARP"
                    else:
                        nav_info = "JUNCTION:LEFT"
        else:
            # Jika berada di jalan, deteksi jenis jalur
            if len(self.road_direction_history) > 1:
                first_dir = self.road_direction_history[0]
                last_dir = self.road_direction_history[-1]
                
                # Hitung perubahan arah menggunakan dot product
                dot_product = first_dir[0]*last_dir[0] + first_dir[1]*last_dir[1]
                
                if dot_product < 0.9:  # ada perubahan arah
                    cross_product = first_dir[0]*last_dir[1] - first_dir[1]*last_dir[0]
                    if cross_product > 0:
                        nav_info = "ROAD:CURVE_LEFT"
                    else:
                        nav_info = "ROAD:CURVE_RIGHT"
                        
        # Publish info navigasi
        nav_msg = String()
        nav_msg.data = nav_info
        self.navigation_info_pub.publish(nav_msg)
        
    def calculate_road_directions(self):
        """Hitung arah jalan untuk setiap segmen waypoint"""
        directions = []
        
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            
            # Hitung vektor arah
            dx = wp2[0] - wp1[0]
            dy = wp2[1] - wp1[1]
            
            # Normalisasi
            magnitude = math.sqrt(dx*dx + dy*dy)
            if magnitude > 0:
                dx /= magnitude
                dy /= magnitude
                
            directions.append([dx, dy])
            
        # Untuk waypoint terakhir, gunakan arah yang sama dengan waypoint sebelumnya
        if directions:
            directions.append(directions[-1])
        else:
            directions.append([1.0, 0.0])  # default
            
        return directions
        
    def calculate_recommended_speeds(self):
        """Hitung kecepatan yang direkomendasikan untuk setiap waypoint"""
        speeds = []
        
        for i in range(len(self.waypoints)):
            # Default speed
            speed = 40.0  # km/h
            
            # Periksa jika di persimpangan
            vx = self.waypoints[i][0]
            vy = self.waypoints[i][1]
            
            for junction in self.junction_points:
                distance = math.sqrt((vx - junction[0])**2 + (vy - junction[1])**2)
                if distance < self.junction_radius:
                    speed = 15.0  # km/h, lebih lambat di persimpangan
                    break
            
            # Periksa jika berada di tikungan tajam
            if i > 0 and i < len(self.waypoints) - 1:
                prev_wp = self.waypoints[i-1]
                curr_wp = self.waypoints[i]
                next_wp = self.waypoints[i+1]
                
                # Hitung vektor arah
                v1 = [curr_wp[0] - prev_wp[0], curr_wp[1] - prev_wp[1]]
                v2 = [next_wp[0] - curr_wp[0], next_wp[1] - curr_wp[1]]
                
                # Normalisasi
                mag1 = math.sqrt(v1[0]**2 + v1[1]**2)
                mag2 = math.sqrt(v2[0]**2 + v2[1]**2)
                
                if mag1 > 0 and mag2 > 0:
                    v1[0] /= mag1
                    v1[1] /= mag1
                    v2[0] /= mag2
                    v2[1] /= mag2
                    
                    # Hitung dot product (kosinus sudut)
                    dot_product = v1[0]*v2[0] + v1[1]*v2[1]
                    
                    # Sudut belokan besar (dot kecil) = kecepatan rendah
                    if dot_product < 0.7:  # > ~45 derajat
                        speed = 20.0  # km/h
                    elif dot_product < 0.9:  # > ~25 derajat
                        speed = 30.0  # km/h
            
            speeds.append(speed)
            
        return speeds
        
    def update_waypoint(self):
        """Update dan publikasikan waypoint terbaru"""
        if self.vehicle_pose is None:
            return
            
        # Cek jika sudah mencapai semua waypoint
        if self.current_waypoint_idx >= len(self.waypoints):
            status_msg = String()
            status_msg.data = "ALL_WAYPOINTS_REACHED"
            self.status_pub.publish(status_msg)
            return
            
        # Cek jika waypoint saat ini telah dicapai
        current_wp = self.waypoints[self.current_waypoint_idx]
        distance = self.calculate_distance(
            self.vehicle_pose.position.x, 
            self.vehicle_pose.position.y,
            current_wp[0], 
            current_wp[1]
        )
        
        # Jika sudah dekat dengan waypoint, pindah ke waypoint berikutnya
        if distance < self.waypoint_threshold:
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_idx} reached! "
                f"Position: ({current_wp[0]:.1f}, {current_wp[1]:.1f})"
            )
            self.current_waypoint_idx += 1
            
            # Cek lagi apakah semua waypoint sudah tercapai
            if self.current_waypoint_idx >= len(self.waypoints):
                status_msg = String()
                status_msg.data = "ALL_WAYPOINTS_REACHED"
                self.status_pub.publish(status_msg)
                return
                
            status_msg = String()
            status_msg.data = f"WAYPOINT_CHANGED:{self.current_waypoint_idx}"
            self.status_pub.publish(status_msg)
        
        # Publikasikan current waypoint
        self.publish_current_waypoint()
        
        # Publikasikan next waypoint jika ada
        if self.current_waypoint_idx + 1 < len(self.waypoints):
            self.publish_next_waypoint()
            
        # Publikasikan indeks waypoint saat ini
        index_msg = Int32()
        index_msg.data = self.current_waypoint_idx
        self.waypoint_index_pub.publish(index_msg)
    
    def publish_current_waypoint(self):
        """Publikasikan waypoint saat ini ke topic"""
        if self.current_waypoint_idx >= len(self.waypoints):
            return
            
        waypoint_msg = PoseStamped()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = "map"
        
        wp = self.waypoints[self.current_waypoint_idx]
        waypoint_msg.pose.position.x = float(wp[0])
        waypoint_msg.pose.position.y = float(wp[1])
        waypoint_msg.pose.position.z = float(wp[2])
        
        # Quaternion identity (tidak ada rotasi)
        waypoint_msg.pose.orientation.w = 1.0
        waypoint_msg.pose.orientation.x = 0.0
        waypoint_msg.pose.orientation.y = 0.0
        waypoint_msg.pose.orientation.z = 0.0
        
        self.current_waypoint_pub.publish(waypoint_msg)
    
    def publish_next_waypoint(self):
        """Publikasikan waypoint berikutnya ke topic"""
        next_idx = self.current_waypoint_idx + 1
        if next_idx >= len(self.waypoints):
            return
            
        waypoint_msg = PoseStamped()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = "map"
        
        wp = self.waypoints[next_idx]
        waypoint_msg.pose.position.x = float(wp[0])
        waypoint_msg.pose.position.y = float(wp[1])
        waypoint_msg.pose.position.z = float(wp[2])
        
        # Quaternion identity (tidak ada rotasi)
        waypoint_msg.pose.orientation.w = 1.0
        waypoint_msg.pose.orientation.x = 0.0
        waypoint_msg.pose.orientation.y = 0.0
        waypoint_msg.pose.orientation.z = 0.0
        
        self.next_waypoint_pub.publish(waypoint_msg)
    
    def publish_path(self):
        """Publikasikan seluruh path ke topic"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        # Tambahkan semua waypoint ke path
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = float(wp[2])
            
            # Quaternion identity (tidak ada rotasi)
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def calculate_distance(self, x1, y1, x2, y2):
        """Hitung jarak Euclidean antara dua titik"""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 