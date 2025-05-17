# Shell Eco-marathon APC Project - AVENIC UNAIR

This is the AVENIC UNAIR development repository for the Shell Eco-marathon Autonomous Programming Competition (APC) using ROS 2 Humble and CARLA Simulator.

---

## 📁 Project Structure

```
sem-apc-example-project/
├── images/                  # Ignore
├── shell_simulation/
│   ├── CMakeLists.txt       # Edit this if you add new nodes or dependencies
│   ├── launch/
│   │   └── shell_simulation.launch.py
│   ├── Non-Used/            # References (not used in build)
│   ├── package.xml          # Edit this if you add new ROS2 dependencies
│   ├── scripts/             # All Python nodes go here
│   │   └── example_control.py
│   ├── shell_simulation/    # Internal files - ignore
│   └── src/                 # Optional C++ nodes (we use Python only)
│       └── example_control.cpp
├── LICENSE
└── README.md
```

---

## 🧠 Project Guidelines (Wajib Baca)

### ✅ Struktur dan Workflow

- **scripts/**: Tempat semua node Python untuk kontrol kendaraan. Di sinilah kamu ngatur publisher/subscriber.
- **launch/**: Untuk file `launch.py` yang menjalankan sistem (jadi sistemnya kaya launcher daripada node yang ada dan yang kamu list disitu).
- **CMakeLists.txt & package.xml**: Harus selalu diupdate saat nambah node atau dependencies baru.
- **Non-Used/**: Referensi aja, gak kepakai saat build.

### 🚫 Hal yang Harus Dihindari

1. **Jangan pakai dependency eksternal** yang tidak standar di ROS atau tidak bisa diinstall via `rosdep`.
   - Misal error seperti `No module named ...` artinya package itu gak ada di build environment Shell dan harus dibuang/manual install (yang *tidak boleh*).
   - Kalau buat training di local/sendiri gapapa asal jangan di panggil saat run node di kode yang akan diupload
2. **Usahakan gunakan hanya topic ROS2 yang disediakan oleh Shell** (lihat bagian bawah).
3. Hindari edit sembarangan file `bridge`/`interface` yang disediakan Shell.
4. Bakal update lagi kalo ada...

### ✅ Hal Wajib Dilakukan

1. **Selalu jalankan `colcon build`** setelah menambah/mengubah file dalam `shell_simulation/`:
   ```bash
   colcon build
   source install/setup.bash
   ```
2. Kalau abis run node kan ngaco tuh mobilnya kita restart aja simulasi dengan `Ctrl+C` lalu:
   ```bash
   ros2 launch carla_shell_bridge main.launch.py
   ```
4. Buat run launcher node kamu jalankan dengan perintah ini:
   ```bash
   ros2 launch shell_simulation nama_launcher_kamu.py
   ```
5. Buat run salah satu node kamu jalankan dengan perintah ini:
   ```bash
   ros2 run shell_simulation nama_node_kamu.py
   ```
5. Gunakan **RViz** untuk memvisualisasi data:
   - Publisher: Node yang mempublikasikan data ke topic.
   - Subscriber: Node yang menerima data dari topic.
   - ROS2 akan otomatis menghubungkan keduanya via topic.
6. Gunakan GPT (dengan bijak!) buat belajar, debug, dan optimasi code.

---

## 🚀 Running the Simulation

Setelah environment Docker siap, ikuti langkah ini:

```bash
docker start carla_server ros_environment
docker exec -it ros_environment /bin/bash

cd ~/shell_ws
colcon build
source install/setup.bash

ros2 launch carla_shell_bridge main.launch.py
# Buka terminal baru (jika lokal, cukup buka tab baru)
ros2 run shell_simulation example_control
```

---

## 🗺️ Official Map & Waypoints

Map: `Town_01`  
![2025_map](images/2025_season_map.png)

**Start Pose** `[x, y, z, roll, pitch, yaw]`:
```text
[280.363739, -129.306351, 0.101746, 0.00, 0.00, 180.00]
```

**Goal Waypoints** `[x, y, z]`:
```
[334.949799, -161.106171, 0.001736]
[339.100037, -258.568939, 0.001679]
[396.295319, -183.195740, 0.001678]
[267.657074, -1.983160, 0.001678]
[153.868896, -26.115866, 0.001678]
[290.515564, -56.175072, 0.001677]
[92.325722, -86.063644, 0.001677]
[88.384346, -287.468567, 0.001728]
[177.594101, -326.386902, 0.001677]
[-1.646942, -197.501282, 0.001555]
[59.701321, -1.970804, 0.001467]
[122.100121, -55.142044, 0.001596]
[161.030975, -129.313187, 0.001679]
[184.758713, -199.424271, 0.001680]
```

---

## 📡 Available ROS Topics

### Published by Simulator:
```
/carla/ego_vehicle/collision         [carla_msgs/CarlaCollisionEvent]
/carla/ego_vehicle/depth_middle/image [sensor_msgs/Image]
/carla/ego_vehicle/gnss              [sensor_msgs/NavSatFix]
/carla/ego_vehicle/imu               [sensor_msgs/Imu]
/carla/ego_vehicle/lane_invasion     [carla_msgs/CarlaLaneInvasionEvent]
/carla/ego_vehicle/odometry          [nav_msgs/Odometry]
/carla/ego_vehicle/speedometer       [std_msgs/Float32]
/carla/ego_vehicle/rgb_front         [sensor_msgs/Image]
/carla/ego_vehicle/vehicle_status    [carla_msgs/CarlaEgoVehicleStatus]
/carla/ego_vehicle/vlp16_1           [sensor_msgs/PointCloud2]
/tf                                  [tf2_msgs/TFMessage]
```

### Command Topics:
```
/throttle_command       [std_msgs/Float64]  # 0.0 - 1.0
/brake_command          [std_msgs/Float64]  # 0.0 - 1.0
/steering_command       [std_msgs/Float64]  # -1.0 (left) to 1.0 (right)
/gear_command           [std_msgs/String]   # "forward" or "reverse"
/handbrake_command      [std_msgs/Bool]     # true / false
```
