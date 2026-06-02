# FAST-LIO2 ROS 2 NEW — Step-by-Step Data, Multi-threading, Math & Visualization Guide

> Tài liệu này được viết cho workspace `FAST_LIO2_ROS2_NEW`, dựa trực tiếp trên source code hiện có trong `FAST_LIO/`, `HeLiPR-File-Player-ROS2/` và paper `FAST_LIO/doc/Fast_LIO_2.pdf`. Mục tiêu là học FAST-LIO2 như một **đường ống dữ liệu đa làn**: dữ liệu ROS 2 đổ vào → callback/timer/luồng nền điều phối → mô hình toán biến đổi dữ liệu → publisher ROS 2 đổ kết quả ra RViz/bag/CSV.

## Phần 1: Kiến trúc Hệ thống ROS 2, Luồng Xử lý & Thiết kế Đa luồng

### 1.1. Sơ đồ khối giao tiếp ROS 2

Trong workspace này có hai khối chạy thực tế thường gặp:

- `helipr_file_player`: node phát lại dữ liệu HeLiPR/City02 từ file, có GUI Qt và nhiều thread con để đọc/publish từng sensor.
- `fastlio_mapping`: node FAST-LIO2 chính trong package `fast_lio`, tên node C++ là `laser_mapping`.

```text
                           ┌──────────────────────────────────────────────┐
                           │ HeLiPR File Player / dataset publisher        │
                           │ package: helipr_file_player                   │
                           │ executable: helipr_file_player                │
                           │ ROS node: helipr_file_player_node             │
                           └──────────────────────────────────────────────┘
        /clock             rosgraph_msgs/msg/Clock        │
        /imu/data_raw      sensor_msgs/msg/Imu            │
        /ouster/points     sensor_msgs/msg/PointCloud2    │  Ouster path
        /velodyne/points   sensor_msgs/msg/PointCloud2    │  Velodyne path
        /aeva/points       sensor_msgs/msg/PointCloud2    │  Aeva path
        /avia/points       livox_ros_driver2/msg/CustomMsg│  Livox Avia path
        /inspva            novatel_gps_msgs/msg/Inspva    │  GNSS/INS debug input
                                                           ▼
┌────────────────────────────────────────────────────────────────────────────────┐
│ FAST-LIO2 ROS 2 node                                                           │
│ package: fast_lio                                                              │
│ executable: fastlio_mapping                                                    │
│ C++ class/node name: LaserMappingNode("laser_mapping")                         │
│ launched by: ros2 launch fast_lio mapping.launch.py                            │
│ parameters: config/*.yaml, e.g. config/city02.yaml                             │
│                                                                                │
│ Subscriptions                                                                  │
│   common.lid_topic: /ouster/points or /avia/points                              │
│   common.imu_topic: /imu/data_raw                                               │
│                                                                                │
│ Internal pipeline                                                              │
│   LiDAR callback → Preprocess → lidar_buffer/time_buffer                        │
│   IMU callback   → timestamp correction → imu_buffer                            │
│   100 Hz timer   → sync_packages → IMU propagation + undistortion               │
│                  → iterated ESKF LiDAR update → ikd-Tree map update             │
│                                                                                │
│ Publishers                                                                     │
│   /Odometry               nav_msgs/msg/Odometry                                 │
│   /path                   nav_msgs/msg/Path                                     │
│   /cloud_registered       sensor_msgs/msg/PointCloud2                           │
│   /cloud_registered_body  sensor_msgs/msg/PointCloud2                           │
│   /cloud_effected         sensor_msgs/msg/PointCloud2                           │
│   /Laser_map              sensor_msgs/msg/PointCloud2                           │
│   TF camera_init → body                                                         │
│                                                                                │
│ Service                                                                        │
│   /map_save               std_srvs/srv/Trigger                                  │
└────────────────────────────────────────────────────────────────────────────────┘
                                                           │
                                                           ▼
                           ┌──────────────────────────────────────────────┐
                           │ RViz2 / ros2 bag / CSV plotting tools        │
                           │ /Odometry, /path, /cloud_registered          │
                           └──────────────────────────────────────────────┘
```

Các topic input/output trên được tạo trực tiếp trong constructor `LaserMappingNode`: Livox Avia dùng subscription `livox_ros_driver2::msg::CustomMsg`, các LiDAR còn lại dùng `sensor_msgs::msg::PointCloud2`, IMU dùng `sensor_msgs::msg::Imu`, còn output gồm `/cloud_registered`, `/cloud_registered_body`, `/cloud_effected`, `/Laser_map`, `/Odometry`, `/path` và TF broadcaster. `FAST_LIO/src/laserMapping.cpp` lines 920-936. Launch file tạo executable `fastlio_mapping` với config YAML và RViz tùy chọn. `FAST_LIO/launch/mapping.launch.py` lines 46-68. File `city02.yaml` ánh xạ Ouster City02 vào `/ouster/points`, `/imu/data_raw`, `lidar_type: 3`, `scan_line: 128`, `timestamp_unit: 3`. `FAST_LIO/config/city02.yaml` lines 3-16.

### 1.2. Sơ đồ phối hợp đa luồng và đồng bộ hóa

#### 1.2.1. FAST-LIO2 node: callback/timer + buffer mutex + ikd-Tree rebuild thread

Trong `fastlio_mapping`, code **không tự tạo `std::thread` cho callback ROS**; nó gọi `rclcpp::spin(std::make_shared<LaserMappingNode>())`, tức executor ROS 2 mặc định sẽ điều phối callback/timer của node. `FAST_LIO/src/laserMapping.cpp` lines 1157-1164. Tuy vậy, node có hai lớp concurrency quan trọng:

1. **Callback/timer ROS 2 chia sẻ buffer**: LiDAR callback và IMU callback ghi vào `lidar_buffer`, `time_buffer`, `imu_buffer`; timer 100 Hz đọc/pop các buffer qua `sync_packages()`.
2. **ikd-Tree có thread nền riêng**: `KD_TREE::start_thread()` tạo `pthread_create(&rebuild_thread, ..., multi_thread_ptr, this)` để rebuild cây song song với thread mapping chính. `FAST_LIO/include/ikd-Tree/ikd_Tree.cpp` lines 167-177.

```text
FAST-LIO2 / laser_mapping

ROS callback lane A: LiDAR callback
  standard_pcl_cbk(sensor_msgs::msg::PointCloud2::UniquePtr)
  hoặc livox_pcl_cbk(livox_ros_driver2::msg::CustomMsg::UniquePtr)
        │
        │ lock mtx_buffer
        ▼
  p_pre->process(...)  ──► PointCloudXYZI::Ptr ptr
  lidar_buffer.push_back(ptr)
  time_buffer.push_back(lidar time)
        │ unlock mtx_buffer
        ▼
  sig_buffer.notify_all()

ROS callback lane B: IMU callback
  imu_cbk(sensor_msgs::msg::Imu::UniquePtr)
        │ timestamp correction by time_offset_lidar_to_imu / time_sync_en
        │ lock mtx_buffer
        ▼
  imu_buffer.push_back(msg)
        │ unlock mtx_buffer
        ▼
  sig_buffer.notify_all()

ROS timer lane C: 100 Hz mapping timer
  timer_callback()
        │
        ▼
  sync_packages(Measures)
        ├─ reads front lidar scan
        ├─ waits logically until last_timestamp_imu >= lidar_end_time
        ├─ pops IMUs whose time <= lidar_end_time
        └─ pops one lidar scan
        │
        ▼ sequential math pipeline per scan
  p_imu->Process(Measures, kf, feats_undistort)
  lasermap_fov_segment()
  voxel downsample
  kf.update_iterated_dyn_share_modified(...)
  map_incremental()
  publish_odometry/path/clouds

Background lane D inside ikd-Tree
  pthread rebuild_thread
        │
        ├─ waits/polls Rebuild_Ptr
        ├─ locks rebuild_ptr_mutex_lock + working_flag_mutex
        ├─ locks search_flag_mutex while copying/replacing subtree
        ├─ logs missed operations with rebuild_logger_mutex_lock
        └─ unlocks so main mapping can continue nearest search/update safely
```

**Điểm khóa trong FAST-LIO2 node:**

- `mtx_buffer` và `sig_buffer` là biến global của `laserMapping.cpp`. `mtx_buffer` bảo vệ `lidar_buffer`, `time_buffer`, `imu_buffer`, `last_timestamp_lidar`, `last_timestamp_imu`; `sig_buffer` chỉ được `notify_all()` trong callback và signal handler, nhưng trong ROS 2 code hiện tại **không có `std::unique_lock` + `sig_buffer.wait(...)` ở timer**. `FAST_LIO/src/laserMapping.cpp` lines 84-109, 283-306, 311-347, 350-378.
- `sync_packages()` hiện **không tự lock `mtx_buffer` bên trong hàm**; vì vậy nếu executor được đổi sang `MultiThreadedExecutor`, người học cần bọc `sync_packages()` bằng `std::unique_lock<std::mutex>` hoặc chuyển buffer sang một lớp thread-safe riêng. `FAST_LIO/src/laserMapping.cpp` lines 383-433.
- Mapping callback chạy tuần tự theo từng scan: sau khi sync được một `MeasureGroup`, nó gọi IMU propagation/undistortion, FOV segmentation, downsample, ESKF update, incremental map update, rồi publish. `FAST_LIO/src/laserMapping.cpp` lines 958-1076.
- `h_share_model()` có nhánh OpenMP `#pragma omp parallel for` nếu macro `MP_EN` được bật; vùng này song song hóa tìm nearest surface/residual theo từng point. `FAST_LIO/src/laserMapping.cpp` lines 677-710.

#### 1.2.2. HeLiPR file player: QThread + nhiều worker thread + condition_variable

File player là nguồn dữ liệu ROS 2 trong hướng dẫn City02. Nó dùng `ROSThread` kế thừa `QThread`, tạo node `helipr_file_player_node`, publisher cho `/imu/data_raw`, `/ouster/points`, `/avia/points`, `/aeva/points`, `/velodyne/points`, `/inspva`, `/clock`, rồi chạy `rclcpp::executors::MultiThreadedExecutor`. `HeLiPR-File-Player-ROS2/src/ROSThread.cpp` lines 66-94.

```text
GUI thread (Qt mainwindow)
        │ start/stop/load/seek
        ▼
ROSThread : QThread
        │ creates ROS 2 node + publishers
        │ starts worker std::thread lanes
        ▼
DataStampThread
        ├─ reads stamp.csv timeline
        ├─ updates processed_stamp_ according to wall timer/play_rate_
        └─ notify worker cv_ when a sensor stamp should be emitted

Sensor worker lanes
        ├─ InspvaThread: waits inspva_thread_.cv_, publishes /inspva
        ├─ ImuThread: waits imu_thread_.cv_, publishes /imu/data_raw
        ├─ OusterThread: waits ouster_thread_.cv_, publishes /ouster/points
        ├─ VelodyneThread: waits velodyne_thread_.cv_, publishes /velodyne/points
        ├─ AviaThread: waits avia_thread_.cv_, publishes /avia/points
        ├─ AevaThread: waits aeva_thread_.cv_, publishes /aeva/points
        └─ LivoxTeleThread: waits livox_tele_thread_.cv_, publishes related Livox data

Shared recording path
        bag_mutex_ protects bag write calls when save_flag_ && process_flag_
```

Worker threads được tạo bằng `std::thread(&ROSThread::DataStampThread, this)` và các thread sensor tương ứng. `HeLiPR-File-Player-ROS2/src/ROSThread.cpp` lines 303-311. Mỗi sensor worker dùng `std::unique_lock<std::mutex> ul(sensor_thread_.mutex_)` rồi `sensor_thread_.cv_.wait(ul)`, và khi cần ghi bag thì dùng `std::lock_guard<std::mutex> lock(bag_mutex_)`. `HeLiPR-File-Player-ROS2/src/ROSThread.cpp` lines 562-594, 701-715, 925-940.

> **Gotcha đọc code:** Đây là `std::condition_variable` của file player, không phải condition variable của FAST-LIO2 core. Trong FAST-LIO2 ROS 2 node, `sig_buffer` đang được notify nhưng không được wait trong vòng xử lý hiện tại.

### 1.3. Bảng ánh xạ Toán - Code - Thread/Topic Component

| Ký hiệu/công thức trong paper | Hàm/biến/file trong source code | Thread/topic/component xử lý |
|---|---|---|
| Scan: raw LiDAR points được tích lũy trong khoảng 10-100 ms trước khi đăng ký vào map | `lidar_buffer`, `time_buffer`, `standard_pcl_cbk()`, `livox_pcl_cbk()`, `p_pre->process(...)` | ROS subscription callback nhận `common.lid_topic`, ghi buffer dưới `mtx_buffer`; callback notify `sig_buffer`. `FAST_LIO/src/laserMapping.cpp` lines 283-306, 311-347. |
| Đồng bộ scan k với các IMU tới `lidar_end_time` | `sync_packages(MeasureGroup &meas)`, `lidar_mean_scantime`, `meas.imu`, `meas.lidar_beg_time`, `meas.lidar_end_time` | Timer 100 Hz gọi tuần tự; đọc/pop `lidar_buffer`, `time_buffer`, `imu_buffer`. `FAST_LIO/src/laserMapping.cpp` lines 383-433. |
| State manifold `M = SO(3) × R^15 × SO(3) × R^3`, state gồm pose/velocity/bias/gravity/extrinsic | `MTK_BUILD_MANIFOLD(state_ikfom, pos, rot, offset_R_L_I, offset_T_L_I, vel, bg, ba, grav)` | Core ESKF state trong `use-ikfom.hpp`; dùng bởi `kf`, `ImuProcess`, `h_share_model`, publisher odom. `FAST_LIO/include/use-ikfom.hpp` lines 12-21. |
| Input IMU `u = [ω_m, a_m]`, process noise `w = [n_g, n_a, n_bg, n_ba]` | `input_ikfom(acc, gyro)`, `process_noise_ikfom(ng, na, nbg, nba)`, `process_noise_cov()` | IMU propagation trong timer lane; covariance được set từ YAML qua `set_gyr_cov`, `set_acc_cov`, bias cov. `FAST_LIO/include/use-ikfom.hpp` lines 23-42. |
| Kinematic model: `Rdot`, `pdot=v`, `vdot=R(a-ba)+g`, bias random walk | `get_f(state_ikfom &s, const input_ikfom &in)` | Hàm động học được đăng ký vào ESKF bằng `kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, ...)`. `FAST_LIO/include/use-ikfom.hpp` lines 47-58; `FAST_LIO/src/laserMapping.cpp` lines 903-904. |
| Jacobian state/noise của propagation | `df_dx(...)`, `df_dw(...)` | ESKF prediction trong `kf_state.predict(dt, Q, in)` khi `ImuProcess::UndistortPcl()` chạy. `FAST_LIO/include/use-ikfom.hpp` lines 61-88; `FAST_LIO/src/IMU_Processing.hpp` lines 272-279. |
| IMU initialization: gravity direction, gyro bias, extrinsic initial value | `ImuProcess::IMU_init()`: `init_state.grav`, `init_state.bg`, `init_state.offset_T_L_I`, `init_state.offset_R_L_I` | Các frame đầu trong timer lane; chưa output undistorted cloud cho tới khi đủ `MAX_INI_COUNT`. `FAST_LIO/src/IMU_Processing.hpp` lines 156-211, 347-370. |
| Forward propagation tới scan end time và backward propagation để khử méo point | `ImuProcess::UndistortPcl()`, `IMUpose`, `kf_state.predict(dt, Q, in)`, `P_compensate` | Timer lane, tuần tự theo scan; mỗi point được đưa về frame LiDAR tại scan end-time. `FAST_LIO/src/IMU_Processing.hpp` lines 213-337. |
| Measurement model point-to-plane `0 = u_j^T(T(x) p_j - q_j)` | `h_share_model()`: transform `p_global`, `ikdtree.Nearest_Search`, `esti_plane`, `pd2`, `normvec`, `corr_normvect` | ESKF update gọi nhiều lần; nếu `MP_EN` bật thì nearest/residual loop chạy OpenMP song song theo point. `FAST_LIO/src/laserMapping.cpp` lines 677-732. |
| Residual `z_j = u_j^T(GT IK IT L Lp_j - Gq_j)` | `normvec->points[i].intensity = pd2`; `ekfom_data.h(i) = -norm_p.intensity` | Trong `h_share_model()`, chuyển residual thành vector measurement cho ESKF. `FAST_LIO/src/laserMapping.cpp` lines 719-729, 790-792. |
| Measurement Jacobian `H_j` theo pose và extrinsic | `ekfom_data.h_x.block<1,12>(...)`, các vector `A`, `B`, `C` | Trong `h_share_model()`, phụ thuộc `extrinsic_est_en`; update hiện dùng 12 cột active cho position/rotation/extrinsic. `FAST_LIO/src/laserMapping.cpp` lines 759-788. |
| Iterated Kalman update với Kalman gain dạng nghịch đảo theo state dimension | `kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time)`; trong IKFoM: `(P_/R).inverse()`, `HTH`, `K_h`, `K_x`, `x_.boxplus(dx_)` | Timer lane; mỗi scan có nhiều iteration tới hội tụ hoặc `max_iteration`. `FAST_LIO/src/laserMapping.cpp` lines 1049-1059; `FAST_LIO/include/IKFoM_toolkit/esekfom/esekfom.hpp` lines 1619-1835. |
| Transform scan đã tối ưu sang global frame | `pointBodyToWorld()`, `feats_down_world`, `publish_frame_world()` | Sau ESKF update; dùng để publish `/cloud_registered` và insert map. `FAST_LIO/src/laserMapping.cpp` lines 1005-1011, 1066-1075. |
| Local map sliding cube/FOV delete | `lasermap_fov_segment()`, `cub_needrm`, `ikdtree.Delete_Point_Boxes(cub_needrm)` | Timer lane; xóa vùng map ngoài cube khi pose gần biên. `FAST_LIO/src/laserMapping.cpp` lines 245-280, 991-992. |
| ikd-Tree incremental insertion + on-tree downsample | `map_incremental()`, `ikdtree.Add_Points(PointToAdd, true)`, `ikdtree.Add_Points(PointNoNeedDownsample, false)` | Timer lane gọi update map; ikd-Tree có thread nền rebuild song song. `FAST_LIO/src/laserMapping.cpp` lines 436-484. |
| ikd-Tree parallel rebuild tránh delay khi rebalance | `KD_TREE::start_thread()`, `multi_thread_rebuild()`, `search_flag_mutex`, `working_flag_mutex`, `rebuild_logger_mutex_lock` | Background pthread trong ikd-Tree; đồng bộ với nearest search/update của mapping lane. `FAST_LIO/include/ikd-Tree/ikd_Tree.cpp` lines 167-190, 201-310. |
| ROS output pose/map/path | `publish_odometry()`, `publish_path()`, `publish_frame_world()`, publishers `/Odometry`, `/path`, `/cloud_registered` | Timer lane sau update; `/path` chỉ publish mỗi 10 pose để tránh RViz crash. `FAST_LIO/src/laserMapping.cpp` lines 661-674, 930-935, 1063-1075. |

## Phần 2: Lộ trình 6 Giai đoạn Bóc tách & Thực hành

### Stage 1 — Data ingress: ROS 2 topics, typed point formats, timestamp semantics

#### 1. Giao diện dữ liệu & ROS 2 Mapping

- Input C++ tối giản:
  - `sensor_msgs::msg::PointCloud2` cho Ouster/Velodyne/Aeva/MID360-like path.
  - `livox_ros_driver2::msg::CustomMsg` cho Livox Avia.
  - `sensor_msgs::msg::Imu` cho IMU.
- Output C++ sau preprocess: `PointCloudXYZI::Ptr`, tức `pcl::PointCloud<pcl::PointXYZINormal>::Ptr`; code dùng trường `curvature` như **offset time theo ms** của point trong scan.
- Mapping ROS 2:
  - City02 Ouster: `/ouster/points` → `sensor_msgs/msg/PointCloud2`; `/imu/data_raw` → `sensor_msgs/msg/Imu`.
  - Livox Avia: `/avia/points` → `livox_ros_driver2/msg/CustomMsg`.

#### 2. Bản chất luồng xử lý

- Module này chạy trong ROS subscription callbacks `standard_pcl_cbk()`, `livox_pcl_cbk()`, `imu_cbk()`.
- Callback LiDAR và IMU ghi shared deques dưới `mtx_buffer`, sau đó `sig_buffer.notify_all()`. `FAST_LIO/src/laserMapping.cpp` lines 283-306, 311-347, 350-378.
- Trong cấu hình hiện tại `rclcpp::spin(...)` thường tuần tự hóa callback trong executor mặc định, nhưng code vẫn có mutex để sẵn sàng bảo vệ buffer khi callback xen kẽ hoặc khi người học đổi executor.

#### 3. Kiến thức lý thuyết nền tảng

- Đọc paper phần System Overview: scan là tập point tích lũy trong 10-100 ms, sau đó đăng ký vào map bằng iterated Kalman filter và ikd-Tree.
- Đọc Measurement Model: mỗi point có timestamp khác nhau, phải đưa về scan end-time trước khi dùng point-to-plane residual.

#### 4. File code cốt lõi cần trích xuất

- `FAST_LIO/src/preprocess.h`: định nghĩa `PointType`, `PointCloudXYZI`, enum `LID_TYPE`, `TIME_UNIT`, struct point Ouster/Velodyne/Livox. `FAST_LIO/src/preprocess.h` lines 11-27, 70-111.
- `FAST_LIO/src/preprocess.cpp`: các handler `avia_handler`, `oust64_handler`, `velodyne_handler`, `mid360_handler`, `default_handler`.
- `FAST_LIO/src/laserMapping.cpp`: callback LiDAR/IMU và buffer.
- `FAST_LIO/config/city02.yaml`: thông số City02 Ouster.

#### 5. Hướng dẫn thực hành & trực quan hóa

Tạo một chương trình C++ tối giản `tools/stage1_dump_scan.cpp`:

```cpp
// Pseudocode: đọc PointCloud2 từ rosbag hoặc callback ROS 2,
// chuyển sang pcl::PointCloud<ouster_ros::Point>, rồi dump CSV.
void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<ouster_ros::Point> raw;
  pcl::fromROSMsg(*msg, raw);
  std::ofstream f("scan_points.csv");
  f << "x,y,z,intensity,ring,t_ns\n";
  for (const auto &p : raw.points) {
    f << p.x << ',' << p.y << ',' << p.z << ','
      << p.intensity << ',' << p.ring << ',' << p.t << '\n';
  }
}
```

Sau đó vẽ bằng Python:

```python
import pandas as pd, matplotlib.pyplot as plt
p = pd.read_csv('scan_points.csv')
plt.scatter(p.x, p.y, c=p.t, s=1, cmap='turbo')
plt.axis('equal'); plt.colorbar(label='Ouster t [ns]')
plt.show()
```

Mục tiêu trực quan: màu theo timestamp phải tạo ra “vệt quét” tuần tự quanh scan. Nếu timestamp toàn 0, Stage 2 khử méo sẽ không thể đúng.

#### 6. Sanity Check

- `ros2 topic hz /ouster/points` phải gần `scan_rate` trong YAML, ví dụ 10 Hz cho City02.
- `ros2 topic hz /imu/data_raw` phải cao hơn LiDAR nhiều lần.
- `ros2 topic echo /ouster/points --once` phải có frame/timestamp tăng đều; nếu `time_sync_en: false`, chênh lệch tuyệt đối giữa `last_timestamp_imu` và `last_timestamp_lidar` không nên vượt cảnh báo 10 s trong callback Livox.

### Stage 2 — Time synchronization, scan packaging, and deterministic buffers

#### 1. Giao diện dữ liệu & ROS 2 Mapping

- Input: `deque<PointCloudXYZI::Ptr> lidar_buffer`, `deque<double> time_buffer`, `deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer`.
- Output: `MeasureGroup Measures`, gồm `meas.lidar`, `meas.lidar_beg_time`, `meas.lidar_end_time`, `meas.imu`.

#### 2. Bản chất luồng xử lý

- `sync_packages()` chạy trong `timer_callback()` 100 Hz.
- Hàm kiểm tra có LiDAR/IMU chưa, ước lượng `lidar_end_time` từ `curvature` point cuối hoặc `lidar_mean_scantime`, rồi chỉ trả về `true` khi `last_timestamp_imu >= lidar_end_time`. `FAST_LIO/src/laserMapping.cpp` lines 383-433.
- Vì `sync_packages()` hiện không lock `mtx_buffer`, bản C++ thuần nên chạy single-thread trước; bản ROS 2 production nếu dùng `MultiThreadedExecutor` phải khóa buffer ở cả producer/consumer.

#### 3. Kiến thức lý thuyết nền tảng

- Measurement Model giả định sau khử méo, point trong scan được xem như lấy mẫu đồng thời tại scan end-time.
- Vì vậy `lidar_end_time` là mốc quan trọng: IMU phải phủ tới mốc này trước khi ESKF update.

#### 4. File code cốt lõi cần trích xuất

- `FAST_LIO/src/laserMapping.cpp`: `sync_packages()`, `lidar_mean_scantime`, `lidar_pushed`, callback timestamp correction.
- `FAST_LIO/include/common_lib.h`: định nghĩa `MeasureGroup` và helper thời gian/pose nếu cần.

#### 5. Hướng dẫn thực hành & trực quan hóa

Viết `stage2_sync_replay.cpp` chạy offline:

```cpp
struct MiniMeasure {
  double lidar_beg_time, lidar_end_time;
  std::vector<double> imu_times;
};

// Giả lập: lidar scans 10 Hz, IMU 100 Hz.
// Đẩy timestamps vào deque, gọi sync_packages_minimal(), dump result CSV.
```

Dump CSV:

```text
scan_id,lidar_beg,lidar_end,num_imu,first_imu,last_imu
0,0.00,0.10,11,0.00,0.10
1,0.10,0.20,10,0.11,0.20
```

Vẽ timeline:

```python
import pandas as pd, matplotlib.pyplot as plt
s = pd.read_csv('sync.csv')
plt.hlines(range(len(s)), s.lidar_beg, s.lidar_end, colors='tab:blue', label='LiDAR scan')
plt.scatter(s.last_imu, range(len(s)), color='tab:red', label='last IMU used')
plt.legend(); plt.xlabel('time [s]'); plt.ylabel('scan id'); plt.show()
```

#### 6. Sanity Check

- Mỗi scan phải có `num_imu > 0`; nếu không, `ImuProcess::Process()` return sớm.
- `last_imu >= lidar_end` cho mọi scan.
- Khi loop bag/dataset bị tua ngược, callback phải clear buffer như code xử lý timestamp nhỏ hơn timestamp trước đó.

### Stage 3 — IMU initialization, propagation, and motion undistortion

#### 1. Giao diện dữ liệu & ROS 2 Mapping

- Input C++: `MeasureGroup`, `esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state`, raw scan `PointCloudXYZI`.
- Output C++: `feats_undistort` (`PointCloudXYZI::Ptr`) và state predicted tới `lidar_end_time`.
- ROS mapping: module này không publish trực tiếp; nó nằm trong timer lane trước `/Odometry` và `/cloud_registered`.

#### 2. Bản chất luồng xử lý

- Chạy tuần tự trong `timer_callback()` sau khi `sync_packages()` trả true.
- Không dùng mutex riêng vì chỉ timer lane đụng `kf`, `feats_undistort`, `p_imu`.
- Đây là stage nên giữ single-thread khi học vì lỗi timestamp/bias sẽ tạo drift khó debug.

#### 3. Kiến thức lý thuyết nền tảng

- State transition model paper Eq. (1)-(2): `Rdot`, `pdot`, `vdot`, bias random walk, gravity, extrinsic bất biến.
- Propagation Eq. (6)-(7): state/covariance propagate qua IMU sampling period.
- Backward propagation/motion compensation: mỗi point được chuyển về frame tại scan end-time.

#### 4. File code cốt lõi cần trích xuất

- `FAST_LIO/include/use-ikfom.hpp`: `state_ikfom`, `input_ikfom`, `get_f`, `df_dx`, `df_dw`.
- `FAST_LIO/src/IMU_Processing.hpp`: `IMU_init()`, `UndistortPcl()`, `Process()`.
- `FAST_LIO/src/laserMapping.cpp`: `p_imu->Process(Measures, kf, feats_undistort)`.

#### 5. Hướng dẫn thực hành & trực quan hóa

Viết `stage3_imu_dead_reckoning.cpp`:

1. Đọc CSV IMU gồm `t, ax, ay, az, gx, gy, gz`.
2. Khởi tạo `state_ikfom` giống `IMU_init()`:
   - `grav = -mean_acc / ||mean_acc|| * 9.81`.
   - `bg = mean_gyr`.
3. Gọi `kf.predict(dt, Q, in)` theo từng IMU.
4. Dump `t, px, py, pz, roll, pitch, yaw`.

Vẽ quỹ đạo IMU:

```python
import pandas as pd, matplotlib.pyplot as plt
x = pd.read_csv('imu_pred.csv')
fig = plt.figure(); ax = fig.add_subplot(projection='3d')
ax.plot(x.px, x.py, x.pz)
ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z')
plt.show()
```

Trực quan khử méo:

- Dump `scan_raw.csv` trước `UndistortPcl()`.
- Dump `scan_undistorted.csv` sau `UndistortPcl()`.
- Vẽ XY hai màu; trong chuyển động nhanh, scan đã khử méo phải ít “cong/nhòe” hơn.

#### 6. Sanity Check

- IMU đứng yên: sau init, roll/pitch ổn định, gyro bias gần mean gyro; vị trí không được “bay” quá nhanh trong vài giây đầu.
- Nếu `curvature`/offset time bị sai đơn vị, `lidar_end_time` và vòng `it_pcl->curvature / 1000` trong undistortion sẽ sai; điểm bị kéo méo mạnh.

### Stage 4 — Direct LiDAR residual: nearest search, plane fitting, Jacobian construction

#### 1. Giao diện dữ liệu & ROS 2 Mapping

- Input: `feats_down_body`, state hiện tại `state_ikfom`, map `ikdtree`.
- Output: `ekfom_data.h_x` và `ekfom_data.h` cho ESKF; các point hiệu quả nằm trong `laserCloudOri`, normal/residual trong `corr_normvect`.
- ROS mapping: không publish trực tiếp; nếu bật `effect_pub_en`, point hiệu quả được publish ra `/cloud_effected`.

#### 2. Bản chất luồng xử lý

- `h_share_model()` được gọi bên trong `kf.update_iterated_dyn_share_modified()` nhiều lần trên cùng một scan.
- Có thể song song theo point bằng OpenMP nếu macro `MP_EN` bật; mỗi index `i` ghi vào slot riêng `feats_down_world[i]`, `Nearest_Points[i]`, `point_selected_surf[i]`, `normvec[i]`, `res_last[i]` nên giảm nguy cơ race ở vòng đầu. `FAST_LIO/src/laserMapping.cpp` lines 684-732.
- Sau vòng parallel, code gom point hiệu quả tuần tự vào `laserCloudOri` và `corr_normvect`. `FAST_LIO/src/laserMapping.cpp` lines 734-756.

#### 3. Kiến thức lý thuyết nền tảng

- Measurement Model Eq. (3)-(5): true LiDAR point + noise phải nằm trên plane local trong global map.
- Residual Eq. (8)-(9): tuyến tính hóa implicit measurement thành `z + H dx + v`.
- Cần hiểu biến đổi `p_global = R_GI * (R_IL * p_body + t_IL) + p_GI`.

#### 4. File code cốt lõi cần trích xuất

- `FAST_LIO/src/laserMapping.cpp`: `h_share_model()`, `esti_plane()`, `pointBodyToWorld()`.
- `FAST_LIO/include/ikd-Tree/ikd_Tree.cpp`: `Nearest_Search()` và cơ chế search lock khi rebuild.

#### 5. Hướng dẫn thực hành & trực quan hóa

Viết `stage4_plane_residual.cpp`:

1. Tạo map giả là mặt phẳng `z=0` với noise nhỏ.
2. Tạo scan giả gồm point gần mặt phẳng nhưng pose lệch `z=0.1`.
3. Với mỗi point, tìm 5 nearest bằng PCL KdTree hoặc gọi `KD_TREE`.
4. Fit plane, tính `pd2 = nᵀp + d`.
5. Dump histogram residual.

Python visualization:

```python
import pandas as pd, matplotlib.pyplot as plt
r = pd.read_csv('residual.csv')
plt.hist(r.pd2, bins=100)
plt.xlabel('point-to-plane residual [m]')
plt.ylabel('count')
plt.show()
```

Mục tiêu: nếu pose lệch 0.1 m theo z, residual trung bình phải xấp xỉ 0.1 trước update và giảm sau update.

#### 6. Sanity Check

- `effct_feat_num` không được bằng 0; nếu bằng 0, `ekfom_data.valid = false` và ESKF bỏ update.
- Nearest point thứ 5 có squared distance ≤ 5 theo logic `pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true`.
- `esti_plane(..., 0.1f)` phải pass cho neighborhood thật sự phẳng.

### Stage 5 — Iterated ESKF update and covariance/state correction

#### 1. Giao diện dữ liệu & ROS 2 Mapping

- Input: `kf`, `LASER_POINT_COV`, callback measurement `h_share_model`.
- Output: state tối ưu `state_point = kf.get_x()`, quaternion `geoQuat`, `pos_lid`, covariance trong `kf`.
- ROS mapping: state sau update publish ra `/Odometry`, `/path`, TF `camera_init → body`.

#### 2. Bản chất luồng xử lý

- Chạy tuần tự trong timer lane: `kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time)`.
- Trong mỗi iteration, ESKF gọi lại `h_share_model()` để recompute residual/Jacobian theo state mới.
- Không nên song song hóa update matrix trước khi hiểu rõ memory layout và Eigen thread-safety.

#### 3. Kiến thức lý thuyết nền tảng

- Prior Eq. (10)-(11): map covariance từ propagated state sang current iterate trên manifold qua Jacobian `Jκ`.
- MAP Eq. (13): kết hợp prior và residual point-to-plane.
- Kalman gain Eq. (14): FAST-LIO2 dùng dạng nghịch đảo theo state dimension để tránh nghịch đảo ma trận measurement rất lớn.
- Posterior Eq. (15): cập nhật state/covariance sau hội tụ.

#### 4. File code cốt lõi cần trích xuất

- `FAST_LIO/include/IKFoM_toolkit/esekfom/esekfom.hpp`: `update_iterated_dyn_share_modified()`.
- `FAST_LIO/src/laserMapping.cpp`: `kf.init_dyn_share(...)`, `kf.update_iterated_dyn_share_modified(...)`, `publish_odometry(...)`.
- `FAST_LIO/include/use-ikfom.hpp`: manifold operators thông qua MTK types.

#### 5. Hướng dẫn thực hành & trực quan hóa

Viết bản mini ESKF 2D trước khi dùng full `state_ikfom`:

- State mini: `[x, y, yaw]`.
- Measurement: point-to-line residual trong 2D.
- Iteration:
  1. Transform scan point bằng pose hiện tại.
  2. Fit line/nearest line.
  3. Build `H`, `z`.
  4. Solve `dx = -(HᵀH + P⁻¹)⁻¹(Hᵀz + P⁻¹ dx_prior)`.
  5. Update pose.

Vẽ live bằng Matplotlib:

```python
# plot map lines, raw scan before update, scan after each iteration
# save frames iter_00.png ... iter_N.png để thấy scan "dính" dần vào map
```

Sau đó mới thay mini solver bằng `esekfom` để tránh vừa học manifold vừa học dataflow.

#### 6. Sanity Check

- Norm `dx_` phải giảm theo iteration; nếu tăng, kiểm tra dấu residual (`ekfom_data.h(i) = -norm_p.intensity`).
- `max_iteration` trong `city02.yaml` đang là 3; tăng quá cao có thể tốn CPU và không cải thiện nếu correspondence sai.
- `/Odometry` orientation quaternion phải normalized; code lấy trực tiếp từ `state_point.rot.coeffs()` sau update.

### Stage 6 — ikd-Tree mapping, ROS 2 wrapping, RViz/bag/CSV validation

#### 1. Giao diện dữ liệu & ROS 2 Mapping

- Input: `feats_down_body`, `feats_down_world`, `Nearest_Points`, `state_point`, `ikdtree`.
- Output:
  - Map nội bộ trong `ikdtree`.
  - `/cloud_registered`: current scan/map-aligned cloud.
  - `/cloud_registered_body`: body-frame cloud nếu bật.
  - `/Laser_map`: full local map nếu `map_en` bật.
  - `/Odometry`, `/path`, TF.
  - Service `/map_save` để save PCD khi `pcd_save_en` bật.

#### 2. Bản chất luồng xử lý

- `map_incremental()` chạy tuần tự sau ESKF update, gọi `ikdtree.Add_Points(...)`.
- ikd-Tree tự có thread nền rebuild; thread này lock `search_flag_mutex`, `working_flag_mutex`, `rebuild_logger_mutex_lock`, `points_deleted_rebuild_mutex_lock` để không phá nearest search/update của main mapping lane. `FAST_LIO/include/ikd-Tree/ikd_Tree.cpp` lines 201-310.
- `map_publish_callback()` chạy timer 1 Hz riêng để publish `/Laser_map` nếu bật. `FAST_LIO/src/laserMapping.cpp` lines 938-945, 1110-1113.

#### 3. Kiến thức lý thuyết nền tảng

- Mapping section paper: map là cube local lớn; khi LiDAR gần biên thì xóa vùng xa nhất khỏi ikd-Tree.
- ikd-Tree hỗ trợ incremental insert, box-wise delete, lazy labels, partial/parallel rebuild, on-tree downsampling.

#### 4. File code cốt lõi cần trích xuất

- `FAST_LIO/src/laserMapping.cpp`: `lasermap_fov_segment()`, `map_incremental()`, publish functions, map save callback.
- `FAST_LIO/include/ikd-Tree/ikd_Tree.cpp` và `.h`: `Build`, `Add_Points`, `Delete_Point_Boxes`, `Nearest_Search`, `multi_thread_rebuild`.
- `FAST_LIO/launch/mapping.launch.py`, `FAST_LIO/config/city02.yaml`.
- `city02_results/export_odom_to_csv.py`, `city02_results/convert_groundtruth_to_csv.py`, `city02_results/plot_city02_xy_aligned.py`.

#### 5. Hướng dẫn thực hành & trực quan hóa

ROS 2 wrapper thực tế:

```bash
colcon build --packages-select fast_lio
source install/setup.zsh
ros2 launch fast_lio mapping.launch.py config_file:=city02.yaml use_sim_time:=true
```

Terminal kiểm tra:

```bash
ros2 topic list
ros2 topic hz /Odometry
ros2 topic hz /path
ros2 topic hz /cloud_registered
```

Record trajectory:

```bash
ros2 bag record -o fastlio2_odom_bag_city02 --topics /Odometry /path /cloud_registered
```

Xuất CSV và vẽ so sánh:

```bash
cd city02_results
python3 convert_groundtruth_to_csv.py Groundtruth.txt groundtruth.csv
python3 export_odom_to_csv.py fastlio2_odom_bag_city02 /Odometry fastlio2.csv
python3 plot_city02_xy_aligned.py
```

Trong RViz: để xem global-looking accumulated map bằng `/cloud_registered`, tăng `Decay Time` từ 30 lên khoảng 10000 như ghi chú vận hành của bạn.

#### 6. Sanity Check

- Map không được “nở bông” quá mức: nếu có, kiểm tra `extrinsic_T/R`, `timestamp_unit`, `time_offset_lidar_to_imu`.
- `/Odometry` phải có tần số gần LiDAR scan rate sau giai đoạn init.
- `ikdtree.size()` và `validnum()` không được tăng vô hạn nếu local map cube hoạt động đúng; khi tới biên, `Delete_Point_Boxes` phải được gọi.

## Phần 3: Implementation Gotchas — bẫy dữ liệu, đa luồng và toán

### 3.1. Bẫy timestamp và đơn vị thời gian

- Ouster City02 dùng `timestamp_unit: 3` tức nanoseconds theo YAML. `FAST_LIO/config/city02.yaml` lines 11-16. Nếu nhầm sang microseconds/milliseconds, `curvature / 1000` trong undistortion sẽ sai scale, dẫn tới point bị compensate quá ít/quá nhiều.
- `sync_packages()` dùng `meas.lidar->points.back().curvature / 1000` để suy ra thời lượng scan. Nếu point cuối không phải point thời gian lớn nhất, phải sort hoặc đảm bảo preprocess gán curvature đúng.
- `imu_cbk()` có cả `time_offset_lidar_to_imu` và self-sync Livox khi `time_sync_en`; không bật self-sync tùy tiện cho dataset đã đồng bộ.

### 3.2. Bẫy mutex/condition variable khi đổi executor

- Code có `mtx_buffer` nhưng `sync_packages()` không lock. Với `SingleThreadedExecutor`, rủi ro thấp hơn; với `MultiThreadedExecutor`, callback producer có thể push/pop đồng thời với timer consumer. Nếu refactor, hãy dùng pattern:

```cpp
bool try_sync_thread_safe(MeasureGroup &meas) {
  std::unique_lock<std::mutex> lk(mtx_buffer);
  return sync_packages(meas);  // sync_packages chỉ thao tác buffer dưới lock
}
```

- Không giữ `mtx_buffer` trong lúc chạy `p_imu->Process()`, ESKF update hoặc map update; nếu giữ lock quá lâu, LiDAR/IMU callback sẽ bị nghẽn và drop dữ liệu.
- `sig_buffer` trong FAST-LIO2 hiện chỉ `notify_all`; nếu muốn consumer chờ condition thật sự, cần predicate rõ ràng, ví dụ `sig_buffer.wait(lk, []{ return flg_exit || (!lidar_buffer.empty() && !imu_buffer.empty()); });`.

### 3.3. Bẫy OpenMP và shared arrays

- `h_share_model()` có OpenMP loop theo point; chỉ an toàn nếu mỗi index ghi slot riêng và các hàm gọi bên trong không mutate shared state không khóa. `ikdtree.Nearest_Search()` phải đồng bộ với rebuild thread bằng mutex nội bộ.
- Không push vào `laserCloudOri` trong loop parallel; code hiện gom tuần tự sau khi đánh dấu `point_selected_surf[i]`, đây là pattern đúng.

### 3.4. Bẫy ikd-Tree rebuild thread

- ikd-Tree rebuild là background pthread, không phải ROS thread. Nó dùng nhiều `pthread_mutex_*`; nếu bạn wrap lại bằng `std::mutex`, đừng trộn lock order tùy tiện vì dễ deadlock.
- `multi_thread_rebuild()` khóa `search_flag_mutex` khi copy/replace subtree và dùng `Rebuild_Logger` để replay operation bị miss. Nếu thêm operation mới vào tree, phải ghi vào logger khi subtree đang rebuild.
- Map save PCD có thể phá realtime: code comment rõ PCD save cần nhiều memory và ảnh hưởng realtime. `FAST_LIO/src/laserMapping.cpp` lines 1167-1176.

### 3.5. Bẫy toán học khi chuyển từ paper sang code

- Paper viết state dimension manifold là 24, error state là 23 vì gravity nằm trên S2. Code `state_ikfom` đúng tinh thần này, nhưng measurement Jacobian FAST-LIO2 optimized chỉ build 12 cột active trong `h_x` cho position/rotation/extrinsic liên quan LiDAR; bias/gravity không trực tiếp xuất hiện trong point-to-plane measurement. `FAST_LIO/src/laserMapping.cpp` lines 759-788.
- Dấu residual rất dễ nhầm: code lưu `pd2` trong `normvec.intensity`, sau đó set `ekfom_data.h(i) = -norm_p.intensity`. Nếu bạn tự viết solver mini mà dùng `+pd2`, update có thể đi ngược.
- Gravity init lấy từ mean accelerometer: nếu robot đang chuyển động trong giai đoạn init, `mean_acc` không còn chỉ là gravity; bias/gravity sẽ sai và làm drift lớn.
- Extrinsic online calibration (`extrinsic_est_en: true`) giúp nhưng cũng tăng độ khó hội tụ. Khi debug City02, nên thử cả true/false để phân biệt lỗi extrinsic với lỗi timestamp.

### 3.6. Bẫy ROS 2 QoS và RViz/bag

- LiDAR non-Livox subscription dùng `rclcpp::SensorDataQoS()`, còn IMU dùng queue depth 10. Nếu file player publish quá nhanh hoặc `/clock`/`use_sim_time` không đúng, timer và topic hz sẽ gây hiểu nhầm.
- `/path` chỉ publish mỗi 10 pose để tránh RViz crash; đừng dùng `/path` để đo full-rate odometry. Dùng `/Odometry` để so sánh groundtruth.
- `/cloud_registered` là current registered scan, không phải global map đầy đủ. Muốn nhìn accumulated effect trong RViz, tăng Decay Time; muốn full local map, bật `publish.map_en` để publish `/Laser_map`.

## Hướng dẫn vận hành City02 trong workspace này

### 1. Build FAST-LIO2 và config City02

```bash
cd ~/Documents/anhthu/fast-lio2
colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy
source install/setup.zsh
colcon build --packages-select fast_lio
source install/setup.zsh
```

Config Ouster City02 đã có trong `FAST_LIO/config/city02.yaml`: `/ouster/points`, `/imu/data_raw`, `lidar_type: 3`, `scan_line: 128`, `scan_rate: 10`, `timestamp_unit: 3`. `FAST_LIO/config/city02.yaml` lines 3-16. Nếu dùng Livox Avia, đổi `common.lid_topic` sang `/avia/points`, `preprocess.lidar_type` sang `1`, và `scan_line` theo sensor.

### 2. Build và chạy HeLiPR file player

```bash
sudo apt update
sudo apt install ros-jazzy-novatel-gps-msgs qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools ros-jazzy-cv-bridge ros-jazzy-vision-opencv -y
colcon build --packages-select helipr_file_player --cmake-args \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DQT_MOC_EXECUTABLE=/usr/lib/qt5/bin/moc \
  -DQt5Core_MOC_EXECUTABLE=/usr/lib/qt5/bin/moc \
  -DQT_RCC_EXECUTABLE=/usr/lib/qt5/bin/rcc \
  -DQt5Core_RCC_EXECUTABLE=/usr/lib/qt5/bin/rcc \
  -DQT_UIC_EXECUTABLE=/usr/lib/qt5/bin/uic \
  -DQt5Widgets_UIC_EXECUTABLE=/usr/lib/qt5/bin/uic
source install/setup.zsh
ros2 run helipr_file_player helipr_file_player
```

### 3. Format dataset City02 bằng symlink

```bash
cd /media/crl/3D/anhthu/City02
mkdir -p LiDAR Inertial_data
ln -sfn ../sensor_data/ouster LiDAR/Ouster
ln -sfn ../sensor_data/Livox_avia LiDAR/Avia
mkdir -p LiDAR/Velodyne LiDAR/Aeva
ln -sfn ../sensor_data/xsens_imu.csv Inertial_data/xsens_imu.csv
touch Inertial_data/inspva.csv
awk -F',' '{gsub(/\r/, "", $2); if ($2=="ouster" || $2=="imu") print $1 "," $2;}' sensor_data/data_stamp.csv > stamp.csv
cut -d',' -f2 stamp.csv | sort | uniq -c
```

Kỳ vọng cho Ouster + IMU: `62478 imu` và `6247 ouster` như log bạn cung cấp.

### 4. Chạy mapping, record, plot

Terminal 1:

```bash
ros2 launch fast_lio mapping.launch.py config_file:=city02.yaml use_sim_time:=true
```

Terminal 2:

```bash
ros2 topic list
cd /home/crl/Documents/anhthu/fast-lio2/src/city02_results
ros2 bag record -o fastlio2_odom_bag_avia --topics /Odometry /path
```

Terminal 3:

```bash
ros2 run helipr_file_player helipr_file_player
ros2 topic hz /path
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
```

Sau khi chạy xong:

```bash
cd ~/Documents/anhthu/fast-lio2/src/city02_results
python3 convert_groundtruth_to_csv.py Groundtruth.txt groundtruth.csv
python3 export_odom_to_csv.py fastlio2_odom_bag_avia /Odometry fastlio2.csv
python3 -m venv .venv_plot
source .venv_plot/bin/activate
pip install --upgrade pip pyyaml
python3 plot_city02_xy_aligned.py
```

## Câu hỏi cần làm rõ trước khi mở rộng tài liệu/code

1. Bạn muốn README này giữ vai trò **tài liệu chính ở root workspace** hay muốn tách thêm các tutorial code thật trong `tools/` cho 6 stage?
2. Bạn muốn tập trung cấu hình City02 cho **Ouster 128** hay cần thêm một nhánh đầy đủ cho **Livox Avia** trong README và YAML?
3. Nếu bạn định đổi `fastlio_mapping` sang `MultiThreadedExecutor`, tôi khuyến nghị làm một PR code riêng để bọc `sync_packages()` bằng lock/predicate rõ ràng thay vì chỉ mô tả trong README.
