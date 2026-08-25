# 机械狗 z1w_v2 V1.0-3D 系统架构文档（重构版）

<!--
  重构时间: 2026-08-18 11:45 CST（首版）/ 2026-08-18 21:00 CST（本次更新: 导航激活 + 感知节点 + 字节序修复）
  基于实测: SSH 直连 Jetson 逐文件核对 dog_ws/src 全部包 + 运行态验证; SSH 进 UpBoard 核对实际部署 robot-software
  上一版: 2026-08-13（ICP 3D 全局定位替换 AMCL + livox 合并 + frame 统一）
  本次重大更新:
    1. 网络拓扑修正 — 雷达改走 USB 转网口 enx00e04c680779(192.168.1.100)，雷达 IP 192.168.1.195（原 .12/.50 已废弃）
    2. 时间戳软件补偿 — livox 驱动 GetEthPacketTimestamp 打补丁，LiDAR 时钟统一到 Unix（PTP 硬件不可行已坐实）
    3. systemd 自启动 — dog-brain.service 已部署并 enabled
    4. 建图存盘 — /map_save 服务已验证，已产出 map_final_*.pcd
    5. RViz 远程坑 — /Laser_map 单条 1.17GB 洪流会饿死 /tf 并触发雷达失同步
    6. ★ Nav2 导航栈全链路激活（2026-08-18 晚）— 修 5 类连环 bug（插件类名/失效BT库/观测源/voxel z/地图源），9 个 Nav2 服务器全 active
    7. ★ 新增 3 个感知节点 — pcd_to_map(3D→2D地图) / elevation_map(2.5D高程图) / step_detector(实时台阶检测)
    8. ★ TCP 字节序修复 — lcm_bridge 大端 >3d 改小端 <3d（匹配 UpBoard x86 小端 recv，原数据全错是 P0#2 隐藏根因）
  设备: NVIDIA Jetson Orin NX (wlP1p1s0=192.168.31.91/24 WiFi 静态+192.168.1.100/24 辅助 IP 连雷达, enP8p1s0=10.0.0.48/24 直连 UpBoard)
  ROS2: Humble Hawksbill (ARM64, JetPack r36.5.0, kernel 5.15.185-tegra)
  工作空间: /home/nvidia/dog_ws (单工作空间, --symlink-install, 6 个包)
  仓库: https://github.com/DLDLDL13579/big_dog_ws.git (origin/main 同步, 14 commits)
  上游定位: myeongw002/FAST_LIO_LOCALIZATION_ROS2 (通用版, 非 SICK 魔改版)
  UpBoard: 10.0.0.6 (Ubuntu 16.04, RT 内核 4.4.86-rt99, SSH xjzx/xjzx)
  UpBoard 部署: /home/xjzx/robot-software/ 预编译部署包(接口源码+控制核心.so+onnx), 控制核心无源码在 UpBoard
-->

---

## 0. 文档导航

- §1 系统拓扑总览（含网络/IP）
- §2 项目文件完整树
- §3 数据流全景（建图 / 导航）
- §4 组件详解（6 大组件逐文件）
- §5 时间戳同步（★ 本次重点）
- §6 TF 帧树
- §7 Topic 完整清单
- §8 运动模型 & 机械参数
- §9 代价地图 3D→2D 投影层次
- §10 双 IMU 闭环隔离
- §11 启动流程（含 systemd / 存图）
- §12 RViz 远程可视化指南与坑
- §13 阻塞项 & 待办（已更新状态）
- §14 关键设计决策总结
- §15 架构演进记录

---

## 1. 系统拓扑总览

```
                         ┌─────────────────────────────────────────────────────────┐
                         │              UpBoard (小脑/脑干, 10.0.0.6)                  │
                         │  (Ubuntu 16.04, RT 内核 4.4.86-rt99, SSH xjzx/xjzx)         │
                         │                                                         │
                         │  ┌─────────────────────────────────────────────────┐    │
                         │  │  YESENSE IMU (串口 /dev/ttyACM0, 460800bps)     │    │
                         │  │  16 轮足关节 (4腿×4关节, EtherCAT 500Hz)          │    │
                         │  │  StateEstimator (18维 KF, 500Hz)                │    │
                         │  │  ConvexMPCLocomotion (MPC 步态控制, 500Hz)       │    │
                         │  └─────────────────────────────────────────────────┘    │
                         │                                                         │
                         │  发布 LCM 通道 (UDP 组播 udpm://239.255.76.67:7667):     │
                         │    ├─ "global_to_robot"  (500Hz, localization_lcmt)    │
                         │    ├─ "state_estimator"  (500Hz, state_estimator_lcmt)│
                         │    ├─ "spi_data"         (500Hz, spi_data_t)           │
                         │    └─ "spi_command"      (500Hz, spi_command_t)        │
                         │  接收 TCP 指令 (监听 :3333, TCP server 已激活):          │
                         │    └─ [flag, yaw_rate(rad/s), velocity_norm[0,1]] ×3 double│
                         │       (小端 '<3d' — 2026-08-18 修复, 匹配 x86 小端 recv)   │
                         │                                                         │
                         │  实际部署形态: /home/xjzx/robot-software/ 预编译包         │
                         │    robot/src(接口源码) + build/*.so(控制核心:              │
                         │    libbiomimetics/libVisionMPC/libWBC_Ctrl 等) +           │
                         │    onnx/QuadActornet+Encodernet + mit_ctrl 可执行           │
                         │    运行: run_mc.sh ./mit_ctrl m r f (RT 内核)             │
                         │    ★ TCP 注入已解锁(rt_rc_interface.cpp): SWF开关双源切换 │
                         │      + 300ms 超时看门狗自动停车 (本地副本已改, 待部署)     │
                         └──────────────┬──────────────┬───────────────────────────┘
                                        │              │
                          LCM UDP 组播   │              │ TCP :3333
            239.255.76.67 → dev enP8p1s0 (ifname 已绑死)│ 指令下发
                                        │              │
    ════════════════════════════════════╪══════════════╪════════════════════════════
                          ROS2 DDS / Ethernet / USB3.2 │
    ════════════════════════════════════╪══════════════╪════════════════════════════
                                        │              │
    ┌───────────────────────────────────┼──────────────┼───────────────────────────┐
    │                        Jetson Orin NX (大脑)     │                           │
    │  ┌─ wlP1p1s0 = 192.168.31.91/24 (WiFi 静态 IP)       │                           │
    │  │             +192.168.1.100/24 (辅助 IP → Mid-360)  │                           │
    │  │  enP8p1s0  = 10.0.0.48/24 (主板网口 → UpBoard)     │                           │
    │  └─────────────────────────────────────────────────────                          │
    │                                                  │                        │
    │  ┌───────────────────────────────────────────────┴──────────────────────┐   │
    │  │                        lcm_bridge                                     │   │
    │  │  bridge_node.py (ROS2 Node, ament_python)                            │   │
    │  │                                                                      │   │
    │  │  LCM Subscriber (lcm_url: udpm://239.255.76.67:7667?ttl=255&ifname=enP8p1s0)│
    │  │  ├─ global_to_robot → /odom (nav_msgs/Odometry, 500Hz)             │   │
    │  │  │                    → TF odom→base_link: ⚠ publish_odom_tf=False │   │
    │  │  │                      (默认关, 让给 FAST-LIO, 见 §10/§15)         │   │
    │  │  ├─ state_estimator → /upboard/state_estimator (sensor_msgs/Imu)   │   │
    │  │  └─ spi_data        → /joint_states (sensor_msgs/JointState, 16关节)│   │
    │  │                                                                      │   │
    │  │  ROS2 Subscriber:                                                    │   │
    │  │  └─ /cmd_vel → TCP 10.0.0.6:3333 struct.pack('<3d', flag, ω, v/0.75)│   │
    │  │      (★ 2026-08-18 字节序修复: 原 '>3d' 大端, UpBoard x86 小端 recv  │   │
    │  │       直读无转换 → 数据全错(垃圾值), 改 '<3d' 小端后匹配)             │   │
    │  └─────────────────────────────────────────────────────────────────────┘   │
    │                                                  │                        │
    │  ┌───────────────────────────────────────────────┴──────────────────────┐   │
    │  │                     dog_sensors                                       │   │
    │  │  ┌─────────────────────────┐    ┌─────────────────────────┐         │   │
    │  │  │ livox_ros_driver2_node  │    │ realsense2_camera_node  │         │   │
    │  │  │ (dog_ws 内, 源码编译)   │    │ (apt ros-humble-*)      │         │   │
    │  │  │ ⚠ pub_handler.cpp 已打  │    │                         │         │   │
    │  │  │   时间戳软件补偿补丁(§5)│    │                         │         │   │
    │  │  │                         │    │                         │         │   │
    │  │  │ 物理: WiFi 辅助 IP → 雷达 │    │ 物理: USB3 直连         │         │   │
    │  │  │ 雷达 IP 192.168.1.195    │    │                         │         │   │
    │  │  │ host  IP 192.168.1.100  │    │                         │         │   │
    │  │  │ → /livox/lidar          │    │ → /camera/depth/        │         │   │
    │  │  │   (CustomMsg, ~10Hz)    │    │   color/points          │         │   │
    │  │  │ → /livox/imu            │    │   (PointCloud2, ~28.9Hz)│         │   │
    │  │  │   (Imu, 200Hz, BMI088)  │    │                         │         │   │
    │  │  └─────────────────────────┘    └─────────────────────────┘         │   │
    │  └─────────────────────────────────────────────────────────────────────┘   │
    │                                                  │                        │
    │  ┌───────────────────────────────────────────────┴──────────────────────┐   │
    │  │                    dog_description                                    │   │
    │  │  robot_state_publisher 加载 dog.urdf (实物 z1w_v2, 19 link/18 joint)  │   │
    │  │  → TF_static:                                                        │   │
    │  │      base_link → {FL,FR,RL,RR}_hip→thigh→calf→wheel                  │   │
    │  │      base_link → livox_frame (X:0.30, Z:0.15)                        │   │
    │  │      base_link → camera_link (X:0.35, Z:0.08, RPY roll=π 倒置修正)   │   │
    │  └─────────────────────────────────────────────────────────────────────┘   │
    │                                                  │                        │
    │  ┌───────────────────────────────────────────────┴──────────────────────┐   │
    │  │                 dog_brain (编排大脑)                                  │   │
    │  │  systemd: dog-brain.service (enabled, mode:=mapping, 自启)          │   │
    │  │                                                                      │   │
    │  │  ┌─ 建图模式 (mapping_launch.py) ──────────────────────────────┐    │   │
    │  │  │  sensors + robot_state_pub + lcm_bridge + FAST-LIO2         │    │   │
    │  │  │   /livox/lidar ─┐                                           │    │   │
    │  │  │                 ├─→ FAST-LIO2 ──→ odom→base_link TF         │    │   │
    │  │  │   /livox/imu  ──┘              ──→ /cloud_registered         │    │   │
    │  │  │                                ──→ /Odometry + /Laser_map   │    │   │
    │  │  │  存图: ros2 service call /map_save std_srvs/srv/Trigger      │    │   │
    │  │  └──────────────────────────────────────────────────────────────┘    │   │
    │  │                                                                      │   │
    │  │  ┌─ 导航模式 (navigation_launch.py) ───────────────────────────┐    │   │
    │  │  │  sensors + robot_state_pub + lcm_bridge + 定位四件套 + Nav2 │    │   │
    │  │  │  定位四件套 (ICP 3D 全局定位, 替换 AMCL):                    │    │   │
    │  │  │   global_map_publisher  → /global_map (PCD地图)             │    │   │
    │  │  │   fastlio_mapping       → /Odometry + odom→base_link TF    │    │   │
    │  │  │   global_localization   → /map_to_odom (ICP匹配)           │    │   │
    │  │  │   transform_fusion      → map→odom TF (绝对定位权)         │    │   │
    │  │  │  Nav2 3D (Navfn + DWB + VelSmoother OPEN_LOOP)            │    │   │
    │  │  └──────────────────────────────────────────────────────────────┘    │   │
    │  │                                                                      │   │
    │  │  → /cmd_vel (Twist) → lcm_bridge → TCP → UpBoard                   │   │
    │  └─────────────────────────────────────────────────────────────────────┘   │
    └───────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 项目文件完整树

```
/home/nvidia/dog_ws/                        ← 主工作空间 (单工作空间, --symlink-install)
│
├── src/
│   ├── dog_brain/                           [ament_cmake, v0.2.0]
│   │   ├── CMakeLists.txt                     install config/launch/maps
│   │   ├── package.xml                        depends: nav2_bringup/robot_state_pub/lcm_bridge/dog_desc/dog_sensors
│   │   ├── config/
│   │   │   ├── nav2_3d_params.yaml           9.8K  Nav2 3D Voxel Layer (DWB vx+wz, voxel+inflation)
│   │   │   └── fast_lio_mapping.yaml          942B  占位(实际用 fast_lio_localization/config/mid360.yaml)
│   │   ├── launch/
│   │   │   ├── bringup_launch.py             ★ 总入口 mode:=mapping|navigation
│   │   │   ├── mapping_launch.py             建图模式 (sensors+rsp+lcm_bridge+FAST-LIO2)
│   │   │   ├── navigation_launch.py          导航模式 (定位四件套 + Nav2)
│   │   │   └── navigation_launch.py.bak_fix  备份
│   │   └── maps/                              ✅ lab_3d_map.pcd (18.5MB, 2026-08-18 15:42 已就位)
│   │                                          (install 需 ln -sf 同步, 见 §12.5)
│   │
│   ├── dog_description/                     [ament_cmake, v0.1.0]  87MB
│   │   ├── urdf/
│   │   │   └── dog.urdf                      18.7K  实物 CAD (z1w_v2, 19 link/18 joint)
│   │   │       base_link + 4×4腿 + livox_frame + camera_link
│   │   │       livox_joint origin (0.30,0,0.15); camera_joint (0.35,0,0.08,rpy roll=π)
│   │   └── meshes/                            17 个 STL
│   │
│   ├── lcm_bridge/                          [ament_python, v0.1.0]
│   │   ├── config/bridge_params.yaml         lcm_url 带 ifname=enP8p1s0; upboard 10.0.0.6:3333
│   │   └── lcm_bridge/bridge_node.py         ★ publish_odom_tf 参数(默认False); /cmd_vel→TCP
│   │
│   ├── dog_sensors/                         [ament_python, v0.1.0]
│   │   ├── config/mid360_config.json         ★ 雷达 192.168.1.195 / host 192.168.1.100 (实测)
│   │   │       lidar_type 8(Mid360s), pcl_data_type 1(CustomMsg)
│   │   │       ports: cmd 56100/56101, push 56200/56201, point 56300/56301, imu 56400/56401
│   │   └── launch/
│   │       ├── all_sensors_launch.py        一键 mid360+d435i
│   │       ├── mid360_launch.py             xfer_format=1(CustomMsg), publish_freq=10, frame=livox_frame
│   │       └── d435i_launch.py               ★ 5 坑已修(见文件头注释)
│   │
│   ├── fast_lio_localization/               [ament_cmake, v0.0.0]  ★ FAST-LIO2 建图+定位
│   │   │                                    (myeongw002/FAST_LIO_LOCALIZATION_ROS2 通用版)
│   │   ├── src/laserMapping.cpp              ★ 含 /map_save 服务 (std_srvs/srv/Trigger, L945)
│   │   ├── src/preprocess.cpp                Livox CustomMsg → PointCloud2
│   │   ├── include/                          ikd-Tree / IKFoM_toolkit (迭代 ESKF)
│   │   ├── config/
│   │   │   ├── mid360.yaml                   ★ 实际使用 (lid/imu=/livox/*, time_sync_en=false,
│   │   │   │                                    extrinsic_est_en=false, pcd_save_en=true, interval=-1,
│   │   │   │                                    map_file_path=./test.pcd)
│   │   │   ├── mid360.yaml.bak_extrinsic_est  备份(extrinsic 抖动根因修复前)
│   │   │   ├── mid360.yaml.bak_timesync       备份
│   │   │   └── avia/horizon/ouster64/velodyne*.yaml
│   │   ├── scripts/
│   │   │   ├── global_map_publisher.py       PCD → /global_map (★ tolist() 性能bug已修: 缓存PointCloud2+tobytes)
│   │   │   ├── global_localization.py         ICP → /map_to_odom (订阅 /initialpose 触发, 0.5Hz)
│   │   │   ├── transform_fusion.py           融合 → map→odom TF
│   │   │   ├── publish_initial_pose.py        ★ 修复: Point/Quaternion 关键字参数 + 循环发布~3s确保送达
│   │   │   ├── pcd_to_map_node.py            ★ 新增: PCD 3D → 2D OccupancyGrid /map (static_layer 地图源)
│   │   │   ├── elevation_map_node.py         ★ 新增: 2.5D 高程图 /elevation_costmap (平地/可越台阶/障碍分级)
│   │   │   └── step_detector_node.py         ★ 新增: D435i 实时前方台阶检测 /step_ahead /step_height (补 Livox 近距盲区)
│   │   └── msg/Pose6D.msg
│   │
│   ├── livox_ros_driver2/                   [ament_cmake, v1.0.0]  ★ 雷达驱动
│   │   └── src/comm/pub_handler.cpp          ★ 已打时间戳软件补偿补丁(§5)
│   │       备份: .bak_timestamp(原版), .bak_unixfix(补丁后)
│   │       GetEthPacketTimestamp(L265): IMU(L117)+点云(L143)共用, 一处补丁修正两者
│   │
│   └── STARTUP_GUIDE.md                      ⚠ 已过时(仍写雷达 .12/.50, §5 阻塞项未更新) — 见本 §
│
├── install/                                    colcon 安装目录 (symlink-install)
├── build/ / log/                              colcon 编译/日志
├── map_final_20260818_111425.pcd              ★ 实测建图产出 17.7MB(~116万点)
├── map_incremental_20260818_111013.pcd        走一半快照 11.8MB
│
└── (用户目录 /home/nvidia/)
    ├── restart_stack.sh                       ★ 一键重启建图栈(pkill 模式写文件内, 避免 SSH 自杀)
    ├── stop_stack.sh                          一键停止建图栈
    └── Miniconda3-latest-Linux-aarch64.sh     (闲置)

/etc/systemd/system/dog-brain.service          ★ systemd 自启动 (enabled, mode:=mapping, ExecStartPre sleep 12)
```

---

## 3. 数据流全景

### 3.1 建图模式数据链路

```
硬件层
  Mid-360S LiDAR ──WiFi 辅助 IP(wlP1p1s0:192.168.1.100, nmcli 已配)─→ 雷达 192.168.1.195
  D435i        ──USB3 直连
  UpBoard 10.0.0.6 ──主板网口(enP8p1s0, 10.0.0.48)─→ LCM 组播 239.255.76.67:7667 + TCP:3333
        │
        ▼
Jetson (dog-brain.service, mode:=mapping)
  dog_sensors:
    Mid-360 → livox_ros_driver2_node (pub_handler.cpp 时间戳补丁)
            → /livox/lidar (CustomMsg 10Hz, 时间戳=Unix, 补偿后)
            → /livox/imu   (Imu 200Hz, 时间戳=Unix)
    D435i   → realsense2_camera_node → /camera/depth/color/points (~28.9Hz)
  lcm_bridge:
    global_to_robot → /odom (500Hz)            [⚠ TF 关, publish_odom_tf=False]
    state_estimator → /upboard/state_estimator (500Hz)
    spi_data        → /joint_states (16关节, 500Hz)
  dog_description: robot_state_publisher → TF_static (URDF)
  FAST-LIO2 (fastlio_mapping, mid360.yaml):
    输入: /livox/lidar + /livox/imu  (同源 BMI088+LiDAR, 紧耦合 ESKF)
    输出: /Odometry (10Hz) + odom→base_link TF (唯一权威)
          /cloud_registered (10Hz, ~127KB/帧, 全局去畸变点云)
          /cloud_registered_body / /cloud_effected / /Laser_map(★1.17GB!) / /path
    存图: ros2 service call /map_save std_srvs/srv/Trigger → ./test.pcd
```

### 3.2 导航模式数据流（ICP 3D 全局定位 + 地图双流 + 感知 + Nav2）

```
dog_brain (navigation_launch.py, ★ 2026-08-18 晚全链路激活)
  ┌─ 定位四件套 (替换 AMCL) ────────────────────────────────────┐
  │ ① global_map_publisher.py  maps/lab_3d_map.pcd → /global_map (frame=map)
  │ ② fastlio_mapping          /livox/* → /Odometry + odom→base_link TF
  │ ③ global_localization.py  ICP(open3d) /cloud_registered + /Odometry + /global_map
  │                           → /map_to_odom  (voxel 0.1, freq 0.5Hz, fitness>0.9 才更新)
  │ ④ transform_fusion.py     /Odometry + /map_to_odom → map→odom TF + /localization
  └──────────────────────────────────────────────────────────────┘
  ┌─ 地图双流 + 感知 (★ 新增, 为越障/模态切换打地基) ───────────┐
  │ ⑤ pcd_to_map_node.py   PCD → /map (2D OccupancyGrid 523x338) → static_layer
  │ ⑥ elevation_map_node.py PCD → /elevation_costmap (平地0/可越台阶80/障碍100)
  │ ⑦ step_detector_node.py D435i点云→ /step_ahead(Bool) + /step_height(m)  [实验性待标定]
  └──────────────────────────────────────────────────────────────┘
  ┌─ Nav2 3D (nav2_3d_params.yaml, ★ 插件类名/观测源/地图源已修复) ┐
  │ Planner: NavfnPlanner(A*, GridBased)  /goal_pose → /plan      │
  │ Controller: DWB (vx+wz, vy=0 差分配置)  max_vel_x=0.75, max_vel_theta=1.5│
  │ global_costmap: static(map_topic=/map→523x338) + voxel + inflation(0.55)│
  │ local_costmap:  voxel(/cloud_registered_body) + voxel(camera) + inflation│
  │   voxel_size 0.05, z_voxels 16, origin_z -0.30                │
  │ velocity_smoother: feedback=OPEN_LOOP  max [0.75,0,1.5]       │
  └──────────────────────────────────────────────────────────────┘
  → /cmd_vel → lcm_bridge → TCP 10.0.0.6:3333 (★ 小端 '<3d') → UpBoard
  ⚠ 机器人本体支持横移(轮=前进后退/腿=转向横移), 但 Nav2 差分配置(vy=0) + TCP无vy槽位
    → 全向横移能力待扩展(见 §8/§13)
```

---

## 4. 组件详解

### 4.1 lcm_bridge — LCM↔ROS2 桥接

文件: `lcm_bridge/lcm_bridge/bridge_node.py` | 可执行: `bridge_node`

```
LCM Channel         LCM Type              → ROS2 Topic/Type
global_to_robot     localization_lcmt     /odom (nav_msgs/Odometry, 500Hz)
  xyz[3]    f[0..2]  位置(m)               pose.position
  vxyz[3]   f[3..5]  速度(m/s)             twist.linear
  rpy[3]    f[6..8]  ZYX欧拉(rad)          ZYX→四元数 orientation
  omegaBody f[9..11] 角速度(rad/s)         twist.angular
  ⚠ odom→base_link TF: publish_odom_tf=False (默认关, 让给 FAST-LIO)
    /odom Odometry 消息仍发布 (备用里程计源, 不广播 TF)
state_estimator     state_estimator_lcmt   /upboard/state_estimator (Imu, 500Hz)
  quat[4]   f[18..21] w,x,y,z             orientation
  omegaBody f[15..17]                      angular_velocity
  aBody[3]  f[22..24]                      linear_acceleration
spi_data            spi_data_t             /joint_states (JointState, 16关节)
  q[16]/qd[16]/tau[16]                     position/velocity/effort
  关节顺序: {FL,FR,RL,RR}_{hip,thigh,calf,wheel}_joint (LCM_LEG_ORDER)

ROS2 → TCP:
  /cmd_vel (Twist) → TCP 10.0.0.6:3333
  struct.pack('<3d', flag, omega, v_norm)   # ★ 2026-08-18 小端修复(原 '>3d' 大端, UpBoard 小端recv无转换→数据全错)
  flag = 1 if |v|>0.01 or |ω|>0.01 else 0
  v_norm = clamp(v/0.75, -1, 1)   # m/s ÷ 0.75 → [0,1] 摇杆等效
  ⚠ 仅 3 个 double, 无 vy 槽位 → 全向横移需扩展协议(见 §8)
```

### 4.2 dog_sensors — 传感器驱动

```
Mid-360S LiDAR
  节点: livox_ros_driver2_node  (dog_ws 内源码编译, ★已打时间戳补丁 §5)
  配置: dog_sensors/config/mid360_config.json
  物理: WiFi 辅助 IP 192.168.1.100/24 (nmcli 已配, 原 USB 网卡已撤)
    雷达 IP 192.168.1.195 / host IP 192.168.1.100
    路由: 自动走 wlP1p1s0 (同子网 192.168.1.0/24)
    ports: cmd 56100/56101, push 56200/56201, point 56300/56301, imu 56400/56401
  输出:
    → /livox/lidar (CustomMsg, 10Hz, frame_id=livox_frame, 时间戳=Unix 补偿后)
    → /livox/imu   (Imu, 200Hz, BMI088, frame_id=livox_frame)
  xfer_format=1 (CustomMsg, PointXYZRTL 带时间戳)

Intel D435i (5 坑已修, 见 d435i_launch.py 文件头)
  节点: realsense2_camera_node (apt)
  修复:
    1. pointcloud__neon_.enable (ARM NEON 版参数名, 非 pointcloud.enable)
    2. __ns:=/ 覆盖 realsense2_camera 4.x 硬编码 namespace='camera'
    3. camera_name='camera' + base_frame_id='link' → frame 根=camera_link (匹配 URDF)
    4. depth/color profile 640x480x30 (D435i 不支持 1280x720x30)
    5. enable_infra1/2=False (Depth 与 Infra 分辨率不一致致 v4l2 超时)
  输出: /camera/depth/color/points (PointCloud2, ~28.9Hz, align_depth=True 带RGB)
        frame_id=camera_depth_optical_frame
```

### 4.3 dog_description — URDF / TF 树

文件: `dog_description/urdf/dog.urdf` (实物 z1w_v2 CAD)

```
19 link / 18 joint:
  base_link (躯干 0.50×0.35×0.15m)
  ├── {FL,FR,RL,RR}_hip_Link → _thigh_Link → _calf_Link → _wheel_Link
  ├── livox_frame   (livox_joint origin 0.30,0,0.15)
  └── camera_link   (camera_joint origin 0.35,0,0.08, rpy roll=π  ← 相机 180° 倒置修正, commit ec8b1d5)
  hip:    revolute axis=X  ±45°
  thigh:  revolute axis=-Y -155°~35°
  calf:   revolute axis=-Y 22°~150°
  wheel:  revolute axis=Y  ±∞
  腿长: hip→thigh 0.137, thigh→calf 0.30, calf→wheel 0.30 (总 ~0.71m), 轮径 0.225m
```

### 4.4 fast_lio_localization — FAST-LIO2 建图 + ICP 定位

来源: `myeongw002/FAST_LIO_LOCALIZATION_ROS2` (通用版, `typedef pcl::PointXYZINormal`, 原生 Livox CustomMsg)

```
建图 fastlio_mapping (C++, laserMapping.cpp):
  输入: /livox/lidar (CustomMsg) + /livox/imu (200Hz)
  算法: 迭代 ESKF + ikd-Tree 增量地图, scan-to-map 直接配准
  mid360.yaml 关键参:
    time_sync_en=false (★ 已统一 Unix, 不需软件再同步)
    extrinsic_est_en=false (★ RViz 抖动根因已关)
    extrinsic_T=[-0.011,-0.02329,0.04412]  extrinsic_R=I
    pcd_save_en=true  interval=-1  map_file_path=./test.pcd
  输出:
    /Odometry (frame=odom, child=base_link, 10Hz)
    odom→base_link TF (唯一权威, lcm_bridge TF 关)
    /cloud_registered (10Hz, ~127KB/帧)
    /cloud_registered_body / /cloud_effected / /Laser_map(★1.17GB, 73M点) / /path
  存图服务: /map_save (std_srvs/srv/Trigger, laserMapping.cpp L945) → ./test.pcd
            调用: ros2 service call /map_save std_srvs/srv/Trigger
            多次调用覆盖同一文件; 要留增量需 mv 重命名

定位四件套 (导航模式):
  ① global_map_publisher.py   PCD → /global_map
  ② fastlio_mapping            /Odometry + odom→base_link TF
  ③ global_localization.py     ICP(open3d) → /map_to_odom (voxel 0.1, freq 0.5, th 0.9)
  ④ transform_fusion.py        map→odom TF + /localization
  首次需 /initialpose 触发 (publish_initial_pose.py)
```

### 4.5 dog_brain — 建图/导航双模式编排

```
模式A mapping (mapping_launch.py):
  ├── dog_sensors (all_sensors_launch)
  ├── robot_state_publisher (dog.urdf)
  ├── lcm_bridge (publish_odom_tf=False)
  └── FAST-LIO2 (fastlio_mapping, mid360.yaml)
  systemd: dog-brain.service 默认即此模式

模式B navigation (navigation_launch.py, ★ 2026-08-18 晚已验证全链路激活):
  ├── dog_sensors + robot_state_publisher + lcm_bridge
  ├── 定位四件套 (fastlio_mapping + global_map_publisher + global_localization + transform_fusion)
  ├── 地图双流+感知 (pcd_to_map → /map / elevation_map → /elevation_costmap / step_detector → /step_ahead)
  └── Nav2 (nav2_bringup, nav2_3d_params.yaml): Navfn + DWB + behavior + VelSmoother(OPEN_LOOP)
        global_costmap(static map_topic=/elevation→/map 523x338 + voxel + inflation)
        local_costmap(voxel /cloud_registered_body + voxel(camera) + inflation)
        → /cmd_vel → lcm_bridge → TCP(小端 '<3d') → UpBoard
  前置: maps/lab_3d_map.pcd ✅ 已就位 (18.5MB, 2026-08-18 15:42) + install 需 symlink(见 §12 坑)
  运行态: 9 个 Nav2 服务器全 active; ICP fitness=1.000; /cmd_vel 有 6 发布者
```

---

## 5. 时间戳同步（★ 本次重构重点）

### 5.1 问题根因
- Livox Mid-360S 内部时钟为**上电起算（boot-relative）**，与主机 Unix 系统时钟相差约 56.6 年。
- 原驱动 `pub_handler.cpp::GetEthPacketTimestamp` 被人改成 NoSync 也用雷达内部时钟 → IMU/点云时间戳非 Unix。
- FAST-LIO 输出（/Odometry、/tf）用 boot 时钟（sec≈2971），而相机/upboard/主机用 Unix（sec≈1.787e9）→ **同一 TF 树混用两种纪元**，雷达-相机融合失效。
- 雷达 IMU↔点云内部同源（差一个扫描周期 ~105ms），本身同步。

### 5.2 PTP 可行性结论（不可行）
- 两块网卡均无 PHC（`ethtool -T` PTP Hardware Clock: none）：
  - USB 网卡 r8152/RTL8153B (enx00e04c680779)：连 software-transmit 都不支持 → ptp4l 主时钟报 "does not support requested timestamping mode"。
  - 主板网口 r8168 (enP8p1s0)：无 PHC。
- linuxptp 3.1.1 已装但用不上。**此硬件 PTP 不可行**（除非雷达改插 r8168 口+重配 IP，属布线决定）。

### 5.3 软件补偿方案（已实施并验证）
- 补丁位置: `livox_ros_driver2/src/comm/pub_handler.cpp::GetEthPacketTimestamp` (L265)
- IMU (L117) 与点云 (L143) 共用此函数 → **一处补丁同时修正两者且同基准**。
- 算法: 实时估计「雷达时钟 → Unix」偏移 `offset_ns`，最小延迟滤波跟踪钟漂：
  - `inst = system_now_ns - radar_stamp_ns`
  - 若 `inst < offset_ns` → `offset_ns = inst`（快速下跟到延迟下界）
  - 否则保持 `offset_ns`（缓慢上漂跟踪 ~30ppm 钟漂）
  - 返回 `radar_stamp + offset_ns`（统一到 Unix）
- 备份: `pub_handler.cpp.bak_timestamp`(原版) / `.bak_unixfix`(补丁后)。colcon build 持久化进 install，重启不丢。

### 5.4 验证结果（ros2 bag 精确测量）
- 全部时间戳统一到 Unix 纪元（此前差 56.6 年）。
- LiDAR IMU vs 系统时钟: **-0.38ms（亚毫秒）**。
- 点云: -115ms（=扫描起始时刻，正确）。
- 相机: -51.3ms（采集管线延迟，首帧 1 次 -54s global_time 启动瞬态）。
- FAST-LIO 同步错误 0，混纪元 TF 树已解决，精度亚毫秒~1ms，满足 LiDAR-相机融合。

### 5.5 ⚠ 已知脆弱性（非自愈）
- 软件 offset 无法瞬时跟踪雷达**时钟跳变**（雷达/USB 网卡短暂掉线重连后内部时钟复位/跃变）。
- 症状: FAST-LIO journal 狂刷 `IMU and LiDAR not Synced`（差 22~60s）、`No Effective Points!`、`VoxelGrid Integer indices overflow`；/Odometry 发散成垃圾（odom→base_link 平移达 -3.6M/-4.6M/-15.2M 米）；RViz "找不到 odom"。
- **不自愈**，需手动 `sudo systemctl restart dog-brain` 重启建图栈（重启会丢失进程内已积累的地图，故建图途中务必先 /map_save 存增量）。
- 触发场景实证: VM RViz 订阅 /Laser_map(1.17GB) 洪流压垮 USB 雷达链路 → 失同步。
- 建议根治: 加看门狗（连续 N 秒 "No Effective Points" 自动重启）。

---

## 6. TF 帧树

```
map ──→ odom ──→ base_link ──┬── {FL,FR,RL,RR}_hip → thigh → calf → wheel
  │        │                 │── livox_frame
  │        │                 │── camera_link
  │        │
  │        │  发布者:
  │        │   odom→base_link:  FAST-LIO2 (fastlio_mapping)  ← 唯一权威
  │        │                     ⚠ lcm_bridge TF 已关 (publish_odom_tf=False)
  │        │                        (其 /odom Odometry 消息仍发, 但不广播 TF → 无双源冲突)
  │        └── map→odom:        transform_fusion.py (导航模式 ICP 绝对定位权)
  │                              (建图模式: FAST-LIO 直接 odom 系, 无 map 系)
  └── base_link→子link:  robot_state_publisher (URDF fixed + 16 revolute)
```

**frame 统一 (2026-08-13)**: 上游 camera_init/body → 标准 odom/base_link (laserMapping.cpp 11 处 + Python 3 处)。所有点云/地图/轨迹均 `odom` 帧。

---

## 7. Topic 完整清单

### 7.1 传感器
| # | Topic | 类型 | 频率 | 发布者 | 帧 |
|---|-------|------|------|--------|-----|
| 1 | `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 10Hz | livox_ros_driver2_node | livox_frame |
| 2 | `/livox/imu` | `sensor_msgs/msg/Imu` | 200Hz | livox_ros_driver2_node | livox_frame |
| 3 | `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | ~28.9Hz | realsense2_camera_node | camera_depth_optical_frame |

### 7.2 LCM 桥接（底盘）
| # | Topic | 类型 | 频率 | 发布者 | 来源 LCM |
|---|-------|------|------|--------|----------|
| 4 | `/odom` | `nav_msgs/msg/Odometry` | ~500Hz | lcm_bridge | global_to_robot |
| 5 | `/upboard/state_estimator` | `sensor_msgs/msg/Imu` | ~500Hz | lcm_bridge | state_estimator |
| 6 | `/joint_states` | `sensor_msgs/msg/JointState` | ~500Hz | lcm_bridge | spi_data |
| — | `/tf`(odom→base_link) | — | — | **已关**(publish_odom_tf=False) | — |

### 7.3 FAST-LIO2
| # | Topic | 类型 | 单条大小 | 发布者 |
|---|-------|------|---------|--------|
| 7 | `/Odometry` | `nav_msgs/msg/Odometry` | 0.7KB | fastlio_mapping |
| 8 | `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | ~127KB | fastlio_mapping |
| 9 | `/cloud_registered_body` | `PointCloud2` | — | fastlio_mapping |
| 10 | `/cloud_effected` | `PointCloud2` | — | fastlio_mapping |
| 11 | `/Laser_map` | `PointCloud2` | **~1.17GB(73M点)** | fastlio_mapping |
| 12 | `/path` | `nav_msgs/msg/Path` | — | fastlio_mapping |
| — | `/cloud_map` `/mapPath` | — | — | **无发布者**(旧残留, RViz 勿用) |

### 7.4 ICP 定位四件套（导航模式）
| # | Topic | 类型 | 发布者 |
|---|-------|------|--------|
| 13 | `/global_map` | `PointCloud2` | global_map_publisher.py |
| 14 | `/map_to_odom` | `Odometry` | global_localization.py |
| 15 | `/localization` | `Odometry` | transform_fusion.py |

### 7.4b 地图双流 + 感知（★ 新增, 2026-08-18）
| # | Topic | 类型 | 发布者 |
|---|-------|------|--------|
| 15a | `/map` | `OccupancyGrid` | pcd_to_map_node.py（3D PCD→2D, 523×338, static_layer 地图源） |
| 15b | `/elevation_costmap` | `OccupancyGrid` | elevation_map_node.py（平地0/可越台阶80/障碍100, 537×337） |
| 15c | `/step_ahead` | `Bool` | step_detector_node.py（D435i 前方台阶） |
| 15d | `/step_height` | `Float32` | step_detector_node.py（估计台阶高度 m） |

### 7.5 Nav2
| # | Topic | 类型 | 发布者 |
|---|-------|------|--------|
| 16 | `/cmd_vel` | `Twist` | controller_server(DWB) vx+wz |
| 17 | `/plan` `/local_plan` | `Path` | planner/controller |
| 18 | `/global_costmap/costmap_raw` `/local_costmap/costmap_raw` | `OccupancyGrid` | costmap |
| 19 | `/goal_pose` | `PoseStamped` | 用户/RViz |

### 7.6 服务
| 服务 | 类型 | 说明 |
|------|------|------|
| `/map_save` | `std_srvs/srv/Trigger` | FAST-LIO 存图 → ./test.pcd (cwd=/home/nvidia/dog_ws) |

### 7.7 ROS2 → UpBoard
| 路径 | 协议 | 格式 |
|------|------|------|
| 10.0.0.6:3333 | TCP | `struct.pack('<3d', flag, ω_z, v_x/0.75)` ★ 小端(2026-08-18 修复) |

---

## 8. 运动模型 & 机械参数

### 8.1 轮腿复合运动模型（★ 2026-08-18 晚修正: 原"差分 vy=0"描述不完整）
```
本体能力 (用户澄清 + 代码核实):
  前进/后退: 轮子驱动 (4 轮, 高效)
  转向:      腿部动作 (+ 轮子配合, 无转向机构)
  横移:      腿部动作 (全向能力, 支持 vy)
  → 机器人本体是全向轮腿复合, RL 策略支持 [vx, vy, yaw] 三维指令
代码证据:
  1. FSM_State_RL::readVelocityCommand: vx=stateDes(6), vy=-stateDes(7), yaw=stateDes(11) → 用 vy
  2. RLConfig: max_cmd={0.8, 0.6, 0.6} (vx/vy/yaw 限幅), 轮腿分工由 RL 策略训练涌现(非显式切换)
  3. ConvexMPCLocomotion: y_vel_cmd = v_des[1]*0.5 (纯腿也支持横移)
  4. rt_rc_interface (at9s): v_des[1] = -right_stick_y (遥控器横移)
当前限制 (能力错配, 待扩展):
  1. Nav2 差分配置: max_vel_y=0.0 / acc_lim_y=0.0 (nav2_3d_params.yaml L109-116)
  2. TCP 协议仅 3 double [flag, ω, v/0.75], 无 vy 槽位 → 横移指令传不到 UpBoard
  3. ConvexMPC 纯腿模式(LOCOMOTION): 腿足步态, 轮不驱动; RL 模式: 轮腿一体
  → 全向横移导航需: Nav2 改全向(max_vel_y>0) + TCP 扩展第4个double=vy + UpBoard 注入 v_des[1]
```

### 8.2 速度约束
| 参数 | 值 | 来源 |
|------|-----|------|
| 最大直行速度 | 0.75 m/s | Nav2 DWB max_vel_x |
| 最大角速度 | 1.5 rad/s (86°/s) | Nav2 DWB max_vel_theta |
| 加/减速 | vx 0.5 m/s², wz 1.0 rad/s² | Nav2 acc/decel_lim |
| MPC 周期 | 0.002s (500Hz) | UpBoard controller_dt |
| 速度归一化 | TCP v_x ÷ 0.75 → [0,1] | bridge_node |

### 8.3 Footprint
```
Nav2 footprint(矩形): [[0.45,0.25],[0.45,-0.25],[-0.45,-0.25],[-0.45,0.25]] padding 0.05
尺寸 0.9m×0.5m (含余量) | 实物躯干 0.50×0.35m
```

---

## 9. 代价地图 3D→2D 投影层次

```
Z 2.0m ┤ Voxel Layer: Livox Mid-360  Z 0.2~2.0m 距离 1.5~5.0m 360° 200k pts/s
       │   中远距宏观避障(桌椅/墙/人)
Z 0.2m ┤────────────────────────────
Z 0.1m ┤ Voxel Layer: D435i Depth   Z 0.1~2.0m 距离 0.7~3.0m FOV 87°×58°
       │   近距补盲(台阶/小物体, 补 Livox 盲区)
Z 0.0m ┤═════════ 地面 ═════════════
       ╔ Inflation Layer  radius 0.55m, cost_scaling 2.5, 迫使 DWB 提前绕行 ╗
       ╚ 覆盖 0~0.7m 物理盲区: 1.Inflation 概率绕行 2.UpBoard 力控+触地兜底  ╗

global_costmap: static_layer + voxel_layer(livox) + inflation
local_costmap:  voxel_layer(livox) + voxel_layer(camera) + inflation
voxel_size 0.05, z_voxels 16, observation livox_scan [+camera_scan local]
```

---

## 10. 双 IMU 闭环隔离

```
闭环1 SLAM 定位
  Livox Mid-360 内置 BMI088 IMU (200Hz) → /livox/imu → FAST-LIO2 → odom→base_link TF
  特点: 与 LiDAR 同硬件精确同步, 紧耦合 ESKF; ★时间戳经软件补偿统一 Unix
  ⚠ /upboard/state_estimator 不进 SLAM (防网络延迟崩建图)

闭环2 运动学控制
  UpBoard YESENSE IMU(500Hz) + SPI 16关节(500Hz) → StateEstimator(18维 KF) → ConvexMPC
  步态控制在 UpBoard 内部, 不依赖 Jetson; 高频实时, 含接触检测+力控

两闭环输入不同、职责不同、互不干扰。
```

---

## 11. 启动流程

### 11.1 systemd 自启动（推荐，已部署）
```bash
# 状态/启停
sudo systemctl status  dog-brain
sudo systemctl start   dog-brain      # 建图模式
sudo systemctl stop    dog-brain
sudo systemctl restart dog-brain      # 失同步时一键恢复
sudo systemctl disable dog-brain      # 取消开机自启
```
- 服务: `/etc/systemd/system/dog-brain.service`，`enabled`，`mode:=mapping`，`ExecStartPre=/bin/sleep 12`(等网络/雷达)，`Restart=on-failure`，`RestartSec=8`，`TimeoutStopSec=20`。
- 时间戳补丁在 systemd 启动下同样生效（install 持久化）。

### 11.2 一键脚本
```bash
/home/nvidia/restart_stack.sh   # pkill+nohup setsid 重拉 mapping 栈 (pkill 模式写文件内避免 SSH 自杀)
/home/nvidia/stop_stack.sh      # 仅停止
```
⚠ **坑**: 不要在 SSH 命令行直接 `pkill -f bringup_launch.py`，会匹配并误杀自己的 SSH 会话；用脚本文件。

### 11.3 手动 launch（调试）
```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/dog_ws/install/setup.bash
ros2 launch dog_brain bringup_launch.py mode:=mapping      # 或 mode:=navigation
```

### 11.4 建图 + 存图
```bash
# 1. 起栈(systemd 或手动), 静止 5~10s 让 IMU 初始化
sudo systemctl start dog-brain
# 2. 走一圈 (RViz 只看 /cloud_registered, ★禁订 /Laser_map)
# 3. 存图(可多次, 中途存增量):
ros2 service call /map_save std_srvs/srv/Trigger
# 4. 重命名保留:
mv /home/nvidia/dog_ws/test.pcd /home/nvidia/dog_ws/map_$(date +%Y%m%d_%H%M).pcd
```

### 11.5 导航（需先有 PCD 地图, ★ 2026-08-18 晚已验证激活链路）
```bash
# 地图已就位(src 与 install), 如需更换:
cp map_final_*.pcd /home/nvidia/dog_ws/src/dog_brain/maps/lab_3d_map.pcd
# ⚠ symlink-install 不链接 build 后新增文件: install 路径需手动 ln -sf (见 §12 坑)
ros2 launch dog_brain navigation_launch.py
# 或: ros2 launch dog_brain navigation_launch.py map_pcd:=/path/to/map.pcd
# 首次定位(★ publish_initial_pose 已修复: 关键字参数+循环发布, 单次调用可靠):
ros2 run fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0
# 验证定位收敛:
ros2 topic echo /map_to_odom nav_msgs/msg/Odometry --once   # 非恒等即收敛
# 发目标(★ 未实测, P0#2):
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"
```
注意: systemd dog-brain.service 是 mapping 模式; 导航需手动 `ros2 launch dog_brain bringup_launch.py mode:=navigation`（nohup setsid）或新增 navigation service。当前导航栈日志: /home/nvidia/nav_test*.log。

### 11.6 网络（WiFi 双 IP 已 nmcli 持久化）
```bash
# LCM 组播绑定 enP8p1s0 (bridge_params.yaml 已写 ifname=enP8p1s0)
# WiFi 双 IP (已 nmcli 持久化, 重启自动生效):
#   主 IP: 192.168.31.91/24 (上网)
#   辅助 IP: 192.168.1.100/24 (Mid-360 雷达通信)
# 若辅助 IP 丢失: sudo nmcli connection modify Xiaomi_A389 +ipv4.addresses 192.168.1.100/24 && sudo nmcli connection up Xiaomi_A389
```

---

## 12. RViz 远程可视化指南与坑（★ 新增）

### 12.1 VM 连接 Jetson
- Parallels 必须 **「桥接」网络**（不要用共享/NAT），让 VM 拿到 192.168.31.x；确认 `ping 192.168.31.91` 通、`ros2 topic list` 能看到 Jetson 话题。
- `ROS_DOMAIN_ID=11`（与 Jetson 一致）。
- 启动 RViz 的终端先 `source ~/dog_ws/install/setup.bash`（VM 上需 colcon build 过 `dog_description`，否则 RobotModel 报 "package does not exist"）。

### 12.2 RViz Display 配置（建图）
- **Fixed Frame = `odom`**（地图/点云/轨迹均此帧）
- TF、RobotModel（/robot_description）、PointCloud2（/cloud_registered 实时）、Path（/path）
- 地图想看累计: 只能本地，远程**禁订 /Laser_map**（见下）

### 12.3 ★ /Laser_map 洪流坑（必读）
- `/Laser_map` 单条 **73,038,816 点 ≈ 1.17GB**（FAST-LIO 累计全图）。
- VM 经网络订阅 → 1GB+ 洪流打满 VM→Mac→WiFi→Jetson 链路 → 体积极小但延迟敏感的 `/tf`/`/tf_static` 被饿死 → RobotModel/点云/底盘帧全部同时消失；且可能压垮 USB 雷达链路 → 雷达掉线重连 → 失同步（§5.5）→ 建图发散。
- `/cloud_registered` 仅 ~127KB/帧（~1MB/s），远程订阅安全。
- 看全图三法: ① Jetson 本地 RViz 订 /Laser_map；② 存 PCD 拷回 VM 离线看（`ros2 run pcl_ros pcd_to_pointcloud <file>`，需 `ros-humble-pcl-ros`）；③ Jetson 加 VoxelGrid 降采样后发布 /Laser_map_downsampled 远程订阅。
- ⚠ `/cloud_map` `/mapPath` 无发布者（旧残留），RViz 勿用。

### 12.4 失同步时 RViz 现象与处置
- 现象: RViz "找不到 odom"、模型消失、/cloud_registered 不动。
- 诊断: `journalctl -u dog-brain -n 60 | grep -iE "not Synced|No Effective"`
- 处置: `sudo systemctl restart dog-brain`（⚠ 会丢进程内地图，建图途中务必先 /map_save 存增量）

### 12.5 ★ 导航模式新增坑（2026-08-18 晚实测）
- **/global_map 读到 0 点** → open3d `read_point_cloud` 读**不存在的文件**静默返回空点云（不抛异常）。`--symlink-install` 不链接 build 后新增的 maps/lab_3d_map.pcd → `ln -sf src/dog_brain/maps/lab_3d_map.pcd install/dog_brain/share/dog_brain/maps/lab_3d_map.pcd`
- **global_map_publisher 无消息** → `points.tolist()` 每周期转 58 万点 CPU 打满 → 已修（一次性缓存 PointCloud2 + `numpy.tobytes()`）
- **publish_initial_pose 发不出** → 单次 `spin_once` 退出太快 DDS 握手未完成 → 已修（循环发布 ~3s）；`Point(x,y,z)` 位置参数 TypeError → 已改关键字参数
- **Nav2 服务器不 active** → 插件类名（斜杠/双冒号家族）、失效 BT 库（always_failure 等）、观测源类型（CustomMsg→PointCloud2）——见 §15 变更 #1
- **D435i 稠密点云 ~27 万点/帧** → `read_points` 逐点解析仅 1Hz → `numpy.frombuffer` 直接解析 15Hz
- **elevation_map RANSAC 不稳** → `segment_plane` 随机采样致每次拟合平面不同 → 改确定性地面基准（z 3% 分位）
- **新脚本必须 chmod +x** → colcon `install(PROGRAMS)` 要求可执行，否则 launch 报 "executable not found on libexec"

---

## 13. 阻塞项 & 待办（已更新状态 2026-08-18 晚）

| # | 项目 | 状态 | 说明 |
|---|------|------|------|
| 1 | ~~Mid-360 网口网段冲突~~ | ✅ 已解决 | 雷达改走 USB 网卡 enx00e04c680779(192.168.1.100)，雷达 192.168.1.195，静态主机路由已配 |
| 2 | ~~Mid-360 物理连接~~ | ✅ 已解决 | USB 转网口接雷达，主板网口接 UpBoard，互不冲突 |
| 3 | 时间戳同步 | ✅ 已解决 | PTP 不可行(无 PHC)，软件补偿补丁已实施，全 Unix，亚毫秒 |
| 4 | RViz RViz2 jitter | ✅ 已解决 | extrinsic_est_en=false (mid360.yaml) |
| 5 | systemd 自启动 | ✅ 已完成 | dog-brain.service enabled |
| 6 | 建图存盘 | ✅ 已验证 | /map_save 服务可用，已产出 map_final_*.pcd |
| 7 | ~~导航 PCD 地图就位~~ | ✅ 已解决 | maps/lab_3d_map.pcd 已就位 (18.5MB, 15:42) + install symlink(§12 坑) |
| 8 | 失同步不自愈 | 🟡 待办 | 软件 offset 无法跟踪雷达时钟跳变，建议加看门狗自动 restart |
| 9 | ~~UpBoard TCP 控制解锁~~ | 🟡 已改待部署 | rt_rc_interface.cpp TCP 注入已写(本地副本: SWF双源+300ms看门狗), 字节序已修(lcm_bridge '<3d'), **未部署未实测** |
| 10 | ~~Nav2 全链路激活~~ | ✅ 已解决 | 修 5 类 bug(插件类名/失效BT库/观测源/voxel z/地图源)，9 服务器全 active，ICP fitness=1.0 |
| 11 | STARTUP_GUIDE.md 过时 | 🟡 待办 | 仍写雷达 .12/.50、§5 阻塞项未更新（本架构文档已为最新） |
| 12 | **发 goal 实测驱动（P0#2）** | 🔴 待办 | /cmd_vel→TCP→UpBoard 驱动狗未实测，需狗在场（字节序已修+注入已写，待部署验证） |
| 13 | **轮/腿/轮腿模态自主切换** | 🔴 缺失 | RL=轮腿一体/LOCOMOTION=纯腿/无纯轮模式，无地形自主切换（需新增 FSM 逻辑） |
| 14 | **全向横移扩展** | 🟡 待办 | Nav2 差分配置(vy=0)+TCP 无 vy 槽位 → 需 Nav2 全向 + TCP 加第4个double=vy + UpBoard 注入 v_des[1] |
| 15 | step_detector 现场标定 | 🟡 待办 | 台阶检测阈值需 RViz 现场标定（当前报 0.079m 抬升，真伪待确认） |
| 16 | D435i 启动瞬态 | 🟡 观察 | "Depth stream start failure" 为开机瞬态自愈，实测 depth 27Hz 正常 |

---

## 14. 关键设计决策总结

| # | 决策 | 原因 |
|---|------|------|
| 1 | LCM 桥接(非 DDS 直连) | UpBoard 用 LCM, 非 ROS2; 桥接最小改动 |
| 2 | ~~差分模型 vy=0~~ → **轮腿复合全向(2026-08-18 修正)** | 轮=前进/后退, 腿=转向/横移, 本体支持 vy; 但 Nav2/TCP 仍差分, 全向待扩展(§8.1) |
| 3 | FAST-LIO2 > SLAM Toolbox | 3D 建图紧耦合精度更高 |
| 4 | Voxel Layer > 2D LaserScan | 3D 雷达必须 3D→2D 投影 |
| 5 | D435i 近距补盲 | Livox 前下方 0~1.5m×0~0.2m 盲区 |
| 6 | 双 IMU 隔离 | BMI088→SLAM 防网络延迟; YESENSE→控制高频 |
| 7 | 建图/导航分离 | 避免 SLAM+定位冲突 |
| 8 | ICP 3D 定位替换 AMCL | 3D 地图不支持 2D AMCL; /scan 无人发布; 全地形需 3D 定位 |
| 9 | frame 统一 odom/base_link | 上游 camera_init/body → 标准命名匹配 Nav2/URDF |
| 10 | publish_odom_tf=False | 避免底盘运动学与激光惯性双 TF 冲突 |
| 11 | **雷达走 USB 网口** | 主板网口留 UpBoard(LCM)，USB 网口接雷达，互不干扰 |
| 12 | **软件时间戳补偿(非 PTP)** | 硬件无 PHC，软件 offset 补偿足够 LiDAR-相机融合(亚毫秒) |
| 13 | **systemd 自启动** | 重启即恢复建图能力，补丁随 install 持久化 |
| 14 | **/Laser_map 远程禁订** | 1.17GB 洪流会饿死 /tf 并触发雷达失同步 |
| 15 | **存 3D PCD, 用降维 2D(/map) + 2.5D(/elevation_costmap)** | 内存/算力受限; 3D 保留建图, 2D 给 Nav2, 高程给越障(2026-08-18) |
| 16 | **自研感知节点(pcd_to_map/elevation/step_detector)** | 避免装 grid_map 等大依赖(apt 源 403 坑); 与占用判据对齐保证一致性 |
| 17 | **TCP 小端 '<3d'** | UpBoard x86 小端 recv 直读无转换, 大端数据全错(2026-08-18 修复) |
| 18 | **TCP 注入双源切换+看门狗** | SWF 开关选遥控/导航, 300ms 超时自动停车防失控(本地副本已写, 待部署) |

---

## 15. 架构演进记录

### 2026-08-13 变更（上一版）
1. ICP 3D 全局定位替换 AMCL（接法 A）：global_map_publisher + fastlio_mapping + global_localization + transform_fusion。
2. frame 统一：camera_init/body → odom/base_link（laserMapping.cpp 11 处 + Python 3 处）。
3. lcm_bridge publish_odom_tf 开关（默认 False），TF 权让 FAST-LIO。
4. livox 驱动合并进 dog_ws。
5. slam_toolbox 旧 2D 架构清理。

### 2026-08-18 变更（本次重构）
1. **网络拓扑修正**：雷达从主板网口迁移到 USB 转网口 enx00e04c680779，IP 改为 host 192.168.1.100 / 雷达 192.168.1.195；mid360_config.json 已更新；静态主机路由 192.168.1.195 dev enx00e04c680779。原 §11 阻塞项 #1/#2 解决。
2. **livox 时间戳软件补偿**：pub_handler.cpp::GetEthPacketTimestamp 打补丁（min-delay 滤波跟踪 ~30ppm 钟漂），IMU+点云共用一处修正，全统一 Unix，亚毫秒。PTP 经 ethtool -T 确证硬件不可行。备份 .bak_timestamp/.bak_unixfix。
3. **RViz jitter 修复**：mid360.yaml extrinsic_est_en=false（原 true 导致 odom 抖动）。
4. **systemd 自启动**：/etc/systemd/system/dog-brain.service（enabled, mode:=mapping, sleep 12, Restart=on-failure）。
5. **一键脚本**：/home/nvidia/restart_stack.sh、stop_stack.sh（pkill 模式写文件避免 SSH 自杀）。
6. **建图存盘验证**：/map_save(std_srvs/srv/Trigger) 已用，产出 map_final_20260818_111425.pcd(17.7MB/~116 万点)。
7. **RViz 远程坑定位**：/Laser_map 1.17GB 洪流会饿死 /tf 并触发雷达失同步；远程只看 /cloud_registered。
8. **失同步脆弱性记录**：雷达重连时钟跳变 → 软件 offset 追不上 → 不自愈 → 需 restart（建议加看门狗）。

### 2026-08-18 晚变更（本次更新: 导航激活 + 感知 + 协议修复, 3 commits）
1. **Nav2 导航栈全链路激活（P0#1 达成）**：修 5 类连环 bug —
   ①插件类名按 pluginlib ID 修正(navfn/behaviors/bt_navigator 斜杠, waypoint 双冒号) ②移除新版 Humble 已删的 BT 库(always_failure 等) ③雷达观测源 /livox/lidar(CustomMsg) 改 /cloud_registered_body(PointCloud2, frame=base_link) ④voxel origin_z 0→-0.30 ⑤pcd_to_map 提供 /map 给 static_layer。9 个 Nav2 服务器全 active，ICP fitness=1.000。
2. **新增 pcd_to_map_node.py**：PCD 3D → 2D OccupancyGrid /map（transient_local, 523×338），static_layer 地图源；修 global_map_publisher tolist() 性能灾难(缓存 PointCloud2)。
3. **新增 elevation_map_node.py**：2.5D 高程图 /elevation_costmap（确定性地面基准 z 3% 分位 + 占用判据对齐 + 高度分级: 低矮<0.5m=可越台阶80/高大=绕行100）。
4. **新增 step_detector_node.py**：D435i 稠密点云(~27万点, numpy.frombuffer 解析 15Hz) → /step_ahead + /step_height，补 Livox 0.5-1.5m 近距盲区（实验性待标定）。
5. **TCP 字节序修复**：lcm_bridge '>3d' 大端 → '<3d' 小端（UpBoard x86 小端 recv 直读，原数据全错是 P0#2 隐藏根因）。
6. **TCP 注入解锁（本地副本, 未部署）**：rt_rc_interface.cpp RL 分支加 SWF 双源切换 + 300ms 超时看门狗自动停车；修横移/pitch 污染。
7. **运动模型修正**：差分 → 轮腿复合全向（轮=前进后退, 腿=转向横移, RL 支持 vy）；Nav2/TCP 仍差分，全向扩展列为待办。

### git 提交历史（14 commits）
```
5011127 feat(perception): 新增实时前方台阶检测节点 step_detector_node
fe95bc8 feat(elevation): 新增 2.5D 高程图节点 elevation_map_node
f164072 feat(navigation): Nav2 导航栈全链路联调修复
ec8b1d5 fix: 修复导航启动 URDF/桥接配置 + 相机图像 180° 倒置
804c490 fix: 修正雷达型号为 Mid360s 并修复建图启动问题
8255cb0 docs: 更新启动指南 - 定位架构切换为 ICP 3D 全局定位 (接法A)
0438598 feat(navigation): 接法A - ICP 3D 全局定位替换 AMCL
1ff5d2a feat(fast_lio_localization): 集成 FAST-LIO2 建图+定位
57eb17d feat: 合并 livox_ros_driver2 雷达驱动 + 新增启动指南
17d1d00 chore(dog_brain): 移除 slam_toolbox 旧 2D 架构残留
5672f33 fix(dog_sensors): D435i 关闭 Infra 流并注释根因
5c3ce03 fix(dog_sensors): D435i launch 修复 - 适配 Jetson ARM/NEON 版
9035a2f fix: lcm_bridge 修复 + 参数更新
3500f11 feat: z1w_v2 机械狗大脑 V1.0-3D 初始提交
```
(注: 另有两个本地未提交的增量: ①Jetson lcm_bridge 字节序 <3d ②UpBoard rt_rc_interface TCP 注入(本地副本))
