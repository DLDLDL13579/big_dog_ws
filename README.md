# 轮腿式机械狗项目 (dog_ws)

> **16-DOF 轮腿四足机器人 — 全地形自主导航系统**
>
> 平台：Jetson Orin NX (ARM64) · Ubuntu 22.04 · ROS 2 Humble
>
> 最后更新：2026-08-24

---

## 目录

- [项目概述](#项目概述)
- [系统架构](#系统架构)
- [硬件平台](#硬件平台)
- [工作空间结构](#工作空间结构)
- [环境配置与构建](#环境配置与构建)
- [快速启动](#快速启动)
- [模块详解](#模块详解)
  - [dog_brain — 大脑 (建图 + 导航)](#dog_brain--大脑)
  - [dog_description — 机器人模型](#dog_description--机器人模型)
  - [dog_sensors — 传感器驱动](#dog_sensors--传感器驱动)
  - [fast_lio_localization — FAST-LIO2 建图与定位](#fast_lio_localization--fast-lio2-建图与定位)
  - [lcm_bridge — UpBoard 通信桥接](#lcm_bridge--upboard-通信桥接)
  - [livox_ros_driver2 — Livox 雷达驱动](#livox_ros_driver2--livox-雷达驱动)
- [TF 树与数据流](#tf-树与数据流)
- [Topic 速查表](#topic-速查表)
- [关键配置说明](#关键配置说明)
- [典型工作流](#典型工作流)
- [调试与诊断](#调试与诊断)
- [已知问题与注意事项](#已知问题与注意事项)
- [依赖项](#依赖项)

---

## 项目概述

本项目实现了一台 **16 自由度轮腿式四足机器人** 的完整自主导航系统，基于 Mini Cheetah 运动学构型（每条腿 4 个关节：abad 侧摆 → hip 俯仰 → knee 膝 → wheel 轮），运行在 NVIDIA Jetson Orin NX 嵌入式平台上。

**核心能力：**

| 功能 | 方案 |
|------|------|
| **SLAM 建图** | FAST-LIO2（激光惯性紧耦合里程计 + ikd-Tree） |
| **全局定位** | ICP 3D 点云匹配（替换 AMCL 2D，适配全地形） |
| **路径规划** | Nav2 + NavFn（A* 全局规划器） |
| **局部控制** | DWB 局部规划器（差分驱动模型 vx + wz） |
| **代价地图** | 3D Voxel Layer（Livox + D435i 双源融合） |
| **小脑通信** | LCM 组播 + TCP（UpBoard 实时控制） |

**为什么不用 AMCL？** 机械狗全地形导航场景下，3D 雷达和深度相机无法输出标准 2D laser scan，将 3D 点云压扁喂给 AMCL 会在楼梯/坑洼场景严重退化。因此定位方案已全面切换为 ICP 3D 全局定位。

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                     Jetson Orin NX (大脑)                           │
│                                                                     │
│  ┌──────────────┐  ┌──────────────────┐  ┌───────────────────────┐ │
│  │ dog_sensors   │  │ fast_lio_        │  │ dog_brain             │ │
│  │              │  │ localization     │  │                       │ │
│  │ ┌──────────┐ │  │                  │  │ ┌───────────────────┐ │ │
│  │ │Mid-360   │─┼──┼→ FAST-LIO2 SLAM │  │ │ Nav2 3D Navigation│ │ │
│  │ │(Livox)   │ │  │ (C++ / ikd-Tree) │  │ │ (DWB + NavFn)     │ │ │
│  │ └──────────┘ │  │                  │  │ └───────────────────┘ │ │
│  │ ┌──────────┐ │  │ ┌──────────────┐ │  │                       │ │
│  │ │D435i     │ │  │ │定位四件套:   │ │  │  config/              │ │
│  │ │(RealSense│ │  │ │① map_pub     │ │  │  nav2_3d_params.yaml  │ │
│  │ └──────────┘ │  │ │② fastlio     │ │  │                       │ │
│  └──────────────┘  │ │③ global_loc  │ │  └───────────────────────┘ │
│                    │ │④ tf_fusion   │ │                            │
│  ┌──────────────┐  │ └──────────────┘ │  ┌───────────────────────┐ │
│  │ lcm_bridge    │  └──────────────────┘  │ dog_description       │ │
│  │              │                          │                       │ │
│  │ LCM←→ROS2    │                          │ URDF + STL meshes     │ │
│  │ TCP→UpBoard  │                          │ (16-DOF wheel-legged) │ │
│  └──────┬───────┘                          └───────────────────────┘ │
│         │                                                           │
└─────────┼───────────────────────────────────────────────────────────┘
          │ LCM (udpm) + TCP (port 3333)
          ▼
┌──────────────────┐
│  UpBoard (小脑)   │
│  10.0.0.6        │
│  运动控制 / 步态  │
│  关节状态 / IMU   │
└──────────────────┘
```

---

## 硬件平台

| 组件 | 型号/规格 | 说明 |
|------|-----------|------|
| **大脑** | NVIDIA Jetson Orin NX | ARM64, Ubuntu 22.04, ROS 2 Humble |
| **小脑** | UpBoard | x86, 实时运动控制 |
| **3D 雷达** | Livox Mid-360 | 360° FOV, 10 Hz 点云 + 200 Hz IMU (BMI088) |
| **深度相机** | Intel RealSense D435i | 640×480@30Hz 深度+彩色, USB 3.0 |
| **机器人本体** | 16-DOF 轮腿四足 | Mini Cheetah 构型, 4 腿 × 4 关节 |
| **关节执行器** | 每腿: abad + hip + knee + wheel | 轮子为 continuous 关节 |

**关键 IP 地址：**

| 设备 | IP | 备注 |
|------|----|------|
| Jetson Orin NX | `192.168.31.91` | 大脑，ROS_DOMAIN_ID=11，WiFi 静态 IP |
| Jetson WiFi 辅助 IP | `192.168.1.100/24` | Mid-360 雷达通信 (nmcli 已配置) |
| UpBoard 小脑 | `10.0.0.6:3333` (TCP) | 通过 `enP8p1s0` 直连 |
| Mid-360 雷达 | `192.168.1.195` | 配置 host IP `192.168.1.100` |
| LCM 组播 | `udpm://239.255.76.67:7667` | ttl=255 |

---

## 工作空间结构

```
dog_ws/
├── src/
│   ├── dog_brain/                  # 大脑：建图/导航 launch + Nav2 配置
│   │   ├── config/
│   │   │   ├── fast_lio_mapping.yaml    # FAST-LIO2 参数占位
│   │   │   └── nav2_3d_params.yaml      # Nav2 3D 导航完整参数
│   │   ├── launch/
│   │   │   ├── bringup_launch.py        # 统一入口 (mode:=mapping|navigation)
│   │   │   ├── mapping_launch.py        # 建图模式
│   │   │   └── navigation_launch.py     # 导航模式 (定位四件套 + Nav2)
│   │   ├── maps/
│   │   │   └── lab_3d_map.pcd           # 3D 全局地图 (导航前置)
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── dog_description/            # 机器人 URDF 描述
│   │   ├── meshes/                      # 17 个 STL 网格文件 (~84MB)
│   │   │   ├── base_link.STL
│   │   │   ├── {FL,FR,RL,RR}_hip_link.STL
│   │   │   ├── {FL,FR,RL,RR}_thigh_link.STL
│   │   │   ├── {FL,FR,RL,RR}_calf_link.STL
│   │   │   └── {FL,FR,RL,RR}_wheel_link.STL
│   │   ├── urdf/
│   │   │   ├── dog.urdf                 # 实物 URDF (STL 网格, 传感器 frame)
│   │   │   ├── dog.urdf.xacro           # 简化 XACRO 版本 (几何体)
│   │   │   └── add_sensor_frames.py     # 传感器 frame 辅助脚本
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── dog_sensors/                # 传感器 launch
│   │   ├── config/
│   │   │   └── mid360_config.json       # Mid-360 网络配置
│   │   ├── launch/
│   │   │   ├── all_sensors_launch.py    # 一键启动所有传感器
│   │   │   ├── d435i_launch.py          # D435i 深度相机
│   │   │   └── mid360_launch.py         # Livox Mid-360 雷达
│   │   ├── dog_sensors/__init__.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── fast_lio_localization/      # FAST-LIO2 建图 + ICP 全局定位
│   │   ├── src/
│   │   │   ├── laserMapping.cpp           # FAST-LIO2 主节点 (C++)
│   │   │   ├── preprocess.cpp/h           # 点云预处理
│   │   ├── include/
│   │   │   ├── ikd-Tree/                  # 增量式 KD-Tree
│   │   │   ├── common_lib.h
│   │   │   ├── so3_math.h
│   │   │   └── Exp_mat.h
│   │   ├── scripts/
│   │   │   ├── global_map_publisher.py    # ① PCD 地图 → /global_map
│   │   │   ├── global_localization.py     # ② ICP 匹配 → /map_to_odom
│   │   │   ├── transform_fusion.py        # ③ 融合 → map→odom TF
│   │   │   ├── pcd_to_map_node.py         # ④ 3D PCD → 2D OccupancyGrid
│   │   │   ├── elevation_map_node.py      # ⑤ 2.5D 高程图 (越障分析)
│   │   │   ├── step_detector_node.py      # ⑥ 实时台阶检测 (D435i)
│   │   │   └── publish_initial_pose.py    # ⑦ 初始位姿发布工具
│   │   ├── config/
│   │   │   ├── mid360.yaml                # Mid-360 FAST-LIO 参数
│   │   │   ├── avia.yaml, horizon.yaml    # 其他雷达配置
│   │   │   ├── velodyne.yaml, ouster64.yaml
│   │   │   └── *_test.yaml                # 测试配置
│   │   ├── launch/
│   │   │   ├── mapping.launch.py
│   │   │   └── velodyne_localization.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── lcm_bridge/                 # UpBoard ↔ ROS2 通信桥接
│   │   ├── lcm_bridge/
│   │   │   ├── bridge_node.py           # 主节点: LCM↔ROS2 + TCP cmd_vel
│   │   │   └── __init__.py
│   │   ├── config/
│   │   │   └── bridge_params.yaml       # LCM/TCP/Topic 配置
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── livox_ros_driver2/          # Livox 3D 雷达 ROS2 驱动
│   │   ├── src/                         # C++ 驱动源码
│   │   ├── config/                      # 各型号雷达 JSON 配置
│   │   ├── 3rdparty/rapidjson/          # RapidJSON 依赖
│   │   ├── launch_ROS2/                 # 官方 launch 文件
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── STARTUP_GUIDE.md            # 启动命令速查指南
│
├── .gitignore
├── map_fresh_20260820_175432.pcd   # 历史地图备份 (工作空间根目录)
├── map_v2_20260820_192409.pcd
└── test.pcd
```

---

## 环境配置与构建

### 前置依赖

```bash
# ROS 2 Humble 基础安装 (已预装)
source /opt/ros/humble/setup.bash

# 额外 Python 依赖
pip install open3d numpy pyyaml lcm
```

### 构建工作空间

```bash
cd /home/nvidia/dog_ws

# 首次构建 (symlink-install: Python/launch/config 修改后无需重 build)
colcon build --symlink-install

# 仅构建特定包
colcon build --symlink-install --packages-select dog_brain dog_sensors lcm_bridge

# 加载环境
source install/setup.bash
```

> **注意：** `dog_ws` 使用 `--symlink-install` 构建。Python 脚本和 launch/config 文件修改后直接生效；**C++ 代码修改后需要重新 build**。

### 每个新终端必须执行

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/dog_ws/install/setup.bash
```

---

## 快速启动

### 一键启动

```bash
# 建图模式: FAST-LIO2 SLAM + 传感器 + LCM 桥接
ros2 launch dog_brain mapping_launch.py

# 导航模式: ICP 全局定位 + Nav2 3D 导航
ros2 launch dog_brain navigation_launch.py

# 自定义地图路径
ros2 launch dog_brain navigation_launch.py map_pcd:=/path/to/map.pcd

# 统一入口 (通过 mode 参数切换)
ros2 launch dog_brain bringup_launch.py mode:=mapping
ros2 launch dog_brain bringup_launch.py mode:=navigation
```

### 分模块启动

```bash
# 仅传感器 (雷达 + 相机)
ros2 launch dog_sensors all_sensors_launch.py

# 仅 D435i 深度相机
ros2 launch dog_sensors d435i_launch.py

# 仅 Livox Mid-360 雷达
ros2 launch dog_sensors mid360_launch.py

# 仅 LCM 桥接
ros2 run lcm_bridge bridge_node --ros-args -p lcm_url:="udpm://239.255.76.67:7667?ttl=255"
```

---

## 模块详解

### dog_brain — 大脑

**职责：** 顶层 launch 编排 + Nav2 导航参数管理

#### Launch 文件

| 文件 | 功能 | 拉起的节点 |
|------|------|-----------|
| `bringup_launch.py` | 统一入口 (`mode:=mapping\|navigation`) | 条件分发到下面两个 |
| `mapping_launch.py` | SLAM 建图模式 | 传感器 + robot_state_pub + lcm_bridge + fastlio_mapping |
| `navigation_launch.py` | 自主导航模式 | 传感器 + robot_state_pub + lcm_bridge + **定位四件套** + **Nav2** |

#### 配置文件

- **`nav2_3d_params.yaml`** — Nav2 完整参数文件 (303 行)，包含：
  - `bt_navigator`: 行为树导航器 (NavigateToPose + NavigateThroughPoses)
  - `controller_server`: DWB 局部规划器 (差分驱动, max_vel_x=0.45 m/s)
  - `local_costmap`: 滚动窗口 4×4m, voxel_layer (Livox + D435i 双源)
  - `global_costmap`: 全图, static_layer + voxel_layer + inflation
  - `planner_server`: NavFn A* 全局规划
  - `behavior_server`: Spin / Backup / Wait 等行为
  - `velocity_smoother`: 速度平滑 (max 0.75 m/s)
  - `waypoint_follower`: 航点导航

- **`fast_lio_mapping.yaml`** — FAST-LIO2 参数占位 (实际参数由 `fast_lio_localization/config/mid360.yaml` 提供)

#### 地图存储

- `maps/lab_3d_map.pcd` — 3D 全局点云地图 (导航模式前置依赖)
- 通过 `ros2 service call /map_save std_srvs/srv/Trigger` 保存

---

### dog_description — 机器人模型

**职责：** 16-DOF 轮腿四足 URDF 描述

#### URDF 结构

```
base_link (躯干 0.729×0.195×0.17m, 27kg)
├── FL: hip(侧摆) → thigh(俯仰) → calf(膝) → wheel(连续旋转)
├── FR: hip(侧摆) → thigh(俯仰) → calf(膝) → wheel(连续旋转)
├── RL: hip(侧摆) → thigh(俯仰) → calf(膝) → wheel(连续旋转)
└── RR: hip(侧摆) → thigh(俯仰) → calf(膝) → wheel(连续旋转)
```

**关节限位：**

| 关节 | 类型 | 限位 (rad) | 力矩 (Nm) |
|------|------|-----------|-----------|
| hip (侧摆) | revolute | ±0.785 | 154 |
| thigh (俯仰) | revolute | -2.705 ~ 0.611 | 260 |
| calf (膝) | revolute | 0.392 ~ 2.615 | 340 |
| wheel (轮) | revolute | 无限 | 52 |

**传感器安装位置 (URDF 中定义)：**

| 传感器 | 安装偏移 (相对 base_link) |
|--------|--------------------------|
| Livox Mid-360 (`livox_frame`) | x=0.45, y=0, z=0.14 |
| D435i (`camera_link`) | x=0.50, y=0, z=0.07, pitch=π (朝后) |

**网格文件：** 17 个 STL 文件 (~84MB)，位于 `meshes/` 目录，包含完整的物理外观模型。

---

### dog_sensors — 传感器驱动

**职责：** 感知传感器的一键 launch 与参数管理

#### Livox Mid-360 雷达

- **Launch:** `mid360_launch.py`
- **输出 Topic:**
  - `/livox/lidar` — PointCloud2 (CustomMsg 格式, 10 Hz, frame=`livox_frame`)
  - `/livox/imu` — sensor_msgs/Imu (BMI088, 200 Hz)
- **驱动:** `livox_ros_driver2` (已合并进 dog_ws)
- **配置:** `config/mid360_config.json`
  - 雷达 IP: `192.168.1.195`
  - Host IP: `192.168.1.100`
  - 端口: cmd=56100, push=56200, point=56300, imu=56400

#### Intel RealSense D435i 深度相机

- **Launch:** `d435i_launch.py`
- **输出 Topic:**
  - `/camera/depth/color/points` — 彩色点云 (~28.9 Hz)
  - `/camera/color/image_raw` — 彩色图像
  - `/camera/depth/image_rect_raw` — 深度图
- **TF frame:** `camera_link` (匹配 URDF)
- **配置要点：**
  - 分辨率: 640×480@30Hz (深度 + 彩色)
  - 点云: ARM NEON 版参数名 `pointcloud__neon_.enable`
  - 红外流已关闭 (避免 v4l2 Frames Timeout)
  - 深度对齐到彩色 (`align_depth.enable: true`)

---

### fast_lio_localization — FAST-LIO2 建图与定位

**职责：** 激光惯性里程计 + ICP 全局定位 + 地图处理

#### C++ 核心节点

| 可执行文件 | 功能 |
|-----------|------|
| `fastlio_mapping` | FAST-LIO2 主节点: LiDAR-IMU 紧耦合里程计, 基于 ikd-Tree 增量式建图 |

**FAST-LIO2 参数 (`config/mid360.yaml`)：**
- 雷达类型: Livox (lidar_type=1), 4 线
- 盲区: 0.5m, 点云降采样: filter_size=0.5
- 外参: T=[-0.011, -0.02329, 0.04412], R=单位阵
- FOV: 360°, 探测距离: 100m
- PCD 保存: 启用 (单文件模式)

#### Python 定位四件套 (导航模式)

| 节点 | 脚本 | 功能 | 输入/输出 |
|------|------|------|----------|
| ① 地图发布 | `global_map_publisher.py` | 加载 PCD → 发布 `/global_map` | PCD 文件 → PointCloud2 |
| ② 全局定位 | `global_localization.py` | ICP 匹配 → `/map_to_odom` | `/cloud_registered` + `/global_map` → Odometry |
| ③ TF 融合 | `transform_fusion.py` | 融合 → `map→odom` TF | `/Odometry` + `/map_to_odom` → TF broadcast |
| ④ 2D 地图 | `pcd_to_map_node.py` | 3D PCD → 2D OccupancyGrid `/map` | PCD 文件 → OccupancyGrid (供 Nav2 static_layer) |

#### 辅助节点

| 节点 | 脚本 | 功能 |
|------|------|------|
| 高程图 | `elevation_map_node.py` | PCD → 2.5D 高程栅格 `/elevation_costmap` (越障/模态切换) |
| 台阶检测 | `step_detector_node.py` | D435i 近距点云 → 前方台阶判断 `/step_ahead` + `/step_height` |
| 初始位姿 | `publish_initial_pose.py` | 命令行工具: 发布 `/initialpose` (x y z yaw pitch roll) |

---

### lcm_bridge — UpBoard 通信桥接

**职责：** LCM ↔ ROS2 双向数据桥接 + TCP 速度指令下发

#### LCM → ROS2 (上行)

| LCM Channel | ROS2 Topic | 数据类型 | 说明 |
|-------------|-----------|---------|------|
| `global_to_robot` | `/odom` | nav_msgs/Odometry | UpBoard 运动学里程计 (xyz + rpy + 速度) |
| `state_estimator` | `/upboard/state_estimator` | sensor_msgs/Imu | 滤波后状态估计 (四元数 + 角速度 + 加速度) |
| `spi_data` | `/joint_states` | sensor_msgs/JointState | 16 关节状态 (位置/速度/力矩) |

#### ROS2 → UpBoard (下行)

| ROS2 Topic | 传输方式 | 格式 | 说明 |
|-----------|---------|------|------|
| `/cmd_vel` | TCP 端口 3333 | 小端 3×double `[flag, yaw_rate, velocity]` | Nav2 速度指令 → UpBoard |

**TCP 速度映射 (2026-08-19 实狗标定)：**
- 线速度: `policy vx = 4.5 × TCP[2]` → `TCP[2] = v / 4.5`
- 角速度: `policy yaw = -2.5 × TCP[1]` → `TCP[1] = -ω / 2.5`
- 发送频率: 50 Hz (持续喂看门狗)
- 加速度斜坡: 线加速度 0.5 m/s², 角加速度 1.0 rad/s²
- 看门狗: cmd_vel 停更 500ms → 自动归零刹停
- 非线性低速映射: 过 deadband (0.075) 补偿

**关节映射 (LCM spi_data → URDF)：**

```
LCM 顺序: FL, FR, RL, RR
关节类型: hip(侧摆), thigh(俯仰), calf(膝), wheel(轮)
```

**配置文件：** `config/bridge_params.yaml`

---

### livox_ros_driver2 — Livox 雷达驱动

**职责：** Livox 3D 激光雷达的 ROS2 设备驱动

- 基于官方 `livox_ros_driver2`，已合并进 dog_ws 统一构建
- 支持 Mid-360 / HAP / AVIA2 / MID360s 等型号
- 输出 CustomMsg 格式点云 (PointXYZRTL)
- 第三方依赖: RapidJSON (已包含在 `3rdparty/`)

---

## TF 树与数据流

### 建图模式 TF 树

```
odom ──(FAST-LIO2)──→ base_link
                        ├── livox_frame    (LiDAR)
                        ├── camera_link    (D435i)
                        ├── FL_hip_Link → FL_thigh_Link → FL_calf_Link → FL_wheel_Link
                        ├── FR_hip_Link → ...
                        ├── RL_hip_Link → ...
                        └── RR_hip_Link → ...
```

### 导航模式 TF 树

```
map ──(transform_fusion)──→ odom ──(FAST-LIO2)──→ base_link
                                                      ├── livox_frame
                                                      ├── camera_link
                                                      └── legs...
```

> **关键设计：** `odom→base_link` TF 由 FAST-LIO2 激光惯性里程计独占提供；`map→odom` TF 由 `transform_fusion` 节点发布。LCM bridge 的 `publish_odom_tf` 默认为 `false`，避免 TF 冲突。

---

## Topic 速查表

### 传感器数据

| Topic | 类型 | 频率 | 来源 | 说明 |
|-------|------|------|------|------|
| `/livox/lidar` | CustomMsg | 10 Hz | Mid-360 | 3D 点云 |
| `/livox/imu` | Imu | 200 Hz | Mid-360 | BMI088 IMU |
| `/camera/depth/color/points` | PointCloud2 | ~29 Hz | D435i | 彩色点云 |
| `/camera/color/image_raw` | Image | 30 Hz | D435i | 彩色图像 |
| `/camera/depth/image_rect_raw` | Image | 30 Hz | D435i | 深度图 |

### 里程计与定位

| Topic | 类型 | 来源 | 说明 |
|-------|------|------|------|
| `/Odometry` | Odometry | FAST-LIO2 | 激光惯性里程计 (odom→base_link) |
| `/odom` | Odometry | lcm_bridge | UpBoard 运动学里程计 |
| `/map_to_odom` | Odometry | global_localization | ICP 匹配结果 |
| `/localization` | Odometry | transform_fusion | 融合后全局定位 (map→base_link) |
| `/global_map` | PointCloud2 | global_map_publisher | 全局 PCD 地图 |

### 导航

| Topic | 类型 | 说明 |
|-------|------|------|
| `/cmd_vel` | Twist | Nav2 速度指令 → TCP → UpBoard |
| `/map` | OccupancyGrid | 2D 占用栅格 (PCD 投影, Nav2 static_layer) |
| `/elevation_costmap` | OccupancyGrid | 2.5D 高程图 (越障分析) |
| `/step_ahead` | Bool | 前方台阶检测标志 |
| `/step_height` | Float32 | 台阶高度估计 |
| `/initialpose` | PoseWithCovarianceStamped | 初始位姿设定 |

### 代价地图

| Topic | 类型 | 说明 |
|-------|------|------|
| `/cloud_registered` | PointCloud2 | FAST-LIO2 全局去畸变点云 |
| `/cloud_registered_body` | PointCloud2 | 机体坐标系点云 (Nav2 voxel_layer 输入) |
| `/cur_scan_in_map` | PointCloud2 | 当前扫描 (odom 系) |
| `/submap` | PointCloud2 | ICP 裁剪子地图 |

### 关节状态

| Topic | 类型 | 说明 |
|-------|------|------|
| `/joint_states` | JointState | 16 关节 (4 腿 × 4 关节, 位置/速度/力矩) |
| `/upboard/state_estimator` | Imu | UpBoard 滤波后状态估计 |

---

## 关键配置说明

### Nav2 3D 参数 (`nav2_3d_params.yaml`)

**运动模型：** 差分驱动 (vx + wz, vy=0)

**速度限制：**
- 最大线速度: 0.45 m/s (DWB), 0.75 m/s (velocity_smoother)
- 最大角速度: 0.35 rad/s (DWB), 1.5 rad/s (smoother)
- 加速度: ax=0.5, az=1.0

**代价地图：**
- 分辨率: 0.05 m
- 机器人 footprint: 0.9×0.5m (`[±0.45, ±0.25]`)
- Voxel 层: voxel_size=0.05, z_voxels=16, origin_z=-0.20
- 膨胀层: inflation_radius=0.45, cost_scaling=2.5
- 双观测源: Livox (`/cloud_registered_body`) + D435i (`/camera/depth/color/points`)
- 物理盲区: 0~0.7m (UpBoard 触地反馈兜底)

**目标容差：** xy=0.25m, yaw=0.3rad

### LCM Bridge 配置 (`bridge_params.yaml`)

```yaml
lcm_url: "udpm://239.255.76.67:7667?ttl=255&ifname=enP8p1s0"
upboard_ip: "10.0.0.6"
upboard_port: 3333
```

### Mid-360 雷达配置 (`mid360_config.json`)

```json
雷达 IP: 192.168.1.195
Host IP: 192.168.1.100
端口: cmd=56100/56101, push=56200/56201, point=56300/56301, imu=56400/56401
```

### FAST-LIO2 参数 (`mid360.yaml`)

- 点云降采样: filter_size_surf=0.5, filter_size_map=0.5
- 外参: T=[-0.011, -0.02329, 0.04412], R=单位阵 (IMU 内置)
- IMU 协方差: acc=0.1, gyr=0.1, bias_acc=0.0001, bias_gyr=0.0001
- FOV: 360°, 探测距离: 100m

---

## 典型工作流

### 1. 建图

```bash
# 环境准备
source /opt/ros/humble/setup.bash
source /home/nvidia/dog_ws/install/setup.bash

# 启动建图模式
ros2 launch dog_brain mapping_launch.py

# ... 遥控机器狗建图 ...

# 保存 3D PCD 地图
ros2 service call /map_save std_srvs/srv/Trigger

# 将生成的 PCD 复制到导航地图路径
cp /path/to/generated.pcd ~/dog_ws/src/dog_brain/maps/lab_3d_map.pcd
```

### 2. 导航

```bash
# 启动导航模式
ros2 launch dog_brain navigation_launch.py

# 设定初始位姿 (在 RViz 中或用命令行工具)
ros2 run fast_lio_localization publish_initial_pose.py 0.0 0.0 0.0 0.0 0.0 0.0

# 通过 RViz2 发送导航目标
rviz2
```

### 3. 调试

```bash
# 查看节点
ros2 node list

# 查看 topic 频率
ros2 topic hz /livox/lidar
ros2 topic hz /camera/depth/color/points
ros2 topic hz /Odometry

# 查看定位状态
ros2 topic echo /map_to_odom --once

# 查看 TF 树
ros2 run tf2_tools view_frames

# 可视化
rviz2
```

---

## 调试与诊断

### 常用命令

```bash
# 查看所有 topic
ros2 topic list

# 查看单条消息
ros2 topic echo /odom --once
ros2 topic echo /joint_states --once

# 检查点云是否正常
ros2 topic hz /livox/lidar          # 应 ≈ 10 Hz
ros2 topic hz /camera/depth/color/points  # 应 ≈ 29 Hz

# 检查定位是否工作
ros2 topic echo /map_to_odom --once  # 应有数据

# TF 监听
ros2 topic echo /tf
```

### 常见问题排查

| 现象 | 可能原因 | 解决方案 |
|------|---------|---------|
| Mid-360 启动报 `bind failed` | 网口网段不匹配 | 确认 `enP8p1s0` IP 与 `mid360_config.json` host_ip 一致 |
| D435i 点云只有 14 Hz | USB 2.0 降级 | 确保使用 USB 3.0 数据线 |
| D435i `Frames Timeout` | 红外流分辨率冲突 | 关闭 Infra1/Infra2 或统一分辨率到 640×480 |
| TF 断裂 | 多节点同时广播 | 确认 `publish_odom_tf=false` (LCM bridge) |
| 导航暴冲/抽搐 | TCP 增益链错误 | 检查 `NAV_V_CHAIN=4.5`, `NAV_W_CHAIN=2.5` 标定值 |
| ICP 定位漂移 | 地图/场景差异大 | 检查 `localization_th` 阈值, 调低 voxel_size |

---

## 已知问题与注意事项

1. **Mid-360 网段配置（已解决）**
   - WiFi 主 IP `192.168.31.91/24` + 辅助 IP `192.168.1.100/24`（雷达通信）
   - 若辅助 IP 丢失，雷达报 `bind failed`，执行:
     `sudo nmcli connection modify Xiaomi_A389 +ipv4.addresses 192.168.1.100/24 && sudo nmcli connection up Xiaomi_A389`

2. **ROS_DOMAIN_ID 隔离**
   - 机械狗默认域 11（~/.bashrc 中配置），与其他项目隔离

3. **Nav2 全链路联调未做**
   - 点云 → 代价地图 voxel_layer 的完整链路需等雷达就位后实测

4. **D435i 红外流**
   - 已关闭。若未来需开启，必须设 `depth_module.infra_profile: '640x480x30'` 与 depth_profile 一致

5. **Livox 驱动已合并**
   - `livox_ros_driver2` 已合并进 `dog_ws`，不再需要单独的 `ws_livox` 工作空间

---

## 依赖项

### ROS 2 包依赖

| 包名 | 类型 | 说明 |
|------|------|------|
| `nav2_bringup` | exec | Nav2 导航框架 |
| `robot_state_publisher` | exec | URDF → TF 广播 |
| `realsense2_camera` | exec | D435i 驱动 (ARM NEON 版) |
| `xacro` | build | XACRO 宏处理 |
| `tf2_ros` | build/exec | TF2 坐标变换 |
| `pcl_conversions` | build | PCL ↔ ROS 转换 |
| `livox_ros_driver2` | build | Livox 雷达驱动 |
| `python3-lcm` | exec | LCM Python 绑定 |

### Python 依赖

| 包 | 用途 |
|----|------|
| `open3d` | 点云处理 (ICP, 体素降采样, PCD 读写) |
| `numpy` | 数值计算 |
| `pyyaml` | 配置文件解析 |
| `lcm` | LCM 通信 |
| `tf_transformations` | 四元数/矩阵变换 |

### 系统依赖

| 库 | 说明 |
|----|------|
| Eigen3 | 线性代数 |
| PCL | 点云库 |
| OpenMP | 并行计算 |
| LCM | 轻量通信中间件 |

---

## 许可证

- `dog_brain`, `dog_description`, `dog_sensors`, `lcm_bridge`: MIT
- `fast_lio_localization`: BSD (基于 LOAM/FAST-LIO)
- `livox_ros_driver2`: MIT (Livox 官方)
