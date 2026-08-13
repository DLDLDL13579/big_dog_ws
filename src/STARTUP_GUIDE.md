# 机械狗 (dog_ws) 常用启动命令速查指南

> 适用设备：Jetson Orin NX `nvidia@192.168.1.48`（ARM64 / Ubuntu 22.04 / ROS Humble）
> 工作空间：`/home/nvidia/dog_ws`（**单工作空间，所有包含雷达驱动已合并**）
> 最后更新：2026-08-13（定位架构已切换为 ICP 3D 全局定位）

---

## 0. 环境准备（每个新终端都要先 source）

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/dog_ws/install/setup.bash
```

> 说明：
> - `dog_ws` 是 `--symlink-install` 构建，改 `src/` 下的 launch/config 直接生效（**Python 无需重 build；C++ 需重 build**）。
> - **Livox 雷达驱动已合并进 dog_ws**（`src/livox_ros_driver2`），不再需要单独的 `ws_livox`。
> - 机械狗默认 `ROS_DOMAIN_ID=0`（不设置即可）；与小车1项目（`ROS_DOMAIN_ID=1`）隔离，勿混。

---

## 1. 一键启动（推荐入口）

```bash
# 建图模式：FAST-LIO2 SLAM + 传感器 + LCM 桥接
ros2 launch dog_brain mapping_launch.py

# 导航模式：FAST-LIO-Localization (ICP 3D 定位) + Nav2 3D 导航
ros2 launch dog_brain navigation_launch.py
```

> ⚠️ **定位方案已切换（2026-08-13）**：彻底移除 AMCL 2D 粒子滤波，绝对定位权交给
> FAST-LIO-Localization 的 ICP 3D 全局定位四件套。理由：机械狗全地形导航，3D 雷达/深度相机
> 无 2D scan 输出，压扁喂 AMCL 会在楼梯/坑洼场景退化。
>
> 定位四件套（navigation_launch.py 自动拉起）：
> - `fastlio_mapping`（FAST-LIO2 激光惯性里程计 → `/Odometry` + `odom→base_link` TF）
> - `global_map_publisher.py`（PCD 地图 → `/global_map`）
> - `global_localization.py`（ICP 匹配 → `/map_to_odom`）
> - `transform_fusion.py`（融合 → `map→odom` TF，**绝对定位权**）

---

## 2. 分模块启动

### 2.1 传感器（雷达 + 相机一键）

```bash
ros2 launch dog_sensors all_sensors_launch.py
```

### 2.2 单独启动 D435i 深度相机

```bash
ros2 launch dog_sensors d435i_launch.py
```

- 点云 topic：`/camera/depth/color/points`（约 28.9 Hz）
- 彩色图：`/camera/color/image_raw`
- 深度图：`/camera/depth/image_rect_raw`
- TF 根 frame：`camera_link`（匹配 URDF）
- 红外流(Infra1/2)已关闭（根因见 d435i_launch.py 文件头注释）

### 2.3 单独启动 Livox Mid-360 雷达

```bash
ros2 launch dog_sensors mid360_launch.py
```

- 点云：`/livox/lidar`（PointCloud2，10 Hz，frame_id=livox_frame）
- IMU：`/livox/imu`（BMI088，200 Hz）
- 驱动：`livox_ros_driver2`（已在 dog_ws 内）

### 2.4 建图（FAST-LIO2）

```bash
ros2 launch dog_brain mapping_launch.py
```

- 依赖 `fast_lio_localization` 包的 `fastlio_mapping`（✅ 已编译）
- 参数：`fast_lio_localization/config/mid360.yaml`（topic `/livox/lidar` + `/livox/imu`）

### 2.5 导航（FAST-LIO-Loc + Nav2）

```bash
ros2 launch dog_brain navigation_launch.py
# 自定义地图路径
ros2 launch dog_brain navigation_launch.py map_pcd:=/path/to/map.pcd
```

- 依赖：`fast_lio_localization` 四件套 + `nav2_bringup`（✅ 已编译）
- 前置：需有 `dog_brain/maps/lab_3d_map.pcd` 地图文件

### 2.6 LCM 桥接（UpBoard ↔ ROS2）

```bash
ros2 run lcm_bridge bridge_node --ros-args -p lcm_url:="udpm://239.255.76.67:7667?ttl=255"
```

（通常已由 `mapping/navigation_launch.py` 自动拉起，无需单独跑）

---

## 3. 常用调试命令

```bash
# 查看所有 topic / node
ros2 topic list
ros2 node list

# 查看点云/雷达频率
ros2 topic hz /camera/depth/color/points
ros2 topic hz /livox/lidar

# 查看单条消息
ros2 topic echo /odom --once
ros2 topic echo /joint_states --once

# 查看 TF 树
ros2 run tf2_tools view_frames
# 或实时监听 TF
ros2 topic echo /tf

# 查看定位是否工作（map→odom TF 由 transform_fusion 发布）
ros2 topic echo /map_to_odom --once

# 可视化
rviz2
```

---

## 4. 关键 IP / 端口 / 话题速查

| 项 | 值 |
|----|----|
| Jetson 大脑 | `nvidia@192.168.1.48` |
| UpBoard 小脑 | `10.0.0.6:3333`（TCP） |
| LCM 组播 | `udpm://239.255.76.67:7667` |
| Mid-360 雷达默认 IP | `192.168.1.12` |
| Mid-360 host 配置 IP | `192.168.1.50`（见 mid360_config.json） |
| 相机点云 topic | `/camera/depth/color/points` |
| 雷达点云 topic | `/livox/lidar` |
| 雷达 IMU topic | `/livox/imu` |
| 激光里程计 topic | `/Odometry`（FAST-LIO2） |
| 运动学里程计 topic | `/odom`（lcm_bridge） |
| 全局地图 topic | `/global_map` |
| 关节状态 topic | `/joint_states` |
| 速度指令 topic | `/cmd_vel` |

---

## 5. 已知阻塞项 / 注意事项

1. **Mid-360 网口网段冲突**（未解决）
   - 当前 `enP8p1s0` = `10.0.0.48/24`（连 UpBoard 的网段）
   - 但 `mid360_config.json` 里 host 要求 `192.168.1.50`，雷达默认 `192.168.1.12`
   - 两个网段不一致 → 需确认 Mid-360 接哪个网口、是否需配双 IP 或改配置
   - 症状：启动 mid360 报 `bind failed` / `Failed to init livox lidar sdk`

2. **地图文件缺失**（导航模式前置）
   - 导航需要 `dog_brain/maps/lab_3d_map.pcd`（由建图模式生成后保存）

3. **D435i 相机**
   - 必须插 USB 3.0 数据线（否则降到 USB 2.1，点云只有 14 Hz）
   - 红外流(Infra1/2)已关闭；若未来需开启，必须设 `depth_module.infra_profile: '640x480x30'` 与 `depth_profile` 一致，否则触发 v4l2 Frames Timeout

4. **ROS_DOMAIN_ID 隔离**
   - 机械狗默认域 0，小车1 项目用域 1，注意区分

5. **Nav2 全链路联调未做**
   - 点云 → 代价地图 voxel_layer 的完整链路需等雷达就位后实测

---

## 6. 典型工作流

```bash
# ① 建图
source /opt/ros/humble/setup.bash
source /home/nvidia/dog_ws/install/setup.bash
ros2 launch dog_brain mapping_launch.py
# ... 推动机器狗建图 ...

# ② 保存 3D 地图（PCD）
ros2 service call /map_save std_srvs/srv/Trigger
# 把生成的 PCD 放到 dog_brain/maps/lab_3d_map.pcd

# ③ 导航
ros2 launch dog_brain navigation_launch.py
```

---

## 附：工作空间结构

```
dog_ws/src/
├── dog_brain/             # 大脑：FAST-LIO2 建图 + Nav2 3D 导航 (launch + config)
├── dog_description/       # URDF 模型 + robot_state_publisher
├── dog_sensors/           # 传感器 launch：D435i + Mid-360 (含 mid360_config.json)
├── lcm_bridge/            # UpBoard ↔ ROS2 桥接
├── livox_ros_driver2/     # Livox 雷达驱动（已合并，Humble 适配版）
└── fast_lio_localization/ # FAST-LIO2 建图 + ICP 全局定位四件套（myeongw002 通用版）
```
