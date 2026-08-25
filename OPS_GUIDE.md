# 机械狗项目运维指南

> 适用对象：项目维护人员 / 后续 Agent  
> 硬件：Jetson Orin NX (大脑) + UpBoard (小脑) + Livox Mid-360 + Intel D435i  
> 最后更新：2026-08-25

---

## 目录

1. [快速启动](#1-快速启动)
2. [网络配置与维护](#2-网络配置与维护)
3. [建图流程](#3-建图流程)
4. [导航流程](#4-导航流程)
5. [systemd 服务管理](#5-systemd-服务管理)
6. [日常运维脚本](#6-日常运维脚本)
7. [故障诊断与处置](#7-故障诊断与处置)
8. [代码修改与构建](#8-代码修改与构建)
9. [远程可视化 (RViz)](#9-远程可视化-rviz)
10. [UpBoard 端操作](#10-upboard-端操作)
11. [安全红线](#11-安全红线)
12. [多机协同与地图共享](#12-多机协同与地图共享)
13. [速查表](#13-速查表)

---

## 1. 快速启动

### 1.1 环境准备

每个新终端必须先 source：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/dog_ws/install/setup.bash
```

> - `ROS_DOMAIN_ID=11` 已在 `~/.bashrc` 中配置，新终端自动生效
> - 工作空间使用 `--symlink-install`：**Python 文件改完直接生效，C++ 文件需重新 build**

### 1.2 启动前检查清单

| 检查项 | 命令 | 预期 |
|--------|------|------|
| WiFi 主 IP | `ip addr show wlP1p1s0` | `192.168.31.91/24` |
| WiFi 辅助 IP | `ip addr show wlP1p1s0` | `192.168.1.100/24` |
| 有线网口 | `ip addr show enP8p1s0` | `10.0.0.48/24` |
| 雷达可达 | `ping -c 2 192.168.1.195` | 通 |
| UpBoard 可达 | `ping -c 2 10.0.0.6` | 通 |
| D435i 已接 | `ls /dev/video*` | 有设备 |
| dog-brain 状态 | `systemctl status dog-brain` | 按需 |

---

## 2. 网络配置与维护

### 2.1 当前网络拓扑

```
                    ┌─────────────────────────────┐
                    │   Jetson Orin NX (大脑)       │
                    │                               │
  WiFi 路由器 ──────┤  wlP1p1s0: 192.168.31.91/24  │
  (192.168.31.x)    │           + 192.168.1.100/24  │──── Mid-360 雷达
                    │                 (辅助 IP)      │     (192.168.1.195)
                    │                               │
                    │  enP8p1s0: 10.0.0.48/24  ─────┼──── UpBoard 小脑
                    │                               │     (10.0.0.6)
                    └─────────────────────────────┘
```

### 2.2 WiFi 静态 IP 配置

已用 nmcli 持久化，重启自动生效：

```bash
# 查看当前配置
nmcli -t -f ipv4.addresses,ipv4.method connection show Xiaomi_A389

# 若需重新设置（一般不需要）
sudo nmcli connection modify Xiaomi_A389 \
  ipv4.method manual \
  ipv4.addresses "192.168.31.91/24, 192.168.1.100/24" \
  ipv4.gateway 192.168.31.1 \
  ipv4.dns "192.168.31.1,8.8.8.8"
sudo nmcli connection up Xiaomi_A389
```

### 2.3 辅助 IP 丢失修复

**症状**：启动 mid360 报 `bind failed` / `Failed to init livox lidar sdk`

**修复**：
```bash
sudo nmcli connection modify Xiaomi_A389 +ipv4.addresses 192.168.1.100/24
sudo nmcli connection up Xiaomi_A389
```

### 2.4 LCM 组播绑定

LCM 组播绑定在 `enP8p1s0`（有线网口），已在 `bridge_params.yaml` 中写死 `ifname=enP8p1s0`，无需额外配置。

---

## 3. 建图流程

### 3.1 启动建图

**方式一：systemd（推荐）**
```bash
sudo systemctl start dog-brain
# 等待 12s (ExecStartPre sleep) + 约 10s 雷达初始化
```

**方式二：手动启动**
```bash
bash /home/nvidia/restart_stack.sh
# 或
source /opt/ros/humble/setup.bash
source /home/nvidia/dog_ws/install/setup.bash
ros2 launch dog_brain bringup_launch.py mode:=mapping
```

### 3.2 建图操作步骤

```
1. 启动建图栈 → 静止 5~10s 让 IMU 初始化
2. 遥控/推动狗走一圈（建图区域）
3. RViz 中观察 /cloud_registered 点云是否正常（禁订 /Laser_map！）
4. 存图: ros2 service call /map_save std_srvs/srv/Trigger
5. 重命名保留: mv test.pcd map_$(date +%Y%m%d_%H%M).pcd
6. 如需继续建图，重复 2-5（每次存图覆盖 test.pcd）
```

### 3.3 存图后部署到导航

```bash
# 将建图产出的 PCD 复制到导航地图位置
cp map_*.pcd /home/nvidia/dog_ws/src/dog_brain/maps/lab_3d_map.pcd

# symlink-install 不链接 build 后新增文件，需手动同步到 install
ln -sf /home/nvidia/dog_ws/src/dog_brain/maps/lab_3d_map.pcd \
       /home/nvidia/dog_ws/install/dog_brain/share/dog_brain/maps/lab_3d_map.pcd
```

### 3.4 建图注意事项

- **远程 RViz 禁订 `/Laser_map`**（1.17GB 洪流会饿死 /tf 并触发雷达失同步）
- 远程只看 `/cloud_registered`（~127KB/帧，安全）
- 建图途中务必先 `/map_save` 存增量，再停栈（否则进程内地图丢失）

---

## 4. 导航流程

### 4.1 前置条件

- [x] 已有 PCD 地图 (`src/dog_brain/maps/lab_3d_map.pcd`)
- [x] install 路径 symlink 已建立
- [x] WiFi 双 IP 正常

### 4.2 启动导航

**推荐方式：使用清理脚本**
```bash
bash /home/nvidia/nav_restart.sh
# 自动清理所有残留节点 → 启动导航模式
# 日志输出到 /home/nvidia/bringup_nav.log
```

**手动方式**
```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/dog_ws/install/setup.bash
ros2 launch dog_brain bringup_launch.py mode:=navigation
```

### 4.3 导航操作步骤

```
1. 启动导航栈 → 等 60s 让 FAST-LIO 初始化
2. 确认 /Odometry 有数据 (~10Hz):
   ros2 topic hz /Odometry
3. 标定初始位姿（二选一）:
   a. RViz 中点击 "2D Pose Estimate" 在地图上标定狗的位置
   b. 命令行: ros2 run fast_lio_localization publish_initial_pose.py <x> <y> 0.45 <yaw> 0 0
4. 确认 ICP 定位收敛:
   ros2 topic echo /map_to_odom --once   # fitness > 0.9 即收敛
5. 遥控器切换到导航档: SWE(急停 off) → SWG(站立 RL) → SWF(导航档 1800)
6. RViz 中点击 "2D Goal Pose" 发导航目标
```

### 4.4 导航注意事项

- **不要用 sudo 启动**（root 身份下 Python 找不到 tf_transformations）
- 每次重拉栈后必须重新给 initialpose
- 遥控器 SWF 通道(ch10) = 导航档，1800μs

---

## 5. systemd 服务管理

### 5.1 dog-brain.service（建图模式）

```bash
# 查看状态
sudo systemctl status dog-brain

# 启动/停止/重启
sudo systemctl start dog-brain
sudo systemctl stop dog-brain
sudo systemctl restart dog-brain      # 失同步时一键恢复

# 开机自启管理
sudo systemctl enable dog-brain       # 启用开机自启
sudo systemctl disable dog-brain      # 取消开机自启

# 查看日志
journalctl -u dog-brain -f            # 实时跟踪
journalctl -u dog-brain -n 60         # 最近 60 行
```

**服务配置**（`/etc/systemd/system/dog-brain.service`）：
- 用户: nvidia
- 模式: mapping（建图）
- ExecStartPre: sleep 12（等网络/雷达就绪）
- Restart: on-failure（崩溃自动重启，间隔 8s）
- TimeoutStopSec: 20

### 5.2 导航模式 systemd（待建）

当前导航模式没有 systemd 服务，需手动启动：
```bash
bash /home/nvidia/nav_restart.sh
```

---

## 6. 日常运维脚本

| 脚本 | 路径 | 功能 | 使用场景 |
|------|------|------|---------|
| `restart_stack.sh` | `/home/nvidia/` | 清理残留 + 重拉建图栈 | 建图模式重启 |
| `stop_stack.sh` | `/home/nvidia/` | 清理所有 ROS 节点 | 完全停止 |
| `nav_restart.sh` | `/home/nvidia/` | 彻底清理 + 重拉导航栈 | 导航模式重启 |
| `watchdog_run.sh` | `/home/nvidia/` | 看门狗测试脚本 | 测试用 |

### 6.1 脚本工作原理

所有重启脚本遵循相同模式：
```
1. pkill -f 各节点进程名（精确匹配，避免误杀 SSH）
2. sleep 2（等待进程退出）
3. pgrep -af 检查残留
4. source ROS 环境
5. nohup setsid ros2 launch ... &（后台启动，脱离 SSH 会话）
```

### 6.2 ⚠️ 为什么不能在 SSH 命令行直接 pkill

```bash
# ❌ 危险！会匹配并误杀自己的 SSH 会话
pkill -f bringup_launch.py

# ✅ 正确：使用脚本文件（脚本内 pkill 不影响调用者）
bash /home/nvidia/restart_stack.sh
```

---

## 7. 故障诊断与处置

### 7.1 故障速查表

| 故障 | 症状 | 处置 |
|------|------|------|
| **雷达冷启动卡死** | LED 亮、ping 通、驱动 init success，但点数=0 | `sudo systemctl restart dog-brain` + 静止 60s |
| **失同步** | FAST-LIO 狂刷 `not Synced` + `No Effective`，位姿发散 | 先 `/map_save`（建图中），再 restart |
| **ICP 定位丢失** | fitness=0.000，`Robot is out of bounds` | 重新给 initialpose |
| **雷达辅助 IP 丢失** | mid360 报 `bind failed` | 重配辅助 IP（见 §2.3） |
| **DDS 抽风** | ros2 topic echo/hz 空返回 | 重试 2-3 次 |
| **相机断流** | local_costmap 报 camera buffer 延迟 | 检查 transform_tolerance 是否 2.0 |
| **/global_map 读到 0 点** | ICP 启动但 fitness=0 | 检查 install 路径 symlink |
| **Nav2 服务器不 active** | 启动后部分服务器 crash | 检查插件类名、BT 库、观测源 |
| **锁存话题过期** | /map /costmap 有数据但已停止发布 | 先验新鲜度（header.stamp vs 当前时间） |
| **QoS 不匹配** | 订阅不到 /map 等锁存话题 | 订阅端用 TRANSIENT_LOCAL + RELIABLE |

### 7.2 失同步详细处置

```bash
# 1. 诊断
journalctl -u dog-brain -n 60 | grep -iE "not Synced|No Effective"

# 2. 如果正在建图，先存图
ros2 service call /map_save std_srvs/srv/Trigger

# 3. 重启
sudo systemctl restart dog-brain
# 或
bash /home/nvidia/restart_stack.sh
```

### 7.3 ICP 定位收敛检查

```bash
# 查看定位结果（fitness > 0.9 为收敛）
ros2 topic echo /map_to_odom nav_msgs/msg/Odometry --once

# 查看 ICP 节点日志
ros2 topic hz /map_to_odom   # 应为 ~1Hz
```

### 7.4 雷达数据检查

```bash
# 雷达点云频率（应 ~10Hz）
ros2 topic hz /livox/lidar

# IMU 频率（应 ~200Hz）
ros2 topic hz /livox/imu

# 点云消息数
ros2 topic echo /livox/lidar --once | grep -c "x:"
```

---

## 8. 代码修改与构建

### 8.1 构建命令

```bash
cd /home/nvidia/dog_ws
colcon build --symlink-install

# 只构建特定包
colcon build --symlink-install --packages-select dog_brain
colcon build --symlink-install --packages-select fast_lio_localization
```

### 8.2 修改生效规则

| 文件类型 | 修改后是否需 rebuild | 说明 |
|---------|-------------------|------|
| Python 脚本 (.py) | ❌ 不需要 | symlink-install 直接链接源文件 |
| Launch 文件 (.py) | ❌ 不需要 | 同上 |
| Config 文件 (.yaml/.json) | ❌ 不需要 | 同上 |
| URDF 文件 (.urdf) | ❌ 不需要 | 同上 |
| C++ 源文件 (.cpp/.h) | ✅ 需要 | `colcon build --symlink-install` |
| CMakeLists.txt | ✅ 需要 | 同上 |

### 8.3 C++ 修改后

```bash
cd /home/nvidia/dog_ws
colcon build --symlink-install --packages-select livox_ros_driver2
# 或
colcon build --symlink-install --packages-select fast_lio_localization
source install/setup.bash
```

### 8.4 Git 操作

```bash
cd /home/nvidia/dog_ws
git status
git diff <file>
git add <file>
git commit -m "描述"
git push origin main    # ⚠️ 需用户明确批准
```

> **铁律**：不要擅自 git push，除非用户明确说"推送/提交到远程"。

---

## 9. 远程可视化 (RViz)

### 9.1 VM 连接 Jetson

- Parallels 必须 **「桥接」网络**（不要用共享/NAT）
- VM 需拿到 `192.168.31.x` 网段
- 确认 `ping 192.168.31.91` 通
- `ROS_DOMAIN_ID=11`（与 Jetson 一致）
- VM 上需先 `colcon build --symlink-install` 过 `dog_description`

### 9.2 RViz 推荐配置

**建图模式：**
```
Fixed Frame: odom
- TF
- RobotModel (/robot_description)
- PointCloud2: /cloud_registered (实时, ~127KB/帧)
- Path: /path
```

**导航模式：**
```
Fixed Frame: map
- TF
- RobotModel
- PointCloud2: /cloud_registered
- Map: /map (2D OccupancyGrid)
- Map: /global_costmap/costmap_raw
- Map: /local_costmap/costmap_raw
- Path: /plan, /local_plan
```

### 9.3 ⚠️ 远程 RViz 禁令

**绝对不要远程订阅 `/Laser_map`！**

- 单条消息 73M 点 ≈ 1.17GB
- 会打满网络链路 → /tf 被饿死 → 所有帧消失
- 可能压垮 USB 雷达链路 → 失同步 → 建图发散

替代方案：
1. Jetson 本地 RViz 看 /Laser_map
2. 存 PCD 拷回 VM 离线看
3. 远程只看 /cloud_registered（安全）

---

## 10. UpBoard 端操作

### 10.1 SSH 连接（经 Jetson 跳板）

```bash
sshpass -p xjzx ssh -J nvidia@192.168.31.91 xjzx@10.0.0.6
# 或从 Jetson 上:
ssh xjzx@10.0.0.6   # 密码: xjzx
```

### 10.2 UpBoard 控制程序

```bash
cd /home/xjzx/robot-software
./run_mc.sh ./mit_ctrl m r f
# m=mode(RL), r=run, f=fast
```

### 10.3 遥控器通道说明

| 通道 | 开关 | 功能 |
|------|------|------|
| ch4 (SWE) | 急停 | 始终有效，任何时候可急停 |
| ch5 (SWG) | 站立 RL | 切换到 RL 策略控制 |
| ch10 (SWF) | 导航档 | 1800μs = 导航模式，TCP 指令生效 |

---

## 11. 安全红线

### 11.1 绝对不可违反

1. **动狗/UpBoard 前**：备份 + 狗趴下 + 人盯 + 遥控器在手（SWE 急停始终有效）
2. **不擅自 git push**（除非用户明确批准）
3. **UpBoard 官方代码尽量不动**

### 11.2 操作规范

| 规则 | 说明 |
|------|------|
| pkill 必须写脚本 | 不能在 SSH 命令行直接 pkill -f（会杀自己 SSH） |
| 不要用 sudo 启动导航 | root 身份下 Python 找不到 tf_transformations 包 |
| 启动后等 60s | 让雷达/FAST-LIO 初始化完成 |
| 每次重拉栈后给 initialpose | ICP 定位需手动触发 |
| 服务调用三步走 | list → type → call，禁止凭记忆猜 |
| 单次 topic echo 失败重试 | 重试 2-3 次再下结论（DDS 间歇抽风） |
| 改配置前先备份 | 一次只改一个逻辑变量 |
| 新脚本必须 chmod +x | colcon install(PROGRAMS) 要求可执行 |

---

## 12. 多机协同与地图共享

### 12.1 系统架构

| 车辆 | IP | Domain | 定位方式 | 地图格式 | 地图路径 |
|------|-----|--------|---------|----------|----------|
| 主车 | 192.168.31.43 (nvidia) | 1 | RTAB-Map ICP (`Reg/Strategy=1`) | `my_room.db` + `my_map.yaml/pgm` | `~/wheeltec_ros2/src/wheeltec_robot_rtab/` |
| robot1 小车 | 192.168.31.47 (sunrise) | 11 | AMCL | 2D 栅格 | `~/robot1_ws/src/robot1_nav/maps/` |
| 机械狗 (robot2) | 192.168.31.91 (nvidia) | 11 | FAST-LIO2 + ICP | 3D PCD + 2D 栅格 | `~/dog_ws/maps/` |

**通信架构：**
```
主车 (Domain 1) ← domain_bridge → robot1/robot2 (Domain 11)
```

桥接的 6 个话题 (每车):
- `/robot_N/soldier_pose` (11→1): 位姿上报
- `/robot_N/odom` (11→1): 里程计上报  
- `/robot_N/battery_state` (11→1): 电池状态
- `/robot_N/cmd_vel` (1→11): 速度控制下行
- `/robot_N/goal_pose` (1→11): 导航目标下行
- `/robot_N/initialpose` (1→11): 初始位姿下行

### 12.2 地图共享方案

**策略：机械狗建图 → 多格式导出 → 三车共用**

```
机械狗 Mid-360 建图 (精度最高)
  │
  ├─ 3D PCD → 机械狗自己用 (ICP 定位)
  │
  └─ 2D OccupancyGrid (export_2d_map.py 导出)
       ├─ 主车用: my_map.yaml/pgm (Nav2 导航)
       ├─ robot1 用: 2D 地图 (AMCL 定位)
       └─ 主车 .db: 同一原点重建 (RTAB-Map 定位)
```

**关键：所有车从同一物理原点出发建图，坐标系自动对齐。**

### 12.3 地图导出工具

```bash
# 基本用法: 从默认路径读取，输出到 ~/dog_ws/maps/
python3 ~/dog_ws/src/fast_lio_localization/scripts/export_2d_map.py

# 指定输入输出，命名为主车的 my_map
python3 ~/dog_ws/src/fast_lio_localization/scripts/export_2d_map.py \
  --pcd ~/dog_ws/src/dog_brain/maps/lab_3d_map.pcd \
  --output ~/dog_ws/maps --name my_map

# 直接推送到主车地图目录 (自动 SCP)
python3 ~/dog_ws/src/fast_lio_localization/scripts/export_2d_map.py \
  --pcd map.pcd --push-to-main

# 推送到 robot1 地图目录 (命名需为 lab_map)
python3 ~/dog_ws/src/fast_lio_localization/scripts/export_2d_map.py \
  --pcd map.pcd --name lab_map --push-to-robot1

# 推送到所有车辆 (主车 + robot1)
python3 ~/dog_ws/src/fast_lio_localization/scripts/export_2d_map.py \
  --pcd map.pcd --push-to-all
```

**三车地图路径：**
```
主车:   ~/wheeltec_ros2/src/wheeltec_robot_rtab/my_map.yaml + my_map.pgm
robot1: ~/robot1_ws/src/robot1_nav/maps/lab_map.yaml + lab_map.pgm
机械狗: ~/dog_ws/maps/ (本地导出)
```

### 12.4 多机协同服务管理

**机械狗端 (192.168.31.91)：**
```bash
systemctl --user status robot2-soldier.service
systemctl --user restart robot2-soldier.service
```

**主车端 (192.168.31.43)：**
```bash
systemctl --user status robot2-domain-bridge.service
systemctl --user restart robot2-domain-bridge.service
```

### 12.5 主车包使用状态

**正在使用：**
- `largemodel` — 大模型服务 (model_service, action_service)
- `mqtt_bridge_ros2` — MQTT 桥 (system_manager + webrtc + web_console)
- `turn_on_wheeltec_robot` — 底盘驱动 + 传感器
- `wheeltec_lidar_ros2` — lslidar 激光雷达
- `wheeltec_robot_rtab` — RTAB-Map + Nav2 导航
- `navigation2-humble` — Nav2 核心
- `domain_bridge` — 多车桥接

**未使用/备用：**
- `wheeltec_robot_nav2` — 旧版导航 (WHEELTEC.yaml)
- `wheeltec_multi` — 旧版多车方案
- `wheeltec_robot_slam` — 旧版 SLAM
- 其他: aruco, auto_recharge, ollama, tts, usb_cam 等

---

## 13. 速查表

### 13.1 IP / 端口 / Topic

| 项 | 值 |
|----|----|
| Jetson SSH | `nvidia@192.168.31.91` |
| Jetson WiFi 主 IP | `192.168.31.91/24` |
| Jetson WiFi 辅助 IP | `192.168.1.100/24` |
| UpBoard | `10.0.0.6` (TCP :3333) |
| Mid-360 雷达 | `192.168.1.195` |
| LCM 组播 | `udpm://239.255.76.67:7667` |
| ROS_DOMAIN_ID | `11` |
| `/livox/lidar` | 雷达点云 (CustomMsg, 10Hz) |
| `/livox/imu` | 雷达 IMU (Imu, 200Hz) |
| `/camera/depth/color/points` | 相机点云 (PointCloud2, ~28.9Hz) |
| `/Odometry` | FAST-LIO 里程计 (10Hz) |
| `/odom` | UpBoard 里程计 (500Hz, 不进 SLAM) |
| `/cloud_registered` | 去畸变点云 (10Hz, ~127KB/帧) |
| `/map` | 2D 栅格地图 (OccupancyGrid) |
| `/cmd_vel` | 速度指令 (Twist) |
| `/map_to_odom` | ICP 定位结果 (1Hz) |
| `/joint_states` | 16 关节状态 (500Hz) |

### 13.2 常用命令

```bash
# 环境
source /opt/ros/humble/setup.bash && source /home/nvidia/dog_ws/install/setup.bash

# 建图
bash /home/nvidia/restart_stack.sh           # 重拉建图栈
bash /home/nvidia/stop_stack.sh              # 停止所有节点
sudo systemctl restart dog-brain             # systemd 重启建图

# 导航
bash /home/nvidia/nav_restart.sh             # 重拉导航栈

# 存图
ros2 service call /map_save std_srvs/srv/Trigger

# 定位
ros2 run fast_lio_localization publish_initial_pose.py 0 0 0.45 0 0 0

# 调试
ros2 topic list                              # 所有 topic
ros2 topic hz /Odometry                      # 频率
ros2 topic echo /map_to_odom --once          # 单条消息
ros2 node list                               # 所有节点
ros2 run tf2_tools view_frames               # TF 树

# 网络
ip -br addr show                             # 所有接口 IP
ping -c 2 192.168.1.195                      # 雷达
ping -c 2 10.0.0.6                           # UpBoard
nmcli connection show                        # 连接状态
```

### 13.3 文件位置

| 文件 | 路径 |
|------|------|
| 工作空间 | `/home/nvidia/dog_ws` |
| 导航地图 | `src/dog_brain/maps/lab_3d_map.pcd` |
| FAST-LIO 存图 | `test.pcd`（工作目录） |
| Nav2 配置 | `src/dog_brain/config/nav2_3d_params.yaml` |
| 雷达配置 | `src/dog_sensors/config/mid360_config.json` |
| FAST-LIO 配置 | `src/fast_lio_localization/config/mid360.yaml` |
| URDF | `src/dog_description/urdf/dog.urdf` |
| systemd 服务 | `/etc/systemd/system/dog-brain.service` |
| 建图日志 | `/home/nvidia/bringup_test.log` |
| 导航日志 | `/home/nvidia/bringup_nav.log` |
| bashrc | `~/.bashrc`（含 ROS_DOMAIN_ID=11） |
