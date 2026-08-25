# 机械狗 z1w_v2 项目当前状态总览

> 生成时间：2026-08-25  
> 工作空间：`/home/nvidia/dog_ws`  
> 仓库：https://github.com/DLDLDL13579/big_dog_ws.git (origin/main)

---

## 0. 一句话概要

**16-DOF 轮腿式机械狗（Mini Cheetah 构型）**，Jetson Orin NX 大脑 + UpBoard 小脑。FAST-LIO2 建图 → ICP 3D 全局定位 → Nav2 3D 导航 → TCP 小端指令 → UpBoard RL 策略驱动运动。**全链路软件栈已打通，导航端到端已验证，待现场实测驱动狗体行走。**

---

## 1. 系统环境

| 项目 | 状态 |
|------|------|
| 硬件平台 | NVIDIA Jetson Orin NX (ARM64, JetPack r36.5.0, kernel 5.15.185-tegra) |
| ROS 2 | Humble Hawksbill |
| ROS_DOMAIN_ID | **11**（~/.bashrc 已设置） |
| 工作空间 | `/home/nvidia/dog_ws`（单工作空间, `--symlink-install`, 6 个包） |
| 构建模式 | `colcon build --symlink-install` |

---

## 2. 网络配置（★ 已更新）

| 接口 | IP | 用途 | 状态 |
|------|-----|------|------|
| `wlP1p1s0` (WiFi) | **192.168.31.91/24**（静态） | 主网络 / SSH / 上网 | ✅ UP |
| `wlP1p1s0` (WiFi 辅助) | **192.168.1.100/24** | Mid-360 雷达通信 | ✅ UP |
| `enP8p1s0` (有线) | 10.0.0.48/24 | UpBoard 底盘直连 | ✅ UP |
| `enx00e04c680779` (USB 网卡) | — | 已停用（雷达已迁移到 WiFi 辅助 IP） | ⬇ DOWN |

**WiFi 双 IP 配置方式（nmcli 已持久化，重启自动生效）：**
```
ipv4.method: manual
ipv4.addresses: 192.168.31.91/24, 192.168.1.100/24
连接名: Xiaomi_A389
```

| 设备 | IP | 通信方式 |
|------|-----|---------|
| UpBoard 小脑 | 10.0.0.6 | LCM 组播 (239.255.76.67:7667, ifname=enP8p1s0) + TCP :3333 |
| Mid-360 雷达 | 192.168.1.195 | 通过 WiFi 辅助 IP 192.168.1.100 通信 |
| D435i 相机 | — | USB3 直连 |

---

## 3. 软件包结构（6 个包）

| 包 | 类型 | 功能 | 关键文件 |
|----|------|------|---------|
| **dog_brain** | ament_cmake | 编排 launch + Nav2 配置 + 地图 | `bringup_launch.py`, `mapping_launch.py`, `navigation_launch.py`, `nav2_3d_params.yaml` |
| **dog_sensors** | ament_python | 传感器 launch 配置 | `mid360_launch.py`, `d435i_launch.py`, `mid360_config.json` |
| **dog_description** | ament_cmake | URDF + STL 模型（19 link / 18 joint） | `dog.urdf` (实物 z1w_v2 CAD) |
| **lcm_bridge** | ament_python | LCM↔ROS2 桥接 + TCP 速度下发 | `bridge_node.py` (TCP 小端 `'<3d'`) |
| **fast_lio_localization** | ament_cmake | FAST-LIO2 建图 + ICP 定位四件套 | `laserMapping.cpp`, `global_localization.py`, `transform_fusion.py`, `pcd_to_map_node.py`, `elevation_map_node.py`, `step_detector_node.py` |
| **livox_ros_driver2** | ament_cmake | Mid-360 雷达驱动（已打时间戳补丁） | `pub_handler.cpp` (软件时间戳补偿) |

---

## 4. 运行模式

### 4.1 建图模式（mapping）
```
传感器 → FAST-LIO2 → odom→base_link TF + /cloud_registered + /Odometry
存图: ros2 service call /map_save std_srvs/srv/Trigger → ./test.pcd
```
- 启动: `ros2 launch dog_brain bringup_launch.py mode:=mapping`
- systemd: `dog-brain.service`（当前 **disabled / inactive**）

### 4.2 导航模式（navigation）
```
传感器 + FAST-LIO2 + 定位四件套 + 地图双流 + 感知 + Nav2 3D
```
- **定位四件套**（替代 AMCL）：global_map_publisher → fastlio_mapping → global_localization → transform_fusion
- **地图双流 + 感知**：pcd_to_map (3D→2D /map) + elevation_map (2.5D 高程图) + step_detector (D435i 台阶检测)
- **Nav2 3D**：Navfn (A* 全局) + DWB (局部) + Voxel Layer 3D 代价地图 + Velocity Smoother
- 启动: `bash /home/nvidia/nav_restart.sh`

---

## 5. 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| max_vel_x | 0.45 m/s | 最大前进速度（降速防飘移） |
| max_vel_theta | 0.35 rad/s | 最大角速度（降速防飘移） |
| inflation_radius | 0.45 m | 膨胀半径 |
| PreferForward.scale | 10.0 | 正向偏好（防倒车） |
| stateful | true | 目标检查器（先到位再对向） |
| xy_goal_tolerance | 0.25 m | 目标到达容差 |
| yaw_goal_tolerance | 0.3 rad | 目标朝向容差 |
| 控制死区 | 0.075 | 官方原版（2026-08-20 回滚） |
| NAV_V_CHAIN | 4.5 | 速度映射系数 |
| NAV_W_CHAIN | 2.5 | 角速度映射系数 |
| NAV_SEND_HZ | 50 | TCP 发送频率 |
| TCP 协议 | `'<3d'` 小端 | flag + ω + v/0.75 |
| LCM 协议 | `'>12f'` 大端 | 里程计/IMU/关节状态 |

---

## 6. 传感器外参（URDF，已实测标定）

| 传感器 | 前方(X) | 侧向(Y) | 高度(Z) |
|--------|---------|---------|---------|
| Livox Mid-360 | 0.45 m | 0 | 0.15 m |
| Intel D435i | 0.50 m | 0 | 0.08 m (roll=π 倒置) |

---

## 7. 地图文件

| 文件 | 大小 | 位置 | 说明 |
|------|------|------|------|
| `lab_3d_map.pcd` | 7.28 MB | `src/dog_brain/maps/` | 导航用 PCD 地图（已就位） |
| `test.pcd` | 7.28 MB | `dog_ws/` | FAST-LIO2 默认存图路径 |
| `map_fresh_20260820_175432.pcd` | 7.28 MB | `dog_ws/` | 历史建图快照 |
| `map_v2_20260820_192409.pcd` | 7.28 MB | `dog_ws/` | 历史建图快照 |

---

## 8. 运维脚本

| 脚本 | 路径 | 功能 |
|------|------|------|
| `restart_stack.sh` | `/home/nvidia/` | 一键重启建图栈（pkill + nohup setsid） |
| `stop_stack.sh` | `/home/nvidia/` | 一键停止建图栈 |
| `nav_restart.sh` | `/home/nvidia/` | 导航模式重拉栈（自动清理全部残留） |
| `watchdog_run.sh` | `/home/nvidia/` | 看门狗脚本 |

---

## 9. Git 状态

### 当前分支
`main` (与 `origin/main` 同步)

### 待提交变更（已 staged，未 commit）
| 文件 | 操作 | 说明 |
|------|------|------|
| `README.md` | 修改 | IP/网络信息更新（WiFi 192.168.31.91 + 双 IP + ROS_DOMAIN_ID=11） |
| `src/STARTUP_GUIDE.md` | 修改 | 同上 |
| `ros项目架构.md` | 新增 | 完整系统架构文档（834 行） |
| `机械狗项目完整文档.md` | 新增 | 合并版完整文档（1140 行） |
| `SYNC_BASELINE.md` | 新增 | 文档同步基线说明 |

### 提交历史（18 commits）
```
e94ef94 docs(readme): 添加轮腿式机械狗项目完整文档
dc38441 config(nav2): 调整导航参数以优化机器人运动性能
a267ddc fix: 传感器优化 - step_detector 降频 10Hz + OPENBLAS、狗体点云屏蔽、URDF 外参修正
8723ffe fix: 导航参数统一 - odom/频率/容差/字节序
e33ac9a fix: ICP 定位修复 - z轴归零、多次标定、降频+OPENBLAS
5011127 feat(perception): 新增实时前方台阶检测节点 step_detector_node
fe95bc8 feat(elevation): 新增 2.5D 高程图节点 elevation_map_node
f164072 feat(navigation): Nav2 导航栈全链路联调修复
ec8b1d5 fix: 修复导航启动 URDF/桥接配置 + 相机图像 180° 倒置
804c490 fix: 修正雷达型号为 Mid360s 并修复建图启动问题
8255cb0 docs: 更新启动指南 - 定位架构切换为 ICP 3D 全局定位
0438598 feat(navigation): ICP 3D 全局定位替换 AMCL
1ff5d2a feat(fast_lio_localization): 集成 FAST-LIO2 建图+定位
57eb17d feat: 合并 livox_ros_driver2 雷达驱动 + 新增启动指南
17d1d00 chore(dog_brain): 移除 slam_toolbox 旧 2D 架构残留
5672f33 fix(dog_sensors): D435i 关闭 Infra 流
5c3ce03 fix(dog_sensors): D435i launch 修复 - 适配 Jetson ARM/NEON
9035a2f fix: lcm_bridge 修复 + 参数更新
3500f11 feat: z1w_v2 机械狗大脑 V1.0-3D 初始提交
```

### 未提交的本地修改（不在 git 中）
- Jetson: `lcm_bridge` 字节序 `'<3d'`（已包含在已提交代码中）
- UpBoard: `rt_rc_interface.cpp` TCP 注入（SWF 双源 + 300ms 看门狗）— **本地副本，未部署**

---

## 10. 完成度评估

| 模块 | 完成度 | 状态 |
|------|--------|------|
| 硬件集成 | **95%** | 三网口打通、LCM/TCP 通信、传感器全部在线 |
| 传感器驱动 | **95%** | Mid-360 10Hz + D435i 28.9Hz + 时间戳 Unix + 外参标定 |
| SLAM 建图 | **85%** | FAST-LIO2 稳定建图、存图服务可用、多张 PCD 产出 |
| ICP 3D 定位 | **80%** | 四件套链路打通、fitness=1.000、多次重定位验证 |
| Nav2 导航 | **80%** | 全链路激活、9 服务器 active、参数调优（降速/防倒车/目标检查） |
| 控制链 | **80%** | TCP 小端已修、RL 策略就绪，**未实测驱动狗体运动** |
| 系统加固 | **30%** | systemd 建图已部署（当前 disabled）、导航 systemd 未建 |
| 文档体系 | **95%** | 4 份完整文档（README + 架构 + 完整文档 + 启动指南） |
| 楼梯间专项 | **0%** | 未开始 |
| **综合** | **~55%** | |

---

## 11. 待办事项

### 🔴 P0 — 关键阻塞
| # | 项目 | 说明 |
|---|------|------|
| 1 | **发 goal 实测驱动狗体运动** | /cmd_vel → TCP → UpBoard 驱动狗未实测，需狗在场 |
| 2 | UpBoard TCP 注入部署 | rt_rc_interface.cpp SWF 双源 + 300ms 看门狗，本地副本已写未部署 |

### 🟡 P1 — 重要待办
| # | 项目 | 说明 |
|---|------|------|
| 3 | 导航参数稳定性验证 | 降速后的导航表现需现场验证 |
| 4 | 导航常驻 systemd | 类似 dog-brain.service 的导航模式自启动 |
| 5 | 全向横移扩展 | Nav2 差分(vy=0) + TCP 无 vy 槽位 → 需 Nav2 全向 + TCP 加第4个 double |
| 6 | step_detector 现场标定 | 台阶检测阈值需 RViz 现场验证 |

### 🟢 P2 — 优化项
| # | 项目 | 说明 |
|---|------|------|
| 7 | 失同步看门狗 | 雷达掉线重连后时钟跳变 → 自动 restart |
| 8 | UpBoard 日志清理 | 689MB 日志占空间 |
| 9 | 建图漂移优化 | 长时间建图的漂移控制 |
| 10 | git push | 3 批次已提交未推送（需用户批准） |

### 🔵 P3 — 远期（楼梯间专项）
| 项目 | 预估 |
|------|------|
| 楼梯检测模块 | 3 天 |
| RL 走楼梯验证 | 2 天 |
| 训练爬楼/越障模型 | 8 周 |
| 楼梯间导航 + 集成验收 | 2 周 |
| **总计** | **约 10-14 周** |

---

## 12. 已知限制

1. **玻璃/金属走廊 ICP 定位失效** — 905nm 激光打玻璃无回波，物理限制
2. **失同步不自愈** — 雷达掉线重连后时钟跳变，需手动 restart
3. **FAST-LIO /Odometry 快速转向时间隔 240ms** — 计算瓶颈，RViz 中位置滞后
4. **RL 模型无地形观测** — 57 维观测不含高度/楼梯信息
5. **全向横移能力未启用** — Nav2 差分(vy=0) + TCP 无 vy 槽位
6. **dog-brain.service 当前 disabled** — 需手动启动建图栈

---

## 13. 文档清单

| 文档 | 位置 | 行数 | 说明 |
|------|------|------|------|
| `README.md` | `dog_ws/` | ~750 | 项目主文档（含架构、启动、配置、调试） |
| `STARTUP_GUIDE.md` | `dog_ws/src/` | ~170 | 启动命令速查 |
| `ros项目架构.md` | `dog_ws/` | ~835 | 完整系统架构（含网络拓扑、数据流、组件详解） |
| `机械狗项目完整文档.md` | `dog_ws/` | ~1140 | 合并版（交接书+进度+联调+记忆+架构） |
| `SYNC_BASELINE.md` | `dog_ws/` | ~49 | 文档同步策略 |
| `PROJECT_STATUS.md` | `dog_ws/` | 本文 | 当前状态总览 |

---

## 14. 当前最紧迫的事

> **需要狗在场，实测 TCP 控制链路能否驱动狗体运动。**  
> 这是 P0#2 的最后一环：软件栈全部就绪，指令格式已修（小端 `<3d`），UpBoard TCP 注入已写（本地副本），唯一缺的是现场实测验证。
