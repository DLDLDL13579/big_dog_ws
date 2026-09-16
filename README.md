# 轮腿式机械狗项目 (dog_ws)

> **16-DOF 轮腿式机械狗（Mini Cheetah 构型）**，Jetson Orin NX 大脑 + UpBoard 小脑。
> 全链路已打通并实测验证：FAST-LIO2 建图 → ICP 3D 全局定位 → Nav2 3D 导航 → TCP 小端指令 → UpBoard RL 策略驱动运动。

> **2026-09-15 当前基线**：2D 平地导航已于 R25（09-07）封板（9 随机目标 78%、越界 0、`/Odometry` 中位 24ms）；2.5D 阶段① 配置经实机验证通过、导航实车闭环打通，楼梯可通行区方案落地中（R27）。详见《运维与状态文档》轮次总览与《2.5D-3D进阶方案》。

## 文档结构

| 文档 | 内容 |
|------|------|
| `README.md`（本文） | 入口：概述 / 架构 / 硬件网络 / 环境构建 / 快速启动 |
| `技术架构与调参.md` | 完整架构 + 数据流 + TF/Topic + 参数 + RViz + 设计决策 + 演进记录 + 启动速查 + Nav2 调参日志 |
| `运维与状态文档.md` | 多机协同接入 + 轮次总览 R15→R27 + 启动检查项 + 运维指南 + 安全红线 |
| `2.5D-3D进阶方案.md` | 2.5D/3D 四阶段路线、阶段① 实施记录与实测结果、三个坑、楼梯可通行区方案 |
| `SYNC_BASELINE.md` | Mac 工作区与设备侧的文档同步基线（权威方、同步约定、安全红线） |
| `Z1W_V2_iassc训练教程.md` | IsaacLab 训练流程教程 |
| `knowledge_base/` | C++ 原理知识库、项目工程案例库 |
| `upboard补丁留档/` | UpBoard TCP 注入补丁的源码、MD5 与重编/回退步骤 |
| `源码订正/` | 源码订正留档（`elevation_map_node.py` + 配套分析脚本） |
| `archive/` | 被取代的整份文档与旧版源码（含过时声明） |

> ⚠️ **历史更正与事故记录已并入正文**：08-27~08-28 的 DWB critics 写法、体素层 z 窗口、B1 TF 回退与 z 漂移归因等结论，见《技术架构与调参》对应章节；R24 的 FastDDS 共享内存静默死亡事故见《运维与状态文档》轮次总览。此处不再重复，以免与正文冲突。

## 目录
- [项目概述](#项目概述)
- [系统架构](#系统架构)
- [硬件平台与网络](#硬件平台与网络)
- [工作空间结构](#工作空间结构6-个包)
- [环境配置与构建](#环境配置与构建)
- [快速启动](#快速启动)
- [典型工作流](#典型工作流)

## 项目概述

机械狗在实验室等结构化场景内完成**自主建图**与**自主导航**：

- **建图模式（mapping）**：FAST-LIO2 激光-惯性里程计实时建图，产出 3D 点云地图（PCD）。
- **导航模式（navigation）**：基于已有 3D 地图，用 ICP 3D 全局定位（替代 AMCL）+ Nav2 3D 导航栈规划并下发速度指令，驱动狗体运动到目标。

## 系统架构

```
传感器           建图/定位              规划/导航             执行
Livox Mid-360  →  FAST-LIO2          →  Nav2 3D          →  TCP 小端指令
D435i 深度相机  →  ICP 3D 全局定位     →  (Navfn + DWB     →  UpBoard 小脑
                 (四件套替代 AMCL)       + Voxel 代价地图)    RL 策略驱动运动
```

- **定位四件套**：global_map_publisher → fastlio_mapping → global_localization → transform_fusion（发布 map→odom，持绝对定位权）。
- **地图双流 + 感知**：pcd_to_map（3D→2D /map）+ elevation_map（2.5D 高程）+ step_detector（D435i 台阶检测）。
- **桥接**：lcm_bridge 负责 LCM（UpBoard→Jetson 里程计/IMU/关节）与 TCP（Jetson→UpBoard 速度，小端 `<3d`）。

### 数据流全景

```mermaid
graph TB
    subgraph SENSOR["传感器"]
        M360["Livox Mid-360<br/>3D 激光"]
        D435["D435i<br/>深度相机"]
        UPIMU["UpBoard<br/>关节 / IMU / 里程计"]
    end

    subgraph BRAIN["Jetson 大脑（ROS2，DOMAIN_ID=11）"]
        subgraph LOC["定位四件套（替代 AMCL）"]
            GMP["global_map_publisher"]
            FLIO["fastlio_mapping<br/>FAST-LIO2"]
            GLOC["global_localization<br/>ICP 3D"]
            TFU["transform_fusion<br/>发布 map→odom"]
        end
        subgraph PERCEPT["感知与地图"]
            P2M["pcd_to_map<br/>3D→2D /map"]
            ELE["elevation_map<br/>2.5D 高程"]
            STEP["step_detector<br/>台阶检测"]
        end
        NAV2["Nav2 3D<br/>Navfn + DWB + Voxel 代价地图"]
    end

    subgraph LITTLE["UpBoard 小脑"]
        RL["RL 策略<br/>ONNX 推理"]
    end

    M360 --> FLIO
    D435 --> GLOC
    D435 --> STEP
    GMP --> FLIO --> GLOC --> TFU
    FLIO --> P2M --> NAV2
    FLIO --> ELE --> NAV2
    STEP --> NAV2
    NAV2 -->|"TCP 小端 3×double"| RL
    UPIMU -->|"LCM 组播 239.255.76.67:7667"| NAV2
    RL -->|"关节力矩"| UPIMU
```

## 硬件平台与网络

| 接口 | IP | 用途 |
|------|-----|------|
| `wlP1p1s0`（WiFi） | 192.168.31.91/24 | 主网络 / SSH / 上网 |
| `enx00e04c680779`（USB 网卡） | 192.168.1.100/32 | Mid-360 雷达直连 |
| `enP8p1s0`（有线） | 10.0.0.48/24 | UpBoard 底盘直连 |

| 设备 | 地址 | 通信 |
|------|------|------|
| Jetson 大脑 | nvidia@192.168.31.91 | SSH |
| UpBoard 小脑 | 10.0.0.6 | LCM 组播 239.255.76.67:7667 + TCP :3333 |
| Mid-360 雷达 | 192.168.1.195 | USB 网卡直连（数据口 56301/56401） |
| D435i 相机 | — | USB3 直连（必须 USB 3.0 线，否则降 14Hz） |

## 工作空间结构（6 个包）

```
dog_ws/src/
├── dog_brain/             # 大脑：建图/导航双模式编排 + Nav2 配置 + 地图
├── dog_description/       # URDF + STL 模型（19 link / 18 joint）
├── dog_sensors/           # 传感器 launch：D435i + Mid-360（含 mid360_config.json）
├── lcm_bridge/            # UpBoard↔ROS2 桥接 + TCP 速度下发
├── livox_ros_driver2/     # Livox 雷达驱动（已合并，含时间戳补偿补丁）
└── fast_lio_localization/ # FAST-LIO2 建图 + ICP 定位四件套 + 感知节点
```

## 环境配置与构建

```bash
# 每个新终端先加载环境（ROS_DOMAIN_ID=11 已在 ~/.bashrc）
source /opt/ros/humble/setup.bash
source /home/nvidia/dog_ws/install/setup.bash

# 构建（单工作空间，symlink-install：Python/launch/config 改后无需重 build，C++ 需重 build）
cd /home/nvidia/dog_ws
colcon build --symlink-install
```

## 快速启动

```bash
# 建图模式：FAST-LIO2 SLAM + 传感器 + LCM 桥接
ros2 launch dog_brain mapping_launch.py

# 导航模式：ICP 3D 定位 + Nav2 3D 导航（自动清理残留节点）
bash /home/nvidia/nav_restart.sh
# 或
ros2 launch dog_brain navigation_launch.py
```

分模块启动（传感器 / 单雷达 / 单相机 / LCM 桥接）、调试命令、IP/端口/Topic 速查，见 `技术架构与调参.md` 附录 D。

## 典型工作流

```bash
# ① 建图
ros2 launch dog_brain mapping_launch.py        # 静止 5~10s 让 IMU 初始化，遥控走一圈

# ② 保存 3D 地图
ros2 service call /map_save std_srvs/srv/Trigger   # 产出 ./test.pcd，重命名保留

# ③ 部署到导航地图
# 将 PCD 复制到 src/dog_brain/maps/lab_3d_map.pcd，并 ln -sf 到 install（见技术文档建图章节）

# ④ 导航
bash /home/nvidia/nav_restart.sh               # 设初始位姿 → RViz 发目标 → 狗体运动
```

> 运维细节、故障处置、当前任务与完成度，见 `运维与状态文档.md`。
> 动 UpBoard / 狗体前，务必遵守 `运维与状态文档.md` 中的安全红线。

## 归属与许可证

⚠️ **本仓库根目录没有 `LICENSE` 文件**，因此不声明统一的项目许可证。
各包的授权状态以各自 `package.xml` 中的 `<license>` 字段为准：

| 包 | `package.xml` 中的声明 |
|---|---|
| `dog_brain` | `MIT` |
| `dog_description` | `MIT` |
| `dog_sensors` | `MIT` |
| `lcm_bridge` | `MIT` |
| `livox_ros_driver2` | `MIT` |
| `fast_lio_localization` | `BSD` |

### 上游第三方包（权利归原作者）

| 包 / 组件 | 来源与归属 |
|---|---|
| `livox_ros_driver2` | **Livox（览沃科技）** 官方雷达驱动；本仓库内已合并并加入时间戳补偿补丁 |
| `fast_lio_localization` | 基于 **FAST-LIO2**（HKU MARS Lab，BSD）与 ICP 全局定位的二次开发 |
| `fastdds_udp.xml` | Fast DDS 通信配置（eProsima Fast DDS，Apache-2.0） |
| `robot2_adapter.py` / `robot2-adapter.service` | 多机协同上行适配器（本项目自研） |

> 建议：若要对外声明 MIT，请补充根目录 `LICENSE` 文件，并把各包 `package.xml`
> 中缺失的许可证字段补全。

### 硬件与安全提示

- 本项目控制的是**真实四足机器人**（16-DOF 轮腿构型），涉及大扭矩执行器与实时控制，
  调试时请确保场地安全、急停可用。
- README 中的控制参数基线（如 R18）**仍在实车对照验证中**，请勿当作已稳定结论使用。
