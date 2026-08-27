# Nav2 调参日志（单变量类轮次制）

> 流程纪律：**一次只改一类变量 → 重启 → 实车验证 → 记录结果**
> 验证脚本：`nav_goal_test.py`（带超程/超时/超速护栏）
> 参数文件：`src/dog_brain/config/nav2_3d_params.yaml`（--symlink-install，改完重启即生效）
> 重启命令：`bash /home/nvidia/nav_restart.sh`（已内置 `ROS_DOMAIN_ID=11`）

---

## R1 — 2026-08-26 — DWB critics 权重（仅 Nav2，FAST-LIO 未动）

### 改动
| 参数 | 旧值 | 新值 | 意图 |
|---|---|---|---|
| BaseObstacle.scale | 1.5（名义） | 8.0 | 加强障碍排斥 |
| PathAlign.scale | 32.0（名义） | 16.0 | 降低贴路径强度 |
| GoalAlign.scale | 24.0（名义） | 12.0 | 降低贴路径强度 |
| BaseObstacle.sum_scores | （默认 true） | false | 全轨迹点求和 → 取最差单点，防轨迹被整体判死 |

### ★ 重大发现（本轮最有价值的结论）
1. **旧版平铺写法从未生效**：`ros2 param get` 实测，修改前 6 个 critic 的运行时值**全部是默认 1.0**。
   ROS 2 Humble 的 yaml 不会把 `FollowPath.BaseObstacle.scale` 展平到嵌套命名空间
   `FollowPath.dwb_critics::BaseObstacle.scale`。即此前所有"调优"（包括 PROJECT_STATUS
   中记载的 PreferForward=10 等）均未实际生效。
2. **已改为嵌套写法**，重启后 `ros2 param get` 逐一确认生效（8.0/16.0/12.0/32.0/24.0/10.0, sum_scores=False）。
3. 验证脚本旧版订阅 `PoseWithCovarianceStamped`，而 `/localization` 实际类型为
   `nav_msgs/Odometry`（transform_fusion 发布），导致脚本"定位不在线"假报错。已适配。

### 实车验证（目标：正前方 2.0m）
| 指标 | 结果 |
|---|---|
| 实际位移（ICP） | 1.76 m / 2.0 m（差 0.24m，恰在 0.25 容差边缘） |
| /cmd_vel 峰值 | 0.45 m/s（达到上限） |
| 巡航形态 | **走-停-走**：中段两次停摆 8~12s（cmd_vx=0） |
| 结局 | 距目标 0.24m 处停死 → BT 触发 Spin 恢复 → behavior_server 报 **Collision Ahead** 中止 → 120s 超时护栏取消 |
| 附带告警 | bt_navigator "tick rate 100.00 was exceeded"（停摆期间）；取消后 planner 报 TF extrapolation into the past |

### 结论
- DWB 权重这次是**第一次真正生效**。
- BaseObstacle=8.0 对当前代价地图（0.5m 降采样点云 + 0.45m 膨胀）偏激：
  高代价区内所有候选轨迹被判碰撞 → 频繁"无有效轨迹"停摆（走-停-走形态）。
- 终点落在膨胀代价区内 → 最后 0.24m DWB 拒绝前进，Spin 恢复也被"前方碰撞"挡住。
- "避障率低"与"卡死不走"是同一枚硬币的两面，都由 **代价地图过粗 + 膨胀过大** 放大。

### 下一轮候选（仍按单变量类）
- **R2 首选：膨胀层**（inflation_radius 0.45→0.30，cost_scaling_factor 2.5→3.5）
  → 直接压缩高代价区面积，治终点停摆与停摆频率
- R3 备选：BaseObstacle.scale 8.0→5.0（若 R2 后仍频繁停摆）
- R4 备选（跨类需用户批准）：FAST-LIO filter_size 0.5→0.15，从根上解决感知颗粒度

## R4 — 2026-08-27 — 体素层 z 窗口收紧 + 垂直分辨率翻倍（局部+全局同步改）

> 注：R2(膨胀半径 0.45→0.30) / R3(z 窗口 -0.2~1.4 → -2.0~2.8) 未单独记录，见 nav2_3d_params.yaml 头部注释。

### 改动（基于 drift_test_20260827 实测漂移数据定窗）
| 参数 | 旧值(R3) | 新值(R4) | 说明 |
|---|---|---|---|
| origin_z | -2.0 | -0.75 | 体素网格底面（odom/map 全局系绝对 z） |
| z_resolution | 0.30 | 0.15 | 垂直分辨率翻倍 |
| z_voxels | 16 | 16 | 2.4/0.15=16，恰好满足 VoxelGrid ≤16 硬限制 |
| z 窗口 | -2.0~2.8m | -0.75~1.65m | 跨度 2.4m |
| min/max_obstacle_height | -2.0 / 2.8 | -0.75 / 1.65 | voxel_layer + livox_scan 观测源，局部+全局共 4 处同步 |

### 备份
- `src/dog_brain/config/nav2_3d_params.yaml.bak_R3_20260827`

### 风险关注
- R3 记载站立时传感器 z≈1.85m(odom 系)，若漂移后仍超新上界 1.65m，
  会报 "Sensor origin out of map bounds" → raytrace 失败。需实车验证。
- 配置为 symlink-install，重启导航栈即生效，无需重新构建。

### 实测验证（2026-08-27 11:42~11:48，352s，趴→站→遥控两圈→趴回原地）
| 指标 | 结果 |
|---|---|
| 闭环误差 | 0.17 m |
| 净漂移 | -0.042 m |
| z 范围 | [-0.118, +0.705] m |
| 趴下 / 站立 z | ≈-0.05 / ≈+0.45 m（峰值 +0.71 为步态起伏） |
| 窗口余量 | 下界 0.63 m，上界 0.94 m |

### 结论
- **窗口 -0.75~1.65m 安全**，垂直分辨率翻倍(0.15)生效，z_voxels=16 合规。
- R3 注释中"站立传感器 z≈1.85m"是旧栈 33 分钟失同步的病态值，正常工况不可复现。
- 上述风险关注项解除。
