#!/usr/bin/env python3
# LCM ↔ ROS2 Bridge
# 从 UpBoard 接收 LCM 数据，发布到 ROS2 topic
# 同时订阅 /cmd_vel，通过 TCP 下发到 UpBoard
#
# UpBoard 源码分析结论:
#   - buf_socket_data[0] = 标志位 (导航模式1/停止0)
#   - buf_socket_data[1] = yaw_rate (rad/s) → omega_des[2] → _yaw_turn_rate
#   - buf_socket_data[2] = velocity → v_des[0]
#   - TCP 端口: 3333, 格式: 3 个 double (大端)
#
# LCM spi_data 关节顺序 (数组索引 0..3, Mini Cheetah 惯例):
#   - abad(hip侧摆): FR, FL, HR, HL  (MiniCheetah 默认)
#   - hip(大腿俯仰):  FR, FL, HR, HL
#   - knee(小腿):     FR, FL, HR, HL
#   - sup(轮子):      FR, FL, HR, HL
# ⚠️ 桥接时映射到 URDF: HR→RR, HL→RL
#
# URDF 关节名 (z1w_v2 实物):
#   FL abad=FL_hip_joint, hip=FL_thigh_joint, knee=FL_calf_joint, wheel=FL_wheel_joint
#   FR, RL, RR 同理
# ⚠️ 需接实狗后验证数组顺序是否与 MiniCheetah 默认一致
#
# 配置加载 (2026-08-14 修复):
#   所有可配置项从 config/bridge_params.yaml 读取, 缺失时回退到本文件默认值。
#   ROS2 参数 (--ros-args -p) 仍可覆盖 upboard_ip / upboard_port / publish_odom_tf。

import math
import struct
import socket
import threading
import time
import os

# Auto-injected: ensure LCM lib is importable
import sys, os as _os
_lcm_path = _os.path.expanduser('~/.local/lib/python3.10/site-packages')
if _lcm_path not in sys.path:
    sys.path.insert(0, _lcm_path)

import yaml
import lcm
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

# ============================================================
# 默认配置（当 bridge_params.yaml 缺失或字段不全时回退）
# ============================================================
DEFAULT_CONFIG = {
    'lcm_url': "udpm://239.255.76.67:7667?ttl=255",
    'upboard_ip': "10.0.0.6",
    'upboard_port': 3333,
    'lcm_channels': {
        'odometry': 'global_to_robot',
        'imu': 'state_estimator',
        'joints': 'spi_data',
    },
    'frames': {
        'odom': 'odom',
        'base_link': 'base_link',
    },
    'topics': {
        'odom': '/odom',
        'nav_odom': '/nav_odom',
        'imu': '/upboard/state_estimator',
        'joint_states': '/joint_states',
        'cmd_vel': '/cmd_vel',
    },
}


def _deep_merge(base, override):
    """递归合并 dict, override 覆盖 base。"""
    out = dict(base)
    for k, v in (override or {}).items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config():
    """加载 config/bridge_params.yaml。

    查找顺序:
      1. ament 包 share 目录 (install 环境)
      2. 源码树 src/<pkg>/config/
      3. ~/dog_ws/src/lcm_bridge/config/
    全部缺失 → 返回 DEFAULT_CONFIG。
    """
    candidates = []

    if get_package_share_directory is not None:
        try:
            pkg_share = get_package_share_directory('lcm_bridge')
            candidates.append(os.path.join(pkg_share, 'config', 'bridge_params.yaml'))
        except Exception:
            pass

    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(here, '..', 'config', 'bridge_params.yaml'))
    candidates.append(os.path.expanduser('~/dog_ws/src/lcm_bridge/config/bridge_params.yaml'))

    for path in candidates:
        try:
            if path and os.path.isfile(path):
                with open(path, 'r') as f:
                    data = yaml.safe_load(f) or {}
                merged = _deep_merge(DEFAULT_CONFIG, data)
                return merged, path
        except Exception as e:
            print(f'[lcm_bridge] 读取配置 {path} 失败: {e}', file=sys.stderr)

    return dict(DEFAULT_CONFIG), None


# ============================================================
# 关节名映射: LCM spi_data 数组 → URDF joint name
# LCM 顺序 (Mini Cheetah 源码): abad[0..3], hip[0..3], knee[0..3], sup[0..3]
# 每条腿的索引: 0=FL, 1=FR, 2=HL, 3=HR  (根据 rt_rc_interface.cpp)
# URDF 命名: {leg}_{type}_joint
# ============================================================
# abad (hip侧摆) → URDF "_hip_joint"
# hip (大腿俯仰) → URDF "_thigh_joint"
# knee (小腿)    → URDF "_calf_joint"
# sup (轮子)     → URDF "_wheel_joint"
LCM_LEG_ORDER = ['FL', 'FR', 'RL', 'RR']  # spi_data 数组索引 0..3 (matches URDF z1w_v2)
LCM_JOINT_TYPES = ['hip', 'thigh', 'calf', 'wheel']  # q_abad/q_hip/q_knee/q_sup → URDF名


def build_joint_names():
    """按 LCM spi_data 字段顺序生成 URDF 关节名"""
    names = []
    for jt_idx, jt in enumerate(LCM_JOINT_TYPES):
        for leg_idx, leg in enumerate(LCM_LEG_ORDER):
            names.append(f'{leg}_{jt}_joint')
    return names


LCM_JOINT_NAMES = build_joint_names()
# ============================================================


class LcmRosBridge(Node):
    def __init__(self):
        super().__init__('lcm_bridge')

        # ── 加载 yaml 配置 ──────────────────────────────────
        self.config, self.config_path = load_config()
        self.lcm_url = self.config.get('lcm_url', DEFAULT_CONFIG['lcm_url'])
        lcm_channels = self.config.get('lcm_channels', DEFAULT_CONFIG['lcm_channels'])
        topics = self.config.get('topics', DEFAULT_CONFIG['topics'])
        frames = self.config.get('frames', DEFAULT_CONFIG['frames'])

        # LCM 频道名
        self.ch_odom = lcm_channels.get('odometry', 'global_to_robot')
        self.ch_imu = lcm_channels.get('imu', 'state_estimator')
        self.ch_joints = lcm_channels.get('joints', 'spi_data')

        # ROS2 topic 名
        self.topic_odom = topics.get('odom', '/odom')
        self.topic_nav_odom = topics.get('nav_odom', '/nav_odom')
        self.topic_imu = topics.get('imu', '/upboard/state_estimator')
        self.topic_joints = topics.get('joint_states', '/joint_states')
        self.topic_cmd_vel = topics.get('cmd_vel', '/cmd_vel')

        # TF frame 名
        self.frame_odom = frames.get('odom', 'odom')
        self.frame_base = frames.get('base_link', 'base_link')

        # ── ROS2 参数（默认值取自 yaml, 可被 --ros-args -p 覆盖）──
        self.declare_parameter('upboard_ip', str(self.config.get('upboard_ip', DEFAULT_CONFIG['upboard_ip'])))
        self.declare_parameter('upboard_port', int(self.config.get('upboard_port', DEFAULT_CONFIG['upboard_port'])))
        # 导航/建图模式下 odom TF 由 FAST-LIO 提供, 此处默认关闭避免 TF 冲突
        self.declare_parameter('publish_odom_tf', False)
        self.upboard_ip = self.get_parameter('upboard_ip').value
        self.upboard_port = self.get_parameter('upboard_port').value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value
        # R12 转向抬底：当前默认开启，确保小角速度越过 UpBoard 死区。
        self.declare_parameter('enable_wz_floor', True)
        self.declare_parameter('wz_floor', self.NAV_W_FLOOR)
        self.wz_floor_en = bool(self.get_parameter('enable_wz_floor').value)
        self.wz_floor = float(self.get_parameter('wz_floor').value)
        self._wz_floor_last_raw = None
        # R13 锁存驻留解锁: 09-02 实车验证轮默认 ON (同 R12 wz_floor 做法, bringup 无参数管道)
        self.declare_parameter('enable_latch_dwell', True)
        self.declare_parameter('latch_dwell_sec', 2.0)
        self.latch_dwell_en = bool(self.get_parameter('enable_latch_dwell').value)
        self.latch_dwell_sec = float(self.get_parameter('latch_dwell_sec').value)
        # R15 wz 方向锁定: 防 DWB ~1Hz 换向致 90% 旋转量互相抵消 (09-02 分析实锤)
        self.declare_parameter('enable_wz_dir_lock', True)
        self.declare_parameter('wz_dir_lock_sec', self.NAV_W_DIR_LOCK_SEC)
        self.wz_dir_lock_en = bool(self.get_parameter('enable_wz_dir_lock').value)
        self.wz_dir_lock_sec = float(self.get_parameter('wz_dir_lock_sec').value)

        # R21: DWB 专用速度反馈。FAST-LIO /Odometry 是真实三维状态，但实车
        # 可出现 1s 以上处理延迟和显著步态横摆速度，不应直接作为二维 DWB 的
        # 当前速度。/nav_odom 只表达控制模型允许的 vx/wz，且使用当前时间戳。
        self.declare_parameter('nav_odom_imu_timeout', self.NAV_ODOM_IMU_TIMEOUT)
        self.declare_parameter('nav_odom_w_tau', self.NAV_ODOM_W_TAU)
        self.declare_parameter('nav_odom_w_deadband', self.NAV_ODOM_W_DEADBAND)
        self.nav_odom_imu_timeout = float(
            self.get_parameter('nav_odom_imu_timeout').value)
        self.nav_odom_w_tau = float(self.get_parameter('nav_odom_w_tau').value)
        self.nav_odom_w_deadband = float(
            self.get_parameter('nav_odom_w_deadband').value)
        self._imu_state_lock = threading.Lock()
        self._latest_imu_wz = 0.0
        self._latest_imu_mono = 0.0
        self._nav_odom_filtered_wz = 0.0

        # === ROS2 Publishers ===
        self.odom_pub = self.create_publisher(Odometry, self.topic_odom, 10)
        self.nav_odom_pub = self.create_publisher(
            Odometry, self.topic_nav_odom, 10)
        # /upboard/state_estimator: 滤波后状态 (含姿态/角速度/加速度) — 非原始 IMU
        self.imu_pub = self.create_publisher(Imu, self.topic_imu, 10)
        self.joint_pub = self.create_publisher(JointState, self.topic_joints, 10)
        # 桥接层完成方向锁、转向抬底和加速度斜坡后的实际 ROS 速度目标。
        # 下次试车录制该话题，即可把 Nav2 原始输出与 TCP 下发前结果分开分析。
        self.cmd_vel_bridge_pub = self.create_publisher(Twist, '/cmd_vel_bridge', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # === ROS2 Subscribers ===
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.topic_cmd_vel, self.cmd_vel_callback, 10)

        # === LCM init (在独立线程运行) ===
        self.lc = lcm.LCM(self.lcm_url)
        self.lc.subscribe(self.ch_odom, self.handle_odom)
        self.lc.subscribe(self.ch_imu, self.handle_imu)
        self.lc.subscribe(self.ch_joints, self.handle_joints)

        self.lcm_thread = threading.Thread(target=self.lcm_loop, daemon=True)
        self.lcm_thread.start()

        # [NAV-TCP v2] 独立定频发送线程 (50Hz): 与 LCM 高频回调解耦, 持续喂看门狗
        self.nav_thread = threading.Thread(target=self._nav_sender_loop, daemon=True)
        self.nav_thread.start()

        self.get_logger().info(f'Config loaded from: {self.config_path}')
        self.get_logger().info(f'LCM URL: {self.lcm_url}')
        self.get_logger().info(f'LCM channels: odom={self.ch_odom}, imu={self.ch_imu}, joints={self.ch_joints}')
        self.get_logger().info(
            f'ROS2 topics: {self.topic_odom}, {self.topic_nav_odom}, '
            f'{self.topic_imu}, {self.topic_joints}, {self.topic_cmd_vel}')
        self.get_logger().info(f'Frames: odom={self.frame_odom}, base_link={self.frame_base}')
        self.get_logger().info(f'UpBoard TCP: {self.upboard_ip}:{self.upboard_port}')
        self.get_logger().info(f'WZ floor: {"ON" if self.wz_floor_en else "OFF"} (floor={self.wz_floor})')
        self.get_logger().info(f'Latch dwell unlock: {"ON" if self.latch_dwell_en else "OFF"} (dwell={self.latch_dwell_sec}s)')
        self.get_logger().info(f'WZ dir lock: {"ON" if self.wz_dir_lock_en else "OFF"} (lock_sec={self.wz_dir_lock_sec}, strong_unlock={self.NAV_W_UNLOCK_STRONG}/{self.NAV_W_UNLOCK_STRONG_SEC}s, reverse->decay30%)')
        self.get_logger().info(
            f'Nav odom: {self.topic_nav_odom} '
            f'(vx=bridge, vy=0, wz=UpBoard IMU, '
            f'imu_timeout={self.nav_odom_imu_timeout}s, '
            f'w_tau={self.nav_odom_w_tau}s)')
        self.get_logger().info(f'Joint mapping: {LCM_JOINT_NAMES[:4]}... ({len(LCM_JOINT_NAMES)} joints)')

    # ── LCM → ROS2 ──────────────────────────────────────────

    def handle_odom(self, channel, data):
        """global_to_robot → /odom + odom→base_link TF"""
        # localization_lcmt: 8B fingerprint + xyz[3] + vxyz[3] + rpy[3] + omegaBody[3]
        # 各 4B float = 12 floats = 48B
        if len(data) < 56:
            return
        f = struct.unpack('>12f', data[8:56])
        x, y, z = f[0], f[1], f[2]
        vx, vy, vz = f[3], f[4], f[5]
        roll, pitch, yaw = f[6], f[7], f[8]
        wx, wy, wz = f[9], f[10], f[11]

        now = self.get_clock().now()

        # Euler → quaternion (ZYX order)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        # 发布 odom→base_link TF
        # ⚠️ 导航/建图模式下由 FAST-LIO 提供 odom→base_link TF (激光惯性里程计)
        #    UpBoard 运动学里程计仅作本体运动闭环, 不在 ROS 侧重复广播 TF
        #    但 /odom Odometry 消息仍需发布 (供调试/备用里程计源)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.frame_odom
        t.child_frame_id = self.frame_base
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w = qw
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        if self.publish_odom_tf:
            self.tf_broadcaster.sendTransform(t)

        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_base
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.x = wx
        odom.twist.twist.angular.y = wy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

    def handle_imu(self, channel, data):
        """state_estimator → /upboard/state_estimator (滤波后状态, 非原始IMU)"""
        # state_estimator_lcmt: 8B + 31 floats
        # quat[4] at offset 18..21, omegaBody[3] at 15..17, aBody[3] at 22..24
        if len(data) < 132:
            return
        f = struct.unpack('>31f', data[8:132])
        quat_w, quat_x, quat_y, quat_z = f[18], f[19], f[20], f[21]
        omega_x, omega_y, omega_z = f[15], f[16], f[17]
        acc_x, acc_y, acc_z = f[22], f[23], f[24]

        with self._imu_state_lock:
            self._latest_imu_wz = float(omega_z)
            self._latest_imu_mono = time.monotonic()

        now = self.get_clock().now()
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = self.frame_base
        imu.orientation.w = quat_w
        imu.orientation.x = quat_x
        imu.orientation.y = quat_y
        imu.orientation.z = quat_z
        imu.angular_velocity.x = omega_x
        imu.angular_velocity.y = omega_y
        imu.angular_velocity.z = omega_z
        imu.linear_acceleration.x = acc_x
        imu.linear_acceleration.y = acc_y
        imu.linear_acceleration.z = acc_z
        self.imu_pub.publish(imu)

    def handle_joints(self, channel, data):
        """spi_data → /joint_states"""
        # spi_data_t LCM 格式 (大端, float=4B, int32=4B):
        #   q_abad[4] + q_hip[4] + q_knee[4] + q_sup[4]       = 16 floats (64B)
        #   qd_abad[4] + qd_hip[4] + qd_knee[4] + qd_sup[4]    = 16 floats (64B)
        #   tau_abad[4] + tau_hip[4] + tau_knee[4] + tau_sup[4] = 16 floats (64B)
        #   flags[4] = 4 int32 (16B) + spi_driver_status = 1 int32 (4B)
        # Total payload: 48 floats + 5 int32 = 212B
        if len(data) < 220:
            return

        pos_raw = struct.unpack('>48f', data[8:200])  # q + qd + tau
        # flags + status at data[200:220]

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = ''
        js.name = LCM_JOINT_NAMES

        # pos_raw layout: 按类型分组, 每组内按 LCM_LEG_ORDER 排
        # q_abad[FL,FR,HL,HR], q_hip[FL,FR,HL,HR], q_knee[FL,FR,HL,HR], q_sup[FL,FR,HL,HR]
        # qd_abad[...], qd_hip[...], qd_knee[...], qd_sup[...]
        # tau_abad[...], tau_hip[...], tau_knee[...], tau_sup[...]
        for i in range(16):
            js.position.append(float(pos_raw[i]))
            js.velocity.append(float(pos_raw[16 + i]))
            js.effort.append(float(pos_raw[32 + i]))

        self.joint_pub.publish(js)

    def lcm_loop(self):
        """LCM 事件循环 (后台线程)"""
        while rclpy.ok():
            self.lc.handle_timeout(100)

    # ── ROS2 → UpBoard TCP ──────────────────────────
    # [NAV-TCP v2] 2026-08-19 重构（修复“抽搐+暴冲”实测事故）:
    #
    #   问题1 (幅值): 旧映射 v/0.75 错误。实测完整增益链:
    #     TCP buf[2] --×1.5(v_scale)--> v_des[0] --direct--> joystickLeft[1]
    #     --deadband (x/2)×(maxVelX-minVelX=6)--> stateDes(6) = 3×1.5×TCP = 4.5×TCP
    #     即 policy vx = 4.5 × TCP buf[2]。发 0.1m/s 时狗收到 0.6m/s 冲刺指令!
    #     转向同理: omega_des[2]=TCP buf[1] --×(-1)--> joystickRight[0]
    #     --deadband (x/2)×(2.5-(-2.5))--> stateDes(11) = -2.5×TCP buf[1] (符号取反!)
    #   问题2 (时序): cmd_vel 回调被 LCM 高频回调抢占 GIL, 实测 56 条指令只发出 13 条(~10Hz);
    #     且停发后 UpBoard 300ms 看门狗归零, 造成指令时断时续 → 狗抽搐后暴冲
    #   问题3 (平滑): 阶跃指令无斜坡, 狗从静止瞬间被命令冲刺
    #
    #   解法: 回调只存目标值(纳秒级, 不受 GIL 饿死影响); 独立线程 50Hz 定频发送
    #   (持续喂看门狗), 带加速度斜坡; cmd_vel 500ms 停更自动归零(桥接层看门狗);
    #   TCP_NODELAY 防 Nagle 合并延迟。
    _tcp_sock = None
    _nav_lock = threading.Lock()
    _nav_target = (0.0, 0.0, 0.0)   # (v, omega, vy) 最新 cmd_vel 目标
    # [LATERAL 2026-09-21] 横移状态: 斜坡后的 vy 与上一拍原始值
    _nav_cur_y = 0.0
    _nav_target_stamp = 0.0    # monotonic 时间戳
    _nav_cur = (0.0, 0.0)      # 斜坡后的当前发送值
    _nav_started = False       # 收到第一条 cmd_vel 才开始发送(建图模式不建 TCP)
    _nav_prev_raw_v = 0.0      # 上一拍原始 cmd_vel vx (R11 减速锁存判据)
    _nav_desc_streak = 0       # 连续严格下降拍数
    _nav_decel_latch = False   # R11 减速直通锁存
    _nav_latch_dwell_raw = None  # R13: 锁存中恒值驻留的比较基准值
    _nav_latch_dwell_t0 = 0.0    # R13: 恒值驻留起点 (monotonic)
    _nav_dwell_rearm = False     # R13: 驻留解锁后重武装(下次连降即锁存, 免 prev>NAV_V_MIN)
    _wz_dir_lock_dir = 0         # R15: 锁定方向 +1/-1, 0=未锁定
    _wz_dir_lock_t0 = 0.0        # R15: 锁定开始时间 (monotonic)
    _wz_dir_opp_t0 = 0.0         # R15: 反方向持续起始时间 (monotonic)

    # 缩放/限幅常数 (源自 UpBoard 代码链条, 勿随意改动):
    NAV_V_CHAIN = 4.5    # m/s → TCP: 1.5(v_scale) × 3(deadband (x/2)×6)
    NAV_W_CHAIN = 2.5    # rad/s → TCP: deadband (x/2)×5, 且符号取反(joystickRight[0]*=-1)
    NAV_SEND_HZ = 50.0       # 发送频率 (远高于 UpBoard 300ms 看门狗阈值)
    NAV_CMD_TIMEOUT = 0.5    # cmd_vel 停更超过此时长 → 目标归零
    NAV_V_ACCEL = 0.5        # 线加速度斜坡限制 m/s^2
    NAV_W_ACCEL = 1.0        # 角加速度斜坡限制 rad/s^2
    # R21 /nav_odom 必须与 DWB 的二维运动边界一致。
    NAV_ODOM_V_MAX = 0.45
    NAV_ODOM_W_MAX = 0.35
    NAV_ODOM_IMU_TIMEOUT = 0.2
    NAV_ODOM_W_TAU = 0.15
    NAV_ODOM_W_DEADBAND = 0.02
    NAV_V_MIN = 0.24         # 非线性映射最低速度 m/s
    # [订正2026-09-21] 曾降到 0.10 试图减少低速放大, 实测 G 点未到达 -> 回滚 0.24。
    #   注: UpBoard 死区现已为 0.03(二进制补丁), 该值不再是死区约束, 而是
    #   「RL 策略低速有效门限」的经验值, 不要仅按死区反推。
    # [NAV-DEADBAND 2026-09-21] 0.24 -> 0.10: UpBoard 死区已由 0.075 改为 0.03
    #   (二进制立即数补丁, 见 机械狗研发资料/死区改值方案分析_20260921.md)。
    #   旧值 0.24 会把 DWB 小速度指令抬到满量程 53%(实测 0.1 档实跑 0.23~0.25 m/s);
    #   新值 0.10 占满量程 22%, 恰好越过死区 0.03 而不过度放大。
    NAV_W_FLOOR = 0.31       # R12 转向抬底
    # [LATERAL 2026-09-21] 横移(vy)通路
    #   增益链: tcp_vy -> v_des[1] -> *(-1) -> stateDes(7)=(x/2)*4=2x -> vy=-d(7)
    #   故 vy_policy = 2 * tcp_vy  =>  NAV_Y_CHAIN = 2.0
    #   符号: 两级负号相消, vy_policy = +cy (与 DWB 同号)
    NAV_Y_CHAIN = 2.0        # m/s -> TCP (无 v_scale, 链条比 vx 短)
    NAV_Y_MAX = 0.30         # 横移最大速度 m/s (保守起步)
    NAV_Y_ACCEL = 0.5        # 横移加速度斜坡 m/s^2
    # ★★ [2026-09-21 实测订正] 0.19 -> 0.31: 0.19 是【死区谷底】而非有效值!
    #   转移曲线(桥接 wz -> /nav_odom 实测 wz):
    #     0.02~0.16 -> 0.008~0.017  完全无效
    #     0.19      -> -0.0370      符号反向(谷底)
    #     0.22      -> +0.0054      无效
    #     0.25/0.28 -> +0.065/+0.090 弱
    #     0.31      -> +0.2003      有效  <== 取此值
    #     0.34      -> +0.1541      有效
    #   原地转向有效比: |wz|>=0.30 时 67%; 0.15~0.21 时仅 29.6%
    #   现象: nav_G_009 狗卡在距目标 0.28m 处 26s 原地转(wz 恒 -0.190),
    #         转不动 -> Failed to make progress -> 超时。
    #   ⚠ 若发现转向过猛/过冲, 可试 0.28; 不要回到 0.19(无效)。
    # ★★ [订正2026-09-21, bag=nav_G_003 分档实证] 0.19 是【RL 转向有效门限】,
    #   不是随便设的: 桥接指令 vs 实测 yaw rate 分档统计 --
    #     |wz| in [0.08,0.12): 实测/指令=0.009, 符号一致率 52.7% (等同随机, 无效)
    #     |wz| in [0.12,0.19): 实测/指令=-0.005, 符号一致率 49.9% (完全无效)
    #     |wz| in [0.19,0.36): 实测/指令=0.779, 符号一致率 94.9% (有效)
    #   曾降到 0.08 导致指令落入无效区 -> 狗转不动、终点反复摆动 47s 进不去容差。
    #   如需再调, 必须先录 bag 验证该门限, 不可只按 UpBoard 死区反推。
    # [NAV-DEADBAND 2026-09-21] 0.19 -> 0.08: 旧值占 max_vel_theta(0.35) 的 54%,
    #   实测 47 次抬底把 DWB 的 -0.0026 放大 73 倍 -> 走弧线/来回摆。
    #   0.08 占满量程 23%, 可做中小幅度转向修正。
    NAV_W_DIR_LOCK_SEC = 0.3  # R17: lock_sec 1.0->0.3 (R16 冻结15.3%代价过大, 降为最小兜底)
    NAV_W_UNLOCK_STRONG = 0.30      # R16: 强修正解锁幅值 rad/s (> wz_floor 0.19, Nav2 强反向信号)
    NAV_W_UNLOCK_STRONG_SEC = 0.3   # R16: 强修正持续时长 s (持续即解锁, 免等满 lock_sec)

    def _tcp_send(self, data: bytes):
        """持久 TCP 发送: 建立连接→发→保持; 断线则关旧建新重发一次"""
        try:
            if self._tcp_sock is None:
                self._tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._tcp_sock.settimeout(0.5)
                try:
                    self._tcp_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except OSError:
                    pass
                self._tcp_sock.connect((self.upboard_ip, self.upboard_port))
            # TCP 是字节流，send() 允许短写。控制协议依赖严格 24B 定长帧，
            # 必须保证整帧进入内核发送缓冲区，否则对端可能从错位开始解析。
            self._tcp_sock.sendall(data)
        except (socket.timeout, ConnectionRefusedError, OSError) as e:
            # 连接坏了: 关闭, 下条消息重建
            try:
                if self._tcp_sock is not None:
                    self._tcp_sock.close()
            except OSError:
                pass
            self._tcp_sock = None
            self.get_logger().warn(
                f'TCP send to {self.upboard_ip}:{self.upboard_port} failed: {e}',
                throttle_duration_sec=2.0)

    def cmd_vel_callback(self, msg: Twist):
        """Nav2 /cmd_vel → 存目标值(由独立发送线程以 50Hz 定频下发)

        UpBoard TCP 3333 格式: 小端 3×double [标志位, yaw_rate, velocity]
        实测增益链 (FSM_State_RL + DesiredStateCommand, 2026-08-19 实狗验证):
          policy vx       = 1.5 × TCP[2] × 3 = 4.5 × TCP[2]
          policy yaw_rate = -2.5 × TCP[1]  (符号取反)
        故: TCP[2] = v / 4.5,  TCP[1] = -omega / 2.5
        转向模型: 速率控制 (direct yaw rate), 非 Ackermann
        """
        v = msg.linear.x       # m/s (前向为正)
        omega = msg.angular.z  # rad/s (左转为正)
        # [LATERAL 2026-09-21] 横移: DWB 的 linear.y (左正右负, ROS 惯例)
        vy = msg.linear.y
        if abs(vy) > self.NAV_Y_MAX:
            vy = math.copysign(self.NAV_Y_MAX, vy)
        with self._nav_lock:
            self._nav_target = (v, omega, vy)
            self._nav_target_stamp = time.monotonic()
            self._nav_started = True

    def _publish_nav_odom(self, linear_x, fallback_wz, now_mono, dt):
        """发布只供 DWB 使用的、模型兼容且时间新鲜的速度反馈。"""
        with self._imu_state_lock:
            imu_wz = self._latest_imu_wz
            imu_age = now_mono - self._latest_imu_mono
            imu_seen = self._latest_imu_mono > 0.0

        imu_fresh = (
            imu_seen
            and imu_age <= self.nav_odom_imu_timeout
            and math.isfinite(imu_wz)
        )
        target_wz = imu_wz if imu_fresh else fallback_wz
        if not imu_fresh and self._nav_started:
            self.get_logger().warn(
                'UpBoard IMU stale for /nav_odom; using bridge command fallback',
                throttle_duration_sec=5.0)

        tau = max(0.0, self.nav_odom_w_tau)
        safe_dt = max(0.0, dt)
        alpha = 1.0 if tau == 0.0 else min(
            1.0, safe_dt / (tau + safe_dt))
        self._nav_odom_filtered_wz += alpha * (
            target_wz - self._nav_odom_filtered_wz)

        nav_wz = self._nav_odom_filtered_wz
        if abs(nav_wz) < self.nav_odom_w_deadband:
            nav_wz = 0.0
        nav_wz = max(-self.NAV_ODOM_W_MAX, min(self.NAV_ODOM_W_MAX, nav_wz))
        nav_vx = max(
            -self.NAV_ODOM_V_MAX,
            min(self.NAV_ODOM_V_MAX, float(linear_x)))

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_odom
        msg.child_frame_id = self.frame_base
        msg.twist.twist.linear.x = nav_vx
        # 机械狗当前 Nav2 配置是非完整二维模型，明确禁止横移。
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = nav_wz
        self.nav_odom_pub.publish(msg)

    def _nav_sender_loop(self):
        """独立定频发送线程: 50Hz 下发斜坡后的速度指令, 喂 UpBoard 看门狗"""
        period = 1.0 / self.NAV_SEND_HZ
        next_t = time.monotonic()
        while rclpy.ok():
            now = time.monotonic()
            with self._nav_lock:
                started = self._nav_started
                tv, tw, ty = self._nav_target
                age = now - self._nav_target_stamp
            if not started:
                self._publish_nav_odom(0.0, 0.0, now, 0.05)
                time.sleep(0.05)
                continue
            # 桥接层看门狗: Nav2 停发 → 目标归零 (平滑刹停)
            if age > self.NAV_CMD_TIMEOUT:
                tv, tw, ty = 0.0, 0.0, 0.0
            # ★ 死区处理: 只抬底, 不放大 (R9; 旧版 [0.01,0.75]→[0.24,0.75] 拉升使 0.45 实发 0.545 超速)
            # R11 减速直通: 减速坡尾(<NAV_V_MIN)不再抬成 0.24 平台(实测会在目标线前重新加速,
            # 刹车过冲 0.14~0.62m), 放行原始值由 UpBoard deadband 自然收零。
            # 锁存: 从 >NAV_V_MIN 连续两次下降更新才锁存(更新间保持不清零); 解锁: |tv|>=NAV_V_MIN+0.04 或归零。
            # 稳态小指令(含单拍下探后持平)仍抬底, 与 R9 行为一致; 转向/watchdog/TCP/缩放不受影响。
            raw_tv = tv
            if abs(tv) <= 0.01:
                self._nav_decel_latch = False
                self._nav_desc_streak = 0
                self._nav_latch_dwell_raw = None
                self._nav_dwell_rearm = False
                tv = 0.0
            else:
                prev_raw = self._nav_prev_raw_v
                if abs(tv) < abs(prev_raw):
                    self._nav_desc_streak += 1
                elif abs(tv) > abs(prev_raw):
                    self._nav_desc_streak = 0
                # 相等=同一条指令被 50Hz 持有多拍: 保留连降计数, 只在上升时清零
                if self._nav_decel_latch:
                    if abs(tv) >= self.NAV_V_MIN + 0.04:
                        self._nav_decel_latch = False
                        self._nav_dwell_rearm = False
                        self._nav_latch_dwell_raw = None
                    elif (self.latch_dwell_en
                          and self._nav_latch_dwell_raw is not None
                          and abs(tv - self._nav_latch_dwell_raw) <= 1e-4):
                        # R13 驻留解锁: 锁存中恒值驻留(同值 ±1e-4)超过 latch_dwell_sec
                        # = DWB 巡航恒值被锁存直通、UpBoard deadband 吃零、狗已冻结
                        # (L5-4/6/7 实测 20s 停死) → 自动释放走抬底路径恢复可执行。
                        # 刹车尾严格递减, 每拍重置基准, 恒不会触发(实测尾长 ≤1.2s)。
                        if now - self._nav_latch_dwell_t0 >= self.latch_dwell_sec:
                            self._nav_decel_latch = False
                            self._nav_desc_streak = 0
                            self._nav_latch_dwell_raw = None
                            self._nav_dwell_rearm = True
                            tv = math.copysign(max(abs(tv), self.NAV_V_MIN), tv)
                    else:
                        self._nav_latch_dwell_raw = tv
                        self._nav_latch_dwell_t0 = now
                    # 锁存中: 原始值直通
                elif self._nav_desc_streak >= 2 and (abs(prev_raw) > self.NAV_V_MIN
                                                     or self._nav_dwell_rearm):
                    self._nav_decel_latch = True
                    self._nav_dwell_rearm = False
                    self._nav_latch_dwell_raw = tv
                    self._nav_latch_dwell_t0 = now
                    # 首拍即直通, tv 保持原始值
                else:
                    tv = math.copysign(max(abs(tv), self.NAV_V_MIN), tv)
                self._nav_prev_raw_v = raw_tv
            # ★ R15 wz 方向锁定: DWB 末端细对准 ~1Hz 交替 ±wz → 狗抬腿惯量大、
            # 每拍换向导致 90% 旋转量互相抵消 (09-02 七轮实锤: L5-4 成功 flip/s=0.03,
            # 六轮失败 flip/s=0.87~1.60)。锁定首次非零 wz 方向, 反方向需持续 ≥wz_dir_lock_sec
            # 才切换; wz=0 解锁。仅作用于非零 wz 段, 看门狗归零不受影响。
            if self.wz_dir_lock_en and abs(tw) > 1e-6:
                d = 1 if tw > 0 else -1
                if self._wz_dir_lock_dir == 0:
                    self._wz_dir_lock_dir = d
                    self._wz_dir_lock_t0 = now
                    self._wz_dir_opp_t0 = 0.0
                elif d == self._wz_dir_lock_dir:
                    self._wz_dir_opp_t0 = 0.0
                else:
                    if self._wz_dir_opp_t0 == 0.0:
                        self._wz_dir_opp_t0 = now
                    # R16 双条件解锁: (a) 反向幅值>=NAV_W_UNLOCK_STRONG 持续
                    # >=NAV_W_UNLOCK_STRONG_SEC 立即解锁 (Nav2 强修正信号穿透);
                    # (b) 反向任意幅值持续 >=wz_dir_lock_sec 解锁。
                    # 当前 R18 默认 lock_sec=0.3s，与 strong 持续时间相同，
                    # 因此强修正快通道只在 lock_sec 后续调大到 >0.3s 时才有额外效果。
                    strong = abs(tw) >= self.NAV_W_UNLOCK_STRONG
                    if ((strong and now - self._wz_dir_opp_t0 >= self.NAV_W_UNLOCK_STRONG_SEC)
                            or (now - self._wz_dir_opp_t0 >= self.wz_dir_lock_sec)):
                        self.get_logger().info(
                            f'wz dir lock switch: {self._wz_dir_lock_dir:+d} -> {d:+d}'
                            + (' (strong)' if strong else ''))
                        self._wz_dir_lock_dir = d
                        self._wz_dir_lock_t0 = now
                        self._wz_dir_opp_t0 = 0.0
                # R17/R18: 锁定期内反向指令衰减到 30% (原 R15 是 copysign
                # 强制按锁向执行=误差放大器；R16 置 0 又造成明显冻结)。
                # 综合实验实锤 G6 终点反向 177.6°)。同向指令保持锁定方向原样。
                # 输出侧置 0 不影响上方解锁计时 (计时基于输入方向 d);
                # wz=0 输入仍走下方 elif 分支归零解锁。
                if self._wz_dir_lock_dir != 0 and d != self._wz_dir_lock_dir:
                    self.get_logger().info(
                        f'wz dir lock hold: reverse {tw:+.3f} -> {tw*0.3:+.3f} (locked {self._wz_dir_lock_dir:+d})',
                        throttle_duration_sec=1.0)
                    tw = tw * 0.3
                else:
                    tw = math.copysign(abs(tw), self._wz_dir_lock_dir)
            elif abs(tw) <= 1e-6:
                self._wz_dir_lock_dir = 0
                self._wz_dir_opp_t0 = 0.0
            # ★ R12 转向抬底 (R18 移到方向锁之后): 只对“确认执行的方向”抬底,
            # 保证同向小指令过 UpBoard 死区; 反向被抑制(decay)的指令不抬底, 保持抑制意图。
            # 方向锁关闭 (或未锁定) 时无条件抬底, 与原 R12 行为一致。
            if self.wz_floor_en and 1e-6 < abs(tw) < self.wz_floor:
                _cur_dir = 1 if tw > 0 else -1
                if (not self.wz_dir_lock_en or self._wz_dir_lock_dir == 0
                        or _cur_dir == self._wz_dir_lock_dir):
                    tw2 = math.copysign(self.wz_floor, tw)
                    if tw != self._wz_floor_last_raw:
                        self.get_logger().info(f'wz floor: raw {tw:+.4f} -> {tw2:+.4f}')
                        self._wz_floor_last_raw = tw
                    tw = tw2
                else:
                    self._wz_floor_last_raw = None
            else:
                self._wz_floor_last_raw = None
            # 加速度斜坡 (防阶跃冲击)
            cv, cw = self._nav_cur
            max_dv = self.NAV_V_ACCEL * period
            max_dw = self.NAV_W_ACCEL * period
            cv += max(-max_dv, min(max_dv, tv - cv))
            cw += max(-max_dw, min(max_dw, tw - cw))
            self._nav_cur = (cv, cw)
            # [LATERAL 2026-09-21] 横移斜坡
            max_dy = self.NAV_Y_ACCEL * period
            cy = self._nav_cur_y + max(-max_dy, min(max_dy, ty - self._nav_cur_y))
            self._nav_cur_y = cy
            bridge_cmd = Twist()
            bridge_cmd.linear.x = float(cv)
            bridge_cmd.angular.z = float(cw)
            self.cmd_vel_bridge_pub.publish(bridge_cmd)
            self._publish_nav_odom(cv, cw, now, period)
            # 映射到 TCP 摇杆等效值 (注意转向符号取反)
            # UpBoard 导航模式: 前向 v_des=1.5×TCP, 倒向 v_des=TCP/2 (无 v_scale);
            # 倒向分母改 ÷1.5 (NAV_V_CHAIN/3) 补偿 ÷2 分支, 前后向均恒等 stateDes[6]=cv
            _v_denom = self.NAV_V_CHAIN if cv >= 0 else self.NAV_V_CHAIN / 3.0
            tcp_v = max(-1.0, min(1.0, cv / _v_denom))
            tcp_w = max(-1.0, min(1.0, -cw / self.NAV_W_CHAIN))
            flag = 1.0 if (abs(cv) > 0.01 or abs(cw) > 0.01 or abs(cy) > 0.01) else 0.0
            # [LATERAL 2026-09-21] TCP 协议 3->4 doubles: 第4个为 vy(横移)
            #   阶段3: 启用真实 vy 映射 (增益链见 NAV_Y_CHAIN 注释)
            tcp_vy = max(-1.0, min(1.0, cy / self.NAV_Y_CHAIN))
            data = struct.pack('<4d', float(flag), float(tcp_w), float(tcp_v), float(tcp_vy))
            self._tcp_send(data)
            # 定频节拍 (落后不追赶, 避免突发)
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()


def main():
    rclpy.init()
    node = LcmRosBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
