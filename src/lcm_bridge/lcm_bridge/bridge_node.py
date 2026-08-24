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

        # === ROS2 Publishers ===
        self.odom_pub = self.create_publisher(Odometry, self.topic_odom, 10)
        # /upboard/state_estimator: 滤波后状态 (含姿态/角速度/加速度) — 非原始 IMU
        self.imu_pub = self.create_publisher(Imu, self.topic_imu, 10)
        self.joint_pub = self.create_publisher(JointState, self.topic_joints, 10)
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
        self.get_logger().info(f'ROS2 topics: {self.topic_odom}, {self.topic_imu}, {self.topic_joints}, {self.topic_cmd_vel}')
        self.get_logger().info(f'Frames: odom={self.frame_odom}, base_link={self.frame_base}')
        self.get_logger().info(f'UpBoard TCP: {self.upboard_ip}:{self.upboard_port}')
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
    _nav_target = (0.0, 0.0)   # (v m/s, omega rad/s) 最新 cmd_vel 目标
    _nav_target_stamp = 0.0    # monotonic 时间戳
    _nav_cur = (0.0, 0.0)      # 斜坡后的当前发送值
    _nav_started = False       # 收到第一条 cmd_vel 才开始发送(建图模式不建 TCP)

    # 缩放/限幅常数 (源自 UpBoard 代码链条, 勿随意改动):
    NAV_V_CHAIN = 4.5    # m/s → TCP: 1.5(v_scale) × 3(deadband (x/2)×6)
    NAV_W_CHAIN = 2.5    # rad/s → TCP: deadband (x/2)×5, 且符号取反(joystickRight[0]*=-1)
    NAV_SEND_HZ = 50.0       # 发送频率 (远高于 UpBoard 300ms 看门狗阈值)
    NAV_CMD_TIMEOUT = 0.5    # cmd_vel 停更超过此时长 → 目标归零
    NAV_V_ACCEL = 0.5        # 线加速度斜坡限制 m/s^2
    NAV_W_ACCEL = 1.0        # 角加速度斜坡限制 rad/s^2
    NAV_V_MIN = 0.24         # 非线性映射最低速度 m/s (v_des = 0.24/3 = 0.08 > deadbandRegion 0.075)

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
            self._tcp_sock.send(data)
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
        with self._nav_lock:
            self._nav_target = (v, omega)
            self._nav_target_stamp = time.monotonic()
            self._nav_started = True

    def _nav_sender_loop(self):
        """独立定频发送线程: 50Hz 下发斜坡后的速度指令, 喂 UpBoard 看门狗"""
        period = 1.0 / self.NAV_SEND_HZ
        next_t = time.monotonic()
        while rclpy.ok():
            now = time.monotonic()
            with self._nav_lock:
                started = self._nav_started
                tv, tw = self._nav_target
                age = now - self._nav_target_stamp
            if not started:
                time.sleep(0.05)
                continue
            # 桥接层看门狗: Nav2 停发 → 目标归零 (平滑刹停)
            if age > self.NAV_CMD_TIMEOUT:
                tv, tw = 0.0, 0.0
            # ★ 非线性速度映射: Nav2 [0.01, 0.75] → [NAV_V_MIN, 0.75]
            # 低速段提升过 deadbandRegion(0.075), 高速段不变, 单调递增
            # 作用于目标值(斜坡之前), 保证加速平滑
            if abs(tv) > 0.001:
                _s = 1.0 if tv > 0 else -1.0
                tv = _s * (self.NAV_V_MIN + (abs(tv) - 0.01) * (0.75 - self.NAV_V_MIN) / 0.74)
            # 加速度斜坡 (防阶跃冲击)
            cv, cw = self._nav_cur
            max_dv = self.NAV_V_ACCEL * period
            max_dw = self.NAV_W_ACCEL * period
            cv += max(-max_dv, min(max_dv, tv - cv))
            cw += max(-max_dw, min(max_dw, tw - cw))
            self._nav_cur = (cv, cw)
            # 映射到 TCP 摇杆等效值 (注意转向符号取反)
            tcp_v = max(-1.0, min(1.0, cv / self.NAV_V_CHAIN))
            tcp_w = max(-1.0, min(1.0, -cw / self.NAV_W_CHAIN))
            flag = 1.0 if (abs(cv) > 0.01 or abs(cw) > 0.01) else 0.0
            data = struct.pack('<3d', float(flag), float(tcp_w), float(tcp_v))
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
