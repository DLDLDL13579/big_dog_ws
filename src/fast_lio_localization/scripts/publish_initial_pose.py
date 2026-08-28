#!/usr/bin/env python3
# coding=utf-8
"""
发布 /initialpose 给 global_localization，触发 ICP 定位初始化。

两种用法:
  1. 手动指定位姿 (原有行为, 向后兼容):
       ros2 run fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0

  2. 自动模式 (launch 调用, 等待系统就绪后发布):
       ros2 run fast_lio_localization publish_initial_pose.py --auto \
           --x 0 --y 0 --z 0 --yaw 0 [--timeout 60]

     自动模式会:
       a. 等待 /Odometry 首帧 (FAST-LIO 就绪)
       b. 等待地图加载宽限期 (默认 8s, 让 global_localization 收到 /global_map)
       c. 检查 /map_to_odom 是否已发布 (若用户已手动设过初值则跳过)
       d. 持续发布 3s 确保 DDS 握手完成
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import tf_transformations
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class PublishInitialPose(Node):
    def __init__(self):
        super().__init__('publish_initial_pose')
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._map_to_odom_received = False
        self._odom_received = False

    def publish_pose(self, x, y, z, roll, pitch, yaw):
        quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.pose.pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        self.publisher_.publish(pose_msg)

    def wait_for_odom(self, timeout):
        """等待 /Odometry 首帧 (FAST-LIO 就绪标志)"""
        self.create_subscription(
            Odometry, '/Odometry',
            lambda msg: setattr(self, '_odom_received', True), 10)
        self.get_logger().info('Waiting for /Odometry (FAST-LIO ready)...')
        t0 = time.time()
        while rclpy.ok() and not self._odom_received:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t0 > timeout:
                self.get_logger().error(
                    f'Timed out waiting for /Odometry after {timeout}s. '
                    f'Is FAST-LIO running?')
                return False
        self.get_logger().info(
            f'/Odometry received after {time.time()-t0:.1f}s.')
        return True

    def check_already_initialized(self, timeout=3.0):
        """检查 /map_to_odom 是否已有数据 (用户可能已手动设过初值)"""
        self.create_subscription(
            Odometry, '/map_to_odom',
            lambda msg: setattr(self, '_map_to_odom_received', True), 10)
        t0 = time.time()
        while rclpy.ok() and not self._map_to_odom_received:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t0 > timeout:
                break
        return self._map_to_odom_received


def main(args=None):
    rclpy.init(args=args)

    # ── 解析参数 ──
    # 关键: 只把 --ros-args 之前的参数传给 argparse.
    # ROS 2 launch 会自动注入 --ros-args -r __node:=xxx,
    # 其中 __node:=xxx 不以 - 开头, 会被 nargs='*' 位置参数误吞.
    if args is None:
        import sys as _sys
        _filtered = []
        for _a in _sys.argv[1:]:
            if _a == '--ros-args':
                break
            _filtered.append(_a)
        _args_to_parse = _filtered
    else:
        _args_to_parse = args

    parser = argparse.ArgumentParser(
        description='Publish /initialpose for ICP localization')
    # 自动模式
    parser.add_argument('--auto', action='store_true',
                        help='自动模式: 等待系统就绪后发布')
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='等待 /Odometry 的超时秒数 (默认 60)')
    parser.add_argument('--map-grace', type=float, default=8.0,
                        help='收到 Odometry 后额外等待地图加载的秒数 (默认 8)')
    # 位姿 (自动模式用 --x/--y/..., 手动模式用位置参数)
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--pitch', type=float, default=0.0)
    parser.add_argument('--roll', type=float, default=0.0)
    # 位置参数 (向后兼容: x y z yaw pitch roll)
    parser.add_argument('pos_args', nargs='*', type=float,
                        help='位置参数: x y z yaw pitch roll')

    # parse_known_args: 忽略未知参数; 且只传 --ros-args 之前的参数
    parsed_args, _unknown = parser.parse_known_args(_args_to_parse)

    if parsed_args.pos_args:
        # 手动位置参数模式 (向后兼容)
        if len(parsed_args.pos_args) != 6:
            print('Usage: publish_initial_pose.py X Y Z YAW PITCH ROLL',
                  file=sys.stderr)
            print('   or: publish_initial_pose.py --auto [--x 0 --y 0 ...]',
                  file=sys.stderr)
            sys.exit(1)
        x, y, z, yaw, pitch, roll = parsed_args.pos_args
        auto = False
    else:
        x, y, z = parsed_args.x, parsed_args.y, parsed_args.z
        roll, pitch, yaw = parsed_args.roll, parsed_args.pitch, parsed_args.yaw
        auto = parsed_args.auto

    node = PublishInitialPose()

    if auto:
        # ── 自动模式: 等待就绪 ──
        if not node.wait_for_odom(parsed_args.timeout):
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

        # 等待地图加载 (global_map_publisher 5s 间隔 + global_localization 处理)
        node.get_logger().info(
            f'Waiting {parsed_args.map_grace}s for global map to load...')
        time.sleep(parsed_args.map_grace)

        # 检查是否已手动初始化
        if node.check_already_initialized(timeout=3.0):
            node.get_logger().info(
                '/map_to_odom already published — user likely set initialpose '
                'manually. Skipping auto-publish.')
            node.destroy_node()
            rclpy.shutdown()
            return

    # ── 发布 initialpose (持续 3s 确保 DDS 握手) ──
    node.get_logger().info(
        f'Publishing initial pose: x={x}, y={y}, z={z}, '
        f'roll={roll}, pitch={pitch}, yaw={yaw}')
    t_end = time.time() + 3.0
    while rclpy.ok() and time.time() < t_end:
        node.publish_pose(x, y, z, roll, pitch, yaw)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info('Initial pose published successfully.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
