#!/usr/bin/env python3
"""
robot2_soldier_node.py — 机械狗多机协同网关节点 (robot2)

职责:
  - 向主车上报: /robot_2/soldier_pose, /robot_2/odom, /robot_2/battery_state
  - 接收主车指令: /robot_2/cmd_vel → /cmd_vel, /robot_2/goal_pose → NavigateToPose, /robot_2/initialpose → /initialpose

设计原则:
  - 独立于导航栈运行，不依赖导航栈启动
  - goal_pose 通过 action client 调用，导航栈未启动时等待连接
  - 电池数据暂用占位值（UpBoard LCM 无电池通道）
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from nav2_msgs.action import NavigateToPose

import time


class Robot2SoldierNode(Node):
    """机械狗协同网关节点，桥接 /robot_2/* 话题与内部话题"""

    def __init__(self):
        super().__init__('robot2_soldier_node')

        # ─── 参数声明 ─────────────────────────────────────────────
        self.declare_parameter('robot_id', 'robot_2')
        self.declare_parameter('battery_percentage', 100.0)
        self.declare_parameter('battery_voltage', 24.0)

        self.robot_id = self.get_parameter('robot_id').value
        self.battery_pct = self.get_parameter('battery_percentage').value
        self.battery_v = self.get_parameter('battery_voltage').value

        # ─── Publishers (上行: 机械狗 → 主车) ─────────────────────
        self.pub_soldier_pose = self.create_publisher(
            PoseWithCovarianceStamped, f'/{self.robot_id}/soldier_pose', 10)
        self.pub_odom = self.create_publisher(
            Odometry, f'/{self.robot_id}/odom', 10)
        self.pub_battery = self.create_publisher(
            BatteryState, f'/{self.robot_id}/battery_state', 10)

        # ─── Subscribers (下行: 主车 → 机械狗) ────────────────────
        self.sub_cmd_vel = self.create_subscription(
            Twist, f'/{self.robot_id}/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_goal_pose = self.create_subscription(
            PoseStamped, f'/{self.robot_id}/goal_pose', self.goal_pose_callback, 10)
        self.sub_initialpose = self.create_subscription(
            PoseWithCovarianceStamped, f'/{self.robot_id}/initialpose',
            self.initialpose_callback, 10)

        # ─── 内部话题转发目标 ─────────────────────────────────────
        self.pub_cmd_vel_internal = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_initialpose_internal = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # ─── 内部话题订阅 (数据源) ────────────────────────────────
        self.sub_localization = self.create_subscription(
            Odometry, '/localization', self.localization_callback, 10)
        self.sub_odom_internal = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # ─── Nav2 Action Client ──────────────────────────────────
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ─── 电池定时发布 (1Hz) ──────────────────────────────────
        self.battery_timer = self.create_timer(1.0, self.battery_timer_callback)

        self.get_logger().info(
            f'[{self.robot_id}] Soldier node started. '
            f'Publishing: soldier_pose, odom, battery_state. '
            f'Subscribing: cmd_vel, goal_pose, initialpose.')

    # ─── 上行回调: /localization → /robot_2/soldier_pose ──────────
    def localization_callback(self, msg: Odometry):
        """将 ICP 定位结果 (map 坐标系) 转换为 PoseWithCovarianceStamped 发布"""
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = msg.header.frame_id  # 'map'
        pose.pose.pose = msg.pose.pose
        pose.pose.covariance = msg.pose.covariance
        self.pub_soldier_pose.publish(pose)

    # ─── 上行回调: /odom → /robot_2/odom ──────────────────────────
    def odom_callback(self, msg: Odometry):
        """直接转发里程计"""
        self.pub_odom.publish(msg)

    # ─── 上行定时: 电池状态 ──────────────────────────────────────
    def battery_timer_callback(self):
        """发布电池状态（占位值，UpBoard 无电池通道）"""
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.header.frame_id = self.robot_id
        battery.voltage = self.battery_v
        battery.percentage = self.battery_pct / 100.0
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        battery.present = True
        self.pub_battery.publish(battery)

    # ─── 下行回调: /robot_2/cmd_vel → /cmd_vel ────────────────────
    def cmd_vel_callback(self, msg: Twist):
        """接收主车速度指令，转发到内部 /cmd_vel"""
        self.pub_cmd_vel_internal.publish(msg)
        self.get_logger().debug(
            f'cmd_vel forwarded: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}')

    # ─── 下行回调: /robot_2/goal_pose → NavigateToPose ────────────
    def goal_pose_callback(self, msg: PoseStamped):
        """接收主车导航目标，调用 Nav2 NavigateToPose action"""
        if not self.nav_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                'NavigateToPose action server not available, goal dropped')
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg

        self.get_logger().info(
            f'Sending goal to Nav2: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})')

        future = self.nav_action_client.send_goal_async(
            goal, feedback_callback=self.nav_feedback_callback)
        future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected')
            return
        self.get_logger().info('Nav2 goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        pass  # 可选: 处理导航反馈

    def nav_result_callback(self, future):
        result = future.result().result
        if result.result == 0:  # ActionResult.SUCCEEDED
            self.get_logger().info('Nav2 goal reached successfully')
        else:
            self.get_logger().warn(f'Nav2 goal failed with result: {result.result}')

    # ─── 下行回调: /robot_2/initialpose → /initialpose ────────────
    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """接收主车初始位姿，转发到内部 /initialpose"""
        self.pub_initialpose_internal.publish(msg)
        self.get_logger().info('initialpose forwarded to ICP localization')


def main(args=None):
    rclpy.init(args=args)
    node = Robot2SoldierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
