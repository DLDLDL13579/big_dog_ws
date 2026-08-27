#!/usr/bin/env python3
"""
nav_goal_test.py — P0 导航端到端实测脚本 (v6 交接书 §21)
==============================================================
在 Jetson 上运行（ros2 humble + dog_ws 环境, 导航模式栈）。
流程: 读狗当前 map 帧位姿 → 计算正前方 --dist 米的导航目标 →
发送 NavigateToPose → 全程监控(cmd_vel/位移) → 超程/超时/超速自动取消。

用法:
  python3 nav_goal_test.py --dist 2.0            # dry-run, 只打印计划
  python3 nav_goal_test.py --dist 2.0 --confirm  # 正式执行

安全护栏:
  1. 必须显式 --confirm; 5s 倒计时 Ctrl+C 可中止
  2. 超程保险: 位移 > dist+1.5m → 自动取消导航
  3. 超时保险: 距发 goal 超 --timeout s(默认 60) → 取消
  4. 超速监控: /cmd_vel |x| > 0.8 m/s → 取消并告警
     (Nav2 max_vel_x=0.75, 正常应远低于此)
  5. 全程遥控器 SWE 急停优先级最高(直接断电机), 脚本护栏只是第二道

观测: 位移双源交叉校验 (2026-08-26 教训修复):
  - /localization (ICP, map 帧 ~10Hz) — 导航参考系, 但可能漂移/姿态敏感, 不能单独作地面真值
  - /odom (UpBoard 轮腿里程计 ~475Hz) — 狗体是否真动的地面真值
  ★ 门控: 若 /localization 位移 >0.3m 而 /odom 位移 <0.02m → 判定"狗体未运动/定位漂移"
     立即取消并报错 (旧版仅用 ICP 差分, 狗未站立时曾误报 1.76m 位移)。
     ⚠️ 2026-08-26 实测: RL 行走时 /odom 少计约 8 倍 (实走 2m 只报 0.28m),
     故门控阈值放宽到 0.02m 只拦"完全不动", 两源分歧时以现场目视为准。
  注意: FAST-LIO /Odometry 的 twist 恒为 0, 不能用作速度源(§21 坑)。

预期(当前 bridge v2 含非线性映射):
  Nav2 巡航 cmd_vel ~0.3-0.5 m/s → 映射后实际 ~0.4-0.6 m/s(放大~1.3-1.6x)
  接近目标时 Nav2 减速 → 映射有 0.24 m/s 地板, 最后一段可能"憋"着走完
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

OVERSPD_CMD = 0.8      # m/s /cmd_vel 告警阈值
OVERRUN_MARGIN = 1.5   # m 超程保险余量
GOAL_TOL_CHECK = 0.5   # m 判定"到达"的容差(报告用)


def yaw_from_quat(q):
    """四元数 → 偏航角"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class NavGoalTest(Node):
    def __init__(self, dist, timeout):
        super().__init__('nav_goal_test')
        self.loc_sub = self.create_subscription(
            Odometry, '/localization', self.on_loc, 20)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_up_odom, 50)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.on_cmd, 20)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.dist = dist
        self.timeout = timeout
        self.loc_count = 0
        self.last_p = None            # (x, y) 定位
        self.start_p = None           # 发 goal 时刻位姿
        self.dist_travelled = 0.0     # vx 无门控路径累积(用差分位姿, 静止已验证稳)
        self.up_last_p = None         # UpBoard /odom 位置 (地面真值)
        self.up_travelled = 0.0       # UpBoard 里程计累积位移
        self.cmd_max = 0.0
        self.cmd_last = 0.0
        self.goal_done = False
        self.goal_result = None
        self.cancel_requested = False
        self.abort_reason = None
        # 后台 executor(rclpy Node.executor 是 property, 不能撞名 → exc)
        self.exc = SingleThreadedExecutor()
        self.exc.add_node(self)
        self.spin_thread = threading.Thread(target=self.exc.spin, daemon=True)
        self.spin_thread.start()

    # ---------- 回调 ----------
    def on_loc(self, msg):
        self.loc_count += 1
        p = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._last_yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.start_p is None:
            self.last_p = p
            return
        if self.last_p is not None:
            dx = p[0] - self.last_p[0]
            dy = p[1] - self.last_p[1]
            self.dist_travelled += (dx * dx + dy * dy) ** 0.5
        self.last_p = p

    def on_cmd(self, msg):
        self.cmd_last = msg.linear.x
        self.cmd_max = max(self.cmd_max, abs(msg.linear.x))

    def on_up_odom(self, msg):
        """UpBoard 轮腿里程计 — 狗体是否真动的地面真值"""
        p = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.up_last_p is not None:
            dx = p[0] - self.up_last_p[0]
            dy = p[1] - self.up_last_p[1]
            self.up_travelled += (dx * dx + dy * dy) ** 0.5
        self.up_last_p = p

    # ---------- 主流程 ----------
    def run(self):
        # 取当前位姿
        self.get_logger().info('等待 /localization ...')
        t_wait = time.monotonic()
        while self.loc_count < 20 and time.monotonic() - t_wait < 5.0:
            time.sleep(0.1)
        if self.loc_count < 20:
            self.get_logger().error('ICP 定位不在线 → 中止')
            return 2
        # 位姿稳定性(2s 漂移)
        a = self.last_p
        time.sleep(2.0)
        b = self.last_p
        drift = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        if drift > 0.08:
            self.get_logger().error(f'定位未收敛(2s 漂移 {drift:.3f}m) → 中止')
            return 2
        self.get_logger().info(f'定位稳定(2s 漂移 {drift * 1000:.0f}mm)')

        # 等待与采样位姿(取最新一帧)
        self.get_logger().info('等待 NavigateToPose action server ...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose 服务不在线 → 中止')
            return 2

        # 倒计时
        self.get_logger().info(f'目标: 前方 {self.dist}m。5s 倒计时(Ctrl+C 中止)')
        for i in range(5, 0, -1):
            self.get_logger().info(f'  {i} ...')
            time.sleep(1.0)

        # 用最新位姿计算 goal
        x0, y0 = self.last_p
        yaw = self._last_yaw  # on_loc 每帧更新
        gx = x0 + self.dist * math.cos(yaw)
        gy = y0 + self.dist * math.sin(yaw)
        self.get_logger().info(
            f'当前位置 ({x0:+.2f},{y0:+.2f}) yaw={math.degrees(yaw):.0f}° → '
            f'目标 ({gx:+.2f},{gy:+.2f})')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # 朝向无所谓, 差速原地转

        # 记起始位姿(重置位移基准)
        self.start_p = self.last_p
        self.dist_travelled = 0.0
        self.up_travelled = 0.0
        t0 = time.monotonic()

        self.get_logger().info('>>> 发送 NavigateToPose goal')
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)
        goal_handle = None
        while not send_future.done():
            time.sleep(0.1)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('goal 被拒!(目标点在障碍/地图外?) → 结束')
            return 3
        self.get_logger().info('goal 已接受, 监控导航过程 ...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

        # 监控循环
        overrun = self.dist + OVERRUN_MARGIN
        while True:
            time.sleep(0.2)
            t = time.monotonic() - t0
            # 打印进度(每 2s)
            if int(t * 5) % 10 == 0:
                self.get_logger().info(
                    f'  t={t:4.1f}s 位移(ICP)={self.dist_travelled:.2f}m '
                    f'位移(UpBoard)={self.up_travelled:.2f}m '
                    f'cmd_vx={self.cmd_last:+.2f} (max {self.cmd_max:.2f})')
            # 护栏
            if self.cmd_max > OVERSPD_CMD:
                self.abort_reason = f'cmd_vel 超速 {self.cmd_max:.2f} m/s'
                break
            # ★ 双源交叉校验: 定位说动了但狗体里程计几乎没动 → 狗未真运动/定位漂移
            # (阈值 0.02 而非 0.05: 实测 RL 行走 /odom 少计约 8 倍, 过严会误杀真实运动)
            if self.dist_travelled > 0.3 and self.up_travelled < 0.02 and t > 5.0:
                self.abort_reason = (
                    f'交叉校验失败: ICP位移={self.dist_travelled:.2f}m 但 '
                    f'UpBoard里程计仅{self.up_travelled:.2f}m → 狗体未运动或定位漂移 '
                    f'(确认狗已站立且处于运动模式!)')
                break
            if self.dist_travelled > overrun:
                self.abort_reason = f'超程 位移>{overrun:.2f}m'
                break
            if t > self.timeout:
                self.abort_reason = f'超时 {self.timeout}s'
                break
            if self.goal_done:
                break
            if self.cancel_requested:
                break

        if self.abort_reason and goal_handle is not None:
            self.get_logger().error(f'>>> 护栏触发: {self.abort_reason} → 取消导航')
            self.cancel_requested = True
            try:
                goal_handle.cancel_goal()
            except Exception as e:
                self.get_logger().warn(f'cancel 异常: {e}')
            # 等取消生效
            t_c = time.monotonic()
            while not self.goal_done and time.monotonic() - t_c < 5.0:
                time.sleep(0.1)
        else:
            # 等结果(最多再 5s)
            t_c = time.monotonic()
            while not self.goal_done and time.monotonic() - t_c < 5.0:
                time.sleep(0.1)

        time.sleep(1.0)  # 让尾部 cmd_vel 到达
        self.report()
        return 0

    def _goal_response_cb(self, future):
        gh = future.result()
        if gh is not None and gh.accepted:
            self.get_logger().info('goal response: accepted')
        else:
            self.get_logger().error('goal response: rejected')

    def _result_cb(self, future):
        self.goal_done = True
        try:
            self.goal_result = future.result().result
            self.get_logger().info(f'导航结果返回 (error_code={self.goal_result.error_code})')
        except Exception as e:
            self.get_logger().warn(f'result 异常: {e}')

    # ---------- 报告 ----------
    def report(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('导航实测报告')
        self.get_logger().info(f'  目标: 前方 {self.dist} m')
        self.get_logger().info(f'  位移(定位): {self.dist_travelled:.3f} m')
        self.get_logger().info(f'  位移(UpBoard地面真值): {self.up_travelled:.3f} m')
        self.get_logger().info(f'  /cmd_vel: max={self.cmd_max:.3f} m/s, 末值={self.cmd_last:+.3f} m/s')
        if self.abort_reason:
            self.get_logger().error(f'  护栏: {self.abort_reason}')
        if self.goal_result is not None:
            ec = getattr(self.goal_result, 'error_code', None)
            if ec is None:
                self.get_logger().info('  NavigateToPose 返回 (Humble Result 无 error_code 字段, 以护栏/位移判定)')
            else:
                self.get_logger().info(
                    f'  NavigateToPose error_code: {ec} '
                    f'({"成功到达" if ec == 0 else "未到达(被取消/失败)"})')
        self.get_logger().info('-' * 60)
        end_dist = abs(self.dist_travelled - self.dist)
        ec = getattr(self.goal_result, 'error_code', None) if self.goal_result is not None else None
        if end_dist < GOAL_TOL_CHECK and (ec == 0 or ec is None) and not self.abort_reason:
            self.get_logger().info('判定: ★ 导航端到端成功(误差<0.5m)')
        elif self.abort_reason:
            self.get_logger().error('判定: 护栏介入, 未正常到达')
        else:
            self.get_logger().warn('判定: 部分完成, 结合现场观察判定')
        self.get_logger().info('=' * 60)


def main():
    ap = argparse.ArgumentParser(description='P0 导航端到端实测(带护栏)')
    ap.add_argument('--dist', type=float, default=2.0, help='前方目标距离 m (≤3.0)')
    ap.add_argument('--timeout', type=float, default=60.0, help='导航超时 s')
    ap.add_argument('--confirm', action='store_true', help='必须显式传入才执行')
    args = ap.parse_args()

    if args.dist > 3.0 or args.dist <= 0:
        print('dist 必须在 (0, 3.0] 区间。')
        sys.exit(1)

    if not args.confirm:
        print('未传 --confirm, 仅打印计划:')
        print(f'  读取当前位姿 → 目标为正前方 {args.dist} m, 超时 {args.timeout}s')
        print('  护栏: 超程 +1.5m / cmd_vel>0.8 / 超时 自动取消')
        sys.exit(0)

    rclpy.init()
    node = NavGoalTest(args.dist, args.timeout)
    rc = 1
    try:
        rc = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('用户中止!')
    finally:
        try:
            node.exc.shutdown()
            node.spin_thread.join(timeout=2.0)
        except Exception:
            pass
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
