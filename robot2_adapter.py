#!/usr/bin/env python3
# robot2 适配器: 把狗的真实话题改名/转换成多机协同约定名 (对齐 robot1 方案)
#  domain 11:
#   /Odometry -> /robot_2/odom (header.frame_id=robot_2/odom, child_frame_id=robot_2/base_link, 对齐 TF 树)
#   /Odometry -> /robot_2/soldier_pose + /robot_2/amcl_pose (用缓存 map->odom 换算到 map 系)
#   /map      -> /robot_2/map (latched)
#   /plan     -> /robot_2/plan
#   /tf       -> /robot_2/tf_relay (map->odom->base_link 两条都改全帧名 robot_2/*)
#  下行(不改狗运动链): /robot_2/goal_pose -> /goal_pose, /robot_2/initialpose -> /initialpose,
#                      /robot_2/cmd_vel -> /cmd_vel (零速探针已验证)
#  2026-09-09 晚修复(详见 多机协同项目/05 评估报告 P1):
#   1) on_tf 第二分支 parent 帧未改名 -> map->robot_2/base_link 断链(unconnected trees)
#   2) on_odom header.frame_id 裸 odom -> RViz Odometry 无法渲染
#   3) soldier/amcl_pose 原把 odom 系数值贴 map 标签(偏差=map->odom, 约0.79m) -> 现按缓存 map->odom 换算
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from tf2_msgs.msg import TFMessage


def latched():
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)


class Adapter(Node):
    def __init__(self):
        super().__init__('robot2_adapter')
        self.pub_odom = self.create_publisher(Odometry, '/robot_2/odom', 10)
        self.pub_soldier = self.create_publisher(PoseWithCovarianceStamped, '/robot_2/soldier_pose', latched())
        self.pub_amcl = self.create_publisher(PoseWithCovarianceStamped, '/robot_2/amcl_pose', latched())
        self.pub_map = self.create_publisher(OccupancyGrid, '/robot_2/map', latched())
        self.pub_plan = self.create_publisher(Path, '/robot_2/plan', 10)
        self.pub_tf = self.create_publisher(TFMessage, '/robot_2/tf_relay', 30)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/robot_2/cmd_vel', self.on_cmd, 10)

        self.create_subscription(Odometry, '/Odometry', self.on_odom, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched())
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 30)
        self.create_subscription(PoseStamped, '/robot_2/goal_pose', self.on_goal, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/robot_2/initialpose', self.on_init, 10)
        self._map_odom = None  # (tx, ty, th): 缓存的 map->odom (来自狗 /tf, 约7.6Hz)
        self.get_logger().info('robot2 adapter up (frame-fix 2026-09-09)')

    def on_odom(self, m):
        # /robot_2/odom: 帧名对齐 TF 树 map->robot_2/odom->robot_2/base_link
        m.header.frame_id = 'robot_2/odom'
        m.child_frame_id = 'robot_2/base_link'
        self.pub_odom.publish(m)
        # soldier/amcl: 用缓存 map->odom 把 odom 系位姿换算到 map 系 (2D 平面合成)
        px, py = m.pose.pose.position.x, m.pose.pose.position.y
        yaw = 2.0 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)
        if self._map_odom is not None:
            tx, ty, th = self._map_odom
            c, s = math.cos(th), math.sin(th)
            px, py = tx + c * px - s * py, ty + s * px + c * py
            yaw += th
        p = PoseWithCovarianceStamped()
        p.header.frame_id = 'map'
        p.header.stamp = m.header.stamp
        p.pose.pose.position.x = px
        p.pose.pose.position.y = py
        p.pose.pose.position.z = m.pose.pose.position.z
        p.pose.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.pose.orientation.w = math.cos(yaw / 2.0)
        p.pose.covariance = m.pose.covariance
        self.pub_soldier.publish(p)
        self.pub_amcl.publish(p)

    def on_map(self, m):
        self.pub_map.publish(m)

    def on_plan(self, m):
        self.pub_plan.publish(m)

    def on_tf(self, msg):
        out = TFMessage()
        for t in msg.transforms:
            pr, ch = t.header.frame_id, t.child_frame_id
            if pr == 'map' and ch == 'odom':
                q = t.transform.rotation
                self._map_odom = (t.transform.translation.x,
                                  t.transform.translation.y,
                                  2.0 * math.atan2(q.z, q.w))
                t.child_frame_id = 'robot_2/odom'
                out.transforms.append(t)
            elif pr == 'odom' and ch == 'base_link':
                # 修复: parent 帧也改名, 保证 map->robot_2/odom->robot_2/base_link 成链
                t.header.frame_id = 'robot_2/odom'
                t.child_frame_id = 'robot_2/base_link'
                out.transforms.append(t)
        if out.transforms:
            self.pub_tf.publish(out)

    def on_cmd(self, m):
        self.pub_cmd.publish(m)

    def on_goal(self, m):
        self.pub_goal.publish(m)

    def on_init(self, m):
        self.pub_init.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Adapter())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
