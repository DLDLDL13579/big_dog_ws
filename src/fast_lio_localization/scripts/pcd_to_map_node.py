#!/usr/bin/env python3
# 把 FAST-LIO 建的 3D PCD 投影成 2D OccupancyGrid 发到 /map, 供 Nav2 static_layer 使用
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header


class PcdToMap(Node):
    def __init__(self):
        super().__init__('pcd_to_map')
        self.declare_parameter('pcd_path', '/home/nvidia/dog_ws/src/dog_brain/maps/lab_3d_map.pcd')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('z_min', 0.15)   # 去地面
        self.declare_parameter('z_max', 1.8)    # 去上方
        self.declare_parameter('occ_thresh', 3) # 每格最小点数才判占用
        # --- FixE 2026-09-20: 孤立噪点剔除 ---
        self.declare_parameter('iso_enable', True)   # 是否启用孤立格剔除
        self.declare_parameter('iso_kernel', 5)      # 邻域窗口边长(格), 须为奇数
        self.declare_parameter('iso_max', 2)         # 邻域内障碍格数 <= 此值即判孤立

        path  = self.get_parameter('pcd_path').value
        res   = self.get_parameter('resolution').value
        zmin  = self.get_parameter('z_min').value
        zmax  = self.get_parameter('z_max').value
        occ_th= self.get_parameter('occ_thresh').value
        self.iso_enable = bool(self.get_parameter('iso_enable').value)
        self.iso_kernel = int(self.get_parameter('iso_kernel').value)
        self.iso_max    = int(self.get_parameter('iso_max').value)

        pcd = o3d.io.read_point_cloud(path)
        pts = np.asarray(pcd.points)
        mask = (pts[:, 2] >= zmin) & (pts[:, 2] <= zmax)
        pts = pts[mask]

        min_x, min_y = float(pts[:, 0].min()), float(pts[:, 1].min())
        max_x, max_y = float(pts[:, 0].max()), float(pts[:, 1].max())
        w = int(np.ceil((max_x - min_x) / res)) + 1
        h = int(np.ceil((max_y - min_y) / res)) + 1

        ix = ((pts[:, 0] - min_x) / res).astype(int)
        iy = ((pts[:, 1] - min_y) / res).astype(int)
        counts = np.zeros((h, w), dtype=np.int32)
        np.add.at(counts, (iy, ix), 1)

        occ = np.zeros((h, w), dtype=np.int8)   # 默认空闲
        occ[counts >= occ_th] = 100             # 占用

        # ★ FixE 2026-09-20: 剔除孤立噪点格。
        #   根因: occ_thresh=3 (每格仅 3 点即判墙) 过于宽松, 建图漂移鬼影/离群点
        #   极易凑够 3 点, 产生 134 个孤立格(占 1.4%)。其中一个落在狗旁 0.28m
        #   (世界坐标 -0.012,0.082, 5x5 邻域 25 格中仅它自己为障碍), 经
        #   inflation_radius=0.55 膨胀后把狗的起点完全封死 => G/H/I 全部规划失败。
        #   判据: 以自身为中心的 iso_kernel x iso_kernel 邻域内, 障碍格数 <= iso_max
        #   即视为孤立噪点并清除。真实柱子/墙面是连续多格, 不受影响。
        if self.iso_enable and (occ >= 100).any():
            k = int(self.iso_kernel)
            pad = k // 2
            obs_bin = (occ >= 100).astype(np.int32)
            # 2D 箱式求和: 先按行累计, 再按列累计
            padded = np.pad(obs_bin, ((pad, pad), (pad, pad)), mode='constant')
            # 积分图: 补一行一列零, 使 cs[a,b] = sum(padded[0:a, 0:b]);
            # k x k 窗口求和 = cs[k:,k:] - cs[:-k,k:] - cs[k:,:-k] + cs[:-k,:-k]
            cs = np.pad(padded, ((1, 0), (1, 0)), mode='constant')
            cs = cs.cumsum(axis=0).cumsum(axis=1)
            total = cs[k:, k:] - cs[:-k, k:] - cs[k:, :-k] + cs[:-k, :-k]
            iso = (obs_bin == 1) & (total <= int(self.iso_max))
            n_iso = int(iso.sum())
            if n_iso:
                occ[iso] = 0
                self.get_logger().info(
                    f'Isolated noise removed: {n_iso} cells '
                    f'(kernel={k}x{k}, max_neighbors={self.iso_max})')

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.info.resolution = float(res)
        msg.info.width = w
        msg.info.height = h
        msg.info.origin.position.x = min_x
        msg.info.origin.position.y = min_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = occ.flatten().astype(int).tolist()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.msg = msg

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.pub.publish(msg)
        self.get_logger().info(
            f'Published /map: {w}x{h} res={res} origin=({min_x:.2f},{min_y:.2f}) '
            f'occupied={int((occ == 100).sum())} cells')
        # 2026-09-14 性能订正: 2.0 -> 30.0
        #   原 2 秒重发每次推 1233KB 全图, 实测 576KB/s, 经 robot2_adapter 上行
        #   占满跨机链路并造成队头阻塞(延迟尖峰 p95 16ms/max 48ms)。
        #   保留 timer 而非删除: RViz 侧用 msg.header.stamp 查 TF, 需周期刷新。
        #   两个订阅者(/global_costmap、/robot2_adapter)均为 TRANSIENT_LOCAL,
        #   latched 首帧由 DDS 自动补发, 不依赖本重发。
        self.create_timer(30.0, self._repub)

    def _repub(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = PcdToMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
