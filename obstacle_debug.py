#!/usr/bin/env python3
"""诊断避障失效: 分析点云中障碍物感知 + 代价地图标记情况"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import VoxelGrid
import numpy as np
import sys


class ObstacleDebug(Node):
    def __init__(self):
        super().__init__('obstacle_debug')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cloud = None
        self.costmap = None
        self.costmap_raw = None
        self.voxel = None
        self.create_subscription(PointCloud2, '/cloud_registered_body',
                                 self.cb_cloud, qos)
        self.create_subscription(Costmap, '/local_costmap/costmap',
                                 self.cb_costmap, qos)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw',
                                 self.cb_costmap_raw, 10)
        self.create_subscription(VoxelGrid, '/local_costmap/voxel_grid',
                                 self.cb_voxel, qos)

    def cb_cloud(self, msg):
        self.cloud = msg

    def cb_costmap(self, msg):
        self.costmap = msg

    def cb_costmap_raw(self, msg):
        self.costmap_raw = msg

    def cb_voxel(self, msg):
        self.voxel = msg


def main():
    rclpy.init()
    node = ObstacleDebug()
    import time
    t0 = time.time()
    while time.time() - t0 < 8 and (node.cloud is None or node.costmap_raw is None):
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.cloud is None:
        print("!! 未收到 /cloud_registered_body")
    else:
        pts = np.array(list(pc2.read_points(node.cloud, field_names=('x', 'y', 'z'),
                                            skip_nans=True)))
        print(f"=== 点云分析 ({len(pts)} 点) ===")
        # 前方 0.2~5m 的点 (base_link 系)
        fwd = pts[(pts['x'] > 0.2) & (pts['x'] < 5.0) & (np.abs(pts['y']) < 2.0)]
        print(f"前方 0.2~5m 点数: {len(fwd)}")
        if len(fwd) > 0:
            # 按 0.5m 距离分段统计
            for d in np.arange(0.5, 5.5, 0.5):
                seg = fwd[(fwd['x'] > d - 0.5) & (fwd['x'] <= d)]
                if len(seg) > 0:
                    zmin, zmax = seg['z'].min(), seg['z'].max()
                    ymin, ymax = seg['y'].min(), seg['y'].max()
                    print(f"  x∈({d-0.5:.1f},{d:.1f}]m: {len(seg):5d} 点, "
                          f"z∈[{zmin:+.2f},{zmax:+.2f}], y∈[{ymin:+.2f},{ymax:+.2f}]")
                else:
                    print(f"  x∈({d-0.5:.1f},{d:.1f}]m:     0 点  ← 盲区!")

    if node.costmap_raw is None:
        print("!! 未收到 /local_costmap/costmap_raw")
    else:
        cm = node.costmap_raw
        md = cm.info
        data = np.array(cm.data).reshape(md.height, md.width)
        print(f"\n=== 代价地图 raw ({md.width}x{md.height}, res={md.resolution}m) ===")
        print(f"原点: ({md.origin.position.x:.2f}, {md.origin.position.y:.2f})")
        # OccupancyGrid: -1=unknown, 0=free, 100=lethal
        vals = data.flatten()
        print(f"unknown(-1): {(vals==-1).sum()}, free(0): {(vals==0).sum()}, "
              f"lethal(100): {(vals==100).sum()}, "
              f"高代价(50-99): {((vals>=50)&(vals<100)).sum()}, "
              f"低代价(1-49): {((vals>0)&(vals<50)).sum()}")
        # 机器人位于 rolling window 中心
        cy, cx = md.height // 2, md.width // 2
        print(f"\n机器人前方代价切片 (y=中心行, x 从机器人往前 0~3m):")
        cells_fwd = int(3.0 / md.resolution)
        row = data[cy, cx:cx + cells_fwd]
        step = int(0.3 / md.resolution)
        for i in range(0, cells_fwd, step):
            seg = row[i:i + step]
            print(f"  前方 {i*md.resolution:.1f}~{(i+step)*md.resolution:.1f}m: "
                  f"max={seg.max()}, mean={seg.mean():.0f}")
        # 打印 lethal 点的坐标 (地图系)
        lethal = np.argwhere(data == 100)
        print(f"\nlethal 障碍物单元格数: {len(lethal)}")
        if len(lethal) > 0:
            # 转换为机器人系坐标 (机器人位于中心)
            for ly, lx in lethal[:20]:
                rx = (lx - cx) * md.resolution
                ry = (ly - cy) * md.resolution
                print(f"  障碍 @ robot系 ({rx:+.2f}, {ry:+.2f}) m")

    if node.voxel is not None:
        v = node.voxel
        print(f"\n=== 体素网格 ({v.size_x}x{v.size_y}x{v.size_z}, "
              f"res={v.resolutions[0]:.2f}m, z_res={v.resolutions[2]:.2f}m) ===")
        print(f"origin: ({v.origin.x:.2f}, {v.origin.y:.2f}, {v.origin.z:.2f})")
        # 统计标记的 voxel (bit array, 每字节 8 个 z voxel)
        marked = sum(bin(b).count('1') for b in v.data)
        print(f"已标记 voxel 数: {marked} / {v.size_x*v.size_y*v.size_z}")
        # 找出标记的 voxel 的 xy 位置 (相对地图原点)
        # data 是 x_size * y_size 字节, 每字节 8 个 z 位 (z从低到高)
        nz_bytes = (v.size_z + 7) // 8
        obstacles = []
        for i, b in enumerate(v.data):
            if b:
                ix = i % v.size_x
                iy = (i // v.size_x) % v.size_y
                # x 从原点向 x+ 方向, y 从原点向 y+ 方向
                wx = v.origin.x + (ix + 0.5) * v.resolutions[0]
                wy = v.origin.y + (iy + 0.5) * v.resolutions[1]
                # z 位掩码
                for bit in range(8):
                    if b & (1 << bit):
                        wz = v.origin.z + bit * v.resolutions[2]
                        obstacles.append((wx, wy, wz))
        print(f"标记的 voxel 位置 (地图系, 前 30 个):")
        for wx, wy, wz in obstacles[:30]:
            print(f"  ({wx:+.2f}, {wy:+.2f}, {wz:+.2f})")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
