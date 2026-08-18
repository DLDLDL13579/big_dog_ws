#!/usr/bin/env python3
# 2.5D 高程图: PCD -> RANSAC 地面平面 -> 离地高度栅格 -> 可通行性 costmap
# 输出 /elevation_costmap (OccupancyGrid): 0=平地(轮式) 80=小台阶/缓坡(需切腿) 100=障碍(绕行)
# 供 Nav2 costmap 层 / 模态切换判断消费
import numpy as np
import open3d as o3d
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header


class ElevationMapNode(Node):
    def __init__(self):
        super().__init__('elevation_map')
        self.declare_parameter('pcd_path', '/home/nvidia/dog_ws/src/dog_brain/maps/lab_3d_map.pcd')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('ground_thresh', 0.03)   # RANSAC 地面内点阈值(m)
        self.declare_parameter('h_flat', 0.06)          # 最高点<此值=平地
        self.declare_parameter('h_step', 0.18)          # 最高点在此以下=可跨越台阶, 以上=障碍
        self.declare_parameter('cost_step', 80)         # 台阶区 cost
        self.declare_parameter('publish_interval', 3.0)

        path = self.get_parameter('pcd_path').value
        res  = self.get_parameter('resolution').value
        gth  = self.get_parameter('ground_thresh').value
        h_flat = self.get_parameter('h_flat').value
        h_step = self.get_parameter('h_step').value
        cost_step = self.get_parameter('cost_step').value

        msg = self._build(path, res, gth, h_flat, h_step, cost_step)
        if msg is None:
            self.get_logger().error('Elevation map build failed')
            return
        self.msg = msg

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(OccupancyGrid, '/elevation_costmap', qos)
        self.pub.publish(self.msg)
        self.create_timer(self.get_parameter('publish_interval').value, self._repub)

    def _build(self, path, res, gth, h_flat, h_step, cost_step):
        pcd = o3d.io.read_point_cloud(path)
        pts = np.asarray(pcd.points)
        if len(pts) == 0:
            self.get_logger().error(f'No points in {path}')
            return None

        # 确定性地面基准: 水平地面假设, 取 z 的 3% 分位 (实验室地面基本水平)
        # (RANSAC segment_plane 随机采样导致每次拟合平面不稳, 故改用确定性基准)
        GZ = np.percentile(pts[:, 2], 3)
        h = pts[:, 2] - GZ   # 离地高度
        self.get_logger().info(f'Ground base z={GZ:.3f}, h range [{h.min():.2f},{h.max():.2f}]')

        # 栅格化: 与 pcd_to_map 占用判据对齐 (z∈[occ_lo,occ_hi] 有点=占用)
        # 占用格内按最高点分级: 低矮=可切腿台阶(cost 80), 高大=绕行障碍(lethal 100)
        occ_lo, occ_hi = 0.15, 1.8       # 同 pcd_to_map 占用高度带(绝对 z, odom 系)
        step_max = 0.50                   # 台阶/高障碍分界(绝对 z)
        min_x, min_y = float(pts[:,0].min()), float(pts[:,1].min())
        ix = ((pts[:,0]-min_x)/res).astype(int); iy = ((pts[:,1]-min_y)/res).astype(int)
        W, H = int(ix.max()+1), int(iy.max()+1)

        obs = pts[(pts[:,2]>=occ_lo)&(pts[:,2]<=occ_hi)]   # 占用候选点
        oix = ((obs[:,0]-min_x)/res).astype(int); oiy=((obs[:,1]-min_y)/res).astype(int)
        cellmax = {}
        for a,b,z in zip(oix,oiy,obs[:,2]):
            k=(a,b); cellmax[k]=max(cellmax.get(k,-1e9), z)

        occ = np.zeros((H,W), dtype=np.int8)   # 默认平地(可走)
        for (a,b),mz in cellmax.items():
            occ[b,a] = 80 if mz < step_max else 100   # 低矮=台阶(可越) 高大=障碍(绕行)

        n_flat=int((occ==0).sum()); n_step=int((occ==80).sum()); n_obs=int((occ==100).sum())
        self.get_logger().info(
            f'Elevation map {W}x{H}: flat={n_flat} step(可越)={n_step} obstacle(绕行)={n_obs} '
            f'({100*n_flat/max(W*H,1):.1f}% flat)')

        msg = OccupancyGrid()
        msg.header = Header(); msg.header.frame_id = 'map'
        msg.info.resolution = float(res); msg.info.width = int(W); msg.info.height = int(H)
        msg.info.origin.position.x = min_x; msg.info.origin.position.y = min_y
        msg.info.origin.orientation.w = 1.0
        msg.data = occ.flatten().astype(int).tolist()
        return msg

    def _repub(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = ElevationMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
