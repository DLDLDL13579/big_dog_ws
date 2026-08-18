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

        path  = self.get_parameter('pcd_path').value
        res   = self.get_parameter('resolution').value
        zmin  = self.get_parameter('z_min').value
        zmax  = self.get_parameter('z_max').value
        occ_th= self.get_parameter('occ_thresh').value

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
        self.create_timer(2.0, self._repub)

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
