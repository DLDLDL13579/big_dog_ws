#!/usr/bin/env python3

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.declare_parameter('map_file_path', '/home/nvidia/dog_ws/src/dog_brain/maps/lab_3d_map.pcd')
        self.declare_parameter('interval', 5)
        path = self.get_parameter('map_file_path').value
        interval = self.get_parameter('interval').value

        self._cloud_msg = None
        if path:
            try:
                pcd = o3d.io.read_point_cloud(path)
                self.get_logger().info(f'Loaded map from: {path}')
                self._cloud_msg = self._build_cloud(pcd)
                self.get_logger().info(
                    f'Converted map to PointCloud2, {self._cloud_msg.width if self._cloud_msg else 0} points')
            except Exception as e:
                self.get_logger().error(f'Failed to load PCD: {e}')
        else:
            self.get_logger().warn('No map_file_path provided; map not loaded')

        self.pub_map = self.create_publisher(PointCloud2, '/global_map', 1)
        self.create_timer(interval, self.publish_map)
        self.get_logger().info(f'Interval for publishing map: {interval} seconds')
        self.get_logger().info('Map Publisher Node Initialized')

    def _build_cloud(self, pcd):
        """一次性把 open3d 点云转成 PointCloud2 并缓存（避免每帧 tolist()）"""
        points = np.asarray(pcd.points, dtype=np.float32).reshape(-1, 3)
        if points.size == 0:
            return None
        n = points.shape[0]
        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = n
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12
        msg.row_step = 12 * n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = points.tobytes()
        return msg

    def publish_map(self):
        if self._cloud_msg is None:
            self.get_logger().warn('Global map is not loaded; skipping publish', throttle_duration_sec=10)
            return
        self._cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_map.publish(self._cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
