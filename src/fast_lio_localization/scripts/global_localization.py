#!/usr/bin/env python3
# coding=utf-8

import copy
import math
import threading
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
import tf_transformations

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header


class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('fast_lio_localization')

        # ─── Parameters ───────────────────────────────────────────
        self.declare_parameter('map_voxel_size', 0.1)
        self.declare_parameter('scan_voxel_size', 0.1)
        self.declare_parameter('freq_localization', 0.5)      # Hz
        self.declare_parameter('localization_th', 0.9)
        self.declare_parameter('fov', 2 * np.pi)
        self.declare_parameter('fov_far', 100.0)

        self.map_voxel_size    = self.get_parameter('map_voxel_size').value
        self.scan_voxel_size   = self.get_parameter('scan_voxel_size').value
        self.freq_localization = self.get_parameter('freq_localization').value
        self.localization_th   = self.get_parameter('localization_th').value
        self.FOV               = self.get_parameter('fov').value
        self.FOV_FAR           = self.get_parameter('fov_far').value

        # ─── State Variables ─────────────────────────────────────
        self.global_map    = None
        self.initialized   = False
        self.T_map_to_odom = np.eye(4)
        self.cur_odom      = None
        self.cur_scan      = None

        # ─── Publishers ──────────────────────────────────────────
        self.pub_pc_in_map   = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
        self.pub_submap      = self.create_publisher(PointCloud2, '/submap', 1)
        self.pub_map_to_odom = self.create_publisher(Odometry,     '/map_to_odom', 1)

        # ─── Subscriptions ───────────────────────────────────────
        self.create_subscription(PointCloud2,                  '/cloud_registered', self.cb_save_cur_scan, 1)
        self.create_subscription(Odometry,                    '/Odometry',         self.cb_save_cur_odom,  1)
        self._map_sub  = self.create_subscription(PointCloud2, '/global_map',               self.cb_init_map,      1)
        self._init_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.cb_init_pose,
            1
        )

        self.get_logger().info('GlobalLocalizationNode initialized.')

    def pc2_to_array(self, pc_msg: PointCloud2) -> np.ndarray:
        """PointCloud2 → (N×3) NumPy array"""
        # Humble 的 read_points_numpy 直接走 NumPy 路径，避免对 10Hz 点云
        # 在 Python 中逐点 append（该路径是 ICP 节点的主要 CPU 开销之一）。
        pts = pc2.read_points_numpy(
            pc_msg, field_names=['x', 'y', 'z'], skip_nans=True)
        pts = np.asarray(pts, dtype=np.float32)
        if pts.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return pts.reshape(-1, 3)

    def cb_init_map(self, msg: PointCloud2):
        pts = self.pc2_to_array(msg)
        if len(pts) == 0:
            self.get_logger().error('Received an empty global map; waiting for a valid map.')
            return
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        self.global_map = self.voxel_down_sample(pcd, self.map_voxel_size)
        self.get_logger().info('Global map received and downsampled.')
        self.destroy_subscription(self._map_sub)

    def cb_init_pose(self, msg: PoseWithCovarianceStamped):
        if self.global_map is None:
            self.get_logger().warn('Waiting for global map before initial localization.')
            return
        if self.cur_scan is None:
            self.get_logger().warn('Waiting for first scan before initial localization.')
            return
        if self.cur_odom is None:
            self.get_logger().warn('Waiting for odometry before initial localization.')
            return

        # /initialpose 表示 base_link 在 map 中的位姿，而本节点需要发布
        # map->odom。原实现直接把 T_map_base 当成 T_map_odom，只有在
        # odom->base_link 近似单位阵时才正确；机器人已移动后重定位会叠加偏移。
        # Nav2/RViz 的初始位姿是平面约束，因此只求 x/y/yaw 校正，
        # 保留 FAST-LIO 给出的机体 roll/pitch/z。
        T_map_to_base = self.pose_to_mat(msg)
        T_odom_to_base = self.pose_to_mat(self.cur_odom)
        yaw_map_base = math.atan2(T_map_to_base[1, 0], T_map_to_base[0, 0])
        yaw_odom_base = math.atan2(T_odom_to_base[1, 0], T_odom_to_base[0, 0])
        yaw_map_odom = math.atan2(
            math.sin(yaw_map_base - yaw_odom_base),
            math.cos(yaw_map_base - yaw_odom_base))
        c, s = math.cos(yaw_map_odom), math.sin(yaw_map_odom)
        R_map_odom = np.array([[c, -s], [s, c]])
        T = np.eye(4)
        T[:2, :2] = R_map_odom
        T[:2, 3] = (
            T_map_to_base[:2, 3]
            - R_map_odom @ T_odom_to_base[:2, 3])
        self.T_map_to_odom = T.copy()

        # 发布 map_to_odom
        odom = Odometry()
        xyz  = tf_transformations.translation_from_matrix(T)
        quat = tf_transformations.quaternion_from_matrix(T)
        odom.pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        odom.header.stamp          = self.get_clock().now().to_msg()
        odom.header.frame_id       = 'map'
        self.pub_map_to_odom.publish(odom)

        if not self.initialized:
            self.initialized = True
            period = 1.0 / self.freq_localization
            self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            'Global localization initialized from planar /initialpose '
            '(converted from map->base_link to map->odom).')

    def cb_save_cur_odom(self, msg: Odometry):
        self.cur_odom = msg

    def cb_save_cur_scan(self, msg: PointCloud2):
        msg.header.frame_id = 'odom'
        msg.header.stamp    = self.get_clock().now().to_msg()
        self.pub_pc_in_map.publish(msg)

        pts = self.pc2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        self.cur_scan = pcd

    def timer_callback(self):
        # ★ CatPaw: 定时 ICP 只做精匹配(scale=1), 不做粗匹配, 避免跑偏
        self.global_localization_refine(self.T_map_to_odom)

    def global_localization_refine(self, pose_est):
        """精匹配(仅 scale=1, 不做粗匹配), 用于定时器微调定位"""
        if self.cur_scan is None or self.cur_odom is None or self.global_map is None:
            return
        self.get_logger().info('Refining localization via ICP...')
        scan_copy = copy.deepcopy(self.cur_scan)

        submap = self.crop_global_map_in_FOV(scan_copy, pose_est, self.cur_odom)
        if len(scan_copy.points) < 10 or len(submap.points) < 10:
            self.get_logger().warn(
                'Skipping ICP refine: scan or cropped submap has too few points.',
                throttle_duration_sec=5.0)
            return

        T, fitness = self.registration_at_scale(scan_copy, submap, initial=pose_est, scale=1)
        self.get_logger().info(f'ICP refine fitness: {fitness:.3f}')

        if fitness > self.localization_th:
            self.T_map_to_odom = T.copy()
            self.T_map_to_odom[2, 3] = 0.0  # z 归零
            odom = Odometry()
            xyz  = tf_transformations.translation_from_matrix(self.T_map_to_odom)
            quat = tf_transformations.quaternion_from_matrix(self.T_map_to_odom)
            odom.pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            odom.header.stamp          = self.get_clock().now().to_msg()
            odom.header.frame_id       = 'map'
            self.pub_map_to_odom.publish(odom)
            self.get_logger().info(f'Refine updated localization (fitness={fitness:.3f}).')
        else:
            self.get_logger().warn(f'Refine failed (fitness={fitness:.3f} below threshold).')

    def global_localization(self, pose_est):
        self.get_logger().info('Performing global localization via ICP...')
        scan_copy = copy.deepcopy(self.cur_scan)

        submap = self.crop_global_map_in_FOV(scan_copy, pose_est, self.cur_odom)

        T, _       = self.registration_at_scale(scan_copy, submap, initial=pose_est, scale=5)
        T, fitness = self.registration_at_scale(scan_copy, submap, initial=T,         scale=1)
        self.get_logger().info(f'ICP fitness: {fitness:.3f}')

        if fitness > self.localization_th:
            self.T_map_to_odom = T.copy()
            self.T_map_to_odom[2, 3] = 0.0  # ★ CatPaw: 强制 z 归零，避免 ICP 漂到地下
            odom = Odometry()
            xyz  = tf_transformations.translation_from_matrix(self.T_map_to_odom)
            quat = tf_transformations.quaternion_from_matrix(self.T_map_to_odom)
            # 올바른 Odometry 메시지 필드 설정
            odom.pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            odom.header.stamp          = self.cur_odom.header.stamp
            odom.header.frame_id       = 'map'
            self.pub_map_to_odom.publish(odom)
            return True

        self.get_logger().warn('Global localization failed (fitness below threshold).')
        return False

    def crop_global_map_in_FOV(self, scan, pose_est, odom):
        T_scan     = self.pose_to_mat(odom)
        T_map2scan = np.linalg.inv(pose_est @ T_scan)

        pts = np.asarray(self.global_map.points)
        hom = np.hstack([pts, np.ones((pts.shape[0],1))])
        pts_scan = (T_map2scan @ hom.T).T

        distance = np.linalg.norm(pts_scan[:, :3], axis=1)
        if self.FOV >= 2*np.pi:
            # 360° 视野仍应按雷达径向距离裁剪。原条件 x<far
            # 会无限保留后方点，实验室地图上等价于每次使用整张图。
            mask = distance < self.FOV_FAR
        else:
            ang  = np.arctan2(pts_scan[:,1], pts_scan[:,0])
            mask = ((pts_scan[:, 0] > 0)
                    & (distance < self.FOV_FAR)
                    & (np.abs(ang) < self.FOV / 2))

        subpts = pts[mask]
        submap = o3d.geometry.PointCloud()
        submap.points = o3d.utility.Vector3dVector(subpts)

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        cloud = pc2.create_cloud_xyz32(header, subpts[::10].tolist())
        self.pub_submap.publish(cloud)

        return submap

    def registration_at_scale(self, scan, submap, initial, scale):
        def down(p): return p.voxel_down_sample(self.scan_voxel_size * scale)
        reg = o3d.pipelines.registration.registration_icp(
            down(scan), down(submap),
            max_correspondence_distance=1.0*scale,
            init=initial,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
        )
        return reg.transformation, reg.fitness

    @staticmethod
    def pose_to_mat(pose_stamped):
        t = pose_stamped.pose.pose.position
        q = pose_stamped.pose.pose.orientation
        return tf_transformations.translation_matrix([t.x,t.y,t.z]) \
             @ tf_transformations.quaternion_matrix([q.x,q.y,q.z,q.w])

    @staticmethod
    def voxel_down_sample(pcd, vs):
        try:
            return pcd.voxel_down_sample(vs)
        except:
            return o3d.geometry.voxel_down_sample(pcd, vs)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
