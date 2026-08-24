#!/usr/bin/env python3
# 实时前方台阶检测: D435i 近距点云 -> base_link -> ROI 台阶判断
# 发布 /step_ahead(Bool) + /step_height(Float32)
# 2026-08-21 优化: OPENBLAS_NUM_THREADS=1 + 10Hz 定时器
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
os.environ['MKL_NUM_THREADS'] = '1'
os.environ['NUMEXPR_NUM_THREADS'] = '1'

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32
import tf2_ros


class StepDetector(Node):
    def __init__(self):
        super().__init__('step_detector')
        self.declare_parameter('cloud_topic', '/camera/depth/color/points')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('roi_x_min', 0.20)
        self.declare_parameter('roi_x_max', 1.50)
        self.declare_parameter('roi_y', 0.40)
        self.declare_parameter('min_points', 30)
        self.declare_parameter('step_h_min', 0.05)
        self.declare_parameter('step_h_max', 0.20)
        self.declare_parameter('rise_ratio', 0.15)
        self.declare_parameter('process_freq', 10.0)

        p = lambda k: self.get_parameter(k).value
        self.roi = (p('roi_x_min'), p('roi_x_max'), p('roi_y'))
        self.min_pts = p('min_points')
        self.hmin, self.hmax = p('step_h_min'), p('step_h_max')
        self.ratio_th = p('rise_ratio')
        self.base_frame = p('base_frame')
        self.process_freq = p('process_freq')

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        self.sub = self.create_subscription(PointCloud2, p('cloud_topic'), self.cb_save, 10)
        self.timer = self.create_timer(1.0 / self.process_freq, self.cb_process)
        self.pub_step = self.create_publisher(Bool, '/step_ahead', 10)
        self.pub_h = self.create_publisher(Float32, '/step_height', 10)
        self.latest_cloud = None
        self.get_logger().info(f'Step detector ready ({self.process_freq}Hz, D435i near-range)')

    def cb_save(self, msg):
        self.latest_cloud = msg

    def cb_process(self):
        msg = self.latest_cloud
        if msg is None:
            return
        try:
            tf = self.tf_buf.lookup_transform(self.base_frame, msg.header.frame_id, Time())
        except Exception:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        R = self._quat_to_R(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])

        n_float = msg.point_step // 4
        arr = np.frombuffer(msg.data, dtype=np.float32)
        pts = arr.reshape(-1, n_float)[:, :3]
        pts = pts[~np.isnan(pts).any(axis=1)]
        if len(pts) == 0:
            return
        pts_b = pts @ R.T + T

        x0, x1, yh = self.roi
        roi = pts_b[(pts_b[:,0] > x0) & (pts_b[:,0] < x1) & (np.abs(pts_b[:,1]) < yh)]
        step = Bool()
        step.data = False
        hmsg = Float32()
        hmsg.data = 0.0
        if len(roi) >= self.min_pts:
            z = roi[:, 2]
            ground = np.percentile(z, 10)
            raised = z[(z > ground + self.hmin)]
            if len(raised) > self.min_pts * self.ratio_th:
                h = float(np.median(raised) - ground)
                if self.hmin <= h <= self.hmax:
                    step.data = True
                    hmsg.data = h
                elif h > self.hmax:
                    step.data = True
                    hmsg.data = h
        self.pub_step.publish(step)
        self.pub_h.publish(hmsg)

    @staticmethod
    def _quat_to_R(x, y, z, w):
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def main():
    rclpy.init()
    node = StepDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
