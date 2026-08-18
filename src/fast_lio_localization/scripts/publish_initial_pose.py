#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import argparse
import time
import tf_transformations
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped

class PublishInitialPose(Node):
    def __init__(self, x, y, z, yaw, pitch, roll):
        super().__init__('publish_initial_pose')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.publish_pose(x, y, z, yaw, pitch, roll)

    def publish_pose(self, x, y, z, yaw, pitch, roll):
        # Roll, pitch, yaw 순서에 따라 쿼터니언 계산 (ROS1과 동일한 tf_transformations 사용)
        quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.pose.pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
        # 현재 노드의 clock을 이용하여 헤더 타임스탬프 설정
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        self.publisher_.publish(pose_msg)
        self.get_logger().info(f'Initial Pose published: x={x}, y={y}, z={z}, yaw={yaw}, pitch={pitch}, roll={roll}')

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('z', type=float)
    parser.add_argument('yaw', type=float)
    parser.add_argument('pitch', type=float)
    parser.add_argument('roll', type=float)
    parsed_args = parser.parse_args()

    node = PublishInitialPose(parsed_args.x, parsed_args.y, parsed_args.z,
                              parsed_args.yaw, parsed_args.pitch, parsed_args.roll)
    # 可靠送达: 循环发布 ~3s, 确保订阅方 DDS 握手完成 (单次 spin_once 易丢消息)
    t_end = time.time() + 3.0
    while time.time() < t_end:
        node.publish_pose(parsed_args.x, parsed_args.y, parsed_args.z,
                          parsed_args.yaw, parsed_args.pitch, parsed_args.roll)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
