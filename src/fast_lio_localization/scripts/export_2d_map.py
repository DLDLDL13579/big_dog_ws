#!/usr/bin/env python3
"""
export_2d_map.py — 将 FAST-LIO 建的 3D PCD 导出为 2D OccupancyGrid 文件

功能:
  1. 读取 3D PCD 点云
  2. 投影为 2D 栅格 (与 pcd_to_map_node.py 逻辑一致)
  3. 保存为 .pgm + .yaml (Nav2 map_server 格式)
  4. 可直接输出到主车/robot1 导航地图目录

三车地图路径:
  - 主车:   ~/wheeltec_ros2/src/wheeltec_robot_rtab/my_map.pgm + my_map.yaml
  - robot1: ~/robot1_ws/src/robot1_nav/maps/lab_map.pgm + lab_map.yaml
  - 机械狗: ~/dog_ws/maps/ (本地导出)

用法:
  # 基本用法: 从默认路径读取，输出到 ~/dog_ws/maps/
  python3 export_2d_map.py

  # 指定输入和输出，命名为主车的 my_map
  python3 export_2d_map.py --pcd /path/to/map.pcd --output ~/dog_ws/maps --name my_map

  # 直接推送到主车地图目录 (自动 SCP)
  python3 export_2d_map.py --pcd map.pcd --push-to-main

  # 推送到 robot1 地图目录 (命名需为 lab_map)
  python3 export_2d_map.py --pcd map.pcd --name lab_map --push-to-robot1

  # 推送到所有车辆 (主车 + robot1)
  python3 export_2d_map.py --pcd map.pcd --name my_map --push-to-all

  # 同时发布到 /map 话题
  python3 export_2d_map.py --pcd map.pcd --publish
"""

import argparse
import numpy as np
import os
import sys

try:
    import open3d as o3d
except ImportError:
    print("Error: open3d not installed. Run: pip3 install open3d")
    sys.exit(1)


def pcd_to_occupancy_grid(pcd_path, resolution=0.05, z_min=0.15, z_max=1.8, occ_thresh=3):
    """将 3D PCD 点云投影为 2D 栅格"""
    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points)

    # 高度过滤：去除地面和上方点
    mask = (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
    pts = pts[mask]

    if len(pts) == 0:
        raise ValueError("No points after filtering! Check z_min/z_max parameters.")

    # 计算栅格尺寸
    min_x, min_y = float(pts[:, 0].min()), float(pts[:, 1].min())
    max_x, max_y = float(pts[:, 0].max()), float(pts[:, 1].max())
    width = int(np.ceil((max_x - min_x) / resolution)) + 1
    height = int(np.ceil((max_y - min_y) / resolution)) + 1

    # 统计每格点数
    ix = ((pts[:, 0] - min_x) / resolution).astype(int)
    iy = ((pts[:, 1] - min_y) / resolution).astype(int)
    counts = np.zeros((height, width), dtype=np.int32)
    np.add.at(counts, (iy, ix), 1)

    # 判定占用: 0=未知, 100=占用, 其余为空闲
    occ = np.zeros((height, width), dtype=np.int8)
    occ[counts >= occ_thresh] = 100

    return {
        'data': occ,
        'width': width,
        'height': height,
        'origin': [min_x, min_y],
        'resolution': resolution,
        'occupied_cells': int((occ == 100).sum()),
        'free_cells': int((occ == 0).sum()),
    }


def save_pgm(occ_data, output_path):
    """保存为 PGM 格式 (Nav2 map_server 兼容)"""
    # Nav2 期望: 0=未知, 255=空闲, 100=占用 (取反)
    # PGM 值: 0-255, 其中 255=白=空闲, 0=黑=占用
    h, w = occ_data['data'].shape
    img = np.full((h, w), 255, dtype=np.uint8)  # 默认空闲
    img[occ_data['data'] == 100] = 0            # 占用=黑色

    # 写入 PGM
    with open(output_path, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'{w} {h}\n'.encode())
        f.write(b'255\n')
        f.write(img.tobytes())


def save_yaml(map_info, output_path, pgm_filename):
    """保存 YAML 元数据"""
    yaml_content = f"""image: {pgm_filename}
mode: trinary
resolution: {map_info['resolution']}
origin: [{map_info['origin'][0]:.2f}, {map_info['origin'][1]:.2f}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.19
"""
    with open(output_path, 'w') as f:
        f.write(yaml_content)


def publish_to_ros(map_info, frame_id='map'):
    """发布到 /map 话题 (可选)"""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from nav_msgs.msg import OccupancyGrid
        from std_msgs.msg import Header

        rclpy.init()
        node = rclpy.create_node('export_map_publisher')

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = frame_id
        msg.info.resolution = map_info['resolution']
        msg.info.width = map_info['width']
        msg.info.height = map_info['height']
        msg.info.origin.position.x = map_info['origin'][0]
        msg.info.origin.position.y = map_info['origin'][1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = map_info['data'].flatten().astype(int).tolist()
        msg.header.stamp = node.get_clock().now().to_msg()

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        pub = node.create_publisher(OccupancyGrid, '/map', qos)

        # 发布几次确保被订阅者接收
        import time
        for _ in range(5):
            pub.publish(msg)
            time.sleep(0.1)

        node.get_logger().info(f'Published /map: {map_info["width"]}x{map_info["height"]}')
        node.destroy_node()
        rclpy.shutdown()
        return True
    except Exception as e:
        print(f"Warning: Could not publish to ROS: {e}")
        return False


def push_to_vehicle(yaml_path, pgm_path, vehicle_ip, vehicle_password, remote_path, vehicle_name):
    """通过 SSH 推送地图到指定车辆"""
    import subprocess
    import os

    yaml_name = os.path.basename(yaml_path)
    pgm_name = os.path.basename(pgm_path)

    print(f"\n推送地图到 {vehicle_name} ({vehicle_ip}) {remote_path} ...")

    # 推送 YAML
    cmd = f"sshpass -p '{vehicle_password}' scp -o StrictHostKeyChecking=no {yaml_path} sunrise@{vehicle_ip}:{remote_path}/{yaml_name}" if 'sunrise' in remote_path else f"sshpass -p '{vehicle_password}' scp -o StrictHostKeyChecking=no {yaml_path} nvidia@{vehicle_ip}:{remote_path}/{yaml_name}"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode == 0:
        print(f"  ✓ {yaml_name} → {remote_path}")
    else:
        print(f"  ✗ Failed to push {yaml_name}: {result.stderr}")

    # 推送 PGM
    cmd = f"sshpass -p '{vehicle_password}' scp -o StrictHostKeyChecking=no {pgm_path} sunrise@{vehicle_ip}:{remote_path}/{pgm_name}" if 'sunrise' in remote_path else f"sshpass -p '{vehicle_password}' scp -o StrictHostKeyChecking=no {pgm_path} nvidia@{vehicle_ip}:{remote_path}/{pgm_name}"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode == 0:
        print(f"  ✓ {pgm_name} → {remote_path}")
    else:
        print(f"  ✗ Failed to push {pgm_name}: {result.stderr}")


def main():
    parser = argparse.ArgumentParser(description='Export 3D PCD to 2D map (yaml+pgm)')
    parser.add_argument('--pcd', type=str, default='/home/nvidia/dog_ws/src/dog_brain/maps/lab_3d_map.pcd',
                        help='Input PCD file path')
    parser.add_argument('--output', type=str, default='/home/nvidia/dog_ws/maps',
                        help='Output directory')
    parser.add_argument('--name', type=str, default='my_map',
                        help='Map filename (without extension)')
    parser.add_argument('--resolution', type=float, default=0.05,
                        help='Map resolution (m/cell)')
    parser.add_argument('--z-min', type=float, default=0.15,
                        help='Min z for filtering (m)')
    parser.add_argument('--z-max', type=float, default=1.8,
                        help='Max z for filtering (m)')
    parser.add_argument('--occ-thresh', type=int, default=3,
                        help='Min points per cell to mark as occupied')
    parser.add_argument('--publish', action='store_true',
                        help='Also publish to /map topic')
    parser.add_argument('--push-to-main', action='store_true',
                        help='Push map to main vehicle via SSH')
    parser.add_argument('--push-to-robot1', action='store_true',
                        help='Push map to robot1 via SSH')
    parser.add_argument('--push-to-all', action='store_true',
                        help='Push map to both main vehicle and robot1')
    parser.add_argument('--main-path', type=str, default='/home/nvidia/wheeltec_ros2/src/wheeltec_robot_rtab',
                        help='Main vehicle map directory (default: wheeltec_robot_rtab)')
    parser.add_argument('--robot1-path', type=str, default='/home/sunrise/robot1_ws/src/robot1_nav/maps',
                        help='Robot1 map directory')

    args = parser.parse_args()

    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)

    print(f"读取点云: {args.pcd}")
    if not os.path.exists(args.pcd):
        print(f"Error: PCD file not found: {args.pcd}")
        sys.exit(1)

    print(f"转换为 2D 栅格: resolution={args.resolution}, z=[{args.z_min}, {args.z_max}]")
    map_info = pcd_to_occupancy_grid(
        args.pcd, args.resolution, args.z_min, args.z_max, args.occ_thresh)

    print(f"栅格大小: {map_info['width']}x{map_info['height']}")
    print(f"原点: ({map_info['origin'][0]:.2f}, {map_info['origin'][1]:.2f})")
    print(f"占用格数: {map_info['occupied_cells']}, 空闲格数: {map_info['free_cells']}")

    # 保存文件
    pgm_path = os.path.join(args.output, f"{args.name}.pgm")
    yaml_path = os.path.join(args.output, f"{args.name}.yaml")

    save_pgm(map_info, pgm_path)
    save_yaml(map_info, yaml_path, f"{args.name}.pgm")

    print(f"\n输出文件:")
    print(f"  {pgm_path}")
    print(f"  {yaml_path}")

    # 可选: 发布到话题
    if args.publish:
        publish_to_ros(map_info)

    # 可选: 推送到主车
    if args.push_to_main or args.push_to_all:
        push_to_vehicle(yaml_path, pgm_path, '192.168.31.43', 'nvidia', args.main_path, '主车')
        # 如果名字是 WHEELTEC，也推送到 Nav2 目录
        if args.name == 'WHEELTEC':
            push_to_vehicle(yaml_path, pgm_path, '192.168.31.43', 'nvidia',
                          '/home/nvidia/wheeltec_ros2/src/wheeltec_robot_nav2/map', '主车 Nav2')

    # 可选: 推送到 robot1
    if args.push_to_robot1 or args.push_to_all:
        push_to_vehicle(yaml_path, pgm_path, '192.168.31.47', 'sunrise', args.robot1_path, 'robot1')

    print("\n完成!")


if __name__ == '__main__':
    main()
