# D435i depth camera launch
# Starts realsense2_camera_node with depth point cloud enabled, RGB enabled
#
# 修复说明 (2026-08-13) — 基于 Jetson ARM/NEON 版 realsense2_camera 4.58.3 实测：
#  1. 点云参数名: ARM 版带 __neon_ 后缀 → 必须用 pointcloud__neon_.enable (不是 pointcloud.enable)
#  2. namespace 硬编码 'camera': 无论 launch 怎么设 namespace / node name，
#     node FQN 都会带 /camera 前缀。用 __ns:=/ 覆盖，使 FQN=/camera，
#     点云 topic 天然 = /camera/depth/color/points，精确匹配 nav2_3d_params.yaml
#  3. frame 根 = {camera_name}_{base_frame_id} → camera_name='camera' + base_frame_id='link'
#     = 'camera_link'，匹配 URDF 的 camera_link link，TF 链 base_link→camera_link 打通
#  4. 修正 profile: D435i 不支持 1280x720x30（深度仅 6Hz、彩色仅 15Hz）→ 改 640x480x30

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        output='screen',
        # 关键修复2: 覆盖 realsense2_camera 4.x 硬编码的 namespace='camera'
        arguments=['--ros-args', '-r', '__ns:=/'],
        parameters=[{
            # 关键修复3: frame 根 = camera_name + '_' + base_frame_id = 'camera_link'
            'camera_name':                'camera',
            'base_frame_id':              'link',
            # Depth stream
            'enable_depth':               True,
            'depth_module.depth_profile': '640x480x30',
            # Color stream (用于点云着色)
            'enable_color':               True,
            'rgb_camera.color_profile':   '640x480x30',
            # 关键修复1: ARM NEON 版点云参数名
            'pointcloud__neon_.enable':   True,
            # 对齐（深度对齐到彩色，点云带 RGB）
            'align_depth.enable':         True,
            # IMU（V1 暂不用）
            'enable_gyro':                False,
            'enable_accel':               False,
        }],
    )

    return LaunchDescription([
        realsense_node,
    ])
