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
#  5. 关闭红外流(Infra1/2)。根因已钉死: Depth 与 Infra 分辨率不一致(Depth 640x480 vs
#     Infra 默认 848x480) 时，同一对 stereo 红外传感器三路流叠加 → UVC 等时端点带宽/模式
#     协商崩溃 → 内核 -71 EPROTO → v4l2 收不到帧 → Frames Timeout，点云 0 帧。
#     解法二选一: (A) 关掉 Infra 流(V1 架构不需要红外，已采用) (B) 统一分辨率 640x480。
#     若未来需要重新开启 Infra，必须同时设 depth_module.infra_profile: '640x480x30' 并
#     与 depth_profile 保持一致，否则会再次触发该超时。

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
            # 关键修复5: 关闭红外流。根因=Depth与Infra分辨率不一致(见文件头说明5)。
            # V1 架构不需要红外，关闭是最干净解法。若未来需开 Infra，必须同时设
            # 'depth_module.infra_profile': '640x480x30' 与 depth_profile 保持一致。
            'enable_infra1':              False,
            'enable_infra2':              False,
            # IMU（V1 暂不用）
            'enable_gyro':                False,
            'enable_accel':               False,
        }],
    )

    return LaunchDescription([
        realsense_node,
    ])
