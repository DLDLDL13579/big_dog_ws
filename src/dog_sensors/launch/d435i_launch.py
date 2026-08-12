# D435i depth camera launch
# Starts realsense2_camera_node with depth point cloud enabled, RGB optional

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d435i_camera',
        output='screen',
        parameters=[{
            # Depth stream
            'enable_depth':          True,
            'depth_module.depth_profile': '1280x720x30',
            # Point cloud
            'enable_color':          True,
            'rgb_camera.color_profile': '1280x720x30',
            # Point cloud (to /camera/depth/color/points)
            'pointcloud.enable':     True,
            # IMU（可选，V1 暂不用）
            'enable_gyro':           False,
            'enable_accel':          False,
            # Frame IDs
            'camera_name':           'd435i',
            'depth_frame_id':        'camera_link',
            'infra_frame_id':        'camera_link',
            'color_frame_id':        'camera_link',
            # 对齐
            'align_depth.enable':    True,
            # 序列号（如果不连接多台可不设）
            # 'serial_no':           '',
            # 发布频率
            'pointcloud.pointcloud_qos': 'SENSOR_DATA',
        }],
    )

    return LaunchDescription([
        realsense_node,
    ])
