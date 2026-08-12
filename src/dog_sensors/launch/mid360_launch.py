# Livox Mid-360 LiDAR launch
# Outputs:
#   /livox/lidar  (CustomMsg → PointCloud2)
#   /livox/imu    (sensor_msgs/Imu, from internal BMI088, 200Hz)

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Config path: dog_sensors/config/mid360_config.json
    pkg_share = get_package_share_directory('dog_sensors')
    config_path = os.path.join(pkg_share, 'config', 'mid360_config.json')

    livox_params = [
        {"xfer_format": 1},           # CustomMsg format (PointXYZRTL)
        {"multi_topic": 0},           # All LiDARs share same topic
        {"data_src": 0},              # 0 = live LiDAR
        {"publish_freq": 10.0},       # 10 Hz
        {"output_data_type": 0},
        {"frame_id": "livox_frame"},
        {"lvx_file_path": ""},
        {"user_config_path": config_path},
        {"cmdline_input_bd_code": "livox0000000001"},
    ]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_params,
    )

    return LaunchDescription([livox_driver])
