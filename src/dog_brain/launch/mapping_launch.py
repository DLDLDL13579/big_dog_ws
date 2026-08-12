# 机械狗建图模式 (Mapping Mode)
# 启动: FAST-LIO2 (LiDAR+IMU 紧耦合) + 传感器 + 状态广播 + 桥接
#
# 输出:
#   /livox/lidar    (PointCloud2, 10Hz)
#   /livox/imu      (Imu, 200Hz, BMI088)
#   /odom            (NavSatOdometry, lcm_bridge)
#   /joint_states    (sensor_msgs)
#   TF: map → odom → base_link → {livox_frame, camera_link, legs}
#
# 建图完成后用 map_saver_cli 保存 3D PCD:
#   ros2 run pcl_ros pointcloud_to_pcd input:=/registered_scan

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    # ── Sensors ─────────────────────────────────────────────────
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('dog_sensors'), 'launch', 'all_sensors_launch.py'
            ])
        )
    )

    # ── Robot Description ───────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': PathJoinSubstitution([
                FindPackageShare('dog_description'), 'urdf', 'dog.urdf'
            ])
        }]
    )

    # ── LCM Bridge (odom + joint_states) ─────────────────────────
    lcm_bridge = Node(
        package='lcm_bridge',
        executable='bridge_node',
        name='lcm_bridge',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('lcm_bridge'), 'config', 'bridge_params.yaml'
        ])]
    )

    # ── FAST-LIO2 ────────────────────────────────────────────────
    # 依赖: livox_ros_driver2 (提供 /livox/lidar + /livox/imu)
    # map → odom TF 由 FAST-LIO2 提供
    fast_lio2 = Node(
        package='fast_lio_sam',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('dog_brain'), 'config', 'fast_lio_mapping.yaml'
        ])],
        remappings=[
            ('/livox/lidar', '/livox/lidar'),
            ('/livox/imu', '/livox/imu'),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        sensors_launch,
        robot_state_pub,
        lcm_bridge,
        fast_lio2,
    ])
