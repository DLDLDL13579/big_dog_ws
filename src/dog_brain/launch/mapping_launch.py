# 机械狗建图模式 (Mapping Mode)
# 启动: FAST-LIO2 (LiDAR+IMU 紧耦合) + 传感器 + 状态广播 + 桥接
#
# 输出:
#   /livox/lidar    (CustomMsg, 10Hz) → FAST-LIO2 消费
#   /livox/imu      (Imu, 200Hz, BMI088)
#   /odom            (Odometry, lcm_bridge, UpBoard 运动学闭环)
#   /cloud_registered (FAST-LIO2 全局去畸变点云)
#   /Odometry        (FAST-LIO2 激光惯性里程计)
#   /joint_states    (sensor_msgs)
#   TF: odom → base_link (FAST-LIO2) → {livox_frame, camera_link, legs}
#
# 建图完成后保存 3D PCD:
#   ros2 service call /map_save std_srvs/srv/Trigger

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera', default_value='true',
        description='是否启动 D435i 相机（建图不需要，可关闭降功耗）')

    # ── Sensors ─────────────────────────────────────────────────
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('dog_sensors'), 'launch', 'all_sensors_launch.py'
            ])
        ),
        launch_arguments={'enable_camera': LaunchConfiguration('enable_camera')}.items(),
    )

    # ── Robot Description ───────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['cat ', PathJoinSubstitution([FindPackageShare('dog_description'), 'urdf', 'dog.urdf'])]), value_type=str)
        }]
    )

    # ── LCM Bridge (odom + joint_states) ─────────────────────────
    # ⚠️ publish_odom_tf=False: odom→base_link TF 由 FAST-LIO2 独占提供
    lcm_bridge = Node(
        package='lcm_bridge',
        executable='bridge_node',
        name='lcm_bridge',
        output='screen',
        parameters=[{'publish_odom_tf': False}]
    )

    # ── FAST-LIO2 (建图) ─────────────────────────────────────────
    # package=fast_lio_localization (myeongw002/FAST_LIO_LOCALIZATION_ROS2)
    # 依赖: livox_ros_driver2 (提供 /livox/lidar CustomMsg + /livox/imu)
    # 输出 odom→base_link TF (激光惯性里程计)
    fast_lio2 = Node(
        package='fast_lio_localization',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('fast_lio_localization'), 'config', 'mid360.yaml'
        ])],
        remappings=[
            ('/livox/lidar', '/livox/lidar'),
            ('/livox/imu', '/livox/imu'),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        enable_camera_arg,
        sensors_launch,
        robot_state_pub,
        lcm_bridge,
        fast_lio2,
    ])
