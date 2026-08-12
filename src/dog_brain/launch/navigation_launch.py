# 机械狗导航模式 (Navigation Mode)
# 启动: FAST-LIO-Localization (重定位) + Nav2 (3D Voxel Layer) + 传感器 + 桥接
#
# 前置条件: 已跑 mapping_launch.py 并保存 3D PCD 地图
# 输入:
#   maps/lab_3d_map.pcd (全局地图)
#   /livox/lidar (实时点云)
#   /odom (lcm_bridge)
# 输出:
#   /cmd_vel → TCP → UpBoard
#   TF: map → odom (FAST-LIO-Loc) → base_link (lcm_bridge)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    map_pcd_arg = DeclareLaunchArgument(
        'map_pcd',
        default_value=PathJoinSubstitution([
            FindPackageShare('dog_brain'), 'maps', 'lab_3d_map.pcd'
        ]),
        description='3D PCD map file for localization')

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

    # ── LCM Bridge ───────────────────────────────────────────────
    lcm_bridge = Node(
        package='lcm_bridge',
        executable='bridge_node',
        name='lcm_bridge',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('lcm_bridge'), 'config', 'bridge_params.yaml'
        ])]
    )

    # ── FAST-LIO-Localization (全局重定位) ───────────────────────
    # 加载 PCD 地图，发布 map → odom TF
    fast_lio_loc = Node(
        package='fast_lio_sam',
        executable='fastlio_localization',
        name='fastlio_localization',
        output='screen',
        parameters=[{
            'pcd_file_path': LaunchConfiguration('map_pcd'),
            'use_imu': True,
            'imu_topic': '/livox/imu',
            'lidar_topic': '/livox/lidar',
        }],
    )

    # ── Nav2 3D Navigation ───────────────────────────────────────
    nav2_params = PathJoinSubstitution([
        FindPackageShare('dog_brain'), 'config', 'nav2_3d_params.yaml'
    ])

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_pcd_arg,
        sensors_launch,
        robot_state_pub,
        lcm_bridge,
        fast_lio_loc,
        nav2_bringup,
    ])
