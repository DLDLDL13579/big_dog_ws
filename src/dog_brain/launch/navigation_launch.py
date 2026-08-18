# 机械狗导航模式 (Navigation Mode)
# 启动: FAST-LIO-Localization (ICP 全局定位) + Nav2 (3D Voxel Layer) + 传感器 + 桥接
#
# 前置条件: 已跑 mapping_launch.py 建图并保存 3D PCD 地图
#
# 定位链路 (接法 A: ICP 3D 全局定位, 彻底替换 AMCL):
#   global_map_publisher.py  → 加载 PCD 地图 → /global_map
#   fastlio_mapping          → FAST-LIO2 激光惯性里程计 → /Odometry + odom→base_link TF
#   global_localization.py   → ICP 匹配 → /map_to_odom
#   transform_fusion.py      → 融合 → map→odom TF (绝对定位权)
#
# 输入:
#   maps/lab_3d_map.pcd (全局地图)
#   /livox/lidar (实时点云)
#   /odom (lcm_bridge, UpBoard 运动学闭环)
# 输出:
#   /cmd_vel → TCP → UpBoard
#   TF: map → odom (FAST-LIO-Loc) → base_link (FAST-LIO2)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
            'robot_description': ParameterValue(Command(['cat ', PathJoinSubstitution([FindPackageShare('dog_description'), 'urdf', 'dog.urdf'])]), value_type=str)
        }]
    )

    # ── LCM Bridge ───────────────────────────────────────────────
    # publish_odom_tf=False: TF 由 FAST-LIO2 提供
    lcm_bridge = Node(
        package='lcm_bridge',
        executable='bridge_node',
        name='lcm_bridge',
        output='screen',
        parameters=[{'publish_odom_tf': False}]
    )

    # ── FAST-LIO-Localization (ICP 全局重定位) ──────────────────
    # 定位四件套, 发布 map→odom TF (绝对定位权)
    # 1. FAST-LIO2 里程计 (发布 /Odometry + odom→base_link TF)
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

    # 2. 全局地图发布 (PCD → /global_map)
    global_map_pub = Node(
        package='fast_lio_localization',
        executable='global_map_publisher.py',
        name='global_map_publisher',
        output='screen',
        parameters=[{'map_file_path': LaunchConfiguration('map_pcd')}]
    )

    # 2b. PCD 3D 地图 → 2D OccupancyGrid (/map, 供 Nav2 static_layer)
    pcd_to_map = Node(
        package='fast_lio_localization',
        executable='pcd_to_map_node.py',
        name='pcd_to_map',
        output='screen',
        parameters=[{'pcd_path': LaunchConfiguration('map_pcd')}]
    )

    # 2c. PCD -> 2.5D 高程图 (/elevation_costmap, 越障/模态切换依据)
    elevation_map = Node(
        package='fast_lio_localization',
        executable='elevation_map_node.py',
        name='elevation_map',
        output='screen',
        parameters=[{'pcd_path': LaunchConfiguration('map_pcd')}]
    )

    # 2d. 实时前方台阶检测 (D435i 近距, 供轮腿模态切换) [实验性,待现场标定]
    step_detector = Node(
        package='fast_lio_localization',
        executable='step_detector_node.py',
        name='step_detector',
        output='screen',
    )

    # 3. 全局定位 (ICP 匹配 → /map_to_odom)
    global_loc = Node(
        package='fast_lio_localization',
        executable='global_localization.py',
        name='global_localization',
        output='screen',
        parameters=[{
            'map_voxel_size': 0.1,
            'scan_voxel_size': 0.1,
            'freq_localization': 0.5,
            'localization_th': 0.9,
        }]
    )

    # 4. TF 融合 (map→odom TF)
    transform_fusion = Node(
        package='fast_lio_localization',
        executable='transform_fusion.py',
        name='transform_fusion',
        output='screen',
    )

    # ── Nav2 3D Navigation (纯避障+路径规划, 不含 AMCL) ─────────
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
        fast_lio2,
        global_map_pub,
        pcd_to_map,
        elevation_map,
        step_detector,
        global_loc,
        transform_fusion,
        nav2_bringup,
    ])
