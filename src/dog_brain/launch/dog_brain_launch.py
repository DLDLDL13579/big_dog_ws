# 机械狗大脑启动文件
# 
# 架构:
# ┌──────────────────────────────────────┐
# │ Jetson Orin NX (大脑)                │
# │                                       │
# │ dog_description                       │
# │   └ robot_state_publisher (URDF→TF)  │
# │                                       │
# │ lcm_bridge                            │
# │   └ LCM → /odom, /imu, /joint_states│
# │   └ /cmd_vel → TCP :3333             │
# │                                       │
# │ slam_toolbox (建图)                   │
# │   └ /scan → /map (map→odom TF)      │
# │                                       │
# │ Nav2 (导航)                           │
# │   └ planner + controller + BT       │
# └──────────────┬───────────────────────┘
#                │ LCM UDP 组播 + TCP
# ┌──────────────┴───────────────────────┐
# │ UpBoard (小脑)                       │
# │   └ StateEstimator + SPI + IMU      │
# └──────────────────────────────────────┘

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 参数声明
    map_file_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='已有地图文件路径 (YAML)')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='使用仿真时间')

    # ── Dog Description ────────────────────────────────────────
    robot_description = Node(
        package='dog_description',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': PathJoinSubstitution([
                FindPackageShare('dog_description'), 'urdf', 'dog.urdf.xacro'
            ])
        }]
    )

    # ── LCM Bridge ──────────────────────────────────────────────
    lcm_bridge = Node(
        package='lcm_bridge',
        executable='bridge_node',
        name='lcm_bridge',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('lcm_bridge'), 'config', 'bridge_params.yaml'
        ])]
    )

    # ── SLAM Toolbox (Online Async) ─────────────────────────────
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('dog_brain'), 'config', 'slam_params.yaml'
        ])],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/map_metadata', '/map_metadata'),
        ]
    )

    # ── Nav2 ────────────────────────────────────────────────────
    nav2_params = PathJoinSubstitution([
        FindPackageShare('dog_brain'), 'config', 'nav2_params.yaml'
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
            # 使用 UpBoard 提供的 odom，不需要 AMCL 的额外 TF
            'map': LaunchConfiguration('map'),
        }.items()
    )

    # ── 完整 Launch ─────────────────────────────────────────────
    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        robot_description,
        lcm_bridge,
        slam_toolbox,
        nav2_bringup,
    ])
