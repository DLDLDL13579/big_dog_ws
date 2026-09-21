# 机械狗大脑总启动入口 (Bringup)
#
# 用法:
#   mode:=mapping    — SLAM 建图模式 (FAST-LIO2)
#   mode:=navigation — 自主导航模式 (FAST-LIO-Loc + Nav2)
#
# 默认: mapping

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='运行模式: mapping | navigation')
    # [2026-09-21] 初始位姿参数（默认=机械狗发车点 H，可命令行覆盖）
    init_args = [
        DeclareLaunchArgument('auto_initial_pose', default_value='true'),
        DeclareLaunchArgument('init_x', default_value='0.000'),
        DeclareLaunchArgument('init_y', default_value='-0.111'),
        DeclareLaunchArgument('init_z', default_value='0.0'),
        DeclareLaunchArgument('init_yaw', default_value='-0.046'),
    ]

    # ★ 2026-09-01: 默认 false (camera_scan/step_detector 均已禁用, 实测无消费方;
    #   dog-brain.service 映射模式早已显式 enable_camera:=false)。需要相机时显式传 true。
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='是否启动 D435i 相机（默认关闭，透传给 mapping/navigation，可显式设 true 开启）')

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('dog_brain'), 'launch', 'mapping_launch.py'
            ])
        ),
        launch_arguments={'enable_camera': LaunchConfiguration('enable_camera')}.items(),
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration('mode'), '" == "mapping"'])
        ),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('dog_brain'), 'launch', 'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'enable_camera': LaunchConfiguration('enable_camera'),
            # [2026-09-21] 转发初始位姿参数，使 nav_restart.sh 可覆盖
            'auto_initial_pose': LaunchConfiguration('auto_initial_pose'),
            'init_x': LaunchConfiguration('init_x'),
            'init_y': LaunchConfiguration('init_y'),
            'init_z': LaunchConfiguration('init_z'),
            'init_yaw': LaunchConfiguration('init_yaw'),
        }.items(),
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration('mode'), '" == "navigation"'])
        ),
    )

    return LaunchDescription([
        *init_args,
        mode_arg,
        enable_camera_arg,
        mapping_launch,
        navigation_launch,
    ])
