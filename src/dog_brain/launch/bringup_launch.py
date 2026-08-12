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

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('dog_brain'), 'launch', 'mapping_launch.py'
            ])
        ),
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
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration('mode'), '" == "navigation"'])
        ),
    )

    return LaunchDescription([
        mode_arg,
        mapping_launch,
        navigation_launch,
    ])
