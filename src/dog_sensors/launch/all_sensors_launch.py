# One-click launch all perception sensors
# Starts: Livox Mid-360 + Intel D435i

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('dog_sensors')

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='是否启动 D435i 深度相机（建图省电实验可关闭）')

    # Livox Mid-360
    mid360_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mid360_launch.py')
        ),
    )

    # D435i depth camera
    d435i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'd435i_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    )

    return LaunchDescription([
        enable_camera_arg,
        mid360_launch,
        d435i_launch,
    ])
