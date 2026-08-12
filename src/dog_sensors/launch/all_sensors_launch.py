# One-click launch all perception sensors
# Starts: Livox Mid-360 + Intel D435i

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('dog_sensors')

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
    )

    return LaunchDescription([
        mid360_launch,
        d435i_launch,
    ])
