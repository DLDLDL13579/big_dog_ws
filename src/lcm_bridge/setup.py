from setuptools import setup
from glob import glob
import os

package_name = 'lcm_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='LCM↔ROS2 bridge for robot dog UpBoard',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = lcm_bridge.bridge_node:main',
            'robot2_soldier_node = lcm_bridge.robot2_soldier_node:main',
        ],
    },
)
