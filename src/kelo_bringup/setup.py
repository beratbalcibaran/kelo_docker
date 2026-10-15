import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'kelo_bringup'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*.rules')),
    ],
    install_requires=['setuptools', 'pyyaml', 'numpy'],
    zip_safe=True,
    maintainer='KELO Team',
    maintainer_email='kelo@example.com',
    description='KELO bringup: hardware, TF, Nav2 (MPPI-Omni), AMCL, Marvelmind, safety layer.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_gate = kelo_bringup.cmd_vel_gate:main',
            'cloud_filter = kelo_bringup.cloud_filter:main',
            'kelo_e2e = kelo_bringup.e2e:main',
            'map_compare = kelo_bringup.map_compare:main',
            'localize = kelo_bringup.localize:main',
            'marvelmind_bridge = kelo_bringup.marvelmind_bridge:main',
            'mm_recorder = kelo_bringup.mm_recorder:main',
            'mm_calibrate = kelo_bringup.mm_calibrate:main',
            'mm_monitor = kelo_bringup.mm_monitor:main',
            'odom_filter = kelo_bringup.odom_filter:main',
            'odom_test = kelo_bringup.odom_test:main',
            'status_node = kelo_bringup.status_node:main',
            'waypoint_tool = kelo_bringup.waypoint_tool:main',
            'save_map = kelo_bringup.save_map:main',
            'sim_robot = kelo_bringup.sim_robot:main',
            'send_goal = kelo_bringup.send_goal:main',
            'wait_for_tf = kelo_bringup.wait_for_tf:main',
        ],
    },
)
