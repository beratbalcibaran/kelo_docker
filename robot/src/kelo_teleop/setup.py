from setuptools import setup

package_name = 'kelo_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KELO Team',
    maintainer_email='kelo@example.com',
    description='Keyboard and joystick teleop for the KELO ROBILE (publishes /cmd_vel)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_keyboard = kelo_teleop.teleop_keyboard:main',
            'teleop_joystick = kelo_teleop.teleop_joystick:main',
        ],
    },
)
