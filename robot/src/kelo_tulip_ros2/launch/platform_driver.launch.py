import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('kelo_tulip_ros2'), 'config', 'platform_driver.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('fix_odom_rotation', default_value='true',
                              description='false: original odometry (yaw rate read 5 times too small)'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='false: another node publishes odom -> base_link'),
        DeclareLaunchArgument('cmd_vel_timeout_s', default_value='0.5',
                              description='zero velocity after this silence on /cmd_vel; 0 turns it off'),
        Node(
            package='kelo_tulip_ros2',
            executable='platform_driver_node',
            name='platform_driver',
            output='screen',
            parameters=[config, {
                'fix_odom_rotation': ParameterValue(LaunchConfiguration('fix_odom_rotation'), value_type=bool),
                'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                'cmd_vel_timeout_s': ParameterValue(LaunchConfiguration('cmd_vel_timeout_s'), value_type=float),
            }],
        ),
    ])
