from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from kelo_bringup.stack import build

ARGS = {
    'source': 'sim', 'mode': 'mapping', 'odom_source': 'robot',
    'map': '/data/maps/current.yaml', 'bag': '', 'bag_rate': '1.0', 'bag_delay': '5.0',
    'record': 'auto', 'initial_pose': '', 'use_rviz': 'false', 'wait_tf_s': '600',
    'sim_x0': '', 'sim_y0': '', 'sim_yaw0': '', 'sim_seed': '0', 'sim_odom_model': '',
    'sim_world': '', 'rebuild_localizer': 'auto', 'views': 'true', 'web': 'true',
}


def _setup(context):
    opts = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    return build(opts)


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
                             + [OpaqueFunction(function=_setup)])
