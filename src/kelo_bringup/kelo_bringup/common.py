import copy
import os

import yaml
from ament_index_python.packages import get_package_share_directory

PKG = 'kelo_bringup'


def share():
    return get_package_share_directory(PKG)


def config_path(name):
    return os.path.join(share(), 'config', name)


def data_dir():
    path = os.environ.get('KELO_DATA_DIR', '/data')
    os.makedirs(path, exist_ok=True)
    return path


def maps_dir():
    path = os.path.join(data_dir(), 'maps')
    os.makedirs(path, exist_ok=True)
    return path


def alignment_file():
    return os.path.join(data_dir(), 'marvelmind_alignment.yaml')


def waypoints_file():
    return os.path.join(data_dir(), 'waypoints.yaml')


def _merge(base, over):
    out = copy.deepcopy(base)
    for key, val in (over or {}).items():
        if isinstance(val, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], val)
        else:
            out[key] = val
    return out


def load_hardware():
    with open(config_path('hardware.yaml'), encoding='utf-8') as f:
        hw = yaml.safe_load(f)
    override = os.path.join(data_dir(), 'hardware.yaml')
    if os.path.isfile(override):
        with open(override, encoding='utf-8') as f:
            hw = _merge(hw, yaml.safe_load(f))
    return hw


def load_alignment():
    path = alignment_file()
    if not os.path.isfile(path):
        return None
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not all(k in data for k in ('x', 'y', 'yaw')):
        return None
    return data


SOURCES = ('sim', 'hardware', 'replay')
ODOM_SOURCES = ('robot', 'lidar')
SENSORS = ('lidar', 'camera', 'marvelmind')


def sensors(hw):
    out = {k: bool((hw.get('sensors') or {}).get(k, True)) for k in SENSORS}
    if not out['lidar']:
        raise ValueError('sensors.lidar: false is not supported (SLAM, AMCL and collision_monitor need the LiDAR)')
    return out


def odom_plan(hw, odom_source):
    if odom_source not in ODOM_SOURCES:
        raise ValueError(f'odom_source must be one of {ODOM_SOURCES}: {odom_source}')
    raw = hw['topics']['odom']
    if odom_source == 'lidar':
        return {'tf_owner': 'rf2o', 'topic': '/odom_rf2o', 'raw_topic': raw}
    if hw['robot']['odom_mode'] == 'corrected':
        return {'tf_owner': 'odom_filter', 'topic': '/odom_corrected', 'raw_topic': raw}
    return {'tf_owner': 'driver', 'topic': raw, 'raw_topic': raw}


def _load_cfg(name):
    with open(config_path(name), encoding='utf-8') as f:
        return yaml.safe_load(f)


def _drop_cm_source(cm, name):
    cm['observation_sources'] = [s for s in cm['observation_sources'] if s != name]
    cm.pop(name, None)


def nav2_params(hw, odom_topic, cfg=None):
    cfg = copy.deepcopy(cfg) if cfg is not None else _load_cfg('nav2_params.yaml')
    sens = sensors(hw)
    for name in ('bt_navigator', 'velocity_smoother', 'controller_server'):
        cfg[name]['ros__parameters']['odom_topic'] = odom_topic
    lc = cfg['local_costmap']['local_costmap']['ros__parameters']
    if sens['camera']:
        lc['camera_layer']['camera']['topic'] = hw['topics']['camera_points']
        lc['camera_layer']['camera_clearing']['topic'] = hw['topics']['camera_clearing']
    else:
        lc['plugins'] = [p for p in lc['plugins'] if p != 'camera_layer']
        lc.pop('camera_layer', None)
    cm = cfg['collision_monitor']['ros__parameters']
    if 'camera' in cm.get('observation_sources', []):
        if sens['camera']:
            cm['camera']['topic'] = hw['topics']['camera_points']
        else:
            _drop_cm_source(cm, 'camera')
    return cfg


def safety_params(hw, cfg=None):
    cfg = copy.deepcopy(cfg) if cfg is not None else _load_cfg('safety.yaml')
    cm = cfg['collision_monitor_teleop']['ros__parameters']
    if 'camera' in cm.get('observation_sources', []):
        if sensors(hw)['camera']:
            cm['camera']['topic'] = hw['topics']['camera_points']
        else:
            _drop_cm_source(cm, 'camera')
    return cfg


def write_params(cfg, prefix):
    import tempfile
    fd, path = tempfile.mkstemp(prefix=prefix, suffix='.yaml')
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        yaml.safe_dump(cfg, f)
    return path
