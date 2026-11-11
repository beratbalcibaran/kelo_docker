import math
import os

import yaml

from kelo_bringup.common import data_dir

STATION_KEYS = ('name', 'x', 'y', 'yaw', 'wait_s', 'stop')


def routes_file():
    return os.path.join(data_dir(), 'routes.yaml')


def station(name, x, y, yaw=None, wait_s=0.0, stop=True):
    return {'name': str(name), 'x': round(float(x), 3), 'y': round(float(y), 3),
            'yaw': None if yaw is None else round(float(yaw), 4),
            'wait_s': max(0.0, float(wait_s)), 'stop': bool(stop)}


def clean_station(raw, index):
    if not isinstance(raw, dict) or 'x' not in raw or 'y' not in raw:
        raise ValueError(f'station {index + 1}: x and y are required')
    yaw = raw.get('yaw')
    return station(raw.get('name') or f'S{index + 1}', raw['x'], raw['y'],
                   None if yaw in (None, '', 'auto') else yaw, raw.get('wait_s', 0.0), raw.get('stop', True))


def clean_route(raw):
    stations = [clean_station(s, i) for i, s in enumerate((raw or {}).get('stations') or [])]
    return {'stations': stations}


def load_routes(path=None):
    path = path or routes_file()
    if not os.path.isfile(path):
        return {}
    with open(path, encoding='utf-8') as f:
        raw = (yaml.safe_load(f) or {}).get('routes') or {}
    return {str(name): clean_route(r) for name, r in raw.items()}


def save_routes(routes, path=None):
    path = path or routes_file()
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'frame': 'map', 'routes': {n: clean_route(r) for n, r in routes.items()}},
                       f, sort_keys=False, allow_unicode=True)
    os.replace(tmp, path)


def heading(ax, ay, bx, by):
    if math.hypot(bx - ax, by - ay) < 1e-6:
        return None
    return math.atan2(by - ay, bx - ax)


def goal_yaw(stations, index, loop, came_from=None):
    """Heading at a station: its own yaw if set, otherwise the direction the robot arrives from
    (no turning on the spot at the station; the turn towards the next station is done when it
    leaves), otherwise facing the next station."""
    s = stations[index]
    if s.get('yaw') is not None:
        return float(s['yaw'])
    if came_from is not None:
        h = heading(came_from[0], came_from[1], s['x'], s['y'])
        if h is not None:
            return h
    nxt = index + 1 if index + 1 < len(stations) else (0 if loop and len(stations) > 1 else None)
    if nxt is not None:
        h = heading(s['x'], s['y'], stations[nxt]['x'], stations[nxt]['y'])
        if h is not None:
            return h
    return 0.0


class Plan:
    """Order of visits: laps = 0 repeats until stopped."""

    def __init__(self, n_stations, laps=1, start=0):
        if n_stations < 1:
            raise ValueError('the route has no stations')
        self.n, self.laps = n_stations, max(0, int(laps))
        self.lap, self.index = 1, min(max(0, int(start)), n_stations - 1)

    @property
    def loop(self):
        return self.laps != 1

    def last_visit(self):
        return self.laps != 0 and self.lap >= self.laps and self.index == self.n - 1

    def advance(self):
        if self.last_visit():
            return False
        self.index += 1
        if self.index >= self.n:
            self.index, self.lap = 0, self.lap + 1
        return True


def stops_here(stations, plan):
    s = stations[plan.index]
    return bool(s.get('stop', True)) or s.get('wait_s', 0.0) > 0.0 or plan.last_visit()


def map_check(grid, info, x, y, clearance):
    """'ok', 'unknown' (outside the map or unmapped) or 'blocked' (an obstacle within clearance)."""
    res = info['resolution']
    ox, oy = info['origin'][0], info['origin'][1]
    h = len(grid)
    w = len(grid[0]) if h else 0
    i, j = int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))
    if not (0 <= i < w and 0 <= j < h) or grid[j][i] < 0:
        return 'unknown'
    r = int(math.ceil(clearance / res))
    for jj in range(max(0, j - r), min(h, j + r + 1)):
        for ii in range(max(0, i - r), min(w, i + r + 1)):
            if grid[jj][ii] >= 65 and math.hypot((ii - i) * res, (jj - j) * res) <= clearance:
                return 'blocked'
    return 'ok'


def load_map_image(map_yaml):
    """Map image of a map_server yaml: (gray image rows top-down, occupancy grid rows bottom-up
    with 100 occupied / 0 free / -1 unknown, info dict)."""
    import cv2
    import numpy as np
    with open(map_yaml, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    image = meta['image'] if os.path.isabs(meta['image']) else os.path.join(os.path.dirname(map_yaml), meta['image'])
    gray = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise OSError(f'map image not readable: {image}')
    p = gray.astype(float) / 255.0
    occ = p if meta.get('negate', 0) else 1.0 - p
    grid = np.full(gray.shape, -1, np.int16)
    grid[occ > float(meta.get('occupied_thresh', 0.65))] = 100
    grid[occ < float(meta.get('free_thresh', 0.25))] = 0
    info = {'resolution': float(meta['resolution']), 'origin': [float(v) for v in meta['origin'][:2]]}
    return gray, grid[::-1].copy(), info


def world_to_pixel(x, y, info, height, scale=1.0):
    res, (ox, oy) = info['resolution'], info['origin']
    return (x - ox) / res * scale, (height - (y - oy) / res) * scale


def pixel_to_world(u, v, info, height, scale=1.0):
    res, (ox, oy) = info['resolution'], info['origin']
    return ox + u / scale * res, oy + (height - v / scale) * res
