import numpy as np
import pytest

from kelo_bringup.camera_grid import FREE, OCCUPIED, UNKNOWN, CameraGrid, fit_to, write_map
from kelo_bringup.logic import clearing_rays
from kelo_bringup.map_compare import load_map

CAM = np.array([0.0, 0.0, 0.68])


def grid():
    g = CameraGrid()
    g.align(0.05, -1.0, -2.0, 120, 80)
    return g


def cell(g, x, y):
    occ = g.occupancy()
    return occ[int((y - g.origin[1]) / g.res), int((x - g.origin[0]) / g.res)]


YS = (-0.18, 0.02, 0.22)


def floor_rays(x0, x1, ys=YS):
    return np.array([[x, y, 0.0] for x in np.arange(x0, x1, 0.05) + 0.02 for y in ys])


def table_top(x0, x1, z=0.62):
    return np.array([[x, y, z] for x in np.arange(x0, x1, 0.05) + 0.02 for y in YS])


def test_table_top_stays_occupied_while_floor_under_it_is_seen_free():
    g = grid()
    for _ in range(10):
        top = table_top(1.5, 2.2)
        g.integrate(CAM, top, np.vstack([floor_rays(0.6, 3.0), top]))
    assert cell(g, 1.8, 0.0) == OCCUPIED
    assert cell(g, 2.1, 0.0) == OCCUPIED
    assert cell(g, 1.0, 0.0) == FREE
    assert cell(g, 2.6, 0.0) == FREE
    assert cell(g, 3.5, 0.0) == UNKNOWN


def test_person_walking_by_is_erased_when_seen_free_again():
    g = grid()
    person = np.array([[1.52, 0.02, z] for z in np.arange(0.1, 1.2, 0.05)])
    for _ in range(3):
        g.integrate(CAM, person, person)
    assert cell(g, 1.5, 0.0) == OCCUPIED
    # the camera sees up to ~19 deg above horizontal: wall points up to 1.7 m clear the upper voxels
    wall = np.array([[3.02, 0.02, z] for z in np.arange(0.1, 1.75, 0.05)])
    for _ in range(15):
        g.integrate(CAM, wall, np.vstack([floor_rays(1.6, 2.9, ys=(0.02,)), wall]))
    assert cell(g, 1.5, 0.0) == FREE
    assert cell(g, 3.0, 0.0) == OCCUPIED


def test_single_noise_hit_is_not_an_obstacle():
    g = grid()
    g.integrate(CAM, np.array([[1.0, 0.5, 0.3]]), np.zeros((0, 3)))
    assert cell(g, 1.0, 0.5) == UNKNOWN


def test_points_outside_height_band_are_ignored():
    g = grid()
    for _ in range(5):
        g.integrate(CAM, np.array([[1.0, 0.0, 0.02], [1.2, 0.0, 1.5]]), np.zeros((0, 3)))
    assert (g.occupancy() == UNKNOWN).all()


def test_align_follows_growing_map_and_keeps_cells():
    g = grid()
    for _ in range(5):
        g.integrate(CAM, table_top(1.5, 1.52), np.zeros((0, 3)))
    g.align(0.05, -3.0, -2.5, 200, 120)
    assert g.width == 200 and g.origin == (-3.0, -2.5)
    assert cell(g, 1.52, 0.0) == OCCUPIED
    assert (g.occupancy() == OCCUPIED).sum() == 3


def test_fit_to_and_write_map_match_lidar_geometry(tmp_path):
    g = grid()
    for _ in range(5):
        g.integrate(CAM, table_top(1.5, 1.52), np.zeros((0, 3)))
    occ = fit_to(g.occupancy(), g.origin, g.res, (-0.5, -1.0), 70, 50)
    assert occ.shape == (50, 70)
    stem = str(tmp_path / 'room_camera')
    write_map(stem, occ, 0.05, (-0.5, -1.0))
    pts, res = load_map(stem + '.yaml')
    assert res == pytest.approx(0.05)
    assert len(pts) == 3
    assert np.allclose(sorted(pts[:, 1]), [-0.175, 0.025, 0.225], atol=1e-6)
    assert np.allclose(pts[:, 0], 1.525, atol=1e-6)


def test_clearing_rays_are_pulled_back_to_max_range():
    k = (100.0, 100.0, 2.0, 2.0)
    depth = np.zeros((5, 5), dtype=np.float32)
    depth[2, 2], depth[2, 3], depth[2, 4] = 1.0, 8.0, 0.0
    rays = clearing_rays(depth, k, 1, 0.3, 4.0, np.eye(3), np.zeros(3))
    assert len(rays) == 2
    assert np.linalg.norm(rays, axis=1).max() == pytest.approx(4.0)
    assert np.linalg.norm(rays, axis=1).min() == pytest.approx(1.0)


def test_low_box_survives_rays_grazing_its_top():
    # 12 cm box at 1.5-1.9 m: each frame only a few top pixels hit it, while every ray to the floor
    # behind it (visible from 2.35 m on) passes 1-10 cm above its top and crosses its voxels
    g = grid()
    rng = np.random.default_rng(0)
    top = np.array([[x, y, 0.12] for x in np.arange(1.5, 1.9, 0.05) + 0.02 for y in YS])
    for _ in range(25):
        hits = top[rng.random(len(top)) < 0.25]
        g.integrate(CAM, hits, np.vstack([floor_rays(0.6, 1.45), floor_rays(2.35, 3.2), hits]))
    occupied = [cell(g, x, y) == OCCUPIED for x in np.arange(1.5, 1.9, 0.05) + 0.02 for y in YS]
    assert all(occupied)
    assert cell(g, 1.0, 0.0) == FREE
    assert cell(g, 2.8, 0.0) == FREE


def test_false_obstacles_counts_only_cells_away_from_objects(tmp_path):
    from kelo_bringup.layer_eval import false_obstacles
    world = tmp_path / 'w.yaml'
    world.write_text('room: {width: 4.0, height: 3.0, wall_height: 2.0}\nstart: {x: 1.0, y: 1.0, yaw: 0.0}\n'
                     'obstacles:\n  - {type: box, name: b, xy: [2.0, 1.0, 2.4, 1.4], z: [0.0, 0.12]}\n')
    g = np.full((60, 80), FREE, dtype=np.int8)       # map origin (-1, -1) = world (0, 0)
    cellat = lambda wx, wy: (int((wy - 1.0 + 1.0) / 0.05), int((wx - 1.0 + 1.0) / 0.05))
    for wx, wy in ((2.2, 1.2), (2.05, 1.05), (2.45, 1.2), (1.0, 2.0)):  # inside, inside, near, phantom
        g[cellat(wx, wy)] = OCCUPIED
    write_map(str(tmp_path / 'm'), g, 0.05, (-1.0, -1.0))
    assert false_obstacles(str(world), str(tmp_path / 'm.yaml')) == {'occupied': 4, 'false': 1}


def test_figure_colours_lidar_and_camera_cells(tmp_path):
    from PIL import Image
    from kelo_bringup.layer_eval import CAMERA_C, OCC_C, figure
    lidar = np.full((40, 40), FREE, dtype=np.int8)
    lidar[10, 10] = OCCUPIED
    cam = np.full((40, 40), UNKNOWN, dtype=np.int8)
    cam[30, 30] = OCCUPIED
    write_map(str(tmp_path / 'm'), lidar, 0.05, (0.0, 0.0))
    write_map(str(tmp_path / 'm_camera'), cam, 0.05, (0.0, 0.0))
    ref = np.full((40, 40), FREE, dtype=np.int8)
    ref[[0, 39], [0, 39]] = OCCUPIED                  # reference extent = the whole 2 x 2 m map
    write_map(str(tmp_path / 'ref'), ref, 0.05, (0.0, 0.0))
    out = str(tmp_path / 'f.png')
    figure(out, str(tmp_path / 'ref.yaml'), [('two layers', str(tmp_path / 'm.yaml'), True)], scale=1)
    colours = {c for _, c in Image.open(out).convert('RGB').getcolors(1 << 16)}
    assert OCC_C in colours and CAMERA_C in colours


def test_saved_unknown_stays_unknown_for_map_server(tmp_path):
    # map_server (trinary): occ = (255 - pixel) / 255; free below free_thresh, unknown between the
    # thresholds. The unknown pixel 205 must not fall under free_thresh (it did with 0.25).
    import yaml
    g = np.full((4, 4), UNKNOWN, np.int8)
    write_map(str(tmp_path / 'u'), g, 0.05, (0.0, 0.0))
    cfg = yaml.safe_load(open(tmp_path / 'u.yaml'))
    occ = (255 - 205) / 255.0
    assert cfg['free_thresh'] < occ < cfg['occupied_thresh']
    assert (255 - 254) / 255.0 < cfg['free_thresh']
