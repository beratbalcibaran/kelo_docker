import math

import numpy as np

from kelo_bringup import scene

LIDAR = (0.45, 0.0, 0.0, -1.6, 0.01)


def beams(dist):
    n = int(round(3.2 / LIDAR[4])) + 1
    return LIDAR + (np.full(n, dist),)


def test_obstacle_behind_the_robot_is_remembered():
    m = scene.ObstacleMemory(max_age=30.0)
    m.add([(1.5, 0.0, 0.2)], 0.0, False)
    m.forget(5.0, (3.0, 0.0, 0.0), beams(5.0))
    assert len(m) == 1
    assert np.allclose(m.centres()[0], (1.525, 0.025))


def test_lidar_seeing_through_a_cell_forgets_it():
    m = scene.ObstacleMemory()
    m.add([(1.5, 0.0, 0.2)], 0.0, False)
    m.forget(1.0, (0.0, 0.0, 0.0), beams(4.0))
    assert len(m) == 0


def test_lidar_does_not_forget_a_low_camera_obstacle():
    m = scene.ObstacleMemory()
    m.add([(1.5, 0.0, 0.1)], 0.0, True)
    m.forget(1.0, (0.0, 0.0, 0.0), beams(4.0))
    assert len(m) == 1
    m.forget(1.0, (0.0, 0.0, 0.0), None, (0.42, 0.0, 0.0, math.radians(40), 0.5, 2.0, np.zeros((0, 3))))
    assert len(m) == 0


def test_camera_still_seeing_it_keeps_it():
    m = scene.ObstacleMemory()
    m.add([(1.5, 0.0, 0.1)], 0.0, True)
    m.forget(1.0, (0.0, 0.0, 0.0), None, (0.42, 0.0, 0.0, math.radians(40), 0.5, 2.0, np.array([[1.51, 0.01, 0.1]])))
    assert len(m) == 1


def test_old_and_far_points_are_dropped_and_duplicates_merged():
    m = scene.ObstacleMemory(max_age=10.0, radius=5.0)
    m.add([(1.0, 1.0, 0.2), (1.01, 1.01, 0.2)], 0.0, False)
    assert len(m) == 1
    m.add([(1.0, 1.0, 0.2)], 4.0, False)
    assert len(m) == 1 and m.t[0] == 4.0
    m.add([(-4.0, 0.0, 0.2)], 4.0, False)
    m.forget(12.0, (0.0, 0.0, 0.0))
    assert len(m) == 2
    m.forget(12.0, (3.0, 0.0, 0.0))
    assert len(m) == 1 and np.allclose(m.centres()[0], (1.025, 1.025))
    m.forget(14.5, (3.0, 0.0, 0.0))
    assert len(m) == 0
