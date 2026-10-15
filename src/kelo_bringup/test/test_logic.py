import math

import numpy as np
import pytest

from kelo_bringup.logic import (Debounce, apply_se2, clamp, kabsch_2d, path_length,
                                select_command, wrap, yaw_from_quat, quat_from_yaw)
from kelo_bringup.mm_calibrate import calibrate

LIM = dict(cmd_timeout=0.5, stop_timeout=1.0, max_vx=0.3, max_vy=0.3, max_wz=0.8)


def test_wrap_and_clamp():
    assert wrap(3 * math.pi) == pytest.approx(math.pi, abs=1e-9) or wrap(3 * math.pi) == pytest.approx(-math.pi, abs=1e-9)
    assert wrap(0.5) == pytest.approx(0.5)
    assert clamp(5, 1) == 1 and clamp(-5, 1) == -1 and clamp(0.2, 1) == 0.2


def test_quat_roundtrip():
    for yaw in (-3.0, -1.0, 0.0, 0.7, 3.0):
        q = quat_from_yaw(yaw)
        assert yaw_from_quat(*q) == pytest.approx(yaw, abs=1e-9)


def test_kabsch_recovers_transform():
    rng = np.random.default_rng(1)
    src = rng.uniform(-3, 3, (80, 2))
    yaw, tx, ty = 0.9, 2.5, -1.2
    px, py = apply_se2(yaw, tx, ty, src[:, 0], src[:, 1])
    dst = np.stack([px, py], axis=1)
    y, x, yv, err = kabsch_2d(src, dst)
    assert y == pytest.approx(yaw, abs=1e-9)
    assert (x, yv) == (pytest.approx(tx, abs=1e-9), pytest.approx(ty, abs=1e-9))
    assert err.max() < 1e-9


def test_kabsch_no_reflection():
    src = np.array([[0, 0], [1, 0], [0, 2], [3, 1]], float)
    dst = src * np.array([1, -1])
    yaw, tx, ty, err = kabsch_2d(src, dst)
    assert err.max() > 0.1


def test_calibrate_noisy_with_outlier():
    rng = np.random.default_rng(2)
    t = np.linspace(0, 6.0, 200)
    src = np.stack([2.0 * np.cos(t), 1.5 * np.sin(t) + 0.2 * t], axis=1)
    yaw, tx, ty = -0.4, 1.0, 3.0
    px, py = apply_se2(yaw, tx, ty, src[:, 0], src[:, 1])
    dst = np.stack([px, py], axis=1) + rng.normal(0, 0.03, (200, 2))
    dst[10] += [2.0, -2.0]
    rows = [(*s, *d) for s, d in zip(src, dst)]
    res = calibrate(rows, max_rms=0.15)
    assert res['yaw'] == pytest.approx(yaw, abs=0.02)
    assert res['x'] == pytest.approx(tx, abs=0.05)
    assert res['stats']['ok'] and res['stats']['rms_m'] < 0.15
    assert res['stats']['outliers'] >= 1
    assert res['stats']['max_m'] > 2.0 and res['stats']['rms_all_m'] > res['stats']['rms_m']


def test_calibrate_rejects_when_many_outliers():
    rng = np.random.default_rng(3)
    t = np.linspace(0, 6.0, 100)
    src = np.stack([2.0 * np.cos(t), 1.5 * np.sin(t) + 0.2 * t], axis=1)
    dst = src.copy()
    dst[::4] += rng.uniform(-3, 3, (25, 2))
    res = calibrate([(*s, *d) for s, d in zip(src, dst)], max_rms=0.15)
    assert not res['stats']['ok']


def test_calibrate_rejects_short_path_and_few_samples():
    rows = [(i * 0.001, 0.0, i * 0.001, 0.0) for i in range(50)]
    with pytest.raises(ValueError):
        calibrate(rows, 0.15)
    with pytest.raises(ValueError):
        calibrate(rows[:5], 0.15)


def test_path_length():
    assert path_length([(0, 0), (3, 4)]) == pytest.approx(5.0)
    assert path_length([(1, 1)]) == 0.0


def test_debounce_requires_sustained_condition():
    d = Debounce(on_delay=2.0, off_delay=3.0)
    assert d.update(True, 0.0) is False
    assert d.update(True, 1.0) is False
    assert d.update(False, 1.5) is False
    assert d.update(True, 2.0) is False
    assert d.update(True, 4.1) is True
    assert d.update(False, 5.0) is True
    assert d.update(False, 8.1) is False


def test_select_command_priority_and_limits():
    now = 10.0
    nav = (9.9, 1.0, -1.0, 2.0)
    tel = (9.9, 0.1, 0.0, 0.0)
    assert select_command(now, tel, nav, None, **LIM) == (0.1, 0.0, 0.0, 'TELEOP')
    assert select_command(now, None, nav, None, **LIM) == (0.3, -0.3, 0.8, 'NAV')
    assert select_command(now, tel, nav, (9.9, True), **LIM) == (0.0, 0.0, 0.0, 'SAFETY_STOP')


def test_select_command_watchdog_and_stale_stop():
    now = 10.0
    old = (9.0, 0.2, 0.0, 0.0)
    assert select_command(now, None, old, None, **LIM) == (0.0, 0.0, 0.0, 'IDLE')
    assert select_command(now, None, (9.9, 0.2, 0, 0), (8.0, True), **LIM)[3] == 'NAV'
    assert select_command(now, None, (9.9, 0.2, 0, 0), (9.9, False), **LIM)[3] == 'NAV'
