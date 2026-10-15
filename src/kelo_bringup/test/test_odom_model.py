import math
import os

import pytest
import yaml

from kelo_bringup.logic import (lin_speed_corrector, odom_lin_speed, odom_yaw_rate, table_value,
                                yaw_rate_corrector)

CFG = os.path.join(os.path.dirname(__file__), '..', 'config')


def load(name):
    with open(os.path.join(CFG, name), encoding='utf-8') as f:
        return yaml.safe_load(f)


HW = load('hardware.yaml')['odom_correction']
SIM = load('sim_world.yaml')['odometry']


@pytest.mark.parametrize('src', [HW, SIM], ids=['hardware', 'sim'])
def test_yaw_k_measured_points(src):
    yk = src['yaw_k']
    for wz, k in ((0.6, 5.3), (1.5, 8.6)):
        odom_deg = 360.0 * odom_yaw_rate(wz, yk) / wz
        assert 360.0 / odom_deg == pytest.approx(k, rel=1e-6)
    assert 360.0 / 5.3 == pytest.approx(67.9, abs=0.1)
    assert table_value(1.05, yk['wz'], yk['k']) == pytest.approx(6.95)
    assert table_value(0.3, yk['wz'], yk['k']) == pytest.approx(5.3)
    assert table_value(2.0, yk['wz'], yk['k']) == pytest.approx(8.6)


@pytest.mark.parametrize('src', [HW, SIM], ids=['hardware', 'sim'])
def test_linear_error_points(src):
    le = src['linear_error']
    for v, e in ((0.06, -0.006), (0.09, -0.010), (0.12, -0.014), (0.15, -0.018)):
        assert odom_lin_speed(v, le) / v - 1.0 == pytest.approx(e, abs=1e-9)


def test_sim_default_is_faulty_model():
    assert SIM['model'] == 'faulty'
    assert load('hardware.yaml')['robot']['odom_mode'] == 'corrected'


def test_correction_inverts_model():
    yaw_corr, lin_corr = yaw_rate_corrector(HW['yaw_k']), lin_speed_corrector(HW['linear_error'])
    for wz in (-1.5, -0.8, -0.1, 0.0, 0.2, 0.6, 0.8, 1.2, 1.5, 2.5):
        assert yaw_corr(odom_yaw_rate(wz, HW['yaw_k'])) == pytest.approx(wz, abs=1e-3)
    for v in (0.0, 0.03, 0.06, 0.1, 0.15, 0.3):
        assert lin_corr(odom_lin_speed(v, HW['linear_error'])) == pytest.approx(v, abs=1e-4)


def test_identity_tables():
    corr = yaw_rate_corrector({'wz': [0.0], 'k': [1.0]})
    assert corr(0.37) == pytest.approx(0.37, abs=1e-3)
    assert lin_speed_corrector({'v': [0.0], 'e': [0.0]})(0.2) == pytest.approx(0.2, abs=1e-4)


def test_one_turn_integration():
    yk = HW['yaw_k']
    dt, wz = 0.02, 0.6
    steps = int(round(2 * math.pi / wz / dt))
    odom = sum(odom_yaw_rate(wz, yk) * dt for _ in range(steps))
    corr = yaw_rate_corrector(yk)
    fixed = sum(corr(odom_yaw_rate(wz, yk)) * dt for _ in range(steps))
    assert math.degrees(odom) == pytest.approx(67.9, abs=0.5)
    assert math.degrees(fixed) == pytest.approx(360.0, abs=0.5)
