"""The pedestrian body follows exactly the script the animated actor uses."""
import math

from cogndn_sim import street
from cogndn_core.paths import sim_config


def test_waypoints_loop_closed_and_timed():
    p = {'id': 'p', 'path': [[0, 0], [12, 0]], 'speed': 1.2, 'pause_s': 2.0}
    wp = street.waypoints(p)
    assert wp[0][1:3] == wp[-1][1:3]
    assert math.isclose(wp[1][0], 10.0)
    assert math.isclose(wp[-1][0], 2 * 10.0 + 2 * (street.TURN_S + 2.0))


def test_state_at_position_velocity_and_wrap():
    wp = street.waypoints({'id': 'p', 'path': [[0, 0], [10, 0]], 'speed': 2.0})
    (x, y), (vx, vy) = street.state_at(wp, 2.5)
    assert math.isclose(x, 5.0) and math.isclose(vx, 2.0) and y == 0 and vy == 0
    (x2, _), (vx2, _) = street.state_at(wp, 2.5 + wp[-1][0])
    assert math.isclose(x2, 5.0) and math.isclose(vx2, 2.0)
    (_, _), (vxb, _) = street.state_at(wp, 7.0)
    assert vxb < 0


def test_config_exclusions():
    st = street.load(sim_config())
    assert not street.active(st, 's5_decoy_sign', 'cows')
    assert street.active(st, 's2_workzone', 'pedestrians')
    assert any(p['id'] == 'ped_crossing' for p in st['pedestrians'])
