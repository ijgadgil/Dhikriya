import math

import numpy as np
import pytest

from cogndn_core.envelope import EnvelopeParams, TTCEnvelope, nearest_in_corridor
from cogndn_core.navigation import BehaviourPlanner, pure_pursuit


def wall(x, n=20, z=-1.0):
    return np.array([[x, -0.5 + i * 0.05, z] for i in range(n)])


def test_corridor_filter():
    p = EnvelopeParams()
    ground = np.array([[10.0, 0.0, -1.79]] * 10)
    side = np.array([[10.0, 3.0, -1.0]] * 10)
    body = np.array([[1.0, 0.0, -1.0]] * 10)
    assert nearest_in_corridor(np.vstack([ground, side, body]), p) is None
    assert nearest_in_corridor(np.vstack([ground, wall(12.0)]), p) == pytest.approx(12.0)


def test_stop_slow_and_latch():
    env = TTCEnvelope()
    env.on_cloud(0.0, wall(20.0))
    v, _, ev = env.apply(0.0, 3.0, 0.1, 3.0)
    assert v == 3.0 and ev is None                        # TTC 6.7 s: no action
    env.on_cloud(0.1, wall(8.0))
    v, _, ev = env.apply(0.1, 3.0, 0.1, 3.0)
    assert 0 < v < 3.0 and ev['kind'] == 'slow'
    env.on_cloud(0.2, wall(3.9))
    assert env.apply(0.2, 3.0, 0.1, 1.0)[2]['kind'] == 'stop'
    env.on_cloud(0.3, wall(25.0, n=0))                    # obstacle gone
    assert env.apply(0.3, 3.0, 0.0, 0.0)[2]['kind'] == 'clear_hold'
    env.on_cloud(1.5, wall(25.0, n=0))
    assert env.apply(1.5, 3.0, 0.0, 0.0) == (3.0, 0.0, None)


def test_stale_lidar_stops():
    env = TTCEnvelope()
    assert env.apply(0.0, 3.0, 0.0, 3.0)[2]['kind'] == 'stale'
    env.on_cloud(0.0, wall(25.0, n=0))
    assert env.apply(1.0, 3.0, 0.0, 3.0)[2]['kind'] == 'stale'


def test_envelope_never_raises_speed():
    env = TTCEnvelope()
    rng = np.random.default_rng(0)
    for k in range(200):
        env.on_cloud(k * 0.05, wall(float(rng.uniform(2, 30)), n=int(rng.integers(0, 20))))
        vn = float(rng.uniform(0, 5))
        assert env.apply(k * 0.05, vn, 0.0, vn)[0] <= vn + 1e-9


def test_planner_turn_replans(world):
    pl = BehaviourPlanner(world, ['m0', 'm1', 'm2', 'm3', 'm4', 'x1', 'e0', 'e1', 'e2', 'e3', 'e4'])
    pl.on_action(0.0, 'turn_left', 'm1', 5.0)
    assert pl.route[-5:] == ['n0', 'n1', 'n2', 'n3', 'n4'] and pl.replans == 1
    pl.on_action(1.0, 'lane_change_left', 'm1', 6.0)
    assert pl.lane == 'L'
    pl.on_action(2.0, 'slow_down', 'm1', 7.0)
    assert pl.target_speed(3.0) == pl.p.slow and pl.target_speed(20.0) == pl.p.cruise
    pl.on_action(3.0, 'turn_right', 'm4', 17.0)            # too close to the junction: frozen
    assert pl.route[-1] == 'n4'


def test_pure_pursuit_tracks_straight_and_curves():
    path = [(float(x), 0.0) for x in range(0, 50)]
    v, w = pure_pursuit(0.0, 0.0, 0.0, path, 3.0, 3.0, 0.05)
    assert v == pytest.approx(3.0) and abs(w) < 1e-6
    v, w = pure_pursuit(0.0, -1.0, 0.0, path, 3.0, 3.0, 0.05)
    assert w > 0                                          # path is to the left -> turn left
    v, _ = pure_pursuit(0.0, 0.0, 0.0, path, 0.0, 3.0, 0.1)
    assert v == pytest.approx(3.0 - 0.3)                  # decel limit
    assert math.isfinite(pure_pursuit(0, 0, 0, [], 3, 3, 0.1)[0])


def test_verified_workzone_moves_the_car_left_and_back(world):
    from cogndn_core.navigation import BehaviourPlanner
    route = ['m0', 'm1', 'm2', 'm3', 'm4', 'x1', 'e0', 'e1']
    pl = BehaviourPlanner(world, route)
    assert pl.update_lane('m1', 5.0, ['m3', 'm4'], 0.0, 25.0) is None and pl.lane == 'R'   # 35 m before m3
    assert pl.update_lane('m2', 0.0, ['m3', 'm4'], 0.0, 40.0) == 'data' and pl.lane == 'L'
    assert pl.update_lane('m4', 10.0, ['m3', 'm4'], 0.0, 90.0) == 'data' and pl.lane == 'L'
    assert pl.update_lane('e0', 2.0, ['m3', 'm4'], 0.0, 112.0) is None and pl.lane == 'R'


def test_blocked_car_passes_on_the_left_then_returns(world):
    from cogndn_core.navigation import BehaviourPlanner
    pl = BehaviourPlanner(world, ['m0', 'm1', 'm2', 'm3', 'm4'])
    assert pl.update_lane('m3', 1.0, [], 2.0, 61.0) is None and pl.lane == 'R'
    assert pl.update_lane('m3', 1.0, [], 5.0, 61.0) == 'blocked' and pl.lane == 'L'
    assert pl.update_lane('m4', 5.0, [], 0.0, 85.0) == 'blocked' and pl.lane == 'L'
    assert pl.update_lane('m4', 19.0, [], 0.0, 102.0) is None and pl.lane == 'R'


def test_detour_data_reroutes_left_at_the_junction(world):
    from cogndn_core.navigation import BehaviourPlanner
    pl = BehaviourPlanner(world, ['m0', 'm1', 'm2', 'm3', 'm4', 'x1', 'e0', 'e1'])
    assert pl.apply_detour('m2', 5.0, 'x1', ['n0', 'n1'])
    assert pl.route[pl.route.index('x1') + 1] == 'n0'


def test_corridor_bends_with_steering_so_a_merge_can_pass_barrels():
    import numpy as np
    from cogndn_core.envelope import EnvelopeParams, nearest_in_corridor
    p = EnvelopeParams()
    barrels = np.array([[x, 0.0, -1.3] for x in (5.0, 9.0, 13.0)])     # straight ahead, 0.5 m tall
    assert nearest_in_corridor(barrels, p, 0.0) == pytest.approx(13.0)  # k-th (3rd) nearest of 3 points
    assert nearest_in_corridor(barrels, p, 0.2) is None                 # steering left: they are beside the path
    left_car = np.array([[8.0, 3.5, -1.0]] * 3)                         # something in the left lane ...
    assert nearest_in_corridor(left_car, p, 0.0) is None
    assert nearest_in_corridor(left_car, p, 0.2) == pytest.approx(8.0)  # ... is in the way once we steer into it


def test_vla_cannot_undo_a_verified_detour(world):
    from cogndn_core.navigation import BehaviourPlanner
    pl = BehaviourPlanner(world, ['m0', 'm1', 'm2', 'm3', 'm4', 'x1', 'e0', 'e1'])
    assert pl.apply_detour('m2', 5.0, 'x1', ['n0', 'n1'])
    pl.on_action(0.0, 'go_straight', 'm2', 10.0)
    assert pl.route[pl.route.index('x1') + 1] == 'n0'


def test_obstacle_hidden_under_the_bonnet_keeps_the_hold():
    env = TTCEnvelope()
    env.on_cloud(0.0, wall(3.5))
    assert env.apply(0.0, 1.0, 0.0, 0.5)[2]['kind'] == 'stop'
    env.on_cloud(1.0, wall(25.0, n=0))                   # gone from view after the car crept 0.5 m closer
    assert env.apply(1.0, 1.0, 0.0, 0.5)[2]['kind'] == 'hold'
    assert env.apply(3.0, 1.0, 0.0, 0.0)[0] == 0.0


def test_tracker_turns_back_when_the_target_is_behind():
    import math
    from cogndn_core.navigation import pure_pursuit
    path = [(float(x), -1.75) for x in range(0, 60)]
    v, w = pure_pursuit(-50.0, 1.79, math.pi, path, 3.0, 5.8, 0.05)    # car heading the wrong way, too fast
    assert v <= 1.0 and abs(w) > 0.1                                    # crawl and turn, not drive away
    v, w = pure_pursuit(10.0, -1.75, 0.0, path, 3.0, 5.8, 0.05)
    assert v <= 3.5                                                      # never far above the target speed


def test_merge_for_data_does_not_flap_back(world):
    from cogndn_core.navigation import BehaviourPlanner
    pl = BehaviourPlanner(world, ['m0', 'm1', 'm2', 'm3', 'm4', 'x1'])
    assert pl.update_lane('m1', 16.0, ['m3'], 0.0, 36.0) == 'data'      # 24 m before m3: merge
    assert pl.update_lane('m1', 14.0, ['m3'], 0.0, 34.0) == 'data'      # pushed back 2 m: stay left
    assert pl.update_lane('m4', 2.0, ['m3'], 0.0, 82.0) is None and pl.lane == 'R'
