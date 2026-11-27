import os
import sys

HERE = os.path.dirname(__file__)
sys.path[:0] = [os.path.join(HERE, '..'), os.path.join(HERE, '..', '..', 'cogndn_core')]

import pytest  # noqa: E402

from cogndn_core.catalogue import DEFAULT_TYPES  # noqa: E402
from cogndn_core.scenario import Scenario  # noqa: E402
from cogndn_core.world import World  # noqa: E402
from cogndn_eval.harness import RunParams, VLAParams, run_once  # noqa: E402
from cogndn_eval.netsim import Network, NetParams  # noqa: E402

CFG = os.path.join(HERE, '..', '..', 'cogndn_sim', 'config')
W = World.from_yaml(os.path.join(CFG, 'town.yaml'))


def sc(sid):
    return Scenario.from_yaml(os.path.join(CFG, 'scenarios', sid + '.yaml'), W)


def test_network_cache_and_aggregation():
    n = Network(NetParams(origin_median_s=0.05, origin_sigma=0.0, one_way_s=0.01, rate_kbps=8000), DEFAULT_TYPES)
    a1, _ = n.request('a', '/cogndn/town/m1/hazard', 'hazard', 0.0)
    a2, _ = n.request('b', '/cogndn/town/m1/hazard', 'hazard', 0.02)      # pending at RSU -> aggregated
    a3, _ = n.request('c', '/cogndn/town/m1/hazard', 'hazard', 1.0)       # cached at RSU
    assert n.origin_requests == 1 and n.rsu_hits == 1
    assert a1 == pytest.approx(0.01 + 0.05 + 0.002 + 0.01)
    assert a2 == pytest.approx(a1)
    assert a3 - 1.0 == pytest.approx(0.01 + 0.001 + 0.002 + 0.01)


def test_deterministic_with_seed():
    a = run_once(sc('s1_livestock'), 'cognitive', RunParams(speed=8.0), seed=3)
    b = run_once(sc('s1_livestock'), 'cognitive', RunParams(speed=8.0), seed=3)
    assert a == b


def test_oracle_upper_bounds_and_control_parity():
    for sid in ('s2_workzone', 's3_closure_detour', 's4_school_crossing'):
        o = run_once(sc(sid), 'oracle', RunParams(speed=8.0), seed=1)
        for pol in ('route_typed', 'rules', 'cognitive'):
            assert run_once(sc(sid), pol, RunParams(speed=8.0), seed=1)['cov_semantic'] <= o['cov_semantic'] + 1e-9
    c = run_once(sc('s0_straight_control'), 'cognitive', RunParams(speed=8.0), seed=1)
    r = run_once(sc('s0_straight_control'), 'route_typed', RunParams(speed=8.0), seed=1)
    assert c['cov_all'] == r['cov_all']


def test_full_miss_rate_degrades_to_route_layer():
    m = run_once(sc('s2_workzone'), 'cognitive', RunParams(speed=8.0), vla=VLAParams(p_miss=1.0), seed=0)
    assert m['cov_semantic'] == 0.0 and m['cov_all'] > 0.5


def test_fleet_shares_rsu():
    one = run_once(sc('s1_livestock'), 'route_typed', RunParams(speed=8.0), seed=0, n_vehicles=1)
    ten = run_once(sc('s1_livestock'), 'route_typed', RunParams(speed=8.0), seed=0, n_vehicles=10, headway_s=5)
    assert ten['origin_kb_per_vehicle_km'] < one['origin_kb_per_vehicle_km'] / 5
