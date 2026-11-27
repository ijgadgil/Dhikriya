import pytest

from cogndn_core.compiler import CognitiveInterestCompiler, CompilerParams

ROUTE = ['m0', 'm1', 'm2', 'm3', 'm4', 'x1', 'e0', 'e1', 'e2', 'e3', 'e4']


def mk(world, **kw):
    return CognitiveInterestCompiler(world, params=CompilerParams(**kw))


def test_route_layer_issues_maps(world):
    c = mk(world, base_k=2, silent_s=1e9)
    c.last_vla = 0.0
    out = c.step(0.0, 'm0', 0.0, 3.0, ROUTE)
    assert sorted(i.name for i in out) == ['/cogndn/town/m0/map', '/cogndn/town/m1/map', '/cogndn/town/m2/map']
    assert all(i.source == 'route' for i in out)
    assert c.step(0.1, 'm0', 0.3, 3.0, ROUTE) == []        # outstanding, not re-sent


def test_freshness_aware_deferral(world):
    c = mk(world, base_k=0, silent_s=1e9)
    c.add_needs(0.0, [{'type': 'hazard', 'zone': 'm3', 'confidence': 0.9}])
    # m3 is 60 m away at 3 m/s -> tau = 20 s; hazard F = 30 s, beta = 0.5 -> release when tau <= ~15 s
    assert c.step(0.0, 'm0', 0.0, 3.0, ROUTE) == []
    assert c.stats.deferred == 1
    out = c.step(6.0, 'm0', 18.0, 3.0, ROUTE)
    assert [i.name for i in out] == ['/cogndn/town/m3/hazard']
    assert out[0].deadline == pytest.approx(6.0 + 42.0 / 3.0)
    assert out[0].lifetime_s == pytest.approx(min(30.0, 14.0 + 2.0))


def test_priority_and_unreachable(world):
    c = mk(world, base_k=1, silent_s=1e9)
    c.add_needs(0.0, [{'type': 'hazard', 'zone': 'm1', 'confidence': 1.0},
                      {'type': 'hazard', 'zone': 'm0', 'confidence': 1.0}])
    out = c.step(0.0, 'm1', 1.0, 3.0, ROUTE)
    names = [i.name for i in out]
    assert '/cogndn/town/m0/hazard' not in names            # behind the vehicle
    assert c.stats.dropped_unreachable >= 1
    assert names[0] == '/cogndn/town/m1/hazard'               # hazard outranks the map tile


def test_signal_without_qualifier_expands(world):
    c = mk(world, base_k=0, silent_s=1e9)
    c.add_needs(0.0, [{'type': 'signal', 'zone': 'x1', 'confidence': 1.0}])
    out = c.step(0.0, 'm4', 15.0, 3.0, ROUTE)
    assert sorted(i.qualifier for i in out) == ['left', 'right', 'straight']


def test_invalid_needs_dropped(world):
    c = mk(world)
    c.add_needs(0.0, [{'type': 'teleport', 'zone': 'm1', 'confidence': 1},
                      {'type': 'hazard', 'zone': 'q9', 'confidence': 1},
                      {'type': 'hazard', 'zone': 'm1', 'qualifier': 'left', 'confidence': 1}])
    assert c.stats.dropped_invalid == 3 and not c.pending


def test_dedup_with_fresh_copy(world):
    c = mk(world, base_k=0, silent_s=1e9)
    c.add_needs(0.0, [{'type': 'hazard', 'zone': 'm1', 'confidence': 1.0}])
    out = c.step(0.0, 'm0', 10.0, 3.0, ROUTE, fresh_until=lambda n: 100.0)
    assert out == [] and c.stats.deduplicated == 1


def test_token_bucket(world):
    c = mk(world, base_k=6, rate_per_s=1.0, burst=2, silent_s=1e9)
    out = c.step(0.0, 'm0', 0.0, 3.0, ROUTE)
    assert len(out) == 2 and c.stats.rate_limited > 0
    assert len(c.step(1.0, 'm0', 3.0, 3.0, ROUTE)) == 1


def test_fallback_when_vla_silent(world):
    c = mk(world, base_k=0, fallback_k=1, silent_s=5.0)
    out = c.step(0.0, 'm0', 0.0, 3.0, ROUTE)
    assert {i.source for i in out} == {'fallback'}
    c2 = mk(world, base_k=0, fallback_k=1, silent_s=5.0)
    c2.add_needs(0.0, [])
    assert c2.step(1.0, 'm0', 0.0, 3.0, ROUTE) == []


def test_signal_need_outside_a_junction_is_dropped(world):
    c = CognitiveInterestCompiler(world)
    c.add_needs(0.0, [{'type': 'signal', 'zone': 'e1', 'qualifier': None, 'confidence': 0.9}])
    assert c.stats.dropped_invalid == 1 and not c.pending
    c.add_needs(0.0, [{'type': 'signal', 'zone': 'x1', 'qualifier': 'straight', 'confidence': 0.9}])
    assert len(c.pending) == 1


def test_far_need_survives_a_stop_and_is_sent_just_in_time(world):
    c = CognitiveInterestCompiler(world)
    route = ['m2', 'm3', 'm4', 'x1', 'e0', 'e1', 'e2']
    c.add_needs(0.0, [{'type': 'crosswalk', 'zone': 'e2', 'qualifier': None, 'confidence': 0.7}])
    c.step(0.0, 'm2', 10.0, 0.0, route)                     # stopped 80 m away: beyond the horizon, kept
    assert '/cogndn/town/e2/crosswalk' in c.pending
    out = c.step(30.0, 'e1', 18.0, 3.0, route)               # 2 m before e2: released now
    assert [i.name for i in out if 'crosswalk' in i.name] == ['/cogndn/town/e2/crosswalk']


def test_on_demand_requests_only_the_current_zone(world):
    from cogndn_core.compiler import CompilerParams
    from cogndn_core.catalogue import DEFAULT_TYPES
    prm = CompilerParams(base_types=tuple(DEFAULT_TYPES), base_k=0, on_demand=True, fallback_k=0)
    c = CognitiveInterestCompiler(world, params=prm)
    c.step(0.0, 'm2', 5.0, 3.0, ['m1', 'm2', 'm3', 'm4', 'x1'])
    zones = {n.zone for n in c.pending.values()} | {i.split('/')[3] for i in c.outstanding}
    assert zones == {'m2'}
