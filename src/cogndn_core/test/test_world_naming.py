import math

import pytest

from cogndn_core.naming import make_name, parse_name


def test_name_roundtrip():
    n = make_name('x1', 'signal', 'left')
    assert n == '/cogndn/town/x1/signal/left'
    assert parse_name(n) == {'area': 'town', 'zone': 'x1', 'type': 'signal', 'qualifier': 'left'}
    assert parse_name(make_name('m3', 'hazard'))['qualifier'] is None


def test_name_rejects_bad_components():
    with pytest.raises(ValueError):
        make_name('m/3', 'hazard')
    with pytest.raises(ValueError):
        parse_name('/other/x')


def test_zones_and_adjacency(world):
    assert len(world.zones) == 21
    assert world.zones['m4'].succ == {'x1': 'straight'}
    assert world.zones['x1'].succ == {'e0': 'straight', 'n0': 'left', 's0': 'right'}


def test_distance(world):
    assert world.distance_to('m0', 0.0, 'x1') == pytest.approx(100.0)
    assert world.distance_to('m0', 5.0, 'n0') == pytest.approx(105.0)
    assert world.distance_to('m2', 3.0, 'm2') == 0.0
    assert math.isinf(world.distance_to('e0', 0.0, 'm0'))


def test_locate_and_lanes(world):
    z, s, lat = world.locate(45.0, -1.75)
    assert z == 'm2' and s == pytest.approx(5.0) and lat == pytest.approx(-1.75)
    assert world.locate(105.0, 0.5)[0] == 'x1'
    pts = world.path_points(['m0', 'm1'], lane='R')
    assert all(abs(p[1] + 1.75) < 1e-9 for p in pts)
    pts = world.path_points(['n0'], lane='R')
    assert all(abs(p[0] - 106.75) < 1e-9 for p in pts)   # right of a northbound lane is +x


def test_route_ahead(world):
    r = ['m0', 'm1', 'm2', 'm3', 'm4', 'x1', 'e0']
    ahead = world.route_ahead(r, 'm1', 10.0, horizon_m=45.0)
    assert [z for z, _ in ahead] == ['m1', 'm2', 'm3']
    assert ahead[1][1] == pytest.approx(10.0)
