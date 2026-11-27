import glob
import json
import os

import pytest

from cogndn_core.catalogue import DEFAULT_TYPES
from cogndn_core.scenario import Scenario, all_names, content_for, signal_phase


def test_all_scenarios_load(world, scen_dir):
    files = sorted(glob.glob(os.path.join(scen_dir, '*.yaml')))
    assert len(files) == 6
    for f in files:
        sc = Scenario.from_yaml(f, world)
        for z in sc.planned_route + sc.truth_route:
            assert z in world.zones
        for r in sc.required:
            assert r.zone in world.zones and r.type in DEFAULT_TYPES
        maps = {r.zone for r in sc.required if r.type == 'map'}
        assert maps == set(sc.truth_route[1:])


def test_detour_scenario_needs_offroute_data(world, scen_dir):
    sc = Scenario.from_yaml(os.path.join(scen_dir, 's3_closure_detour.yaml'), world)
    names = {r.name for r in sc.required}
    assert '/cogndn/town/x1/signal/left' in names and '/cogndn/town/n0/map' in names
    assert sc.route_s('x1') == pytest.approx(100.0) and sc.route_s('n1') == pytest.approx(130.0)


def test_content_size_and_signal():
    c = content_for('/cogndn/town/m1/map', 0.0)
    assert abs(len(json.dumps(c, sort_keys=True, separators=(',', ':'))) - 200000) < 50
    assert signal_phase(5.0, 'straight') == ('green', 7.0)
    assert signal_phase(5.0, 'left')[0] == 'red'
    sd = {'/cogndn/town/m3/hazard': {'category': 'animal', 'status': 'active'}}
    assert content_for('/cogndn/town/m3/hazard', 0.0, sd)['category'] == 'animal'


def test_all_names(world):
    names = all_names(world)
    assert '/cogndn/town/x1/signal/left' in names and '/cogndn/town/m0/signal' not in ' '.join(names)
