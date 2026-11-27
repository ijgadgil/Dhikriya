"""Scenario definitions: required data (ground truth), cues, and the content producers serve."""
import json
from dataclasses import dataclass, field

import yaml

from .catalogue import DEFAULT_TYPES, SIGNAL_MOVEMENTS
from .naming import make_name


@dataclass
class Required:
    name: str
    zone: str
    type: str
    qualifier: object
    deadline_zone: str
    cues: list = field(default_factory=list)
    semantic: bool = True     # False for route map tiles added automatically


class Scenario:
    def __init__(self, cfg, world):
        self.cfg = cfg
        self.id = cfg['id']
        self.world = world
        self.instruction = cfg.get('instruction', '')
        self.planned_route = list(cfg['planned_route'])
        self.truth_route = list(cfg.get('truth_route', self.planned_route))
        self.data = cfg.get('data', {}) or {}
        self.replan_cue = cfg.get('replan_cue')
        req = []
        for r in cfg.get('required', []) or []:
            req.append(Required(make_name(r['zone'], r['type'], r.get('qualifier')), r['zone'], r['type'],
                                r.get('qualifier'), r.get('deadline_zone', r['zone']), r.get('cues', [])))
        named = {r.name for r in req}
        for z in self.truth_route[1:]:          # the spawn zone's tile is preloaded before departure
            n = make_name(z, 'map')
            if n not in named:
                req.append(Required(n, z, 'map', None, z, [{'kind': 'route', 's': 0.0, 'visible_m': 1e9}],
                                    semantic=False))
        self.required = req

    @classmethod
    def from_yaml(cls, path, world):
        with open(path) as f:
            return cls(yaml.safe_load(f), world)

    def route_s(self, zone, route=None):
        """Along-route distance of the entry of `zone` on the (truth) route."""
        route = route or self.truth_route
        s = 0.0
        for z in route:
            if z == zone:
                return s
            s += self.world.zones[z].length
        return None

    def length(self, route=None):
        return sum(self.world.zones[z].length for z in (route or self.truth_route))


def signal_phase(t, movement, cycle=30.0):
    """Deterministic fixed-time plan: straight green 0-12 s, left 12-18 s, right 18-24 s, all-red 24-30 s."""
    windows = {'straight': (0.0, 12.0), 'left': (12.0, 18.0), 'right': (18.0, 24.0)}
    ph = t % cycle
    a, b = windows[movement]
    if a <= ph < b:
        return 'green', round(b - ph, 1)
    nxt = (a - ph) % cycle
    return 'red', round(nxt, 1)


def content_for(name, t, scenario_data=None, types=None, pad=True):
    """Content served by producers for `name` at time t (dict); padded to the nominal size."""
    from .naming import parse_name
    types = types or DEFAULT_TYPES
    p = parse_name(name)
    zt, ty, q = p['zone'], p['type'], p['qualifier']
    if scenario_data and name in scenario_data:
        c = dict(scenario_data[name])
    elif ty == 'signal':
        st, ttc = signal_phase(t, q or 'straight')
        c = {'movement': q, 'phase': st, 'time_to_change_s': ttc, 'summary': '%s %s, changes in %.0f s' % (q, st, ttc)}
    elif ty == 'map':
        c = {'zone': zt, 'summary': 'map tile for %s' % zt}
    elif ty == 'hazard':
        c = {'status': 'clear', 'summary': 'no hazards reported in %s' % zt}
    elif ty == 'workzone':
        c = {'status': 'clear', 'summary': 'no road works in %s' % zt}
    elif ty == 'detour':
        c = {'status': 'clear', 'summary': 'no detour at %s' % zt}
    elif ty == 'crosswalk':
        c = {'status': 'clear', 'occupied': False, 'summary': 'no crossing activity in %s' % zt}
    else:
        c = {'summary': '%s for %s' % (ty, zt)}
    c['name'] = name
    if pad and ty in types:
        target = int(types[ty].size_kb * 1000)
        base = len(json.dumps(c, sort_keys=True, separators=(',', ':')))
        if target > base + 12:
            c['pad'] = 'x' * (target - base - 10)
    return c


def all_names(world, types=None):
    """Every name a producer for this town can serve."""
    types = types or DEFAULT_TYPES
    out = []
    for z in world.zones.values():
        for t in types.values():
            if t.qualified:
                if z.kind == 'intersection':
                    out += [make_name(z.id, t.name, m) for m in SIGNAL_MOVEMENTS]
            else:
                out.append(make_name(z.id, t.name))
    return out
