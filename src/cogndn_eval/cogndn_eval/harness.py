"""Headless experiment harness.

Drives one or more vehicles along a scenario at constant speed and runs the *same* Cognitive
Interest Compiler as the ROS nodes against the network model in netsim.py. Need sources:

  SyntheticVLA  cue-driven VLA model with measured latency and injected errors
                (miss rate, false-request rate, wrong-zone rate) -> robustness experiments
  ReplayVLA     replays Cognitive States recorded from real llava-phi3 runs in Gazebo
  RulesSource   detector-only rules (object classes -> data types); cannot read signs
  OracleSource  knows every required item at t = 0 (upper bound)

Policies (all fall back to an on-demand fetch if an item is missing at its deadline):
  ip_ondemand, ndn_ondemand, route_typed, route_all, rules, cognitive, cognitive_hybrid, oracle
"""
import math
import random
from dataclasses import dataclass, field

from cogndn_core.catalogue import DEFAULT_TYPES
from cogndn_core.compiler import CognitiveInterestCompiler, CompilerParams
from cogndn_core.naming import make_name

from .netsim import Network, NetParams

POLICIES = ('ip_ondemand', 'ndn_ondemand', 'route_typed', 'route_all', 'rules', 'cognitive',
            'cognitive_hybrid', 'oracle')
RULES_MAP = {'cow': 'hazard', 'person': 'crosswalk'}
VLA_TYPES = ('hazard', 'signal', 'workzone', 'detour', 'crosswalk', 'scene')


@dataclass
class VLAParams:
    lat_lo: float = 3.4                # s, uniform latency bounds (replace with measured samples)
    lat_hi: float = 5.9
    lat_samples: tuple = ()
    p_miss: float = 0.0                # per required item, per run: the VLA never asks for it
    p_false: float = 0.0               # per cycle: one spurious request
    p_wrong_zone: float = 0.0          # per request: zone shifted by one along the route


@dataclass
class RunParams:
    speed: float = 3.0
    dt: float = 0.1
    k: int = 3                         # look-ahead zones for route layers
    hit_s: float = 0.001               # local cache hit time
    compiler: CompilerParams = field(default_factory=CompilerParams)


# ---------------------------------------------------------------------------- need sources
def _cue_visible(cue, s):
    if cue['kind'] == 'route':
        return True
    return cue['s'] - cue['visible_m'] <= s <= cue['s'] + 5.0


class OracleSource:
    def __init__(self, sc, rng, **_):
        self.sc, self.done = sc, False

    def poll(self, t, s):
        if self.done:
            return [], None
        self.done = True
        return [_need(r) for r in self.sc.required if r.semantic], self.sc.truth_route


class RulesSource:
    def __init__(self, sc, rng, **_):
        self.sc, self.sent, self.replanned = sc, set(), False

    def poll(self, t, s):
        out = []
        for r in self.sc.required:
            if not r.semantic or r.name in self.sent:
                continue
            for c in r.cues:
                if c['kind'] == 'object' and RULES_MAP.get(c.get('cls')) == r.type and _cue_visible(c, s):
                    out.append(_need(r))
                    self.sent.add(r.name)
                    break
        route = None
        rc = self.sc.replan_cue
        if rc and not self.replanned and _cue_visible(rc, s):
            self.replanned, route = True, self.sc.truth_route
        return out, route


class SyntheticVLA:
    def __init__(self, sc, rng, vla=None, **_):
        self.sc, self.rng, self.p = sc, rng, vla or VLAParams()
        sem = [r for r in sc.required if r.semantic]
        self.understood = {r.name: rng.random() >= self.p.p_miss for r in sem}
        self.next_start, self.inflight = 0.0, None
        self.replanned = False
        self.cycles = 0

    def _latency(self):
        if self.p.lat_samples:
            return self.rng.choice(self.p.lat_samples)
        return self.rng.uniform(self.p.lat_lo, self.p.lat_hi)

    def poll(self, t, s):
        out, route = None, None
        if self.inflight and t >= self.inflight[0]:
            _, cap_s = self.inflight
            self.inflight = None
            self.cycles += 1
            out, route = self._think(cap_s)
        if self.inflight is None and t >= self.next_start:
            lat = self._latency()
            self.inflight = (t + lat, s)
            self.next_start = t + lat
        return out, route

    def _think(self, s):
        sc, out, route = self.sc, [], None
        tr = sc.truth_route
        for r in sc.required:
            if not r.semantic or not self.understood[r.name]:
                continue
            if not any(_cue_visible(c, s) for c in r.cues):
                continue
            n = _need(r, conf=0.9)
            if self.p.p_wrong_zone and self.rng.random() < self.p.p_wrong_zone and r.zone in tr:
                i = tr.index(r.zone)
                n['zone'] = tr[min(len(tr) - 1, i + self.rng.choice((-1, 1)))]
            out.append(n)
            if not self.replanned and sc.truth_route != sc.planned_route:
                self.replanned, route = True, sc.truth_route
        if self.p.p_false and self.rng.random() < self.p.p_false:
            ahead = [z for z in tr if sc.route_s(z) is not None and 0 <= sc.route_s(z) - s <= 100.0] or tr[-1:]
            out.append({'type': self.rng.choice(VLA_TYPES), 'zone': self.rng.choice(ahead),
                        'qualifier': None, 'confidence': 0.6})
        return out, route


class ReplayVLA:
    """records: [{'s': capture position, 'latency_s': L, 'needs': [...], 'replan': bool}]."""

    def __init__(self, sc, rng, records=None, speed=3.0, **_):
        self.sc = sc
        self.ev = sorted(records or [], key=lambda r: r['s'])
        self.speed = speed
        self.i = 0

    def poll(self, t, s):
        out, route = None, None
        while self.i < len(self.ev) and s >= self.ev[self.i]['s'] + self.speed * self.ev[self.i].get('latency_s', 0):
            r = self.ev[self.i]
            out = (out or []) + r.get('needs', [])
            if r.get('replan'):
                route = self.sc.truth_route
            self.i += 1
        return out, route


def _need(r, conf=1.0):
    return {'type': r.type, 'zone': r.zone, 'qualifier': r.qualifier, 'confidence': conf}


# ---------------------------------------------------------------------------- one vehicle
class VehicleRun:
    def __init__(self, vid, sc, policy, run, net, rng, t0=0.0, vla=None, records=None, types=None):
        self.vid, self.sc, self.policy, self.run, self.net = vid, sc, policy, run, net
        self.types = types or DEFAULT_TYPES
        self.t0 = t0
        self.route = list(sc.planned_route) if policy != 'oracle' else list(sc.truth_route)
        self.store = {}            # name -> (arrival, fresh_until)
        self.inflight = {}         # name -> (arrival, gen)
        self.fetched_kb = 0.0
        self.useful = set()
        self.req = {r.name: r for r in sc.required}
        self.deadline_s = {r.name: sc.route_s(r.deadline_zone) for r in sc.required}
        self.results = {}
        self.ondemand_kb = 0.0
        cp = CompilerParams(**{**run.compiler.__dict__})
        cp.rate_est_kbps = net.p.rate_kbps
        cp.rtt_est = 2 * net.p.one_way_s + net.p.origin_median_s
        if policy == 'route_typed':
            cp.base_types, cp.base_k = ('map', 'hazard', 'signal'), run.k
        elif policy == 'route_all':
            cp.base_types, cp.base_k = tuple(self.types), run.k
        elif policy == 'cognitive_hybrid':
            cp.base_types, cp.base_k = ('map', 'hazard', 'signal'), run.k
        else:
            cp.base_types, cp.base_k = ('map',), run.k
        cp.fallback_k = run.k if policy in ('cognitive', 'cognitive_hybrid') else 0
        self.comp = None if policy in ('ip_ondemand', 'ndn_ondemand') else CognitiveInterestCompiler(
            sc.world, self.types, cp)
        src = {'rules': RulesSource, 'cognitive': SyntheticVLA, 'cognitive_hybrid': SyntheticVLA,
               'oracle': OracleSource}.get(policy)
        if policy in ('cognitive', 'cognitive_hybrid') and records is not None:
            src = ReplayVLA
        self.src = src(sc, rng, vla=vla, records=records, speed=run.speed) if src else None
        if self.comp and self.src is None:
            self.comp.last_vla = math.inf * -1  # no VLA: fallback layer governed by fallback_k
        self.replan_s = None
        self.sent = []             # (time since departure, name) of every Interest this vehicle sent

    def fresh_until(self, name):
        v = self.store.get(name)
        return v[1] if v else None

    def _deliver(self, t):
        for name, (arr, gen) in list(self.inflight.items()):
            if arr <= t:
                ty = name.split('/')[4]
                self.store[name] = (arr, gen + self.types[ty].freshness_s)
                self.fetched_kb += self.types[ty].size_kb
                if name in self.req:
                    self.useful.add(name)
                del self.inflight[name]
                if self.comp:
                    self.comp.on_data(name)

    def _send(self, name, ty, t, lifetime=4.0):
        self.sent.append((round(t - self.t0, 3), name))
        if self.policy == 'ip_ondemand':
            res = self._ip(name, ty, t)
        else:
            res = self.net.request(self.vid, name, ty, t, lifetime)
        if res:
            self.inflight[name] = res
        return res

    def _ip(self, name, ty, t):
        saved = self.net.p.rsu_cache, self.net.p.aggregation
        self.net.p.rsu_cache, self.net.p.aggregation = False, False
        try:
            return self.net.request(('ip', self.vid), name, ty, t, 30.0)
        finally:
            self.net.p.rsu_cache, self.net.p.aggregation = saved

    def step(self, t):
        sc, run = self.sc, self.run
        if t < self.t0:
            return True
        s = run.speed * (t - self.t0)
        total = sc.length()
        if s > total:
            return False
        zone, s_in = self._locate(s)
        self._deliver(t)
        if self.src:
            needs, route = self.src.poll(t, s)
            if route and route != self.route:
                self.route, self.replan_s = list(route), s
            if needs is not None and (needs or isinstance(self.src, (SyntheticVLA, ReplayVLA))):
                self.comp.add_needs(t, needs, 'cognitive')
        if self.comp:
            if sc.replan_cue and self.route != sc.truth_route and _cue_visible(sc.replan_cue, s):
                # every stack's own perception eventually sees the closure and replans
                self.route, self.replan_s = list(sc.truth_route), s
            for it in self.comp.step(t, zone, s_in, run.speed, self.route, self.fresh_until):
                if it.name not in self.inflight:
                    self._send(it.name, it.type, t, it.lifetime_s)
        self._check_deadlines(t, s)
        return True

    def _locate(self, s):
        acc = 0.0
        for z in self.sc.truth_route:
            L = self.sc.world.zones[z].length
            if s < acc + L:
                return z, s - acc
            acc += L
        z = self.sc.truth_route[-1]
        return z, self.sc.world.zones[z].length

    def _check_deadlines(self, t, s):
        for name, sd in self.deadline_s.items():
            if name in self.results or sd is None or s < sd:
                continue
            r = self.req[name]
            v = self.store.get(name)
            if v and v[0] <= t and t < v[1]:
                self.results[name] = {'on_time': True, 'delay': self.run.hit_s, 'semantic': r.semantic}
                continue
            if name in self.inflight:
                delay = self.inflight[name][0] - t
            else:
                res = self._send(name, r.type, t, 30.0)
                delay = (res[0] - t) if res else 30.0
                self.ondemand_kb += self.types[r.type].size_kb
            self.results[name] = {'on_time': False, 'delay': max(self.run.hit_s, delay), 'semantic': r.semantic}

    def metrics(self):
        res = self.results
        sem = [v for v in res.values() if v['semantic']]
        allv = list(res.values())
        delays = sorted(v['delay'] for v in allv)
        km = self.sc.length() / 1000.0
        useful_kb = sum(self.types[self.req[n].type].size_kb for n in self.useful)
        return {
            'cov_semantic': (sum(v['on_time'] for v in sem) / len(sem)) if sem else float('nan'),
            'cov_all': sum(v['on_time'] for v in allv) / max(1, len(allv)),
            'n_semantic': len(sem),
            'delay_mean_ms': 1000.0 * sum(delays) / max(1, len(delays)),
            'delay_p95_ms': 1000.0 * (delays[min(len(delays) - 1, int(0.95 * len(delays)))] if delays else 0.0),
            'delay_sem_mean_ms': 1000.0 * sum(v['delay'] for v in sem) / len(sem) if sem else float('nan'),
            'fetched_kb': self.fetched_kb,
            'useful_kb': useful_kb,
            'precision': useful_kb / self.fetched_kb if self.fetched_kb else float('nan'),
            'wasted_kb_per_km': (self.fetched_kb - useful_kb) / km,
            'kb_per_km': self.fetched_kb / km,
            'interests': self.comp.stats.issued if self.comp else 0,
            'replan_s': self.replan_s,
        }


# ---------------------------------------------------------------------------- experiment
def run_once(sc, policy, run=None, net=None, vla=None, seed=0, n_vehicles=1, headway_s=10.0, records=None,
             types=None):
    """Simulate n vehicles (same scenario and policy) sharing one road-side unit. Returns a dict with
    per-vehicle mean metrics and network-level counters."""
    run = run or RunParams()
    types = types or DEFAULT_TYPES
    network = Network(net or NetParams(), types, seed=seed)
    vs = [VehicleRun(i, sc, policy, run, network, random.Random(seed * 1000 + i), t0=i * headway_s, vla=vla,
                     records=records, types=types) for i in range(n_vehicles)]
    t, alive = 0.0, set(range(n_vehicles))
    while alive:
        for v in vs:
            if v.vid in alive and not v.step(t):
                alive.discard(v.vid)
        t += run.dt
    ms = [v.metrics() for v in vs]
    out = {k: _nanmean([m[k] for m in ms]) for k in ms[0] if k != 'replan_s'}
    km = sc.length() / 1000.0 * n_vehicles
    out.update({'origin_requests': network.origin_requests, 'origin_kb_per_vehicle_km': network.origin_kb / km,
                'rsu_hit_ratio': network.rsu_hits / max(1, network.rsu_requests), 'n_vehicles': n_vehicles})
    return out


def _nanmean(xs):
    xs = [x for x in xs if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return sum(xs) / len(xs) if xs else float('nan')


def item_name(zone, ty, q=None):
    return make_name(zone, ty, q)
