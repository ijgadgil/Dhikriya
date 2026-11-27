"""C1 — Cognitive Interest Compiler.

Turns information needs (from the VLA, from the planned route, or from a fallback rule)
into named, prioritised NDN Interests that are released at the right time.

For a need j on zone z_j with data type k_j:
  d_j   road distance from the vehicle to the entry of z_j                 (World.distance_to)
  tau_j = d_j / max(v, v_min)                    time until the data is needed       (C1-1)
  theta_j = rtt_est + size_k / rate_est           expected retrieval time             (C1-2)
  pi_j  = c_j * kappa_k / (1 + tau_j / tau_0)     request priority                    (C1-3)
  r_j   = now + max(0, tau_j - theta_j - beta * F_k)   release time (freshness-aware)  (C1-4)
  lambda_j = clip(tau_j + delta, lambda_min, lambda_max)  Interest lifetime            (C1-5)
with c_j the confidence, kappa_k the type criticality, F_k the freshness period.
A need is dropped when its zone is unreachable or tau_j exceeds the horizon H.
Released Interests are de-duplicated and pass a token bucket (rate rho, burst b),
highest priority first.
"""
import math
from dataclasses import dataclass, field

from .catalogue import DEFAULT_TYPES, SIGNAL_MOVEMENTS
from .naming import make_name


@dataclass
class CompilerParams:
    v_min: float = 1.0            # m/s, floor for the speed estimate
    horizon_s: float = 60.0       # H: ignore needs further away in time
    tau0: float = 5.0             # s, urgency scale
    beta: float = 0.5             # fraction of the freshness period to keep at arrival
    rtt_est: float = 0.08         # s, running estimate of the network round trip
    rate_est_kbps: float = 10000.0  # kbit/s, running estimate of the downlink rate
    delta: float = 2.0            # s, lifetime slack
    lifetime_min: float = 1.0
    lifetime_max: float = 30.0
    rate_per_s: float = 20.0      # rho: Interests per second
    burst: int = 10               # b
    base_types: tuple = ('map',)  # route-essential types fetched for the next K zones
    base_k: int = 3
    on_demand: bool = False       # no prediction: request every base type for the zone the car is in, when it enters
    fallback_types: tuple = ('map', 'hazard')
    fallback_k: int = 3
    silent_s: float = 15.0        # VLA silence that activates the fallback layer
    need_ttl_s: float = 120.0     # a cognitive need expires if not re-asserted (it also ends when its zone is passed)


@dataclass
class NamedInterest:
    name: str
    zone: str
    type: str
    qualifier: object
    priority: float
    deadline: float      # absolute time the data is needed
    release: float       # absolute time the Interest may be sent
    lifetime_s: float
    source: str          # 'route' | 'cognitive' | 'fallback'
    size_kb: float
    late_risk: bool = False


@dataclass
class _Need:
    zone: str
    type: str
    qualifier: object
    confidence: float
    source: str
    created: float


@dataclass
class CompilerStats:
    received: int = 0
    dropped_unreachable: int = 0
    dropped_invalid: int = 0
    deduplicated: int = 0
    deferred: int = 0
    rate_limited: int = 0
    issued: int = 0
    by_source: dict = field(default_factory=dict)


class CognitiveInterestCompiler:
    def __init__(self, world, types=None, params=None):
        self.world = world
        self.types = types or DEFAULT_TYPES
        self.p = params or CompilerParams()
        self.pending = {}            # name -> _Need
        self.outstanding = {}        # name -> expiry time of the sent Interest
        self.tokens = float(self.p.burst)
        self.last_t = None
        self.last_vla = None
        self.stats = CompilerStats()

    # ------------------------------------------------------------------ inputs
    def add_needs(self, now, needs, source='cognitive'):
        """needs: iterable of objects/dicts with type, zone, qualifier, confidence."""
        if source == 'cognitive':
            self.last_vla = now
        for n in needs:
            g = n if isinstance(n, dict) else n.__dict__ if not hasattr(n, 'model_dump') else n.model_dump()
            self.stats.received += 1
            t, z, q = g.get('type'), g.get('zone'), g.get('qualifier')
            dt = self.types.get(t)
            if (dt is None or z not in self.world.zones or (q and not dt.qualified)
                    or (dt.qualified and self.world.zones[z].kind != 'intersection')):
                self.stats.dropped_invalid += 1
                continue
            if dt.qualified and not q:
                qs = SIGNAL_MOVEMENTS          # unspecified movement: ask for all of them
            else:
                qs = (q,)
            for qq in qs:
                name = make_name(z, t, qq)
                old = self.pending.get(name)
                conf = float(g.get('confidence', 1.0))
                if old is None:
                    self.pending[name] = _Need(z, t, qq, conf, source, now)
                else:
                    old.confidence = max(old.confidence, conf)
                    old.source = self._merge_src(old.source, source)
                    if source == 'cognitive':
                        old.created = now

    @staticmethod
    def _merge_src(a, b):
        order = {'cognitive': 0, 'route': 1, 'ondemand': 1, 'fallback': 2}
        return a if order.get(a, 9) <= order.get(b, 9) else b

    def on_data(self, name):
        self.outstanding.pop(name, None)

    # ------------------------------------------------------------------ main step
    def step(self, now, zone, s_in_zone, speed, route, fresh_until=None):
        """Return the Interests to send now. fresh_until(name) -> absolute time until which a
        cached copy stays fresh (or None)."""
        p = self.p
        self._refill(now)
        self._add_layer(now, zone, s_in_zone, route, p.base_types, p.base_k, 'route', 1.0, fresh_until)
        if p.on_demand:
            self._add_layer(now, zone, s_in_zone, [zone], p.base_types, 1, 'ondemand', 1.0, fresh_until)
        if self.last_vla is None or now - self.last_vla > p.silent_s:
            self._add_layer(now, zone, s_in_zone, route, p.fallback_types, p.fallback_k, 'fallback', 0.5,
                            fresh_until)
        for k in [k for k, exp in self.outstanding.items() if exp <= now]:
            del self.outstanding[k]
        v = max(speed, p.v_min)
        ready = []
        for name, nd in list(self.pending.items()):
            if nd.source == 'cognitive' and now - nd.created > p.need_ttl_s:
                del self.pending[name]
                continue
            d = self.world.distance_to(zone, s_in_zone, nd.zone)
            if not math.isfinite(d):           # behind the car or off the route: the need is over
                del self.pending[name]
                self.stats.dropped_unreachable += 1
                continue
            tau = d / v
            if tau > p.horizon_s:              # too far ahead *for now* (e.g. while stopped): keep it, decide later
                self.stats.deferred += 1
                continue
            dt = self.types[nd.type]
            deadline = now + tau
            if name in self.outstanding or (fresh_until and (fresh_until(name) or -1) >= deadline):
                del self.pending[name]
                self.stats.deduplicated += 1
                continue
            theta = p.rtt_est + dt.size_kb * 8.0 / p.rate_est_kbps
            release = now + max(0.0, tau - theta - p.beta * dt.freshness_s)
            if release > now:
                self.stats.deferred += 1
                continue
            prio = nd.confidence * dt.criticality / (1.0 + tau / p.tau0)
            life = min(p.lifetime_max, max(p.lifetime_min, tau + p.delta))
            ready.append(NamedInterest(name, nd.zone, nd.type, nd.qualifier, prio, deadline, release, life,
                                       nd.source, dt.size_kb, late_risk=tau < theta))
        ready.sort(key=lambda i: (-i.priority, i.deadline))
        out = []
        for it in ready:
            if self.tokens < 1.0:
                self.stats.rate_limited += 1
                continue
            self.tokens -= 1.0
            del self.pending[it.name]
            self.outstanding[it.name] = now + it.lifetime_s
            self.stats.issued += 1
            self.stats.by_source[it.source] = self.stats.by_source.get(it.source, 0) + 1
            out.append(it)
        return out

    # ------------------------------------------------------------------ helpers
    def _refill(self, now):
        if self.last_t is not None:
            self.tokens = min(float(self.p.burst), self.tokens + (now - self.last_t) * self.p.rate_per_s)
        self.last_t = now

    def _add_layer(self, now, zone, s_in_zone, route, types, k, source, conf, fresh_until=None):
        if not route or k <= 0:
            return
        ahead = self.world.route_ahead(route, zone, s_in_zone, horizon_m=math.inf)[:k + 1]
        needs = []
        for zid, _ in ahead:
            for t in types:
                dt = self.types.get(t)
                if dt is None:
                    continue
                if dt.qualified:
                    if self.world.zones[zid].kind != 'intersection':
                        continue
                    for q in SIGNAL_MOVEMENTS:
                        needs.append({'type': t, 'zone': zid, 'qualifier': q, 'confidence': conf})
                else:
                    needs.append({'type': t, 'zone': zid, 'qualifier': None, 'confidence': conf})
        for n in needs:
            name = make_name(n['zone'], n['type'], n['qualifier'])
            if name in self.pending or name in self.outstanding:
                continue
            if fresh_until and (fresh_until(name) or -1) > now:
                continue
            self.pending[name] = _Need(n['zone'], n['type'], n['qualifier'], conf, source, now)
