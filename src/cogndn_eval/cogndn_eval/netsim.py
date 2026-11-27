"""Network model used by the headless harness: vehicle links, a shared road-side NDN cache with
Interest aggregation, and an origin producer. Calibrate its parameters from the NDNd + tc-netem
runs (cogndn_ndn/scripts/calibrate_link.py) before using it for sweeps.

Times in seconds, sizes in kB, rates in kbit/s. A Data object generated at time g stays fresh
until g + F (F = freshness period of its type) in every cache that holds it.
"""
import random
from dataclasses import dataclass


@dataclass
class NetParams:
    rate_kbps: float = 10000.0        # vehicle downlink
    one_way_s: float = 0.010          # vehicle <-> RSU propagation (tc netem delay)
    loss: float = 0.0                 # Interest/Data loss probability (one retransmission)
    rto_s: float = 0.2
    rsu_proc_s: float = 0.001
    origin_median_s: float = 0.060    # RSU -> origin round trip (lognormal)
    origin_sigma: float = 0.35
    origin_samples: tuple = ()        # optional empirical samples (s); overrides the lognormal
    rsu_cache: bool = True
    aggregation: bool = True


class Network:
    def __init__(self, params, types, seed=0):
        self.p = params
        self.types = types
        self.rng = random.Random(seed)
        self.rsu = {}            # name -> (gen_time, fresh_until, ready_time)
        self.pit = {}            # name -> (ready_time, gen_time)
        self.link_free = {}      # vehicle -> time the downlink becomes idle
        self.origin_requests = 0
        self.origin_kb = 0.0
        self.rsu_hits = 0
        self.rsu_requests = 0

    def _origin_delay(self):
        if self.p.origin_samples:
            return self.rng.choice(self.p.origin_samples)
        return self.p.origin_median_s * self.rng.lognormvariate(0.0, self.p.origin_sigma)

    def request(self, vehicle, name, dtype, t, lifetime_s=4.0):
        """Express an Interest at time t. Returns (arrival_time, gen_time) or None on timeout."""
        p = self.p
        size = self.types[dtype].size_kb
        fresh = self.types[dtype].freshness_s
        t_rsu = t + p.one_way_s
        self.rsu_requests += 1
        hit = self.rsu.get(name) if p.rsu_cache else None
        if hit and hit[2] <= t_rsu < hit[1]:
            ready, gen = t_rsu + p.rsu_proc_s, hit[0]
            self.rsu_hits += 1
        elif p.aggregation and name in self.pit and self.pit[name][0] >= t_rsu:
            ready, gen = self.pit[name]
        else:
            d = self._origin_delay()
            ready = t_rsu + d
            gen = t_rsu + d / 2.0
            self.origin_requests += 1
            self.origin_kb += size
            self.pit[name] = (ready, gen)
            if p.rsu_cache:
                self.rsu[name] = (gen, gen + fresh, ready)
        start = max(ready, self.link_free.get(vehicle, 0.0))
        end = start + size * 8.0 / p.rate_kbps
        self.link_free[vehicle] = end
        arrival = end + p.one_way_s
        if p.loss and self.rng.random() < p.loss:
            arrival += p.rto_s
        if arrival - t > lifetime_s:
            return None
        return arrival, gen
