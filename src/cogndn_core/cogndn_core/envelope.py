"""LiDAR time-to-collision safety envelope. It may only lower the commanded speed.

From the point cloud (sensor frame, x forward) keep points inside the driving corridor and
above the ground; d is the k-th smallest forward distance (robust to single stray returns).
  v_close = max(v_ego, -dd/dt)            closing speed (static obstacles: v_ego)       (E-1)
  TTC     = d / max(v_close, eps)                                                       (E-2)
  v_brake = sqrt(2 * a_brake * max(0, d - d_stop))   fastest speed that can still stop  (E-3)
  v_ttc   = v_nom * clip((TTC - T_stop) / (T_slow - T_stop), 0, 1)                      (E-4)
  v_out   = min(v_nom, v_brake, v_ttc); STOP (latched) if d <= d_stop or TTC <= T_stop.
The stop is released only after the corridor has been clear for t_clear seconds; stale LiDAR
(older than t_stale) forces a stop.
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class EnvelopeParams:
    yaw_deg: float = 0.0
    ground_z: float = -1.8
    min_h: float = 0.25
    max_h: float = 2.5
    half_width: float = 1.4
    x_min: float = 3.0       # own bonnet lies inside this distance
    x_max: float = 30.0
    min_points: int = 3
    d_stop: float = 6.5      # m; leaves room to steer around the obstacle afterwards
    t_stop: float = 1.2
    t_slow: float = 4.0
    a_brake: float = 3.0
    t_clear: float = 1.0
    t_stale: float = 0.5
    ema: float = 0.5
    jump_m: float = 3.0      # a larger change in d is a new object, not motion
    rate_max: float = 15.0   # m/s, plausibility cap on the closing-rate estimate
    max_offset: float = 4.0  # m, largest sideways bend of the corridor (about one lane)
    max_curvature: float = 0.21   # 1/m, the car's tightest turn (steer_max / wheel base)


def nearest_in_corridor(xyz, p, curvature=0.0):
    """Forward distance to the nearest raised return inside the corridor. The corridor follows the
    commanded path: its centre bends by y = curvature * x^2 / 2 (left positive), so an obstacle the car is
    steering around (e.g. barrels while merging left) is not treated as being in its way."""
    if xyz is None or len(xyz) == 0:
        return None
    c, s = math.cos(math.radians(p.yaw_deg)), math.sin(math.radians(p.yaw_deg))
    xf = xyz[:, 0] * c + xyz[:, 1] * s
    yl = -xyz[:, 0] * s + xyz[:, 1] * c
    h = xyz[:, 2] - p.ground_z
    yc = np.clip(0.5 * curvature * xf ** 2, -p.max_offset, p.max_offset)
    m = (xf >= p.x_min) & (xf <= p.x_max) & (np.abs(yl - yc) <= p.half_width) & (h >= p.min_h) & (h <= p.max_h)
    if int(m.sum()) < p.min_points:
        return None
    k = p.min_points - 1
    return float(np.partition(xf[m], k)[k])


class TTCEnvelope:
    def __init__(self, params=None):
        self.p = params or EnvelopeParams()
        self.d = None
        self.t_cloud = None
        self.d_prev = None
        self.rate = 0.0
        self.latched = False
        self.clear_since = None
        self.last_reason = ''
        self.xyz = None
        self.last_seen = None        # (time, distance) of the last return in the corridor
        self.travel_since_seen = 0.0
        self.t_apply = None

    def on_cloud(self, now, xyz):
        d = nearest_in_corridor(xyz, self.p)
        if (d is not None and self.d_prev is not None and self.t_cloud is not None and now > self.t_cloud
                and abs(self.d_prev - d) <= self.p.jump_m):
            r = max(-self.p.rate_max, min(self.p.rate_max, (self.d_prev - d) / (now - self.t_cloud)))
            self.rate = self.p.ema * r + (1 - self.p.ema) * self.rate
        elif d is not None and self.d_prev is not None and abs(self.d_prev - d) > self.p.jump_m:
            self.rate = 0.0
        elif d is None:
            self.rate = 0.0
        self.d_prev = d
        self.d = d
        self.xyz = xyz
        if d is not None:
            self.last_seen = (now, d)
            self.travel_since_seen = 0.0
        self.t_cloud = now

    def apply(self, now, v_nom, w_nom, v_ego):
        """Returns (v, w, event) where event is None or a dict describing an intervention."""
        p = self.p
        if self.t_apply is not None:
            self.travel_since_seen += abs(v_ego) * max(0.0, now - self.t_apply)
        self.t_apply = now
        if self.t_cloud is None or now - self.t_cloud > p.t_stale:
            return 0.0, 0.0, {'kind': 'stale', 'detail': 'no fresh LiDAR'}
        d = self.d
        kappa = 0.0 if abs(v_nom) < 0.05 else max(-p.max_curvature, min(p.max_curvature, w_nom / v_nom))
        if abs(kappa) > 0.02 and self.xyz is not None:     # steering: check the bent corridor instead
            d = nearest_in_corridor(self.xyz, p, kappa)
        if d is None:
            ttc = math.inf
        else:
            ttc = d / max(max(v_ego, self.rate), 1e-3)
        if d is not None and (d <= p.d_stop or ttc <= p.t_stop):
            self.latched, self.clear_since = True, None
            return 0.0, 0.0, {'kind': 'stop', 'distance': round(d, 2), 'ttc': round(min(ttc, 99.0), 2)}
        if self.latched:
            # An obstacle that was last seen just beyond x_min has not vanished: it is under the bonnet, out of view.
            # (Only if the car has moved since: a stopped car cannot drive into it, so then it really left.)
            if d is None and self.last_seen is not None and self.last_seen[1] < p.x_min + 1.5 \
                    and self.travel_since_seen > 0.3 and abs(kappa) <= 0.02:
                self.clear_since = None
                return 0.0, 0.0, {'kind': 'hold', 'distance': round(self.last_seen[1], 2)}
            if d is not None and d <= p.d_stop + 3.0:
                self.clear_since = None
                return 0.0, 0.0, {'kind': 'hold', 'distance': round(d, 2)}
            if self.clear_since is None:
                self.clear_since = now
            if now - self.clear_since < p.t_clear:
                return 0.0, 0.0, {'kind': 'clear_hold'}
            self.latched = False
        if d is None:
            return v_nom, w_nom, None
        v_brake = math.sqrt(2.0 * p.a_brake * max(0.0, d - p.d_stop))
        frac = max(0.0, min(1.0, (ttc - p.t_stop) / (p.t_slow - p.t_stop)))
        v = min(v_nom, v_brake, v_nom * frac)
        if v < v_nom - 1e-6:
            return v, w_nom, {'kind': 'slow', 'distance': round(d, 2), 'ttc': round(min(ttc, 99.0), 2),
                              'v_cap': round(v, 2)}
        return v_nom, w_nom, None
