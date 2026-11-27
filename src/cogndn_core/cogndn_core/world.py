"""Road graph of the simulated town: zones, adjacency, projection and travel distance.

The town is described in YAML (cogndn_sim/config/town.yaml). Each road is a straight
polyline split into zones of fixed length; intersections are square zones whose
successors are labelled with a movement (straight / left / right).
"""
import heapq
import math
from dataclasses import dataclass, field

import yaml


@dataclass
class Zone:
    id: str
    road: str
    start: tuple
    end: tuple
    length: float
    kind: str = 'road'                       # 'road' or 'intersection'
    succ: dict = field(default_factory=dict)  # successor zone id -> movement label

    @property
    def heading(self):
        return math.atan2(self.end[1] - self.start[1], self.end[0] - self.start[0])

    def point_at(self, s, lateral=0.0):
        """World (x, y) at distance s along the zone, shifted `lateral` metres to the left."""
        t = 0.0 if self.length <= 0 else max(0.0, min(1.0, s / self.length))
        x = self.start[0] + t * (self.end[0] - self.start[0])
        y = self.start[1] + t * (self.end[1] - self.start[1])
        h = self.heading
        return (x - lateral * math.sin(h), y + lateral * math.cos(h))


class World:
    def __init__(self, zones, lane_width=3.5):
        self.zones = {z.id: z for z in zones}
        self.lane_width = lane_width

    # ------------------------------------------------------------------ construction
    @classmethod
    def from_yaml(cls, path):
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f))

    @classmethod
    def from_dict(cls, cfg):
        zl = float(cfg.get('zone_length', 20.0))
        zones = []
        for r in cfg['roads']:
            (x0, y0), (x1, y1) = r['start'], r['end']
            total = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(round(total / zl)))
            prev = None
            for i in range(n):
                a, b = i / n, (i + 1) / n
                z = Zone('%s%d' % (r['prefix'], i), r['id'],
                         (x0 + a * (x1 - x0), y0 + a * (y1 - y0)),
                         (x0 + b * (x1 - x0), y0 + b * (y1 - y0)), total / n)
                if prev is not None:
                    prev.succ[z.id] = 'straight'
                zones.append(z)
                prev = z
        for it in cfg.get('intersections', []):
            c, half = it['center'], float(it.get('size', 10.0)) / 2.0
            z = Zone(it['id'], 'intersection', (c[0] - half, c[1]), (c[0] + half, c[1]), 2 * half,
                     kind='intersection')
            z.succ = {zid: mv for zid, mv in it['exits'].items()}
            zones.append(z)
        w = cls(zones, float(cfg.get('lane_width', 3.5)))
        for it in cfg.get('intersections', []):
            for zid in it['entries']:
                w.zones[zid].succ[it['id']] = 'straight'
        return w

    # ------------------------------------------------------------------ queries
    def locate(self, x, y):
        """Project a world position onto the nearest road zone. Returns (zone_id, s_along, lateral)."""
        best = None
        for z in self.zones.values():
            if z.kind == 'intersection':
                cx, cy = (z.start[0] + z.end[0]) / 2, (z.start[1] + z.end[1]) / 2
                if abs(x - cx) <= z.length / 2 and abs(y - cy) <= z.length / 2:
                    return z.id, z.length / 2, 0.0
                continue
            dx, dy = z.end[0] - z.start[0], z.end[1] - z.start[1]
            L2 = dx * dx + dy * dy
            t = max(0.0, min(1.0, ((x - z.start[0]) * dx + (y - z.start[1]) * dy) / L2))
            px, py = z.start[0] + t * dx, z.start[1] + t * dy
            d = math.hypot(x - px, y - py)
            lat = (-(x - z.start[0]) * dy + (y - z.start[1]) * dx) / math.sqrt(L2)
            if best is None or d < best[0]:
                best = (d, z.id, t * z.length, lat)
        return best[1], best[2], best[3]

    def distance_to(self, cur_zone, s_in_zone, target):
        """Shortest road distance (m) from the current position to the entry of `target`.
        Returns 0 if already inside target and math.inf if unreachable."""
        if target == cur_zone:
            return 0.0
        start = self.zones[cur_zone]
        dist = {}
        pq = [(max(0.0, start.length - s_in_zone), nid) for nid in start.succ]
        heapq.heapify(pq)
        while pq:
            d, zid = heapq.heappop(pq)
            if zid in dist:
                continue
            dist[zid] = d
            if zid == target:
                return d
            for nid in self.zones[zid].succ:
                if nid not in dist:
                    heapq.heappush(pq, (d + self.zones[zid].length, nid))
        return math.inf

    def route_ahead(self, route, cur_zone, s_in_zone, horizon_m):
        """Zones of `route` from the current one onward whose entry lies within horizon_m.
        Returns [(zone_id, distance_to_entry)]."""
        if cur_zone not in route:
            return [(cur_zone, 0.0)]
        out, d = [], -s_in_zone
        for zid in route[route.index(cur_zone):]:
            if d > horizon_m:
                break
            out.append((zid, max(0.0, d)))
            d += self.zones[zid].length
        return out

    def path_points(self, route, lane='R', step=1.0):
        """Dense reference path along `route` on the given lane ('R' right, 'L' left)."""
        lat = -self.lane_width / 2 if lane == 'R' else self.lane_width / 2
        pts = []
        for i, zid in enumerate(route):
            z = self.zones[zid]
            if z.kind == 'intersection' and 0 < i < len(route) - 1:
                a = self.zones[route[i - 1]].point_at(self.zones[route[i - 1]].length, lat)
                b = self.zones[route[i + 1]].point_at(0.0, lat)
                n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
                pts += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n)]
                continue
            n = max(1, int(z.length / step))
            pts += [z.point_at(z.length * k / n, lat) for k in range(n + 1)]
        return pts

    def movement(self, from_zone, to_zone):
        return self.zones[from_zone].succ.get(to_zone)
