"""Behaviour planner (VLA action -> route, lane, target speed) and pure-pursuit tracker."""
import math
from dataclasses import dataclass


@dataclass
class PlannerParams:
    cruise: float = 3.0       # m/s
    slow: float = 1.5         # m/s
    slow_hold_s: float = 8.0
    decision_margin_m: float = 5.0   # turn decisions are frozen this close to the intersection
    merge_lookahead_m: float = 25.0  # start merging this far before a zone whose right lane is closed
    block_wait_s: float = 4.0        # stopped this long by an obstacle in the lane -> pass it on the left
    pass_len_m: float = 40.0         # ... and stay left for this distance


class BehaviourPlanner:
    def __init__(self, world, route, params=None):
        self.w = world
        self.route = list(route)
        self.p = params or PlannerParams()
        self.lane = 'R'
        self.target = self.p.cruise
        self.slow_until = -1.0
        self.stopped = False
        self.replans = 0
        self.lane_by = 'vla'         # who chose the current lane: vla | data | blocked
        self.data_turn = {}          # junction -> movement required by verified detour data
        self.pass_until_s = None

    def branch_route(self, inter_id, movement):
        z = self.w.zones[inter_id]
        nxt = [zid for zid, mv in z.succ.items() if mv == movement]
        if not nxt:
            return None
        out, cur = [], nxt[0]
        while cur and cur not in out:
            out.append(cur)
            succ = [zid for zid, mv in self.w.zones[cur].succ.items() if mv == 'straight']
            cur = succ[0] if succ else None
        return out

    def on_action(self, now, action, zone, s_in_zone):
        if action == 'stop':
            self.stopped = True
        elif action != 'stop':
            self.stopped = False
        if action == 'slow_down':
            self.slow_until = now + self.p.slow_hold_s
        elif action in ('lane_change_left', 'lane_change_right'):
            self.lane = 'L' if action == 'lane_change_left' else 'R'
        elif action in ('turn_left', 'turn_right', 'go_straight'):
            mv = {'turn_left': 'left', 'turn_right': 'right', 'go_straight': 'straight'}[action]
            self._set_branch(zone, s_in_zone, mv, respect_data=True)

    def _set_branch(self, zone, s_in_zone, movement, respect_data=False):
        if zone not in self.route:
            return
        i = self.route.index(zone)
        for j in range(i, len(self.route)):
            zid = self.route[j]
            if self.w.zones[zid].kind != 'intersection':
                continue
            if respect_data and zid in self.data_turn and self.data_turn[zid] != movement:
                return                    # verified detour data outranks the model's own turn choice
            if self.w.distance_to(zone, s_in_zone, zid) < self.p.decision_margin_m:
                return
            br = self.branch_route(zid, movement)
            if br and self.route[j + 1:j + 1 + len(br)] != br:
                self.route = self.route[:j + 1] + br
                self.replans += 1
            return

    def update_lane(self, zone, s_in_zone, closed_right, blocked_s, route_s):
        """Lane from verified data and from being blocked; returns the reason ('data', 'blocked') or None.
        closed_right: zones whose right lane is closed according to verified work-zone data.
        blocked_s: how long the safety envelope has been holding the car stopped.
        route_s: distance travelled along the route (m)."""
        why = None
        for zid in closed_right:
            d = self.w.distance_to(zone, s_in_zone, zid)
            near = zid == zone or 0.0 <= d <= self.p.merge_lookahead_m
            # once merged for data, stay left until the closed zone is behind the car (no flapping)
            still_ahead = self.lane_by == 'data' and math.isfinite(d)
            if near or still_ahead:
                why = 'data'
                break
        if why is None and route_s is not None:
            if self.pass_until_s is not None and route_s < self.pass_until_s:
                why = 'blocked'
            elif blocked_s >= self.p.block_wait_s and self.lane == 'R' and self.w.zones[zone].kind != 'intersection':
                self.pass_until_s = route_s + self.p.pass_len_m
                why = 'blocked'
            else:
                self.pass_until_s = None
        if why:
            self.lane, self.lane_by = 'L', why
        elif self.lane_by in ('data', 'blocked'):
            self.lane, self.lane_by = 'R', 'vla'
        return why

    def apply_detour(self, zone, s_in_zone, junction, via):
        """Verified detour data: take the junction exit that leads to via[0]. Returns True if the route changed."""
        if junction not in self.route or not via:
            return False
        mv = self.w.movement(junction, via[0])
        before = list(self.route)
        if mv:
            self.data_turn[junction] = mv
            self._set_branch(zone, s_in_zone, mv)
        return self.route != before

    def target_speed(self, now):
        if self.stopped:
            return 0.0
        return self.p.slow if now < self.slow_until else self.p.cruise

    def path(self, zone):
        i = self.route.index(zone) if zone in self.route else 0
        return self.w.path_points(self.route[i:], lane=self.lane)


@dataclass
class TrackerParams:
    lookahead_min: float = 4.0
    lookahead_gain: float = 0.8
    wheel_base: float = 2.86
    steer_max: float = 0.6
    accel: float = 1.5
    recover_speed: float = 1.0   # m/s while turning back towards a target that is behind the car
    decel: float = 3.0


def pure_pursuit(x, y, yaw, path, v_target, v_now, dt, p=None):
    """Returns (v_cmd, yaw_rate_cmd). Speed follows v_target within accel/decel limits."""
    p = p or TrackerParams()
    if not path:
        return 0.0, 0.0
    dv = v_target - v_now
    lim = (p.accel if dv > 0 else p.decel) * max(dt, 1e-3)
    v = max(0.0, v_now + max(-lim, min(lim, dv)))
    if v_target > 0.0:
        v = min(v, v_target + 0.5)      # if the car is pushed or runs over speed, never keep commanding it
    i0 = min(range(len(path)), key=lambda i: (path[i][0] - x) ** 2 + (path[i][1] - y) ** 2)
    Ld = p.lookahead_min + p.lookahead_gain * v
    tgt = path[-1]
    for px, py in path[i0:]:
        if math.hypot(px - x, py - y) >= Ld:
            tgt = (px, py)
            break
    dx, dy = tgt[0] - x, tgt[1] - y
    lx = math.cos(yaw) * dx + math.sin(yaw) * dy
    ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
    alpha = math.atan2(ly, lx)
    kmax = math.tan(p.steer_max) / p.wheel_base
    if abs(alpha) > math.radians(100):
        # Target behind the car (heading flipped, e.g. after an interrupted lane change). Plain pure pursuit
        # gives sin(alpha) ~ 0 here and would drive straight on, away from the route; instead crawl and turn
        # fully towards the target until it is in front again.
        v = min(v, p.recover_speed)
        return v, v * math.copysign(kmax, alpha)
    kappa = 2.0 * math.sin(alpha) / max(Ld, 1e-3)
    kappa = max(-kmax, min(kmax, kappa))
    return v, v * kappa
