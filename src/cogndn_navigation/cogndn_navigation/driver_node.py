"""Behaviour planner + pure-pursuit tracker. VLA actions change route, lane and target speed; the
result is a *nominal* command on /cogndn/cmd_nominal. Only the safety envelope drives the car.

Every policy replans to the scenario's true route when the closure becomes visible to the car's
own sensors (replan_cue), exactly as in the harness; VLA-driven runs may replan earlier."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import Header

from cogndn_core.naming import parse_name
from cogndn_core.navigation import BehaviourPlanner, PlannerParams, TrackerParams, pure_pursuit
from cogndn_core.rosutil import JsonLog, PoseTracker, load_scenario, redis_store
from cogndn_msgs.msg import CognitiveState, EnvelopeEvent, RouteState


class DriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')
        self.world, self.sc = load_scenario(self)
        for n, d in (('cruise', 3.0), ('slow', 1.5), ('follow_vla', True), ('rate_hz', 20.0), ('use_data', True)):
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.pl = BehaviourPlanner(self.world, self.sc.planned_route, PlannerParams(cruise=p('cruise'), slow=p('slow')))
        self.tp = TrackerParams()
        self.follow = p('follow_vla')
        self.pose = PoseTracker(self, self.world, self.sc.cfg['spawn'])
        self.create_subscription(CognitiveState, '/cogndn/cognitive_state', self._state, 10)
        self.use_data = p('use_data')
        self.store, _ = redis_store(self)
        self.blocked_since = None
        self.create_subscription(EnvelopeEvent, '/cogndn/envelope_events', self._env, 10)
        self.closed_mem = set()      # zones with a verified closed right lane, kept until the car has passed them
        self.cmd = self.create_publisher(Twist, '/cogndn/cmd_nominal', 10)
        self.rs = self.create_publisher(RouteState, '/cogndn/route_state', 10)
        self.log = JsonLog('driver')
        self.last, self.k = None, 0
        self.replanned_by_cue = False
        self.create_timer(1.0 / p('rate_hz'), self._tick)

    def _state(self, m):
        if self.follow and m.valid and self.pose.ready:
            before = list(self.pl.route)
            self.pl.on_action(time.time(), m.action, self.pose.zone, self.pose.s_in)
            if self.pl.route != before:
                self.log.write(kind='replan', by='vla', route=self.pl.route, x=self.pose.x, y=self.pose.y)

    def _env(self, m):
        if m.kind in ('stop', 'hold'):
            self.blocked_since = self.blocked_since or time.time()
        else:
            self.blocked_since = None

    def _data_rules(self, now):
        """Verified network data drives manoeuvres: work zones close the right lane, detours reroute."""
        closed, s = [], self._route_s()
        if self.use_data:
            try:
                items = self.store.fresh_items(now, zones=self.pl.route)
            except Exception:
                items = []
            for it in items:
                if not it.sig_ok:
                    continue
                n, c = parse_name(it.name), it.content or {}
                if n['type'] in ('workzone', 'hazard') and c.get('status') == 'active' \
                        and 'R' in c.get('lanes_closed', []):
                    closed.append(n['zone'])
                if n['type'] == 'detour' and c.get('status') == 'active' and \
                        self.pl.apply_detour(self.pose.zone, self.pose.s_in, n['zone'], c.get('via', [])):
                    self.log.write(kind='replan', by='data', route=self.pl.route, x=self.pose.x, y=self.pose.y)
        # Data can expire (hazard bulletins are fresh for 30 s) before the car reaches the zone; a closure that
        # was verified once stays in force until the zone is behind the car.
        self.closed_mem |= set(closed)
        here, s_in = self.pose.zone, self.pose.s_in
        self.closed_mem = {z for z in self.closed_mem
                           if z == here or math.isfinite(self.world.distance_to(here, s_in, z))}
        closed = sorted(self.closed_mem)
        blocked = 0.0 if self.blocked_since is None else now - self.blocked_since
        before = self.pl.lane
        why = self.pl.update_lane(self.pose.zone, self.pose.s_in, closed, blocked, s)
        if self.pl.lane != before:
            self.log.write(kind='lane', lane=self.pl.lane, by=why or 'clear', x=self.pose.x, y=self.pose.y,
                           closed=closed)

    def _tick(self):
        if not self.pose.ready:
            return
        now = time.time()
        dt = 0.05 if self.last is None else now - self.last
        self.last = now
        rc = self.sc.replan_cue
        if rc and not self.replanned_by_cue and self.pl.route != self.sc.truth_route:
            s = self._route_s()
            if s is not None and rc['s'] - rc['visible_m'] <= s:
                self.pl.route = list(self.sc.truth_route)
                self.replanned_by_cue = True
                self.log.write(kind='replan', by='perception_cue', route=self.pl.route, x=self.pose.x, y=self.pose.y)
        if self.k % 10 == 0:
            self._data_rules(now)
        path = self.pl.path(self.pose.zone)
        v, w = pure_pursuit(self.pose.x, self.pose.y, self.pose.yaw, path, self.pl.target_speed(now),
                            self.pose.v or 0.0, dt, self.tp)
        if self.pose.zone == self.pl.route[-1] and self.pose.s_in > self.world.zones[self.pose.zone].length - 3:
            v, w = 0.0, 0.0                          # end of route: stop
        self.cmd.publish(Twist(linear=Vector3(x=float(v)), angular=Vector3(z=float(w))))
        self.k += 1
        if self.k % 4 == 0:
            self.rs.publish(RouteState(header=Header(frame_id='world'), route=self.pl.route, lane=self.pl.lane,
                                       target_speed=float(self.pl.target_speed(now)), zone=self.pose.zone,
                                       s_in_zone=float(self.pose.s_in), replans=self.pl.replans))
            self.log.write(kind='pose', x=self.pose.x, y=self.pose.y, yaw=self.pose.yaw, v=self.pose.v,
                           zone=self.pose.zone, s=self._route_s(), v_cmd=v, lane=self.pl.lane)

    def _route_s(self):
        r = self.sc.truth_route
        if self.pose.zone not in r:
            return None
        return self.sc.route_s(self.pose.zone) + self.pose.s_in


def main():
    rclpy.init()
    rclpy.spin(DriverNode())


if __name__ == '__main__':
    main()
