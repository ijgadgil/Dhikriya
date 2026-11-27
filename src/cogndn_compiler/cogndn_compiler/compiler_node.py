"""C1 - Cognitive Interest Compiler node.

policy:
  cognitive         route maps (next K zones) + VLA information needs        (proposed)
  cognitive_hybrid  route_typed layer + VLA information needs
  route_typed       map + hazard + signal for the next K zones, VLA ignored  (baseline)
  route_all         every data type for the next K zones, VLA ignored         (baseline)
  ondemand          only the current zone's tile, requested on entry          (baseline)
All policies share the same compiler, so differences come only from the information they use.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from cogndn_core.catalogue import DEFAULT_TYPES
from cogndn_core.compiler import CognitiveInterestCompiler, CompilerParams
from cogndn_core.rosutil import JsonLog, PoseTracker, load_scenario, redis_store
from cogndn_msgs.msg import CognitiveState, DataArrival, InterestBatch, NamedInterest, RouteState

LAYERS = {'cognitive': ('map',), 'cognitive_hybrid': ('map', 'hazard', 'signal'),
          'route_typed': ('map', 'hazard', 'signal'), 'route_all': tuple(DEFAULT_TYPES),
          # on demand: no prediction; every type is requested for a zone only once the car is in it
          'ondemand': tuple(DEFAULT_TYPES),
          # ablations of the proposed method use the same layers as 'cognitive'
          'cognitive_vlm': ('map',), 'cognitive_ocr': ('map',)}


class CompilerNode(Node):
    def __init__(self):
        super().__init__('compiler_node')
        self.world, self.sc = load_scenario(self)
        for n, d in (('policy', 'cognitive'), ('k', 3), ('rate_hz', 10.0), ('rate_est_kbps', 10000.0),
                     ('rtt_est', 0.08), ('beta', 0.5), ('tau0', 5.0), ('horizon_s', 60.0), ('interest_rate', 20.0),
                     ('burst', 10), ('silent_s', 15.0)):
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.policy = p('policy')
        cp = CompilerParams(rate_est_kbps=p('rate_est_kbps'), rtt_est=p('rtt_est'), beta=p('beta'), tau0=p('tau0'),
                            horizon_s=p('horizon_s'), rate_per_s=p('interest_rate'), burst=p('burst'),
                            silent_s=p('silent_s'), base_types=LAYERS[self.policy],
                            base_k=0 if self.policy == 'ondemand' else p('k'),
                            on_demand=self.policy == 'ondemand',
                            fallback_k=p('k') if self.policy.startswith('cognitive') else 0)
        self.comp = CognitiveInterestCompiler(self.world, DEFAULT_TYPES, cp)
        if not self.policy.startswith('cognitive'):
            self.comp.last_vla = float('-inf')
        self.store, _ = redis_store(self)
        self.pose = PoseTracker(self, self.world, self.sc.cfg['spawn'])
        self.route = list(self.sc.planned_route)
        self.create_subscription(CognitiveState, '/cogndn/cognitive_state', self._state, 10)
        self.create_subscription(RouteState, '/cogndn/route_state', lambda m: setattr(self, 'route', list(m.route)), 10)
        self.create_subscription(DataArrival, '/cogndn/data', lambda m: self.comp.on_data(m.name), 50)
        self.pub = self.create_publisher(InterestBatch, '/cogndn/interests', 10)
        self.log = JsonLog('compiler')
        self.create_timer(1.0 / p('rate_hz'), self._tick)
        self.get_logger().info('compiler policy=%s k=%d' % (self.policy, p('k')))

    def _state(self, m):
        # needs from the camera's sign reading are attached even when the model's own answer was invalid
        if not self.policy.startswith('cognitive') or (not m.valid and not m.needs):
            return
        needs = [{'type': n.type, 'zone': n.zone, 'qualifier': n.qualifier or None, 'confidence': n.confidence}
                 for n in m.needs]
        self.comp.add_needs(time.time(), needs, 'cognitive')
        self.log.write(kind='needs', needs=needs, action=m.action)

    def _tick(self):
        if not self.pose.ready:
            return
        now = time.time()
        out = self.comp.step(now, self.pose.zone, self.pose.s_in, self.pose.v or 0.0, self.route,
                             fresh_until=self.store.fresh_until)
        if not out:
            return
        b = InterestBatch(header=Header(frame_id='world'))
        b.header.stamp = self.get_clock().now().to_msg()
        for i in out:
            b.interests.append(NamedInterest(name=i.name, zone=i.zone, type=i.type, qualifier=i.qualifier or '',
                                             priority=float(i.priority), deadline=float(i.deadline),
                                             lifetime_s=float(i.lifetime_s), source=i.source,
                                             late_risk=bool(i.late_risk)))
            self.log.write(kind='interest', name=i.name, source=i.source, priority=i.priority, deadline=i.deadline,
                           lifetime_s=i.lifetime_s, late_risk=i.late_risk, zone_now=self.pose.zone, x=self.pose.x,
                           y=self.pose.y)
        self.pub.publish(b)


def main():
    rclpy.init()
    rclpy.spin(CompilerNode())


if __name__ == '__main__':
    main()
