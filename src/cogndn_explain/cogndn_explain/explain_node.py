"""C3 - evidence-linked explanations. One hash-chained ledger record per VLA decision, binding the
action and rationale to (a) recent detections, (b) the verified NDN Data names/digests that were
in the prompt, (c) envelope interventions; plus a grounding score and a provenance check."""
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cogndn_core.grounding import check_provenance, ground
from cogndn_core.ledger import Ledger
from cogndn_core.paths import run_dir
from cogndn_core.rosutil import JsonLog, redis_store
from cogndn_msgs.msg import CognitiveState, DetectionArray, EnvelopeEvent


class ExplainNode(Node):
    def __init__(self):
        super().__init__('explain_node')
        self.declare_parameter('window_s', 3.0)
        self.store, _ = redis_store(self)
        self.ledger = Ledger(os.path.join(run_dir(), 'ledger.db'))
        self.dets, self.events = [], []
        self.create_subscription(DetectionArray, '/cogndn/detections', self._dets, 10)
        self.create_subscription(EnvelopeEvent, '/cogndn/envelope_events', self._ev, 10)
        self.create_subscription(CognitiveState, '/cogndn/cognitive_state', self._state, 10)
        self.pub = self.create_publisher(String, '/cogndn/explanation', 10)
        self.log = JsonLog('explain')

    def _dets(self, m):
        now = time.time()
        w = self.get_parameter('window_s').value
        self.dets = [d for d in self.dets if now - d['t'] <= w] + [
            {'t': now, 'class': d.cls, 'confidence': d.confidence} for d in m.detections]

    def _ev(self, m):
        self.events.append({'t': time.time(), 'kind': m.kind, 'distance': m.distance, 'ttc': m.ttc})

    def _state(self, m):
        now = time.time()
        cited = [{'name': n, 'digest': d} for n, d in zip(m.context_names, m.context_digests)]
        items = [it for it in (self.store.get(c['name']) for c in cited) if it]
        g = ground(m.rationale, m.action, self.dets, items)
        prov = check_provenance(cited, self.store)
        rec = {'action': m.action, 'rationale': m.rationale, 'valid': m.valid, 'zone': m.zone,
               'position': [round(m.x, 2), round(m.y, 2)], 'model': m.model, 'latency_s': round(m.latency_s, 3),
               'needs': [{'type': n.type, 'zone': n.zone, 'qualifier': n.qualifier} for n in m.needs],
               'evidence': {'detections': sorted({d['class'] for d in self.dets}),
                            'data': [dict(c, signer=next((i.signer for i in items if i.name == c['name']), ''))
                                     for c in cited],
                            'envelope': self.events},
               'grounding': g, 'provenance': prov}
        seq, h = self.ledger.append(now, rec)
        self.ledger.commit()
        self.events = []
        self.pub.publish(String(data=json.dumps({'seq': seq, 'hash': h[:16], **rec})))
        self.log.write(seq=seq, hash=h, grounding=g['grounding'], claims=g['claims'], unsupported=g['unsupported'],
                       provenance=[p['status'] for p in prov], action=m.action)

    def destroy_node(self):
        ok, bad = self.ledger.verify()
        self.get_logger().info('ledger chain intact=%s first_bad=%s records=%d' % (ok, bad, self.ledger.seq))
        self.ledger.close()
        super().destroy_node()


def main():
    rclpy.init()
    n = ExplainNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()


if __name__ == '__main__':
    main()
