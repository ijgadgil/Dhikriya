"""VLA on llava-phi3: camera + detections + verified NDN context -> Cognitive State.

Inference runs in a worker thread, back to back (min_interval_s apart), always on the newest
frame, so it never blocks perception, the compiler or the safety envelope. Each cycle is logged
(cognition.jsonl) with its capture position, latency, raw output and the context it was given;
these logs feed the paper's VLA statistics and the harness replay (make_replay).
"""
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from cogndn_core.context import render_context, select_context
from cogndn_core import signocr
from cogndn_core.rosutil import JsonLog, PoseTracker, load_scenario, redis_store
from cogndn_core.vla import REQUESTABLE, OllamaVLA, build_prompt, needs_from_text
from cogndn_msgs.msg import CognitiveState, DetectionArray, InformationNeed, RouteState


class VLANode(Node):
    def __init__(self):
        super().__init__('vla_node')
        self.world, self.sc = load_scenario(self)
        for n, d in (('model', 'llava-phi3'), ('ollama_url', 'http://localhost:11434'), ('min_interval_s', 0.0),
                     ('horizon_m', 120.0), ('sign_horizon_m', 100.0), ('context_k', 4), ('max_side', 672), ('jpeg_quality', 85),
                     ('num_predict', 320), ('use_context', True), ('timeout_s', 30.0),
                     ('ocr', True), ('ocr_hz', 3.0), ('sign_memory_s', 8.0),
                     ('use_model', True), ('ocr_only_period_s', 3.0)):
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.vla = OllamaVLA(p('model'), p('ollama_url'), timeout=p('timeout_s'), num_predict=p('num_predict'))
        self.model_name = p('model')
        self.store, _ = redis_store(self)
        self.bridge = CvBridge()
        self.pose = PoseTracker(self, self.world, self.sc.cfg['spawn'])
        self.img, self.dets, self.route = None, [], list(self.sc.planned_route)
        self.create_subscription(Image, '/vla/camera/image_raw', lambda m: setattr(self, 'img', m),
                                 qos_profile_sensor_data)
        self.create_subscription(DetectionArray, '/cogndn/detections', self._dets, 10)
        self.create_subscription(RouteState, '/cogndn/route_state', lambda m: setattr(self, 'route', list(m.route)), 10)
        self.pub = self.create_publisher(CognitiveState, '/cogndn/cognitive_state', 10)
        self.log = JsonLog('cognition')
        self.sign_mem = {}                   # sign text -> last time it was read
        if p('ocr') and signocr.AVAILABLE:
            threading.Thread(target=self._ocr_loop, daemon=True).start()
        elif p('ocr'):
            self.get_logger().warn('Tesseract not found: camera sign OCR disabled (sudo apt install tesseract-ocr)')
        threading.Thread(target=self._loop, daemon=True).start()
        self.get_logger().info('VLA pipeline on %s (scenario %s)' % (self.model_name, self.sc.id))

    def _ocr_loop(self):
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        while rclpy.ok():
            t0 = time.time()
            if self.img is not None:
                try:
                    txt = signocr.read_sign_text(self.bridge.imgmsg_to_cv2(self.img, desired_encoding='bgr8'))
                    if txt:
                        self.sign_mem[txt] = time.time()
                except Exception as e:  # OCR must never stop the VLA
                    self.get_logger().warn('OCR failed: %s' % e, throttle_duration_sec=10.0)
            time.sleep(max(0.0, 1.0 / p('ocr_hz') - (time.time() - t0)))

    def _recent_signs(self, memory_s):
        now = time.time()
        return [t for t, ts in list(self.sign_mem.items()) if now - ts <= memory_s]

    def _dets(self, m):
        self.dets = [{'class': d.cls, 'confidence': d.confidence} for d in m.detections]

    def _loop(self):
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        while rclpy.ok():
            if self.img is None or not self.pose.ready:
                time.sleep(0.1)
                continue
            t0 = time.time()
            self._cycle(p)
            time.sleep(max(0.0, p('min_interval_s') - (time.time() - t0)))

    def _cycle(self, p):
        x, y, zone, s_in = self.pose.x, self.pose.y, self.pose.zone, self.pose.s_in
        frame = self.bridge.imgmsg_to_cv2(self.img, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        k = p('max_side') / float(max(h, w))
        if k < 1.0:
            frame = cv2.resize(frame, (int(w * k), int(h * k)))
        jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, p('jpeg_quality')])[1].tobytes()
        ahead = self.world.route_ahead(self.route, zone, s_in, p('horizon_m'))
        if zone not in [z for z, _ in ahead]:
            ahead = [(zone, 0.0)] + ahead
        zone_ids = [z for z, _ in ahead]
        inters = {z: {mv: n for n, mv in self.world.zones[z].succ.items()} for z in zone_ids
                  if self.world.zones[z].kind == 'intersection'}
        sel = []
        if p('use_context'):
            now = time.time()
            items = self.store.fresh_items(now, zones=zone_ids)
            sel = select_context(items, lambda z: self.world.distance_to(zone, s_in, z), k=p('context_k'))
        lines = render_context(sel, time.time())
        ocr_texts = self._recent_signs(p('sign_memory_s'))
        best = max(ocr_texts, key=len) if ocr_texts else ''      # one reading for the prompt, not every variant
        prompt = build_prompt(self.sc.instruction, zone, ahead, inters, self.dets, lines, best)
        if p('use_model'):
            r = self.vla.infer(prompt, jpeg, zone_ids, tuple(REQUESTABLE))
        else:                          # ablation "OCR only": no vision-language model, the sign head alone
            time.sleep(p('ocr_only_period_s'))
            r = {'state': None, 'latency_s': 0.0, 'raw': '', 'dropped': 0, 'error': ''}
        st = r['state']
        msg = CognitiveState(header=Header(frame_id='world'), zone=zone, x=float(x), y=float(y),
                             latency_s=float(r['latency_s']), model=self.model_name, valid=st is not None)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.context_names = [it.name for it, _ in sel]
        msg.context_digests = [it.digest for it, _ in sel]
        signs, texts = [], list(ocr_texts)
        if st is not None:
            msg.action, msg.rationale = st.action, st.rationale
            msg.needs = [InformationNeed(type=n.type, zone=n.zone, qualifier=n.qualifier or '',
                                         confidence=float(n.confidence)) for n in st.needs]
            signs = list(st.signs)
            texts += signs + [st.rationale]
        else:
            msg.action = 'keep_lane'
        # the sign head runs on what the camera read even if the model's answer was invalid
        have = {(n.type, n.zone) for n in msg.needs}
        from_signs = [n for n in needs_from_text(texts, ahead, set(inters), p('sign_horizon_m'))
                      if (n['type'], n['zone']) not in have]
        msg.needs += [InformationNeed(type=n['type'], zone=n['zone'], qualifier='', confidence=n['confidence'])
                      for n in from_signs]
        self.pub.publish(msg)
        self.log.write(x=x, y=y, zone=zone, s_in=s_in, latency_s=r['latency_s'], valid=st is not None,
                       action=msg.action, rationale=msg.rationale, dropped=r['dropped'], error=r['error'],
                       needs=[{'type': n.type, 'zone': n.zone, 'qualifier': n.qualifier or None,
                               'confidence': n.confidence} for n in msg.needs],
                       context=list(msg.context_names), detections=self.dets, raw=r['raw'][:600],
                       signs=signs, ocr=ocr_texts, needs_from_signs=len(from_signs))
        if r['error'] and not r['raw']:
            time.sleep(2.0)       # Ollama unreachable (e.g. restarting): back off instead of spinning


def main():
    rclpy.init()
    rclpy.spin(VLANode())


if __name__ == '__main__':
    main()
