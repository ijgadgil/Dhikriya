"""VLA pipeline on llava-phi3: prompt construction and the Ollama client.

llava-phi3 is a vision-language model; the pipeline becomes a VLA because the output is
constrained by a JSON schema to a Cognitive State whose `action` field is executed by the
behaviour planner (always under the LiDAR safety envelope).
"""
import base64
import json
import time
import urllib.request

from .schema import json_schema, parse_state

REQUESTABLE = {
    'hazard': 'reports of animals, accidents or obstacles in a zone',
    'signal': 'traffic-light phase for one movement (straight/left/right) at an intersection',
    'workzone': 'lane closures and road works in a zone',
    'detour': 'alternative-route map when a road is closed',
    'crosswalk': 'pedestrian presence at a crossing',
    'scene': 'road-side camera summary of a zone',
}

SYSTEM = ('You are the driving cognition of an autonomous car. Decide one action, list at most four pieces of '
          'information you will need from the network within the next minute that you do not already have, and '
          'give a one-sentence rationale that mentions only what you can see in the image, the sign text read by '
          'the camera, the detected objects, or the verified network context. Map tiles are fetched '
          'automatically; never request them.')

# Sign-reading head: words the model read on signs (or wrote in its rationale) -> information needs.
# Keyword lists are fixed in advance and are the same for every scenario and map.
SIGN_RULES = (
    ('workzone', ('ROAD WORK', 'ROADWORK', 'WORK ZONE', 'CONSTRUCTION', 'LANE CLOSED', 'MEN AT WORK')),
    ('hazard', ('CATTLE', 'COW', 'LIVESTOCK', 'ANIMAL', 'ACCIDENT', 'HAZARD')),
    ('crosswalk', ('PEDESTRIAN', 'SCHOOL', 'CHILDREN', 'ZEBRA')),
    ('detour', ('DETOUR', 'ROAD CLOSED', 'DIVERSION')),
)


def needs_from_text(texts, ahead, intersections, horizon_m=100.0, confidence=0.7):
    """texts: strings read by the VLA; ahead: [(zone_id, distance_m)] along the route, nearest first;
    intersections: zone ids that are junctions. A warning sign refers to the road ahead, so each matched
    type is requested for the zones within `horizon_m`; a closure/detour sign asks for the detour at the
    next junction and for hazard reports beyond it."""
    words = ' '.join(t.upper() for t in texts if t)
    squeezed = words.replace(' ', '')     # OCR often drops spaces: ROADWORK, ROADCLOSED, DETOURLEFT
    out, seen = [], set()

    def add(t, z):
        if (t, z) not in seen:
            seen.add((t, z))
            out.append({'type': t, 'zone': z, 'qualifier': None, 'confidence': confidence})
    near = [z for z, d in ahead if d <= horizon_m]
    for t, keys in SIGN_RULES:
        if not any(k in words or k.replace(' ', '') in squeezed for k in keys):
            continue
        if t == 'detour':
            junctions = [z for z, _ in ahead if z in intersections]
            if junctions:
                add('detour', junctions[0])
                after = [z for z, _ in ahead][[z for z, _ in ahead].index(junctions[0]) + 1:][:1]
                for z in after:
                    add('hazard', z)
        else:
            for z in near:
                if t == 'workzone' and z in intersections:
                    continue
                add(t, z)
    return out


def build_prompt(instruction, zone, ahead, intersections, detections, context_lines, sign_text=''):
    """ahead: [(zone_id, distance_m)]; intersections: {zone_id: {movement: next_zone}}."""
    z_lines = []
    for zid, d in ahead:
        extra = ''
        if zid in intersections:
            extra = ', intersection; exits ' + ', '.join('%s->%s' % (m, n) for m, n in intersections[zid].items())
        z_lines.append('- %s at %.0f m%s' % (zid, d, extra))
    det = ', '.join('%s %.2f' % (d['class'], d['confidence']) for d in detections[:8]) or 'none'
    ctx = '\n'.join(context_lines) if context_lines else 'none'
    types = '\n'.join('- %s: %s' % (k, v) for k, v in REQUESTABLE.items())
    signs = sign_text or 'none'
    return ('%s\n\nRoute instruction: %s\nCurrent zone: %s\nZones ahead:\n%s\n\nDetected objects: %s\n'
            'Sign text read by the camera: %s\n\n'
            'Verified network context:\n%s\n\nInformation you may request:\n%s\n\nAnswer with JSON only.'
            % (SYSTEM, instruction, zone, '\n'.join(z_lines), det, signs, ctx, types))


class OllamaVLA:
    def __init__(self, model='llava-phi3', url='http://localhost:11434', timeout=30.0,
                 num_predict=320, num_ctx=2048, keep_alive='60m'):
        self.model, self.url, self.timeout = model, url.rstrip('/'), timeout
        self.options = {'temperature': 0, 'num_predict': num_predict, 'num_ctx': num_ctx}
        self.keep_alive = keep_alive

    def infer(self, prompt, image_jpeg, zone_ids, type_names=tuple(REQUESTABLE)):
        """Returns dict(state, latency_s, raw, dropped, error)."""
        payload = {
            'model': self.model, 'stream': False, 'keep_alive': self.keep_alive, 'options': self.options,
            'format': json_schema(zone_ids, type_names),
            'messages': [{'role': 'user', 'content': prompt,
                          'images': [base64.b64encode(image_jpeg).decode()] if image_jpeg else []}],
        }
        req = urllib.request.Request(self.url + '/api/chat', data=json.dumps(payload).encode(),
                                     headers={'Content-Type': 'application/json'})
        t0 = time.monotonic()
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                body = json.loads(r.read())
        except Exception as e:  # network / server errors are reported, never raised
            return {'state': None, 'latency_s': time.monotonic() - t0, 'raw': '', 'dropped': 0, 'error': str(e)}
        lat = time.monotonic() - t0
        raw = body.get('message', {}).get('content', '')
        st, dropped, err = parse_state(raw, set(zone_ids), set(type_names))
        return {'state': st, 'latency_s': lat, 'raw': raw, 'dropped': dropped, 'error': err}
