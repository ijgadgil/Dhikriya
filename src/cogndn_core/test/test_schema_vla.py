import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

from cogndn_core.schema import json_schema, parse_state
from cogndn_core.vla import REQUESTABLE, OllamaVLA, build_prompt


def test_schema_enums():
    s = json_schema(['m2', 'm3'], ['hazard'])
    need = s['$defs']['InformationNeed']['properties']
    assert need['zone']['enum'] == ['m2', 'm3'] and need['type']['enum'] == ['hazard']
    assert 'stop' in s['properties']['action']['enum']


def test_parse_drops_unknown():
    raw = json.dumps({'action': 'slow_down', 'rationale': 'cattle sign', 'needs': [
        {'type': 'hazard', 'zone': 'm3', 'confidence': 0.9},
        {'type': 'hazard', 'zone': 'zz', 'confidence': 0.9}]})
    st, dropped, err = parse_state(raw, {'m3'}, {'hazard'})
    assert st.action == 'slow_down' and len(st.needs) == 1 and dropped == 1 and err == ''
    st, _, err = parse_state('{"action": "fly"}')
    assert st is None and err


def test_prompt_contains_context():
    p = build_prompt('go straight', 'm1', [('m1', 0), ('x1', 80)], {'x1': {'straight': 'e0'}},
                     [{'class': 'cow', 'confidence': 0.8}], ['[/cogndn/town/m3/hazard] livestock'])
    assert 'x1 at 80 m, intersection' in p and 'cow 0.80' in p and 'livestock' in p
    for k in REQUESTABLE:
        assert k in p


class _Fake(BaseHTTPRequestHandler):
    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        assert body['format']['properties']['action']
        assert body['options']['temperature'] == 0
        out = {'message': {'content': json.dumps({'action': 'stop', 'rationale': 'road closed',
                                                  'needs': [{'type': 'detour', 'zone': 'x1', 'confidence': 1.0}]})}}
        data = json.dumps(out).encode()
        self.send_response(200)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *a):
        pass


def test_ollama_client_roundtrip():
    srv = HTTPServer(('127.0.0.1', 0), _Fake)
    th = threading.Thread(target=srv.handle_request, daemon=True)
    th.start()
    vla = OllamaVLA(url='http://127.0.0.1:%d' % srv.server_port, timeout=5)
    r = vla.infer('prompt', b'\xff\xd8jpeg', ['x1', 'n0'])
    assert r['error'] == '' and r['state'].action == 'stop' and r['state'].needs[0].type == 'detour'
    srv.server_close()


def test_ollama_client_reports_errors():
    r = OllamaVLA(url='http://127.0.0.1:1', timeout=0.5).infer('p', None, ['m1'])
    assert r['state'] is None and r['error']


def _ahead(world, zone, s_in, route):
    return world.route_ahead(route, zone, s_in, 120.0)


def test_sign_head_maps_each_scenario_sign_to_its_required_data(world):
    from cogndn_core.vla import needs_from_text
    inter = {z for z, zz in world.zones.items() if zz.kind == 'intersection'}
    straight = ['m0', 'm1', 'm2', 'm3', 'm4', 'x1', 'e0', 'e1', 'e2', 'e3', 'e4']

    def names(texts, zone, s_in, route=straight):
        return {(n['type'], n['zone']) for n in needs_from_text(texts, _ahead(world, zone, s_in, route), inter)}
    assert {('workzone', 'm3'), ('workzone', 'm4')} <= names(['ROAD WORK', 'RIGHT LANE CLOSED'], 'm1', 4.0)
    assert ('hazard', 'm3') in names(['CATTLE CROSSING AHEAD'], 'm1', 10.0)
    assert ('crosswalk', 'e2') in names(['SCHOOL ZONE', 'PEDESTRIAN CROSSING'], 'm3', 10.0)
    got = names(['ROAD CLOSED AHEAD', 'DETOUR LEFT'], 'm2', 0.0)
    assert ('detour', 'x1') in got and ('hazard', 'e0') in got
    assert not any(t == 'crosswalk' for t, _ in names(['CATTLE CROSSING'], 'm1', 0.0))
    assert names([], 'm1', 0.0) == set()


def test_schema_leaves_sign_transcription_to_ocr():
    from cogndn_core.schema import json_schema, parse_state
    s = json_schema(['m1'], ['hazard'])
    assert 'signs' not in s['properties'] and list(s['properties'])[0] == 'action'
    st, _, _ = parse_state('{"action": "keep_lane", "needs": [], "rationale": ""}', {'m1'}, {'hazard'})
    assert st is not None and st.signs == []


def test_sign_head_tolerates_ocr_merged_words(world):
    from cogndn_core.vla import needs_from_text
    inter = {z for z, zz in world.zones.items() if zz.kind == 'intersection'}
    route = ['m0', 'm1', 'm2', 'm3', 'm4', 'x1', 'e0', 'e1', 'e2', 'e3', 'e4']
    ahead = world.route_ahead(route, 'm2', 0.0, 120.0)
    got = {(n['type'], n['zone']) for n in needs_from_text(['ROADCLOSED AHEAD DETOURLEFT'], ahead, inter)}
    assert ('detour', 'x1') in got


def test_camera_ocr_reads_a_sign():
    import pytest
    from cogndn_core import signocr
    if not signocr.AVAILABLE:
        pytest.skip('tesseract not installed')
    assert 'ROAD' in signocr._selftest()
