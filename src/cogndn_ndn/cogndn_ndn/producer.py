"""Signed producer for the road-side unit (runs in the RSU network namespace).

Serves every /cogndn/town/... name with scenario content, segmented and signed by the
authority allowed for its type. Objects are generated once per freshness period so all
segments of one version are consistent. --forge signs hazards with an unauthorised key
(robustness experiment). Every served object version is logged as JSON lines.

  python3 -m cogndn_ndn.producer --scenario s1_livestock --socket /run/cogndn/rsu.sock
"""
import argparse
import asyncio
import json
import logging
import os
import time

from ndn.app import NDNApp
from ndn.security import KeychainDigest
from ndn.transport.stream_face import UnixFace

from cogndn_core.catalogue import DEFAULT_TYPES
from cogndn_core.grounding import canonical
from cogndn_core.naming import PREFIX, AREA, parse_name
from cogndn_core.scenario import content_for

from .security import TrustPolicy
from .segments import make_segment, object_name, SEG_SIZE


class ObjectCache:
    def __init__(self, scenario_data, policy, forge_types=(), log=None, t0=None):
        self.data, self.policy, self.forge = scenario_data, policy, set(forge_types)
        self.objs = {}            # name -> (expires, bytes, signer)
        self.log = log
        self.t0 = t0 if t0 is not None else time.time()
        self.versions = 0

    def get(self, name, now=None):
        now = time.time() if now is None else now
        o = self.objs.get(name)
        if o and o[0] > now:
            return o
        dt = parse_name(name)['type']
        fresh = DEFAULT_TYPES[dt].freshness_s
        content = canonical(content_for(name, now - self.t0, self.data))
        signer, kn = self.policy.signer_for(dt, 'attacker' if dt in self.forge else None)
        o = (now + fresh, content, signer)
        self.objs[name] = o
        self.versions += 1
        if self.log:
            self.log.write(json.dumps({'t': now, 'name': name, 'bytes': len(content), 'signer': kn}) + '\n')
            self.log.flush()
        return o

    def segment(self, seg_name_str, k, now=None):
        name = object_name(seg_name_str)
        exp, content, signer = self.get(name, now)
        dt = parse_name(name)['type']
        return make_segment(name, content, k, DEFAULT_TYPES[dt].freshness_s, signer, SEG_SIZE)


def main():
    from ndn.encoding import Component, Name
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', required=True, help='scenario YAML path or id')
    ap.add_argument('--socket', default='/run/cogndn/rsu.sock')
    ap.add_argument('--keys', default=os.path.expanduser('~/.cogndn/keys'))
    ap.add_argument('--trust', default=None)
    ap.add_argument('--origin-delay-ms', type=float, default=0.0, help='extra producer-side delay (origin RTT)')
    ap.add_argument('--forge', default='', help='comma-separated types signed with the attacker key')
    ap.add_argument('--log', default='producer_log.jsonl')
    a = ap.parse_args()
    import yaml
    from .paths import scenario_path, trust_path
    sc = yaml.safe_load(open(scenario_path(a.scenario)))
    policy = TrustPolicy(a.trust or trust_path(), a.keys)
    cache = ObjectCache(sc.get('data', {}) or {}, policy, [t for t in a.forge.split(',') if t], open(a.log, 'a'))
    app = NDNApp(face=UnixFace(a.socket), keychain=KeychainDigest())
    served = {'n': 0}

    @app.route('%s/%s' % (PREFIX, AREA))
    def on_interest(name, param, app_param):
        k = Component.to_number(name[-1]) if Component.get_type(name[-1]) == Component.TYPE_SEGMENT else 0
        seg_name = Name.to_str(name)

        async def reply():
            if a.origin_delay_ms:
                await asyncio.sleep(a.origin_delay_ms / 1000.0)
            try:
                app.put_raw_packet(cache.segment(seg_name, k))
                served['n'] += 1
                if k == 0 and cache.log:      # one line per object request that reached the origin
                    cache.log.write(json.dumps({'t': time.time(), 'req': seg_name}) + '\n')
            except (ValueError, KeyError) as e:
                logging.warning('cannot serve %s: %s', seg_name, e)
        asyncio.ensure_future(reply())

    logging.basicConfig(level=logging.INFO)
    logging.info('producer for %s on %s (scenario %s)', PREFIX + '/' + AREA, a.socket, sc['id'])
    app.run_forever()


if __name__ == '__main__':
    main()
