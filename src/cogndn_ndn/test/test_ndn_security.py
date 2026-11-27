import os
import sys

HERE = os.path.dirname(__file__)
sys.path[:0] = [os.path.join(HERE, '..'), os.path.join(HERE, '..', '..', 'cogndn_core')]

from cogndn_core.grounding import canonical  # noqa: E402
from cogndn_core.context import digest  # noqa: E402
from cogndn_core.scenario import content_for  # noqa: E402
from cogndn_ndn.security import TrustPolicy, make_keys  # noqa: E402
from cogndn_ndn.wire import decode, encode  # noqa: E402

TRUST = os.path.join(HERE, '..', 'config', 'trust.yaml')


def setup(tmp_path):
    make_keys(str(tmp_path), TRUST)
    return TrustPolicy(TRUST, str(tmp_path))


def test_sign_and_verify(tmp_path):
    pol = setup(tmp_path)
    name = '/cogndn/town/m3/hazard'
    c = content_for(name, 0.0, {name: {'category': 'animal', 'status': 'active'}})
    signer, kn = pol.signer_for('hazard')
    item, why = decode(encode(name, c, 30.0, signer), pol, now=10.0)
    assert item.sig_ok and why == '' and item.signer == kn
    assert item.content['category'] == 'animal' and item.fresh_until == 40.0
    assert item.digest == digest(canonical(c))


def test_wrong_authority_rejected(tmp_path):
    pol = setup(tmp_path)
    signer, _ = pol.signer_for('signal')             # intersection key signing a hazard
    item, why = decode(encode('/cogndn/town/m3/hazard', {'status': 'clear'}, 30, signer), pol, 0.0)
    assert not item.sig_ok and 'not allowed' in why


def test_forged_key_rejected(tmp_path):
    pol = setup(tmp_path)
    signer, _ = pol.signer_for('hazard', role_override='attacker')
    item, why = decode(encode('/cogndn/town/m3/hazard', {'status': 'clear'}, 30, signer), pol, 0.0)
    assert not item.sig_ok


def test_bit_flip_rejected(tmp_path):
    pol = setup(tmp_path)
    signer, _ = pol.signer_for('hazard')
    wire = bytearray(encode('/cogndn/town/m3/hazard', {'status': 'active', 'category': 'animal'}, 30, signer))
    i = bytes(wire).find(b'animal')
    wire[i] ^= 0x01
    item, why = decode(bytes(wire), pol, 0.0)
    assert not item.sig_ok and why == 'bad signature'


def test_map_tile_size(tmp_path):
    pol = setup(tmp_path)
    signer, _ = pol.signer_for('map')
    wire = encode('/cogndn/town/m1/map', content_for('/cogndn/town/m1/map', 0.0), 3600, signer)
    assert 199000 < len(wire) < 202000


def test_segmentation_roundtrip(tmp_path):
    from cogndn_ndn.producer import ObjectCache
    from cogndn_ndn.segments import parse_segment
    pol = setup(tmp_path)
    cache = ObjectCache({}, pol, t0=0.0)
    name = '/cogndn/town/m1/map'
    seg0 = cache.segment(name + '/seg=0', 0, now=1.0)
    n, k, last, meta, c0, sig = parse_segment(seg0)
    assert n == name and k == 0 and last == 28 and len(seg0) < 8800
    parts = [c0] + [parse_segment(cache.segment(name + '/seg=%d' % i, i, now=2.0))[4] for i in range(1, last + 1)]
    blob = b''.join(parts)
    assert digest(blob) == digest(cache.get(name, now=3.0)[1])        # one consistent version
    assert pol.verify(name, sig)[0]
    assert cache.versions == 1


def test_forge_option_uses_attacker_key(tmp_path):
    from cogndn_ndn.producer import ObjectCache
    from cogndn_ndn.segments import parse_segment
    pol = setup(tmp_path)
    cache = ObjectCache({}, pol, forge_types=['hazard'], t0=0.0)
    sig = parse_segment(cache.segment('/cogndn/town/m3/hazard/seg=0', 0, now=0.0))[5]
    assert not pol.verify('/cogndn/town/m3/hazard', sig)[0]


class FakeApp:
    """Stands in for NDNApp: answers Interests from an ObjectCache, calling the validator like python-ndn."""

    def __init__(self, cache, drop=()):
        self.cache, self.drop, self.sent = cache, set(drop), 0

    async def express_interest(self, name, must_be_fresh=True, can_be_prefix=False, lifetime=4000, validator=None):
        from ndn.encoding import Name
        from ndn.types import InterestTimeout
        from cogndn_ndn.segments import parse_segment
        self.sent += 1
        s = Name.to_str(name)
        k = int(s.rsplit('seg=', 1)[1])
        if (s, self.sent) in self.drop or s in self.drop:
            self.drop.discard(s)
            raise InterestTimeout()
        n, k2, last, meta, content, sig = parse_segment(self.cache.segment(s, k, now=0.0))
        await validator(name, sig)
        return name, meta, content


def test_fetcher_reassembles_and_verifies(tmp_path):
    import asyncio
    from cogndn_ndn.fetcher import fetch
    from cogndn_ndn.producer import ObjectCache
    pol = setup(tmp_path)
    cache = ObjectCache({}, pol, t0=0.0)
    app = FakeApp(cache, drop={'/cogndn/town/m1/map/seg=5'})
    item, st = asyncio.run(fetch(app, '/cogndn/town/m1/map', pol, clock=lambda: 100.0))
    assert item.sig_ok and st['segments'] == 29 and st['retx'] == 1
    assert item.content['zone'] == 'm1' and item.fresh_until == 3700.0
    forged = ObjectCache({}, pol, forge_types=['hazard'], t0=0.0)
    item, st = asyncio.run(fetch(FakeApp(forged), '/cogndn/town/m3/hazard', pol, clock=lambda: 0.0))
    assert not item.sig_ok and st['rejected'] == 1
