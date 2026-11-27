"""Encoding and decoding of Cognitive-NDN Data packets (content = canonical JSON)."""
import json

from ndn.encoding import MetaInfo, Name, make_data, parse_data

from cogndn_core.context import ContextItem, digest
from cogndn_core.grounding import canonical


def encode(name, content, freshness_s, signer):
    return make_data(name, MetaInfo(freshness_period=int(freshness_s * 1000)), canonical(content), signer=signer)


def decode(wire, policy, now):
    """Parse and verify a Data packet. Returns (ContextItem, reason) - sig_ok False when rejected."""
    name, meta, content, sig = parse_data(wire, with_tl=True)
    n = Name.to_str(name)
    ok, signer, why = policy.verify(n, sig)
    return to_item(n, meta, bytes(content or b''), signer, ok, now), why


def to_item(name, meta, content_bytes, signer, ok, now):
    try:
        c = json.loads(content_bytes.decode() or '{}')
    except ValueError:
        c, ok = {}, False
    fp = (meta.freshness_period or 0) / 1000.0
    return ContextItem(name, c, digest(content_bytes), signer, ok, now, now + fp)
