"""Named context store (verified NDN Data) and C2 context selection.

Two interchangeable back-ends: MemoryContextStore (tests, harness) and RedisContextStore
(runtime, shared between ROS processes). Items expire when their freshness period ends.

C2 selection: among fresh items, each item i on zone z_i with type k_i gets
  sigma_i = kappa_k / (1 + d_i / L0)                                              (C2-1)
where d_i is the road distance to z_i; the top-k items are rendered into the VLA prompt.
"""
import hashlib
import json
from dataclasses import asdict, dataclass

from .catalogue import DEFAULT_TYPES
from .naming import parse_name


def digest(content_bytes):
    return hashlib.sha256(content_bytes).hexdigest()


@dataclass
class ContextItem:
    name: str
    content: dict
    digest: str
    signer: str
    sig_ok: bool
    received_at: float
    fresh_until: float

    @property
    def zone(self):
        return parse_name(self.name)['zone']

    @property
    def type(self):
        return parse_name(self.name)['type']


class MemoryContextStore:
    def __init__(self):
        self._d = {}

    def put(self, item):
        self._d[item.name] = item

    def get(self, name):
        return self._d.get(name)

    def fresh_until(self, name):
        it = self._d.get(name)
        return it.fresh_until if it else None

    def fresh_items(self, now):
        return [i for i in self._d.values() if i.fresh_until > now]

    def tamper(self, name, new_content):
        """Test helper: replace content without updating the digest."""
        self._d[name].content = new_content


class RedisContextStore:
    """Redis layout: hash ctx:<name>; sorted set zidx:<zone> (score = fresh_until);
    channel ctx_updates carries the name of every new item."""

    def __init__(self, client, grace_s=5.0, channel='ctx_updates'):
        self.r = client
        self.grace = grace_s
        self.channel = channel

    def put(self, item, now=None):
        key = 'ctx:' + item.name
        d = asdict(item)
        d['content'] = json.dumps(item.content, sort_keys=True)
        d['sig_ok'] = int(item.sig_ok)
        ttl_ms = int(max(1.0, item.fresh_until - (now if now is not None else item.received_at) + self.grace) * 1000)
        pipe = self.r.pipeline()
        pipe.hset(key, mapping={k: str(v) for k, v in d.items()})
        pipe.pexpire(key, ttl_ms)
        pipe.zadd('zidx:' + item.zone, {item.name: item.fresh_until})
        pipe.publish(self.channel, item.name)
        pipe.execute()

    def get(self, name):
        h = self.r.hgetall('ctx:' + name)
        if not h:
            return None
        h = {(k.decode() if isinstance(k, bytes) else k): (v.decode() if isinstance(v, bytes) else v)
             for k, v in h.items()}
        return ContextItem(h['name'], json.loads(h['content']), h['digest'], h['signer'], h['sig_ok'] == '1',
                           float(h['received_at']), float(h['fresh_until']))

    def fresh_until(self, name):
        v = self.r.hget('ctx:' + name, 'fresh_until')
        return float(v) if v is not None else None

    def fresh_items(self, now, zones=None):
        out = []
        keys = ['zidx:' + z for z in zones] if zones else [k.decode() if isinstance(k, bytes) else k
                                                           for k in self.r.scan_iter('zidx:*')]
        for k in keys:
            for n in self.r.zrangebyscore(k, now, '+inf'):
                it = self.get(n.decode() if isinstance(n, bytes) else n)
                if it:
                    out.append(it)
        return out

    def tamper(self, name, new_content):
        self.r.hset('ctx:' + name, 'content', json.dumps(new_content, sort_keys=True))


def select_context(items, distance_of_zone, k=4, L0=40.0, types=None):
    """C2: rank fresh items by sigma_i and return the top k (item, sigma)."""
    types = types or DEFAULT_TYPES
    scored = []
    for it in items:
        d = distance_of_zone(it.zone)
        if d is None or d == float('inf') or not it.sig_ok:
            continue
        kap = types[it.type].criticality if it.type in types else 0.3
        scored.append((it, kap / (1.0 + d / L0)))
    scored.sort(key=lambda p: -p[1])
    return scored[:k]


def render_context(selected, now):
    """Compact prompt lines, one per selected item."""
    lines = []
    for it, _ in selected:
        summary = it.content.get('summary') or json.dumps(it.content)[:120]
        lines.append('[%s] %s (signed by %s, verified, %.0f s old)' % (it.name, summary, it.signer,
                                                                        max(0.0, now - it.received_at)))
    return lines
