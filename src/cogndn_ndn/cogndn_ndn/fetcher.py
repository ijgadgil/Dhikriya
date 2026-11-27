"""Verifying segmented fetch with python-ndn (used by the consumer node and the fleet test).

fetch(app, name) expresses /name/seg=0 with MustBeFresh, learns the last segment from
FinalBlockId, fetches the rest with a window of `window` outstanding Interests, verifies every
segment against the trust policy and returns one ContextItem for the reassembled object."""
import asyncio
import json
import time

from ndn.encoding import Component, Name
from ndn.types import InterestNack, InterestTimeout, ValidationFailure

from cogndn_core.context import ContextItem, digest


_GLOBAL = {}


def _global_sem(limit):
    loop = asyncio.get_running_loop()
    key = (id(loop), limit)
    if key not in _GLOBAL:
        _GLOBAL[key] = asyncio.Semaphore(limit)
    return _GLOBAL[key]


async def fetch(app, name, policy, lifetime_ms=4000, window=8, retries=3, clock=time.time,
                rto_ms=1500, global_window=24):
    """lifetime_ms bounds the whole fetch (the compiler's prefetch lifetime); each segment Interest uses a
    short retransmission timeout rto_ms and is retried, so one lost packet costs ~rto_ms, not the lifetime.
    global_window caps segment Interests in flight across all concurrent fetches."""
    t0 = clock()
    seg_life = int(min(lifetime_ms, rto_ms))
    retries = max(retries, int(lifetime_ms // max(1, seg_life)) - 1) if lifetime_ms > seg_life else retries
    retries = min(retries, 6)
    gsem = _global_sem(global_window)
    stats = {'segments': 0, 'bytes': 0, 'retx': 0, 'rejected': 0}
    signer_seen = set()

    async def one(k):
        for attempt in range(retries + 1):
            verdict = {}

            async def validator(_name, sig_ptrs):
                verdict['ok'], verdict['signer'], verdict['why'] = policy.verify(name, sig_ptrs)
                return True          # accept on the wire; the item is flagged sig_ok=False if rejected

            try:
                async with gsem:
                    n, meta, content = await app.express_interest(
                        Name.from_str(name) + [Component.from_segment(k)], must_be_fresh=True,
                        can_be_prefix=False, lifetime=seg_life, validator=validator)
                ok, signer = verdict.get('ok', False), verdict.get('signer', '')
                if not ok:
                    stats['rejected'] += 1
                signer_seen.add(signer)
                stats['segments'] += 1
                stats['bytes'] += len(content or b'')
                last = Component.to_number(meta.final_block_id) if meta.final_block_id else k
                return bytes(content or b''), last, meta, ok
            except (InterestTimeout, InterestNack):
                stats['retx'] += 1
                if attempt == retries:
                    raise
            except ValidationFailure:
                stats['rejected'] += 1
                raise

    first, last, meta, ok0 = await one(0)
    parts = {0: first}
    oks = [ok0]
    sem = asyncio.Semaphore(window)

    async def guarded(k):
        async with sem:
            c, _, _, ok = await one(k)
            parts[k] = c
            oks.append(ok)
    if last > 0:
        await asyncio.gather(*(guarded(k) for k in range(1, last + 1)))
    blob = b''.join(parts[k] for k in range(last + 1))
    now = clock()
    try:
        content = json.loads(blob.decode())
    except ValueError:
        content, oks = {}, [False]
    fp = (meta.freshness_period or 0) / 1000.0
    item = ContextItem(name, content, digest(blob), sorted(signer_seen)[0] if signer_seen else '', all(oks), now,
                       now + fp)
    stats['latency_s'] = now - t0
    return item, stats
