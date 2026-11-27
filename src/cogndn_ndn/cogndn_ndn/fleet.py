"""Real-NDN measurement tool (runs against the forwarders started by scripts/ndn_up.sh).

  --mode smoke      fetch a hazard bulletin and a map tile twice through the vehicle forwarder
  --mode calibrate  measure miss / vehicle-cache hit / RSU-cache hit times per data type and print
                    NetParams for the harness (cogndn_eval.netsim)
  --mode fleet      N vehicles (processes, each a TCP face to the RSU over the netem link) replay an
                    Interest schedule exported from the harness, headway apart; reports latency,
                    verification and the number of object requests that reached the origin
"""
import argparse
import asyncio
import json
import multiprocessing as mp
import os
import statistics
import time

from ndn.app import NDNApp
from ndn.security import KeychainDigest
from ndn.transport.stream_face import TcpFace, UnixFace

from cogndn_core.catalogue import DEFAULT_TYPES

from .fetcher import fetch
from .paths import trust_path
from .security import TrustPolicy


def _app(face):
    return NDNApp(face=face, keychain=KeychainDigest())


def run_with(app, coro_fn):
    out = {}

    async def go():
        try:
            out['r'] = await coro_fn(app)
        finally:
            app.shutdown()
    app.run_forever(after_start=go())
    return out.get('r')


def smoke(a, policy):
    async def body(app):
        rows = []
        for name in ('/cogndn/town/m3/hazard', '/cogndn/town/m1/map', '/cogndn/town/m3/hazard', '/cogndn/town/m1/map'):
            item, st = await fetch(app, name, policy)
            rows.append((name, round(st['latency_s'] * 1000, 1), st['segments'], item.sig_ok, item.signer))
        return rows
    for r in run_with(_app(UnixFace(a.socket)), body):
        print('%-26s %7.1f ms  segments=%-3d verified=%s signer=%s' % r)
    print('second fetch of each name should be a vehicle-cache hit (much faster)')


def calibrate(a, policy):
    types = ['hazard', 'signal', 'workzone', 'crosswalk', 'map', 'detour']
    zones = ['m0', 'm1', 'm2', 'm3', 'm4', 'e0', 'e1', 'e2', 'e3', 'e4', 'n0', 'n1', 'n2', 'n3', 'n4']
    res = {}

    async def via_vehicle(app):
        for t in types:
            # signal is qualified by movement at junction x1: three distinct names, so every first fetch is a real miss
            names = (['/cogndn/town/x1/signal/%s' % q for q in ('straight', 'left', 'right')] if t.startswith('signal')
                     else ['/cogndn/town/%s/%s' % (z, t) for z in zones])
            for name in names:
                i1, s1 = await fetch(app, name, policy)
                i2, s2 = await fetch(app, name, policy)
                res.setdefault(t, {'miss': [], 'veh_hit': [], 'rsu_hit': []})
                res[t]['miss'].append(s1['latency_s'])
                res[t]['veh_hit'].append(s2['latency_s'])
                res[t]['names'] = res[t].get('names', []) + [name]
                await asyncio.sleep(0.05)
        return res

    async def via_rsu(app):
        # Short-lived types (signal 5 s, crosswalk 3 s) have expired in the RSU cache by now, so fetch once to
        # refill it and time the second fetch; otherwise an "RSU hit" would really be a trip to the producer.
        for t, d in res.items():
            short = DEFAULT_TYPES[t].freshness_s < 30
            for name in d['names']:
                if short:
                    await fetch(app, name, policy)
                _, s = await fetch(app, name, policy)
                d['rsu_hit'].append(s['latency_s'])
        return res
    run_with(_app(UnixFace(a.socket)), via_vehicle)
    run_with(_app(TcpFace(a.rsu_host, 6363)), via_rsu)
    print('%-16s %10s %10s %10s  (median ms)' % ('type', 'miss', 'veh_hit', 'rsu_hit'))
    med = {}
    for t, d in res.items():
        med[t] = {k: 1000 * statistics.median(d[k]) for k in ('miss', 'veh_hit', 'rsu_hit')}
        print('%-16s %10.1f %10.1f %10.1f' % (t, med[t]['miss'], med[t]['veh_hit'], med[t]['rsu_hit']))
    small = med['hazard']
    one_way = max(0.0, (small['rsu_hit'] - small['veh_hit']) / 2000.0)
    origin = max(0.0, (small['miss'] - small['rsu_hit']) / 1000.0)
    big = med['map']
    xfer = max(1e-3, (big['rsu_hit'] - small['rsu_hit']) / 1000.0)
    rate = 200 * 8 / xfer
    params = {'one_way_s': round(one_way, 4), 'origin_median_s': round(origin, 4), 'rate_kbps': round(rate)}
    print('\nSuggested NetParams for the harness:', json.dumps(params))
    if a.out:
        json.dump({'medians_ms': med, 'netparams': params}, open(a.out, 'w'), indent=1)


def _vehicle(args):
    vid, sched, host, delay, tpath, kdir = args
    policy = TrustPolicy(tpath, kdir)

    async def body(app):
        await asyncio.sleep(delay)
        t0 = time.monotonic()
        out = []

        async def one(t, name):
            await asyncio.sleep(max(0.0, t - (time.monotonic() - t0)))
            try:
                item, st = await fetch(app, name, policy)
                out.append({'vid': vid, 'name': name, 't': t, 'lat_ms': 1000 * st['latency_s'], 'ok': item.sig_ok})
            except Exception as e:  # timeouts are results, not crashes
                out.append({'vid': vid, 'name': name, 't': t, 'lat_ms': None, 'ok': False, 'err': type(e).__name__})
        await asyncio.gather(*(one(t, n) for t, n in sched))
        return out
    return run_with(_app(TcpFace(host, 6363)), body) or []


def fleet(a, policy):
    sched = json.load(open(a.schedule))
    jobs = [(i, sched, a.rsu_host, i * a.headway, trust_path(), a.keys) for i in range(a.vehicles)]
    t0 = time.time()
    with mp.Pool(a.vehicles) as p:
        rows = [r for rs in p.map(_vehicle, jobs) for r in rs]
    lat = [r['lat_ms'] for r in rows if r['lat_ms'] is not None]
    reqs = 0
    if a.producer_log and os.path.exists(a.producer_log):
        for line in open(a.producer_log):
            d = json.loads(line)
            if 'req' in d and d['t'] >= t0:
                reqs += 1
    summary = {'vehicles': a.vehicles, 'requests': len(rows), 'timeouts': sum(r['lat_ms'] is None for r in rows),
               'verified': sum(r['ok'] for r in rows), 'lat_median_ms': statistics.median(lat) if lat else None,
               'lat_p95_ms': sorted(lat)[int(0.95 * (len(lat) - 1))] if lat else None, 'origin_object_requests': reqs}
    print(json.dumps(summary, indent=1))
    if a.out:
        json.dump({'summary': summary, 'rows': rows}, open(a.out, 'w'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('smoke', 'calibrate', 'fleet'), default='smoke')
    ap.add_argument('--socket', default='/run/cogndn/veh.sock')
    ap.add_argument('--rsu-host', default='10.77.0.2')
    ap.add_argument('--keys', default=os.path.expanduser('~/.cogndn/keys'))
    ap.add_argument('--schedule', help='JSON [[t, name], ...] from cogndn_eval.export_schedule')
    ap.add_argument('--vehicles', type=int, default=5)
    ap.add_argument('--headway', type=float, default=5.0)
    ap.add_argument('--producer-log', default=os.path.expanduser('~/cogndn_logs/producer_s1_livestock.jsonl'))
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    policy = TrustPolicy(trust_path(), a.keys)
    {'smoke': smoke, 'calibrate': calibrate, 'fleet': fleet}[a.mode](a, policy)


if __name__ == '__main__':
    main()
