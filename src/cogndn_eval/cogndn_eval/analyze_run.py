"""Compute paper metrics from one or more closed-loop Gazebo runs (directories written by run_scenario.sh).

Per run: on-time coverage of required data (same definition as the harness), bytes per km, VLA
validity and latency, explanation grounding, ledger integrity (+ tamper test), envelope events.
  ros2 run cogndn_eval analyze_run ~/cogndn_logs/run_*  --csv real_runs.csv
"""
import argparse
import math
import csv
import glob
import json
import os
import shutil
import sqlite3
import statistics
import tempfile

from cogndn_core.ledger import Ledger
from cogndn_core.paths import scenario_path, town_path
from cogndn_core.scenario import Scenario
from cogndn_core.world import World


def jl(path):
    return [json.loads(x) for x in open(path)] if os.path.exists(path) else []


def tamper_detected(ledger_path):
    """Copy the ledger, alter one stored decision and check that verification catches it."""
    tmp = tempfile.mkdtemp()
    cp = os.path.join(tmp, 'l.db')
    # The live ledger is in WAL mode: recent records may sit in ledger.db-wal (e.g. after an interrupted run),
    # so copy through SQLite's backup API instead of copying the file.
    src, con = sqlite3.connect(ledger_path), sqlite3.connect(cp)
    src.backup(con)
    src.close()
    has = con.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='records'").fetchone()
    n = con.execute('SELECT COUNT(*) FROM records').fetchone()[0] if has else 0
    if n == 0:
        con.close()
        shutil.rmtree(tmp, ignore_errors=True)
        return None
    con.execute("UPDATE records SET body = replace(body, 'keep_lane', 'stop') WHERE seq = ?", ((n + 1) // 2,))
    con.execute("UPDATE records SET body = body || ' ' WHERE seq = ?", ((n + 1) // 2,))
    con.commit()
    con.close()
    ok, bad = Ledger(cp).verify()
    shutil.rmtree(tmp, ignore_errors=True)
    return not ok


def analyze(d):
    meta = json.load(open(os.path.join(d, 'run.json')))
    w = World.from_yaml(town_path())
    sc = Scenario.from_yaml(scenario_path(meta['scenario']), w)
    poses = [p for p in jl(os.path.join(d, 'driver.jsonl')) if p.get('kind') == 'pose' and p.get('s') is not None]
    arr = [a for a in jl(os.path.join(d, 'consumer.jsonl')) if a.get('ok')]
    out = {'run': os.path.basename(d), **meta}
    # coverage
    res = []
    for r in sc.required:
        sd = sc.route_s(r.deadline_zone)
        hit = next((p['t'] for p in poses if p['s'] >= sd), None)
        if hit is None:
            res.append({'name': r.name, 'reached': False, 'semantic': r.semantic})
            continue
        ok = any(a['name'] == r.name and a['t'] <= hit and a['fresh_until'] > hit and a['sig_ok'] for a in arr)
        res.append({'name': r.name, 'reached': True, 'on_time': ok, 'semantic': r.semantic})
    reached = [x for x in res if x['reached']]
    sem = [x for x in reached if x['semantic']]
    out['cov_semantic'] = sum(x['on_time'] for x in sem) / len(sem) if sem else None
    out['cov_all'] = sum(x['on_time'] for x in reached) / len(reached) if reached else None
    out['missed'] = [x['name'] for x in reached if not x['on_time']]
    out['not_reached'] = [x['name'] for x in res if not x['reached']]
    # distance actually driven (sum of pose-to-pose steps): robust to off-route zones and lane changes
    dist = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y']) for a, b in zip(poses, poses[1:])) / 1000.0 \
        if len(poses) > 1 else None
    out['distance_m'] = round(dist * 1000.0, 1) if dist else 0.0
    kb = sum(a.get('bytes', 0) for a in arr) / 1000.0
    out['fetched_kb'] = kb
    out['kb_per_km'] = kb / dist if dist else None
    out['fetch_latency_ms_median'] = 1000 * statistics.median([a['latency_s'] for a in arr]) if arr else None
    out['rejected_data'] = sum(1 for a in arr if not a['sig_ok'])
    # VLA
    cog = jl(os.path.join(d, 'cognition.jsonl'))
    if cog:
        lat = sorted(c['latency_s'] for c in cog)
        out.update({'vla_cycles': len(cog), 'vla_valid_rate': sum(c['valid'] for c in cog) / len(cog),
                    'vla_latency_median_s': statistics.median(lat), 'vla_latency_p95_s': lat[int(0.95 * (len(lat) - 1))],
                    'vla_needs_per_cycle': statistics.fmean(len(c['needs']) for c in cog),
                    'vla_actions': {a: sum(c['action'] == a for c in cog) for a in sorted({c['action'] for c in cog})}})
    # explanations
    ex = jl(os.path.join(d, 'explain.jsonl'))
    if ex:
        # An explanation with no factual claims is trivially "grounded"; average only over those that make claims.
        withc = [e for e in ex if e.get('claims')]
        out['explanations'] = len(ex)
        out['explanations_with_claims'] = len(withc)
        out['grounding_mean'] = statistics.fmean(e['grounding'] for e in withc) if withc else None
        prov = [s for e in ex for s in e['provenance']]
        out['provenance_ok_rate'] = (sum(s == 'ok' for s in prov) / len(prov)) if prov else None
    lp = os.path.join(d, 'ledger.db')
    if os.path.exists(lp):
        out['ledger_intact'] = Ledger(lp).verify()[0]
        try:
            out['ledger_tamper_detected'] = tamper_detected(lp)
        except Exception as e:  # the check works on a temporary copy; never let it stop the analysis
            out['ledger_tamper_detected'] = None
            out['ledger_check_error'] = str(e)[:120]
    env = jl(os.path.join(d, 'envelope.jsonl'))
    if env:
        out['envelope_events'] = {k: sum(e['kind'] == k for e in env) for k in sorted({e['kind'] for e in env})}
        out['min_obstacle_distance_m'] = min(e.get('min_distance', 1e9) for e in env)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--csv', default=None)
    a = ap.parse_args()
    rows = []
    for pat in a.runs:
        for d in sorted(glob.glob(pat)):
            if os.path.exists(os.path.join(d, 'run.json')):
                r = analyze(d)
                rows.append(r)
                print(json.dumps(r, indent=1, default=str))
    if a.csv and rows:
        keys = sorted({k for r in rows for k in r})
        with open(a.csv, 'w', newline='') as f:
            wr = csv.DictWriter(f, keys)
            wr.writeheader()
            for r in rows:
                wr.writerow({k: json.dumps(v) if isinstance(v, (dict, list)) else v for k, v in r.items()})


if __name__ == '__main__':
    main()
