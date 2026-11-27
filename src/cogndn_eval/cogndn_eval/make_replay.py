"""Turn real llava-phi3 logs (cognition.jsonl from Gazebo runs) into harness replay records, so the
network experiments can be repeated with the *real* VLA outputs instead of the synthetic model.
  ros2 run cogndn_eval make_replay ~/cogndn_logs/run_* > replay.json
  ros2 run cogndn_eval experiments --only E1 --replay replay.json --out results_replay"""
import argparse
import glob
import json
import os

from cogndn_core.paths import scenario_path, town_path
from cogndn_core.scenario import Scenario
from cogndn_core.world import World


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    a = ap.parse_args()
    w = World.from_yaml(town_path())
    out = {}
    for pat in a.runs:
        for d in sorted(glob.glob(pat)):
            mp = os.path.join(d, 'run.json')
            if not os.path.exists(mp):
                continue
            meta = json.load(open(mp))
            if meta.get('policy') != 'cognitive' or meta['scenario'] in out:     # the full method only
                continue
            sc = Scenario.from_yaml(scenario_path(meta['scenario']), w)
            recs = []
            for c in (json.loads(x) for x in open(os.path.join(d, 'cognition.jsonl'))):
                z, s_in, _ = w.locate(c['x'], c['y'])
                base = sc.route_s(z)
                if base is None or not c.get('valid'):
                    continue
                recs.append({'s': base + s_in, 'latency_s': c['latency_s'], 'needs': c['needs'],
                             'replan': c['action'] in ('turn_left', 'turn_right') and sc.planned_route != sc.truth_route})
            out[meta['scenario']] = recs
    print(json.dumps(out))


if __name__ == '__main__':
    main()
