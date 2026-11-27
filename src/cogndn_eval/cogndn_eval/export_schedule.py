"""Export the Interest schedule of one simulated vehicle for replay on the real NDN data plane.

  python3 -m cogndn_eval.export_schedule --scenario s3_closure_detour --policy cognitive --speed 8 > sched.json
  python3 -m cogndn_ndn.fleet --mode fleet --schedule sched.json --vehicles 10 --headway 5
"""
import argparse
import json
import os
import random

from cogndn_core.catalogue import DEFAULT_TYPES
from cogndn_core.scenario import Scenario
from cogndn_core.world import World

from .experiments import default_paths
from .harness import RunParams, VehicleRun
from .netsim import Network, NetParams


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='s1_livestock')
    ap.add_argument('--policy', default='cognitive')
    ap.add_argument('--speed', type=float, default=8.0)
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    town, sdir = default_paths()
    sc = Scenario.from_yaml(os.path.join(sdir, a.scenario + '.yaml'), World.from_yaml(town))
    net = Network(NetParams(), DEFAULT_TYPES, seed=a.seed)
    v = VehicleRun(0, sc, a.policy, RunParams(speed=a.speed), net, random.Random(a.seed))
    t = 0.0
    while v.step(t):
        t += 0.1
    print(json.dumps(v.sent))


if __name__ == '__main__':
    main()
