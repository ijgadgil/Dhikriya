#!/usr/bin/env bash
# Runs every unit test and the linter without ROS or a simulator (same as CI).
set -e
cd "$(dirname "$0")"
export PYTHONPATH=$PWD/src/cogndn_core:$PWD/src/cogndn_ndn:$PWD/src/cogndn_eval:$PWD/src/cogndn_sim:${PYTHONPATH:-}
python3 -m flake8 src
for p in cogndn_core cogndn_ndn cogndn_eval cogndn_sim; do
  echo "== $p"; python3 -m pytest -q src/$p/test
done
