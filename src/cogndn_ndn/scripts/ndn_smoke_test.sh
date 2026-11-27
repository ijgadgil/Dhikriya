#!/usr/bin/env bash
# End-to-end check of the real data plane: fetch a hazard bulletin and a segmented map tile through
# vehicle NDNd -> netem -> RSU NDNd -> signed producer, then fetch again to see the cache hit.
set -e
python3 -m cogndn_ndn.fleet --mode smoke --socket /run/cogndn/veh.sock
