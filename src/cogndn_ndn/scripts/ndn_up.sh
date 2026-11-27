#!/usr/bin/env bash
# Bring up the Cognitive-NDN data plane on one machine:
#   vehicle NDNd (main namespace, 10.77.0.1)  <== veth + tc netem ==>  RSU NDNd + producer (netns "rsu", 10.77.0.2)
# Usage: bash ndn_up.sh <scenario> [delay_ms=10] [loss_pct=0] [rate_mbit=10] [origin_ms=60] [forge_types=""]
# Needs: sudo rights, ndnd on PATH, the cogndn workspace sourced (python3 -m cogndn_ndn.* must import).
set -euo pipefail
SCEN=${1:?scenario id, e.g. s1_livestock}; DELAY=${2:-10}; LOSS=${3:-0}; RATE=${4:-10}; ORIGIN=${5:-60}; FORGE=${6:-}
CFG=$(python3 -c "import os,cogndn_ndn;print(os.path.dirname(cogndn_ndn.__file__))")/../config
[ -f "$CFG/ndnd_veh.yml" ] || CFG=$(ros2 pkg prefix cogndn_ndn)/share/cogndn_ndn/config
USER_NAME=${SUDO_USER:-$USER}
LOG=${COGNDN_LOG:-$HOME/cogndn_logs}; mkdir -p "$LOG"
NDND=$(command -v ndnd)
PY=$(command -v python3)

sudo mkdir -p /run/cogndn && sudo chmod 777 /run/cogndn
sudo ip netns add rsu 2>/dev/null || true
if ! ip link show veh0 >/dev/null 2>&1; then
  sudo ip link add veh0 type veth peer name rsu0
  sudo ip link set rsu0 netns rsu
fi
sudo ip addr replace 10.77.0.1/24 dev veh0 && sudo ip link set veh0 up
sudo ip netns exec rsu ip addr replace 10.77.0.2/24 dev rsu0
sudo ip netns exec rsu ip link set rsu0 up
sudo ip netns exec rsu ip link set lo up
# Larger UDP socket buffers: a 200 kB tile is 29 x 7 kB segments, each split into ~6 UDP fragments, and
# several tiles can be in flight; default buffers (~208 kB) drop fragments and stall the fetch.
for ns in "" "ip netns exec rsu"; do
  sudo $ns sysctl -q -w net.core.rmem_max=8388608 net.core.rmem_default=4194304 \
    net.core.wmem_max=8388608 net.core.wmem_default=4194304 2>/dev/null || true
done
# netem on both directions (each side shapes its egress)
sudo tc qdisc replace dev veh0 root netem delay ${DELAY}ms loss ${LOSS}% rate ${RATE}mbit
sudo ip netns exec rsu tc qdisc replace dev rsu0 root netem delay ${DELAY}ms loss ${LOSS}% rate ${RATE}mbit

# forwarders
nohup "$NDND" fw run "$CFG/ndnd_veh.yml" > "$LOG/ndnd_veh.log" 2>&1 &
sudo ip netns exec rsu sudo -u "$USER_NAME" nohup "$NDND" fw run "$CFG/ndnd_rsu.yml" > "$LOG/ndnd_rsu.log" 2>&1 &
for s in veh rsu; do for i in $(seq 50); do [ -S /run/cogndn/$s.sock ] && break; sleep 0.1; done; done

# vehicle -> RSU route for the whole /cogndn namespace (UDP face over the netem link)
NDN_CLIENT_TRANSPORT=unix:///run/cogndn/veh.sock "$NDND" fw route-add prefix=/cogndn face=udp://10.77.0.2:6363

# signed producer inside the RSU namespace
SIMCFG=${COGNDN_SIM_CONFIG:-$(ros2 pkg prefix cogndn_sim 2>/dev/null)/share/cogndn_sim/config}
sudo ip netns exec rsu sudo -u "$USER_NAME" env PYTHONPATH="${PYTHONPATH:-}" AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-}" \
  COGNDN_SIM_CONFIG="$SIMCFG" HOME="$HOME" \
  nohup "$PY" -m cogndn_ndn.producer --scenario "$SCEN" --socket /run/cogndn/rsu.sock --keys "$HOME/.cogndn/keys" \
  --origin-delay-ms "$ORIGIN" ${FORGE:+--forge $FORGE} --log "$LOG/producer_${SCEN}.jsonl" > "$LOG/producer.log" 2>&1 &
sleep 1
echo "NDN plane up: scenario=$SCEN delay=${DELAY}ms loss=${LOSS}% rate=${RATE}Mbit origin=${ORIGIN}ms forge=${FORGE:-none}"
echo "Vehicle socket: /run/cogndn/veh.sock   RSU: tcp4://10.77.0.2:6363   logs: $LOG"
