#!/usr/bin/env bash
# One closed-loop run: NDN data plane + Redis + Gazebo + full stack, then analysis.
# Usage: run_scenario.sh <scenario> <policy> [model=llava-phi3] [duration_s=120] [rate_mbit=10] [delay_ms=10]
#                        [loss_pct=0] [origin_ms=60] [forge_types=] [headless=true] [map=town|mcity_vla]
set -uo pipefail
SCEN=${1:?scenario}; POLICY=${2:?policy}; MODEL=${3:-llava-phi3}; DUR=${4:-120}; RATE=${5:-10}; DELAY=${6:-10}
LOSS=${7:-0}; ORIGIN=${8:-60}; FORGE=${9:-}; HEADLESS=${10:-true}; MAP=${11:-mcity_vla}
NDN_SCRIPTS=$(ros2 pkg prefix cogndn_ndn)/share/cogndn_ndn/scripts
STAMP=$(date +%Y%m%d_%H%M%S)
export COGNDN_RUN_DIR=$HOME/cogndn_logs/run_${STAMP}_${SCEN}_${MAP}_${POLICY}_${MODEL//[:\/]/-}
mkdir -p "$COGNDN_RUN_DIR"
cat > "$COGNDN_RUN_DIR/run.json" <<JSON
{"scenario": "$SCEN", "map": "$MAP", "policy": "$POLICY", "model": "$MODEL", "rate_mbit": $RATE, "delay_ms": $DELAY,
 "loss_pct": $LOSS, "origin_ms": $ORIGIN, "forge": "$FORGE", "started": "$STAMP"}
JSON
echo "== run dir: $COGNDN_RUN_DIR"
redis-cli ping >/dev/null || { echo "redis-server is not running (sudo systemctl start redis-server)"; exit 1; }
redis-cli flushall >/dev/null
if [[ $POLICY == cognitive* ]]; then
  curl -s http://localhost:11434/api/generate -d "{\"model\": \"$MODEL\", \"keep_alive\": \"60m\", \"options\": {\"num_ctx\": 2048}}" >/dev/null \
    || { echo "Ollama is not reachable on :11434"; exit 1; }
fi
bash "$NDN_SCRIPTS/ndn_down.sh" >/dev/null 2>&1
COGNDN_LOG=$COGNDN_RUN_DIR bash "$NDN_SCRIPTS/ndn_up.sh" "$SCEN" "$DELAY" "$LOSS" "$RATE" "$ORIGIN" "$FORGE" || exit 1
ros2 bag record -s mcap -o "$COGNDN_RUN_DIR/bag" /cogndn/cognitive_state /cogndn/interests /cogndn/data \
  /cogndn/envelope_events /cogndn/route_state /cogndn/explanation /vla/odom /model/prius_vla/cmd_vel \
  > "$COGNDN_RUN_DIR/bag.log" 2>&1 &
BAG=$!
ros2 launch cogndn_bringup cogndn.launch.py scenario:=$SCEN policy:=$POLICY model:=$MODEL headless:=$HEADLESS map:=$MAP \
  > "$COGNDN_RUN_DIR/launch.log" 2>&1 &
LAUNCH=$!
echo "== running $SCEN ($MAP) / $POLICY / $MODEL for ${DUR}s (log: launch.log)"
sleep "$DUR"
kill -INT $LAUNCH $BAG 2>/dev/null; sleep 5
pkill -INT -f "cogndn_bringup cogndn.launch.py" 2>/dev/null; pkill -f "gz sim" 2>/dev/null
bash "$NDN_SCRIPTS/ndn_down.sh" >/dev/null 2>&1
ros2 run cogndn_eval analyze_run "$COGNDN_RUN_DIR" | tee "$COGNDN_RUN_DIR/summary.json"
