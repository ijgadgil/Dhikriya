#!/usr/bin/env bash
# Repeated closed-loop runs for the paper's real-simulation results (policy comparison).
# Usage: run_matrix.sh [repeats=3] [maps="mcity_vla"] [policies="cognitive route_typed route_all ondemand"]
# Ablation: run_matrix.sh 3 mcity_vla "cognitive_vlm cognitive_ocr"
# Resumable: a (scenario, policy, map) combination that already has `repeats` finished runs is skipped,
# so after a crash or shutdown just start the same command again. ~3 min per run plus a 20 s cool-down.
# Needs passwordless sudo for ip/tc/sysctl (see the setup notes) because it runs unattended for hours.
REPEATS=${1:-3}; MAPS=${2:-mcity_vla}; POLICIES=${3:-cognitive route_typed route_all ondemand}
MODEL=llava-phi3
SCENARIOS="s0_straight_control s1_livestock s2_workzone s3_closure_detour s4_school_crossing s5_decoy_sign"
done_runs() { ls -d "$HOME"/cogndn_logs/run_*_"$1"_"$3"_"$2"_"$MODEL" 2>/dev/null | while read -r d; do
              [ -f "$d/summary.json" ] && echo "$d"; done | wc -l; }
total=0; for w in $MAPS; do for s in $SCENARIOS; do for p in $POLICIES; do total=$((total + REPEATS)); done; done; done
n=0
for r in $(seq "$REPEATS"); do
  for w in $MAPS; do
    for s in $SCENARIOS; do
      for p in $POLICIES; do
        n=$((n + 1))
        if [ "$(done_runs "$s" "$p" "$w")" -ge "$r" ]; then continue; fi
        echo "[$(date +%H:%M)] run $n/$total: $s / $p / $w (repeat $r)"
        bash "$(dirname "$0")/run_scenario.sh" "$s" "$p" "$MODEL" 120 10 10 0 60 "" true "$w" > /tmp/matrix_last_run.txt 2>&1 \
          || echo "   run failed - see /tmp/matrix_last_run.txt"
        sleep 20      # let the GPU and CPU cool down between runs
      done
    done
  done
done
ros2 run cogndn_eval analyze_run "$HOME"/cogndn_logs/run_* --csv "$HOME/cogndn_logs/real_runs.csv" > /dev/null
echo "[$(date +%H:%M)] finished. summary table: $HOME/cogndn_logs/real_runs.csv"
