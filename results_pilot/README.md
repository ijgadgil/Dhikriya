# Pilot results (headless harness, synthetic VLA model)

Produced by `python3 -m cogndn_eval.experiments --out results_pilot --seeds 30` (about 1 minute on 2 cores).

IMPORTANT: these runs use the *synthetic* VLA model (cue-driven, llava-phi3 latency range 3.4-5.9 s, no
errors unless injected) and the *uncalibrated* network model. They validate the pipeline and show the expected
trends; they are NOT paper results. For the paper: (1) calibrate the network model on the real NDNd data
plane (`fleet --mode calibrate`), (2) record real llava-phi3 runs in Gazebo and replay them
(`make_replay` + `experiments --replay`), (3) report the closed-loop Gazebo runs (`run_matrix.sh`).

Files: `E*_runs.csv` (every run), `E*_summary.csv` (mean and 95% CI per condition), `tests.json` (paired
Wilcoxon tests), `fig_e1_main.png`, `fig_e2_frontier.png`, `fig_e3_robustness.png`, `fig_e5_fleet.png`.
