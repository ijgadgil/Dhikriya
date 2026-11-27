# Cognitive-NDN

**A VLA-based framework for predictive content caching and explainable navigation in autonomous vehicles.**

An on-board vision-language-action pipeline (llava-phi3 with schema-constrained output) states what
information it will need next. The **Cognitive Interest Compiler (C1)** turns those needs into named,
prioritised NDN Interests; signed Data fetched over a real NDN data plane (NDNd, vehicle + road-side unit)
**grounds the next decision (C2)**; and every manoeuvre is recorded with **evidence-linked, hash-chained
explanations (C3)** that cite the names and digests of the data used. A LiDAR time-to-collision envelope is
the only component allowed to command the car.

| Package | Content |
|---|---|
| `cogndn_core` | ROS-free library: C1 compiler, C2 context selection, C3 grounding + ledger, envelope, planner/tracker, VLA client, scenarios |
| `cogndn_msgs` | ROS 2 messages |
| `cogndn_sim` | Generated Gazebo worlds for 6 scenarios in two maps (synthetic town and the earlier Mcity map `mcity_vla`, same road graph), Prius model, bridge, launch |
| `cogndn_perception` | YOLOv10n detector, LiDAR clustering |
| `cogndn_cognition` | VLA node (Ollama) |
| `cogndn_compiler` | C1 node with all baseline policies |
| `cogndn_ndn` | Keys/trust, signed segmented producer, verifying consumer, netns+netem data plane, fleet/calibration tool |
| `cogndn_navigation` | Driver (planner + pure pursuit) and the LiDAR TTC safety envelope |
| `cogndn_explain` | C3 node and ledger |
| `cogndn_eval` | Headless harness, experiments E1–E6, statistics, figures, run analysis, VLA replay |
| `cogndn_bringup` | Full-stack launch, parameters, run scripts, environment check |

Quick start (after the set-up guide): `bash run_tests.sh`, then
`bash $(ros2 pkg prefix cogndn_bringup)/share/cogndn_bringup/scripts/run_scenario.sh s1_livestock cognitive`.

**Two maps, one ground truth.** Every scenario exists as `worlds/<scenario>.sdf` (minimal town, the controlled
baseline) and `worlds/<scenario>_mcity_vla.sdf` (the earlier project's Mcity map, model `city_vla` with unchanged files, placed under the same road graph by
`config/mcity_vla.yaml`: scale 0.15, rotated -90 deg). Content names, required items and metrics are identical, so a
result can be compared across maps; Mcity adds real lane markings, kerbs, islands and side roads as visual clutter
for the VLA. Select with `map:=mcity_vla` in the launch files, or the 11th argument of `run_scenario.sh`.
The Mcity mesh is visual only (collision is a flat ground plane), so kerbs never disturb the vehicle dynamics.
The mcity_vla worlds also close the hollow city_vla facades with solid blocks and add a scripted Indian-street
scene (`config/street_indian.yaml`): walking pedestrians (one crosses the ego lane, one child), stray cows,
street dogs (one trotting), parked cars and autorickshaws, and oncoming traffic. Every actor follows a fixed
script, so all policies face the same street; cows are left out of `s1_livestock` and `s5_decoy_sign`, and
through-traffic out of `s3_closure_detour`, so each scenario keeps its meaning.
Gazebo actors have no physics, so every walker also gets an invisible capsule `<id>_body` (VelocityControl +
OdometryPublisher). `street_bodies_node` (started by `sim.launch.py` for `map:=mcity_vla`) runs the same script
on simulation time and keeps each body inside its actor; the camera and LiDAR see the actor, and the car
physically collides with the body. Started with plain `gz sim`, the bodies simply stand at their start points.

See `docs/Cognitive-NDN_Build_and_Setup_Guide.docx` for the full explanation and step-by-step set-up.
