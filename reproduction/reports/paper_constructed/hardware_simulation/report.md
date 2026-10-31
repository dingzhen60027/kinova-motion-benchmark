# Constructed hardware-condition simulation: measured report

The formal run requested **15 trials** with **35,000 roadmap nodes**. It recorded **15 strictly validated virtual state updates** and **15 validated changes of the scheduled target-side label**. Terminal statuses: success_numerical: 15.

**This is a constructed kinematic planning simulation, not a physical robot experiment or a reproduction of camera measurements.** It does not reproduce the paper’s 15 physical trajectories or Table II latency. The Forest/FR3 grid statistics remain separate. A `completed` summary denotes accounting completion, not success of every trial.

Raw run: `/home/eric/wjg/wd/csdecomp/reproduction/runs/20261026T035010.325340Z_constructed_hardware_simulation`. Run state: `completed_with_failures_retained`. Saved-record accounting audit: **passed**. The audit reads saved inputs, trajectory samples, collision flags and distances; it does not rerun collision geometry or certify continuous safety.

## Published settings and actual constructed inputs

| Item | Paper hardware experiment | This run |
|---|---|---|
| Trials / node count | 15 trajectories / 35,000 nodes | 15 requested / 34975 actual nodes |
| Robot / gripper | KUKA LBR iiwa 7 R800; 11 collision boxes | Pinned public iiwa7 R800 (8 boxes) + WSG (3 boxes); author calibration unavailable |
| Collision pairs | Reported 105 self pairs | 52 static including table; 10 robot/table; 42 robot/robot |
| Online robot geometries | 11 boxes | 11 boxes, including anchored base |
| DRM distance caps / neighbors | 0.45 m / 4.5 rad / 10 | 0.45 / 4.5 / 10 |
| Offline voxel full side | 0.06 m | 0.06 m; native half-side 0.029999999329447746 |
| Online voxel side / sphere radius | 0.08 m / circumsphere | 0.08 m / 0.069282032 m |
| EI-ZO / lazy step | FR3 parameters, max. 20 planes / 0.1 rad | `{'delta': 0.005, 'epsilon': 0.005, 'num_particles': 10000, 'max_hyperplanes_per_iteration': 20, 'mixing_steps': 60, 'configuration_margin': 0.01, 'bisection_steps': 9, 'max_iterations': 30, 'tau': 0.5}` / 0.1 rad |
| Joint velocity / acceleration | ±1 rad/s / ±1 rad/s² | ±1.0 / ±1.0 |
| Sensing and execution | Three Intel RealSense D415 cameras and physical robot execution | Neither present; no sensor/controller/device latency |

Parameter sources are Section VII-B, Appendix D (Hardware) and Table VI. The 30-iteration EI-ZO cap follows the draft TeX comment; it does not establish the δ/ε guarantee for every returned set. Degree-six Bezier curves, relative SCSTO tolerance 0.01, validation limits, attempt counts and watchdogs are explicit construction choices. No extra collision pairs or filters were introduced to force the reported 105-pair count.

The public WSG is welded at translation `(0, 0, 0.09)` and RPY `(-pi/2, 0, pi/2)`; the declared TCP is `(0, -0.12, 0)` in its frame. These are public-URDF frame adaptation choices, not author calibration. Source bytes and hashes are retained in the run’s `source_snapshot/reproduction/hardware_constructed/assets/source_manifest.json`.

Frozen initial/right configuration: `[-0.800000011920929, 0.6000000238418579, 0.0, -0.800000011920929, 0.0, 0.0, 0.0]`. Left target-generating configuration: `[0.800000011920929, 0.6000000238418579, 0.0, -0.800000011920929, 0.0, 0.0, 0.0]`. Right target-generating configuration: `[-0.800000011920929, 0.6000000238418579, 0.0, -0.800000011920929, 0.0, 0.0, 0.0]`. Right target anchor equals the separately checked initial anchor: `True`. Preflight status: `passed`; initial free: `True`; left anchor free: `True`.

The anchors were chosen before any roadmap or planning by a preregistered finite geometric scan, not by planner outcomes. The first candidate pair passing the fixed workspace, static model and all 15 voxel/exact-solid scenes was selected. The original colliding anchors and source snapshot are retained in `reports/paper_constructed/hardware_simulation/check_01`; the complete scan and selection are in `anchor_construction_01`. The revised preflight checks all three anchors against all 15 layouts. The target poses are fixed FK transforms saved in `fixed_targets.json`. The 15 source-defined rack/box placements are frozen in `scenes/00.json`–`14.json`; no target or obstacle is regenerated after a planner/solver failure. The table dimensions/center and every rack/box component are declared construction data. A nominal alternating target schedule is not evidence of 15 completed side-to-side motions. Failed trials hold the actual virtual state, so a subsequent scheduled target may be on the same side as the last completed one.

The synthetic cloud samples all box surfaces without ray tracing, occlusion, camera noise or calibration. Known table points are saved but excluded from online voxels because the exact table is already static in the collision model. Surface voxels do not certify solid occupancy. Saved final trajectories receive a separate Drake check against the exact constructed rack/box solids; that check, native/independent voxel checks, solver success, endpoint, bounds and derivative checks must all pass before state advancement.

## Recorded trial outcomes and state chain

| Trial | Scheduled target | Terminal status | Virtual update | Joint-state change (L2 rad) | Exact solid collision samples | Wall time (s) |
|---:|---|---|---|---:|---:|---:|
| 1 | left | success_numerical | True | 4.219858 | 0 | 1.127 |
| 2 | right | success_numerical | True | 7.081765 | 0 | 1.165 |
| 3 | left | success_numerical | True | 7.081765 | 0 | 1.023 |
| 4 | right | success_numerical | True | 7.081765 | 0 | 1.012 |
| 5 | left | success_numerical | True | 7.081765 | 0 | 0.835 |
| 6 | right | success_numerical | True | 7.081765 | 0 | 1.067 |
| 7 | left | success_numerical | True | 7.081765 | 0 | 1.083 |
| 8 | right | success_numerical | True | 7.081765 | 0 | 1.119 |
| 9 | left | success_numerical | True | 7.081765 | 0 | 1.005 |
| 10 | right | success_numerical | True | 7.081765 | 0 | 1.031 |
| 11 | left | success_numerical | True | 7.081765 | 0 | 0.849 |
| 12 | right | success_numerical | True | 7.081765 | 0 | 1.081 |
| 13 | left | success_numerical | True | 7.081765 | 0 | 1.062 |
| 14 | right | success_numerical | True | 7.081765 | 0 | 1.044 |
| 15 | left | success_numerical | True | 7.081765 | 0 | 1.041 |

There were 15 validated nonzero virtual joint-state changes. A zero change after failure is a recorded hold-state rule, not a zero-duration timing measurement. Missing wall times remain missing. The configured per-trial watchdog is 240 s and offline watchdog is 1800 s; neither value is substituted for measured runtime. Timeouts may preserve measured subcomponents under `partial_record`; those are separately listed in `report_audit.json` and are not silently pooled into completed timing fields.

## Timing with explicit denominators

Means and sample standard deviations below use only rows containing that actual timing field. Failed trials remain in the status denominator. Missing/unattempted components are never replaced by zero. The strict-success column is an additional conditional view, not a replacement denominator.

| Component | Recorded n / requested | All recorded mean ± SD (ms) | Strict-success recorded n | Strict-success mean ± SD (ms) |
|---|---:|---:|---:|---:|
| Synthetic surface generation (not perception) | 15 / 15 | 2.310 ± 0.265 | 15 | 2.310 ± 0.265 |
| Virtual point-cloud voxelization | 15 / 15 | 0.188 ± 0.043 | 15 | 0.188 ± 0.043 |
| Native collision-set construction | 15 / 15 | 3.686 ± 1.649 | 15 | 3.686 ± 1.649 |
| Online IK | 15 / 15 | 295.757 ± 5.005 | 15 | 295.757 ± 5.005 |
| DRM path query | 15 / 15 | 41.081 ± 15.189 | 15 | 41.081 ± 15.189 |
| EI-ZO / safe-set construction and recovery | 15 / 15 | 30.831 ± 11.865 | 15 | 30.831 ± 11.865 |
| SCSTO | 15 / 15 | 22.843 ± 0.496 | 15 | 22.843 ± 0.496 |
| Pipeline numerical validation | 15 / 15 | 89.417 ± 22.583 | 15 | 89.417 ± 22.583 |
| Independent exact scene-box validation | 15 / 15 | 211.274 ± 35.141 | 15 | 211.274 ± 35.141 |
| Sum of recorded planning components | 15 / 15 | 394.198 ± 24.039 | 15 | 394.198 ± 24.039 |
| Whole trial wall time | 15 / 15 | 1036.255 ± 90.613 | 15 | 1036.255 ± 90.613 |

Synthetic surface construction and voxelization are not camera perception. `planning_component_seconds` is the sum of the actually recorded collision-set, IK, DRM, safe-set and SCSTO components; it excludes model setup, map loading, warmup, synthetic-cloud generation and final independent checks. On a failed trial it can be a partial sum, with contributing field names retained per row. It is not end-to-end camera-to-robot latency. SCSTO duration is planned kinematic trajectory time, not measured execution time.

Planned trajectory duration: 6.978 ± 0.961 s, n=15. Region count: 1.000 ± 0.000, n=15. Online voxels: 93.933 ± 31.301, n=15. Offline-map status: `completed`; recorded offline total: 100.684 s. Failed offline partial records stay nested in the raw roadmap record; missing totals are not inferred.

## Audit, scope and retained failures

The posthoc audit recomputes status counts, success counts and every summary mean/SD/denominator. It checks the saved start-state chain, unchanged targets, recorded exact-solid flags versus signed distances, trajectory identity between voxel and exact-solid checks, endpoint identity, and each required gate before a virtual update. It separately verifies actual robot/voxel pair counts when available. An accounting audit can pass even when every planning trial fails.

No inconsistencies were found in these saved-record checks.

## Separate smoke run

The optional smoke run `/home/eric/wjg/wd/csdecomp/reproduction/runs/20261026T034913.809331Z_constructed_hardware_simulation` used 800 nodes and requested 2 trials; statuses: `{'failed': 2}`. Its 0 virtual successes are not pooled with the formal run. Saved-record audit issues: `[]`.

All source snapshots, IK attempts, seed paths, clouds, corridors, candidates, validation arrays, process exits and failures remain in the raw run. Static replay includes all requested trials rather than a successful subset. There is no claim of the paper’s original calibration, camera observations, obstacle arrangements, 105-pair model, physical controller behavior, 15 physical successes or published latency. The newly constructed supplement addresses executable planning under similar public parameters while preserving these missing experimental conditions.
