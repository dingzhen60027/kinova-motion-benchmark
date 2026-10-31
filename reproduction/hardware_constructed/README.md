# Constructed simulation of the paper's hardware conditions

This supplement runs real IK, native dynamic-roadmap planning, EI-ZO inflation,
SCSTO with the released author implementation, and numerical collision/derivative
validation. It does **not** operate a robot, contact a camera, simulate a motor
controller, or recover the authors' recorded hardware experiment. Its results
remain separate from the Forest/FR3 grids and from the paper's Table II.

The public KUKA LBR iiwa 7 R800 model has eight collision boxes; the public WSG
adds three. The gripper transform is the explicitly declared public-URDF frame
adaptation used in the verified public-model throughput diagnostic:
translation `(0, 0, 0.09)` and RPY `(-pi/2, 0, pi/2)`. The TCP is additionally
placed at `(0, -0.12, 0)` in the gripper frame. Neither transform is an author
calibration. `assets/source_manifest.json` identifies the actual pinned bytes;
`model.py` rejects mismatched source hashes. Static collision counts are measured,
not modified to equal the paper's 105 pairs. All eleven robot boxes participate
in online obstacle checking, including the anchored base.

## Conditions

Published settings used here are 35,000 roadmap nodes, task-space distance 0.45 m,
configuration-space distance 4.5 rad, ten neighbors, offline full voxel side
0.06 m, lazy edge step 0.1 rad, online voxel side 0.08 m and circumsphere radius
`sqrt(3)*0.04` m. EI-ZO uses the paper's FR3 settings with 20 maximum planes per
iteration. Velocity and acceleration bounds are ±1 rad/s and ±1 rad/s². The
30-iteration EI-ZO cap comes from the commented author TeX; capped runs do not
provide an unconditional δ/ε certificate. Spline order, numerical validation,
IK-attempt count, timing budgets, table, rack and box dimensions, poses and layout
sequence are declared constructions.

`scenes.py` fixes fifteen layouts before planning. Each changes the shoe rack or
box position/presence. Both left/right targets are fixed forward-kinematic poses
of the configurations in `config.json`. Every trial keeps its scheduled target;
there is no retry with a different scene or pose. The simulated state begins at
the declared right-side configuration. It advances to a trajectory endpoint only
when native, independent voxel-model, exact scene-box, derivative and endpoint
checks all pass. It stays unchanged after a failed trial. This is a kinematic
state update, not simulated physical execution.

The virtual cloud samples every face of the table, rack components and box on a
0.04 m grid. It has no ray tracing, occlusion, noise, camera calibration or device
latency. Original table points remain in the saved cloud; they are excluded from
online voxels as known background because the exact table is already static in
the model. Occupied grid cells are replaced by 0.08 m circumspheres. These surface
voxels are not claimed to certify solid occupancy. Every saved final trajectory
also receives an independent Drake check against the actual constructed rack and
box solids; this second check gates virtual state advancement.

## Execute after other timed experiments stop

Use the existing Conda environment; no additional installation is required.

```bash
# Small real CUDA/Drake model check, including both declared target anchors.
bash local/python.sh -m reproduction.hardware_constructed.run check \
  --output reproduction/reports/paper_constructed/hardware_simulation/check_new

# Explicitly smaller smoke run. It is not the 35,000-node/15-trial condition.
bash local/python.sh -m reproduction.hardware_constructed.run run \
  --nodes 800 --trials 0 1

# Published roadmap size and all fifteen frozen scene modifications.
bash local/python.sh -m reproduction.hardware_constructed.run run
```

Every execution creates a fresh `*_constructed_hardware_simulation` directory.
The offline native map is built once and reused by all selected trials. The
35,000-node run has an 1800 s offline watchdog and 240 s per-trial watchdog;
these are declared resource limits, not paper parameters. Child processes retain
stdout/stderr, exit codes, partial results and exceptions. A timeout never becomes
a success. Model-check errors mark unattempted trials explicitly.

The run saves exact source snapshots, configuration, fixed targets, all fifteen
scene descriptions, native map, sampled nodes, synthetic cloud points and solid
labels, voxel centers/keys, online IK attempts, DRM path and validation points,
all pre-solve regions/seeds, every SCSTO candidate, independent checks and replay
samples. No failed input is discarded. A unique pointer under
`reports/paper_constructed/hardware_simulation` links each outcome to its raw run.
The v5 portable packager includes this package, its pinned sources and all these
raw artifacts; it does not merge this supplement into the main-grid statistics.

## Timing and interpretation

Synthetic surface generation and voxelization are separately timed. Neither is
called camera perception time. Collision-set construction, IK, DRM, SCS creation
and recovery, SCSTO optimization, validation, model setup, map loading and warmup
have separate recorded fields. `planning_component_seconds` sums collision-set,
IK, DRM, SCS and SCSTO time; it excludes virtual-cloud construction, setup, loading,
warmup and final checks, and is not a camera-to-motion latency. Reports give each
metric's actual recorded denominator. Total wall time and every unsuccessful
trial remain visible.

The initial real preflight failed because the public link 5 and link 7 collision
boxes overlapped by 4.17 mm. Its configuration, source snapshot and raw checks
remain in `check_01`. Before any planning, the declared geometric-only finite
scan selected its first valid mirrored pair; the unchanged 15 scenes and all
voxel/exact-solid models had at least 29.70 mm clearance at these anchors. The
new `check_02` actually passed. The construction contract, all 420 candidate
pairs and predicates are retained in `reports/paper_constructed/
hardware_simulation/anchor_construction_01`. Full planning outcomes are reported
from the saved runs; a passing model check does not establish planning success.

## Static replay after execution

```bash
bash local/python.sh -m reproduction.hardware_constructed.plot \
  --run-dir <HARDWARE_SIMULATION_RUN> \
  --output reproduction/reports/paper_constructed/hardware_simulation/replay
```

The PNG includes every requested trial, with its status, real public collision
boxes at the saved start, the declared table/rack/box solids, fixed target and
saved TCP trajectory when available. Rejected trajectories are colored separately;
solid-geometry failures are labeled. A translucent robot shows the saved candidate
endpoint even when that candidate was rejected. This is a static evidence replay,
not an animation or record of physical execution. The plotter is excluded from
computational identity; its source hash, input hashes, display subsampling and
figure hash are recorded independently in `plot_manifest.json`.
