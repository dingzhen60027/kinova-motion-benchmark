# Canonical Kinova model and collision audit

The benchmark uses the full online-example Kinova + welded Robotiq primitive collision model from `csdecomp/tests/test_assets/directives/kinova_sens_on_table.yml`. Both new offline roadmaps and online methods construct `benchmarks.generic_motion_v1.model.Model`; existing example roadmaps are not imported. This is an RSS-inspired Kinova adaptation, not the RSS paper's FR3 sphere model.

## What the upstream examples actually do

- `examples/build_DRM.py` uses the CSD parser with `simple_kinova_sens_on_table.yml`, `reduced_kinova_sens.urdf`, and `collapsed_robotiq.urdf`. It samples 8,000 configurations, uses 20 neighbors and a 0.1-radian edge step, and creates a pose map for `kinova::end_effector_link`.
- `examples/kinova_mintime_planning.py` parses the full directives with Drake and converts the resulting primitive model to CSD. It loads the previously built example map. Its goal is chosen from roadmap poses near a requested Cartesian point; the new benchmark instead fixes all seven joint coordinates before any planner runs.
- `examples/test_eizo.py` uses the full directives through the CSD parser and four 0.02 m spheres. That is an isolated inflation example, not the benchmark's frozen 0.04 m geometry protocol.
- `examples/utils.py:get_fake_voxel_map` resets the global NumPy seed to 3 and produces 20 spheres in `[-0.1,1.2] × [-1.2,1.2] × [0,1.5]`. Repeated calls are the same scene. The benchmark does not treat such repeated calls as independent environments.

The reduced/full filename difference does **not** itself establish a physical geometry mismatch. The actual reduced model has 20 links and the full converted model has 29. Both have 20 collision boxes, 15 moving collision geometries, seven active joints with matching limits, and the same 137 checked geometry pairs. On the saved 512 random configurations their CUDA collision classifications agree. A further 64-configuration forward-kinematics comparison finds per-geometry world-transform and TCP differences only at float32 scale (maximum about 3.58e-7 in the homogeneous transform entries). The reduced model collapses fixed links and the fixed gripper chain. The machine-readable audit records exact comparison values. The benchmark still selects one construction and hashes it to prevent later silent drift.

## Frozen model

Joint order is `kinova_joint_1_q` through `kinova_joint_7_q`. Nominal URDF limits are ±[3.10, 2.24, 3.10, 2.57, 3.10, 2.09, 3.10] rad; the actual CSD float32 values and Drake double values are saved separately. The base is welded to world at identity. World is right-handed and lengths are metres. TCP is the existing `kinova::end_effector_link`; no task tool is added.

The fixed world contains the original base, table, two tripod boxes, and protective wall. Full geometry dimensions, local transforms, zero-configuration world poses, checked and excluded pairs, source file hashes, and canonical robot/collision protocol hashes are saved in `configs/robot.json`.

CSD `URDFParser::finalizeScene` excludes world/world, same-link, adjacent-link and all-fixed-chain geometry pairs. These 137 pairs are exactly the native Drake SceneGraph candidates for the selected model; the adapter asserts this equality by pair identity. CSD treats the world-welded base as world, so the base is not tested against online voxel spheres. This behavior is preserved for all methods; the generator can separately measure new-obstacle overlap with fixed geometry using `fixed_geometry_sphere_clearance`.

The current CSD collision kernels support boxes and spheres, with touching regarded as collision. They use float32 transforms and geometric calculations. The independent checker uses Drake double-precision poses and signed-distance queries for all static pairs. For online spheres it computes double-precision signed sphere/box distances from those Drake poses, avoiding thousands of additional SceneGraph bodies. `Model.nto_plant()` registers the identical online spheres only when Drake's native nonlinear minimum-distance constraint needs them, and checks every resulting collision pair by geometry name. Two online spheres add exactly 30 pairs; an actual eight-state test matches the independent minimum distances within 1e-10 m.

## Actual G0 run

`reports/model_audit/audit.json` and `raw_samples.npz` retain every tested state and classification. PCG64 seed 861730 is independent of dataset generation. The audit checks 512 random configurations and 528 points on 16 independently sampled edges, first without additional obstacles and then with 30 online spheres. All four groups have zero CSD/Drake classification differences. Sixty-six probes around moving-finger contact with a 0.04 m sphere also have zero differences. There are 307 base-free random configurations; their observed TCP bounds are approximately x=[-0.399,0.911], y=[-0.871,0.864], z=[0.073,1.212] m. These samples justify an obstacle ROI audit; they do not prove every point in that box reachable.

A fixed 2e-6 m contact band acknowledges float32 geometry uncertainty even though this audit observed no mismatches. Endpoint generation uses an additional positive clearance margin. Final validation must retain both native and independent results; a state within the contact band cannot be promoted to a robust collision-free result. Finite state/edge sampling is numerical validation, not a mathematical continuous-collision certificate.

## Roadmap source audit

`RoadmapOptions.offline_voxel_resolution` is a **half-side**: `roadmap_builder.cpp` divides workspace extents by twice that value. Offline voxel spheres use `GetOfflineVoxelRadius()`, whereas online `Plan` receives the actual sphere radius and its lazy edge checks use `online_edge_step_size`. `BuildCollisionSet` hashes online centres into the offline map; neither the map nor node-validity checks establish continuous path safety. The new runner must therefore validate the returned actual path and retain failures. Map extent/centre, grid side, online radius, edge steps, and build time are separate protocol values. Offline map construction is never an online planning time.

## Reproducible commands

Run from the repository root, preserving the existing Conda environment:

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.model
bash local/python.sh -m pytest benchmarks/generic_motion_v1/tests/test_model_consistency.py -q
```

The model tests passed (8 tests). Existing Drake emits a deprecation warning from the unmodified upstream bridge's `is_floating()` call; the call currently works. No upstream CUDA, example, URDF, environment, or previous result was edited.
