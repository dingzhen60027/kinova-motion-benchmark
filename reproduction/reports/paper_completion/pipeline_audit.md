# Paper-aligned geometry pipeline audit

This audit reads the paper's sections V-B, VI-A/B/C and Appendices B/C/D against the released native implementation. All historical inputs/results remain unchanged under `reproduction/paper_protocol` and its v3 run. Fixes are confined to the new `reproduction/paper_completion` package.

## Implemented corrections

1. **Online target IK (V-B).** A scene now fixes its original sampled start and target end-effector pose. It no longer gates the whole scene on an IK solution warmed from an unrelated random configuration. For each roadmap, online IK selects nodes after the native collision set, ranks their recorded poses, independently checks candidates, and solves the original position-and-orientation IK task. The metric is the released native metric: translation norm + 0.2 × relative rotation angle. The maximum of 10 IK attempts and lack of translation cutoff remain explicitly declared unpublished reconstruction choices. Every candidate, failure, selected goal, timing, raw goal and float32 representation is saved in `online_pose_ik.json`. No start-component restriction or task replacement is used to force success.
2. **Endpoint representation.** `endpoint_audit` compares the original double configuration with the exact float32→double round trip, including CUDA, independent collision, domain, and joint error checks. This only defines the representation at the native boundary. It does not displace an endpoint or relax collision tolerances. A colliding canonical endpoint remains invalid. IK uses the separately declared 1e-5 m positive collision margin in the new model adapter and must retain the original pose tolerances.
3. **Multiple paths (VI-C).** A native no-alternate-path exception now terminates only additional-path construction. It preserves the first valid corridor and proceeds to LGCS optimization, as VI-C specifies. The no-path event is explicitly saved. Other unexpected exceptions are still raised. Alternate paths require CUDA plus independent validation; Forest also requires exact line-circle checks.
4. **Counterexample recovery (VI-B).** Returned float32 halfspaces are represented with their measured seed residual and an outward double ULP. Path coverage is tested with zero halfspace slack, avoiding acceptance of a 1e-6 gap. After native edits, all original seed segments are retained; both paths are restored when VI-C provided a second path.
5. **Native edit input.** The old SCSTO failure occurred inside `EditRegionsCuda`, not the author trajectory solver. Native `HPolyhedron::PointInSet` uses 1e-9 tolerance; converting a double region to float32 can discard a seed. The new adapter checks containment using that actual native predicate. Only if it fails, it outward-rounds the bound using the standard dimension-dependent float32 dot-product error bound, verifies the seed in native code again, and records the maximum change. An alteration above 1e-5 is rejected. Collision checking tolerances remain unchanged and every resulting trajectory is independently validated.
6. **Native defaults.** The prior adapter reduced native lazy planning attempts from 100 to 20 and A* expansion limit from 100000 to 20000, neither specified by the paper. The new adapter keeps upstream defaults: 100 attempts, 100000 expansions, 50 endpoint steering attempts and 10 blocked outgoing edges before discarding a node. The published lazy step remains 0.1; shortcutting is enabled and voxel padding is zero. `environment.native_planner_options` captures these runtime values.
7. **Failure evidence.** Before every optimization, the actual halfspaces, seed edges, original/second paths and recovery history are saved as `pre_solve_*`. Thus a Gurobi license exception retains the concrete mathematical input needed for later replay.

## Actual fixed-input regressions

Command (existing Conda environment):

```bash
bash local/python.sh -m reproduction.check_completion_pipeline > reproduction/reports/paper_completion/diagnostics/pipeline/final_regressions.log 2>&1
```

The driver uses the frozen old v3 FR3 inputs, configuration dictionary, seed path and inflation seed 1337 with the new package adapter. The final targeted regression completed at 17:16:54 local time, before the new package limit asset changed at 17:20:14; its driver now explicitly pins the unchanged old asset path so later replay retains those old limits. Historical input/model-limit records are never rewritten. The final main run separately uses the prepublication official joint-limit asset; these targeted regressions are not pooled with it. The driver does not regenerate a case to obtain success.

| Fixed historical case | Earlier failure | Current actual result |
|---|---|---|
| e00_r09_n12000 / LGCS | Native second-route no-path exception | Second route still fails; first route retained; real Gurobi solve and 1344-sample independent/CUDA validation pass; cost 6.70158347 rad |
| e09_r07_n06000 / SCSTO | `EditRegionsCuda`: line start not contained in region 0 | Real author SCSTO, one counterexample-recovery iteration, 2001-sample independent/CUDA validation pass; duration 6.36628486 s |

For the SCSTO case, maximum outward float32 halfspace bound adjustment was 3.09944153e-6 in unnormalized constraint units. This is explicitly reported numerical region conversion, not a collision clearance or success threshold. Pre-solve input artifacts were verified present in the final regression. Both old failure records are embedded in the new regression JSON; the first attempted repair's missing explicit Python binding tolerance error is retained in `diagnostics/pipeline/failed_attempt01`.

The fixed old e08 target remains an endpoint failure: original double passes independent and CUDA checks, float32 canonical representation passes CUDA but fails independent checks, with joint L2 representation error 2.13920636e-7 rad. It is not silently moved or relabeled.

## Native DRM failures remain a substantive limitation

All 400 old v3 FR3 case logs were read. Exactly 197 native failures contain `Path does not exist.`; 8 more exhausted steering/lazy attempts. The 197 graph failures are distributed as e00: 5, e01: 40, e02: 40, e04: 40, e05: 32, e07: 40. The remaining 8 are all e05.

Command for one preselected graph-failure probe:

```bash
bash local/python.sh -m reproduction.check_completion_pose_ik > reproduction/reports/paper_completion/diagnostics/pipeline/pose_ik_probe.log 2>&1
```

On **e01_r01_n03000**, keeping the exact old target pose and obstacle centers, the native collision-set-pruned adjacency graph has two connected components of **1174 and 1161 nodes**. The nearest available node to the start is in the first component; the nearest node to both the old random-IK goal and the new pose-map-IK goal is in the second. New online pose IK succeeds on its first candidate, but native planning still reports no path. Its full IK result, graph component sizes and node IDs are saved in `diagnostics/pipeline/pose_ik_e01_r01_n03000/result.json`.

Before adding any online obstacles, the same 3000-node static roadmap already has two components of **1512 and 1488 nodes**, with 18,848 undirected edges. The same start/old-goal/new-goal nearest-node component distinction already exists there. Joint 7 ranges are [-2.28043, -0.87661] in the first component and [0.90624, 2.32222] in the second; the other six joint ranges largely overlap. These statistics are saved in `static_connectivity.json` and `static_component_coordinates.json`.

This demonstrates a concrete static-roadmap connectivity limitation and prevents claiming that online pose-map IK repairs all 197 failures. The finite graph alone does **not** prove that the true continuous configuration space is disconnected. No collision filters, target poses or graph-component IK restrictions were added to obtain favorable benchmark results.

## Independent official-mesh diagnostic

The additional CPU-only command was actually run:

```bash
bash local/python.sh -m reproduction.diagnose_fr3_geometry > reproduction/reports/paper_completion/diagnostics/pipeline/mesh_comparison.log 2>&1
```

It retains the same kinematic chain, removes sphere collision geometry from a separate diagnostic Drake plant, and registers the convex hulls of the pinned official **link5** and **hand** collision STL meshes. Only the implicated body pair is queried. No benchmark plant or filter is altered.

At the fixed conventional home configuration `[0, -pi/4, 0, -3*pi/4, 0, pi/2, 0]`, the sphere model reports **-0.0351577 m** for `fr3_link5_sphere_1` versus `fr3_hand_sphere_0` (and -0.0253943 m for link5 sphere 3 against the same hand sphere). The official link5/hand mesh convex hulls at precisely that configuration have **+0.0370481 m** separation. At the fixed old roadmap node 2922, sweeping only joint 7 over 31 predetermined values from -1.5 to +1.5 rad gives sphere collisions at 22 values; the two official mesh convex hulls are separated at all 31 values.

Because a convex hull contains its underlying mesh, positive separation between these hulls proves that this **specific body pair** is not colliding for the official collision meshes at those configurations. The fitted spheres thus introduce demonstrable false collision obstacles for this pair. This does not certify the entire robot, other body pairs, continuous paths, or the authors' unpublished geometry. In particular, silently excluding that pair would conceal the reconstruction limitation and is not done.

The joint scan CSV, all collision geometry names and signed distances, converted OBJ mesh inputs and the full raw JSON are preserved under `diagnostics/pipeline/mesh_comparison/`. The next model-fidelity task is a verified collision approximation preserving relevant mesh separation; it cannot be replaced by reporting improved solver success or by changing filters based on benchmark outcomes.

## Evidence and boundaries

- Fixed-input raw regression arrays and logs: `diagnostics/pipeline/final_regressions/`.
- All 400 historical log classifications: `diagnostics/pipeline/final_regressions/historical_failure_classification.json`.
- Boundary goal audit and test summaries: `diagnostics/pipeline/final_regressions/regression_summary.json`.
- Online IK/connectivity probe: `diagnostics/pipeline/pose_ik_e01_r01_n03000/`.
- Unsuccessful first repair attempt: `diagnostics/pipeline/failed_attempt01/`.

These are targeted adapter regressions and one diagnostic planning probe, not new 400-case paper results. Dense collision validation is numerical, not a continuous collision certificate. Main-run results must be generated separately after freezing the full new package.
