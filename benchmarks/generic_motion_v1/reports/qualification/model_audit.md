# Canonical Kinova model qualification

This audit qualifies the implemented Kinova benchmark model and its numerical collision interface. It does not establish physical robot fidelity, exact Franka/FR3 reproduction, continuous trajectory safety, or correctness for every configuration. The same agent implemented the original adapter and these additional tests: the oracle implementations are separate code paths, not an independent institutional replication.

The current receipt is [`model_audit_current.json`](model_audit_current.json), with every sampled state, edge and boundary centre in [`model_audit_current_raw.npz`](model_audit_current_raw.npz). The complete actual invocation log is [`model_audit_current.log`](model_audit_current.log). The previous qualification receipt, original G0, failed prototype log and failing regression log remain present. No frozen manifest, canonical configuration, third-party source, CUDA algorithm, or historical attempt was overwritten.

## Model identity and independent construction

The canonical source remains `csdecomp/tests/test_assets/directives/kinova_sens_on_table.yml`, loaded into Drake and converted into CSD. Offline roadmap construction and online planning use this same model. The seven generalized positions are `kinova_joint_1_q` through `kinova_joint_7_q`, in that order.

| Quantity | Actual audit result |
|---|---:|
| Links, including world | 29 |
| Collision primitives | 20 boxes |
| Moving / fixed primitives | 15 / 5 |
| Checked / excluded base collision pairs | 137 / 53 |
| Joint states used for full FK comparison | 67 |
| Source box dimension and local transform discrepancy | 0 |
| Manual XML vs Drake maximum matrix element discrepancy | 5.552e-16 |
| CSD vs Drake maximum geometry rotation matrix element discrepancy | 2.771e-7 |
| CSD vs Drake maximum geometry translation norm discrepancy | 2.219e-7 m |
| CSD rotation orthogonality residual | 4.769e-7 |
| Nearest polar rotation angular discrepancy | 2.466e-7 rad |
| Float32 vs source joint limit discrepancy | 9.537e-8 rad |

`ManualURDF` parses the XML dimensions, local origins, joint axes and limits directly, applies explicit directive weld transforms, and computes NumPy forward kinematics. It compares all link poses, all collision box poses and the complete TCP transform, including rotation. A separately constructed Drake plant supplies the second FK and distance implementation. The independently reconstructed fixed-component/direct-adjacency/world-world filter sets match both Drake and CSD exactly in this model. The test checks the world frame and the table, wall, both camera tripods, robot base and fixed finger assembly; it does not replace their geometry.

The historical reduced/full comparison remains in `reports/model_audit/audit.json`: reduced/full have 20/29 links, but both have the same 20 box dimensions, 15 moving boxes, 137 checked pairs and limits. Across the original 64 FK states their maximum full transform discrepancy was 3.577e-7. Different URDF file names therefore did not by themselves demonstrate different collision geometry. New benchmark maps nevertheless use only the canonical full-model construction and are hash checked.

Frozen model identity is unchanged:

- Robot: `a25f447c8caf4aff6dc76196c3d57cc940f00e802367f48034f06a9973927c75`.
- Collision geometry: `896ab9f55c5d8081b843358bd85cde2daae1f432564593f86218bf61733ce4fa`.
- Current guarded adapter source: `5b152e3e5ca484e5378856de11747c45826653df8a1183582fe90e0958fc5116`.

## Collision tests and actual development geometry

Randomness is NumPy PCG64 with seed `20261027931`. For the independent online oracle, each 0.04 m sphere is genuinely registered in a fresh Drake SceneGraph; its distances do not reuse the adapter's analytic sphere/box formula. Base collision distances also use Drake's checked pair set. Online spheres are static world geometry: robot–sphere pairs are checked, fixed-world–sphere overlap is a separate scene-validity predicate.

| Probe set | States | CUDA / Drake free | Label disagreements |
|---|---:|---:|---:|
| Base random full 7-joint states | 512 | 306 / 306 | 0 |
| Base, 12 independently sampled edges × 25 states | 300 | 176 / 176 | 0 |
| Same random states with 24 spheres | 512 | 273 / 273 | 0 |
| Same edges with 24 spheres | 300 | 156 / 156 | 0 |
| Actual sparse dev layout, 207 spheres | 65 | 38 / 38 | 0 |
| Actual dense dev layout, 692 spheres | 65 | 22 / 22 | 0 |
| Actual bottleneck dev layout, 320 spheres | 65 | 51 / 51 | 0 |
| Actual route-choice dev layout, 434 spheres | 65 | 46 / 46 | 0 |

Each actual layout uses the first frozen case of that family, 32 fresh uniform states and 33 states on its unchanged start-to-goal edge. Scene IDs, hashes, actual voxel extrema and full voxel arrays are saved. These checks use real generated obstacles within the benchmark ROI, not solely arbitrary obstacles. They cover four of the 20 dev layouts and do not replace the separate full manifest audit. The largest adapter versus native Drake signed-distance difference in these four scenes was 1.111e-16 m. A separate 20-probe fixed-world sphere check, including interior and exterior points, differed by at most 2.776e-17 m.

The 306 independently collision-free base states yield sampled moving collision bounds from `[-0.5095, -1.0378, 0.0053]` to `[1.0305, 0.9947, 1.3643]` m. These are empirical bounds for checking the obstacle ROI, not a proof of pointwise reachability.

## Contact discrepancies are retained

The acceptance band remains 2e-6 m. It is an empirical policy for distinguishing near-contact floating-point disagreement; it is **not** a derived global numerical error bound and must not erase false-free observations.

| Boundary probes | Probes | False free | False collision | Maximum false-free penetration | Maximum false-collision positive clearance |
|---|---:|---:|---:|---:|---:|
| 12 oracle-bisected static transitions, both sides | 108 | 3 | 11 | 4.025e-8 m | 6.424e-8 m |
| Moving-box sphere face/edge/corner contacts | 630 | 11 | 5 | 1.971e-8 m | 2.142e-8 m |

There were zero observed disagreements outside the band. For the static probes, converting only the configuration to float32 changes the native Drake clearance by up to 8.961e-8 m. CSD transforms additionally exhibit the rotation and translation errors reported above. These are measured maxima over these probes, not universal upper bounds.

The sphere experiment covers all 15 moving boxes, face/edge/corner features, both signs and seven offsets. A dedicated native Drake pair-distance oracle distinguishes the targeted contact from the global minimum: 280 target contacts are exposed and 350 are masked by another collision. Both targeted and global distances are retained. This matters because the original G0 only tested one finger geometry: its 66 aggregate boundary checks contained 6 positive and 60 negative clearances, so an all-negative masked contact could otherwise appear to validate the intended boundary. Sphere/box and box/box upstream predicates also differ in exact-tangent inequality conventions. The final trajectory validator must continue using the independent checker and positive-clearance policy; native CUDA “free” near contact alone is insufficient evidence.

## Defects, repairs and limits

**Repaired adapter input defect.** Before the change, a finite float64 voxel coordinate `1e300` was accepted and became float32 infinity. The original qualification JSON records `rejected=false`, nonfinite stored centres and the cast warning; it never sent this malformed scene to CUDA. The targeted regression failed before the repair. The new adapter validates finite scalar protocol radius, strict N×3 voxel shape and float32 representability before conversion, and commits state only after successful construction. Invalid arrays cannot replace the previously valid scene. Configuration arrays receive the same representability guard before CUDA. These are input guards; valid frozen scenes retain identical float32 centres, 0.04 m radii, collision pairs and geometry hashes.

The actual CPU model test run passed **32 tests** in `model_qualification_pytest_after.log`, including malformed shapes, NaN/infinite values, finite overflow, invalid radius types, state preservation, default empty input, float32 quantization, manual FK and native signed-distance signs. The new G0 reran all numerical probes on the repaired adapter and again matched the frozen model inventory. Its receipt carries the current source SHA; the original G0 and original configuration are not rewritten to manufacture a match.

**Confirmed upstream CPU-wrapper defect; not modified.** In `csdecomp/src/pybind/csdecomp/collision_checker_bindings.h`, `rob_geom_ids(robot_geometry_indices.size())` creates an already-sized vector, after which the code appends the actual IDs. For this model that prefixes 15 copies of geometry zero, incorrectly adding the fixed robot base to CPU voxel checks. At three saved sphere centres `[0,0,0.02]`, `[0,0,0.04]`, `[0,0,0.08]`, the same configuration is CUDA-free and independent-Drake-free with +0.019025 m robot clearance, but `CheckCollisionFreeVoxels` reports collision. Those deliberately constructed spheres overlap fixed base geometry and are invalid benchmark scenes, so this is not evidence of affected retained cases. The benchmark uses CUDA and its independent Drake adapter, not that CPU fallback. Any future CPU fallback must repair and qualify the wrapper before use.

**Remaining scope.** Box primitives are the repository's collision approximation; this audit does not certify their physical envelope against every visual mesh or actual hardware. It does not exhaust configuration space, prove continuous collision freedom, or establish C-space homotopy. Boundary sampling is biased toward deliberately constructed transitions and is not a generalization-error estimate. The sample bands and raw near-contact failures must remain visible in downstream reports.

## Reproducible commands and evidence

Commands actually run in the existing Conda environment from the repository root:

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.model_audit --output benchmarks/generic_motion_v1/reports/qualification/model_audit_current.json > benchmarks/generic_motion_v1/reports/qualification/model_audit_current.log 2>&1
bash local/python.sh -m pytest -q benchmarks/generic_motion_v1/tests/test_model_consistency.py benchmarks/generic_motion_v1/tests/test_model_qualification.py > benchmarks/generic_motion_v1/reports/qualification/model_qualification_pytest_after.log 2>&1
bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.model_lifecycle_readonly_probes --output benchmarks/generic_motion_v1/reports/qualification/model_lifecycle_readonly_before.json > benchmarks/generic_motion_v1/reports/qualification/model_lifecycle_readonly_before.log 2>&1
```

For a new audit invocation choose a fresh `--output` file. The auditor refuses to overwrite an existing receipt or its raw arrays. No environment rebuild or dependency replacement was performed.

## Additional lifecycle review

The read-only review confirmed that current pairing rejects duplicate case/roadmap/method-seed units and inconsistent scene clusters, open-control rows share one physical scene cluster, and offline statistics resolve the precise map metadata path. The bootstrap retains all repeated jobs from each scene together; four-layout pilot comparisons still have only four independent layout clusters. Common-success cost differences do not support unconditional ranking across unequal success sets.

Two further issues were reproduced using explicitly synthetic CPU fixtures, not planning results: a sealed result with a conflicting method/case identity was accepted; and a conflicting embedded `result.validation.cost=999` overrode the separately validated `validation.json` cost of 2. A real short-lived synthetic child also demonstrated that unconditional coordinator metadata replacement hid the worker's conflicting identity. These records are preserved in `model_lifecycle_readonly_before.json`. The parent subsequently added identity-conflict rejection with preservation of the worker output, result/job identity verification, and preference for independent validation evidence. Follow-up verification is recorded separately so the failing evidence remains intact.

The actual CPU rerun `model_lifecycle_readonly_after_v2.json` confirms that the conflicting sealed result is rejected, the reported cost is 2, and the synthetic worker returns `infrastructure_error` with its original wrong identity preserved. The earlier `after.json` probe looked only for the generic provenance rejection filename, so it incorrectly reported preservation as false; the identity-specific preserved file is now checked and included in full in `after_v2.json`. This instrumentation correction changes no experiment. The actual follow-up command was:

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.model_lifecycle_readonly_probes --output benchmarks/generic_motion_v1/reports/qualification/model_lifecycle_readonly_after_v2.json > benchmarks/generic_motion_v1/reports/qualification/model_lifecycle_readonly_after_v2.log 2>&1
```
