# Frozen v1 data-design audit

This is a new, read-only CPU audit at HEAD `c8b91f310`. It does not run a planner, invoke a collision engine, regenerate a dataset, or edit any frozen input or historical result. The actual command completed in 6.30 seconds; **8,617 / 8,617 integrity, RNG-replay and arithmetic checks passed**. Passing these checks is not a claim that the benchmark has no design limitations.

Evidence: [machine-readable audit](audit.json), [individual checks](checks.json), [actual command output](run.log), [CPU audit source](audit.py), [sphere-cover computation](geometry_effects.py), [sphere-cover rows](sphere_cover_extent.json), [sphere-cover log](geometry_effects.log).

## What was independently recomputed

- All 490 registered rows, all referenced generation receipts and input-file hashes, row hashes, 109 scene records, and archived original source hashes.
- PCG64 proposal values for all seven joints, deterministic scene/endpoint/method/solver seed derivation, first acceptable endpoint pair, requested distance bin, rejection accounting and exhaustion. This uses the stored CUDA/Drake classification arrays; it does **not** assert freshly measured collision equivalence.
- The final accepted source-geometry proposal from each seed and recorded proposal index, followed by exact reconstruction of all saved voxel centres and coverage metadata. The fixed-equipment rejection oracle itself was not rerun.
- Joint limits, endpoint distances, all stored numerical straight-line sample positions and their collision flags. There are 61,011 saved straight-line samples and 146,816 saved endpoint proposals.
- Cross-split exact duplicate scenes, individual endpoints and unordered endpoint pairs. No unintended exact duplicate was found. Open controls intentionally share one physical base world.
- Existing formal-test output inventory: 24 runs and 9,900 saved results. No new formal run was performed.

## Registered, usable and blocked input counts

“Blocked” below means that collision was detected on the saved joint-linear interpolation at a 0.04 rad sampling step. A clear label is finite-sampling evidence, not a proof of continuous collision freedom; a blocked label does not prove the planning problem infeasible.

| Split / family | Registered | Valid | Straight-line blocked / valid | Valid scene IDs | Distinct valid obstacle geometries |
|---|---:|---:|---:|---:|---:|
| smoke, all | 10 | 8 | 6 / 8 | 4 | 4 |
| dev, sparse_clutter | 15 | 15 | 14 / 15 | 5 | 5 |
| dev, dense_clutter | 15 | 15 | 14 / 15 | 5 | 5 |
| dev, bottleneck | 15 | 15 | 11 / 15 | 5 | 5 |
| dev, route_choice | 15 | 15 | 8 / 15 | 5 | 5 |
| dev, all | 60 | 60 | 47 / 60 | 20 | 20 |
| test, sparse_clutter | 100 | 95 | 68 / 95 | 19 | 19 |
| test, dense_clutter | 100 | 85 | 74 / 85 | 17 | 17 |
| test, bottleneck | 100 | 100 | 74 / 100 | 20 | 20 |
| test, route_choice | 100 | 95 | 65 / 95 | 19 | 19 |
| test, open_control | 20 | 20 | 9 / 20 | 4 | 1 |
| test, all | 420 | 395 | 290 / 395 | 79 | 76 |

The 25 invalid test rows are **five entire scenes × five requested pairs**: one sparse, three dense, and one route-choice layout. Every one has `q_start=null`: all 4,096 sampled configurations were rejected before finding even a first valid state. They are not failures to sample a long-distance goal. The smoke dense-clutter scene likewise yields two invalid rows with no first endpoint. These observations do not establish that those scene free spaces are mathematically empty.

The user's previous decision to preserve v1 means that these 27 generation failures remain in the registration ledger and are excluded from the legal-planning denominator. They are not replaced or relabelled as planner failures. Among valid test cases there are **75 random obstacle layouts plus one shared base-only geometry**, not 395 independent environments or 79 independent physical worlds. The existing formal report already separates the 20 open-control pairs and avoids a scene-generalization confidence interval for them.

## Design findings that matter for subsequent research

### 1. The test has already been exposed to planning results

All 395 valid v1 test problems have historical planning outputs. The 24 formal runs were performed on 2026-10-28: 9,300 legal planning calls and 600 invalid-input exits, 9,900 registered outputs in total. The explicit execution record is [formal completion](../../benchmark_v1_formal_completion.md); the new audit independently counted the result files and their case IDs. Historical test results precede the later 2026-10-29 baseline-repair and 2026-10-30 extension commits.

This establishes **test exposure**, not proof of deliberate test-driven tuning. It means v1 cannot be represented in future work as an untouched final holdout. Existing comparisons remain useful, provided source versions and the exposure chronology are stated. Any future genuinely held-out generalization claim needs a newly frozen, method-independent suite after the new method and its tuning protocol are fixed. Do not silently regenerate or replace v1.

### 2. Scene names do not establish the intended C-space challenge

The bottleneck generator places two finite 0.3 m × 0.065 m × 0.4 m walls with a random in-plane gap and yaw. Endpoints are sampled independently in all seven joints. No condition makes the robot traverse the gap, and there is no geometric barrier excluding travel around or above these finite walls.

A deliberately limited diagnostic finds only **1 / 15 dev pairs and 18 / 100 test pairs** with TCP endpoints strictly on opposite sides of the walls' normal plane. This does not prove the remaining arm motions avoid the walls, nor does the plane-side test establish C-space connectivity. It does show why the label cannot by itself be used as evidence of mandatory narrow-passage planning.

Route-choice consists of two groups of six source spheres with randomized centres and radii. All 26 registered route-choice layouts have separated groups (the smallest cross-group source-sphere surface gap is 0.0482 m). Within each group the source spheres form one or two connected components; 10 of the 26 layouts have one group with two components. There is no requirement that endpoint motion confront these groups or that two C-space routes exist. “bottleneck” and “route_choice” remain **workspace geometry labels only**.

### 3. The actual planning geometry is materially larger than the drawn source spheres

All methods use the same conservative union of radius-0.04 m voxel balls. This prevents gaps in the cover, but coarse conservatism matters because the source clutter spheres have radii as small as 0.025 m.

For all **1,173 source spheres**, a new analytic CPU calculation of the furthest point of the finite ball union finds outward excess of **0.03465–0.05943 m**, and outer radial reach of **1.822–2.464 × the source-sphere radius**. These are maximum radial extents, **not** uniform-radius inflation or volume ratios. A source-geometry rendering alone therefore understates what the planner avoids.

For all 26 bottleneck layouts, source gaps are 0.2212–0.3355 m, while the projected gap of the actual ball cover is **0.1787–0.2930 m**, a reduction of approximately 0.0425 m. The cover is shared by all methods, so this is not a method-specific fairness error. It is a substantial environment-definition issue: reports and replay should identify the ball union as the operational collision geometry. A finer-cover sensitivity study belongs in a separate version, not a modification of v1.

The coordinating audit also executed a separate [fresh collision probe](../geometry_run_v2/report.json), using the same 128 base-free configurations per scene. It observed source-geometry versus cover-free counts of **80 versus 0** in `test-route_choice-011`, and **17 versus 0** in `test-dense_clutter-009`; across the five dev scenes per family, dense was **381 versus 213 / 640** and sparse **456 versus 303 / 640**. These are finite probe samples, not a proof of empty cover free space or an estimate based on independent scene draws. They directly demonstrate that the conservative representation can reject configurations free in the source geometry. That separate probe, unlike this local design script, performed new collision calls. Its implementation and raw evidence should be read for its exact scope.

### 4. Dev and test have different prespecified distance mixtures

The public three-bin cycle is short / medium / long. Three pairs per dev layout gives **20 / 20 / 20** valid pairs. Five pairs per test layout gives **158 / 158 / 79** valid pairs: 40% / 40% / 20%. The normalized-distance mean is **0.3800** in dev and **0.3478** in test. In addition, dev has no open-control pairs, while test has 20.

This is deterministic implementation of the frozen configuration, not hidden method-based selection. It does mean aggregate dev success rates do not directly estimate aggregate test rates. Subsequent reports should give distance-bin × family strata and declare any standardized weighting before examining new-method results. A future version can explicitly balance the mixtures rather than inheriting the five-pair cycle accidentally.

### 5. The generation selection rule is method-independent, with limited diversity

The saved proposals independently replay to the **first** endpoint pair satisfying collision, clearance, joint-limit and requested-distance checks. Scene generation only rejects overlap with fixed equipment. The generator source imports no planning algorithm; no method's success is used in these rules. This is a strength of v1.

However, dense clutter is entirely spheres, sparse clutter mixes spheres and boxes, and the structured families use only two finite walls or two six-sphere groups. The structured-family centre ranges are hard-coded in `scene.py`, independent of the generic JSON ROI. Exact replay confirms the saved values; it does not make the environment a comprehensive distribution of manipulator scenes. Samples are uniform in a bounded seven-joint box, conditional on the stated acceptance rule, rather than uniform over reachable workspace or globally feasible planning problems.

### 6. Current source and original generation source are not identical

All frozen configurations and data files still match their original hashes. Five current source files differ from generation-time hashes: `generator/audit.py`, `generator/endpoints.py`, `generator/manifest.py`, `generator/voxelize.py`, and `model.py`. Their exact originals remain in the explicit `qualification/frozen_v1_source` archive. The final geometry and voxelization are reproduced exactly with current pure CPU code, and all stored endpoint selections replay exactly.

This is documented source evolution, not evidence of changed data. Reproduction should cite the generation receipt and archived source, not claim that the current Git HEAD generated the frozen dataset. All 490 rows retain the same robot and collision-geometry identities.

## What this audit does and does not authorize

The frozen v1 data are internally consistent under the checks above and remain useful for controlled algorithm development, regression and transparent comparison. This audit does not establish global feasibility, continuous collision safety, practical dynamic execution, public-benchmark representativeness, or a hidden final-test protocol. It does not justify deleting difficult cases or adjusting endpoints for a method.

For subsequent research, preserve v1 and its exposure history; report family, distance and straight-line strata; retain the one-world status of open_control; identify source versus operational obstacle geometry; and separately plan an external public benchmark plus a new held-out version. Do not infer scene difficulty from a family name or from one planner's success.

## Actual commands and reproducibility

Run from the repository root using the existing Conda launcher:

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.reports.benchmark_design_audit_v2.local_design.audit \
  > benchmarks/generic_motion_v1/reports/benchmark_design_audit_v2/local_design/run.log 2>&1
bash local/python.sh -m benchmarks.generic_motion_v1.reports.benchmark_design_audit_v2.local_design.geometry_effects \
  > benchmarks/generic_motion_v1/reports/benchmark_design_audit_v2/local_design/geometry_effects.log 2>&1
```

The first command refuses to overwrite this evidence. To repeat it, provide a new directory, for example `--output /tmp/benchmark_design_reaudit_20261030`. The sphere-cover command verifies an existing matching output without replacing it; its optional `--output` accepts a new file. Neither command constructs CSDecomp/Drake models or invokes CUDA, solvers, or planners. Source hashes, exact IDs, all distributions, individual checks, and historical run inventory are in `audit.json`.
