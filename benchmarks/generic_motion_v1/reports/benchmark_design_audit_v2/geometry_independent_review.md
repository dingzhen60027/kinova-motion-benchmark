# Independent review of geometry probe v3

**Accepted within its stated finite-diagnostic scope.** The independent review passed all checks recorded in [geometry_independent_review.json](geometry_independent_review.json). It did not invoke CUDA, a collision engine, or a planning method. [Actual output](geometry_independent_review_v3.log) and [review script](geometry_independent_review.py) are preserved.

The reviewed execution is [geometry_run_v3/report.json](geometry_run_v3/report.json), not the earlier v2 report. During review, two v2 evidence limitations were identified: source-model collision candidates were compared by count only, and unchanged straight-line tests did not save their full sample arrays. The coordinating implementation addressed both in a new v3 run, preserving v2. This review's initial attempt overlapped that source update and failed before completion because its lightweight canonical metadata object lacked the newly required inspector field; the [initial failure log](geometry_independent_review.log) and [failed review source](geometry_independent_review_failed_source.py) are retained. The final v3 reviewer reads raw arrays only.

What was checked:

- All v3 artifact checksums, all bound scene/config/manifest/source hashes, and canonical URDF/directive asset hashes match. The original robot/source geometry files are unchanged.
- All 2,048 full-joint base-world proposals exactly reproduce from PCG64 seed `2026103001`. The 128-state pool is exactly the first 128 proposals satisfying both base collision classifications and the 0.1 mm clearance threshold, before any added obstacle is examined. Selection is independent of all planners and the scene-specific results.
- The same pool appears identically in all 109 scene NPZ files. Every source-free count, cover-free count, added collision count, containment-miss count, CUDA/Drake disagreement count, endpoint count and minimum clearance was independently reduced from saved arrays. All 926 accepted endpoints exactly match the 463 valid frozen cases and pass the saved source and cover predicates.
- The source-model code loads the same `kinova_sens_on_table.yml` and package assets as canonical `Model`, adds only the manifest source spheres/boxes as world geometry, retains the same joint order/limits, and uses native Drake filters. V3 explicitly compares the **set of geometry-name pairs**, including every source obstacle versus exactly the canonical moving-robot name set, rather than cardinality alone. Every scene reaches the saved `source_collision_pair_identities_verified=true` receipt. This review inspects that construction and its bound source; it does not independently rebuild all models.
- All 463 straight-line NPZ files contain exact frozen-endpoint interpolation at both 0.04 and 0.005 rad, with matching mask shapes. All four masks (base/cover × coarse/fine) independently reproduce the case flags and aggregate summaries. Cover-free samples never contradict base collisions. The original stored labels, fresh coarse labels and fine labels all agree; no label changed.
- No planning library is imported or called by the probe workflow. `Model.cuda` and `Model.clearance` are collision functions; the probe never constructs DRM, EI-ZO, OMPL or a trajectory optimizer. Source geometry is only diagnostic and does not replace the benchmark's operational cover.

Interpretation constraints:

1. The 128-state pool is reused across scenes. Its 13,952 scene-state tests are paired diagnostics; they are not 13,952 independent environments or independent planning tasks. Aggregate percentages must not be assigned an independence-based confidence interval.
2. `test-route_choice-011` has 80 source-free and zero cover-free pool samples; `test-dense_clutter-009` has 17 versus zero. These demonstrate sampled exclusions introduced by the cover. Zero out of 128 is **not** proof of an empty free space, nor does it prove any start-goal planning problem infeasible.
3. The unchanged straight-line labels are evidence at the two evaluated numerical grids. Drake directly checks any newly detected collision witnesses, but there were no changed labels in this run. It is incorrect to describe all finer samples as independently classified by Drake or as continuously certified.
4. A fixed-world collision combines robot self-collision and collision with fixed equipment. `fine_base_blocked` does not isolate those causes. `fine_added_obstacle_only_blocked` counts lines newly blocked after adding the obstacle cover; it does not count every line that touches an added obstacle, because an already-base-blocked line may also touch one.
5. Existing v1 test exposure remains a separate statistical limitation. This new probe inspects inputs and geometry only and produces no new planner score.

Reproduce the review from repository root using the existing Conda launcher:

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.reports.benchmark_design_audit_v2.geometry_independent_review
```

The script intentionally refuses to overwrite its existing review JSON. For subsequent execution, use a separately versioned copy/output; do not replace this evidence.
