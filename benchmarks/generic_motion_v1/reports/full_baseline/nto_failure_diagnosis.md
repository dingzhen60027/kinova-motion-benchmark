# NTO failure diagnosis: partial sealed observations

Audit time: 2026-10-28 approximately 03:23 UTC, while dev was still running. These are **partial observations, not final rates**. Selection comprises three smoke optimizer/candidate failures plus the first ten sealed dev optimizer/candidate failures in manifest order. Frontend failures were not selected. Planning source: `72984027030175b7bca78e2dcf592b38f0c1864b`. No planner, solver, GPU collision check, or runtime-source modification was used for this diagnosis.

## Evidence-supported conclusion

No clear control-point transposition, knot scaling, endpoint substitution, solver-selection, or candidate-reconstruction bug was found in these thirteen records. Two distinct failures occur:

1. SNOPT returns an unsuccessful candidate: `info=31` / Drake `kIterationLimit`, or `info=13` / `kInfeasibleConstraints`. Those candidates have substantial collision-constraint violations and are correctly retained as `solver_failed`. This is not proof that the motion-planning task is infeasible.
2. The finite nonlinear program succeeds, but the returned curve collides between collocation locations. SNOPT `info=1` and IPOPT `status=0` are not collision-free trajectory certificates. The denser final validator correctly rejects these candidates.

This supports the observed classifications. It does not establish that this reconstruction is the author's exact NTO implementation or the strongest available NTO baseline.

## Adapter checks

All thirteen inspected records preserve the original DRM path element-for-element in `nto_initial.npz:original_path`. Distance uses SNOPT, order-two B-splines, and original DRM controls. Time uses IPOPT, order-four splines, and one extra copy at each endpoint. No control resampling or minimum-control-count refinement occurs. The enforced collision grid is the declared `k/150, k=1,...,149` author-comment convention. Published prose says 150 points, so this discrepancy remains disclosed.

The registered-sphere pair count is `137 + 15 × number_of_voxels`, matching the canonical model. Independent SciPy `BSpline` evaluation agrees with stored Drake samples to at most `1.7763568394002505e-15 rad` for these thirteen records. Endpoint residuals are at most `5.28e-13 rad`. Joint-limit checks pass, and time candidates pass derivative bounds. These failures are not caused by observed endpoint or derivative-scaling errors.

Every inspected initial DRM path passed the shared finite-resolution geometry gate. Minimum recorded seed clearances range from `0.00115638 m` to `0.01902503 m`. Those checks are numerical, not continuous proofs.

Installed Drake documentation confirms that `KinematicTrajectoryOptimization(trajectory)` sets the supplied spline as its initial guess. The adapter calls that constructor, not the zero-initialization overload. `AddPathLengthCost` sums control-point distances, which equals PWL length for this order-two distance spline.

## Selected actual records

`D` means minimum distance / SNOPT; `T` means minimum time / IPOPT. Solve time excludes frontend and final validation. Collision counts are recorded native / independent counts.

| Run / case | Objective | Native status | Final status | Solve seconds | Min clearance (m) | Collisions |
|---|---|---|---|---:|---:|---:|
| smoke / bottleneck-000-p00 | D | info 31, iteration limit | solver_failed | 5.2043 | -0.09310323 | 1840 / 1840 |
| smoke / bottleneck-000-p00 | T | status 0, success | collision_failed | 11.7202 | -0.0000546071 | 12 / 12 |
| smoke / route_choice-000-p01 | D | info 13, infeasible constraints | solver_failed | 2.4298 | -0.08684487 | 1019 / 1019 |
| dev / sparse_clutter-002-p01 | D | info 1, solution found | collision_failed | 0.3285 | -0.000304086 | 20 / 20 |
| dev / sparse_clutter-002-p02 | D | info 31, iteration limit | solver_failed | 8.5449 | -0.08328280 | 899 / 899 |
| dev / sparse_clutter-003-p00 | D | info 1, solution found | collision_failed | 5.7790 | -0.000612569 | 33 / 33 |
| dev / sparse_clutter-003-p00 | T | status 0, success | collision_failed | 2.0682 | -0.000256373 | 8 / 8 |
| dev / sparse_clutter-003-p01 | D | info 1, solution found | collision_failed | 0.1462 | -0.000350882 | 25 / 25 |
| dev / sparse_clutter-003-p02 | D | info 1, solution found | collision_failed | 0.2385 | -0.0000101393 | 6 / 5 |
| dev / sparse_clutter-004-p00 | D | info 1, solution found | collision_failed | 0.1586 | -0.0000249807 | 8 / 8 |
| dev / sparse_clutter-004-p02 | D | info 31, iteration limit | solver_failed | 3.1987 | -0.05075956 | 455 / 455 |
| dev / dense_clutter-002-p00 | D | info 13, infeasible constraints | solver_failed | 1.4287 | -0.05331733 | 675 / 676 |
| dev / dense_clutter-002-p01 | D | info 13, infeasible constraints | solver_failed | 5.1372 | -0.08620382 | 1098 / 1098 |

Each row is under `results/rss2025_full_{smoke,dev}_v1/jobs/<job_id>/attempts/0001/`. Exact job IDs, in table order:

```text
smoke-bottleneck-000-p00-NTO_PAPER-minimum_distance-670914e4
smoke-bottleneck-000-p00-NTO_PAPER-minimum_time-8b155276
smoke-route_choice-000-p01-NTO_PAPER-minimum_distance-c52cdd71
dev-sparse_clutter-002-p01-NTO_PAPER-minimum_distance-87667801
dev-sparse_clutter-002-p02-NTO_PAPER-minimum_distance-462e42b6
dev-sparse_clutter-003-p00-NTO_PAPER-minimum_distance-9c82d729
dev-sparse_clutter-003-p00-NTO_PAPER-minimum_time-cd9f390e
dev-sparse_clutter-003-p01-NTO_PAPER-minimum_distance-9d154440
dev-sparse_clutter-003-p02-NTO_PAPER-minimum_distance-dfda4491
dev-sparse_clutter-004-p00-NTO_PAPER-minimum_distance-68bd625e
dev-sparse_clutter-004-p02-NTO_PAPER-minimum_distance-d9eb82ec
dev-dense_clutter-002-p00-NTO_PAPER-minimum_distance-a5581945
dev-dense_clutter-002-p01-NTO_PAPER-minimum_distance-5a43f054
```

## Evidence of missed collisions between grid points

For smoke `bottleneck-000-p00`, minimum time, IPOPT records 414 iterations, `EXIT: Optimal Solution Found`, constraint violation `0.0`, overall NLP error `1.3987438899602029e-09`, and duration `5.142784740329223 s`.

All twelve stored negative-clearance samples lie at normalized times `0.70709258` through `0.71304819`, strictly inside the single interval `[106/150,107/150] = [0.70666667,0.71333333]`. That interval contains no enforced internal collision location. Minimum clearance is `-5.460709873688491e-05 m`, outside the 2-micrometre contact ambiguity band, and both checkers agree on the twelve colliding samples. This is direct evidence of between-grid collision failure.

Dev `sparse_clutter-003-p00`, minimum time, records 100 IPOPT iterations, `Optimal Solution Found`, constraint violation `0.0`, and overall NLP error `4.9092804018508019e-07`. Its eight negative-clearance samples fall inside `[54/150,55/150]`; minimum clearance is `-0.0002563732296597884 m`.

For the five inspected SNOPT solution-found candidates rejected by final geometry validation, maximum collision-constraint excess above the unit upper bound at the 149 enforced locations is:

| Dev case | Max enforced constraint excess |
|---|---:|
| sparse_clutter-002-p01 | `1.8389734179891093e-12` |
| sparse_clutter-003-p00 | `5.161204796877428e-12` |
| sparse_clutter-003-p01 | `5.455902396533929e-11` |
| sparse_clutter-003-p02 | `1.9416290797380498e-08` |
| sparse_clutter-004-p00 | `1.7431611709639583e-10` |

These values are below the configured `1e-6` feasibility tolerance. Stored negative-clearance samples occur between grid points. In this distance transcription, `solver_F` contains 164 entries: objective, 149 collision values, then 14 fixed endpoint coordinates. The final 14 values independently match the requested endpoints, supporting this interpretation of the raw vector.

By contrast, unsuccessful SNOPT smoke bottleneck and route-choice results have maximum enforced collision-constraint excesses `16.5533` and `17.1045`. These are not successful candidates rejected solely for borderline numerical contact.

## Checker disagreements and logging limitations

Two samples disagree between the recorded collision implementations in this selected subset:

- Dev `sparse_clutter-003-p02`, distance, sample 1186: independent clearance `+4.241579504293824e-08 m`.
- Dev `dense_clutter-002-p00`, distance, sample 1052: independent clearance `-2.6840196783489384e-12 m`.

Both fall within the frozen `±2e-6 m` contact ambiguity band. Each trajectory also has collisions outside that band; the disagreement does not determine its failed classification. These partial findings do not imply zero dev-wide checker disagreements. The sealed final audit must report the complete total.

The selected `snopt.log` files contain only the SNOPT version header despite requested major-print settings. Termination codes, elapsed solver time, full constraint values and multiplier arrays are preserved separately. Exact SNOPT major/minor iteration counts cannot be recovered. In particular, `info=31` / `kIterationLimit` does **not** establish that the configured 2000 major-iteration limit itself was reached; another internal iteration limit could be responsible. This is a diagnostic logging limitation.

## Interpretation boundaries

Retain this frozen paper-profile reconstruction. Increasing collocation density or clearance, inserting controls, substituting a failed candidate with the initial path, or changing solvers would define a separately declared strengthened baseline. Do not alter this run retrospectively. These results alone cannot establish that NTO generally is weak: solver version, smooth distance aggregation, initial control count, discretization, the Kinova collision geometry and unreleased author-transcription details affect the outcome.

The independent sealed smoke audit has actually run successfully:

```bash
bash local/python.sh benchmarks/generic_motion_v1/reports/full_baseline/independent_nto_audit.py --run benchmarks/generic_motion_v1/results/rss2025_full_smoke_v1 --output benchmarks/generic_motion_v1/reports/full_baseline/smoke_nto_audit.json
```

Run it against dev only after all jobs are sealed. See [smoke_nto_audit.json](smoke_nto_audit.json) and [independent_nto_audit.py](independent_nto_audit.py).
