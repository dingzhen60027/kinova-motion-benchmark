# Complete dev NTO solver failures and backend evidence

Generated 2026-10-28T04:00:11.803014+00:00. Source commit: 72984027030175b7bca78e2dcf592b38f0c1864b. All 900 dev jobs are sealed; this report examines all 31 NTO records whose final status is solver_failed (30 distance, 1 time), out of 360 NTO jobs. Original outcomes are unchanged.

## Raw status distribution

| Solver | Native field/code | Drake result | Count |
|---|---|---|---:|
| IPOPT | status=1 | SolutionResult.kIterationLimit | 1 |
| SNOPT | info=13 | SolutionResult.kInfeasibleConstraints | 7 |
| SNOPT | info=31 | SolutionResult.kIterationLimit | 10 |
| SNOPT | info=32 | SolutionResult.kIterationLimit | 6 |
| SNOPT | info=41 | SolutionResult.kSolverSpecificError | 5 |
| SNOPT | info=43 | SolutionResult.kSolverSpecificError | 1 |
| SNOPT | exception/no native return code | exception | 1 |

The saved IPOPT exception-free failure reports Max iteration exceeded; its native log ends at iteration 2000 and states Maximum Number of Iterations Exceeded. SNOPT codes 31 and 32 are preserved separately; its logs contain only version headers, so exact observed iteration counts are unavailable. Codes 41 and 43 are not assigned an unverified causal interpretation.

## Drake geometry-derivative exception

Case: dev-route_choice-002-p00. Job: dev-route_choice-002-p00-NTO_PAPER-minimum_distance-48b608ce.

Attempt: benchmarks/generic_motion_v1/results/rss2025_full_dev_v1/jobs/dev-route_choice-002-p00-NTO_PAPER-minimum_distance-48b608ce/attempts/0001.

Solver: SNOPT distance; native call elapsed 6.398992366157472 s. Final sealed status remains solver_failed. The shared DRM seed passed its numerical validation. No optimizer candidate or raw final solution was returned.

Saved exception:

```text
RuntimeError("Exception while evaluating SNOPT costs and constraints: 'Failure at external/drake+/geometry/proximity/distance_to_shape_touching.cc:122 in BoxBoxGradient(): condition '(X_WA * p_ACa - X_WB * p_BCb).norm() < kEps' failed.'")
```

Saved traceback:

```text
Traceback (most recent call last):
  File "/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/runners/paper_nto.py", line 222, in paper_nto
    result = solver.Solve(prog)
RuntimeError: Exception while evaluating SNOPT costs and constraints: 'Failure at external/drake+/geometry/proximity/distance_to_shape_touching.cc:122 in BoxBoxGradient(): condition '(X_WA * p_ACa - X_WB * p_BCb).norm() < kEps' failed.'
```

This is a backend geometry-derivative evaluation failure, not a solver certificate that the task is infeasible. The internal condition failed in Drake distance_to_shape_touching.cc:122, inside BoxBoxGradient. The canonical Kinova box collision model is unchanged; its behavior cannot be equated with the paper original FR3 sphere model. No original record is relabeled.

## All affected attempts

Each link points to the saved native result; full paths, source evidence SHA256 values and raw results are in [nto_backend_failures.json](nto_backend_failures.json).

| Case / roadmap | Objective | Raw code | Saved validation | Native result |
|---|---|---|---|---|
| dev-sparse_clutter-002-p02 / kinova7-full-n6000-s0-e258c5f9914c | minimum_distance | 31 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-sparse_clutter-002-p02-NTO_PAPER-minimum_distance-462e42b6/attempts/0001/nto_solver_result.json) |
| dev-sparse_clutter-004-p02 / kinova7-full-n6000-s0-e258c5f9914c | minimum_distance | 31 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-sparse_clutter-004-p02-NTO_PAPER-minimum_distance-d9eb82ec/attempts/0001/nto_solver_result.json) |
| dev-dense_clutter-002-p00 / kinova7-full-n6000-s0-e258c5f9914c | minimum_distance | 13 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-dense_clutter-002-p00-NTO_PAPER-minimum_distance-a5581945/attempts/0001/nto_solver_result.json) |
| dev-dense_clutter-002-p01 / kinova7-full-n6000-s0-e258c5f9914c | minimum_distance | 13 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-dense_clutter-002-p01-NTO_PAPER-minimum_distance-5a43f054/attempts/0001/nto_solver_result.json) |
| dev-bottleneck-000-p01 / kinova7-full-n6000-s0-e258c5f9914c | minimum_distance | 31 | success_validated_numerical | [record](../../results/rss2025_full_dev_v1/jobs/dev-bottleneck-000-p01-NTO_PAPER-minimum_distance-013e5065/attempts/0001/nto_solver_result.json) |
| dev-bottleneck-000-p02 / kinova7-full-n6000-s0-e258c5f9914c | minimum_distance | 31 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-bottleneck-000-p02-NTO_PAPER-minimum_distance-383b7f04/attempts/0001/nto_solver_result.json) |
| dev-bottleneck-001-p02 / kinova7-full-n6000-s0-e258c5f9914c | minimum_distance | 13 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-bottleneck-001-p02-NTO_PAPER-minimum_distance-48855244/attempts/0001/nto_solver_result.json) |
| dev-bottleneck-004-p01 / kinova7-full-n6000-s0-e258c5f9914c | minimum_distance | 41 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-bottleneck-004-p01-NTO_PAPER-minimum_distance-71d3bbd5/attempts/0001/nto_solver_result.json) |
| dev-route_choice-002-p00 / kinova7-full-n6000-s0-e258c5f9914c | minimum_distance | exception | None | [record](../../results/rss2025_full_dev_v1/jobs/dev-route_choice-002-p00-NTO_PAPER-minimum_distance-48b608ce/attempts/0001/nto_solver_result.json) |
| dev-route_choice-002-p01 / kinova7-full-n6000-s0-e258c5f9914c | minimum_distance | 13 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-route_choice-002-p01-NTO_PAPER-minimum_distance-c53f6104/attempts/0001/nto_solver_result.json) |
| dev-route_choice-003-p01 / kinova7-full-n6000-s0-e258c5f9914c | minimum_distance | 41 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-route_choice-003-p01-NTO_PAPER-minimum_distance-8ef49866/attempts/0001/nto_solver_result.json) |
| dev-route_choice-004-p00 / kinova7-full-n6000-s0-e258c5f9914c | minimum_distance | 41 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-route_choice-004-p00-NTO_PAPER-minimum_distance-43d238ca/attempts/0001/nto_solver_result.json) |
| dev-sparse_clutter-003-p02 / kinova7-full-n6000-s1-e258c5f9914c | minimum_distance | 13 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-sparse_clutter-003-p02-NTO_PAPER-minimum_distance-43e16ea0/attempts/0001/nto_solver_result.json) |
| dev-dense_clutter-002-p01 / kinova7-full-n6000-s1-e258c5f9914c | minimum_distance | 31 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-dense_clutter-002-p01-NTO_PAPER-minimum_distance-420853ea/attempts/0001/nto_solver_result.json) |
| dev-bottleneck-001-p02 / kinova7-full-n6000-s1-e258c5f9914c | minimum_distance | 32 | success_validated_numerical | [record](../../results/rss2025_full_dev_v1/jobs/dev-bottleneck-001-p02-NTO_PAPER-minimum_distance-1c0dc07c/attempts/0001/nto_solver_result.json) |
| dev-bottleneck-004-p01 / kinova7-full-n6000-s1-e258c5f9914c | minimum_distance | 31 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-bottleneck-004-p01-NTO_PAPER-minimum_distance-a8eb3288/attempts/0001/nto_solver_result.json) |
| dev-bottleneck-004-p02 / kinova7-full-n6000-s1-e258c5f9914c | minimum_distance | 31 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-bottleneck-004-p02-NTO_PAPER-minimum_distance-ef5f9153/attempts/0001/nto_solver_result.json) |
| dev-route_choice-003-p01 / kinova7-full-n6000-s1-e258c5f9914c | minimum_distance | 13 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-route_choice-003-p01-NTO_PAPER-minimum_distance-4c4f8816/attempts/0001/nto_solver_result.json) |
| dev-sparse_clutter-003-p00 / kinova7-full-n6000-s2-e258c5f9914c | minimum_distance | 32 | success_validated_numerical | [record](../../results/rss2025_full_dev_v1/jobs/dev-sparse_clutter-003-p00-NTO_PAPER-minimum_distance-32f5f434/attempts/0001/nto_solver_result.json) |
| dev-sparse_clutter-003-p01 / kinova7-full-n6000-s2-e258c5f9914c | minimum_distance | 32 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-sparse_clutter-003-p01-NTO_PAPER-minimum_distance-d286af33/attempts/0001/nto_solver_result.json) |
| dev-sparse_clutter-003-p02 / kinova7-full-n6000-s2-e258c5f9914c | minimum_distance | 32 | success_validated_numerical | [record](../../results/rss2025_full_dev_v1/jobs/dev-sparse_clutter-003-p02-NTO_PAPER-minimum_distance-c79bd7c1/attempts/0001/nto_solver_result.json) |
| dev-sparse_clutter-004-p00 / kinova7-full-n6000-s2-e258c5f9914c | minimum_distance | 43 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-sparse_clutter-004-p00-NTO_PAPER-minimum_distance-4d30a589/attempts/0001/nto_solver_result.json) |
| dev-sparse_clutter-004-p01 / kinova7-full-n6000-s2-e258c5f9914c | minimum_distance | 31 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-sparse_clutter-004-p01-NTO_PAPER-minimum_distance-9030e34e/attempts/0001/nto_solver_result.json) |
| dev-sparse_clutter-004-p01 / kinova7-full-n6000-s2-e258c5f9914c | minimum_time | 1 | invalid_endpoint | [record](../../results/rss2025_full_dev_v1/jobs/dev-sparse_clutter-004-p01-NTO_PAPER-minimum_time-d2e382a8/attempts/0001/nto_solver_result.json) |
| dev-sparse_clutter-004-p02 / kinova7-full-n6000-s2-e258c5f9914c | minimum_distance | 32 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-sparse_clutter-004-p02-NTO_PAPER-minimum_distance-297942c1/attempts/0001/nto_solver_result.json) |
| dev-dense_clutter-002-p01 / kinova7-full-n6000-s2-e258c5f9914c | minimum_distance | 13 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-dense_clutter-002-p01-NTO_PAPER-minimum_distance-16e5781b/attempts/0001/nto_solver_result.json) |
| dev-bottleneck-000-p02 / kinova7-full-n6000-s2-e258c5f9914c | minimum_distance | 41 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-bottleneck-000-p02-NTO_PAPER-minimum_distance-6de702a7/attempts/0001/nto_solver_result.json) |
| dev-bottleneck-004-p01 / kinova7-full-n6000-s2-e258c5f9914c | minimum_distance | 41 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-bottleneck-004-p01-NTO_PAPER-minimum_distance-0f216ec3/attempts/0001/nto_solver_result.json) |
| dev-bottleneck-004-p02 / kinova7-full-n6000-s2-e258c5f9914c | minimum_distance | 31 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-bottleneck-004-p02-NTO_PAPER-minimum_distance-2115708d/attempts/0001/nto_solver_result.json) |
| dev-route_choice-000-p02 / kinova7-full-n6000-s2-e258c5f9914c | minimum_distance | 31 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-route_choice-000-p02-NTO_PAPER-minimum_distance-4ccece59/attempts/0001/nto_solver_result.json) |
| dev-route_choice-004-p00 / kinova7-full-n6000-s2-e258c5f9914c | minimum_distance | 32 | collision_failed | [record](../../results/rss2025_full_dev_v1/jobs/dev-route_choice-004-p00-NTO_PAPER-minimum_distance-c739d06b/attempts/0001/nto_solver_result.json) |

## Interpretation boundaries

- All31 records retain their sealed run status solver_failed. Nothing is reclassified, deleted, retried, or replaced.
- The single BoxBoxGradient assertion is a Drake geometry-derivative evaluation failure while SNOPT evaluates costs/constraints; it is not evidence that the underlying motion-planning problem is infeasible or that NTO intrinsically fails.
- Its canonical Kinova collision geometry contains boxes; original paper FR3 sphere geometry may behave differently. This is an explicit model/backend reproduction limitation.
- SNOPT info13 maps to Drake kInfeasibleConstraints, not a proof of globally infeasible motion planning.
- SNOPT info31/info32 are kept distinct. Exact major/minor iteration counts are absent from the header-only SNOPT logs; do not infer counts solely from the configured2000-major-iteration cap.
- SNOPT info41/info43 are retained as raw codes and Drake kSolverSpecificError; no unverified fine-grained cause is asserted.
- IPOPT details.status=1 here is documented by the saved details description as Max iteration exceeded and the log reports2000 iterations; do not confuse this enum with another IPOPT return-code enumeration.
- Four solver_failed candidates passed finite numerical geometry validation. They remain solver_failed and are not counted as successful baseline solves.
- Failures from dense curve validation after solver success are outside this31-record solver_failed subset and are separately preserved as collision_failed.
- This report is read-only postprocessing of a complete900-job dev run, not a new solver/GPU experiment.

The complete independent reconstruction audit is [dev_nto_audit.json](dev_nto_audit.json). It verifies sealed hashes and numerical spline reconstruction, but does not turn failed native optimizer calls into successful solves.
