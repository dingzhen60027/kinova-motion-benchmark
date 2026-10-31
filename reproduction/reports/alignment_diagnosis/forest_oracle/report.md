# Exact Forest segment/circle oracle diagnostic

All ten predetermined scenes were evaluated using their original e00…e09_r00_n00200 DRM paths. The original node count, endpoints, domain, path-length objective, warm start and SNOPT settings were retained. Pointwise collision constraints were replaced with exact line-segment/circle clearance constraints with a 1 mm margin. This diagnostic is not the paper's NTO method and does not overwrite earlier results.

Oracle valid cases: **10/10**. Original pointwise-NTO valid cases on these same paths: **3/10**. All candidates and solver failures are saved.

| Case | Original NTO status | Oracle status | Original nodes | Oracle length (m) | Minimum clearance (mm) |
|---|---|---|---:|---:|---:|
| e00_r00_n00200 | validation_failed | success_numerical | 3 | 10.724593 | 1.000000 |
| e01_r00_n00200 | success_numerical | success_numerical | 2 | 10.630146 | 141.077896 |
| e02_r00_n00200 | validation_failed | success_numerical | 5 | 10.774128 | 1.000000 |
| e03_r00_n00200 | validation_failed | success_numerical | 3 | 10.630279 | 1.000000 |
| e04_r00_n00200 | success_numerical | success_numerical | 2 | 10.630146 | 2.505770 |
| e05_r00_n00200 | success_numerical | success_numerical | 2 | 10.630146 | 6.601381 |
| e06_r00_n00200 | validation_failed | success_numerical | 3 | 10.630349 | 1.000000 |
| e07_r00_n00200 | validation_failed | success_numerical | 3 | 11.115789 | 1.000000 |
| e08_r00_n00200 | optimization_failed | success_numerical | 4 | 11.245954 | 1.000000 |
| e09_r00_n00200 | validation_failed | success_numerical | 3 | 10.630770 | 1.000000 |

The analytic Jacobian and path-length gradient were checked with central finite differences for a separate geometric control and all ten original paths. AutoDiff identity and nontrivial chain gradients were checked. A known feasible three-node path around one circle was solved and independently validated; a fixed two-node path through that circle was correctly rejected by both the independent geometry and the solver.

Controls: True; ten input gradient checks: True; input hashes unchanged: True; source hashes unchanged: True.

The validation computes line-to-center distance independently through cross products and endpoint tests. This certifies the saved PWL paths against the declared circle model in floating-point arithmetic; it does not extend to robot swept volumes or nonlinear splines. The 1 mm margin is accepted to the unchanged 1e−6 solver feasibility scale, while zero-penetration, endpoint and domain checks are also reported separately.

The ten successful oracle solves establish that these fixed low-node-count paths can support nontrivial collision-free optimization under the exact segment constraints. They do not recover the authors' unpublished NTO choices or reproduce their performance. A failed local solve would not prove global infeasibility. Timings are recorded for diagnosis only and must not be compared with the paper. See manifest.json for the frozen configuration, source/input hashes and exact execution command; cases/* contains raw candidates, constraints, gradients, SNOPT logs and validation.

## Implementation preflight and additional verification

The first implementation preflight stopped before any solver call because Drake InitializeAutoDiff returned an n×1 array and the callback adapter initially expected a flat vector. The derivative control caught the zero-width derivative output. The adapter now explicitly flattens its input. The failed source snapshot, manifest, traceback and logs remain in the sibling `forest_oracle_preflight_01` directory. No failed experimental candidate was removed.

A second independent line-circle calculation from `diagnose_task_distribution.py` verified all ten saved solutions with minimum-clearance agreement within 1e−12 m. Original paths are bytewise-equivalent as arrays, node counts are unchanged, and all final path lengths are no larger than their original DRM lengths. See `postrun_audit.json`. All ten experimental solver calls and the two positive/negative solver controls ran on CPU. The raw original NTO records remain alongside the new candidates; none was replaced.
