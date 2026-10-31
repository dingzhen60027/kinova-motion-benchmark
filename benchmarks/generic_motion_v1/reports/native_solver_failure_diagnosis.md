# Native baseline failure diagnosis

Two sealed failures were checked using the saved paths, regions and manifest endpoints: `smoke-open_control-000-p01` and `dev-bottleneck-000-p01`. The diagnosis used CPU solves only; it neither changed the original results nor produced replacement benchmark scores.

Both failures came from the adapter's float32/float64 endpoint handoff. The native DRM returns float32 configurations, while the fixed manifest and optimizer use their original float64 endpoints. Every saved seed edge satisfied its assigned region, and each neighboring pair of regions contained their shared native path vertex. There was no corridor order or connectivity mismatch. However, the original start configurations exceeded a region inequality by `3.8019699721836275e-8` and `3.4001621851409425e-8`, respectively.

Actual calls to the installed author's `scsplanning.polygonal.get_points` gave:

| Case | Original regions + original endpoints | Original regions + native float32 endpoints | Tiny b correction + unchanged original endpoints |
|---|---|---|---|
| smoke open-control p01 | Solver failure | Solve succeeded | Solve succeeded |
| dev bottleneck 000 p01 | Solver failure | Solve succeeded | Solve succeeded |

The corrected regions also allowed the installed author's `biconvex` SCSTO solver to return trajectories in both cases. Their durations were 4.9344262 s and 6.2301606 s. These are isolated CPU diagnostic outputs; they have not undergone a new full GPU and independent collision validation and must not replace the failed benchmark records.

The repair must keep manifest endpoints, collision geometry and plane normals unchanged. It should apply a strictly bounded numerical correction to the region offsets, recording the original and corrected halfspaces. The same operation is needed for newly inflated, reused and recovered regions before optimization. A correction beyond the fixed normalized threshold must fail explicitly.

Run from the repository root:

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.reports.hardening_proposal.diagnose_native_solver_failure
```

Numerical evidence and before/after hashes of the original sealed artifacts are in `native_solver_failure_diagnosis.json`. The first direct-file invocation failed because the repository package was not on `sys.path`; its failure log is retained beside the successful module-invocation log in `hardening_proposal/`.
