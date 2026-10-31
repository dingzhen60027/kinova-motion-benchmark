# Baseline admission audit

This audit concerns the native baseline wrapper, offline DRM builder, and common numerical validator. It does not change the canonical robot, generator distribution, frozen configuration, test manifest, CUDA implementation, or historical results. No GPU planning was run by this audit. The parent qualification run separately performs the current-source GPU smoke checks.

## Reproduced defects and repair

Three pre-repair CPU exploits are retained in [baseline_pre_repair.json](baseline_pre_repair.json): a NaN fixed goal, an unparameterized polygon claiming `minimum_time`, and an infinite endpoint tolerance all incorrectly returned `success_validated_numerical`. These were public-validator contract defects; the production runner already rejected nonfinite generated endpoints and emitted the correct representation. Their existence alone does not establish that the historical successful trajectories were invalid.

The fixes reject malformed inputs before interpreting them as evidence:

| Admission risk | Required behavior now | Evidence |
|---|---|---|
| NaN/Inf, string or boolean endpoints; ragged arrays | Require finite numeric vectors of exactly seven joints; no implicit boolean/string angle conversion | Adversarial endpoint tests |
| PWL cost mislabeled as seconds | Require declared objective and matching representation; minimum-distance B-splines must have order two; minimum-time needs a timed C1 representation | Objective and spline tests |
| NaN/infinite tolerances, invalid budgets and sample caps, noninteger seeds | Validate finite configuration types/ranges and unsigned 32-bit method/DRM seeds before native calls | Config and seed tests; NaN-budget saved-failure test |
| Missing/mismatched joint velocity, acceleration or endpoint-rest contract | Check explicit manifest vectors against frozen evaluation limits in both runner and standalone validator | Motion-contract tests |
| Scalar, broadcastable or nonboolean collision output; infinite clearance | Require one boolean classification and finite independent clearance per sample | Broken-checker tests |
| Discontinuous spline presented as smooth | Require clamped finite knots and interior multiplicity at most order minus two for time; verify Bezier C0/C1 junction residuals and endpoint velocity | Spline multiplicity tests; independent SciPy oracle |
| Malformed/nonfinite region constraints; coverage arithmetic overflow | Require finite A[m,7]/b[m], finite residual and bounded correction arithmetic | Region shape, NaN and overflow tests |
| DRM success flag with malformed or disconnected path | Require actual finite N×7 native output and unchanged fixed endpoints; never reshape arbitrary output into a path | Native-path tests |
| Offline build error or partial directory overwritten | Save exception/stage/failure metadata; preserve partial sampling arrays; refuse any pre-existing nonempty destination | Injected offline-failure and preservation tests |
| Missing solver treated as ordinary algorithm failure | Probe the method's actual solver; save availability evidence and classify unavailable dependency separately | Real Ipopt CPU solve and simulated unavailable-solver test |

`validate.py` checks endpoint/objective/motion constraints at its entry, samples each supported curve, checks derivative control-hull bounds, and rejects malformed collision or halfspace outputs. Configuration validation is in `roadmap.load_config`. `run_case` still executes the same native DRM/EI-ZO and author LSCS/SCSTO algorithms. NTO now probes its actual Ipopt solver without requiring unrelated SCSPlanning dependencies. The EI-ZO probe continues to execute Clarabel and inspect the installed author modules.

The standalone validator remains a numerical feasibility check. A solver failure cannot become a successful method result: the NTO runner retains `solver_failed` even if its returned candidate happens to pass numerical checks. Control-hull derivative and position bounds are sufficient, potentially conservative tests; failure is not a proof that no feasible curve exists.

## Actual verification

The targeted suite completed **110 tests, all passed**, using the existing `local/python.sh` environment. It includes an independent analytic cubic `q(t)=0.1(3(t/3)^2−2(t/3)^3)` and SciPy `BSpline` comparison to Drake samples, plus real author LSCS/SCSTO and Ipopt CPU solves. The tests deliberately inject malformed external evidence rather than only mirroring implementation formulas. See [baseline_pytest.log](baseline_pytest.log).

All successful candidates from the three relevant historical runs were then read from sealed files and rechecked:

| Run | Historical successes | New contract passes | New contract failures |
|---|---:|---:|---:|
| smoke_v3_fixed | 8 | 8 | 0 |
| dev_eizo_v2 | 198 | 198 | 0 |
| dev_paired_v2 | 24 | 24 | 0 |
| Total | 230 | 230 | 0 |

This covers 114 distance and 116 time candidates. Required candidate, sample, validation, input and result hashes were checked against each attempt's receipt. Current reconstructed q and time arrays are exactly equal to the saved arrays. Only under this equality does the audit reuse the corresponding historical native and independent collision classifications. It does **not** represent a fresh collision experiment.

SciPy BPoly/BSpline independently reevaluated positions and derivative samples. Across all 230 candidates, maximum endpoint error was 2.15694e−8 rad and maximum SciPy-versus-validator position disagreement was 3.10862e−15 rad. Across time candidates, maximum independent endpoint speed was 2.43720e−9 rad/s, and maximum combined position/velocity junction residual was 7.92193e−9. All are below the frozen 1e−5 numerical tolerance. Independent dense derivative values never exceeded the claimed conservative derivative bounds. The original success counts and objective values did not change.

Per-attempt hashes, bounds and oracle values are in [baseline_saved_candidate_reaudit.json](baseline_saved_candidate_reaudit.json), summarized in [baseline_saved_candidate_reaudit.md](baseline_saved_candidate_reaudit.md). [baseline_evidence.json](baseline_evidence.json) records current source/config hashes and maxima. Historical result files were not rewritten.

## Budget, randomness and comparison boundaries

- The outer worker watchdog is necessary for the 60 s total wall-time limit: Python cannot safely preempt a running native CUDA kernel or solver. `run_case` checks between stages and also marks a returned late call as timeout in its finalizer. The parent audit tests the subprocess watchdog and interruption handling separately.
- Cold function time includes model, dependency probe, GPU initialization, roadmap loading and online work. Python process startup and teardown belong to the separately saved outer process measurement. The reported warm time is the online portion of that same cold execution after setup, **not a second hot-repeat experiment**. Offline map construction remains separately reported.
- Configured optimizer budget cannot exceed total budget. NTO passes actual remaining time to Ipopt; native author SCSTO lacks a safe Python interrupt and is bounded by the outer watchdog. A time-limited result is not converted to success based on a completed candidate alone.
- Offline samples use saved PCG64 draws with a separate DRM seed; EI-ZO receives the saved unsigned 32-bit method seed. Optimizer initialization is deterministic from the common DRM path; unused solver seeds remain labeled unused. Native map serialization may vary due to unordered-map order; actual bytes are hashed.
- Both methods still use the same canonical geometry, fixed endpoints, native roadmap, objective and frozen limits. The wrapper verifies roadmap/model hashes. Current manifest voxel spheres fit inside the configured cache workspace, with minimum full-sphere clearance to its boundary of 0.595685 m for smoke, 0.487773 m for dev, and 0.466726 m for frozen test inputs. The test check only reads geometry; no test planning or scores were run.
- Native pre-shortcut paths and collision-query counts are not exposed by the upstream API. No equal-collision-query-budget comparison can be claimed. The existing NTO comparison is the declared Drake KTO/Ipopt adaptation, not a recovered exact paper implementation.
- Numerical collision samples remain finite-resolution evidence, not a mathematical continuous collision certificate. C1 continuity and full-segment conservative derivative bounds do not change that limitation.

## Reproducible commands

Run from the repository root:

```bash
bash local/python.sh -m pytest benchmarks/generic_motion_v1/tests/test_baseline*.py -q
bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.baseline_reaudit
```

No environment rebuild, dependency installation, formal test planning, commit, or push was performed by this sub-audit.
