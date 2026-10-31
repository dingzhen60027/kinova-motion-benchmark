# Analysis, denominator and input-provenance audit

This review covers `reproduction/paper_completion/analyze.py`, `reproduction/audit_paper_completion.py`, and `reproduction/report_completion.py`. It changes analysis/reporting only. The frozen numerical package implementation hash remains `40c9f3cdf57783cd7ae5d856d4b4e10332dd37091aa45a7c38718cb860d01611`. No GPU kernels, optimizers, new planning problems or benchmark reruns were launched for this audit.

## Corrected definitions and checks

- **CUDA CFR is independent of solver success.** The archived author TeX `source_evidence/arxiv_text/figures/benchmarks/benchmark_results.tex:35` comments that NTO may fail while still ending on a collision-free trajectory. The main text `sections/experiments.tex:65` describes collision-free trajectories given DRM success. Consequently `native_cuda_cfr_given_drm` counts all saved final trajectories passing CUDA collision checking, divided by native DRM successes, even if the solver reports failure. `solver_success_and_native_cuda_free_rate_given_drm` retains the conjunction separately; `validated_rate_given_native_drm` additionally requires independent geometry and the other feasibility checks. Missing candidates/checks are not counted free and remain in the DRM denominator. These CUDA results use the installed binary's predicate, not a recovered original author binary.
- **Interrupted attempts remain attempts.** A timeout/process error whose saved failure stage identifies an active method contributes to the attempted count even when no complete method time was written. Such timings remain absent rather than being invented or averaged as zero. `attempts_with_completed_timing` and `interrupted_attempts_without_timing` make the distinction explicit.
- **Recovery incidence has explicit denominators.** The analyzer reads saved per-iteration validation and pre-solve records, including an attempted correction followed by an exception. It records triggering cases, completed edit iterations, and rates given attempted methods, native DRM success, independently valid seeds and solver success. Paper VII-A5 reports Forest distance/time rates 5.6%/0.46% and Franka 40.22%/21.36%, but does not establish exact counts or LSCS/LGCS pooling; no unsubstantiated equivalence of denominators is asserted.
- **Original endpoint means the actual raw endpoint.** The previous audit called `input.goal` the original goal even though it had already been converted to float32. The corrected audit reads the accepted IK attempt's raw double goal and `scene.original_start`, separately retaining planning inputs and native path endpoints. It checks the frozen target pose, exact float32 round trip, selected roadmap candidate, case input, and actual goal pose residual. The actual FR3 pilot raw-goal representation error is `4.623960956854552e-08` rad, which the previous labeling would have hidden as zero.
- **Source identity covers the manifest itself.** Source-file hashes must recompute to `implementation_hash`; configuration values must recompute to `config_hash`; frozen source bytes must match the file map. Current computational sources and dependency versions are checked as appropriate. Archival summaries may explicitly skip current-source equality, but collision-depth recomputation is refused under source drift because it evaluates the current model.
- **Saved outcomes are reconciled.** A missing/mismatched method checkpoint is an artifact issue. Solver-success results require validation and replay artifacts; the saved CUDA flags must agree with replay arrays and the result record. A completed manifest cannot hide a nonterminal/missing case.
- **No blanket probability certificate.** The report now states the paper IV-C caveat that a maximum EI-ZO iteration count can invalidate the probabilistic guarantee. The 30-iteration cap does not establish δ/ε certificates for every returned region; numerical trajectory validation remains the observed check.

The report generator preserves full requested denominators, separates unconditional CUDA-free counts from the solver conjunction and strict success, emits `recovery_rates.csv`, and only calls a reconstructed run complete when every requested case is terminal. Its independent-audit fields match the audit output schema, including `issues` and `independently_rechecked_fixed_points`.

## Actual executed verification

The following lightweight checks ran while the separate main experiment continued. The driver reads saved records and tests metadata logic; it does not construct a robot, call CUDA, or solve optimization problems.

```bash
bash local/python.sh -m py_compile reproduction/paper_completion/analyze.py reproduction/audit_paper_completion.py reproduction/report_completion.py
bash local/python.sh -m reproduction.check_completion_analysis > reproduction/reports/paper_completion/diagnostics/analysis_checks/verification.log 2>&1
bash local/python.sh -m py_compile reproduction/report_completion.py
```

All commands returned exit code 0. The analysis driver reported **8 checks passed**:

1. The actual completed FR3 pilot's source snapshot, manifest source digest and configuration digest agree.
2. Changing a configuration value in memory is detected by the configuration hash check.
3. Changing a source hash in memory is detected by the implementation-hash check.
4. The actual pilot retains the fixed target pose and preserves the nonzero raw-double-to-canonical-goal difference.
5. Changing the IK target pose in memory is detected.
6. The actual completed 400-case Forest summary reconciles source and artifact records with all 400 cases terminal.
7. An explicitly labeled synthetic bookkeeping fixture verifies that a failed solver's CUDA-free trajectory contributes to CFR but not the solver-success conjunction.
8. An explicitly labeled synthetic bookkeeping fixture verifies that an interrupted method contributes an attempt without a fabricated elapsed time.

The two synthetic bookkeeping checks are **analysis unit fixtures, not experimental results**, and are not included in any benchmark count. Their definition is preserved with that label. The real Forest data were only summarized into a diagnostic output directory; these were existing main-run results, not additional cases.

Evidence:

- `diagnostics/analysis_checks/verification.log`: actual command output.
- `diagnostics/analysis_checks/verification.json`: eight successful checks, actual raw-goal representation error and real Forest counts.
- `diagnostics/analysis_checks/actual_forest_summary/`: actual completed Forest records summarized during verification.
- `diagnostics/analysis_checks/synthetic_bookkeeping_fixture/` and `synthetic_fixture_definition.json`: explicitly labeled metadata fixtures.
- `reproduction/check_completion_analysis.py`: rerunnable driver containing all assertions.

These **8 analysis checks are separate from the 18 pytest tests** recorded elsewhere. They are not described as 26 pytest tests. No complete independent geometric audit was run in this review; the root task performs that after the main runs complete.
