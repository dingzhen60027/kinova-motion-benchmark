# Formal baseline execution and limitations

The frozen Kinova v1 study has completed all 24 planned blocks. Its 9,900 unique
registered slots contain 9,300 legal planning calls and 600 invalid-input exits.
Every terminal receipt passed integrity verification, and all 72 independent
trajectory/provenance audit invocations passed. These checks certify the saved
evidence and numerical checks, not universal planner success or continuous
collision freedom.

[Full results and six figures](benchmark_v1_formal_baselines.md) ·
[Numerical metrics contract](../BASELINE_PROTOCOL.md) ·
[Input readiness](baseline_study/test_input_readiness.md) ·
[Saved replay gallery](baseline_study/formal_replays_v1/index.html) ·
[Failure review](baseline_study/formal_failure_review.md) ·
[Independent statistics review](baseline_study/formal_report_independent_review.md)

## Fixed evaluation universe

The user's approved choice preserves all 420 v1 registrations: 395 valid input
problems and 25 endpoint-generation failures. No test scene or endpoint was
replaced, and no failed planning call was selectively retried. The 25 invalid
inputs are not planner failures or proofs of global infeasibility.

The main experiment uses three 6,000-node roadmap seeds and five method/objective
combinations: 6,300 registered slots, of which 5,925 are legal. The obstacle main
suite contains 375 problems from 75 valid physical scenes, hence 1,125 legal
queries per method. The 20 control problems share one physical base world and
produce 60 queries per method; they have separate tables and no scene-generalization
confidence interval.

The size study uses the same input-only selection of 80 IDs at 3,000, 6,000,
9,000 and 12,000 requested nodes, with three roadmap seeds. It retains 75 legal
and five invalid IDs. Its 4,800 observations include 1,200 reused main-study
observations; only 3,600 are additional registered slots. The second 3,000-node
roadmap retained 2,999 nodes after native pruning; it was neither replaced nor
relabelled as another budget.

## Main observed outcomes

These are the obstacle-suite results at 6,000 nodes. All 1,125 legal queries per
method remain in the denominator, including dependency and infrastructure errors.

| Objective | Method | Numerically validated success | Rate |
| --- | --- | ---: | ---: |
| Minimum distance | DRM → EI-ZO → LSCS | 776 / 1125 | 68.98% |
| Minimum distance | Two DRM routes → EI-ZO → LGCS | 682 / 1125 | 60.62% |
| Minimum distance | DRM → NTO distance | 255 / 1125 | 22.67% |
| Minimum time | DRM → EI-ZO → SCSTO | 775 / 1125 | 68.89% |
| Minimum time | DRM → NTO time | 637 / 1125 | 56.62% |

The full report contains cold p50/p90/p95, successful costs, common-success paired
cost differences, all-legal latency differences, stage completion counts, memory,
clearance, strata and fixed-size comparisons. Costs from different successful
subsets are descriptive and must not be used for an unpaired quality ranking.
Offline roadmap construction is reported separately from online timing.

An independent row-level review recomputed 255 groups and 204 paired point
estimates, with 213,502 arithmetic/eligibility checks passing. All six figures
were visually inspected. Eight saved replays cover the five method/objective
combinations, two EI-ZO obstacle-detour examples and a clearly labelled failed
case's verified seed. Actual browser tests exercised playback, pause and frame
navigation on success and seed-only pages without JavaScript errors; nine
screenshots and the temporary server's shutdown record are preserved. Display
replays are illustrations, not an additional success-rate sample.

## Environment and provenance

All planning used source commit
`c8adbd8cef44db25b41559e8fbf2b38e120b624a`; later evidence commits do not change that
runtime identity. The observed upstream `origin/main` was
`7661a97bb7c4931be73d2bc4f8c45c3094ff476e`. Native CSDecomp CUDA and the canonical
robot/obstacle model remained unchanged.
The raw outcomes, roadmaps and logging-check artifacts were committed as
`4fa061e31df705093c40b3beb1536601682b29ac`.

The existing Conda environment was retained: Ubuntu 22.04.5, Python 3.10.22,
CSDecomp 0.1.2, Drake 1.51.1, SCSPlanning 0.1.0, pybezier 0.1.4, NumPy 2.2.6,
SciPy 1.15.3, Clarabel 0.11.1 and gurobipy 13.0.3. Hardware was Quadro RTX 8000
(49,152 MiB), driver 570.211.01. No environment replacement, package installation
or uv migration occurred. [Post-run environment checks](baseline_study/environment_postformal.json)
confirmed the frozen runtime and all 183 installed dependency file hashes.

The formal protocol SHA-256 is
`8d152ac237cf39fe9602bcf54d47fbe8e30471dd5e3096ff05375ff4b3fcd7d7`.
The unchanged test manifest SHA-256 is
`9fcbe5ccae28d8681586b1ac6c9986f51f7cc7b5cdc19352c101910e3b6ed03b`.
The protocol binds model geometry, filters, data files, roadmap files, seeds,
budgets, exact method pairs and the full passing input audit.

The 24 recorded resume subprocesses all exited 0, from 2026-10-28 04:57:14 UTC
to 13:58:59 UTC. Their recorded process durations sum to 32,460.33 seconds; this
batch wall time is not an individual planning latency. The original launcher
session returned exit code 143 when collected afterward. Its cause is unresolved;
the complete 24-block ledger, individual subprocess exits and independent audits
are preserved separately in the [launcher observation](baseline_study/formal_launch_session_observation.json).
No job was rerun to hide that discrepancy.

## Limits that matter for subsequent research

- This is a Kinova adaptation. It does not recover the paper's private FR3
  collision-sphere model, original query list or hardware experiments. NTO and
  LGCS include declared local mathematical reconstructions. The official
  SCSPlanning Clarabel backend differs from the paper's LSCS/MOSEK setup.
- LGCS encountered 94 obstacle-suite and three control license-size rejections
  in the main study. These remain `unsupported`; the constrained deployment is
  not evidence that an unrestricted LGCS solver is inferior.
- NTO distance had zero validated successes on 843 straight-line-blocked
  obstacle queries. Its solver/parameter and 149-versus-150 constraint ambiguity,
  iteration limits and saved backend failures remain relevant. Any further
  implementation improvement must be developed on dev under a new declared
  protocol, not tuned to this frozen test set.
- The main study's 115 infrastructure errors correspond to 23 common
  case/roadmap pairs across all five combinations, each with the native DRM
  `closest_node >= 0` assertion. The SCSTO solver failure is a saved Clarabel
  numerical failure. Neither is a proof that the input problem is infeasible.
- The 25 main `validation_undetermined` outcomes lie within the fixed ±2 µm
  contact band. They are excluded from valid successes. The shared finite
  collision sampler and conservative voxel cover have documented limitations;
  no continuous collision-free certificate is claimed.
- Bootstrap resamples physical scene clusters with all endpoint/roadmap repeats
  together, conditional on these three fixed roadmaps. Independent auditors
  check raw evidence and trajectory algebra; they do not independently recompute
  the bootstrap intervals. Warm timing is the recorded online portion of the same
  cold run, not a separately repeated persistent-process experiment.

## Actual commands and artifacts

Run commands from the repository root with `bash local/python.sh`.

```bash
# The actually executed coordinator; completed runs are preserved on resume.
bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_study.run_formal_study

# Read the exact successful audit/export/seal/render sequence and prerequisites.
cat benchmarks/generic_motion_v1/reports/baseline_study/formal_postprocess_execution.md
```

The [formal command ledger](baseline_study/formal_commands.jsonl) records all
prepare/resume argv, timestamps and return codes. The
[postprocessing command ledger](baseline_study/formal_postprocess_commands.jsonl)
records metrics export, bundle sealing and rendering. The
[independent audit](baseline_study/formal_audit/integrity_audit.json) binds its
72 exact auditor commands, source hashes and reports. The
[replay execution report](baseline_study/formal_replay_execution.md) records
deterministic selection and checks without replanning.

All original successes, failures, partial timeout artifacts, arrays, regions,
solver logs and receipts remain in `results/rss2025_formal_v1_*`. Evidence is
saved incrementally on `benchmark/generic-motion-v1`; no push or main-branch
update is part of this evaluation. A new experiment must use a new run/protocol
instead of rewriting these frozen records.
