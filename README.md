# Kinova Motion Planning Benchmark

A fixed-input benchmark for seven-joint Kinova Gen3 motion planning, built on [CSDecomp](https://github.com/wernerpe/csdecomp). Every method receives the same static obstacle scene, `q_start` and `q_goal`. The project evaluates validated joint-space path length and constrained trajectory duration in separate tracks.

Project repository: [dingzhen60027/kinova-motion-benchmark](https://github.com/dingzhen60027/kinova-motion-benchmark). CSDecomp and the other dependency authors retain their original attribution and licenses.

The upstream CI workflows are preserved as inactive examples in `.github/workflow-examples/`; this repository does not automatically run them.

The upstream C++/CUDA library and the existing Conda environment are preserved. The public entry points are `scripts/benchmark.py` for data and `scripts/evaluate.py` for experiments.

## Quick start

```bash
git clone https://github.com/dingzhen60027/kinova-motion-benchmark.git
cd kinova-motion-benchmark
```

Use the existing `.conda/envs/csdecomp` environment through the launcher; do not recreate it or install into Conda base.

Conda environments are local and are not included in Git. To use an existing environment from another checkout, set `CSDECOMP_CONDA_PREFIX` to that environment's absolute prefix. The default remains this project's `.conda/envs/csdecomp`.

```bash
# Input integrity, real GPU/solver checks, and available methods.
bash local/python.sh scripts/benchmark.py check
bash local/python.sh scripts/evaluate.py check
bash local/python.sh scripts/evaluate.py methods

# Inspect one fixed problem, then run the paper pipeline on both tracks.
bash local/python.sh scripts/benchmark.py show smoke-bottleneck-000-p00
bash local/python.sh scripts/evaluate.py run \
  --split smoke --problem smoke-bottleneck-000-p00 \
  --methods EI_ZO --track both --seeds 0 \
  --output /tmp/kinova-smoke-example

bash local/python.sh scripts/evaluate.py verify --run /tmp/kinova-smoke-example
```

Use a fresh output path when preparing a run. To continue it, use `resume`; completed outcomes, including failures, are retained. The launcher resolves the repository root internally. From another working directory, invoke `local/python.sh` by its absolute path; the script and configuration arguments above remain repository-relative.

The environment specification is [environment.yml](environment.yml), with exact package records in [local/conda-explicit.txt](local/conda-explicit.txt) and [local/requirements.lock.txt](local/requirements.lock.txt). Actual hardware, imports and solver checks are recorded in [results/checks.json](benchmarks/generic_motion_v1/results/checks.json). Run the complete regression suite with `bash local/python.sh -m pytest -q`; the shared pytest configuration avoids collisions between historical test-module names. Run the unchanged upstream tests with `bash local/python.sh -m pytest -q --import-mode=prepend csdecomp/tests/csdecomp_test.py csdecomp/tests/drake_dep_test.py`; their import mode must load the installed CSDecomp package rather than a collection-created namespace.

The verified host runs Ubuntu 22.04.5 LTS, Python 3.10.22, Drake 1.51.1 and Clarabel 0.11.1 on a Quadro RTX 8000 (48 GiB; NVIDIA driver 570.211.01). The environment check recorded OMP/OpenBLAS/MKL thread settings of 4/1/1. The launcher forces OpenBLAS and MKL to one thread and defaults OMP to four, honoring an explicit OMP override; serial workers inherit these settings. SCIP separately uses one solver and LP thread.

## Benchmark

| Split | Registered problems | Valid inputs | Generation failures |
|---|---:|---:|---:|
| smoke | 10 | 8 | 2 |
| dev | 60 | 60 | 0 |
| test | 420 | 395 | 25 |
| **Total** | **490** | **463** | **27** |

[benchmark/problems.jsonl](benchmarks/generic_motion_v1/problems.jsonl) is the single problem index; [splits.json](benchmarks/generic_motion_v1/splits.json) defines membership. Each problem references a stored scene and its original endpoint arrays. [features.csv](benchmarks/generic_motion_v1/features.csv) contains endpoint distance and validity, straight-line checks, obstacle/voxel counts, generation failures and validated feasibility witnesses. [metadata.json](benchmarks/generic_motion_v1/metadata.json) records sources, seeds, hashes, model identity and generation statistics.

The five environment categories are:

| Category | Actual geometry | Valid dev / test |
|---|---|---:|
| `sparse_clutter` | Sparse mixtures of spheres and boxes | 15 / 95 |
| `dense_clutter` | Dense spheres, with a different size distribution from sparse clutter | 15 / 85 |
| `bottleneck` | Two finite walls separated by a gap | 15 / 100 |
| `route_choice` | Two clusters of spheres | 15 / 95 |
| `open_control` | The fixed table/equipment scene without added random obstacles | 0 / 20 |

Scene randomness and endpoint randomness use separate SHA256-derived PCG64 streams. Planner seeds are repetitions of fixed problems, not new environments. No problem is selected according to a planner's success. Invalid generation records remain registered; a valid input without a verified path has unknown feasibility.

The two smoke generation failures are both dense-clutter inputs; that category has valid development and test coverage. Receipt-bound numerical path witnesses exist for 7 smoke, 49 dev and 362 test problems. The other 45 legal problems retain unknown feasibility; no failed planner result is treated as proof of infeasibility.

The legal test set contains **75 random obstacle layouts plus one shared base environment**, rather than 395 independent environments. The 25 test generation failures belong to five complete scenes. Dev has no open-control tasks; its short/medium/long distance ratio is 1:1:1, whereas legal test uses 2:2:1. Results therefore include category breakdowns and scene-cluster uncertainty. Test has historical evaluation exposure and is not an untouched holdout.

The category names describe workspace geometry: they do not establish C-space narrow passages or distinct homotopy classes. Sparse and dense clutter are not a density-only experiment. The conservatively voxelized obstacles can substantially reduce free space relative to their source primitives; the actual sphere union is the evaluated geometry for added obstacles; the fixed environment retains its model collision boxes.

To rebuild the reference index or independently regenerate a development split:

```bash
bash local/python.sh scripts/benchmark.py index
bash local/python.sh scripts/benchmark.py generate \
  --split dev --seed 20261027 --config configs/benchmark.json \
  --output /tmp/kinova-regenerated-dev
```

Regeneration writes to a separate destination and does not replace the canonical inputs. Current experiments use the existing 6,000-node roadmaps at seeds 0, 1 and 2. Rebuilding a roadmap is an offline operation:

```bash
bash local/python.sh scripts/benchmark.py build-roadmap \
  --nodes 6000 --seed 0 --output /tmp/kinova-roadmap-rebuild
```

## Model and validation

All builders and planners use the full Kinova model from [`kinova_sens_on_table.yml`](csdecomp/tests/test_assets/directives/kinova_sens_on_table.yml). Its seven joints are ordered as `kinova_joint_1_q` through `kinova_joint_7_q`; the metric is Euclidean distance in bounded joint coordinates, in radians, without angular wrapping. The base is fixed at the world origin, the Robotiq fingers are welded, and the TCP is `kinova::end_effector_link`. Joint limits, geometry dimensions and collision-filter pairs are recorded in [benchmark/configs/robot.json](benchmarks/generic_motion_v1/configs/robot.json).

The fixed table, camera/tripod assemblies and protective box remain present. Added obstacles are represented by a conservative union of spheres of radius **0.04 m**, with their actual float32 centers saved in each scene. Offline roadmap, online planning and final validation share this geometry; conservative search margins are separately recorded in the resolved evaluation configuration.

Every candidate is checked for fixed endpoints, joint limits, self/environment collision and required continuity. Collision validation uses native CUDA checks and an independent double-precision checker based on Drake poses and primitive distances. Their geometry and collision pairs are checked for agreement. Native planners retain their declared internal edge/check steps; every final output uses the same maximum **0.005 rad** validation step; points within the **±2 μm contact band** do not count as validated success. These are numerical checks, not a mathematical continuous collision certificate.

For the time track, all seven joints have velocity limits of **1 rad/s**, acceleration limits of **1 rad/s²**, and zero endpoint velocity. Endpoint acceleration is bounded but is not constrained to zero. Curve derivatives and segment continuity are checked independently of solver success. Path length is evaluated from the actual saved polygon or curve, rather than the solver's internal composite objective.

## Baselines

All methods use the model and validator above. The source review checks actual optimizer calls, constraints and stopping conditions; configuration names alone do not establish method identity. [Implementation evidence](benchmarks/generic_motion_v1/results/trend_diagnostics/method_identity_review.json) includes 12 source files verified against the official STOMP, SCSPlanning and TrajOpt commits. [methods.json](benchmarks/generic_motion_v1/results/methods.json) records detailed identities and completed coverage.

| Method ID | Actual implementation and initialization | Native objective | Tracks |
|---|---|---|---|
| `EI_ZO` | Native DRM → EI-ZO → author LSCS / SCSTO, with Clarabel; collision feedback, region editing and corridor repair retained | PWL length / Bézier trajectory duration in the corridor | Path, time |
| `LGCS` | Perspective GCS MISOCP reconstruction with SCIP; starts from the validated DRM path and EI-ZO regions | PWL length in the supplied region graph | Path |
| `NTO_PAPER` | Published-profile reconstruction with Drake KTO; SNOPT for path, IPOPT for time; validated DRM initialization | Path-length transcription / trajectory duration | Path, time |
| `NTO_REFINED` | NTO adaptation with clearance and iterative collision collocation; time track refines the initial spline to at least 16 controls without changing its shape | Same track objectives, richer time parameterization | Path, time |
| `NTO` | Legacy Drake KTO/IPOPT adaptation with a separate saved profile | Path-length transcription / trajectory duration | Path, time |
| `RRTCONNECT` | [Official OMPL 2.0.1](https://github.com/ompl/ompl/tree/2.0.1) RRTConnect and native vertex reduction; fixed endpoints | First feasible path, then shortening | Path |
| `RRTSTAR` | Official OMPL 2.0.1 RRT* with native rewiring and uninterrupted search | Joint-space path length | Path |
| `CHOMP` | Covariant functional-gradient implementation; validated DRM initialization | Acceleration smoothness and a C-space clearance integral | Path |
| `STOMP` | [Official STOMP core](https://github.com/ros-industrial/stomp) with canonical-geometry and correlated-noise callbacks; validated DRM initialization | Acceleration control cost and obstacle potential | Path |
| `TRAJOPT` | [Author SQP/BPMPD core](https://github.com/joschu/trajopt) with canonical collision collocation; validated DRM initialization | Squared joint differences with collision constraints | Path |

CHOMP uses a C-space clearance functional rather than the original workspace body-point functional. TrajOpt uses segment collocation rather than the original OpenRAVE swept-volume collision transcription. These are explicitly identified adaptations. LGCS and NTO are mathematical reconstructions, while EI-ZO, LSCS, SCSTO, OMPL, STOMP and the TrajOpt optimizer call their stated upstream implementations. NTO can leave its initial corridor. SCSTO optimizes trajectory geometry and segment durations; it is not just time parameterization.

Every path-track result is compared by the length of its actual saved path, even when its native objective includes other terms. A correct implementation may fail on a problem; success requires the independent output checks, not just a solver success flag. The Kinova suite evaluates these methods on its own fixed robot and task distribution.

### Completed evaluation and repair evidence

The serial evaluation uses runtime commit `6c3144ff64`; each `run_manifest.json` records the exact source hashes and resolved configuration. The shared-region run was prepared at publication commit `e3cef56b03`, with all 82 runtime source hashes, input hashes and roadmap files identical. All four repaired cohorts are complete:

| Run | Seeds | Registered calls | Legal calls |
|---|---|---:|---:|
| `baseline_smoke` | 0 | 140 | 112 |
| `baseline_dev` | 0, 1, 2 | 2,520 | 2,520 |
| `baseline_test` | 0 | 5,880 | 5,530 |
| `baseline_initial_regions` | 0 | 676 | 676 |

All **9,216 calls** are sealed and pass full receipt checks, with **zero infrastructure errors**. Smoke has 71 numerical successes; dev has 1,545. Test has **4,009 numerical successes, 759 searches without a solution, 140 timeouts, 369 validation failures, 253 solver failures and 350 invalid-input calls**. Those 350 calls represent the original 25 generation failures across 14 supported method–track combinations. Shared-region comparison has 671 successes and 5 timeouts over all 338 eligible pairs. [Test evidence](benchmarks/generic_motion_v1/results/logs/baseline_test_acceptance.json), [shared-region evidence](benchmarks/generic_motion_v1/results/logs/baseline_initial_regions_acceptance.json).

Development uses all 60 fixed problems and three paired seeds; test uses all 395 legal problems and seed 0. Success counts below describe reliability; cost comparisons use paired common-success problems.

| Method | Dev path / 180 | Dev time / 180 | Test path / 395 | Test time / 395 |
|---|---:|---:|---:|---:|
| `EI_ZO` | 131 | 131 | 338 | 338 |
| `LGCS` | 127 | — | 327 | — |
| `NTO_PAPER` | 44 | 101 | 113 | 269 |
| `NTO_REFINED` | 99 | 109 | 276 | 297 |
| `NTO` | 44 | 107 | 123 | 265 |
| `RRTCONNECT` | 146 | — | 362 | — |
| `RRTSTAR` | 129 | — | 316 | — |
| `CHOMP` | 130 | — | 336 | — |
| `STOMP` | 131 | — | 336 | — |
| `TRAJOPT` | 116 | — | 313 | — |

On the 338 test problems solved by the EI-ZO path pipeline, CHOMP and STOMP each validate 336 outputs; LGCS validates 327, with 11 timeouts; TrajOpt validates 313, with 25 collision failures. These methods share the same 54 frontend failures. Their full-flow success rates therefore reflect both finite-roadmap coverage and backend behavior. NTO_PAPER path retains 148 validation failures and 73 solver failures; its low validated success is not hidden or treated as an infrastructure crash. The refined profile is reported separately.

Selected test comparisons below use **competitor minus EI-ZO**, with 95% scene-cluster bootstrap intervals. Negative objective differences favor the competitor. Every row uses the same common-success queries for its cost and computation-time differences.

| Comparison / competitor | Common successes / scene clusters | Objective difference [95% CI] | Cold computation difference [95% CI] |
|---|---:|---:|---:|
| Full-flow path / RRTConnect | 338 / 75 | −0.286 rad [−0.534, −0.073] | −1.773 s [−2.445, −1.286] |
| Full-flow time / NTO_REFINED | 297 / 74 | −1.224 s [−1.424, −1.057] | +3.906 s [2.916, 4.997] |
| Shared initial regions / LGCS | 333 / 75 | −0.214 rad [−0.325, −0.123] | +5.191 s [4.111, 6.500] |

RRTConnect also solves 24 test problems that EI-ZO does not; EI-ZO has no exclusive success in that pair. NTO_REFINED time produces shorter trajectories on the 297 common successes, while EI-ZO solves another 41 problems. Shared-region cold time covers loading/revalidation and backend work; its source frontend cost is reported separately. Full-denominator outcome and latency statistics remain in [summary.csv](benchmarks/generic_motion_v1/results/summary.csv). The [dev](benchmarks/generic_motion_v1/results/trend_diagnostics/completed_dev_comparison.json) and [test](benchmarks/generic_motion_v1/results/trend_diagnostics/completed_test_comparison.json) repair comparisons preserve the earlier configurations and outcomes, including the historical shorter OMPL search cap.

The preceding native-first smoke at `6ad41275c7` and its **402/2,520** completed development prefix remain separate diagnostics. The prefix stopped safely when the region-handoff defect was reproduced; no active query was interrupted, no old test query had started, and no observation was selected by outcome to replace a repaired-cohort result.

These counts cover all **10 methods / 14 supported method–track combinations**. Use `evaluate.py status --run results/<run>` for coverage. Current summaries use `current_dev_repaired`, `current_test_repaired` and their explicitly separated conditional comparisons. Historical scores remain under `current_dev`, `current_test` and `current_dev_repaired_s000`; they are not substituted for repaired-cohort results.

The planning/runtime regression suite passed **1,288 tests**, and three unchanged upstream tests passed. Final catalog checks passed 13 tests; the offline-row plotting repair passed 12 targeted checks. Repairs and their reproducible evidence are:

- **DRM query correctness.** Failed endpoint edges no longer exclude otherwise free vertices; query-local state resets; isolated same-node queries and steering endpoints are handled. The quantized cache is reconciled against the original roadmap nodes at the actual online obstacle centers and radius. Independent Drake checks confirmed 2,316 cache-excluded configurations were free. The portable CPU sidecar retains the graph and CUDA implementation, with 48 targeted checks and six real-GPU positive/negative fixtures. [Evidence](benchmarks/generic_motion_v1/results/trend_diagnostics/drm_attachment_audit/comparison.json).
- **Endpoint/search budgets.** Dev-only diagnostics separate finite-graph coverage from exhausted attachment budgets. The current profile uses 1,000 planning attempts, 1,000 endpoint candidates per endpoint per attempt, 100,000 expansions and 0.01-rad steering; the internal 0.01-m padding is retained. OMPL now uses the remaining 60-second outer budget with explicit validation and simplification reserves. Historical 10-second OMPL results remain separate.
- **NTO trajectory freedom.** In an actual 2-rad rest-to-rest IPOPT control problem, four cubic controls produced 6.000 s; 16 controls produced 3.015762 s against a 3 s lower bound. Shape-preserving insertion changed the initial curve by at most 4.44e-16 rad. `NTO_REFINED` uses this refinement; the published-profile reference remains separate.
- **Drake contact-gradient recovery.** Only the known [near-contact BoxBoxGradient assertion](https://github.com/RobotLocomotion/drake/issues/21675) is recovered. Every canonical collision pair and the native aggregate constraint remain present; unknown errors fail explicitly. Normal solves bind the native evaluator directly. Only this exact assertion permits one guarded restart per query, using the original remaining deadline and initial guess; subsequent refinement rounds keep the guarded evaluator. Sixty-nine targeted checks and seven independent real-Drake API checks pass. The end-to-end GPU regression reproduced the exception and retained the colliding candidate as a failure. Normal-path replay matched all 415 original IPOPT iteration rows and the final trajectory arrays. [Evidence](benchmarks/generic_motion_v1/results/trend_diagnostics/nto_collision_independent_review.json).
- **Native region handoff.** A real recovery region needed 23 float32 increments, exceeding the old arbitrary limit of 16 despite remaining inside the existing tolerance. The fallback now searches only representable offsets inside both original correction bounds, with the complete native matrix and fixed anchors. Independent CPU replay found the least passing offset at a normalized change of 1.724e-7, below the unchanged 1e-6 limit; five normal historical regions retained byte-identical arrays and records. Final collision validation remains mandatory. [Evidence](benchmarks/generic_motion_v1/results/trend_diagnostics/native_handoff_independent/review.json).
- **Failure accounting.** Unknown exceptions, CUDA errors and process crashes retain infrastructure-failure status even when a deadline follows. A numerical handoff rejection requires its typed error and bound-checked, receipt-bound evidence. A source-specific reader preserves 1,368 normal no-route outcomes from the older DRM implementation without accepting that old exception in the current runtime. [Evidence](benchmarks/generic_motion_v1/results/trend_diagnostics/exception_classification_applied.json).
- **Result export.** Offline roadmap rows have no comparison membership. The final plotting filter handles those rows without removing their costs from the full CSV. This postprocessing repair was applied after all timed runs; the original export error, frozen runtime hashes and regression checks are preserved. [Evidence](benchmarks/generic_motion_v1/results/trend_diagnostics/completed_offline_figure_fix.json).

The actual LGCS GPU regression for `dev-dense_clutter-002-p01` retains the same input, seed, roadmap, configuration and original failing handoff arrays. After the repair it passes the common 1,278-sample validation with a 6.346409-rad path in 22.25 s end to end. The initial colliding candidate and old failed query remain saved. This is one development regression, not a full-test success rate. [Run evidence](benchmarks/generic_motion_v1/results/trend_diagnostics/handoff_repair_gpu_verification.json).

[Paper/source checks](benchmarks/generic_motion_v1/results/trend_diagnostics/literature.json) compare Werner et al., TrajOpt, STOMP, MotionBenchMaker and recent accelerated planners with their actual robots, inputs, initialization and timing definitions. They help identify suspicious behavior; they do not prescribe a universal ranking. Finite-roadmap coverage and sampled collision constraints remain real limitations. A failed query does not prove infeasibility, and a numerically validated output is not a continuous collision certificate.

## Evaluation

[configs/evaluation.json](configs/evaluation.json) selects the existing method settings; each run saves the fully resolved configuration. The outer budget is **60 seconds per query**, with one GPU query at a time. This cap includes process startup, model/dependency setup, GPU warm-up, frontend search, region generation, optimization, repairs, validation and output. An internal stopping criterion may finish earlier: OMPL searches within the remaining outer deadline, reserving 5 seconds for validation and 0.5 seconds for bounded vertex reduction; its explicit native ceiling is 60 seconds. Historical runs used a 10-second native cap and remain separate. Offline roadmap construction time, node counts and storage are reported once per roadmap identity, separately from online query statistics; reuse across methods and queries does not multiply the offline cost.

Two tracks are reported separately:

- **Path:** actual joint-space path length, in radians.
- **Time:** validated trajectory execution duration, in seconds, under the common motion limits.

Computation latency is not trajectory duration. `cold_e2e_seconds` is external end-to-end wall time; `within_call_query_seconds` subtracts one-time setup from that same cold run and is not a separately measured warm run. Historical dev logs support serial benchmark execution but do not prove the absence of unrelated GPU load; their timings are kept separate from the new test run. A first-valid time is recorded only when supported by a validation event. The installed OMPL wheel does not bind its native intermediate-solution callback; the main baseline does not interrupt search to produce a curve. Its unavailable first-valid timestamps remain empty. The separate observer diagnostic records actual validation events and their overhead, with no retrospective early-success times.

The result selection distinguishes three comparisons:

- **End-to-end:** every legal input remains in the denominator, including frontend failures.
- **Fixed frontend:** methods share the byte-identical, independently validated DRM path. Frontend failures are excluded from this conditional backend comparison. A method whose saved paths differ is excluded from this entire conditional comparison, while its full-flow results remain available.
- **Fixed initial regions:** both LSCS and LGCS load the same receipt-bound initial EI-ZO path, regions and corridor. Each receives a fresh 60-second budget including setup and source revalidation; source-load failures remain in the predeclared denominator. Initial corridor generation is reported separately as recorded source stages. Later graph expansion and collision repair remain part of each method. The repaired comparison includes all 338 eligible source-stage pairs; eligibility does not depend on final optimizer success. The older 283-pair experiment remains historical evidence.

The summary keeps each mode separate. On the same problem and planner seed it reports common-success objective and cold-time differences, plus common-success backend-time differences for fixed-frontend comparisons. All-finished latency differences remain a separate statistic, so an early failure cannot become a common-success speed advantage. Four paired outcome counts—both succeed, only A succeeds, only B succeeds, neither succeeds—share the same matched legal denominator. Each metric reports its actual nonmissing sample and scene-cluster counts. Confidence intervals resample complete scene clusters, keeping correlated queries/repetitions together; all open-control tasks share one physical cluster. Single-cluster results have no bootstrap interval.

Unified terminal statuses are `success_numerical`, `invalid_input`, `no_solution_within_budget`, `timeout`, `solver_failure`, `validation_failed` and `infrastructure_error`. Success requires a saved candidate, passing common validation, intact artifact receipts and normal process completion within the query budget. Solver convergence is a separate field: a validated output from a nonconverged solver can qualify, while a converged solver with an invalid output cannot. Raw statuses and failure evidence remain unchanged. Timeout wall time includes process termination and cleanup, so it can slightly exceed 60 seconds. A failed search does not establish infeasibility. Missing measurements remain empty, and no fallback algorithm is silently credited to another method.

## Run, resume, summarize and replay

```bash
# Dev: three paired seeds. Only supported method/track combinations are prepared.
bash local/python.sh scripts/evaluate.py batch \
  --split dev --methods EI_ZO RRTCONNECT --track path --seeds 0 1 2 \
  --output /tmp/kinova-dev-example --prepare-only
bash local/python.sh scripts/evaluate.py resume --run /tmp/kinova-dev-example
bash local/python.sh scripts/evaluate.py status --run /tmp/kinova-dev-example
bash local/python.sh scripts/evaluate.py verify --run /tmp/kinova-dev-example

# Complete the saved serial smoke/dev/test and paired-region evaluation, then verify and summarize.
bash local/python.sh results/logs/complete_repair_evaluation.py

# Or resume an individual run; existing outcomes are not repeated.
bash local/python.sh scripts/evaluate.py resume --run results/baseline_smoke
bash local/python.sh scripts/evaluate.py resume --run results/baseline_dev
bash local/python.sh scripts/evaluate.py resume --run results/baseline_test

# Reproduce the complete paired shared-initial-region comparison in a new run.
bash local/python.sh scripts/evaluate.py batch \
  --split test --methods EI_ZO LGCS --track path --seeds 0 \
  --mode fixed_initial_regions --initial-source-run results/baseline_test \
  --output /tmp/kinova-shared-initial-regions --prepare-only
bash local/python.sh scripts/evaluate.py resume --run /tmp/kinova-shared-initial-regions

# Refresh only the explicitly selected comparisons.
bash local/python.sh scripts/evaluate.py summarize \
  --selection results/selection.json --output results
```

[results/selection.json](benchmarks/generic_motion_v1/results/selection.json) names the exact runs and methods used in each comparison. Fresh runs outside this selection remain available as raw evidence; add their path to the appropriate comparison before summarizing them. Avoid adding overlapping observations of the same problem, seed, method and objective.

For replay, use the `attempt_path` from `records.csv`:

```bash
bash local/python.sh scripts/evaluate.py replay \
  --attempt '<attempt_path from records.csv>' --output /tmp/kinova-replay
```

Actual test examples are available as self-contained [successful replay](benchmarks/generic_motion_v1/results/figures/repaired_success/replay.html) and [failed replay](benchmarks/generic_motion_v1/results/figures/repaired_failure/replay.html); open the downloaded HTML locally. Every displayed joint state matches the saved validator arrays. The failed TrajOpt example retains a sampled clearance of −1.80 mm and is counted as a failure. [Replay checks](benchmarks/generic_motion_v1/results/logs/completed_replay_acceptance.json).

Outputs are centralized:

| File or directory | Contents |
|---|---|
| [results/records.csv](benchmarks/generic_motion_v1/results/records.csv) | One row per actual attempt, including unsuccessful attempts, source/input/config hashes, times, validation and artifact paths |
| [results/summary.csv](benchmarks/generic_motion_v1/results/summary.csv) | Selected comparisons, coverage, per-category status, paired outcomes/costs/times, cluster intervals and distinct offline roadmap costs |
| [results/inventory.json](benchmarks/generic_motion_v1/results/inventory.json) | Existing run coverage, pending run paths and accounting rules |
| [results/figures/](benchmarks/generic_motion_v1/results/figures/) | Success rate, time breakdown and measured progress when available |
| [results/trajectories/](benchmarks/generic_motion_v1/results/trajectories/), [results/logs/](benchmarks/generic_motion_v1/results/logs/) | Run-level links to original paths, coefficients, regions, process/solver logs and receipts |

All attempts are indexed. Summary selection uses the first terminal attempt per job; interrupted attempts explicitly awaiting resume remain visible. It never selects the best retry. New observations can be indexed repeatedly without overwriting raw attempts. New run plans reference the original fixed problems instead of copying their obstacle arrays into every job. Source provenance uses Git commits and hashes; commit adapter changes before preparing a run, and keep runtime code fixed while it is pending.

Development contains 180 calls per supported method–track combination. Main OMPL evaluation uses uninterrupted native search. The incumbent-observer diagnostic was stopped after 127 terminal calls and one preserved interruption because repeated validation consumed search time; it remains a separate diagnostic group. Original full-flow, legacy NTO and shared-initial-region calls retain separate run plans and evidence.

## Repository layout

```text
README.md
local/                         Existing Conda launcher and package locks
environment.yml                Link to local/environment.yml
configs/benchmark.json         Generator configuration
configs/evaluation.json        Evaluation overrides
scripts/benchmark.py           Data CLI
scripts/evaluate.py            Evaluation CLI
benchmark/                     Public alias of benchmarks/generic_motion_v1/
  problems.jsonl               Unified task-reference index
  splits.json, features.csv    Membership and task characteristics
  metadata.json                Model, sources, hashes and counts
  scenes/, inputs/, manifests/ Frozen original inputs
  generator/, runners/         Generation, adapters, validation and analysis
  tests/, roadmaps/            Regression tests and offline roadmaps
results/                       Public alias of benchmark/results/
  records.csv, summary.csv
  selection.json, inventory.json
  figures/, trajectories/, logs/
  baseline_smoke/, baseline_dev/, baseline_test/
                             Complete repaired full-flow evidence
  baseline_initial_regions/  Complete paired shared-initial-region evidence
  smoke/, test/, repaired_dev/
                             Historical full-flow and seed-0 evidence
  repaired_native_smoke/, repaired_dev_all_seeds/
                             Earlier diagnostic evidence, including failures
  trend_diagnostics/         Literature checks, repairs and paired evidence
  nto_legacy_smoke/, nto_legacy_dev/, nto_legacy_test/
  fixed_initial_regions/     Paired shared-initial-region experiment
  dev_sampling/              Interrupted observer diagnostic
csdecomp/, examples/            Preserved upstream library and examples
```

`benchmark/` and `results/` are relative symbolic links, not duplicate datasets or experiment trees. The physical package path is retained to preserve imports and historical evidence references. New methods extend the existing adapter/registry and configuration, and reuse the dataset loader, runner, validator and result exporter.

The [benchmark background comparison](benchmarks/generic_motion_v1/BENCHMARK_OVERVIEW.md) retains earlier literature context. Historical Forest/FR3 investigations and earlier Kinova runs remain in their original locations because they contain unique trajectories and failure evidence. They are not part of the current main comparison unless explicitly selected. The historical test results have their own comparison group; no earlier score is silently substituted for a new run.

The native dependency sources, licenses and build receipts remain under `benchmark/native/` and `benchmark/reports/extended_baselines_v1/`. The isolated OMPL, STOMP and TrajOpt backends use the existing environment; their setup commands and exact build records remain available in Git history and the recorded source audits. `results/methods.json` records the current implementation checks.

## Source and citation

The core method is [*Superfast Configuration-Space Convex Set Computation on GPUs for Online Motion Planning*](https://www.roboticsproceedings.org/rss21/p045.html), Werner et al., RSS 2025. The [MIT license](LICENSE), upstream source attribution and dependency/model licenses are preserved. [README.legacy.md](README.legacy.md) retains the upstream build and toolbox documentation.

Sealed experiment records retain their original source commit IDs. `benchmark/provenance/` preserves exact source exports for commits predating this repository; verification still checks the SHA-256 values recorded with each experiment.
