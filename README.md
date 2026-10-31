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

The two smoke generation failures are both dense-clutter inputs; that category has valid development and test coverage. Receipt-bound numerical path witnesses currently exist for 7 smoke, 49 dev and 358 test problems. The other 49 legal problems retain unknown feasibility; no failed planner result is treated as proof of infeasibility.

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

## Methods

Stored method IDs are stable. Their implementations and objectives are intentionally distinguished:

| ID | Implementation and initialization | Test path / 395 | Test time / 395 |
|---|---|---:|---:|
| `EI_ZO` | Native DRM and EI-ZO; author LSCS/SCSTO with Clarabel. Collision feedback, region editing and corridor repair are retained. | 283 | 283 |
| `LGCS` | Perspective GCS MISOCP reconstruction, solved with SCIP, initialized from the same validated DRM path. | 282 | — |
| `NTO_PAPER` | Published-profile reconstruction using Drake KTO, SNOPT for path and IPOPT for time; initialized from the validated DRM path. | 108 | 239 |
| `NTO_REFINED` | Separately identified NTO adaptation with positive clearance and iterative collision collocation. | 240 | 266 |
| `NTO` | Legacy Drake KTO/IPOPT adaptation: original DRM controls for path; shape-preserving knot insertion for time. | 114 | 239 |
| `RRTCONNECT` | [Official OMPL 2.0.1](https://github.com/ompl/ompl) RRTConnect and native vertex reduction, searching directly from the fixed endpoints. | 358 | — |
| `RRTSTAR` | Official OMPL 2.0.1 RRT* and native vertex reduction, with uninterrupted native search and common final validation. | 289 | — |
| `CHOMP` | CHOMP adaptation: acceleration-metric covariant descent on a C-space smoothness/obstacle functional. Uses the validated DRM path. | 282 | — |
| `STOMP` | Unmodified [official STOMP core](https://github.com/ros-industrial/stomp) with canonical-geometry task and correlated-noise callbacks. Uses the validated DRM path. | 282 | — |
| `TRAJOPT` | TrajOpt adaptation: [author trust-region SQP/BPMPD core](https://github.com/joschu/trajopt) with sampled canonical collision constraints. Uses the validated DRM path. | 264 | — |

Counts are numerically validated full-flow outputs from `current_test`; a dash denotes an unsupported track. Every supported combination also completed all 60 development problems at seeds 0, 1 and 2. Per-category, failure and paired quality statistics are in [summary.csv](benchmarks/generic_motion_v1/results/summary.csv).

CHOMP, STOMP and TrajOpt have native smoothness/collision objectives; their reported comparison metric remains validated path length. CHOMP uses a C-space arc-length collision functional rather than the original workspace body-point functional. The TrajOpt adapter uses segment collocation rather than the original OpenRAVE swept-volume implementation. NTO can leave its initial DRM corridor while retaining the common collision and motion constraints. `NTO_PAPER` and `NTO_REFINED` use a declared 2,000-iteration cap; legacy `NTO` uses IPOPT with a 500-iteration cap. Numerical tolerances and initial-duration policies are recorded local settings. The Kinova suite evaluates the paper's planning pipeline on this robot; it does not change the robot to match a method. The `NTO` legacy adaptation is a separate active method; it is not an alias for `NTO_PAPER` or `NTO_REFINED`. Earlier NTO runs used an older roadmap profile and are kept separate.

All **10 methods / 14 supported method–track combinations** have completed the selected smoke, development and test evaluations. The regression suite passes **1,129 tests**, plus **three unchanged upstream tests**. Completed coverage, including the separate legacy NTO runs, is:

| Evaluation | Paired seeds | Registered calls | Legal calls | Numerically validated outputs |
|---|---|---:|---:|---:|
| smoke | 0 | 140 | 112 | 40 |
| dev | 0, 1, 2 | 2,520 | 2,520 | 1,364 |
| test | 0 | 5,880 | 5,530 | 3,529 |

These are accounting totals across methods and tracks, not an aggregate performance score. The separate shared-initial-region experiment completed **283 eligible task pairs / 566 backend calls**, with 565 numerically validated outputs and one timeout. Eligibility depends on the saved initial path and corridor, not the original optimizer's final success. Detailed method comparisons remain in `summary.csv`.

## Evaluation

[configs/evaluation.json](configs/evaluation.json) selects the existing method settings; each run saves the fully resolved configuration. The outer budget is **60 seconds per query**, with one GPU query at a time. This cap includes process startup, model/dependency setup, GPU warm-up, frontend search, region generation, optimization, repairs, validation and output. An internal stopping criterion may finish earlier: OMPL currently has a 10-second search budget and bounded vertex reduction. Offline roadmap construction time, node counts and storage are reported once per roadmap identity, separately from online query statistics; reuse across methods and queries does not multiply the offline cost.

Two tracks are reported separately:

- **Path:** actual joint-space path length, in radians.
- **Time:** validated trajectory execution duration, in seconds, under the common motion limits.

Computation latency is not trajectory duration. `cold_e2e_seconds` is external end-to-end wall time; `within_call_query_seconds` subtracts one-time setup from that same cold run and is not a separately measured warm run. Historical dev logs support serial benchmark execution but do not prove the absence of unrelated GPU load; their timings are kept separate from the new test run. A first-valid time is recorded only when supported by a validation event. The installed OMPL wheel does not bind its native intermediate-solution callback; the main baseline does not interrupt search to produce a curve. Its unavailable first-valid timestamps remain empty. The separate observer diagnostic records actual validation events and their overhead, with no retrospective early-success times.

The result selection distinguishes three comparisons:

- **End-to-end:** every legal input remains in the denominator, including frontend failures.
- **Fixed frontend:** methods share the byte-identical, independently validated DRM path. Frontend failures are excluded from this conditional backend comparison. A method whose saved paths differ is excluded from this entire conditional comparison, while its full-flow results remain available.
- **Fixed initial regions:** both LSCS and LGCS load the same receipt-bound initial EI-ZO path, regions and corridor. Each receives a fresh 60-second budget including setup and source revalidation; source-load failures remain in the predeclared denominator. Initial corridor generation is reported separately as recorded source stages. Later graph expansion and collision repair remain part of each method. Two of the 283 available original test pairs had different independently generated regions; those full-flow results are preserved, and the shared-snapshot experiment reruns the complete eligible cohort.

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

# Resume is safe for completed runs: existing outcomes are not repeated.
bash local/python.sh scripts/evaluate.py resume --run results/test
bash local/python.sh scripts/evaluate.py resume --run results/nto_legacy_smoke
bash local/python.sh scripts/evaluate.py resume --run results/nto_legacy_dev
bash local/python.sh scripts/evaluate.py resume --run results/nto_legacy_test
bash local/python.sh scripts/evaluate.py resume --run results/fixed_initial_regions

# Reproduce the complete paired shared-initial-region comparison in a new run.
bash local/python.sh scripts/evaluate.py batch \
  --split test --methods EI_ZO LGCS --track path --seeds 0 \
  --mode fixed_initial_regions --initial-source-run results/test \
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
  smoke/, test/               Original full-flow run evidence
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
