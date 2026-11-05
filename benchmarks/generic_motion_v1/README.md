# Benchmark reference

[Project overview](../../README.md) · [Conda and native dependencies](../../local/README.md)

Commands below run from the repository root. `benchmark/` and `results/` are aliases of this package and its results directory; there is one physical dataset. Original manifests and sealed artifacts remain at their recorded paths.

## Dataset

| Split | Registered | Valid inputs | Generation failures | Purpose |
|---|---:|---:|---:|---|
| `smoke` | 10 | 8 | 2 | Functional checks |
| `dev` | 60 | 60 | 0 | Configuration and regression tests |
| `test` | 420 | 395 | 25 | Designated baseline evaluation set |
| **Total** | **490** | **463** | **27** | |

[Problems](problems.jsonl) reference stored scenes and endpoints. [Splits](splits.json), [features](features.csv), and [metadata](metadata.json) record membership, endpoint checks/distances, direct-connection checks, obstacle counts, rejection reasons, seeds and hashes.

| Category | Actual workspace geometry | Valid dev / test |
|---|---|---:|
| `sparse_clutter` | Mixture of spheres and boxes | 15 / 95 |
| `dense_clutter` | Dense spheres with a different size distribution | 15 / 85 |
| `bottleneck` | Two finite walls separated by a gap | 15 / 100 |
| `route_choice` | Two clusters of spheres | 15 / 95 |
| `open_control` | Fixed table/equipment without added random obstacles | 0 / 20 |

- Scene and endpoint sampling use independent SHA256-derived PCG64 streams. All seven joints are sampled. Input acceptance uses geometry, endpoint validity and distance criteria, never planner success. Planner seeds repeat fixed problems.
- Test contains **75 random layouts plus one shared base environment**; problems within a layout are correlated. Dev/test random layouts are disjoint. Open-control queries share the base environment. Test has historical evaluation exposure: it is **not an untouched holdout**.
- Dev uses equal short/medium/long proportions; valid test uses 2:2:1. Both smoke generation failures are dense-clutter inputs. All failures remain registered.
- Sparse/dense clutter is not a density-only comparison. Category names do not establish C-space passages or homotopy classes; open control is not obstacle-free.
- Input validity does not prove solvability. **418** problems have stored numerical path witnesses; **45** valid problems retain unknown feasibility. A planning failure is not a generation failure or proof of infeasibility.

## Model and evaluation protocol

Effective settings: [evaluation.json](../../configs/evaluation.json). Joint limits, geometry and collision pairs: [robot.json](configs/robot.json).

| Item | Shared definition |
|---|---|
| Robot | Full [Kinova model](../../csdecomp/tests/test_assets/directives/kinova_sens_on_table.yml); fixed base, welded Robotiq fingers; TCP `kinova::end_effector_link` |
| Joint metric | `kinova_joint_1_q` through `kinova_joint_7_q`; bounded Euclidean coordinates in radians, without angular wrapping |
| Obstacles | Fixed table/equipment plus conservative unions of 0.04 m radius spheres at frozen float32 centers |
| Path objective | Actual saved PWL/curve length; composite solver costs are not substituted |
| Time objective | Duration with velocity ≤ 1 rad/s, acceleration ≤ 1 rad/s², zero endpoint velocity; endpoint acceleration bounded, not fixed to zero |
| Validation | Endpoints, bounds, continuity, self/environment collision and applicable derivatives; CUDA plus independent double-precision geometry checks |
| Collision resolution | Maximum final joint step 0.005 rad; the ±2 μm contact band does not count as validated success |
| Online budget | 60 s/query including startup, setup, frontend, optimization, repair, validation and output; one GPU planning job at a time |
| Offline work | 6,000-node roadmaps at seeds 0, 1, 2; construction time and storage reported separately |
| Repetitions | First terminal attempt per declared job; no best-of retry selection |

**Comparison modes.** Full-flow evaluation includes each method's frontend. Fixed-frontend summaries condition on an identical validated seed path. `fixed_initial_regions` gives EI-ZO and LGCS the same saved path and initial regions under a fresh backend budget; source frontend costs remain separate. These modes are not pooled.

**Timing.** Computation time and trajectory execution time are distinct. Cold end-to-end time includes startup; `within_call` decomposes that run and is not a separately measured warm-start experiment. First-valid-solution times and progress curves appear only when actually observed. OMPL reserves budget for validation and simplification. Effective settings and thread policies are captured per run.

**Validation and status.** Solver convergence and numerical validity are separate fields. Numerical sampling is not a continuous mathematical certificate; timeout does not prove infeasibility. Statuses are `success_numerical`, `invalid_input`, `no_solution_within_budget`, `timeout`, `solver_failure`, `validation_failed`, and `infrastructure_error`.

## Methods

Ten method IDs, including three configurations of the NTO family, support 14 method–track combinations. All share the model and final validator. Except for independent OMPL planners, methods start from an independently validated DRM path.

| Method ID | Implementation | Native objective | Tracks |
|---|---|---|---|
| `EI_ZO` | Native DRM → EI-ZO → author LSCS / SCSTO with Clarabel; collision feedback and region/corridor repair retained | Corridor path length / duration | Path, time |
| `LGCS` | Perspective GCS MISOCP reconstruction using SCIP | Path length in the supplied region graph | Path |
| `NTO_PAPER` | Published-profile reconstruction in Drake KTO; SNOPT path / IPOPT time | Path-length transcription / duration | Path, time |
| `NTO_REFINED` | NTO adaptation with clearance, adaptive collision collocation and richer time splines | Path-length transcription / duration | Path, time |
| `NTO` | Historical Drake KTO/IPOPT profile with a 500-iteration limit | Path-length transcription / duration | Path, time |
| `RRTCONNECT` | Official OMPL 2.0.1 RRTConnect and native vertex reduction | First feasible path, then shortening | Path |
| `RRTSTAR` | Official OMPL 2.0.1 RRT* with native rewiring and vertex reduction | Joint-space path length | Path |
| `CHOMP` | Covariant-gradient adaptation with a C-space clearance functional | Smoothness + obstacle cost | Path |
| `STOMP` | Official ros-industrial core with canonical collision and correlated-noise callbacks | Smoothness + obstacle cost | Path |
| `TRAJOPT` | Author SQP/BPMPD core with canonical collision collocation | Squared joint differences + collision constraints | Path |

CHOMP's C-space functional and TrajOpt's sampled collision constraints differ from the original workspace-body and swept-volume formulations. LGCS and NTO are mathematical reconstructions. SCSTO optimizes geometry and timing as the time backend of `EI_ZO`; it is not an additional method ID. Identity and source details are in the [method catalog](results/methods.json) and [source review](results/trend_diagnostics/method_identity_review.json).

## Results and evidence

The current table in the [project overview](../../README.md) uses the complete `NTO_REFINED` retest at runtime commit `f4446e54ec`, alongside the other methods' existing `baseline_dev` / `baseline_test` runs at `6c3144ff64`. **Dev: 60 problems × seeds 0, 1, 2. Test: 395 valid problems × seed 0 only. Test seeds 1 and 2 have not been run.** Manifests preserve source hashes and effective settings.

The NTO retest completed all **1,220 registered calls** (1,166 legal inputs and 54 invalid-input calls), with no native crashes or infrastructure errors. It uses the fixed METIS/MC64 solver policy and keeps experimental early return disabled. Counts below are independently validated successes:

| Split and repetitions | Path | Time |
|---|---:|---:|
| Smoke: 8 legal problems × seed 0 | 4 / 8 | 4 / 8 |
| Dev: 60 problems × seeds 0, 1, 2 | 99 / 180 | 116 / 180 |
| Test: 395 problems × seed 0 | 276 / 395 | 302 / 395 |

Both the complete-flow and fixed-frontend comparisons include all applicable existing baselines. All shared frontend path hashes agree. Test path outcomes and common-success costs exactly match the previous refined run; time has 14 newly successful and 9 formerly successful problems, for 297 → 302 successes. The [matched comparison](results/nto_metis/comparison/paired_before_after_summary.csv) records these as observations across source revisions, not a controlled single-factor ablation. `NTO_PAPER` and legacy `NTO` retain their historical runs.

The historical smoke/dev/test cohorts contain 8,540 recorded calls, including explicit invalid inputs. A separate shared-initial-region comparison adds 676 calls on 338 eligible problems. All 9,216 declared historical calls completed, with zero infrastructure errors. Their original CSVs and evidence remain unchanged.

| Evidence | Use |
|---|---|
| [records.csv](results/nto_metis/comparison/records.csv) | Current NTO retest and the existing baseline controls; filter by run identity |
| [summary.csv](results/nto_metis/comparison/summary.csv) | Outcomes, latency, objectives and paired differences; filter by comparison identity |
| [selection.json](results/nto_metis/comparison/selection.json) | Complete-flow and fixed-frontend comparisons for dev/test |
| [Retest verification](results/nto_metis/comparison/verification.json) | All 1,220 new calls, six new/control run checks and 1,003 recorded IPOPT parameter files |
| [Historical summary](results/summary.csv) | Original comparison and diagnostic cohorts, retained unchanged |
| [Completion checks](results/rigor_checks/completed_evaluation.json) | Run completion, receipts, input/protocol identity and metrics |
| [Input/regression checks](results/rigor_checks/checks.json) | Model/input rechecks, test evidence and historical-result preservation |

Cost comparisons use paired common-success problems, accompanied by full-denominator success rates and uncertainty clustered by physical scene. Do not pool historical cohorts or compare only each method's own success subset. The later runtime passed 1,330 regression tests and three upstream tests; 28 further planner calls recorded 10 successes and 18 classified failures/timeouts as separate functional diagnostics. NTO's collisions between constraint points remain validation failures even when the solver reports convergence.

### Correctness checks

The [November 2026 checks](results/rigor_checks/strict_check_20261105/verification.json) preserve the frozen inputs and published CSVs. They independently recompute all endpoint proposals and direct-line samples, cross-check all 926 accepted endpoints in a separately constructed Drake scene, and check conservative obstacle coverage. All 9,216 primary calls and paired statistics were reconciled against their saved evidence. The lowest-clearance successful DEV output for each of the 14 method–track combinations also passed fresh CUDA/Drake validation.

Run summaries now share numerical acceptance with the primary CSV reader, retain the first terminal attempt, and preserve raw solver status. Verification and resume use the same terminal rule; later interrupted attempts cannot rerun completed jobs. Model audits write to a new evidence directory and cannot replace the canonical robot inventory. The final regression passed **1,387 tests**, plus three upstream tests; intermediate failures and subsequent passing regressions remain recorded.

A fresh all-method smoke query recorded **9 numerical successes, 3 validation failures, 1 solver failure and 1 timeout**. These are functional checks, excluded from the published score. Two CPU replays and a same-input GPU replay (60-second timeout, identical frontend/initial spline) did not reproduce the historical native `libdrake` SIGSEGV; its root cause remains unresolved. All attempts remain recorded. Finite tests and collision samples do not establish universal absence of bugs or continuous collision freedom.

The subsequent [official-source and native-route checks](results/rigor_checks/upstream_review_20261105/verification.json) compare CSDecomp against its official repository, issues and wheel. The installed Drake 1.51.1 pins IPOPT 3.14.19; [IPOPT #848](https://github.com/coin-or/Ipopt/pull/848) fixes a later SPRAL memory-lifetime defect. Address-level inspection localizes the historical fault offset in the current binary to SPRAL's `mo_split` matching-order function; the original crash lacks a sealed binary hash or full stack, so #848 is not an established cause.

**New NTO runs explicitly use `linear_solver=spral`, `spral_order=metis`, `spral_scaling=mc64`.** Setting only the order is insufficient: IPOPT's matching scaling can override it. A real GDB comparison on identical initial variables and constraints recorded 21 entries into `mo_split` with matching and zero with METIS/MC64. This configuration avoids that faulting path without changing the model, objectives, constraints, endpoints or budget. The Conda environment and native libraries remain unchanged. Effective options and the actual loaded library SHA are saved before solving. Historical scores retain their original configuration; new results are separate. The complete regression after this change passed **1,412 tests**.

Nine subsequent GPU calls recorded four numerical successes, three validation failures, one solver failure and one timeout, with no native crash. The original crash query, `dev-dense_clutter-001-p01`, used its original early-return profile with only the linear-system settings changed: its identical saved frontend and initial spline produced a numerically valid **8.50 s trajectory in 41.83 s of computation**. This single-query diagnostic is not a new population score or proof of the underlying native defect's cause.

```bash
# Read-only verification of this delivery; write a new JSON destination.
bash local/python.sh results/rigor_checks/strict_check_20261105/verify_delivery.py \
  --output runs/correctness-check.json
bash local/python.sh results/rigor_checks/upstream_review_20261105/verify.py \
  --output runs/upstream-check.json
```

### NTO profiles and failure analysis

Use `NTO_REFINED` as the strengthened NTO comparison profile. `NTO_PAPER` preserves the published-parameter reconstruction, and `NTO` preserves the historical adaptation. These are three configurations of one method family. The reconstruction uses local Drake code; the author's NTO implementation is unavailable in the supplied upstream repository.

| Setting | `NTO_PAPER` | `NTO_REFINED` | `NTO` (legacy) |
|---|---|---|---|
| Path / time solver | SNOPT / IPOPT | SNOPT / IPOPT | IPOPT / IPOPT |
| Collision constraint locations | 149 interior points, `k/150` | Same initial points; adaptive refinement up to 600 points / 4 rounds | 150 interior points, `k/151` |
| Required clearance | 0 mm | 1 mm, with a 0.5 mm refinement guard | 0.01 mm |
| Solver iteration limits | SNOPT major 2,000, total 10,000; IPOPT 2,000 | SNOPT major 2,000, total 100,000; IPOPT 2,000 | IPOPT 500 |
| Time spline | Cubic, DRM nodes + duplicated endpoints | Same initial curve, knot insertion to at least 16 controls | Knot insertion to at least 16 controls |

All profiles use the same model, inputs, 60-second full-flow budget and final validator. Path optimization uses order-2 splines, whose control-polygon length is the actual PWL length. Time optimization uses the same motion limits; Drake enforces derivative bounds through conservative spline control-hull constraints ([KTO documentation](https://drake.mit.edu/doxygen_cxx/classdrake_1_1planning_1_1trajectory__optimization_1_1_kinematic_trajectory_optimization.html)). SNOPT's major and total iteration limits are distinct ([solver documentation](https://ccom.ucsd.edu/~optimizers/docs/snopt/options.html)).

The historical [receipt/count audit](results/rigor_checks/nto_profiles/counts.json) checks all 3,600 registered dev/test NTO calls and 99,387 sealed files. Counts agree with the original CSVs. On test, all six historical profile–track combinations share **338 identical validated frontend paths**; the remaining 57 problems have the same frontend failure (54 searches without a solution, 3 timeouts). Historical backend outcomes on those 338 paths are:

| Profile / track | Numerically valid | Validation failed | Solver failed | Backend timeout |
|---|---:|---:|---:|---:|
| Published / path | 113 | 148 | 73 | 4 |
| Refined / path | 276 | 4 | 27 | 31 |
| Legacy / path | 123 | 122 | 92 | 1 |
| Published / time | 269 | 48 | 6 | 15 |
| Refined / time | 297 | 0 | 1 | 40 |
| Legacy / time | 265 | 21 | 50 | 2 |

The [CPU geometry recheck](results/rigor_checks/nto_profiles/geometry/summary.json) reconstructed **all 343 validation-failed outputs**, reevaluated their actual collision constraint locations and independently recomputed the worst saved collision witness. Reconstruction and witness-distance differences were zero. All discrete constraints satisfied their saved solver tolerance, but 329 curves had penetrating witnesses and 14 remained in the contact uncertainty band. Published/path included 139 penetrating curves (up to 22.52 mm) and 9 contact-band cases. Refined/path still had four penetrating curves (0.752–2.456 mm); they remain failures. This check reuses saved curves, without optimizing test problems or changing scores.

Refined/path converts 136 published/path validation failures into valid outputs. Its clearance, adaptive constraint locations and solver limits change together, so the overall improvement cannot be attributed to a single change. Path control counts stay unchanged; the time profile additionally increases spline capacity. Nonconverged candidates count as numerical successes only when the common validator passes them; solver convergence alone never establishes success.

Legacy's 500-iteration limit accounts for 90 of its 92 test/path solver failures and all 50 test/time solver failures. A separate [DEV-only paired diagnostic](results/rigor_checks/nto_profiles/iteration_probe/verification.json) selected the first seed-0 iteration-limit failure per track before running either cap. Only the cap changed:

| Fixed dev problem | 500 iterations | 2,000 iterations |
|---|---|---|
| `dev-bottleneck-000-p02`, path | Iteration limit; invalid candidate | Iteration limit; invalid candidate |
| `dev-dense_clutter-001-p01`, time | Iteration limit; invalid candidate | 60-second timeout |

These four real calls retained identical original frontend paths and passed artifact integrity checks. They show that raising the iteration cap alone did not recover these two failures; they do not estimate a population success rate or provide a controlled timing comparison. Historical test settings, inputs and published CSVs remain unchanged.

**Experimental early return.** The opt-in `nto_refined.early_accept_validated` setting returns an intermediate NTO candidate only after it passes the complete common validator. This prevents later refinement from discarding that candidate. The final validator still runs; both validation costs count toward the online budget. It can stop with sampled clearance below the profile's 0.5 mm refinement guard, so treat this as a separate configuration and inspect clearance alongside success and cost. The default remains disabled.

On all 60 DEV problems at seed 0, historical versus enabled results were 31→31 path successes and 35→36 time successes. One run recorded a native `libdrake.so` SIGSEGV near the deadline; it remains an infrastructure error. Historical and enabled runtimes differ, including failure-classification fixes, so this is observational. A [same-source control](results/rigor_checks/dev_nto_incumbent/control_summary.json) on 12 geometry-selected DEV problems (24 calls) gave 7→7 path and 7→8 time successes. Its seven common successes per track had zero median paired change in objective; this small, nonrandom control does not establish an overall quality or latency improvement. Selection was made after part of the enabled batch had run, without using outcomes. The [full-run summary](results/rigor_checks/dev_nto_incumbent/dev_seed0_summary.json), control and [crash review](results/rigor_checks/dev_nto_incumbent/native_crash_review.json) link to the saved attempts and receipts. No new test score is claimed.

```bash
bash local/python.sh scripts/evaluate.py run \
  --split dev --problem dev-sparse_clutter-000-p00 --methods NTO_REFINED \
  --track time --seeds 0 \
  --config results/rigor_checks/dev_nto_incumbent/early_evaluation.json \
  --output runs/nto-early-example
```

Reproduce the diagnostics with new output paths:

```bash
bash local/python.sh results/rigor_checks/check_nto_profiles.py --output runs/nto-counts.json
bash local/python.sh results/rigor_checks/check_nto_profile_geometry.py \
  --run results/baseline_test --methods NTO NTO_PAPER NTO_REFINED \
  --output runs/nto-geometry
bash local/python.sh results/rigor_checks/probe_nto_iterations.py --output runs/nto-dev-cap
```

## Commands

Each subcommand supports `--help`. Use a new output directory per experiment. `resume` preserves completed failures and does not rerun completed jobs. Commit runtime code changes before preparing new experiments; source-hash mismatches prevent incomplete historical runs from silently continuing with changed code. From another working directory, invoke `local/python.sh` by its absolute path.

### Check and run one problem

```bash
bash local/python.sh scripts/benchmark.py check
bash local/python.sh scripts/evaluate.py check
bash local/python.sh scripts/evaluate.py methods
bash local/python.sh scripts/benchmark.py show smoke-bottleneck-000-p00
bash local/python.sh scripts/evaluate.py run \
  --split smoke --problem smoke-bottleneck-000-p00 \
  --methods EI_ZO --track both --seeds 0 --output runs/quickstart
bash local/python.sh scripts/evaluate.py verify --run runs/quickstart
```

`benchmark.py check` checks input hashes/references, not the full collision audit. `evaluate.py check` executes real GPU collision, solver and positive/negative checks.

### Batch and resume

```bash
# 60 dev problems × 3 paired seeds × 2 path methods.
bash local/python.sh scripts/evaluate.py batch \
  --split dev --methods EI_ZO RRTCONNECT --track path --seeds 0 1 2 \
  --output runs/dev-comparison --prepare-only
bash local/python.sh scripts/evaluate.py resume --run runs/dev-comparison
bash local/python.sh scripts/evaluate.py status --run runs/dev-comparison
bash local/python.sh scripts/evaluate.py verify --run runs/dev-comparison
```

Omit `--methods` for all supported methods; `--track both` uses their supported tracks. Single-problem/smoke runs default to seed 0; dev/test batches default to 0, 1, 2. `resume --max-jobs N` limits a session.

### Summarize and replay

```bash
bash local/python.sh scripts/evaluate.py summarize \
  --selection results/nto_metis/comparison/selection.json --output runs/current-summary
bash local/python.sh scripts/evaluate.py replay \
  --attempt results/nto_metis_test/jobs/test-sparse_clutter-000-p01-NTO_REFINED-minimum_time-0ab9c422/attempts/0001 \
  --output runs/replay-example
```

Replay writes `replay.html`. Use another trajectory's `attempt_path` from `records.csv` to inspect it. For new comparisons, adapt the existing selection schema to reference your run and retain distinct comparison identities. Summaries preserve failures, pending work, stage timings and scene-cluster paired statistics.

For the completed NTO retest, `bash local/python.sh results/nto_metis/complete.py --output results/nto_metis/rebuilt-comparison` additionally enforces the full run matrix, source/configuration agreement, actual solver options and matched historical inputs before publishing. Use a new output path. Add `--coverage-only` to monitor completion without producing scores.

### Generate and build an offline roadmap

```bash
# Frozen dev configuration and generation seed; separate destination required.
bash local/python.sh scripts/benchmark.py generate \
  --split dev --output runs/regenerated-dev
bash local/python.sh scripts/benchmark.py build-roadmap \
  --nodes 6000 --seed 0 --output runs/roadmap-rebuild
```

Generation accepts `--seed` and `--config`. `benchmark.py index` rebuilds the reference index without resampling inputs. Offline construction is outside online query timing.

### Tests and extension

```bash
bash local/python.sh -m pytest -q
bash local/python.sh -m pytest -q --import-mode=prepend \
  csdecomp/tests/csdecomp_test.py csdecomp/tests/drake_dep_test.py
```

The second command checks the installed upstream package with its required import mode. GPU tests need the configured environment. Upstream workflows are inactive examples in [.github/workflow-examples/](../../.github/workflow-examples/); automated GPU CI is not configured.

To add a planner, implement an adapter in [runners/](runners/), declare its identity/objectives in [methods.py](runners/methods.py), and connect [worker dispatch](batch.py). Reuse the frozen inputs, validator and result contracts; add a targeted test and configuration.

## Upstream citation

This Kinova dataset does not reproduce the private Franka/FR3 inputs or KUKA hardware trials. For the original algorithm, cite [Werner et al., RSS 2025](https://www.roboticsproceedings.org/rss21/p045.html):

```bibtex
@inproceedings{WernerP-RSS-25,
  author    = {Peter Werner and Richard Cheng and Tom Stewart and Russ Tedrake and Daniela Rus},
  title     = {Superfast Configuration-Space Convex Set Computation on GPUs for Online Motion Planning},
  booktitle = {Proceedings of Robotics: Science and Systems},
  year      = {2025},
  doi       = {10.15607/RSS.2025.XXI.045}
}
```
