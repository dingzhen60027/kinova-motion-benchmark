# Generic Kinova Motion Planning Benchmark

A reproducible seven-joint planning suite for the DRM, EI-ZO, LSCS, LGCS, SCSTO
and nonlinear trajectory optimization methods studied by Werner et al. at
RSS 2025. The robot is **Kinova Gen3** with a fixed gripper, known static
obstacles, fixed joint-space endpoints and explicit motion limits. Every accepted
trajectory passes the same independent numerical validator.

[Specification](task_spec/BENCHMARK_SPEC.md) · [Measured audit](reports/benchmark_v1_audit.md) ·
[Model audit](reports/model_audit_notes.md) · [Generator configuration](configs/generator.json) ·
[Evaluation configuration](configs/evaluation.json) · [Baseline metrics protocol](BASELINE_PROTOCOL.md) ·
[Paper comparator results](reports/benchmark_v1_paper_comparators.md) ·
[Paper comparator replays](reports/paper_comparators_v1/replays/index.html) ·
[Current baseline results](reports/benchmark_v1_reliable_baselines.md) ·
[Delivery and interpretation](reports/benchmark_v1_reliable_completion.md) ·
[Current replays](reports/baseline_repair_v2/development_replays_v1/index.html) ·
[Original development comparison](reports/benchmark_v1_baseline_metrics.md) ·
[Formal results](reports/benchmark_v1_formal_baselines.md) ·
[Execution and limitations](reports/benchmark_v1_formal_completion.md) ·
[Saved replays](reports/baseline_study/formal_replays_v1/index.html)

The paper baseline has five method/objective combinations: **LSCS, LGCS,
SCSTO, NTO distance and NTO time**. The [full baseline report](reports/benchmark_v1_full_baseline.md)
separates measured Kinova results, the historical Forest reference grid and
unresolved original-paper assets. All methods share native DRM, one canonical robot model, frozen tasks, a seed-path geometry gate,
a 60-second external deadline and the same independent numerical validator.
LSCS, LGCS and SCSTO use EI-ZO regions; NTO optimizes the shared DRM path directly.

The [current paper comparison](reports/benchmark_v1_paper_comparators.md) includes
**380 new original-NTO slots** (376 legal calls and four invalid-input records),
paired with **950 reused sealed slots** whose runtime and dependency hashes match.
Across all 60 dev tasks and three fixed roadmaps, the original-paper groups yield
**110/180 LSCS, 110/180 LGCS, 35/180 NTO distance, 110/180 SCSTO and 88/180 NTO time**.
The 1,330 combined records keep original and strengthened NTO as distinct methods.
See the [paper-method audit](reports/paper_comparators_v1/paper_coverage_audit.md)
for the complete published comparison list and implementation mapping.

The preceding strengthening study is complete: **950 registered combinations**,
including 940 legal planning calls and 10 invalid-input records. On all 60 dev
problems with three fixed roadmaps, successful queries are **110/180 LSCS,
110/180 LGCS, 87/180 refined NTO distance, 110/180 SCSTO and 100/180 refined NTO time**.
Each combination has the same 110 accepted DRM seeds and 70 frontend failures.
The [report](reports/benchmark_v1_reliable_baselines.md) gives full failure
categories, paired costs, cold/warm timing, scene-cluster intervals and the
comparison with the original baseline configuration.

Use the current [RSS 2025 engineering profile](configs/rss2025_reliable.json) with its matching roadmaps. The original [RSS 2025 profile](configs/rss2025_eizo.json) retains the earlier configuration.
The [source audit](reports/paper_algorithm/source_audit.md) distinguishes published
parameters from local implementation choices. The historical example profile,
[earlier single-path results](reports/benchmark_v1_paper_algorithm.md) and
[benchmark qualification](reports/benchmark_v1_qualification.md) remain preserved.
The frozen test registry contains **395 legal problems and 25 endpoint-generation
failures**, not 420 legal problems. The [input readiness audit](reports/baseline_study/test_input_readiness.md)
documents the full checks and the conservative voxel model's limitations. The
approved formal study keeps this v1 dataset unchanged and scores the 395 legal
inputs; input failures stay separately visible.

The formal study is complete: all **9,900 registered slots** are sealed and
independently audited, comprising **9,300 legal planning calls and 600 invalid
input slots** across the main and prespecified roadmap-size studies. This count
includes method and roadmap repetitions; it is not a count of independent
environments. The main and scaling reports reuse 1,200 observations at 6,000
nodes. Successes, failures, unsupported solves and numerical uncertainty remain
in the saved evidence. See the formal results for separate obstacle/control
tables, paired comparisons, scene-cluster intervals and timing denominators.

## Run in the existing environment

Run from the repository root using its existing Conda launcher. No environment
replacement, package reinstall or uv migration is required.

The current engineering profile is
[`rss2025_reliable.json`](configs/rss2025_reliable.json): the native DRM uses two
checked failure returns, LGCS solves the same mixed-integer convex problem with
SCIP. Choose `NTO_PAPER` for the original NTO transcription; `NTO_REFINED` adds a fixed 1 mm clearance and bounded collision-point
refinement to the paper's SNOPT/IPOPT transcription. The original
`rss2025_eizo.json` and `NTO_PAPER` profiles remain available with their archived
results. See the [development protocol](reports/baseline_repair_v2/development_protocol.json),
[DRM repair](reports/baseline_repair_v2/drm_safe_query_audit.md),
[LGCS solver](reports/baseline_strengthening/lgcs_v1/lgcs_scip_adapter_report.md),
and [NTO implementation](reports/baseline_study/nto_refined_implementation.md).

```bash
bash local/python.sh -m benchmarks.generic_motion_v1 --help
bash local/python.sh -m pytest benchmarks/generic_motion_v1/tests -q

# Audit the frozen inputs without running a planner.
bash local/python.sh -m benchmarks.generic_motion_v1 audit \
  --manifest benchmarks/generic_motion_v1/manifests/smoke.jsonl

# Run one fixed problem with both objectives. The output directory must be new.
bash local/python.sh -m benchmarks.generic_motion_v1 run \
  --manifest benchmarks/generic_motion_v1/manifests/smoke.jsonl \
  --roadmaps benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s000 \
  --config benchmarks/generic_motion_v1/configs/rss2025_reliable.json \
  --methods EI_ZO LGCS NTO_PAPER \
  --problem smoke-bottleneck-000-p00 --output /tmp/kinova_example_run

bash local/python.sh -m benchmarks.generic_motion_v1 summarize --run /tmp/kinova_example_run
bash local/python.sh -m benchmarks.generic_motion_v1 verify --run /tmp/kinova_example_run
```

The audit report links actual completed runs and replays. A completed experiment
may contain failures: artifact integrity and numerical success are separate.

## Fixed tasks and models

| Split | Layouts | Problems | Use |
|---|---:|---:|---|
| smoke | 5 | 10 | End-to-end engineering checks, including invalid inputs |
| dev | 20 | 60 | Algorithm development and paired baseline checks |
| test | 84 IDs | 395 legal + 25 invalid | Frozen registry; execution requires the approved study protocol |

The four obstacle families are sparse clutter, dense clutter, workspace
bottlenecks and workspace route choices. The last two describe geometry, not
proven configuration-space topology. `open_control` has no additional obstacles;
its different IDs share one physical base environment and one statistical scene
cluster. No grasp, assembly or task reward is part of a problem.

Both offline and online code construct the **same full Kinova model** through
`model.Model`: seven joints, a fixed gripper, the unchanged table and static
equipment. Robot source files, collision geometry, all collision-filter pairs,
joint order and limits are hashed. Previously built example roadmaps are not
accepted without this identity. The reduced/full upstream assets were compared
numerically; differing file names alone do not establish differing physics.

Scenes use world-frame metres and conservatively cover their original spheres
and oriented boxes with fixed 0.04 m voxel spheres. Endpoints sample all seven
joints with a 0.02 rad limit margin. Scene and endpoint RNG streams are separate
PCG64 streams; the manifests also declare independently derived EI-ZO and
solver seed fields. Current optimizer initialization is deterministic, so the
reserved solver seed is explicitly marked unused.

Sampling rejection counts and invalid inputs remain visible. A planner's success
never selects a scene or endpoint. `feasibility_unknown` does not become
“infeasible” when a roadmap search or optimizer fails.

## Generate and build

```bash
# Use a fresh output root to create an independent reproducibility copy.
bash local/python.sh -m benchmarks.generic_motion_v1 generate \
  --split dev --seed 20261027 --output /tmp/kinova_generated

bash local/python.sh -m benchmarks.generic_motion_v1 build-roadmap \
  --nodes 6000 --seed 0 \
  --config benchmarks/generic_motion_v1/configs/rss2025_eizo.json \
  --output /tmp/kinova_roadmap
```

Optional `--config` accepts a generator JSON for `generate`, or an evaluation
JSON for `build-roadmap`, `run` and `batch`. Changing model geometry, the voxel
radius or the generator distribution requires a new protocol/version and new
G0 audit. Existing manifests, roadmaps and results are never overwritten.

The test generator requires a passing G0/G1/G2 acceptance receipt. Formal
`batch` execution also requires `--study-protocol PATH --study-block NAME`:
the approved protocol must bind the complete input audit, actual authorization,
model/data/roadmap hashes, exact IDs, method/objective combinations, configuration
and budget. `resume` rechecks these bindings. Test generation and input audits
alone do not authorize performance evaluation or tuning.

## Baselines and timing

| Method | Distance objective | Time objective |
|---|---|---|
| `EI_ZO` | Native DRM → native EI-ZO → official LSCS | Native DRM → native EI-ZO → official SCSTO |
| `LGCS` | Native DRM, up to two routes → EI-ZO → perspective GCS MISOCP; explicit Gurobi or SCIP backend | Not a time optimizer |
| `NTO_PAPER` | Shared DRM → order-two Drake KTO/SNOPT | Shared DRM → order-four Drake KTO/IPOPT |
| `NTO_REFINED` | Same SNOPT transcription with fixed clearance and collision-point refinement | Same IPOPT transcription with fixed clearance and collision-point refinement |
| `NTO` | Historical local IPOPT adapter | Historical local adapter with enriched cubic initialization |

Distance is joint-space Euclidean length in radians. Time is duration in seconds
with every joint limited to 1 rad/s and 1 rad/s², with stationary endpoints.
Never rank these two costs together.

The installed SCSPlanning matches the author's published Clarabel implementation.
Relative to the paper's LSCS/MOSEK setup, this is an adapted solver configuration;
the CUDA algorithms are unchanged. `NTO_PAPER` uses the published solver/order
profile, with no extra knot insertion. Its 149 interior collision constraints
follow the author's archived `k/Nc` comment for `Nc=150`; the published count is
ambiguous. Its rest-endpoint control copies and solver options are declared local
choices. `LGCS` implements the paper's mixed-integer graph formulation. Backend
selection is explicit: the original profile uses Gurobi; the current engineering
profile uses SCIP and records the incumbent, dual bound, gap and termination
status. `NTO_REFINED` retains the NTO objectives, solvers and control-point count,
with at most four solves and 600 collision points under the same total deadline.
Every attempt retains actual solver probes, parameters, candidates, logs and
source identities.

The seed-path geometry gate is recorded separately from the native DRM return.
Summaries report both conditional denominators. The current Python EI-ZO API does
not expose statistical-test acceptance versus iteration-cap termination, so a
returned region is not reported as a certified probabilistically safe set.

The external 60 s deadline includes process startup, model construction, map
loading, planning, optimization and validation. Offline DRM construction is
reported separately. `warm_query_seconds` measures the online portion after
initialization within that same execution; it is not a separately repeated warm
solve or a hidden subtraction from cold latency. GPU memory is a sampled process
allocation peak, not an exact allocator maximum.

## Batch, resume and inspect

```bash
bash local/python.sh -m benchmarks.generic_motion_v1 batch \
  --manifest benchmarks/generic_motion_v1/manifests/dev.jsonl \
  --roadmaps benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s000 \
  --config benchmarks/generic_motion_v1/configs/rss2025_reliable.json \
  --methods EI_ZO LGCS NTO_REFINED \
  --output /tmp/kinova_paired --prepare-only

bash local/python.sh -m benchmarks.generic_motion_v1 resume --run /tmp/kinova_paired
bash local/python.sh -m benchmarks.generic_motion_v1 summarize --run /tmp/kinova_paired
bash local/python.sh -m benchmarks.generic_motion_v1 verify --run /tmp/kinova_paired

# Supply an actual saved jobs/<job-id>/attempts/0001 directory.
bash local/python.sh -m benchmarks.generic_motion_v1 replay \
  --attempt /tmp/kinova_paired/jobs/ACTUAL_JOB_ID/attempts/0001 \
  --output /tmp/kinova_replay
```

Resume preserves completed successes and failures and takes an exclusive run lock.
Interrupted attempts retain their partial evidence and retry in a new numbered
directory. Executing pending jobs requires matching source, input and roadmap hashes;
completed historical runs remain verifiable from their frozen source snapshots. Each worker saves stage progress before the
next operation; an external timeout retains partial artifacts and its true
status. Explicit failures include invalid endpoints, front-end failure,
optimization failure, collision/limit failure, numerical uncertainty, timeout
and infrastructure/CUDA errors.

Summary tables keep all declared legal tasks in the main denominator, report
conditional success given DRM success, and separate success/failure timing
quantiles. Confidence intervals resample whole scenes; paired comparisons match
case, roadmap seed, algorithm seed and objective. A successful-cost mean from a
different subset is not a fair method ranking.

The independent validator uses the matched Drake collision geometry and finite
sampling alongside CSD checks. Complete polynomial derivative-control bounds
check time trajectories. A finite collision check is **not a mathematical
continuous collision certificate**. The native Python interface does not expose
the internal pre-shortcut DRM path; that absence is recorded rather than filled
with a fabricated “raw” trajectory.

## Layout

```text
configs/       Canonical robot, generation and evaluation protocols
generator/     Independent scene/endpoint streams, conservative voxels, JSONL audit
model.py       One CSD/Drake model and collision-pair convention
roadmap.py     Native offline DRM builder
runners/       Native baselines, independent validation, aggregation and playback
batch.py       Frozen jobs, external deadlines, receipts and safe resume
manifests/     Fixed smoke/dev/test tasks
scenes/        Original obstacles and actual voxel coordinates
inputs/        Raw generation samples and input-audit arrays
roadmaps/      Native maps, actual sampled nodes and offline metadata
results/       Versioned attempts, candidates, regions, logs and summaries
reports/       Actual commands, model/data audits, measured acceptance report
tests/         Determinism, geometry coverage, collision matching and lifecycle tests
task_spec/     Unmodified imported task documents and archive checksum
```
