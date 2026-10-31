# CSDecomp Research Workbench

A reproducible motion-planning workbench built on [CSDecomp](https://github.com/wernerpe/csdecomp). The main suite compares the RSS 2025 paper's DRM/EI-ZO, LSCS, LGCS, SCSTO and nonlinear trajectory optimization on fixed seven-joint Kinova tasks. Each run saves its inputs, solver output, independent numerical validation and replay artifacts.

The repository preserves the upstream CUDA/C++ toolbox, the existing Conda environment, and all earlier experiments. The Kinova suite uses one canonical robot and collision model, four static obstacle families, and separate distance and time objectives. A historical Forest suite is also available.

Start with the [Kinova benchmark guide](benchmarks/generic_motion_v1/README.md)
and the [paper comparator results](benchmarks/generic_motion_v1/reports/benchmark_v1_paper_comparators.md).
The [published-paper audit](benchmarks/generic_motion_v1/reports/paper_comparators_v1/paper_coverage_audit.md)
maps every Table I method to its implementation. The complete paper comparison
uses LSCS, LGCS and NTO for distance, and SCSTO and NTO for time.

The current comparison combines **380 newly registered original-NTO slots**
(376 legal calls and four invalid-input records) with **950 sealed observations**
from the same runtime, inputs, numerical dependencies and budget. The resulting
1,330 records include both original and strengthened NTO, under separate labels.
On all 60 development problems and three fixed roadmaps, the original-paper
method groups give these numerically validated results:

| Objective | Method | Successful queries / 180 |
| --- | --- | ---: |
| Distance | DRM → EI-ZO → LSCS | 110 |
| Distance | DRM → EI-ZO → LGCS (SCIP) | 110 |
| Distance | DRM → NTO (SNOPT, original profile) | 35 |
| Time | DRM → EI-ZO → SCSTO | 110 |
| Time | DRM → NTO (IPOPT, original profile) | 88 |

All five combinations share 110 accepted DRM seeds and 70 frontend failures.
The separately reported strengthened NTO succeeds on **87/180 distance** and
**100/180 time** queries; it does not replace the original NTO rows.
The [strengthening study](benchmarks/generic_motion_v1/reports/benchmark_v1_reliable_baselines.md)
retains its implementation comparisons and original evidence.
The [evaluation protocol](benchmarks/generic_motion_v1/BASELINE_PROTOCOL.md)
defines failure-inclusive success and latency, same-task quality comparisons,
scene-cluster uncertainty and independent numerical validation. Open the
[paper comparator replay gallery](benchmarks/generic_motion_v1/reports/paper_comparators_v1/replays/index.html)
to inspect saved trajectories.

The archived [original-profile formal study](benchmarks/generic_motion_v1/reports/benchmark_v1_formal_baselines.md)
retains its 9,900 registered slots, including 9,300 legal calls, with
[execution evidence](benchmarks/generic_motion_v1/reports/benchmark_v1_formal_completion.md).
The frozen test registry remains **395 legal problems plus 25 generation failures**;
see the [input audit](benchmarks/generic_motion_v1/reports/baseline_study/test_input_readiness.md).
Historical Forest and FR3 investigations are documented separately in the
[coverage audit](benchmarks/generic_motion_v1/reports/full_baseline/coverage_audit.md).


[Quick start](#quick-start) · [Experiment protocol](docs/research_protocol.md) · [Architecture and extensions](research_platform/README.md) · [Baseline acceptance](research_platform/reports/strengthened_baselines/report.md) · [Platform acceptance](research_platform/reports/platform_report.md) · [Upstream and historical guide](README.legacy.md)

## Quick start

The tested platform is Ubuntu 22.04, Conda Python 3.10 and an NVIDIA GPU. Run from the repository root using the existing `.conda/envs/csdecomp` environment.

```bash
bash local/python.sh -m benchmarks.generic_motion_v1 --help
bash local/python.sh -m pytest benchmarks/generic_motion_v1/tests -q

# One fixed smoke problem, both objectives, five method/objective combinations.
bash local/python.sh -m benchmarks.generic_motion_v1 run \
  --manifest benchmarks/generic_motion_v1/manifests/smoke.jsonl \
  --roadmaps benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s000 \
  --config benchmarks/generic_motion_v1/configs/rss2025_reliable.json \
  --methods EI_ZO LGCS NTO_PAPER \
  --problem smoke-bottleneck-000-p00 --output /tmp/kinova_example_run

bash local/python.sh -m benchmarks.generic_motion_v1 verify --run /tmp/kinova_example_run
bash local/python.sh -m benchmarks.generic_motion_v1 summarize --run /tmp/kinova_example_run
```

Use a new output directory for each experiment and `resume` to continue an existing run. The [Kinova guide](benchmarks/generic_motion_v1/README.md) documents generation, configuration, batch execution and replay. This command uses SCIP for LGCS and the original NTO transcription. Add `NTO_REFINED` to run the separately named strengthened variants under the same total budget.

The original environment locks are [conda-explicit.txt](reproduction/conda-explicit.txt) and [requirements.lock.txt](reproduction/requirements.lock.txt). Each benchmark run additionally freezes the actual numerical dependencies it uses. The isolated SCIP dependency and native DRM failure-handling module have their own build receipts; they leave the installed Conda packages intact. See the [development protocol](benchmarks/generic_motion_v1/reports/baseline_repair_v2/development_protocol.json) for exact solver settings and budgets.

## Historical Forest experiments

The separate historical workbench uses `bash local/research.sh`. For example, `bash local/research.sh run --config configs/research/forest_smoke.json --output runs/forest_demo` creates its own saved run. The following configurations and counts belong to this track.

| Configuration | Task | Purpose |
| --- | --- | --- |
| [forest_smoke.json](configs/research/forest_smoke.json) | Reconstructed Figure 6 scene; 200 roadmap nodes | Fast end-to-end validation |
| [fr3_smoke.json](configs/research/fr3_smoke.json) | Public FR3 geometry; constructed tabletop obstacles; 3,000 nodes | Robot pipeline validation |
| [forest_batch.json](configs/research/forest_batch.json) | Three declared random scenes; one shared roadmap | Batch execution and resume |
| [forest_baselines.json](configs/research/forest_baselines.json) | Four fixed development seeds; eight methods | Original and strengthened NTO comparison |
| [fr3_baselines.json](configs/research/fr3_baselines.json) | Two fixed development seeds; eight methods | Same-input robot baseline comparison |
| [forest_baseline_regression.json](configs/research/forest_baseline_regression.json) | Figure 6; eight methods | Known-case regression, separate from development grid |

Each case fixes its scene, endpoints, roadmap and independently validated DRM seed before any method runs. A failed shared plan blocks downstream methods and remains in the requested-case denominator.

| Method | Objective | Implementation |
| --- | --- | --- |
| DRM | Path length | Native dynamic-roadmap path |
| LSCS | Path length | Shortest path through a convex-set sequence |
| LGCS | Path length | Mixed-integer graph-of-convex-sets formulation |
| SCSTO | Duration | Sequential convex optimization with preserved initial knots |
| NTO_distance | Path length | Nonlinear optimization with pointwise collision constraints |
| NTO_time | Duration | Nonlinear time optimization with pointwise collision constraints |
| NTO_distance_strong | Path length | Exact Forest segment constraints; adaptive collision constraints for FR3 |
| NTO_time_strong | Duration | Enriched cubic spline, collision-witness refinement and cumulative budget |

Distance and duration objectives are summarized separately. The NTO formulations can return infeasible trajectories; every candidate is checked independently and its failure remains visible. See the [protocol](docs/research_protocol.md) for units, timing scopes, budgets and success criteria.

The strengthened methods are explicit additional baselines. They retain the same task and DRM warm start; they do not replace the historical paper-style transcriptions. Their bounded refinement settings and the fixed development grid are described in [Baseline conditions](docs/baseline_conditions.md). Forest curves from **every** method receive continuous-parameter polynomial clearance checks, implemented numerically in float64; FR3 validation remains sampled.

The [measured baseline report](research_platform/reports/strengthened_baselines/report.md) records 208 passing platform tests and a fixed six-case development run: distance NTO passed 2/6 cases versus 5/6 for its strengthened variant; time NTO passed 3/6 versus 6/6. A known Figure 6 regression is reported separately. These small acceptance counts are not a general performance ranking; all unsuccessful candidates and solver logs are retained.

## Configure, resume and inspect

```bash
# Validate a configuration and inspect all resolved defaults before running.
bash local/research.sh config configs/research/forest_batch.json

# Freeze the full batch, then execute only one selected case.
bash local/research.sh run --config configs/research/forest_batch.json \
  --output runs/forest_batch --prepare-only
bash local/research.sh resume --run runs/forest_batch --case e000_r000_n00200

# Continue the remaining cases; completed failures are retained by default.
bash local/research.sh resume --run runs/forest_batch
bash local/research.sh status --run runs/forest_batch
bash local/research.sh verify --run runs/forest_batch
bash local/research.sh summarize --run runs/forest_batch

# An explicit retry creates a new attempt and preserves the old evidence.
bash local/research.sh resume --run runs/forest_batch \
  --retry-job method-e000_r000_n00200-NTO_distance
```

Copy a configuration to define new seeds, roadmap sizes, method subsets, budgets or supported `scene_options`. Unknown keys, duplicate seeds, invalid dimensions and non-finite values are rejected. Configuration, source snapshots, numerical binaries and GPU identity are frozen for each run. Changed inputs require a new run; resume rejects incompatible state.

`verify` checks artifact integrity **and completeness of the declared job plan**. Its `all_valid` field separately states whether every method passed numerical validation. A complete experiment may contain solver failures, collisions or timeouts. Reports include every requested case and all attempt histories; paired comparisons require matching inputs and objectives.

## Repository layout

```text
configs/research/          Small, explicit experiment configurations
benchmarks/generic_motion_v1/  Main fixed-input Kinova suite and paper baselines
research_platform/        Configuration, native adapters, execution, evidence and reports
tests/research_platform/   CPU regression tests, real solver and process-lifecycle tests
local/                    Conda setup and consistent Python/CLI launchers
runs/                     Generated immutable experiments (git-ignored)
docs/research_protocol.md Experiment contract and result interpretation
reproduction/             Preserved historical implementations, assets and evidence
csdecomp/                 Upstream CUDA/C++ algorithms and Python bindings
examples/                 Upstream examples
```

New baselines belong in the workbench adapter, with declared objective units, saved candidate trajectories and independent validation. The [extension guide](research_platform/README.md) describes the concrete interfaces. Historical evidence under `reproduction/` stays unchanged.

## Provenance and citation

The underlying algorithms are described in [*Superfast Configuration-Space Convex Set Computation on GPUs for Online Motion Planning*](https://arxiv.org/abs/2504.10783). The [upstream guide](README.legacy.md) retains source-build instructions, examples and the history of this checkout’s reconstruction attempts. The original [MIT license](LICENSE) and attribution are preserved; external models and numerical dependencies retain their respective licenses.

```bibtex
@article{werner2025superfast,
  title={Superfast Configuration-Space Convex Set Computation on GPUs for Online Motion Planning},
  author={Werner, Peter and Cheng, Richard and Stewart, Tom and Tedrake, Russ and Rus, Daniela},
  journal={arXiv preprint arXiv:2504.10783},
  year={2025}
}
```
