# Paper-method baseline study

This study evaluates planning algorithms on fixed Kinova tasks. The target is a
fair baseline for later algorithm research. Recovering the paper's private FR3
assets is not a prerequisite for this track, and Kinova scores are not presented
as the original FR3 scores.

## Methods and questions

| Objective | Paper methods | Measured quantity |
|---|---|---|
| Minimum distance | DRM → EI-ZO → LSCS; up to two DRM routes → EI-ZO → LGCS; DRM → NTO/SNOPT | Sum of joint-space segment lengths, rad |
| Minimum time | DRM → EI-ZO → SCSTO; DRM → NTO/IPOPT | Continuous trajectory duration, s |
| Frontend reference | Validated native DRM PWL seed | Distance only; no claim of a minimum-time trajectory |

These five method/objective combinations remain distinct. The author's native
DRM and EI-ZO are reused. LSCS/SCSTO call the installed author SCSPlanning code.
LGCS and NTO are the declared local mathematical transcriptions documented in
the [method audit](reports/full_baseline/coverage_audit.md), not recovered private
author code. The older `NTO` adapter is separate and cannot silently replace
`NTO_PAPER` in a table.

The experiments answer three questions:

1. How frequently does each complete pipeline return a numerically valid result
   under the same total deadline?
2. Given the same independently validated DRM seed, how do optimization success,
   returned quality, computation and collision recovery differ?
3. How do those results change with roadmap size, obstacle family, endpoint
   distance and whether the straight joint-space connection is blocked?

## Shared conditions

Use the existing Conda environment and the canonical full Kinova collision
model. Scene, endpoints, joint limits, voxel radius, validation tolerances,
velocity/acceleration limits and the initial DRM seed are shared. The two
objectives are never combined into one ranking. All joints have limits of
1 rad/s and 1 rad/s² for the time problem, with stationary endpoints.

The [RSS profile](configs/rss2025_eizo.json) is fixed before test execution.
Each process has a 60-second external deadline, including startup, loading,
planning, optimization and numerical validation. GPU workers are serial.
Offline roadmap construction is reported separately. Every failed job is kept;
there is no successful-only rerun, endpoint replacement or best-seed selection.

An exposed seed does not establish bitwise GPU reproducibility. Every result
therefore also records the actual map, path, arrays, source and input hashes.

## Dataset readiness comes first

The frozen v1 test file contains **420 records: 395 valid inputs and 25 endpoint
generation failures**. The failures occupy all five endpoint slots in five
layouts. These records must remain visible, with their original samples and
rejection counts; they are not 25 failed planning attempts and are not 25 valid
planning problems.

The current user instruction requires checking test samples before running the
formal experiment. Input audit, conservative voxel coverage, independent
collision diagnostics and split/duplicate checks are therefore prerequisites.
No planner is used to decide whether an input is retained. The original v1
manifest is not rewritten. Any later decision to produce 420 *valid* problems
requires a separately named dataset version and a declared input-only sampling
rule, with v1 preserved.

The input audit is recorded in
`reports/baseline_study/test_input_readiness.md`. On 2026-10-28 the user chose
“保留 v1，评测395合法题（推荐）” after reviewing the 395-valid/25-invalid
decision. The approved study therefore retains every original record and
evaluates the 395 legal inputs. The runner requires the separately frozen,
hashed study protocol and complete input audit before any formal execution.

## Metrics and denominators

| Category | Definition and interpretation |
|---|---|
| Primary success | Validated final solutions / all legal requested tasks. Include frontend failures, optimizer failures, timeout, unsupported dependencies and infrastructure failures in this deployment outcome. Show their counts separately. |
| Conditional success | Report both success / native DRM returns and success / independently validated DRM seeds. Neither replaces the primary denominator. |
| Quality | Distance in rad or duration in s. Pair cost differences on the same cases where both methods succeeded; publish the common-success count and selection caveat. |
| Latency | Cold end-to-end p50/p90/p95 and maximum over legal tasks, plus success/failure strata. Invalid-input fast exits never dilute legal-query latency. Publish paired all-legal and common-success differences. |
| Completion curve | Fraction of legal tasks with a final validated output by a displayed time, measured from the fixed 60-second runs. This is not a rerun with another optimizer budget or a first-feasible-iterate measurement. |
| Geometry and motion | Minimum sampled clearance, endpoint residual, joint limits, continuous polynomial derivative bounds where available, and detailed collision/limit/uncertain failures. Finite collision sampling is not a continuous mathematical certificate. |
| Computational structure | Regions, facets, recovery rounds, recorded GPU memory peak and available stage times, each with its observed sample count. Missing stages are never filled with zero. |
| Frontend quality | Recomputed length of the independently validated DRM seed, and the successful distance optimizer's fractional improvement over that seed. DRM is a reference, not an additional independently timed optimizer. |
| Optional scale normalization | Distance divided by the endpoint chord; time divided by an obstacle-free independent-axis rest-to-rest bound. These are descriptive relaxations, not known optimal solutions or tight optimality gaps. |

The original seed reference is taken from the prespecified LSCS call, not the
fastest of five repeated frontend calls. Its recorded stage sum excludes Python
startup and dependency-probe overhead and must not be labelled standalone cold
DRM latency.

LGCS currently has a restricted Gurobi license. A model-size rejection is an
observed support limitation, not evidence that LGCS cannot plan the task. A
secondary paired analysis may exclude dependency/infrastructure-affected tasks
from **both** methods, with the changed denominator and selection bias explicit.
It cannot turn that selected subset into an unconditional algorithm ranking.

## Experimental matrix

The existing development experiment contains 60 fixed inputs × 3 roadmap seeds
× 5 method/objective combinations = 900 observations at 6000 nodes. It is
development evidence, not the formal held-out result.

The specification's formal matrix, after input readiness is resolved, is:

| Block | Inputs | Roadmap sizes | Seeds | Method/objective combinations | Distinct planned jobs |
|---|---:|---|---|---:|---:|
| Main | All 420 frozen records | 6000 | 0, 1, 2 | 5 | 6300 |
| Size effect | 80 IDs fixed without planner results | 3000, 9000, 12000 | 0, 1, 2 | 5 | 3600 additional |
| Size reference | The same 80 IDs | 6000 | 0, 1, 2 | 5 | Reuse the main observations; do not count again |

The 80-ID selection covers all 80 obstacle scene IDs, one endpoint slot
per scene: choose endpoint slot `scene_index % 5` within each family, with
scene indices 0–19. This yields 20 slots per family and the original 40/40/20%
short/medium/long distance-bin proportions, independent of planning outcomes. Invalid selected
records remain invalid; do not choose a different endpoint after seeing the
result. The no-added-obstacle control is reported separately from the four
obstacle families.

With the current frozen v1 records, each method has 1185 legal main queries and
75 invalid-input slots. The obstacle-only main table has 1125 legal queries
from 75 physical scene clusters; open_control has 60 legal queries but only one
physical world. Across both, there are 76 valid physical scene clusters, not
1260 independent environments. Fully invalid scene IDs remain in input-flow
accounting, not algorithm-success denominators.

Roadmap-size groups use the requested node budget. Record the actual retained
node count separately: the native builder retained 2999 nodes for the 3000-node
seed-1 map, and the requested count for the other eleven maps. This small native
pruning difference does not create a fifth size group or justify rebuilding a
seed until its node count or performance is more favorable.

## Statistics and saved evidence

Use a scene-cluster bootstrap with all endpoint and roadmap repetitions kept
together, 2000 replicates and PCG64 seed 45827. A single physical control world
does not receive a scene-generalization confidence interval. Always pair fixed
case, objective, roadmap seed/size and method seed; compare the same model,
input and evaluation protocol. Missing pair members or mixed algorithm/config
versions are errors, not silently reduced intersections.

Keep smoke, development, main test and size-effect outputs separate. Test
performance must not select tolerances, solver parameters, collision radii or
dataset membership. A change after inspecting test results needs a newly
declared protocol and an explicit exploratory label.

For each run retain all requested tasks, immutable attempt receipts, source
snapshots, raw paths, controls, regions, solver output, numerical validation and
process outcomes. Completed failure attempts are not overwritten by resume.
The metrics exporter is
`python -m benchmarks.generic_motion_v1.runners.study_metrics`; its CSV has one
row per real sealed attempt and its JSON includes denominators and selection
rules. It cannot establish completion of an unexecuted preregistered matrix.
