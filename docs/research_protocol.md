# Research protocol

> Historical Forest/FR3 protocol for `research_platform/`. Its budgets and
> result-selection rules apply to those preserved experiments. For the current
> Kinova benchmark, protocol and commands, use the [project README](../README.md).

This document defines the experimental contract implemented by `research_platform/`.
The platform reuses the preserved research implementations in `reproduction/` and
records their numerical variants explicitly. The public Forest and constructed
FR3 profiles support controlled comparisons; they do not establish reproduction
of the authors' original benchmark, unpublished assets, or reported statistics.

## Cases, shared inputs, and acceptance

A case is the Cartesian product of an environment seed, a roadmap seed, and a
roadmap size. The canonical configuration fixes this grid and the method list
before execution. The job graph separates scene generation, offline roadmap
construction, online planning, and each method. A single-case invocation runs
that case and its dependencies; all other declared cases remain pending.

Every method in a case receives the same canonical start, the same goal, and the
same originating DRM path. For FR3, online pose-map IK selects one joint-space
goal in the shared planning job. Methods do not independently rerun IK or choose
an easier goal. Endpoints use the recorded native float32 round-trip
representation; endpoint audits and the original sampling provenance are retained.
A DRM seed that fails independent validation blocks **all** downstream methods,
including NTO. A finite-roadmap planning failure does not prove continuous-space
infeasibility.

The success denominator is every planned method job:

```
strict_success_rate = strict_successes / planned_method_jobs_for_that_method
strict_success = status == "success"
                 and solver_success is true
                 and validation_passed is true
```

Pending, blocked, interrupted, stale, timed-out, solver-failed, validation-failed,
and license-unavailable jobs stay in this denominator. Solver return status alone
is insufficient. Validation checks native and independent collision predicates,
endpoints, bounds, region residuals where applicable, and the trajectory's
applicable derivative constraints. Distance-only paths are not interpreted as
time-feasible trajectories. Validation uses the resolved sampling and tolerance
settings. All Forest representations additionally minimize circle clearance over
each polynomial piece using float64 stationary roots and endpoints. Numerical
conversion/root problems are recorded as unresolved and do not pass. This
continuous-parameter numerical check is **not an interval certificate**. FR3
acceptance remains sampled. A failed finite candidate is saved and may be replayed.

## Objectives and paired comparisons

| Method group | Reported objective | Forest unit | FR3 unit |
| --- | --- | --- | --- |
| DRM, LSCS, LGCS, NTO_distance, NTO_distance_strong | Euclidean configuration-space path length | m | rad |
| SCSTO, NTO_time, NTO_time_strong | Trajectory duration | s | s |

FR3 path length is the norm of joint-coordinate increments, not TCP travel
distance. Polygonal paths and order-two B-splines use their exact polygonal length.
Other curves use the saved validation samples to estimate path length; the result
records `path_length_source`. Time objectives use trajectory duration.

Within a run, a cost pair requires the same case, objective, cost unit, and
nonempty `input_hash`, plus strict success and finite cost for both methods. The
hash binds the shared `input.json`, `seed_path.npz`, and resolved protocol identity.
The report includes the union of planned cases and records exclusions, so a
common-success cost mean cannot be mistaken for an unconditional success rate.
Distance and time objectives are never pooled. A zero denominator cost is omitted
from ratios only; it remains eligible for cost differences.

Reports provide sample counts, means, medians, and sample standard deviations.
They do not infer independent-trial confidence intervals from repeated roadmaps
in the same environment. Comparisons across separate runs additionally require
review of configuration, source, runtime, and sampling identities; an input hash
alone is not a cross-run equivalence certificate.

## Timing and resource budgets

Keep the following scopes separate when reporting performance:

| Record | Scope |
| --- | --- |
| `timing.construction_seconds` | Corridor construction and recovery work timed by the method pipeline, when available |
| `timing.optimization_seconds` | Accumulated solver-call time recorded by the method; summarized as `solver_seconds` |
| `timing.validation_seconds` | Recorded independent validation work, including pipeline iterations when applicable |
| `timing.wall_seconds` | Backend execution time; summarized as `worker_wall_seconds` |
| Attempt receipt `wall_seconds` | Externally measured whole subprocess, including startup and cleanup; preferred for summary `wall_seconds` |
| Scene, roadmap, and plan job results/receipts | Shared preparation, including offline maps, online collision-set construction, FR3 IK, DRM planning, and seed validation |

Component timers do not necessarily sum to whole-job wall time: initialization,
serialization, imports, and other work have distinct boundaries. Missing timing
is null, not zero. A blocked job has no executed method attempt for timing means.
The DRM method job reuses and validates the shared seed; its method timer does
**not** include the online planner. Use the plan job's timer when reporting DRM
planning latency.

For an end-to-end case measurement, state which shared preparation costs are
included and whether offline work is amortized. Count each shared job once at its
declared reuse scope; do not add the same roadmap or plan once per method and
present that as measured total run time. The summary's method wall ratios exclude
shared preparation and are conditional on common validated inputs.

The provided smoke configurations declare a 120 s external watchdog **per job**
and a 30 s internal solver option. These are local experimental budgets, not
paper-specified values. The internal option is forwarded to LGCS and NTO; its
meaning follows the backend, including IPOPT's CPU-time option. LSCS and SCSTO
currently rely on the external watchdog rather than a separate internal 30 s
deadline. Recovery can make multiple solver calls, each with its applicable
internal limit. Thus, 30 s is not a uniform total optimization budget across all
methods. The common enforced boundary is the external job watchdog.

The strengthened NTO methods use `solver_seconds` as a **cumulative inner wall
budget** across program construction, solver calls, independent checks and
refinement. Each new solver call receives the remaining amount. No post-budget
validation is started by the adapter; a late candidate cannot become a success.
This is stricter than the older per-call solver option. All methods still share
the external job watchdog. Do not describe the internal option as an equal
cumulative budget for every method. See [baseline conditions](baseline_conditions.md)
for fixed development grids, internal clearance and refinement semantics.

After the watchdog expires, process-group termination has an additional cleanup
grace of up to 2 s before escalation to SIGKILL. Recorded wall time includes this
cleanup and operating-system overhead; it is not guaranteed to equal the nominal
watchdog exactly. Timeout receipts override a late-written worker success.

## Attempts, resume, and dependency lineage

Attempt directories are append-only. Default resume continues pending,
interrupted, or stale work; completed numerical failures are retained unless an
explicit retry is requested. Retrying a failed job creates a new numbered attempt
and preserves its previous logs, candidates, and receipts. If a dependency is
retried, its consumers become stale and cannot retain accepted status from an
older dependency selection.

Statistics always select the **latest attempt of each planned job**, including a
new failure after an earlier success. They never choose the best attempt and
never average historical retries into algorithm performance. `summary.csv` and
`summary.json` expose `attempt_number`, `attempt_count`, and `history_statuses`.
The history records every attempt's original receipt status and seal state; the
current row also applies dependency validity. Unsealed attempts are marked
interrupted. Resume seals abandoned evidence only after acquiring its execution
lock, then creates another attempt.

Typical run layout:

```text
runs/<run>/
├── manifest.json             # canonical config, complete cases/jobs, source hashes
├── resolved.json             # effective protocol and scene definition
├── environment.json          # numerical runtime fingerprint
├── identity.json             # frozen input-file inventory
├── source_snapshot/          # exact inventoried source and assets
├── jobs/<job-id>/attempts/
│   ├── 0001/
│   │   ├── attempt.json      # job, retry reason, selected dependency receipts
│   │   ├── process.json      # subprocess identity, command, watchdog
│   │   ├── stdout.log
│   │   ├── stderr.log
│   │   ├── result.json
│   │   ├── receipt.json      # sealed file inventory and external process outcome
│   │   └── ...               # scene/map, input, candidate, validation, samples
│   └── 0002/                 # retained separately if retried
└── reports/
    ├── summary.csv           # one row per planned method job
    ├── method_summary.csv
    ├── summary.json
    ├── paired.csv
    ├── summary.html
    └── replay/<case-id>/
```

Artifacts vary by job and outcome. Planning attempts store the shared
`input.json` and `seed_path.npz` when available. Method attempts retain
`candidate.json`, `candidate.npz`, `validation.json`, and
`trajectory_samples.npz` when produced. Absence after failure is recorded rather
than replaced with fabricated arrays.

## Identity, verification, and replay

The canonical configuration hash identifies declared choices; the resolved hash
also covers effective protocol and scene sources. The manifest inventories
source-file hashes and records Git HEAD and dirty state. `identity.json` covers
the manifest, resolved protocol, environment record, and source snapshot.
Attempt receipts inventory output contents, carry their own digest, and bind the
selected dependency receipts. These hashes detect inconsistent or modified
artifacts; they are not signatures or proof of scientific correctness.

The runtime fingerprint includes Python and executable, package versions,
selected numerical binaries and package source files, platform, thread settings,
CUDA visibility, and GPU/driver identity. It is more specific than a package
version list, but it is not a complete operating-system image or a guarantee of
bitwise solver determinism. Resume rejects source inventory/content or runtime
changes. Create a new run for changed code, assets, configuration, or numerical
environment; do not edit an existing manifest or replace its snapshot.

Verification separates three claims:

- `integrity_passed`: the recorded frozen inputs and retained evidence pass their
  content checks.
- `complete`: every planned job is represented, with no unsealed attempts, latest interrupted jobs, or stale
  selected dependencies.
- `all_valid`: completion plus successful preparation jobs and strict acceptance
  of every planned method job.

A complete run can contain validly recorded numerical failures. `verify` checks
integrity and completeness; its successful exit does not imply `all_valid`.
`status` can inspect incomplete runs. Neither command reruns numerical validation
or upgrades a finite-sample check to a mathematical certificate.

Replay reads saved inputs and samples and verifies their receipts. FR3 FK uses
the frozen, hash-matched 33-sphere asset, never a substituted working-tree robot.
Dashed candidates failed acceptance. If downstream methods were blocked, a saved
failed shared seed is labeled as an input, not as a successful method result.
Sampling is reduced for display and NaN-axis collision-witness rows are excluded
from the displayed trajectory; the original samples remain intact. The slider
uses normalized progress along each method's saved axis, not measured execution
or solver time. Distance methods use a path parameter; time methods use seconds.
Reports and replay manifests record the generating `reporting.py` SHA-256.
Derived reports are outside the frozen experiment inputs.

## Operational limits and future experiments

The scheduler serializes jobs and uses a per-run lock plus a GPU lock inside the
current repository checkout. This is not a machine-wide lock: another checkout,
a manually launched process, or an external program can still use the same GPU.
Run `doctor --probe` separately from benchmarks; its GPU and solver probes are not
scheduled under the benchmark lock. Inspect the per-solver results even when the overall doctor check is ready.
A successful tiny solver probe does not establish an unrestricted license for
larger problems.

The coordinator cleans its subprocess group on timeout, ordinary exit, handled
termination, and metadata-write failure. Linux workers also install a parent-death
signal. SIGKILL cannot execute Python cleanup handlers, and process groups are
not a general container boundary for descendants that deliberately create a new
session. After a hard kill, inspect remaining processes and execution locks before
resuming; an active orphan must not be treated as immutable abandoned evidence.

Before a formal comparison, freeze the seed grid, scene profiles and geometry,
roadmap sizes, objective groups, method variants, solver/license availability,
thread settings, internal and external budgets, validation tolerances, recovery
limits, timing accounting, retry policy, and exclusion rules. Treat environment
seeds as the primary sampling units when assessing uncertainty; roadmap
repetitions within one environment are related observations. Distinguish the
fixed Figure 6 scene from randomly generated environments and keep development
cases separate from a declared evaluation set. Predeclare how new baselines
consume the same shared inputs and how configuration changes create new run IDs.
These are requirements for a subsequent registered experiment, not evidence that
such an experiment has already been run.

Implementation references: [configuration](../research_platform/config.py),
[scheduler](../research_platform/runner.py), [receipts](../research_platform/storage.py),
[backend](../research_platform/backend.py), [reporting](../research_platform/reporting.py),
and [runtime diagnostics](../research_platform/diagnostics.py).
