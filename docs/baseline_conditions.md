# Baseline conditions

> Historical Forest/FR3 baseline conditions. The 120-second budget below belongs
> to those preserved experiments. For the current Kinova benchmark and its
> evaluation settings, use the [project README](../README.md).

The strengthened NTO methods address observed transcription weaknesses. They
are additional, explicitly configured numerical baselines, not new research
algorithms or recovered author settings. Historical implementations and results
remain under `reproduction/`.

## Methods and fixed problems

| Objective | Preserved comparison | Strengthened baseline |
|---|---|---|
| Configuration-space path length | DRM, LSCS, LGCS, NTO_distance | NTO_distance_strong |
| Duration with velocity/acceleration bounds and stationary endpoints | SCSTO, NTO_time | NTO_time_strong |

All methods receive the same geometry, collision pairs, scene, canonical start,
goal and originating DRM path. FR3 IK is resolved once in shared preparation.
No method changes an endpoint, substitutes a different roadmap seed, or skips
shared-input validation. A failed input blocks all downstream methods and stays
in the denominator. Distances are meters in Forest and Euclidean joint radians
in FR3; durations are seconds. The resolved protocol records the actual limits.

The preserved time NTO repeats endpoint controls to impose standstill. The
strong time NTO starts from precisely that curve and inserts knots to reach at
least 16 controls. Knot insertion preserves the initial curve while giving the
optimizer additional variables and a less restrictive derivative-control
envelope. Inserted knots and measured shape error are saved. The trajectory
family changes explicitly; this is not claimed to be an identical finite
optimization problem. Objective, physical constraints and final acceptance do
remain common.

## Strengthened transcriptions

For Forest distance planning, every segment has an exact segment-to-circle
clearance constraint. Its derivatives reuse the separately checked diagnostic
implementation. For FR3 distance and both time tasks, Drake signed-distance
constraints are initially placed at 150 interior parameters. Validation
witnesses add constraints at the offending parameters; a finite previous
iterate supplies the next warm start. Knot insertion can add freedom during
refinement. Solver failure is not converted into success by returning the DRM
seed or accepting a geometrically feasible failed iterate.

The canonical `strong_nto` configuration declares these bounds before execution:

| Setting | Default |
|---|---:|
| Minimum time-spline controls | 16 |
| Maximum controls | 64 |
| Initial interior collision parameters | 150 |
| Maximum collision parameters | 1,000 |
| Maximum additional constraints per refinement | 64 |
| Maximum refinements after the first solve | 5 |
| Optimization clearance target | 0.001 m |

The clearance is an internal optimization buffer, **not** a new common task
requirement. Every method's shared terminal collision criterion is unchanged.
Strong methods also report whether their own target was met. Fixed endpoints
incompatible with this buffer produce `optimization_clearance_infeasible`;
they are not moved and the margin is not tuned after observing the outcome.
For FR3, a missing minimum full-curve clearance means that quantity was not
computed, not that the complete curve satisfies a 1 mm certificate.

## Shared validation and unresolved results

All methods retain the same native collision queries, independent geometry,
endpoint/bound tolerances and time-derivative acceptance. In Forest, every
polygon, B-spline and Bezier also receives a full-piece numerical minimum
clearance calculation: endpoints and stationary points of squared circle
distance. The checker includes the robot point radius. It records polynomial
conversion and root diagnostics, checks witnesses in the original curve
representation, and rejects unresolved numerical cases. It does not waive
negative clearance using solver feasibility tolerance.

This float64 calculation avoids a fixed grid's between-sample blind spots but
is not an outward-rounded or exact-arithmetic certificate. FR3 still uses finite
sampling against the matched sphere/box model, not a continuous certificate or
physical-robot mesh guarantee. No method receives a weaker final checker.
Forest witnesses enter both NTO refinement and geometric-method recovery.

Time trajectories continue to satisfy the shared derivative-control bounds and
stationary endpoint tests. This release reduces conservative transcriptions by
adding representation freedom; it does not selectively replace one method's
derivative checks with a looser test.

## Budgets, selection and retained evidence

Each configured method job has the same **120 s external process watchdog**.
This includes startup, model setup, optimization, checks and output. Cleanup can
add up to the separately recorded 2 s grace. Shared online planning/IK is recorded
separately; include it consistently when interpreting end-to-end latency.

For strong NTO, the 30 s `solver_seconds` setting is a cumulative inner wall
acceptance budget covering construction, solver calls, validation and refinement.
Every new solver call receives only the remaining budget. A native call can
return late, but the result then remains `budget_exhausted`; the external
watchdog is the hard process boundary. No additional final validation starts
after an internal stop. There is no best-of selection from explicit retries.

The old methods retain their documented per-call solver option; LSCS/SCSTO rely
on the outer watchdog. Thus, 30 s is not a uniform cumulative cap for all methods.
The common enforced comparison boundary is 120 s. Component construction,
solver and validation time are recorded separately from full method wall time.

Each refinement writes its input curve, solver parameters, actual solver status,
candidate, independent check and stopping reason before the next round. Early
stops, failed validation, solver failures and budget exhaustion remain visible.
Strict success requires a completed successful method, successful solver and
successful validation. Artifact verification and numerical success are separate.

## Fixed development acceptance

These configurations are small engineering/diagnostic acceptance runs, not a
formal paper benchmark or a basis for general performance claims:

| Configuration | Environment seeds | Roadmap seed / size | Cases |
|---|---|---|---:|
| `forest_baselines.json` | 10, 11, 12, 13 | 0 / 200 | 4 |
| `fr3_baselines.json` | 10, 11 | 0 / 3,000 | 2 |
| `forest_baseline_regression.json` | Figure 6, fixed ID 0 | 0 / 200 | 1 known regression |

All eight methods are declared in each configuration. The six development cases
use consecutive IDs chosen before their results; old seeds 0-9 were already used
for earlier diagnostics. No failed or easy case is replaced. Figure 6 is a known
case and is reported separately. Lazy-edge resolution is 0.01 and independent
validation resolution is 0.002; full source, inputs and runtime are frozen.

```bash
bash local/research.sh config configs/research/forest_baselines.json
bash local/research.sh run --config configs/research/forest_baselines.json \
  --output runs/my_forest_baselines
bash local/research.sh verify --run runs/my_forest_baselines
bash local/research.sh replay --run runs/my_forest_baselines \
  --case e010_r000_n00200

# Fixed full acceptance; use a new label on every execution.
bash local/python.sh research_platform/reports/strengthened_baselines/acceptance_driver.py \
  --label my_baseline_acceptance
```

Success-only paired costs are secondary diagnostics. Always retain the complete
declared denominator, blocked inputs, numerical failures and actual wall times.
These baselines strengthen the current shared-seed comparison; they do not claim
to cover all state-of-the-art planners, multi-start methods or continuous FR3
collision certification.
