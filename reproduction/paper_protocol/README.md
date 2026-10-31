# Paper-parameter Forest and FR3 reconstruction

This implementation runs the two simulation families from arXiv:2504.10783v1
using the published roadmap sizes, seed-factor counts, EI-ZO parameters, NTO
solvers and collision-constraint counts. It preserves the earlier Forest run
under `reproduction/` and records this protocol in new timestamped runs.

**The original FR3 collision model and random inputs are unavailable.** The
included 33-sphere model is a deterministic fit to pinned official FR3 collision
geometry, with official kinematics, joint limits and SRDF filters. It is a
reconstruction, not the authors' original sphere centers and radii. Its 211 static
candidate pairs plus 640 robot–obstacle pairs differ from the paper's 1187 total.
Matching the sphere count alone does not establish model equivalence.

## Run

Use the existing Conda prefix `.conda/envs/csdecomp` from the repository root.
The launcher isolates Python and library paths and fixes the measured CPU
thread settings. No extra runtime dependencies were needed for this protocol.

```bash
# One complete FR3 case: real pose IK, offline DRM, all five methods and validation.
bash local/python.sh -m reproduction.paper_protocol.run run --environment fr3 --environments 0 --roadmap-seeds 0 --sizes 3000

# Full factorial experiments, 400 requested cases for each environment.
bash local/python.sh -m reproduction.paper_protocol.run run --environment forest
bash local/python.sh -m reproduction.paper_protocol.run run --environment fr3

# Numerical, GPU, kinematic and genuine solver checks. Run outside timing runs.
bash local/python.sh -m pytest -q reproduction/tests
```

A new FR3 run can reuse compatible prepared assets with `--cache-from <FR3_RUN_DIR>`.
The runner checks model/data source hashes, all scene/map settings, dependency
versions and cache file hashes before copying; case results are never reused.

Each run prints `RUN_DIR=...`. Use that exact directory for resume and analysis:

```bash
bash local/python.sh -m reproduction.paper_protocol.run resume --run-dir <RUN_DIR>
bash local/python.sh -m reproduction.paper_protocol.analyze --run-dir <FOREST_RUN_DIR> <FR3_RUN_DIR> --output reproduction/reports/paper_protocol --audit
bash local/python.sh -m reproduction.paper_protocol.throughput --run-dir <FR3_RUN_DIR>
```

Resume checks all computational sources, fitted asset data and numerical
dependency versions. Completed successes and failures are skipped unchanged.
Each resume saves a separate GPU telemetry snapshot. Source/runtime changes
require a fresh run rather than mixing versions.

## Published settings

| Parameter | Forest | FR3 |
|---|---|---|
| Environments × roadmap seeds × sizes | 10 × 10 × 4 | 10 × 10 × 4 |
| Roadmap sizes | 200, 400, 800, 1600 | 3000, 6000, 9000, 12000 |
| Neighbors; task/configuration distance caps | 10; 10 / 10 | 10; 10 / 10 |
| Offline voxel side; lazy edge step | 0.06 m; 0.1 | 0.06 m; 0.1 |
| EI-ZO δ / ε | 0.05 / 0.01 | 0.005 / 0.005 |
| EI-ZO particles / mixing steps / faces per iteration | 1000 / 30 / 10 | 10000 / 60 / 10 |
| EI-ZO configuration margin | 0.01 | 0.01 |
| Distance NTO | SNOPT, Nc=20 | SNOPT, Nc=150 |
| Time NTO | SNOPT, Nc=100 | IPOPT, Nc=150 |

The [configuration](config.json) and frozen computational sources record the
reconstruction inputs and implementation choices, including solver tolerances
and library defaults not specified by the paper. The [parameter audit](../reports/paper_protocol/parameter_audit.csv)
separates matched values, reconstruction assumptions and solver/model differences.

NTO uses the actual Drake `MinimumDistanceLowerBoundConstraint` with a zero
distance bound and the default 0.01 influence offset. Constraints are uniformly
placed, including endpoints; no knot snapping or extra collision margin is
added. Minimum-time B-splines have order four, with their control points warm
started from DRM nodes; arclength resampling enforces a minimum of eight controls. Four controls with
rest endpoints constrain a cubic spline to a straight line, eliminating detours.
The completed v2 Forest run and interrupted v2 FR3 run retain that diagnostic
profile separately; v3 uses eight as an explicit unpublished implementation choice.
The original influence offset, sampling locations and minimum control count were
not published, so these details remain explicit reconstruction assumptions.

LSCS solves the published SOCP with Clarabel because no MOSEK license is
available. LGCS solves an integer SOCP with Gurobi; any restricted-license or
time-limit failure stays in the results. SCSTO calls the pinned author biconvex
implementation. Both minimum-time methods use the same frozen unit velocity and
acceleration bounds. The numerical simulation bounds and Bezier degree were not
published; the chosen values come from the released example/hardware section.

## Inputs, model and verification

The 33-sphere fit uses official FR3 collision meshes at revision
`7aeeddc449edf8d62b594f9e36a81da53e7796f9`. The generated numerical asset,
[fr3_33.json](assets/fr3_33.json), contains source hashes, allocations, fitting
rules, kinematics, limits, filters and mesh-probe coverage measurements. License
and notice files are retained beside it. Regeneration is available through
`python -m reproduction.paper_protocol.build_assets` when the pinned source
files under `reproduction/vendor/franka_description` are present. Fetch and
verify those public files with `bash local/python.sh -m reproduction.fetch_fr3_sources`
before regenerating; the numerical model is already included for ordinary runs.

Each FR3 environment contains 20 fixed obstacle spheres. Exact obstacle radii,
distributions, start configurations and goal poses were not released; the chosen
distributions are frozen in `config.json`. Reachable left-side goal poses are
generated by forward kinematics, then solved using genuine collision-constrained
pose IK from separate initial configurations. Every IK attempt is recorded.
Starts are sampled on the right. The same environment/endpoints are used across
all roadmap seeds, sizes and methods.

Forty static FR3 roadmaps are built once and reused across the ten dynamic
environments. The native API takes voxel half-side, so 0.03 is passed for the
paper's 0.06 m side. LGCS blocks an internal region and invokes the native DRM
again to obtain its second route. No Python surrogate planner replaces the
native DRM in this FR3 protocol.

Forest retains the earlier implementation: circles are static plant geometry,
so no voxel collision map is built. Its initial path is native DRM; its LGCS
second route uses exact NetworkX search over the native roadmap after blocking
an internal region, with exact line-circle edge checks. This implements the
blocking/search idea with a different planner backend; its time is included in
corridor construction and is an additional comparison limitation.

Native and Drake collision filters are explicitly synchronized; the released
bridge otherwise rebuilds default filters. Independent validation checks the
same fitted geometry in Drake, and Forest polygonal paths additionally receive
exact line-circle checks. Curve samples are selected from a continuous speed
bound with maximum requested joint step 0.005 and at least 2001 samples.
Continuous derivative-control-point bounds, endpoints, domain membership and
Bezier C0/C1 continuity are also checked. These are numerical collision checks,
not continuous collision certificates or a proof of sphere-fit volume coverage.

An independently invalid native seed is retained. It is not inflated into a
convex corridor; NTO still receives it as a warm start. Analyses report native
DRM success and independently valid seed success separately, and include every
requested case in the main denominator.

## Saved evidence

`manifest.json` contains the frozen configuration, numerical versions and source
hashes. `source_snapshot/` keeps the exact computational files. FR3 `scenes/`
contains obstacles, endpoints, target poses and IK attempts; `roadmaps/` contains
the actual native maps and sample configurations. Each `cases/<ID>/` contains
the native seed, optimization coefficients, region inequalities, numerical
validation, sampled replay, solver states, timings and failure traces.

Construction and optimization times include failed attempts and any region
recovery. Offline map preparation, warm-up, allocation, independent validation
and file writing are separate; the reported TO+CS time is not an end-to-end
robot-system latency. Cost comparisons use common validated cases, with complete
denominators and separate solver/validation statuses. The 60 s solver limit and
240 s case watchdog are recorded reconstruction limits, not paper parameters.

KUKA hardware experiments remain unavailable without the corresponding system
or recorded sensor/execution data. The current CPU, GPU, driver and numerical
versions also differ from the paper; no exact reproduction of its headline
speedup is asserted.
