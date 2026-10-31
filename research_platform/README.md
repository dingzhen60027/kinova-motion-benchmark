# Research platform

A reproducible experiment runner for native CSDecomp planning and trajectory-optimization baselines. The platform separates shared task preparation, method execution, independent validation, and reporting. It retains failed attempts and the inputs needed to inspect them.

This package uses the existing Conda environment and the algorithms under `reproduction/`. Historical implementations and recorded experiments remain unchanged. The current environments are public-asset reconstructions suitable for declared research studies; they do not constitute recovery of the authors' private benchmark or hardware experiments.

## Quick start

Run commands from the repository root with the existing environment launcher:

```bash
bash local/python.sh -m research_platform doctor --probe
bash local/python.sh -m research_platform methods
bash local/python.sh -m research_platform config configs/research/forest_smoke.json
bash local/python.sh -m research_platform run \
  --config configs/research/forest_smoke.json \
  --output runs/forest-study-01
```

The output directory must be new. `doctor` checks installed dependencies and GPU availability; `--probe` also executes small collision and solver checks. A small Gurobi solve does not establish that the installed license permits larger LGCS models.

Inspect and replay the run:

```bash
bash local/python.sh -m research_platform status --run runs/forest-study-01
bash local/python.sh -m research_platform verify --run runs/forest-study-01
bash local/python.sh -m research_platform summarize --run runs/forest-study-01
bash local/python.sh -m research_platform replay \
  --run runs/forest-study-01 --case e000_r000_n00200
```

`verify` checks evidence integrity and completion of the declared job grid. A verified, complete run can contain numerical failures. Algorithm success is recorded separately for every method.

Use `run --prepare-only` to freeze the configuration, source snapshot, environment fingerprint, and job graph without executing numerical jobs. `run --case <id>` executes one case and its shared dependencies. The complete declared grid remains in the manifest, so unexecuted cases remain visible.

## Configuration

Start with [forest_smoke.json](../configs/research/forest_smoke.json), [forest_batch.json](../configs/research/forest_batch.json), or [fr3_smoke.json](../configs/research/fr3_smoke.json). Configuration uses strict JSON: unknown keys, duplicate keys, duplicate grid entries, and nonfinite numbers are rejected.

```json
{
  "version": 1,
  "name": "forest-study",
  "environment": "forest",
  "scene_profile": "random",
  "environment_seeds": [0, 1, 2],
  "roadmap_seeds": [0],
  "roadmap_sizes": [200, 400],
  "methods": ["DRM", "LSCS", "LGCS", "SCSTO", "NTO_distance", "NTO_time"],
  "limits": {"job_timeout_seconds": 120, "solver_seconds": 30},
  "planning": {"lazy_edge_step": 0.01, "validation_step": 0.002},
  "scene_options": {"obstacles": 20, "obstacle_radius": 0.3},
  "method_options": {"nto_collision_bound": 0.001}
}
```

This example explicitly changes obstacle density and NTO clearance. It defines a new experiment; it does not change prior runs. Inspect the `config` command's output before execution: `resolved.json` contains the complete effective protocol, scene definition, provenance, and source hashes.

| Profile | Task construction | Supported scene overrides |
|---|---|---|
| Forest `figure6` | Fixed circle centers and endpoints extracted from the author's vector illustration | None; `environment_seeds` must be `[0]` |
| Forest `random` | Seeded uniform circle centers; default endpoints come from the audited illustration | Obstacle count/radius, center bounds, domain bounds, start and goal |
| FR3 `public_table` | Reconstructed 33-sphere FR3, public table dimensions, seeded spherical obstacles, right-to-left pose tasks | Obstacle count/radius/bounds, right/left TCP boxes, table size/center, offline-map extent/center, endpoint sampling limit, IK attempt limit |

Forest domain changes propagate to the native offline map's extent and center. The moving point is represented by a sphere of radius `1e-6 m`. The FR3 profile uses joint-space sampling followed by workspace and collision filtering; its target pose is the forward kinematics of a sampled feasible configuration. This is not uniform sampling over Cartesian position and orientation.

`method_options` currently exposes `nto_collision_bound`, `nto_influence_distance`, `nto_major_iterations`, `bezier_degree`, `scsto_relative_tolerance`, and `maximum_recovery_iterations`. Other numerical choices remain visible in the resolved protocol. Unknown method options are rejected rather than silently ignored.

The default lazy collision step is `0.01`; independent validation uses `0.002`, in meters for Forest or joint-space radians for FR3. SNOPT, IPOPT, and Gurobi receive the configured solver limit. Every job, including LSCS, SCSTO, preparation, and validation, also runs in an isolated subprocess under the external wall-time limit. The solver limit is not a separate Clarabel limit for each SCSTO subproblem.

## Shared inputs and method definitions

The job graph is:

```text
scene(environment seed) ──┐
                         ├─ plan(case) ── method(case, method name)
roadmap(seed, size) ──────┘
```

A case is one environment seed, roadmap seed, and roadmap size. The plan job resolves FR3 online pose-map IK once, computes the native DRM path once, and independently validates that path. All method jobs use the same saved joint-space endpoints, scene, and seed path. They do not repeat IK. If shared preparation fails or the DRM path fails validation, every dependent method is blocked, including NTO.

Each method result carries an `input_hash` binding its resolved protocol, `input.json`, and `seed_path.npz`. Method comparisons require identical input identities. LGCS may construct an additional route under the same task as part of its explicitly defined algorithm.

| Method | Objective | Actual implementation |
|---|---|---|
| `DRM` | Path length | Shared native `DrmPlanner` result, independently rechecked |
| `LSCS` | Path length | EI-ZO corridor and Clarabel SOCP |
| `LGCS` | Path length | Two-route convex-set graph when available; Gurobi perspective MISOCP |
| `SCSTO` | Duration | Author's alternating convex optimizer with Clarabel |
| `NTO_distance` | Path length | Drake KinematicTrajectoryOptimization, order-two B-spline, SNOPT |
| `NTO_time` | Duration | Drake KinematicTrajectoryOptimization, cubic B-spline; SNOPT for Forest, IPOPT for FR3 |
| `NTO_distance_strong` | Path length | Exact segment/circle constraints for Forest; adaptive Drake collision collocation for FR3 |
| `NTO_time_strong` | Duration | Shape-preserving knot insertion, adaptive collision collocation, SNOPT/IPOPT as above |

The platform applies two named numerical repairs: strict Gurobi feasibility/integrality tolerances for LGCS, and preservation of all initial polygonal knots for SCSTO. These use function-local dependency injection and leave the historical implementations and installed modules unchanged. Results record `numerical_variant` and solver metadata. A Gurobi license rejection remains a failure; the platform does not silently substitute another method.

NTO retains pointwise collision constraints. The default time initialization repeats endpoint controls to impose standstill; it does not automatically add control points or select a safety margin based on observed outcomes. Explicit changes to exposed settings become part of the experiment identity.

Select `NTO_distance_strong` and/or `NTO_time_strong` for the strengthened transcriptions. The optional root `strong_nto` object is fully expanded into the canonical configuration: `minimum_control_points=16`, `maximum_control_points=64`, `initial_collocation_points=150`, `maximum_collocation_points=1000`, `maximum_refinements=5`, `maximum_new_points=64`, `clearance=0.001`. Unknown or inconsistent limits are rejected. These settings do not change the old NTO methods. Each `refinements/round_*` stores its warm start, solver settings, candidate and validation; the complete result records the stopping reason. See [baseline conditions](../docs/baseline_conditions.md).

## Validation and interpretation

Solver status and geometric validity are separate. Strict method success requires all three conditions:

```text
status == "success"
solver_success == true
validation_passed == true
```

Validation checks candidate shape and finite values before numerical work. Nonpositive durations, inconsistent spline intervals, invalid region assignments, and candidates requiring more than 200,000 validation points are rejected. The implementation does not silently coarsen the sampling grid to fit the budget.

All Forest polygonal, B-spline and Bezier trajectories additionally receive complete-piece stationary-point clearance checks against circles. The float64 polynomial conversion and roots are checked for numerical problems; unresolved checks fail acceptance. This is not an interval-arithmetic certificate. Newly found collision witnesses also enter geometric recovery. FR3 uses independent Drake queries against the matched sphere/box geometry, alongside native CUDA checks. Time trajectories retain the shared endpoint, region, derivative-control bounds and continuity checks. Finite FR3 collision sampling is not a continuous collision certificate, and matching the reconstructed geometry is not certification against the physical robot or its original nonconvex meshes. A successful optimizer can still produce `validation_failed`.

Distance costs are exact polygonal lengths for DRM, LSCS, LGCS, and order-two NTO. Time-method path lengths are sampled estimates, identified by `path_length_source`; their optimization cost is duration. Forest distance is in meters, FR3 distance is Euclidean joint-space length in radians, and duration is in seconds.

Reporting retains every declared case in the denominator, including blocked, failed, and unexecuted work. Paired cost comparisons use common strictly successful cases, matching input hashes, and the same objective. Shared scene, roadmap, and plan work is reported separately from method execution. Method wall time includes work that is not represented by optimizer time alone; offline preparation should not be silently added to one baseline and omitted from another.

## Evidence, resume, and retries

```text
runs/<run>/
  manifest.json                 # Frozen job graph and source identities
  resolved.json                 # Complete protocol and scene definition
  environment.json              # Runtime, binaries, thread settings and GPU
  source_snapshot/              # Exact source files used by this run
  jobs/<job>/attempts/0001/
    attempt.json                # Selected dependency receipts
    stdout.log, stderr.log      # Actual worker output
    result.json                 # Numerical outcome and metrics, when produced
    receipt.json                # Sealed file hashes and process outcome
    ...                         # Scene, native map, paths or solver evidence
  reports/                      # Derived summaries and saved-candidate replay
```

Plan attempts save `input.json`, `seed_path.npz`, and independent validation. Method attempts save `candidate.json`/`candidate.npz`, `validation.json`, and `trajectory_samples.npz` when a candidate is available. Geometric methods also preserve intermediate regions, optimization inputs, and recovery records. A timeout or early failure may leave a partial artifact set; its logs and receipt remain available.

```bash
# Continue pending, interrupted, or stale work.
bash local/python.sh -m research_platform resume --run runs/forest-study-01

# Explicitly create another attempt for one failed job.
bash local/python.sh -m research_platform resume --run runs/forest-study-01 \
  --retry-job method-e000_r000_n00200-LGCS
```

Normal resume does not retry completed numerical failures. `--retry-failed` requests new attempts for failed jobs; it does not overwrite their earlier attempts. Reports select the latest attempt and expose retry history rather than selecting the best result. Replacing a dependency makes downstream results stale until rerun against the new receipt.

Resume requires unchanged source inventory, frozen inputs, and numerical runtime fingerprint. After changing code, dependencies, GPU identity, or experiment settings, create a new run. Editing manifests or receipts to bypass this check invalidates the experiment.

`summarize` writes case-level `summary.csv`, aggregate `method_summary.csv`, `paired.csv`, `summary.json`, and `summary.html`. `replay` reads saved candidates and snapshot-matching FR3 assets without invoking planners or CUDA. It emits PNG and interactive HTML views plus a replay manifest. Failed candidates are labeled; missing methods remain listed. Replay uses physical seconds only for time trajectories, and a normalized parameter for geometric paths. Auxiliary collision witnesses with `time=NaN` are excluded from playback and retained in the raw arrays.

## Implementation map

| Module | Responsibility |
|---|---|
| [`config.py`](config.py) | Strict configuration, effective scene/protocol resolution, case grid |
| [`runner.py`](runner.py) | Job graph, shared dependencies, sequential GPU scheduling, subprocess watchdog |
| [`storage.py`](storage.py) | Atomic records, locks, immutable attempts, hash receipts and dependency freshness |
| [`backend.py`](backend.py) | Real model construction, native planning, numerical methods and validation |
| [`diagnostics.py`](diagnostics.py) | Dependency checks, runtime fingerprint and executable probes |
| [`reporting.py`](reporting.py) | Receipt-verified summaries, paired comparisons and read-only replay |
| [`__main__.py`](__main__.py) | Public command-line interface |

The scheduler owns `result.json` and process status. A backend job writes only its own attempt's numerical artifacts. Dependency directories are immutable.

## Adding a baseline

Methods are explicitly registered in source; there is no implicit plugin discovery. Implement a candidate-producing function, add an explicit branch in `backend._method`, and retain the common serialization, validation, metrics, and failure handling. Do not regenerate a scene, resolve another IK goal, modify dependency files, or substitute a different seed path inside the method.

For example, a diagnostic straight-line baseline can use the frozen endpoints:

```python
def straight_candidate(environment):
    return {
        "kind": "polygon",
        "method": "STRAIGHT",
        "points": np.vstack([environment.start, environment.goal]),
        "segment_regions": [0],
        "solver_success": True,
        "solver": "analytic_straight_line",
        "solver_status": "candidate_constructed",
    }
```

Add this as a branch before the geometric pipeline branch in `backend._method`:

```python
elif method == "STRAIGHT":
    candidate = straight_candidate(env)
    variant = "straight_line_v1"
```

The existing method tail then saves the candidate and applies independent collision validation. `solver_success=True` means the candidate was constructed; it does not establish collision freedom. This example describes an extension and is not included in the shipped method registry.

Complete the integration in the following places:

1. Add the method name to `config.METHODS` and `backend.METHODS`; add typed, validated options to the configuration schema if needed.
2. Register its objective in `reporting.METHOD_OBJECTIVES`. For a new time method, update backend `TIME_METHODS`, duration metrics, and replay-axis semantics; these currently identify `SCSTO`, `NTO_time` and `NTO_time_strong`.
3. Preserve the method's numerical version, actual solver parameters, candidate arrays, and solver failures. Return the standard fields `status`, `solver_success`, `validation_passed`, `metrics`, and `timing` through the common method path.
4. Include every new implementation or external asset in `runner.sources`. Flat `research_platform/*.py` files are discovered; nested packages and additional assets need explicit source-inventory coverage.
5. Add tests for a known solution, an infeasible or invalid candidate, frozen-input equality, budget handling, and reporting. Use a new run for integration tests; do not patch a sealed run.

The accepted candidate representations are polygonal points, composite Bezier controls with positive durations, and B-spline controls with knots/order/duration. A new representation needs a corresponding preflight validator, numerical validator, serialization contract, and replay adapter before it can enter comparisons.

## Status reference

| Status | Meaning |
|---|---|
| `success` | Job completed; method jobs additionally require solver and validation success |
| `scene_failed`, `ik_failed`, `endpoint_invalid` | Shared task preparation did not yield usable inputs |
| `planning_failed` | Native planner did not produce a usable path; not a proof of continuous-space infeasibility |
| `solver_failed` | A solver reported failure, even if a saved candidate happens to be feasible |
| `validation_failed` | A returned path/candidate failed independent acceptance |
| `license_unavailable` | Solver license prevented execution at the required model size |
| `blocked` | A required shared dependency did not succeed |
| `timeout` | External job wall-time budget expired |
| `worker_failed`, `error` | Worker/process or adapter exception; inspect saved logs and traceback |
| `interrupted` | Work stopped before the attempt was sealed |
| `stale_dependency` | A selected dependency attempt no longer matches the recorded receipt |

Unexecuted jobs remain in the declared grid and are reported as missing work. Run `verify` to distinguish complete evidence collection from partial execution, and inspect method statuses to assess numerical outcomes.

## Tests

```bash
bash local/python.sh -m pytest -q tests/research_platform
```

The test suite covers configuration, evidence integrity, subprocess lifecycle, numerical adapters, and reporting contracts. The backend tests include actual small solver problems; native GPU behavior is exercised separately through `doctor --probe` and the explicitly configured smoke runs. Test success alone does not establish benchmark performance or continuous robot safety.
