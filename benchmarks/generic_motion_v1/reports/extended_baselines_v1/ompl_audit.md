# OMPL extension: implementation and integration evidence

The extension uses the official OMPL native `RRTConnect` and `RRTstar` planners from release 2.0.1, tagged at source commit `c509861210a63ec962bbec72c52823abc16b102e`. The Python adapter provides state and motion collision callbacks on the existing frozen Kinova model; it does not reimplement tree search, nearest-neighbour queries, RRT* rewiring, or native vertex reduction.

The [official installation instructions](https://ompl.kavrakilab.org/installation.html) identify Python wheels as a supported distribution. The wheel was downloaded from official PyPI and installed with `--no-deps --target .local_setup/solver_probes/ompl`. Existing Conda dependencies were not replaced. The initially requested 2.0.2 wheel was unavailable on both the configured mirror and official PyPI; both failed commands remain in the installation logs. Version 2.0.1 is the actual installed version.

`ompl_provenance.json` records the official wheel URL and SHA256, every loaded native library's SHA256, the release source commit, and downloaded source-file hashes. The downloaded BSD-licensed source excerpts and license are under `ompl_sources/`.

## Fixed planning profile

- All seven joints use a bounded `RealVectorStateSpace(7)` in radians, preserving the exact canonical lower/upper bounds. No `SO(2)` wrapping or robot-model changes are introduced.
- Each query initializes OMPL's native RNG once in a fresh process. The requested method seed is retained; zero, which OMPL reserves for random initialization, maps explicitly to one. Production method seeds are recorded by the run controller.
- Native range is 0.5 rad. Native motion checking batches configurations through the same CSD CUDA checker with Euclidean joint-space sample spacing at most 0.01 rad, including both endpoints. Bounds are also checked explicitly.
- Each search has up to 10 seconds, further bounded by the caller's original deadline after reserving 0.5 seconds for simplification and 5 seconds for final validation. RRTConnect stops at its first exact solution; RRT* improves its path-length objective until the declared planning deadline. RRT* goal bias is 0.05, rewiring factor 1.1, and delayed collision checking is enabled.
- Both algorithms receive the same native vertex reduction: at most 100 attempts, 50 empty attempts, full vertex range, and the declared simplification time cap. Raw and simplified paths are separately retained.
- Only native `Exact solution` together with `hasExactSolution()` produces a candidate. An approximate path is saved for diagnosis but cannot become a successful output. Search exhaustion does not establish geometric infeasibility.
- The returned PWL candidate still passes the existing independent 0.005-rad CUDA/Drake numerical validation. Sampling does not establish a mathematical continuous-collision certificate.

RRTConnect is a feasibility search followed by bounded shortening; RRT* optimizes native path length for a finite budget. Neither should be described as producing a certified global optimum. These geometric methods do not implement the minimum-time objective. There is no DRM load, query, or offline roadmap cost for either method; a roadmap/repetition identifier in the comparison table is solely for pairing with the other methods' fixed input and seed.

## Actual integration probes

The two integration probes used the first frozen smoke task, `smoke-sparse_clutter-000-p00`, with its unchanged 213 online spheres and fixed endpoints. A deliberately short two-second search budget was used for integration only, before the ten-second study profile; these records are not score-table observations.

| Native method | Actual outcome | Evidence |
|---|---|---|
| RRTConnect | Passed common numerical validation; length 6.429310697 rad; minimum sampled clearance 0.005696142 m; adapter plus validation 0.948053 s | `ompl_canonical_rrtconnect/` |
| RRT* | Two-second search returned an approximate solution; classified planner timeout, not success | `ompl_canonical_rrtstar/` |

The exact paths, native search graph, approximate path, input case, dependency hashes, candidate and validation arrays, and native logs are saved. RRT*'s failed probe is retained without replacement by an easier input.

Commands:

```bash
bash local/python.sh -m pip download --index-url https://pypi.org/simple --no-deps --only-binary=:all: --dest .local_setup/solver_probes/ompl_wheels ompl==2.0.1
bash local/python.sh -m pip install --no-deps --target .local_setup/solver_probes/ompl .local_setup/solver_probes/ompl_wheels/ompl-2.0.1-*.whl
bash local/python.sh -m pytest -q benchmarks/generic_motion_v1/tests/test_ompl_baselines.py
bash local/python.sh -m benchmarks.generic_motion_v1.reports.extended_baselines_v1.ompl_canonical_probe RRTCONNECT
bash local/python.sh -m benchmarks.generic_motion_v1.reports.extended_baselines_v1.ompl_canonical_probe RRTSTAR
```

The probe script intentionally refuses to overwrite an existing output directory.

## Regression evidence

`ompl_tests.log` records 21 passing tests, including real native solves for both algorithms, an impermeable full-space barrier that cannot be crossed by approximate-result relabelling, fixed-seed RRTConnect tree/path equality across fresh processes, and exact motion-spacing checks. The toy geometries used by these tests are explicitly regression fixtures and are not included as benchmark tasks.

The extended-method analyzer uses the existing sealed-evidence loader and unchanged scene-cluster bootstrap. It requires all six methods on every fixed input and repetition, rejects mixed source/configuration/budget protocols, checks shared DRM outcomes only for LSCS/CHOMP/STOMP/TrajOpt, and marks OMPL's conditional DRM metrics inapplicable. It records each native objective separately from the common evaluated PWL path length. Sixteen accounting tests pass in `ompl_extended_metrics_tests_v2.log`. The initial regression failure is preserved in `ompl_extended_metrics_tests.log`; it exposed a raw-result `drm_success=False` value hidden by the general-purpose summary's independent-frontend normalization. The new analyzer now verifies raw frontend fields directly before accepting evidence.

Additional integration checks were completed before source freeze: `ompl_extended_metrics_tests_v3.log` records 23 passing accounting tests, including the independent sampler's completed-stage timing contract. `ompl_integration_tests.log` records 22 passing preparation/dispatch tests. These check standalone OMPL operation without a roadmap, declared seed-index pairing, rejection of unsupported configuration, preservation of invalid endpoints, and timeout/approximate/native-error classification. They use clearly marked CPU-only mocks and do not contribute experimental successes. Actual robot evidence remains the canonical probes and subsequent sealed study runs.
