# CHOMP and STOMP integration audit

Both adapters execute an optimizer on the frozen seven-joint Kinova model. The STOMP adapter directly invokes the unmodified official C++ core. CHOMP implements covariant descent with a documented C-space obstacle functional, so its label must retain that objective qualification. The geometry used by both adapters is the complete canonical primitive geometry, not a replacement collision-sphere robot.

## Source and method identity

| Method | Executed implementation | Native objective |
|---|---|---|
| CHOMP, canonical-geometry C-space cost | Fixed-endpoint acceleration-metric covariant descent implemented in `runners/chomp_stomp.py` | Squared second differences plus a signed-clearance potential integrated over joint-space arc length |
| STOMP | Official `ros-industrial/stomp`, commit `b1a87c80f7338caae25a5c689b876da15492aa75`; unchanged `src/stomp.cpp`, `src/utils.cpp` and three public headers | Official normalized acceleration control cost plus the task's waypoint signed-clearance potential |

[Zucker et al., CHOMP (IJRR 2013)](https://publications.ri.cmu.edu/chomp-covariant-hamiltonian-optimization-for-motion-planning) gives the covariant update in equations 15/19. The implementation uses that update, a positive-definite fixed-endpoint acceleration metric, fixed endpoints, and backtracking. It does not rename a generic nonlinear optimizer. The original equation 3 integrates workspace body-point paths; the benchmark adapter integrates a canonical-geometry clearance potential in C-space. This is an explicit objective adaptation, not an invocation of the MoveIt CHOMP plugin. Hamiltonian Monte Carlo is not enabled.

[The official STOMP Task interface](https://github.com/ros-industrial/stomp/blob/b1a87c80f7338caae25a5c689b876da15492aa75/include/stomp/task.h) delegates cost, noise and update filtering to application callbacks. Our bridge supplies those callbacks and preserves the upstream probability calculation, rollout combination, control-cost computation, incumbent logic and termination. Correlated noise uses `numpy.random.PCG64` with the saved method seed, inverse-acceleration covariance and exactly zero endpoint noise. The update filter calls the official smoothing matrix, enforces the canonical joint limits and restores the exact fixed endpoints. There is no random wall-clock seeding.

The [official MoveIt STOMP documentation](https://moveit.github.io/moveit_tutorials/doc/stomp_planner/stomp_planner_tutorial.html) describes the same optimizer/task separation and parameters. This integration avoids requiring a ROS installation and does not replace the project's Conda environment.

## Discrete objective and geometry

For signed minimum primitive clearance `d`, the CHOMP potential is `-d + epsilon/2` in penetration, `(d-epsilon)^2/(2 epsilon)` in the influence band, and zero outside it; `epsilon = 0.05 m`. The CHOMP objective is

```
U(q) = 0.5 sum_i ||q[i+2]-2q[i+1]+q[i]||^2
       + 10 sum_i 0.5 (c(d[i])+c(d[i+1])) ||q[i+1]-q[i]||.
```

The update uses the inverse Hessian of the acceleration term on interior knots, scaled by its largest diagonal entry. The discrete obstacle derivative includes both the clearance-potential derivative and arc-length derivative. A regression test compares the complete analytic gradient with an independently computed central difference of the complete objective.

The canonical derivative helper computes exact primitive active-feature gradients using Drake witness points and Jacobians and analytic box/sphere formulas for online voxels. Nonsmooth feature ties use symmetric finite differences. Its actual distance and derivative audit is recorded separately in `trajopt_gradient*`. CHOMP's earlier finite-difference implementation remains evidenced in the first probe; the final adapter uses the shared analytic helper.

STOMP's task state cost is `c(d)/epsilon`; its native control-cost weight is `0.10`. These are **native optimizer objectives**. The common reported quality metric is the Euclidean seven-joint PWL arc length in radians, computed by the existing benchmark validator. Neither adapter claims to directly minimize execution time. An artificial optimizer timestep is not reported as physical trajectory duration.

Both use `Model.clearance` with the original static collision pairs, primitive dimensions and 0.04 m online obstacle spheres. The 0.05 m potential influence band is a cost parameter, not an inflated or shrunken collision model. Optimizer validity checks use 0.05 rad segment sampling and the existing 2 micrometre contact band; final acceptance still requires the existing, denser independent/CUDA validator. Finite sampling is not a continuous-collision certificate.

## Fixed profile and inputs

- Initialization: 64 evenly spaced arc-length positions **plus every original DRM vertex**, preserving every original segment and both endpoints exactly. Maximum 256 knots; exceeding the cap is an explicit error. The actual count and full initialization are saved.
- CHOMP: at most 40 iterations, analytic geometry derivative with 1e-5 rad finite-difference tie policy, maximum per-joint update 0.08 rad, eight backtracking attempts, fixed weights 1/10, relative objective tolerance 1e-7.
- STOMP: at most 40 iterations, 12 fresh rollouts per iteration, no previous-rollout reuse, five additional valid iterations (the upstream stopping rule runs six valid iterations), noise scale 0.10 rad, cost sensitivity 10, control weight 0.10, maximum per-joint update 0.08 rad.
- All work uses the shared wall-clock deadline supplied by the runner. The runner reserves five seconds for validation; callbacks check the deadline during optimization. Deadlines are failures, not successful seed fallbacks.
- The optimizer retains raw inputs, per-iteration costs and latest trajectory; STOMP additionally saves rollout noise and covariance. Successful native returns retain the complete trajectory history, parameters, native objective, PWL cost and result flags. Exceptions save their traceback and geometry-query statistics.

No test tasks were used to choose this profile. The following are CPU preflight probes on existing smoke tasks and archived DRM seeds, not new benchmark scores.

## Actual CPU preflight

| Case | Method | Optimizer time | Output PWL length | Dense CPU check |
|---|---|---:|---:|---|
| smoke-open_control-000-p01 | CHOMP, earlier finite-difference gradient | 4.530 s | 5.957945 rad | 1,288 samples; minimum clearance 15.735 mm |
| smoke-open_control-000-p01 | STOMP | 0.695 s | 5.957935 rad | 1,260 samples; minimum clearance 13.676 mm |
| smoke-bottleneck-000-p00 | CHOMP, final analytic gradient | 7.945 s | 8.257911 rad | 1,747 samples; minimum clearance 18.192 mm |
| smoke-bottleneck-000-p00 | STOMP | 3.395 s | 8.742729 rad | 1,839 samples; minimum clearance 19.025 mm |

The bottleneck input path length was 8.792769 rad. CHOMP completed 37 iterations, reduced its native objective from 0.883455 to 0.830525, and changed its candidate. STOMP also changed its candidate. The open-control STOMP run retained the initial incumbent; this is recorded as `candidate_changed=false`, not disguised as improvement. Both algorithms still executed their optimizer. Probes check the complete path at the standard 0.005 rad resolution with CPU geometry; the main study must additionally perform the ordinary CUDA cross-check and independent result validation.

All 23 targeted regression tests passed, including independent objective-gradient checks, exact seed-polyline preservation, native STOMP execution, deterministic noise replay, actual optimizer updates, invalid input/profile rejection and saved deadline failures.

## Build and failure evidence

The initial build using Ubuntu's system pybind11 headers crashed while converting NumPy 2 arrays into the native solver. `chomp_stomp_tests_failure_system_pybind.log` preserves the actual segmentation-fault test output. An isolated pybind11 2.13.6 header package fixes the bridge compatibility; it is installed under `.local_setup/solver_probes/pybind11_2_13`, **not** into Conda. The existing environment and upstream optimizer were unchanged. The repeated native tests pass and the real canonical probes above ran afterward.

`chomp_stomp_build.json`, `chomp_stomp_dependency_probe.json`, `chomp_stomp_pybind_provenance.json` and `native/vendor/stomp/PROVENANCE.json` record compile commands, hashes, source commit and binary identity. The vendor copy is byte-for-byte equal to the pinned upstream files; Apache-2.0 license text is included. The downloaded CHOMP paper is retained locally under `.local_setup/solver_probes/literature` with its SHA in `chomp_stomp_audit.json`, rather than redistributing the publisher's PDF in this source addition.

## Reproducible commands

Run from the repository root:

```bash
bash local/python.sh -m pip download --no-deps --dest .local_setup/solver_probes/wheels pybind11==2.13.6
bash local/python.sh -m pip install --no-deps --target .local_setup/solver_probes/pybind11_2_13 .local_setup/solver_probes/wheels/pybind11-2.13.6-py3-none-any.whl
bash local/python.sh -m benchmarks.generic_motion_v1.runners.chomp_stomp build
bash local/python.sh -m benchmarks.generic_motion_v1.runners.chomp_stomp probe
bash local/python.sh -m pytest benchmarks/generic_motion_v1/tests/test_chomp_stomp.py -q
bash local/python.sh -m benchmarks.generic_motion_v1.reports.extended_baselines_v1.chomp_stomp_probe --problem smoke-bottleneck-000-p00 --method CHOMP --output-suffix new_check
bash local/python.sh -m benchmarks.generic_motion_v1.reports.extended_baselines_v1.chomp_stomp_probe --problem smoke-bottleneck-000-p00 --method STOMP --output-suffix new_check
```

Probe output directories refuse overwrite. The normal CLI `run`, `batch`, `resume`, `verify`, `summarize` and `replay` integration is handled by the shared extended-method runner. Its new-run root and protocol provide the authoritative complete end-to-end study results; the four preflights above are only dependency, geometry and runtime evidence.
