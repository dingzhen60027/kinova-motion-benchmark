"""RSS 2025 Franka NTO settings on the frozen Kinova collision model.

The paper did not release its NTO transcription. This is a declared Drake
reconstruction, separate from the older strengthened ``NTO`` adapter. It uses
the published solvers and spline orders without inserting extra controls.
The unpublished collocation convention and rest-endpoint initialization are
recorded explicitly; these are not evidence of exact author-code reproduction.
"""
from __future__ import annotations

from pathlib import Path
import time
import traceback

import numpy as np

from ..roadmap import write
from .validate import _numeric_array
from .ipopt_options import settings as ipopt_settings, evidence as ipopt_evidence


PROFILE = dict(
    name="rss2025_franka_nto_reconstruction_v1",
    distance_solver="SNOPT",
    time_solver="IPOPT",
    distance_spline_order=2,
    time_spline_order=4,
    collocation_parameter_Nc=150,
    collocation_locations="author_tex_interior_k_over_Nc",
    maximum_iterations=2000,
    clearance_m=0.0,
    influence_distance_m=0.01,
    velocity_limit_rad_s=1.0,
    acceleration_limit_rad_s2=1.0,
    maximum_optimizer_seconds=60.0,
    time_duration_bounds_seconds=[0.05, 100.0],
    solver_feasibility_tolerance=1e-6,
    warm_start_resampled=False,
    minimum_control_points=None,
    sources=dict(
        published="RSS 2025 Werner et al., VII-A.3 and Appendix D; 4th order time spline, SNOPT distance, IPOPT time, Nc=150",
        collocation="Archived author arXiv TeX commented appendix: k/Nc for k=1,...,Nc-1; 149 bindings. Published text says Nc points, so exact implementation is unresolved.",
        endpoint_copies="Reconstruction choice: one extra copy at each endpoint of the cubic warm start, making its endpoint velocities zero while retaining all DRM nodes.",
        engineering="Iteration limit 2000, solver tolerances, initialization duration length+2, positive duration bounds, and 60-second budget are explicit local protocol choices, not published exact settings.",
    ),
)


def _objective(case):
    objective = case.get("_objective")
    if objective not in ("minimum_distance", "minimum_time"):
        raise ValueError("Paper NTO requires a declared minimum_distance or minimum_time objective")
    return objective


def _solver(objective):
    from pydrake.all import IpoptSolver, SnoptSolver
    if objective not in ("minimum_distance", "minimum_time"):
        raise ValueError("Unsupported paper NTO objective")
    return SnoptSolver() if objective == "minimum_distance" else IpoptSolver()


def dependency_probe(out, objective, config=None):
    """Solve a real bounded LP using the solver needed by this objective."""
    from pydrake.all import MathematicalProgram
    out = Path(out)
    solver = _solver(objective)
    record = dict(method="NTO_PAPER", objective=objective,
                  solver="SNOPT" if objective == "minimum_distance" else "IPOPT",
                  available=bool(solver.available()), enabled=bool(solver.enabled()),
                  profile=PROFILE["name"],
                  fidelity="Published solver profile; declared Drake reconstruction on Kinova, not original FR3 experiment")
    if objective == 'minimum_time':
        record['linear_system'] = ipopt_evidence(config)
    write(out / "paper_nto_dependencies.json", record)
    if not record["available"] or not record["enabled"]:
        raise ImportError(f"{record['solver']} is unavailable or disabled")
    program = MathematicalProgram()
    x = program.NewContinuousVariables(1)
    program.AddBoundingBoxConstraint(1.0, 2.0, x)
    program.AddLinearCost(x[0])
    if objective == 'minimum_time':
        for key, value in ipopt_settings(config).items():
            program.SetSolverOption(solver.solver_id(), key, value)
    tic = time.perf_counter()
    result = solver.Solve(program)
    solution = float(result.GetSolution(x)[0])
    record.update(probe_success=bool(result.is_success()),
                  probe_status=str(result.get_solution_result()),
                  probe_solution=solution, probe_seconds=time.perf_counter() - tic)
    write(out / "paper_nto_dependencies.json", record)
    if not record["probe_success"] or not np.isclose(solution, 1.0, atol=1e-6, rtol=0):
        raise RuntimeError(f"{record['solver']} actual dependency probe failed")
    return record


def warm_start(path, objective, *, minimum_time_control_points=None):
    """Construct the published-profile seed, optionally refine its knot basis.

    Knot insertion preserves the entire initial curve and its derivatives.
    The original paper profile leaves this option unset.  The distinct refined
    adapter uses it to avoid a single cubic's unnecessarily loose derivative
    control hull after a DRM path has been reduced to just two nodes.
    """
    from pydrake.all import BsplineBasis, BsplineTrajectory, KnotVectorType
    if objective not in ("minimum_distance", "minimum_time"):
        raise ValueError("Unsupported paper NTO objective")
    original = _numeric_array(path, "Paper NTO DRM path")
    if original.ndim != 2 or original.shape[1] != 7 or len(original) < 2 or not np.isfinite(original).all():
        raise ValueError("Paper NTO requires at least two finite ordered seven-joint DRM nodes")
    timed = objective == "minimum_time"
    if (minimum_time_control_points is not None
            and (isinstance(minimum_time_control_points, (bool, np.bool_))
                 or not isinstance(minimum_time_control_points, (int, np.integer))
                 or minimum_time_control_points < 4)):
        raise ValueError("Minimum time control count must be an integer >= 4")
    controls = np.vstack([original[0], original, original[-1]]) if timed else original.copy()
    order = 4 if timed else 2
    duration = float(np.linalg.norm(np.diff(original, axis=0), axis=1).sum() + 2.0) if timed else 1.0
    if not np.isfinite(duration):
        raise ValueError("Nonfinite paper NTO warm-start duration")
    basis = BsplineBasis(order, len(controls), KnotVectorType.kClampedUniform, 0.0, duration)
    initial = BsplineTrajectory(basis, [q[:, None] for q in controls])
    if timed and minimum_time_control_points is not None:
        while len(initial.control_points()) < minimum_time_control_points:
            unique = np.unique(initial.basis().knots())
            index = int(np.argmax(np.diff(unique)))
            initial.InsertKnots([float((unique[index] + unique[index + 1]) / 2)])
        controls = np.asarray([q.ravel() for q in initial.control_points()])
    return initial, original.copy(), controls, order, duration


def _save_result(result, traj, out):
    """Save even unsuccessful native optimizer values before reconstruction."""
    controls = np.asarray(result.GetSolution(traj.control_points()), dtype=float).T
    duration = float(result.GetSolution(traj.duration()))
    arrays = dict(decision_variables=np.asarray(result.GetSolution(traj.prog().decision_variables())),
                  controls=controls, duration=np.asarray(duration),
                  normalized_knots=np.asarray(traj.basis().knots(), dtype=float))
    details = result.get_solver_details()
    detail_values = {}
    for key in ("info", "solve_time", "status", "F", "Fmul", "xmul", "g", "lambda_val", "z_L", "z_U"):
        if hasattr(details, key):
            value = getattr(details, key)
            if isinstance(value, np.ndarray):
                arrays[f"solver_{key}"] = value
            else:
                detail_values[key] = value
    np.savez_compressed(out / "nto_solver_raw.npz", **arrays)
    status = dict(solver_success=bool(result.is_success()),
                  solver_status=str(result.get_solution_result()),
                  solver_objective=float(result.get_optimal_cost()),
                  native_solver_details=detail_values,
                  returned_controls_finite=bool(np.isfinite(controls).all()),
                  returned_duration_finite=bool(np.isfinite(duration)),
                  raw_solution="nto_solver_raw.npz")
    if hasattr(details, "ConvertStatusToString"):
        status["native_status_description"] = details.ConvertStatusToString()
    write(out / "nto_solver_result.json", status)
    return controls, duration, arrays["normalized_knots"], status


def paper_nto(path, model, case, cfg, out, deadline, *, _profile=None,
              _locations=None, _initial_candidate=None, _collision_recovery_state=None):
    """Return the common runner candidate with independent validation pending.

    The caller owns input/seed validation and the external wall-clock watchdog.
    SNOPT's time limit and IPOPT's CPU-time limit alone do not replace that
    watchdog. Both are restricted to the remaining online budget here.
    """
    from pydrake.all import (CommonSolverOption, KinematicTrajectoryOptimization,
                            SolverOptions)
    from .nto_collision_constraint import RobustMinimumDistanceConstraint, NativeFirstCollisionSolve
    profile = PROFILE if _profile is None else _profile
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    objective = _objective(case)
    timed = objective == "minimum_time"
    started = time.perf_counter()
    if not np.isfinite(deadline):
        raise ValueError("Paper NTO requires a finite external online deadline")
    if float(cfg["velocity_limit_rad_s"]) != 1.0 or float(cfg["acceleration_limit_rad_s2"]) != 1.0:
        raise ValueError("Paper NTO cannot change the frozen unit velocity/acceleration limits")
    initial, original, controls, order, initial_duration = warm_start(
        path, objective,
        minimum_time_control_points=profile.get("minimum_time_control_points"))
    expected_control_count = len(controls)
    if _initial_candidate is not None:
        from pydrake.all import BsplineBasis, BsplineTrajectory
        # Refinement changes only this solve's initial variables. The original
        # DRM path and its count remain saved. Any initial knot refinement has
        # already been applied above; later rounds retain that same basis.
        controls = _numeric_array(_initial_candidate["controls"], "Refinement controls")
        knots = _numeric_array(_initial_candidate["knots"], "Refinement knots")
        if (controls.shape != (expected_control_count, 7)
                or not np.isfinite(controls).all() or not np.isfinite(knots).all()
                or _initial_candidate["spline_order"] != order
                or not np.isfinite(_initial_candidate["duration"])
                or _initial_candidate["duration"] <= 0):
            raise ValueError("Invalid finite refinement warm start")
        initial_duration = float(_initial_candidate["duration"])
        initial = BsplineTrajectory(BsplineBasis(order, knots), [q[:, None] for q in controls])
    endpoints = _numeric_array([case["q_start"], case["q_goal"]], "Paper NTO fixed endpoints")
    if endpoints.shape != (2, 7) or not np.isfinite(endpoints).all():
        raise ValueError("Malformed paper NTO fixed endpoints")
    if np.max(np.abs(original[[0, -1]] - endpoints)) > float(cfg["endpoint_tolerance_rad"]):
        raise ValueError("Paper NTO DRM seed endpoints differ from the fixed task")
    np.savez_compressed(out / "nto_initial.npz", controls=controls,
                        knots=np.asarray(initial.basis().knots()), original_path=original,
                        duration=np.asarray(initial_duration))
    native = model.nto_plant()
    traj = KinematicTrajectoryOptimization(initial)
    prog = traj.get_mutable_prog()
    traj.AddPositionBounds(model.lower, model.upper)
    traj.AddPathPositionConstraint(endpoints[0], endpoints[0], 0.0)
    traj.AddPathPositionConstraint(endpoints[1], endpoints[1], 1.0)
    if timed:
        traj.AddDurationCost(1.0)
        traj.AddDurationConstraint(*profile["time_duration_bounds_seconds"])
        traj.AddVelocityBounds(-np.ones(7), np.ones(7))
        traj.AddAccelerationBounds(-np.ones(7), np.ones(7))
        traj.AddPathVelocityConstraint(np.zeros(7), np.zeros(7), 0.0)
        traj.AddPathVelocityConstraint(np.zeros(7), np.zeros(7), 1.0)
    else:
        traj.AddPathLengthCost()
        traj.AddDurationConstraint(1.0, 1.0)
    collision_backend = RobustMinimumDistanceConstraint(
        native, profile["clearance_m"], profile["influence_distance_m"], out)
    locations = np.arange(1, profile["collocation_parameter_Nc"], dtype=float) / profile["collocation_parameter_Nc"]
    if _locations is not None:
        locations = np.asarray(_locations, dtype=float)
        if (locations.ndim != 1 or not np.isfinite(locations).all()
                or np.any(locations <= 0) or np.any(locations >= 1)
                or np.any(np.diff(locations) <= 0)):
            raise ValueError("Refinement locations must be sorted distinct interior parameters")
    collision_solve = NativeFirstCollisionSolve(traj, collision_backend, locations,
                                               _collision_recovery_state)
    remaining = min(profile["maximum_optimizer_seconds"], float(cfg["optimizer_budget_seconds"]),
                    float(deadline) - time.perf_counter())
    solver = _solver(objective)
    name = "IPOPT" if timed else "SNOPT"
    solve_options = None
    if timed:
        options = dict(max_iter=profile["maximum_iterations"], max_cpu_time=remaining,
                       tol=profile["solver_feasibility_tolerance"], print_level=0,
                       file_print_level=5, output_file=str((out / "ipopt.log").resolve()))
        options.update(ipopt_settings(cfg))
        logging = dict(interface="IPOPT output_file", path=options["output_file"])
        reported_iteration_limits = None
    else:
        options = {"Major iterations limit": profile["maximum_iterations"],
                   "Major feasibility tolerance": profile["solver_feasibility_tolerance"],
                   "Major optimality tolerance": profile["solver_feasibility_tolerance"],
                   "Major print level": 1, "Minor print level": 0,
                   "Time limit": remaining}
        if "snopt_total_iterations" in profile:
            options["Iterations limit"] = profile["snopt_total_iterations"]
            options["Minor iterations limit"] = profile["snopt_minor_iterations"]
        # The direct "Print file" option produces only a header with this
        # Drake/SNOPT build. The common option preserves the full native log.
        # Keep all numerical options on prog unchanged.
        solve_options = SolverOptions()
        log_path = str((out / "snopt.log").resolve())
        solve_options.SetOption(CommonSolverOption.kPrintFileName, log_path)
        logging = dict(interface="SolverOptions/CommonSolverOption.kPrintFileName",
                       path=log_path, numerical_solver_options_changed=False)
        reported_iteration_limits = dict(
            observed_backend="Drake 1.51.1 / SNOPT 7.4-1.2",
            values={"Iterations limit": 10000, "Major iterations limit": 2000,
                    "Minor iterations limit": 500},
            explicitly_configured=["Major iterations limit"],
            reported_defaults_only=["Iterations limit", "Minor iterations limit"],
            defaults_overridden=False,
            evidence="benchmarks/generic_motion_v1/reports/baseline_study/snopt_option_probes_v3/common_file_rows164/snopt.log",
            scope="Previously observed CPU probe values, not a fresh runtime query; total/minor defaults are not overridden")
        if "snopt_total_iterations" in profile:
            reported_iteration_limits.update(
                values={"Iterations limit": profile["snopt_total_iterations"],
                        "Major iterations limit": profile["maximum_iterations"],
                        "Minor iterations limit": profile["snopt_minor_iterations"]},
                explicitly_configured=["Iterations limit", "Major iterations limit", "Minor iterations limit"],
                reported_defaults_only=[], defaults_overridden=True,
                scope="Explicit adapted profile settings; original NTO_PAPER defaults remain unchanged")
    parameters = dict(profile=profile, objective=objective, solver=name, parameters=options,
                      solver_logging=logging, reported_iteration_limits=reported_iteration_limits,
                      solver_available=bool(solver.available()), solver_enabled=bool(solver.enabled()),
                      collision_pairs=int(native.collision_pair_count),
                      collocation_parameter_Nc=profile["collocation_parameter_Nc"],
                      collocation_points=len(locations), collocation_locations=locations,
                      warm_start_node_count=len(original), warm_start_control_count=len(controls),
                      warm_start_endpoint_copies=2 if timed else 0, warm_start_resampled=False,
                      warm_start_inserted_knots=expected_control_count-len(original)-(2 if timed else 0),
                      minimum_time_control_points=profile.get("minimum_time_control_points"),
                      knot_refinement="shape-preserving longest-knot-span bisection" if timed and profile.get("minimum_time_control_points") is not None else None,
                      warm_start_duration_seconds=initial_duration, spline_order=order,
                      collision_evaluator="Drake MinimumDistanceLowerBoundConstraint on canonical model.nto_plant()",
                      collision_backend="native_first_Drake_exact_issue21675_guarded_retry",
                      collision_backend_evidence="nto_collision_backend.json",
                      refinement_warm_start=_initial_candidate is not None,
                      legacy_cfg_nto_used=False, construction_seconds=time.perf_counter() - started,
                      watchdog="Caller must enforce external wall-clock budget; IPOPT max_cpu_time is CPU time")
    if timed:
        parameters['linear_system'] = ipopt_evidence(cfg)
    write(out / "nto_parameters.json", parameters)
    if remaining <= 0:
        raise TimeoutError("Paper NTO construction exhausted online budget")
    if not solver.available() or not solver.enabled():
        raise ImportError(f"Paper NTO {name} is unavailable or disabled")
    for key, value in options.items():
        prog.SetSolverOption(solver.solver_id(), key, value)
    solve_started = time.perf_counter()
    try:
        result = collision_solve.solve(
            solver, min(float(deadline), solve_started + remaining),
            timeout_option="max_cpu_time" if timed else "Time limit",
            log_path=logging["path"], solver_options=solve_options)
    except Exception as exc:
        write(out / "nto_solver_result.json", dict(solver=name, solver_success=False,
              solver_status="exception", error=repr(exc), traceback=traceback.format_exc(),
              solve_seconds=time.perf_counter() - solve_started))
        raise
    finally:
        collision_backend.save()
    optimized_controls, duration, normalized_knots, status = _save_result(result, traj, out)
    status["solve_seconds"] = time.perf_counter() - solve_started
    write(out / "nto_solver_result.json", status)
    # This is exactly KTO's normalized-time reconstruction. Keeping the raw
    # arrays also permits preserving failed/nonfinite candidates without
    # invoking native BsplineBasis constructors on invalid solver values.
    with np.errstate(invalid="ignore", over="ignore"):
        knots = normalized_knots * duration
    return dict(kind="bspline", objective=objective, solver=name,
                solver_success=status["solver_success"], solver_status=status["solver_status"],
                solver_objective=status["solver_objective"], spline_order=order,
                controls=optimized_controls, knots=knots, duration=duration,
                provenance=profile.get("provenance", "published_Franka_NTO_profile_reconstructed_on_frozen_Kinova"),
                profile=profile["name"], collocation_parameter_Nc=profile["collocation_parameter_Nc"],
                collocation_points=len(locations), warm_start_node_count=len(original),
                warm_start_endpoint_copies=2 if timed else 0, warm_start_resampled=False,
                warm_start_inserted_knots=expected_control_count-len(original)-(2 if timed else 0))
