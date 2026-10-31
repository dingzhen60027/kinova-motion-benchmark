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


def dependency_probe(out, objective):
    """Solve a real bounded LP using the solver needed by this objective."""
    from pydrake.all import MathematicalProgram
    out = Path(out)
    solver = _solver(objective)
    record = dict(method="NTO_PAPER", objective=objective,
                  solver="SNOPT" if objective == "minimum_distance" else "IPOPT",
                  available=bool(solver.available()), enabled=bool(solver.enabled()),
                  profile=PROFILE["name"],
                  fidelity="Published solver profile; declared Drake reconstruction on Kinova, not original FR3 experiment")
    write(out / "paper_nto_dependencies.json", record)
    if not record["available"] or not record["enabled"]:
        raise ImportError(f"{record['solver']} is unavailable or disabled")
    program = MathematicalProgram()
    x = program.NewContinuousVariables(1)
    program.AddBoundingBoxConstraint(1.0, 2.0, x)
    program.AddLinearCost(x[0])
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


def warm_start(path, objective):
    """Retain every ordered DRM node; only duplicate cubic endpoint controls."""
    from pydrake.all import BsplineBasis, BsplineTrajectory, KnotVectorType
    if objective not in ("minimum_distance", "minimum_time"):
        raise ValueError("Unsupported paper NTO objective")
    original = _numeric_array(path, "Paper NTO DRM path")
    if original.ndim != 2 or original.shape[1] != 7 or len(original) < 2 or not np.isfinite(original).all():
        raise ValueError("Paper NTO requires at least two finite ordered seven-joint DRM nodes")
    timed = objective == "minimum_time"
    controls = np.vstack([original[0], original, original[-1]]) if timed else original.copy()
    order = 4 if timed else 2
    duration = float(np.linalg.norm(np.diff(original, axis=0), axis=1).sum() + 2.0) if timed else 1.0
    if not np.isfinite(duration):
        raise ValueError("Nonfinite paper NTO warm-start duration")
    basis = BsplineBasis(order, len(controls), KnotVectorType.kClampedUniform, 0.0, duration)
    initial = BsplineTrajectory(basis, [q[:, None] for q in controls])
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


def paper_nto(path, model, case, cfg, out, deadline):
    """Return the common runner candidate with independent validation pending.

    The caller owns input/seed validation and the external wall-clock watchdog.
    SNOPT's time limit and IPOPT's CPU-time limit alone do not replace that
    watchdog. Both are restricted to the remaining online budget here.
    """
    from pydrake.all import KinematicTrajectoryOptimization, MinimumDistanceLowerBoundConstraint
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    objective = _objective(case)
    timed = objective == "minimum_time"
    started = time.perf_counter()
    if not np.isfinite(deadline):
        raise ValueError("Paper NTO requires a finite external online deadline")
    if float(cfg["velocity_limit_rad_s"]) != 1.0 or float(cfg["acceleration_limit_rad_s2"]) != 1.0:
        raise ValueError("Paper NTO cannot change the frozen unit velocity/acceleration limits")
    initial, original, controls, order, initial_duration = warm_start(path, objective)
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
        traj.AddDurationConstraint(*PROFILE["time_duration_bounds_seconds"])
        traj.AddVelocityBounds(-np.ones(7), np.ones(7))
        traj.AddAccelerationBounds(-np.ones(7), np.ones(7))
        traj.AddPathVelocityConstraint(np.zeros(7), np.zeros(7), 0.0)
        traj.AddPathVelocityConstraint(np.zeros(7), np.zeros(7), 1.0)
    else:
        traj.AddPathLengthCost()
        traj.AddDurationConstraint(1.0, 1.0)
    constraint = MinimumDistanceLowerBoundConstraint(
        native.plant, PROFILE["clearance_m"], native.plant_context,
        influence_distance_offset=PROFILE["influence_distance_m"])
    locations = np.arange(1, PROFILE["collocation_parameter_Nc"], dtype=float) / PROFILE["collocation_parameter_Nc"]
    for s in locations:
        traj.AddPathPositionConstraint(constraint, float(s))
    remaining = min(PROFILE["maximum_optimizer_seconds"], float(cfg["optimizer_budget_seconds"]),
                    float(deadline) - time.perf_counter())
    solver = _solver(objective)
    name = "IPOPT" if timed else "SNOPT"
    if timed:
        options = dict(max_iter=PROFILE["maximum_iterations"], max_cpu_time=remaining,
                       tol=PROFILE["solver_feasibility_tolerance"], print_level=0,
                       file_print_level=5, output_file=str((out / "ipopt.log").resolve()))
    else:
        options = {"Major iterations limit": PROFILE["maximum_iterations"],
                   "Major feasibility tolerance": PROFILE["solver_feasibility_tolerance"],
                   "Major optimality tolerance": PROFILE["solver_feasibility_tolerance"],
                   "Major print level": 1, "Minor print level": 0,
                   "Time limit": remaining, "Print file": str((out / "snopt.log").resolve())}
    parameters = dict(profile=PROFILE, objective=objective, solver=name, parameters=options,
                      solver_available=bool(solver.available()), solver_enabled=bool(solver.enabled()),
                      collision_pairs=int(native.collision_pair_count),
                      collocation_parameter_Nc=PROFILE["collocation_parameter_Nc"],
                      collocation_points=len(locations), collocation_locations=locations,
                      warm_start_node_count=len(original), warm_start_control_count=len(controls),
                      warm_start_endpoint_copies=2 if timed else 0, warm_start_resampled=False,
                      warm_start_duration_seconds=initial_duration, spline_order=order,
                      collision_evaluator="Drake MinimumDistanceLowerBoundConstraint on canonical model.nto_plant()",
                      legacy_cfg_nto_used=False, construction_seconds=time.perf_counter() - started,
                      watchdog="Caller must enforce external wall-clock budget; IPOPT max_cpu_time is CPU time")
    write(out / "nto_parameters.json", parameters)
    if remaining <= 0:
        raise TimeoutError("Paper NTO construction exhausted online budget")
    if not solver.available() or not solver.enabled():
        raise ImportError(f"Paper NTO {name} is unavailable or disabled")
    for key, value in options.items():
        prog.SetSolverOption(solver.solver_id(), key, value)
    solve_started = time.perf_counter()
    try:
        result = solver.Solve(prog)
    except Exception as exc:
        write(out / "nto_solver_result.json", dict(solver=name, solver_success=False,
              solver_status="exception", error=repr(exc), traceback=traceback.format_exc(),
              solve_seconds=time.perf_counter() - solve_started))
        raise
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
                provenance="published_Franka_NTO_profile_reconstructed_on_frozen_Kinova",
                profile=PROFILE["name"], collocation_parameter_Nc=PROFILE["collocation_parameter_Nc"],
                collocation_points=len(locations), warm_start_node_count=len(original),
                warm_start_endpoint_copies=2 if timed else 0, warm_start_resampled=False)
