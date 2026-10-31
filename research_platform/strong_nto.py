"""Explicitly strengthened NTO transcriptions with bounded adaptive refinement.

These are additional baselines, not recovered settings from the paper. All
rounds share a wall-clock budget and the unchanged, shared DRM endpoints. A
feasible failed-solver iterate is saved but never reported as solver success.
"""
from __future__ import annotations

from pathlib import Path
import time
import traceback

import numpy as np

from research_platform.config import STRONG_NTO_DEFAULTS


DEFAULTS = dict(STRONG_NTO_DEFAULTS)


def _clean(value):
    if isinstance(value, np.ndarray):
        return _clean(value.tolist())
    if isinstance(value, np.generic):
        return _clean(value.item())
    if isinstance(value, float):
        return value if np.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_clean(v) for v in value]
    return value


def _write(path, value):
    from reproduction.common import atomic
    atomic(path, _clean(value))


def _save_candidate(directory, candidate):
    arrays = {k: v for k, v in candidate.items() if isinstance(v, np.ndarray)}
    np.savez_compressed(directory / "candidate.npz", **arrays)
    _write(directory / "candidate.json", {k: v for k, v in candidate.items() if k not in arrays})


def _options(options):
    supplied = {} if options is None else dict(options)
    unknown = set(supplied) - set(DEFAULTS)
    if unknown:
        raise ValueError(f"Unknown strong NTO options: {sorted(unknown)}")
    answer = dict(DEFAULTS, **supplied)
    for key in set(DEFAULTS) - {"clearance"}:
        value = answer[key]
        minimum = 0 if key == "maximum_refinements" else 1
        if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < minimum:
            raise ValueError(f"{key} must be an integer >= {minimum}")
    if answer["minimum_control_points"] < 4 or answer["maximum_control_points"] < answer["minimum_control_points"]:
        raise ValueError("Control limits must satisfy 4 <= minimum <= maximum")
    if answer["initial_collocation_points"] > answer["maximum_collocation_points"]:
        raise ValueError("Initial collocation count exceeds maximum")
    if isinstance(answer["clearance"], bool) or not np.isfinite(answer["clearance"]) or answer["clearance"] < 0:
        raise ValueError("clearance must be finite and nonnegative")
    return answer


def _trajectory(candidate):
    from pydrake.all import BsplineBasis, BsplineTrajectory
    return BsplineTrajectory(BsplineBasis(candidate["spline_order"], candidate["knots"].tolist()),
                             [p[:, None] for p in candidate["controls"]])


def _insert_to_count(trajectory, target):
    """Boehm knot insertion changes available variables, not the warm curve."""
    times = np.linspace(trajectory.start_time(), trajectory.end_time(), 257)
    before = trajectory.vector_values(times)
    inserted = []
    while len(trajectory.control_points()) < target:
        unique = np.unique(trajectory.basis().knots())
        i = int(np.argmax(np.diff(unique)))
        knot = float((unique[i] + unique[i + 1]) / 2)
        trajectory.InsertKnots([knot])
        inserted.append(knot)
    error = float(np.max(np.abs(trajectory.vector_values(times) - before)))
    if error > 1e-10:
        raise RuntimeError("Knot insertion changed the warm-start curve")
    return inserted, error


def _initial(path, minimum_time, minimum_controls):
    from pydrake.all import BsplineBasis, BsplineTrajectory, KnotVectorType
    order = 4 if minimum_time else 2
    controls = np.vstack([path[0], path, path[-1]]) if minimum_time else path.copy()
    duration = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum() + 2) if minimum_time else 1.
    trajectory = BsplineTrajectory(BsplineBasis(order, len(controls), KnotVectorType.kClampedUniform,
                                               0., duration), [q[:, None] for q in controls])
    added, error = _insert_to_count(trajectory, max(len(controls), minimum_controls if minimum_time else 0))
    return trajectory, {"warm_start_inserted_knots": added, "warm_start_shape_max_error": error,
                        "warm_start_endpoint_copies": 2 if minimum_time else 0}


def _endpoint_audit(environment, clearance):
    points = np.asarray([environment.start, environment.goal], dtype=float)
    if environment.kind == "forest":
        centers = np.asarray(environment.base.centers)
        distances = (np.linalg.norm(points[:, None] - centers[None, :], axis=2).min(axis=1)
                     - environment.base.radius) if len(centers) else np.full(2, np.inf)
    else:
        distances = []
        for q in points:
            environment.drake.SetPositions(environment.pc, q)
            query = environment.sg.get_query_output_port().Eval(environment.sg.GetMyContextFromRoot(environment.context))
            pairs = query.ComputeSignedDistancePairwiseClosestPoints(clearance + 1e-6)
            distances.append(min((pair.distance for pair in pairs), default=np.inf))
        distances = np.asarray(distances)
    return {"requested_clearance": clearance, "endpoint_clearances": distances,
            "clearance_feasible_at_fixed_endpoints": bool(np.all(distances >= clearance - 1e-6)),
            "endpoints_displaced": False,
            "null_distance_means": "no pair within query threshold or empty obstacle set"}


def _solver(program, name, config, remaining, directory):
    from pydrake.all import SnoptSolver, IpoptSolver
    if name not in ("SNOPT", "IPOPT"):
        raise ValueError(f"Unsupported NTO solver {name}")
    solver = SnoptSolver() if name == "SNOPT" else IpoptSolver()
    parameters = ({"Major iterations limit": config["nto_major_iterations"],
                   "Major feasibility tolerance": 1e-6, "Major optimality tolerance": 1e-6,
                   "Time limit": remaining, "Print file": str((directory / "snopt.log").resolve())}
                  if name == "SNOPT" else
                  {"max_iter": config["nto_major_iterations"], "max_cpu_time": remaining,
                   "tol": 1e-6, "print_level": 0, "file_print_level": 5,
                   "output_file": str((directory / "ipopt.log").resolve())})
    for key, value in parameters.items():
        program.SetSolverOption(solver.solver_id(), key, value)
    _write(directory / "solver_parameters.json", parameters)
    return solver


def _candidate_from_trajectory(trajectory, minimum_time, **metadata):
    return dict(kind="bspline", method="NTO_time" if minimum_time else "NTO_distance",
                controls=np.asarray([q.ravel() for q in trajectory.control_points()]),
                knots=np.asarray(trajectory.basis().knots()), spline_order=4 if minimum_time else 2,
                duration=float(trajectory.end_time() - trajectory.start_time()),
                control_points=len(trajectory.control_points()), **metadata)


def _solve_round(initial, environment, minimum_time, locations, clearance, deadline, directory):
    """Build and solve a real Drake program; no subprocess or global patching."""
    from pydrake.all import KinematicTrajectoryOptimization, MathematicalProgram, MinimumDistanceLowerBoundConstraint
    from pydrake.autodiffutils import ExtractValue
    cfg = environment.protocol
    exact = environment.kind == "forest" and not minimum_time
    problem = "time" if minimum_time else "distance"
    name = cfg["nto_solvers"][environment.kind][problem]
    started = time.perf_counter()
    controls = np.asarray([q.ravel() for q in initial.control_points()])
    if exact:
        from reproduction.alignment_diagnosis.forest_oracle import (
            length_value_jacobian, distance_value_jacobian, autodiff_result)
        program = MathematicalProgram()
        x = program.NewContinuousVariables(controls.size, "q")
        program.AddBoundingBoxConstraint(np.tile(environment.lower, len(controls)),
                                         np.tile(environment.upper, len(controls)), x)
        program.AddBoundingBoxConstraint(environment.start, environment.start, x[:2])
        program.AddBoundingBoxConstraint(environment.goal, environment.goal, x[-2:])

        def objective(z):
            value, jacobian = length_value_jacobian(ExtractValue(z).ravel())
            return autodiff_result(z, value, jacobian)[0]

        def constraint(z):
            value, jacobian = distance_value_jacobian(ExtractValue(z).ravel(), environment.base.centers)
            return autodiff_result(z, value, jacobian)

        program.AddCost(objective, x)
        n = (len(controls) - 1) * len(environment.base.centers)
        bindings = []
        if n:
            bindings.append(program.AddConstraint(constraint, np.full(n, environment.base.radius + clearance),
                                                   np.full(n, np.inf), x))
        program.SetInitialGuess(x, controls.ravel())
        trajectory = None
    else:
        trajectory = KinematicTrajectoryOptimization(initial)
        program = trajectory.get_mutable_prog()
        trajectory.AddPositionBounds(environment.lower, environment.upper)
        trajectory.AddPathPositionConstraint(environment.start, environment.start, 0.)
        trajectory.AddPathPositionConstraint(environment.goal, environment.goal, 1.)
        if minimum_time:
            trajectory.AddDurationCost(1.)
            trajectory.AddDurationConstraint(.05, 100.)
            trajectory.AddVelocityBounds(-np.ones(environment.dimension) * cfg["velocity_limit"],
                                          np.ones(environment.dimension) * cfg["velocity_limit"])
            trajectory.AddAccelerationBounds(-np.ones(environment.dimension) * cfg["acceleration_limit"],
                                              np.ones(environment.dimension) * cfg["acceleration_limit"])
            trajectory.AddPathVelocityConstraint(np.zeros(environment.dimension), np.zeros(environment.dimension), 0.)
            trajectory.AddPathVelocityConstraint(np.zeros(environment.dimension), np.zeros(environment.dimension), 1.)
        else:
            trajectory.AddPathLengthCost()
            trajectory.AddDurationConstraint(1., 1.)
        constraint = MinimumDistanceLowerBoundConstraint(environment.drake, clearance, environment.pc,
                         influence_distance_offset=cfg["nto_influence_distance"])
        bindings = [trajectory.AddPathPositionConstraint(constraint, float(s)) for s in locations]
    construction = time.perf_counter() - started
    remaining = deadline - time.perf_counter()
    if remaining <= 0:
        raise TimeoutError("Cumulative optimization budget exhausted during construction")
    solver = _solver(program, name, cfg, remaining, directory)
    started = time.perf_counter()
    result = solver.Solve(program)
    seconds = time.perf_counter() - started
    if exact:
        from pydrake.all import BsplineTrajectory
        result_trajectory = BsplineTrajectory(initial.basis(), [q[:, None] for q in result.GetSolution(x).reshape(controls.shape)])
    else:
        result_trajectory = trajectory.ReconstructTrajectory(result)
    details = result.get_solver_details()
    _write(directory / "solver_details.json", {
        key: getattr(details, key) for key in ("info", "status", "xmul", "F", "Fmul", "solve_time") if hasattr(details, key)})
    violations = []
    for binding in bindings:
        values = result.EvalBinding(binding)
        violations.extend(np.maximum(binding.evaluator().lower_bound() - values,
                                     values - binding.evaluator().upper_bound()))
    candidate = _candidate_from_trajectory(result_trajectory, minimum_time,
        solver=name, solver_success=bool(result.is_success()), solver_status=str(result.get_solution_result()),
        objective=float(result.get_optimal_cost()),
        collision_evaluator="exact_segment_circle" if exact else "Drake MinimumDistanceLowerBoundConstraint",
        exact_segment_circle_constraints=n if exact else 0,
        collocation_points=0 if exact else len(locations),
        collocation_locations=[] if exact else locations.tolist(),
        maximum_collision_constraint_violation=float(max([0.] + violations)),
        requested_optimization_clearance=clearance,
        collision_constraint_violations=np.asarray(violations),
        warm_start_controls=controls, warm_start_knots=np.asarray(initial.basis().knots()),
        warm_start_duration=float(initial.end_time() - initial.start_time()),
        solver_budget_remaining_seconds=remaining)
    return candidate, {"construction_seconds": construction, "optimization_seconds": seconds}


def _witnesses(validation, candidate, clearance):
    supplied = list(validation.get("normalized_collision_parameters", []))
    minimum = validation.get("minimum_clearance")
    if minimum is not None and minimum < clearance - 1e-6:
        supplied.extend(validation.get("normalized_minimum_parameters", []))
        supplied.extend(witness["normalized_parameter"] for witness in validation.get("clearance_witnesses", [])
                        if witness.get("clearance", np.inf) < clearance - 1e-6)
    sample = validation.get("_samples", {})
    if all(key in sample for key in ("time", "cuda", "independent")):
        times = np.asarray(sample["time"])
        hit = ~np.asarray(sample["cuda"], bool) | ~np.asarray(sample["independent"], bool)
        start = candidate["knots"][candidate["spline_order"] - 1]
        supplied.extend(((times[hit] - start) / candidate["duration"]).tolist())
    values = np.asarray(supplied, dtype=float)
    return np.unique(values[np.isfinite(values) & (values > 0.) & (values < 1.)])


def _new_locations(existing, witnesses, maximum_new, maximum_total):
    values = np.asarray([s for s in witnesses if not np.any(np.isclose(s, existing, rtol=0., atol=1e-10))])
    count = min(len(values), maximum_new, maximum_total - len(existing))
    if count <= 0:
        return np.empty(0)
    return values[np.unique(np.linspace(0, len(values) - 1, count, dtype=int))]


def solve(path, environment, *, minimum_time, options=None, out, validate_callback):
    """Run strengthened NTO, retaining every iterate and the termination reason.

    ``validate_callback(candidate, environment)`` is the shared independent
    validator. It may additionally return normalized collision/minimum-distance
    parameters; otherwise finite colliding samples supply refinement witnesses.
    Validation remains separate from solver convergence and optimization margin.
    """
    started = time.perf_counter()
    options = _options(options)
    budget = float(environment.protocol["optimizer_time_limit_seconds"])
    if not np.isfinite(budget) or budget <= 0:
        raise ValueError("Optimizer budget must be finite and positive")
    deadline = started + budget
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1] != environment.dimension or len(path) < 2 or not np.isfinite(path).all():
        raise ValueError("Strong NTO requires a finite ordered DRM path")
    if not np.array_equal(path[0], environment.start) or not np.array_equal(path[-1], environment.goal):
        raise ValueError("DRM endpoints disagree with shared environment endpoints")
    out = Path(out)
    rounds = out / "refinements"
    rounds.mkdir(parents=True, exist_ok=True)
    initial, warm_metadata = _initial(path, minimum_time, options["minimum_control_points"])
    candidate = _candidate_from_trajectory(initial, minimum_time, solver_success=False,
                    solver_status="not_solved", warm_start_original_nodes=path.copy(), **warm_metadata)
    validation, history = None, []
    timing = dict(construction_seconds=0., optimization_seconds=0., validation_seconds=0.)
    _write(out / "strong_nto_manifest.json", {"options": options, "total_budget_seconds": budget,
        "budget_scope": "cumulative construction, solve, validation and refinement wall time; external watchdog also applies",
        "warm_start_original_nodes": path, "objective": "time" if minimum_time else "distance",
        "shared_collision_acceptance": "zero penetration; optimization margin is separately reported",
        "provenance": "strengthened baseline, not author settings", **warm_metadata})
    audit = _endpoint_audit(environment, options["clearance"])
    _write(out / "optimization_endpoint_audit.json", audit)
    status = None
    if len(initial.control_points()) > options["maximum_control_points"]:
        status = "control_limit_exceeded"
    elif not audit["clearance_feasible_at_fixed_endpoints"]:
        status = "optimization_clearance_infeasible"
    locations = np.arange(1, options["initial_collocation_points"] + 1, dtype=float) / (options["initial_collocation_points"] + 1)
    for index in range(options["maximum_refinements"] + 1):
        if status is not None:
            break
        if time.perf_counter() >= deadline:
            status = "budget_exhausted"
            break
        directory = rounds / f"round_{index:03d}"
        directory.mkdir()
        record = {"round": index, "status": "started", "elapsed_before_seconds": time.perf_counter() - started,
                  "control_points": len(initial.control_points()),
                  "collocation_points": 0 if environment.kind == "forest" and not minimum_time else len(locations)}
        _write(directory / "round.json", record)
        np.savez_compressed(directory / "warm_start.npz",
            controls=np.asarray([q.ravel() for q in initial.control_points()]),
            knots=np.asarray(initial.basis().knots()), collocation_locations=locations)
        # Never pair a new iterate or an exception with the preceding round's
        # validation: the final candidate/check must identify the same curve.
        validation = None
        try:
            candidate, costs = _solve_round(initial, environment, minimum_time, locations,
                                              options["clearance"], deadline, directory)
            candidate.update(warm_start_original_nodes=path.copy(), strong_nto_round=index, **warm_metadata)
            for key, value in costs.items():
                timing[key] += value
            _save_candidate(directory, candidate)
            record.update(solver_success=candidate["solver_success"], solver_status=candidate["solver_status"], **costs)
            _write(directory / "round.json", record)
            if time.perf_counter() >= deadline:
                raise TimeoutError("Cumulative optimization budget exhausted before validation")
            before = time.perf_counter()
            validation = validate_callback(candidate, environment)
            seconds = time.perf_counter() - before
            timing["validation_seconds"] += seconds
            _write(directory / "validation.json", {k: v for k, v in validation.items() if k != "_samples"})
            if "_samples" in validation:
                np.savez_compressed(directory / "trajectory_samples.npz", **validation["_samples"])
            minimum = validation.get("minimum_clearance")
            margin_passed = minimum is None or minimum >= options["clearance"] - 1e-6
            record.update(validation_seconds=seconds, validation_passed=bool(validation["passed"]),
                          minimum_clearance=minimum, optimization_clearance_check=(
                              "not_available_on_full_curve" if minimum is None else "passed" if margin_passed else "failed"))
            if time.perf_counter() >= deadline:
                status = "budget_exhausted"
            elif candidate["solver_success"] and validation["passed"] and margin_passed:
                status = "success"
            elif index == options["maximum_refinements"]:
                status = "solver_failed" if not candidate["solver_success"] else "refinement_exhausted"
            else:
                witnesses = _witnesses(validation, candidate, options["clearance"])
                additions = _new_locations(locations, witnesses, options["maximum_new_points"], options["maximum_collocation_points"])
                locations = np.sort(np.r_[locations, additions])
                record.update(available_witnesses=len(witnesses), added_collocation_locations=additions)
                # A failed but finite numerical iterate is only a warm start.
                # It never becomes a successful output without a new solve.
                if (np.isfinite(candidate["controls"]).all() and np.isfinite(candidate["knots"]).all()
                        and candidate["duration"] > 0):
                    initial = _trajectory(candidate)
                count = len(initial.control_points())
                target = min(options["maximum_control_points"], count + max(1, count // 2))
                inserted, error = _insert_to_count(initial, target)
                record.update(refinement_inserted_knots=inserted, refinement_shape_max_error=error)
                if not len(additions) and not inserted:
                    status = "refinement_capacity_exhausted"
            record["status"] = status or "refining"
        except Exception as error:
            status = "budget_exhausted" if isinstance(error, TimeoutError) else "numerical_error"
            record.update(status=status, exception=repr(error), traceback=traceback.format_exc())
        record["elapsed_after_seconds"] = time.perf_counter() - started
        history.append(record)
        _write(directory / "round.json", record)
        _write(out / "refinement_history.json", history)
    status = status or "refinement_exhausted"
    candidate.update(strong_nto_termination=status, strong_nto_options=options)
    # A later numerical error must not reuse a successful previous iterate as
    # the final successful baseline result. The actual solver flag is retained.
    candidate["strong_nto_completed"] = status == "success"
    timing["cumulative_seconds"] = time.perf_counter() - started
    answer = dict(candidate=candidate, validation=validation, timing=timing, history=history, status=status,
                  termination_reason=status, budget_seconds=budget, rounds=len(history))
    _save_candidate(out, candidate)
    _write(out / "strong_nto_result.json", {k: v for k, v in answer.items() if k not in ("candidate", "validation")})
    return answer
