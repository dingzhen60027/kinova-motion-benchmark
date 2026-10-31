"""Official OMPL RRTConnect and RRTstar on the frozen seven-joint model.

Only the OMPL state/motion validity interface is supplied here. Tree expansion,
nearest neighbours, rewiring, sampling and path simplification execute in the
official native OMPL library. A returned candidate still requires the shared
independent validator; finite motion sampling is not a continuous certificate.
"""
from __future__ import annotations

import hashlib
import importlib.metadata
from pathlib import Path
import sys
import time
import traceback

import numpy as np

from ..roadmap import ROOT, write


DEPENDENCY_DIR = ROOT / ".local_setup/solver_probes/ompl"
PROFILE = dict(
    name="ompl_2_0_1_kinova7_native_v1",
    methods=["RRTCONNECT", "RRTSTAR"],
    objective="minimum_distance",
    state_space="RealVectorStateSpace(7), radians, bounded, no angular wrapping",
    range_rad=0.5,
    motion_step_rad=0.01,
    planning_seconds=10.0,
    simplification_seconds=0.5,
    simplification_max_steps=100,
    simplification_max_empty_steps=50,
    validation_reserve_seconds=5.0,
    goal_bias=0.05,
    rewire_factor=1.1,
    delay_collision_checking=True,
    exact_solutions_only=True,
    record_validated_progress=False,
    progress_poll_seconds=0.5,
    sources={
        "release": "https://github.com/ompl/ompl/tree/c509861210a63ec962bbec72c52823abc16b102e",
        "RRTCONNECT": "https://github.com/ompl/ompl/blob/2.0.1/src/ompl/geometric/planners/rrt/src/RRTConnect.cpp",
        "RRTSTAR": "https://github.com/ompl/ompl/blob/2.0.1/src/ompl/geometric/planners/rrt/src/RRTstar.cpp",
        "install": "https://ompl.kavrakilab.org/installation.html",
    },
)


class OmplNoExactSolution(RuntimeError):
    """A native completed search without an exact solution, never a fallback."""

    def __init__(self, status):
        self.solver_status = str(status)
        super().__init__(f"OMPL did not return an exact solution: {status}")


def _imports():
    if not DEPENDENCY_DIR.is_dir():
        raise ImportError(f"Missing isolated official OMPL dependency: {DEPENDENCY_DIR}")
    dependency = str(DEPENDENCY_DIR.resolve())
    if dependency not in sys.path:
        sys.path.insert(0, dependency)
    from ompl import base as ob, geometric as og, util as ou
    import ompl
    if not Path(ompl.__file__).resolve().is_relative_to(DEPENDENCY_DIR.resolve()):
        raise ImportError("OMPL was already imported from a different installation")
    return ob, og, ou


def dependency_probe(out=None):
    ob, og, ou = _imports()
    del ob, og, ou
    files = [path for path in DEPENDENCY_DIR.rglob("*")
             if path.is_file() and (".so" in path.name or path.name in ("METADATA", "WHEEL"))]
    result = dict(
        package="ompl", version=importlib.metadata.version("ompl"),
        location=str(DEPENDENCY_DIR.relative_to(ROOT)),
        native_implementation=True,
        environment_modified=False,
        source_commit="c509861210a63ec962bbec72c52823abc16b102e",
        files_sha256={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in sorted(files)},
    )
    if result["version"] != "2.0.1":
        raise RuntimeError("OMPL version differs from the declared native profile")
    if out is not None:
        write(Path(out) / "ompl_dependencies.json", result)
    return result


def settings(cfg):
    supplied = cfg.get("ompl", {})
    if not isinstance(supplied, dict):
        raise ValueError("ompl settings must be an object")
    keys = ("range_rad", "motion_step_rad", "planning_seconds", "simplification_seconds",
            "simplification_max_steps", "simplification_max_empty_steps",
            "validation_reserve_seconds", "goal_bias", "rewire_factor", "record_validated_progress",
            "progress_poll_seconds")
    if set(supplied) - set(keys):
        raise ValueError("Unknown OMPL settings: " + str(sorted(set(supplied) - set(keys))))
    value = {key: supplied.get(key, PROFILE[key]) for key in keys}
    for key, item in value.items():
        if key == "record_validated_progress":
            if not isinstance(item, bool):
                raise ValueError("OMPL record_validated_progress must be boolean")
            continue
        if isinstance(item, bool) or not isinstance(item, (float, int)) or not np.isfinite(item):
            raise ValueError(f"OMPL {key} must be finite numeric")
        if item <= 0:
            raise ValueError(f"OMPL {key} must be positive")
    if value["motion_step_rad"] > 0.01 or value["goal_bias"] >= 1 or value["rewire_factor"] <= 1:
        raise ValueError("OMPL motion step <= .01, goal bias < 1 and rewire factor > 1 required")
    for key in ("simplification_max_steps", "simplification_max_empty_steps"):
        if not isinstance(value[key], int):
            raise ValueError(f"OMPL {key} must be an integer")
    return value


def native_seed(seed):
    """Preserve positive seed identities and give requested zero its own stream.

    OMPL reserves native zero. Reserve UINT32_MAX for requested zero instead
    of aliasing zero and one. The reserved input is rejected explicitly.
    """
    if (isinstance(seed, bool) or not isinstance(seed, (int, np.integer))
            or not 0 <= seed < 2**32-1):
        raise ValueError("OMPL requested seed must be an integer in [0, 2**32-2]")
    return int(seed) if seed else 2**32-1


def motion_samples(a, b, step):
    a, b = np.asarray(a, float), np.asarray(b, float)
    if a.shape != (7,) or b.shape != (7,) or not np.isfinite([a, b]).all():
        raise ValueError("OMPL states must be finite seven-joint vectors")
    if not np.isfinite(step) or not 0 < step <= .01:
        raise ValueError("Motion sampling step must lie in (0, .01] rad")
    count = max(2, int(np.ceil(np.linalg.norm(b - a) / step)) + 1)
    return np.linspace(a, b, count)


def _coordinates(state):
    return np.asarray([state[index] for index in range(7)], float)


def solve(model, case, cfg, out, deadline, seed, method):
    """Run native OMPL without DRM, using one process per seeded query.

    ``deadline`` is the caller's absolute perf_counter deadline. Model loading,
    this call and the final common validator all belong to that same online
    budget. RRTConnect stops at its first exact solution; RRTstar keeps improving
    for the declared planning duration. Both receive the same native, bounded
    vertex-reduction postprocessing and retain the pre-simplification path.
    """
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    parameters = settings(cfg)
    if method not in PROFILE["methods"]:
        raise ValueError(f"Unsupported OMPL method {method}")
    if case.get("_objective", "minimum_distance") != "minimum_distance":
        raise ValueError("Geometric OMPL baselines implement minimum_distance only")
    if not np.isfinite(deadline):
        raise ValueError("OMPL requires a finite shared deadline")
    mapped_seed = native_seed(seed)
    result = dict(profile=PROFILE, settings=parameters, method=method,
                  requested_seed=int(seed), native_seed=mapped_seed,
                  seed_mapping="positive identities preserved; requested zero maps to reserved UINT32_MAX",
                  random_generator="OMPL native RNG, one fresh process per query",
                  robot_sha256=model.robot_sha256,
                  collision_geometry_sha256=model.collision_geometry_sha256,
                  input_frontend="independent native sampling, no DRM initialization",
                  solver_success=False, status="initializing", final_validation_required=True)
    counters = dict(state_calls=0, motion_calls=0, collision_configurations=0,
                    free_motion_calls=0, invalid_motion_calls=0,
                    deadline_rejected_calls=0, bounds_rejected_calls=0)
    result["collision_counters"] = counters
    write(out / "ompl_parameters.json", result)
    planner = si = None
    graph_saved = False
    try:
        result["dependency"] = dependency_probe(out)
        ob, og, ou = _imports()
        ou.RNG.setSeed(mapped_seed)
        space = ob.RealVectorStateSpace(7)
        bounds = ob.RealVectorBounds(7)
        lower, upper = np.asarray(model.lower, float), np.asarray(model.upper, float)
        if lower.shape != (7,) or upper.shape != (7,) or not np.isfinite([lower, upper]).all() or np.any(lower >= upper):
            raise ValueError("Canonical model must expose finite increasing bounds for all seven joints")
        for index in range(7):
            bounds.setLow(index, float(lower[index]))
            bounds.setHigh(index, float(upper[index]))
        space.setBounds(bounds)
        # A native fallback resolution is also set, although custom batched
        # motion validation supplies the precise Euclidean-radian resolution.
        space.setLongestValidSegmentFraction(parameters["motion_step_rad"] / float(np.linalg.norm(upper-lower)))
        si = ob.SpaceInformation(space)
        planning_deadline = min(started + parameters["planning_seconds"],
                                float(deadline) - parameters["validation_reserve_seconds"] - parameters["simplification_seconds"])
        if planning_deadline <= time.perf_counter():
            raise TimeoutError("No online time remains after reserving common validation")
        active_deadline = [planning_deadline]

        def collision_free(q):
            if time.perf_counter() >= active_deadline[0]:
                counters["deadline_rejected_calls"] += 1
                return False
            if not np.isfinite(q).all() or np.any(q < lower) or np.any(q > upper):
                counters["bounds_rejected_calls"] += 1
                return False
            counters["collision_configurations"] += len(q)
            mask = np.asarray(model.cuda(q), bool)
            if mask.shape != (len(q),):
                raise ValueError("Canonical collision checker returned an unexpected shape")
            return bool(mask.all())

        def valid_state(state):
            counters["state_calls"] += 1
            return collision_free(_coordinates(state)[None, :])

        class BatchedMotionValidator(ob.MotionValidator):
            def __init__(self, information):
                super().__init__(information)

            def checkMotion(self, a, b):
                counters["motion_calls"] += 1
                valid = collision_free(motion_samples(_coordinates(a), _coordinates(b), parameters["motion_step_rad"]))
                counters["free_motion_calls" if valid else "invalid_motion_calls"] += 1
                return valid

        validator = BatchedMotionValidator(si)
        si.setStateValidityChecker(valid_state)
        si.setMotionValidator(validator)
        si.setup()
        start, goal = space.allocState(), space.allocState()
        q_start, q_goal = np.asarray(case["q_start"], float), np.asarray(case["q_goal"], float)
        if q_start.shape != (7,) or q_goal.shape != (7,) or not np.isfinite([q_start, q_goal]).all():
            raise ValueError("Endpoints must be finite seven-joint vectors")
        for index in range(7):
            start[index], goal[index] = float(q_start[index]), float(q_goal[index])
        problem = ob.ProblemDefinition(si)
        problem.setStartAndGoalStates(start, goal, 1e-12)
        problem.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))
        progress = None
        if parameters["record_validated_progress"]:
            from .validated_progress import ValidatedProgress
            progress = ValidatedProgress(model, case, cfg, out, started, float(deadline),
                origin="OMPL adapter entry; includes native dependency setup and all in-run validation")
            result["progress_observation_policy"] = dict(
                mode="persistent native solve continuation" if method == "RRTSTAR" else "native return and simplification",
                callback_limitation="Installed OMPL 2.0.1 wheel exposes setIntermediateSolutionCallback but cannot convert Python callable",
                poll_seconds=parameters["progress_poll_seconds"] if method == "RRTSTAR" else None,
                tree_reset=False, random_stream_reset=False,
                validation_and_scheduling_charged_to_search_budget=True,
                first_valid_semantics="observed validation completion; upper bound on internal first feasible solution time")
        planner = og.RRTConnect(si) if method == "RRTCONNECT" else og.RRTstar(si)
        planner.setRange(parameters["range_rad"])
        if method == "RRTSTAR":
            planner.setGoalBias(parameters["goal_bias"])
            planner.setRewireFactor(parameters["rewire_factor"])
            planner.setDelayCC(True)
        planner.setProblemDefinition(problem)
        planner.setup()
        result["status"] = "planning"
        write(out / "ompl_result.json", result)
        tic = time.perf_counter()
        if progress is not None and method == "RRTSTAR":
            # The installed Python wheel cannot bind the native intermediate
            # callback. Re-enter solve on the SAME native planner: its tree,
            # sampler, RNG and best incumbent are retained. This is bounded
            # continuation, not independent restarts or best-of experiments.
            continuation_calls = 0
            while True:
                slice_deadline = min(planning_deadline,
                    time.perf_counter() + parameters["progress_poll_seconds"])
                status = planner.solve(ob.PlannerTerminationCondition(
                    lambda: time.perf_counter() >= slice_deadline))
                continuation_calls += 1
                if problem.hasExactSolution():
                    incumbent = np.asarray([_coordinates(state)
                        for state in problem.getSolutionPath().getStates()])
                    progress.record(incumbent, "native_continuation",
                        float(np.linalg.norm(np.diff(incumbent, axis=0), axis=1).sum()),
                        admissible_deadline=planning_deadline)
                if time.perf_counter() >= planning_deadline:
                    break
            result["native_continuation_calls"] = continuation_calls
        else:
            status = planner.solve(ob.PlannerTerminationCondition(lambda: time.perf_counter() >= planning_deadline))
        result.update(native_solve_seconds=time.perf_counter()-tic,
                      native_status=status.asString(), native_status_code=int(status),
                      has_exact_solution=bool(problem.hasExactSolution()),
                      has_approximate_solution=bool(problem.hasApproximateSolution()),
                      native_solution_count=int(problem.getSolutionCount()))
        graph = ob.PlannerData(si)
        planner.getPlannerData(graph)
        vertices = np.array([_coordinates(graph.getVertex(i).getState()) for i in range(graph.numVertices())]).reshape(-1, 7)
        edges = np.array([(i, j) for i in range(graph.numVertices()) for j in graph.getEdges(i)], dtype=np.int64).reshape(-1, 2)
        np.savez_compressed(out / "ompl_search_graph.npz", vertices=vertices, edges=edges,
                            start_vertices=np.array([i for i in range(graph.numVertices()) if graph.isStartVertex(i)], dtype=np.int64),
                            goal_vertices=np.array([i for i in range(graph.numVertices()) if graph.isGoalVertex(i)], dtype=np.int64))
        graph_saved = True
        result.update(search_graph_vertices=len(vertices), search_graph_edges=len(edges))
        raw = None
        if problem.hasSolution():
            raw = np.array([_coordinates(state) for state in problem.getSolutionPath().getStates()])
            np.savez_compressed(out / "ompl_raw_path.npz", points=raw)
            result["raw_path_length_rad"] = float(np.linalg.norm(np.diff(raw, axis=0), axis=1).sum())
        if status.getStatus() != ob.PlannerStatus.EXACT_SOLUTION or not problem.hasExactSolution():
            result["status"] = "no_exact_solution"
            raise OmplNoExactSolution(status.asString())
        if raw is None or len(raw) < 2 or np.max(np.abs(raw[[0, -1]] - [q_start, q_goal])) > 1e-10:
            raise RuntimeError("Native exact path did not retain frozen endpoints")
        if progress is not None:
            progress.record(raw, "native_return", result["raw_path_length_rad"])
        simplified = og.PathGeometric(problem.getSolutionPath())
        active_deadline[0] = min(time.perf_counter() + parameters["simplification_seconds"],
                                 float(deadline) - parameters["validation_reserve_seconds"])
        tic = time.perf_counter()
        changed = og.PathSimplifier(si).reduceVertices(
            simplified, parameters["simplification_max_steps"],
            parameters["simplification_max_empty_steps"], 1.0)
        points = np.array([_coordinates(state) for state in simplified.getStates()])
        np.savez_compressed(out / "ompl_simplified_path.npz", points=points)
        cost = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        if cost > result["raw_path_length_rad"] + 1e-9:
            raise RuntimeError("Native vertex reduction unexpectedly increased path length")
        result.update(simplification_seconds=time.perf_counter()-tic,
                      simplification_changed=bool(changed), simplified_path_length_rad=cost,
                      status="exact_solution_candidate", solver_success=True)
        if progress is not None:
            progress.record(points, "native_simplified_return", cost)
            result["validated_progress"] = progress.summary
        return dict(kind="polygon", points=points, objective="minimum_distance",
                    solver_success=True, solver=f"OMPL {method}",
                    solver_status=status.asString(), native_status=status.asString(),
                    provenance=PROFILE["name"], raw_path_length_rad=result["raw_path_length_rad"],
                    planning_seconds=result["native_solve_seconds"],
                    simplification_seconds=result["simplification_seconds"])
    except Exception as exc:
        result.update(exception=repr(exc), traceback=traceback.format_exc())
        if result["status"] != "no_exact_solution":
            result["status"] = "exception"
        raise
    finally:
        result.update(elapsed_seconds=time.perf_counter()-started, search_graph_saved=graph_saved)
        write(out / "ompl_result.json", result)
