"""Actual native planning and numerical baselines behind immutable platform jobs.

This adapter preserves the historical reproduction modules. Numerical repairs
are injected into individual function calls; module globals are never patched.
The scheduler owns process budgets, attempts and result.json; this module owns
the numerical inputs, candidates and validation evidence inside one attempt.
"""
from __future__ import annotations

import builtins
from copy import deepcopy
from pathlib import Path
import time
import traceback
from types import FunctionType, SimpleNamespace

import numpy as np


BACKEND_VERSION = "research_backend_v2"
from .config import METHODS as CONFIG_METHODS
METHODS = set(CONFIG_METHODS)
TIME_METHODS = {"SCSTO", "NTO_time", "NTO_time_strong"}
LEGACY_NTO_METHODS = {"NTO_distance_strong": "NTO_distance", "NTO_time_strong": "NTO_time"}


def _clone(function, **replacements):
    namespace = dict(function.__globals__)
    namespace.update(replacements)
    return FunctionType(function.__code__, namespace, function.__name__,
                        function.__defaults__, function.__closure__)


def _write(path, value):
    from reproduction.common import atomic
    atomic(path, value)


def _read(path):
    from reproduction.common import read
    return read(path)


def _identity(job):
    return {key: job[key] for key in ("case_id", "method", "environment_seed", "roadmap_seed", "roadmap_size") if key in job}


def _kind(resolved):
    return resolved["scene_definition"]["kind"]


def _environment(protocol, scene, prepared=None):
    if scene["kind"] == "forest":
        from reproduction.environment_reconstruction.model import FixedForest
        env = FixedForest(protocol, scene)
    else:
        from reproduction.paper_constructed.model import FR3
        env = FR3(protocol, scene["obstacles"])
        env.start = np.asarray(scene["start"], np.float32).astype(float)
    if prepared is not None:
        env.start = np.asarray(prepared["start"], dtype=float)
        env.goal = np.asarray(prepared["goal"], dtype=float)
        if (not np.array_equal(env.start, env.start.astype(np.float32).astype(float))
                or not np.array_equal(env.goal, env.goal.astype(np.float32).astype(float))):
            raise ValueError("Prepared endpoints must already use the native float32 representation")
    if hasattr(env, "goal"):
        env.cfg.update(start=env.start.copy(), goal=env.goal.copy())
    return env


def candidate_preflight(candidate, environment):
    """Reject malformed or unbounded-cost validation work before any GPU query."""
    kind, d = candidate.get("kind"), environment.dimension
    cfg = environment.cfg
    cap = int(cfg.get("validation_max_samples", 200000))
    step = float(cfg["validation_step"])
    floor = int(cfg["validation_minimum_samples"])
    if cap < 2 or floor < 2 or not np.isfinite(step) or step <= 0:
        raise ValueError("Invalid validation sample budget")
    if kind == "polygon":
        q = np.asarray(candidate.get("points"), dtype=float)
        if q.ndim != 2 or len(q) < 2 or q.shape[1] != d or not np.isfinite(q).all():
            raise ValueError("Polygon candidate must contain at least two finite configurations")
        regions = candidate.get("segment_regions", [])
        if len(regions) != len(q)-1:
            raise ValueError("Polygon segment-region count mismatch")
        count = 1 + sum(max(1, int(np.ceil(np.linalg.norm(v-u)/step))) for u, v in zip(q[:-1], q[1:]))
    elif kind == "bezier":
        controls = np.asarray(candidate.get("controls"), dtype=float)
        durations = np.asarray(candidate.get("durations"), dtype=float)
        if (controls.ndim != 3 or controls.shape[0] < 1 or controls.shape[1] < 3 or controls.shape[2] != d
                or durations.shape != (len(controls),) or not np.isfinite(controls).all()
                or not np.isfinite(durations).all() or not np.all(durations > 0)):
            raise ValueError("Bezier controls and strictly positive durations must be finite and shape-consistent")
        if len(candidate.get("segment_regions", [])) != len(controls):
            raise ValueError("Bezier segment-region count mismatch")
        velocity_controls = ((controls.shape[1]-1)*np.diff(controls, axis=1)/durations[:, None, None])
        speed = float(np.linalg.norm(velocity_controls, axis=2).max())
        count = max(floor, int(np.ceil(durations.sum()*speed/step))+1)
    elif kind == "bspline":
        from pydrake.all import BsplineBasis, BsplineTrajectory
        controls = np.asarray(candidate.get("controls"), dtype=float)
        knots = np.asarray(candidate.get("knots"), dtype=float)
        order = candidate.get("spline_order")
        duration = float(candidate.get("duration", np.nan))
        if (not isinstance(order, (int, np.integer)) or order < 2 or controls.ndim != 2
                or len(controls) < order or controls.shape[1] != d or not np.isfinite(controls).all()
                or knots.ndim != 1 or len(knots) != len(controls)+order or not np.isfinite(knots).all()
                or np.any(np.diff(knots) < 0) or not np.isfinite(duration) or duration <= 0):
            raise ValueError("B-spline candidate has invalid controls, knots, order or duration")
        actual_duration = float(knots[-order]-knots[order-1])
        if actual_duration <= 0 or not np.isclose(duration, actual_duration, rtol=1e-9, atol=1e-10):
            raise ValueError("B-spline duration disagrees with its knot interval")
        trajectory = BsplineTrajectory(BsplineBasis(order, knots.tolist()), [p[:, None] for p in controls])
        derivative = trajectory.MakeDerivative(1)
        speed = max(float(np.linalg.norm(p)) for p in derivative.control_points())
        count = max(floor, int(np.ceil(duration*speed/step))+1)
    else:
        raise ValueError(f"Unknown candidate representation: {kind}")
    if count > cap:
        raise ValueError(f"Candidate requires {count} validation samples, exceeding the declared cap {cap}")
    return {"expected_samples": count, "maximum_samples": cap, "finite_shape_checks_passed": True}


def validate_candidate(candidate, environment, regions=None):
    from reproduction.paper_constructed.validation import validate
    preflight = candidate_preflight(candidate, environment)
    if regions:
        assignments = np.asarray(candidate.get("segment_regions"))
        if assignments.dtype.kind not in "iu" or np.any(assignments < 0) or np.any(assignments >= len(regions)):
            raise ValueError("Candidate refers to a nonexistent region")
        if candidate["kind"] == "bezier" and len(regions) != len(candidate["controls"]):
            raise ValueError("Bezier-region alignment must be one-to-one for the reused validator")
    # Historical validation branches on the exact method name. Canonicalize a
    # private view so strengthened distance paths are not tested as timed paths.
    legacy = dict(candidate, method=LEGACY_NTO_METHODS.get(candidate.get("method"), candidate.get("method")))
    answer = validate(legacy, environment, regions)
    if environment.kind == "forest":
        from .continuous_checks import forest_curve_check
        continuous = forest_curve_check(candidate, environment)
        answer["continuous_geometry"] = continuous
        answer["normalized_collision_parameters"] = continuous["normalized_collision_parameters"]
        answer["normalized_minimum_parameters"] = continuous.get("normalized_minimum_parameters", [])
        answer["clearance_witnesses"] = continuous.get("clearance_witnesses", [])
        answer["minimum_clearance"] = continuous.get("minimum_clearance")
        # Give geometric recovery the same newly detected witnesses as NTO.
        # Auxiliary witness rows have no replay time and never affect costs.
        points = np.asarray(continuous["collision_points"], dtype=float).reshape(-1, environment.dimension)
        if len(points):
            if candidate["kind"] in ("bezier", "polygon"):
                pieces = np.asarray(continuous["collision_segments"], dtype=int)
            else:
                pieces = np.zeros(len(points), dtype=int)
            assignments = np.asarray(candidate.get("segment_regions", [0]), dtype=int)
            region_ids = assignments[pieces] if regions else np.zeros(len(points), dtype=int)
            answer["collision_points"] = np.vstack([answer["collision_points"], points])
            answer["collision_regions"] = np.r_[answer["collision_regions"], region_ids]
            samples = answer["_samples"]
            samples["q"] = np.vstack([samples["q"], points])
            samples["time"] = np.r_[samples["time"], np.full(len(points), np.nan)]
            samples["cuda"] = np.r_[samples["cuda"], environment.cuda(points)]
            samples["independent"] = np.r_[samples["independent"], environment.independent(points)]
            answer["samples"] = len(samples["q"])
            answer["cuda_collisions"] = int((~samples["cuda"]).sum())
            answer["native_cuda_collision_free"] = bool(samples["cuda"].all())
            answer["independent_collisions"] = int((~samples["independent"]).sum())
            answer["model_disagreements"] = int((samples["cuda"] != samples["independent"]).sum())
        answer["passed"] = bool(answer["passed"] and continuous["resolved"] and continuous["collision_free"])
        answer["collision_free"] = bool(answer["collision_free"] and continuous["resolved"] and continuous["collision_free"])
        answer["independent_collision_free"] = bool(answer["independent_collision_free"] and continuous["resolved"] and continuous["collision_free"])
    answer["preflight"] = preflight
    return answer


def lgcs_configured(regions, start, goal, time_limit):
    """Original GCS model, explicit strict tolerances and an actual solver limit."""
    import gurobipy as gp
    from reproduction.paper_constructed.optimization import lgcs
    from reproduction.alignment_diagnosis.geometric_solvers import STRICT_GUROBI_PARAMETERS
    models = []
    parameters = dict(STRICT_GUROBI_PARAMETERS, TimeLimit=float(time_limit))

    class ModelProxy:
        def __init__(self, *args, **kwargs):
            self.model = gp.Model(*args, **kwargs)
            models.append(self.model)

        def __getattr__(self, name):
            return getattr(self.model, name)

        def optimize(self):
            # The historical function writes a literal 60 s while constructing
            # its model. Set the requested value immediately before optimize.
            for key, value in parameters.items():
                self.model.setParam(key, value)
            return self.model.optimize()

    proxy = SimpleNamespace(**vars(gp))
    proxy.Model = ModelProxy
    real_import = builtins.__import__

    def local_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "gurobipy" and level == 0:
            return proxy
        return real_import(name, globals, locals, fromlist, level)

    local_builtins = dict(vars(builtins), __import__=local_import)
    try:
        result = _clone(lgcs, __builtins__=local_builtins)(regions, start, goal)
        result["solver_parameters"].update(parameters)
        result["numerical_variant"] = "gcs_strict_tolerances_v1"
        if models and models[-1].SolCount:
            result["solver_quality"] = {key: float(getattr(models[-1], key)) for key in ("MaxVio", "ConstrVio", "IntVio", "BoundVio")}
        return result
    except Exception as exc:
        if hasattr(exc, "optimization_metadata"):
            exc.optimization_metadata["solver_parameters"].update(parameters)
        raise
    finally:
        for model in models:
            model.dispose()


def _save_validation(out, validation, method):
    public = {key: value for key, value in validation.items() if key != "_samples"}
    _write(out / "validation.json", public)
    sample = validation["_samples"]
    np.savez_compressed(out / "trajectory_samples.npz", q=sample["q"], time=sample["time"],
                        native=sample["cuda"], independent=sample["independent"])
    return "time_seconds" if method in TIME_METHODS else "normalized_path_parameter"


def _save_candidate(out, candidate):
    arrays = {key: value for key, value in candidate.items() if isinstance(value, np.ndarray)}
    np.savez_compressed(out / "candidate.npz", **arrays)
    _write(out / "candidate.json", {key: value for key, value in candidate.items() if not isinstance(value, np.ndarray)})


def _metrics(candidate, validation, method):
    if validation is None:
        return {"cost": None, "cost_type": "time" if method in TIME_METHODS else "distance",
                "path_length": None, "duration": None}
    samples = validation["_samples"]
    q = samples["q"][np.isfinite(samples["time"])]
    length_source = "sampled_curve_length"
    if candidate.get("kind") == "polygon":
        q = np.asarray(candidate["points"])
        length_source = "exact_polygonal_length"
    elif candidate.get("kind") == "bspline" and candidate.get("spline_order") == 2:
        # An order-two B-spline is exactly the control polygon. A uniform
        # validation grid can miss knots and underestimate its true length.
        q = np.asarray(candidate["controls"])
        length_source = "exact_order_two_bspline_length"
    path_length = float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum())
    timed = method in TIME_METHODS
    return {"cost": float(validation["cost"]) if timed else path_length,
            "cost_type": "time" if timed else "distance", "path_length": path_length,
            "path_length_source": length_source,
            "duration": float(validation["duration"]) if timed else None}


def forest_map_geometry(protocol):
    """An offline voxel map must cover the actual configured domain."""
    lower, upper = (np.asarray(protocol[name], dtype=float) for name in ("domain_lower", "domain_upper"))
    if lower.shape != (2,) or upper.shape != (2,) or not np.isfinite([lower, upper]).all() or np.any(upper <= lower):
        raise ValueError("Invalid Forest map bounds")
    side = float(protocol["offline_voxel_side"])
    return np.r_[upper-lower, side], np.r_[(upper+lower)/2, 0.]


def _build_forest_roadmap(protocol, seed, size, out):
    """Native roadmap construction with configured, rather than literal, bounds."""
    import csdecomp as csd
    from reproduction.paper_constructed.model import ForestEnvironment
    env = ForestEnvironment(protocol, 0)
    nodes = np.random.default_rng(np.random.SeedSequence([2025, seed])).uniform(env.lower, env.upper, (size, 2)).astype(np.float32)
    np.save(out / "samples.npy", nodes)
    options = csd.RoadmapOptions()
    extent, center = forest_map_geometry(protocol)
    options.robot_map_size_x, options.robot_map_size_y, options.robot_map_size_z = extent
    options.map_center = center
    options.offline_voxel_resolution = protocol["offline_voxel_side"]/2
    options.max_task_space_distance_between_nodes = protocol["max_task_space_distance"]
    options.max_configuration_distance_between_nodes = protocol["max_configuration_distance"]
    options.edge_step_size = protocol["lazy_edge_step"]
    options.nodes_processed_before_debug_statement = 100000
    builder = csd.RoadmapBuilder(env.native, "point", options)
    started = time.perf_counter()
    builder.add_nodes_manual(nodes.T)
    builder.build_roadmap(protocol["roadmap_neighbors"])
    builder.build_pose_map()
    builder.build_collision_map()
    builder.write(str(out / "native.map"))
    drm = builder.get_drm()
    ids = sorted(drm.id_to_node_map)
    np.savez_compressed(out / "roadmap.npz", node_ids=ids,
        nodes=np.asarray([drm.id_to_node_map[i] for i in ids]),
        edges=np.asarray([(i, j) for i, adjacent in drm.node_adjacency_map.items() for j in adjacent]))
    result = {"seed": seed, "requested_nodes": size, "actual_nodes": len(ids),
        "offline_seconds": time.perf_counter()-started, "status": "ready",
        "voxel_side": protocol["offline_voxel_side"], "native_voxel_half_side": options.offline_voxel_resolution,
        "offline_map_extent": extent, "offline_map_center": center,
        "domain_lower": env.lower, "domain_upper": env.upper,
        "collision_map_entries": len(drm.collision_map), "active_robot_spheres": len(env.geometry_ids),
        "static_candidate_pairs": env.minimal.num_collision_pairs,
        "obstacles": "online only; independent of offline roadmap"}
    _write(out / "roadmap.json", result)
    return result


def _scene(job, resolved, out):
    definition, protocol = deepcopy(resolved["scene_definition"]), resolved["protocol"]
    seed = int(job["environment_seed"])
    if definition["kind"] == "forest":
        if "centers" not in definition:
            sample = definition["sampling"]
            centers = np.random.default_rng(seed).uniform(sample["center_lower"], sample["center_upper"], (sample["count"], 2))
            definition.update(centers=centers.tolist(), obstacle_centers=centers.tolist())
        definition.update(status="ready", environment_seed=seed)
        scene = definition
        _write(out / "scene.json", scene)
        from reproduction.paper_constructed.data import endpoint_audit
        env = _environment(protocol, scene)
        checks = {"start": endpoint_audit(env, env.start), "goal": endpoint_audit(env, env.goal)}
        _write(out / "endpoint_audit.json", checks)
        valid = all(item["canonical_valid"] for item in checks.values())
    else:
        from reproduction.paper_constructed.data import make_scene
        scene = make_scene(protocol, seed, out)
        scene.update(kind="fr3", environment_seed=seed, profile=definition["profile"])
        _write(out / "scene.json", scene)
        valid = scene["status"] == "ready"
    return {"status": "success" if valid else "scene_failed", "scene_status": scene["status"], "environment": definition["kind"]}


def _roadmap(job, resolved, out):
    if _kind(resolved) == "forest":
        build_roadmap = _build_forest_roadmap
    else:
        from reproduction.paper_constructed.data import build_roadmap
    record = build_roadmap(resolved["protocol"], int(job["roadmap_seed"]), int(job["roadmap_size"]), out)
    return {"status": "success", "environment": _kind(resolved), "roadmap": record}


def _plan(job, resolved, dependencies, out):
    from reproduction.paper_constructed.data import planner, online_pose_ik, endpoint_audit
    scene = _read(dependencies["scene"] / "scene.json")
    env = _environment(resolved["protocol"], scene)
    p = planner(env)
    p.LoadRoadmap(str(dependencies["roadmap"] / "native.map"))
    started = time.perf_counter()
    p.BuildCollisionSet(env.voxels)
    collision_set_seconds = time.perf_counter()-started
    ik_seconds = 0.
    if scene["kind"] == "fr3":
        started = time.perf_counter()
        ik = online_pose_ik(env, p.drm, np.asarray(scene["target_pose"]), out, p.collision_set)
        ik_seconds = time.perf_counter()-started
        if ik["status"] != "success":
            return {"status": "ik_failed", "solver_success": False, "validation_passed": False, "ik_status": ik["status"]}
        env.goal = np.asarray(ik["goal"], dtype=float)
        env.cfg.update(start=env.start, goal=env.goal)
    prepared = {"kind": scene["kind"], "start": env.start, "goal": env.goal,
                "centers": scene["centers"] if scene["kind"] == "forest" else scene["obstacles"],
                "target_pose": scene.get("target_pose"), "scene": scene,
                "endpoint_representation": "float32_roundtrip_without_endpoint_displacement"}
    _write(out / "input.json", prepared)
    audits = {"start": endpoint_audit(env, env.start), "goal": endpoint_audit(env, env.goal)}
    _write(out / "endpoint_audit.json", audits)
    if not all(item["canonical_valid"] for item in audits.values()):
        return {"status": "endpoint_invalid", "solver_success": False, "validation_passed": False}
    started = time.perf_counter()
    try:
        success, path = p.Plan(env.start, env.goal, env.voxels, env.voxel_radius)
    except RuntimeError as exc:
        if "No collision-free path exists in roadmap" not in str(exc):
            raise
        _write(out / "planning_failure.json", {"exception": repr(exc), "scope": "Finite roadmap failure; not continuous-space infeasibility"})
        return {"status": "planning_failed", "solver_success": False, "validation_passed": False,
                "failure_code": "native_no_path", "native_planner_options": env.native_planner_options}
    planning_seconds = time.perf_counter()-started
    path = np.asarray(path, dtype=float)
    np.savez_compressed(out / "seed_path.npz", path=path)
    if not success or path.ndim != 2 or len(path) < 2:
        return {"status": "planning_failed", "solver_success": False, "validation_passed": False}
    candidate = {"kind": "polygon", "method": "DRM", "points": path, "segment_regions": [0]*(len(path)-1),
                 "solver_success": True, "solver": "native_DrmPlanner", "solver_status": "native_returned_path"}
    _save_candidate(out, candidate)
    started = time.perf_counter()
    validation = validate_candidate(candidate, env)
    validation_seconds = time.perf_counter()-started
    axis = _save_validation(out, validation, "DRM")
    return {"status": "success" if validation["passed"] else "validation_failed",
            "solver_success": True, "validation_passed": validation["passed"],
            "validation": {k: v for k, v in validation.items() if k not in ("_samples", "collision_points", "collision_regions")},
            "metrics": _metrics(candidate, validation, "DRM"), "axis_semantics": axis,
            "native_planner_options": env.native_planner_options,
            "timing": {"collision_set_seconds": collision_set_seconds, "ik_seconds": ik_seconds,
                       "planning_seconds": planning_seconds, "validation_seconds": validation_seconds}}


def _method(job, resolved, dependencies, out):
    method = job["method"]
    if method not in METHODS:
        raise ValueError(f"Unsupported method {method}")
    plan_dir = dependencies["plan"]
    # Every method, including NTO, requires the identical independently valid
    # DRM input. A failed path is not silently rescued for just one baseline.
    if not (plan_dir / "validation.json").exists() or not _read(plan_dir / "validation.json").get("passed"):
        return {"status": "blocked", "reason": "shared_DRM_seed_not_independently_valid",
                "solver_success": False, "validation_passed": False}
    prepared = _read(plan_dir / "input.json")
    env = _environment(resolved["protocol"], prepared["scene"], prepared)
    path = np.load(plan_dir / "seed_path.npz")["path"]
    timing = {"construction_seconds": 0., "optimization_seconds": 0., "validation_seconds": 0.}
    regions, validation, method_status, termination_reason = None, None, None, None
    if method == "DRM":
        candidate = {"kind": "polygon", "method": method, "points": path,
                     "segment_regions": [0]*(len(path)-1), "solver_success": True,
                     "solver": "native_DrmPlanner", "solver_status": "shared_validated_seed"}
        variant = "shared_native_DRM_seed_v1"
    elif method in LEGACY_NTO_METHODS:
        from .strong_nto import solve as solve_strong
        outcome = solve_strong(path, env, minimum_time=method == "NTO_time_strong",
            options=resolved["config"]["strong_nto"], out=out, validate_callback=validate_candidate)
        candidate, validation = outcome["candidate"], outcome.get("validation")
        method_status, termination_reason = outcome.get("status"), outcome.get("termination_reason")
        timing.update(outcome.get("timing", {}))
        _write(out / "refinement_history.json", {k: v for k, v in outcome.items()
            if k not in ("candidate", "validation")})
        variant = "adaptive_nto_exact_forest_segments_v1" if method == "NTO_distance_strong" and env.kind == "forest" else "adaptive_nto_enriched_spline_v1"
    elif method.startswith("NTO"):
        from reproduction.paper_constructed.optimization import nto
        started = time.perf_counter()
        candidate = nto(path, env, minimum_time=method == "NTO_time")
        timing["optimization_seconds"] = time.perf_counter()-started
        variant = "point_collocation_original_transcription_v1"
    else:
        from reproduction.paper_constructed.pipeline import Pipeline
        from reproduction.paper_constructed.data import attach_alternate
        from reproduction.alignment_diagnosis.geometric_solvers import scsto_preserve_knots
        attach_alternate(env, dependencies["roadmap"] / "native.map", out)
        pipe = Pipeline(env, int(job["roadmap_seed"]))
        time_limit = env.protocol["optimizer_time_limit_seconds"]
        solve = _clone(Pipeline.solve,
            lgcs=lambda regions, start, goal: lgcs_configured(regions, start, goal, time_limit),
            scsto=scsto_preserve_knots, validate=validate_candidate)
        # Pipeline.solve only forwards this argument to alternate_path; that
        # helper reads a new private map and never mutates this shared map.
        outcome = solve(pipe, method, path, None, out)
        candidate, regions = outcome["candidate"], outcome["regions"]
        validation = outcome.get("validation")
        timing.update({key: float(outcome.get(key, 0.)) for key in timing})
        _write(out / "pipeline.json", {k: v for k, v in outcome.items() if k not in ("candidate", "regions", "validation")})
        variant = {"LSCS": "clarabel_socp_v1", "LGCS": "gcs_strict_tolerances_v1",
                   "SCSTO": "author_biconvex_preserve_all_initial_knots_v1"}[method]
    candidate.update(method=method, numerical_variant=variant)
    _save_candidate(out, candidate)
    if validation is None and method not in LEGACY_NTO_METHODS:
        started = time.perf_counter()
        # Even a failed NTO candidate is preserved and checked when finite;
        # solver failure remains failure independently of candidate feasibility.
        if candidate.get("points") is not None or candidate.get("controls") is not None:
            validation = validate_candidate(candidate, env, regions)
        timing["validation_seconds"] += time.perf_counter()-started
    solver_success = bool(candidate.get("solver_success", False))
    passed = bool(validation is not None and validation["passed"])
    axis = _save_validation(out, validation, method) if validation is not None else None
    status = "success" if solver_success and passed else "solver_failed" if not solver_success else "validation_failed"
    if method_status is not None and method_status != "success":
        status = method_status
    return {"status": status, "solver_success": solver_success, "validation_passed": passed,
            "termination_reason": termination_reason,
            "solver": candidate.get("solver"), "solver_status": candidate.get("solver_status"),
            "numerical_variant": variant, "metrics": _metrics(candidate, validation, method),
            "timing": timing, "axis_semantics": axis,
            "validation": None if validation is None else {k: v for k, v in validation.items() if k not in ("_samples", "collision_points", "collision_regions")}}


def execute_job(job: dict, resolved: dict, dependencies: dict[str, Path], out: Path) -> dict:
    """Execute exactly one scheduler-authorized job; never write dependencies."""
    out = Path(out)
    dependencies = {key: Path(path) for key, path in dependencies.items()}
    if any(out.resolve() == path.resolve() or path.resolve() in out.resolve().parents
           or out.resolve() in path.resolve().parents for path in dependencies.values()):
        raise ValueError("Attempt output must not be inside an immutable dependency")
    out.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    try:
        if job["kind"] == "scene":
            result = _scene(job, resolved, out)
        elif job["kind"] == "roadmap":
            result = _roadmap(job, resolved, out)
        elif job["kind"] == "plan":
            result = _plan(job, resolved, dependencies, out)
        elif job["kind"] == "method":
            result = _method(job, resolved, dependencies, out)
        else:
            raise ValueError(f"Unknown job kind: {job['kind']}")
    except Exception as exc:
        license_failure = getattr(exc, "errno", None) in (10009, 10010) or "license" in str(exc).lower()
        result = {"status": "license_unavailable" if license_failure else "error",
                  "solver_success": False, "validation_passed": False, "exception": repr(exc),
                  "traceback": traceback.format_exc(), "optimization_metadata": getattr(exc, "optimization_metadata", None)}
        _write(out / "failure.json", result)
    result.update(_identity(job), backend_version=BACKEND_VERSION, environment=_kind(resolved))
    result.setdefault("timing", {})["wall_seconds"] = time.perf_counter()-started
    return result
