"""CHOMP covariant descent and the official ros-industrial STOMP core.

Both optimize a supplied DRM path without changing its endpoints or collision
geometry. CHOMP uses the paper's trajectory-metric descent on the explicit
C-space line-integral potential below; this avoids replacing canonical boxes
with MoveIt's collision spheres. STOMP calls unchanged upstream C++ code with
the canonical signed-distance task callback. Their native smoothness/obstacle
costs are retained separately from the common PWL-length evaluation metric.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sysconfig
import time
import traceback

import numpy as np

from ..roadmap import write
from .validate import _numeric_array

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parents[1]
NATIVE = HERE / "native"
STOMP_COMMIT = "b1a87c80f7338caae25a5c689b876da15492aa75"
BUILD = ROOT / ".local_setup/solver_probes/stomp-bridge-v1"
PYBIND = ROOT / ".local_setup/solver_probes/pybind11_2_13/pybind11/include"
COMMON = dict(initial_knots=64, maximum_knots=256, obstacle_influence_m=.05,
              collision_check_step_rad=.05, safety_band_m=2e-6)
CHOMP_PROFILE = dict(**COMMON, maximum_iterations=40, smoothness_weight=1.,
                     obstacle_weight=10., finite_difference_rad=1e-5,
                     learning_rate=1., maximum_update_rad=.08,
                     backtracking_steps=8, relative_cost_tolerance=1e-7)
STOMP_PROFILE = dict(**COMMON, maximum_iterations=40, iterations_after_valid=5,
                     rollouts=12, noise_std_rad=.10, cost_sensitivity=10.,
                     control_cost_weight=.10, maximum_update_rad=.08)


def _hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def native_sources():
    return sorted([NATIVE / "stomp_python.cpp", *(
        NATIVE / "vendor/stomp").rglob("*.cpp"), *(
        NATIVE / "vendor/stomp").rglob("*.h")])


def build_stomp(build_dir=None):
    """Explicit offline build, with receipts; no package/environment mutation."""
    build = Path(build_dir or BUILD)
    build.mkdir(parents=True, exist_ok=True)
    suffix = sysconfig.get_config_var("EXT_SUFFIX")
    binary = build / ("_generic_stomp" + suffix)
    sources = {str(p.relative_to(ROOT)): _hash(p) for p in native_sources()}
    command = ["g++", "-O3", "-DNDEBUG", "-std=c++17", "-shared", "-fPIC",
               "-I" + sysconfig.get_paths()["include"], "-I" + str(PYBIND),
               "-I/usr/include/eigen3", "-I" + str(NATIVE / "vendor/stomp/include"),
               str(NATIVE / "stomp_python.cpp"),
               str(NATIVE / "vendor/stomp/src/stomp.cpp"),
               str(NATIVE / "vendor/stomp/src/utils.cpp"),
               "-lconsole_bridge", "-o", str(binary)]
    receipt = dict(upstream_url="https://github.com/ros-industrial/stomp",
                   upstream_commit=STOMP_COMMIT, upstream_core_modified=False,
                   command=command, sources_sha256=sources, binary=str(binary),
                   purpose="offline dependency build excluded from online planning",
                   pybind11_version="2.13.6 (isolated headers; NumPy 2 compatible)",
                   pybind11_header_sha256=_hash(PYBIND / "pybind11/pybind11.h"),
                   pybind11_numpy_header_sha256=_hash(PYBIND / "pybind11/numpy.h"))
    started = time.perf_counter()
    result = subprocess.run(command, capture_output=True, text=True)
    (build / "build.log").write_text(result.stdout + result.stderr)
    receipt.update(returncode=result.returncode, seconds=time.perf_counter()-started)
    if result.returncode == 0:
        receipt["binary_sha256"] = _hash(binary)
    write(build / "build.json", receipt)
    if result.returncode:
        raise RuntimeError(f"STOMP offline build failed; see {build / 'build.log'}")
    return receipt


def _backend(build_dir=None):
    build = Path(build_dir or BUILD)
    record = json.loads((build / "build.json").read_text())
    binary = Path(record["binary"])
    expected = {str(p.relative_to(ROOT)): _hash(p) for p in native_sources()}
    if record.get("sources_sha256") != expected or _hash(binary) != record.get("binary_sha256"):
        raise RuntimeError("STOMP compiled backend/source hash mismatch; rebuild offline")
    spec = importlib.util.spec_from_file_location("_generic_stomp", binary)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, record


def dependency_probe(out=None):
    module, record = _backend()
    matrix = np.asarray(module.smoothing_matrix(16))
    if matrix.shape != (16, 16) or not np.isfinite(matrix).all():
        raise RuntimeError("STOMP actual native smoothing-matrix probe failed")
    record = dict(record, probe_success=True, native_matrix_shape=list(matrix.shape))
    if out:
        write(Path(out) / "stomp_dependencies.json", record)
    return record


def initial_trajectory(path, count=64, maximum=256):
    """Add arc-spaced knots but retain every original vertex (no corner cut)."""
    path = _numeric_array(path, "CHOMP/STOMP DRM seed")
    if path.ndim != 2 or path.shape[1] != 7 or len(path) < 2 or not np.isfinite(path).all():
        raise ValueError("Expected at least two finite seven-joint seed vertices")
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    keep = np.r_[True, lengths > 0]
    unique = path[keep]
    if len(unique) < 2:
        raise ValueError("CHOMP/STOMP seed has zero arc length")
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(unique, axis=0), axis=1))]
    positions = np.unique(np.r_[arc, np.linspace(0., arc[-1], int(count))])
    if len(positions) > maximum:
        raise ValueError("CHOMP/STOMP initialization exceeds fixed knot cap")
    result = np.column_stack([np.interp(positions, arc, unique[:, j]) for j in range(7)])
    result[0], result[-1] = path[0], path[-1]
    return result


def acceleration_metric(n):
    """Fixed-endpoint second-difference metric, positive definite on interiors."""
    if n < 3:
        raise ValueError("At least three knots required")
    d2 = np.diff(np.eye(n), n=2, axis=0)
    interior = d2[:, 1:-1]
    metric = interior.T @ interior
    inverse = np.linalg.solve(metric, np.eye(n-2))
    return d2, metric, inverse


def obstacle_potential(distance, influence=.05):
    """CHOMP's linear penetration / quadratic near-obstacle / zero far field."""
    distance = np.asarray(distance, float)
    values = np.where(distance < 0, -distance + influence/2,
                      np.maximum(influence-distance, 0.)**2 / (2*influence))
    derivative = np.where(distance < 0, -1.,
                          -np.maximum(influence-distance, 0.) / influence)
    return values, derivative


def discrete_cost_gradient(q, clearance, clearance_gradient, profile):
    """Exact derivative of discrete arc-integral and squared accelerations.

    Uobs = sum_i .5*(c_i+c_{i+1})*||q_{i+1}-q_i||.  The line integral
    approximates CHOMP's reparameterization-invariant obstacle functional.
    Using signed clearance in C-space preserves the frozen full geometry.
    """
    q = np.asarray(q, float)
    potential, slope = obstacle_potential(clearance, profile["obstacle_influence_m"])
    delta = np.diff(q, axis=0)
    length = np.linalg.norm(delta, axis=1)
    average = .5 * (potential[:-1] + potential[1:])
    obstacle = float(average @ length)
    acceleration = np.diff(q, n=2, axis=0)
    smooth = .5 * float(np.sum(acceleration**2))
    gradient = np.zeros_like(q)
    weight = profile["smoothness_weight"]
    gradient[:-2] += weight * acceleration
    gradient[1:-1] -= 2 * weight * acceleration
    gradient[2:] += weight * acceleration
    if clearance_gradient is not None:
        gradient_potential = slope[:, None] * np.asarray(clearance_gradient)
        g = np.zeros_like(q)
        unit = np.divide(delta, length[:, None], out=np.zeros_like(delta),
                         where=length[:, None] > 1e-14)
        g[:-1] -= average[:, None] * unit
        g[1:] += average[:, None] * unit
        g[:-1] += .5 * length[:, None] * gradient_potential[:-1]
        g[1:] += .5 * length[:, None] * gradient_potential[1:]
        gradient += profile["obstacle_weight"] * g
    return (profile["smoothness_weight"] * smooth +
            profile["obstacle_weight"] * obstacle), gradient, dict(
                native_smoothness_cost=smooth, native_obstacle_cost=obstacle)


class GeometryTask:
    def __init__(self, model, profile, deadline):
        self.model, self.profile, self.deadline = model, profile, deadline
        self.clearance_calls = 0
        self.clearance_states = 0
        self.geometry_seconds = 0.
        self.analytic_gradient_calls = 0

    def check_deadline(self):
        if time.perf_counter() >= self.deadline:
            raise TimeoutError("CHOMP/STOMP exhausted shared planning budget")

    def clearance(self, q):
        self.check_deadline()
        start = time.perf_counter()
        result = self.model.clearance(q)
        self.geometry_seconds += time.perf_counter()-start
        self.clearance_calls += 1
        self.clearance_states += len(q)
        self.check_deadline()
        if not np.isfinite(result).all():
            raise ValueError("Nonfinite canonical signed-distance callback")
        return result

    def gradient(self, q):
        step = self.profile["finite_difference_rad"]
        if hasattr(self.model, "_robot_geometries"):
            from .canonical_distance import clearance_gradient
            self.check_deadline()
            started = time.perf_counter()
            _, gradients = clearance_gradient(self.model, q, self.deadline, step)
            self.geometry_seconds += time.perf_counter()-started
            self.analytic_gradient_calls += 1
            self.check_deadline()
            return gradients
        probes = np.tile(q, (14, 1))
        n = len(q)
        for j in range(7):
            probes[(2*j)*n:(2*j+1)*n, j] += step
            probes[(2*j+1)*n:(2*j+2)*n, j] -= step
        values = self.clearance(probes).reshape(7, 2, n)
        return ((values[:, 0]-values[:, 1])/(2*step)).T

    def polyline_validity(self, q):
        step = self.profile["collision_check_step_rad"]
        counts = np.maximum(2, np.ceil(np.linalg.norm(np.diff(q, axis=0), axis=1)/step).astype(int)+1)
        points = np.vstack([np.linspace(a, b, int(n)) for a, b, n in zip(q[:-1], q[1:], counts)])
        distance = self.clearance(points)
        return bool(np.all(distance > self.profile["safety_band_m"])), float(distance.min())


def _chomp(initial, task, out, profile):
    q = initial.copy()
    d2, metric, inverse = acceleration_metric(len(q))
    preconditioner = inverse / np.max(np.diag(inverse))
    np.savez_compressed(out / "chomp_metric.npz", d2=d2, metric=metric,
                        inverse=inverse, preconditioner=preconditioner)
    history, trajectories = [], [q.copy()]
    stop_reason = "maximum_iterations"
    distance = task.clearance(q)
    cost, _, parts = discrete_cost_gradient(q, distance, None, profile)
    initial_cost = cost
    for iteration in range(1, profile["maximum_iterations"] + 1):
        task.check_deadline()
        # Endpoints are fixed, so their clearance derivatives never contribute
        # to an update; retain zero rows for those boundary conditions.
        gradient_distance = np.zeros_like(q)
        gradient_distance[1:-1] = task.gradient(q[1:-1])
        cost, gradient, parts = discrete_cost_gradient(q, distance, gradient_distance, profile)
        update = -profile["learning_rate"] * preconditioner @ gradient[1:-1]
        scale = max(1., np.abs(update).max()/profile["maximum_update_rad"])
        update /= scale
        accepted = False
        previous_cost = cost
        for backtrack in range(profile["backtracking_steps"]):
            candidate = q.copy()
            candidate[1:-1] = np.clip(q[1:-1] + update*(.5**backtrack),
                                     task.model.lower, task.model.upper)
            candidate_distance = task.clearance(candidate)
            candidate_cost, _, candidate_parts = discrete_cost_gradient(
                candidate, candidate_distance, None, profile)
            if candidate_cost < cost - 1e-12:
                q, distance, cost, parts = candidate, candidate_distance, candidate_cost, candidate_parts
                accepted = True
                break
        record = dict(iteration=iteration, native_cost=cost, accepted=accepted,
                      backtracking_count=backtrack+1, minimum_knot_clearance_m=float(distance.min()),
                      maximum_proposed_update_rad=float(np.abs(update).max()), **parts)
        history.append(record)
        trajectories.append(q.copy())
        # Incremental evidence remains available when an external deadline kills
        # the process before the final candidate can be returned.
        with (out / "chomp_iterations.jsonl").open("a") as stream:
            stream.write(json.dumps(record, allow_nan=False)+"\n")
        np.save(out / "optimizer_latest.npy", q)
        if not accepted:
            stop_reason = "line_search_stationary"
            break
        if previous_cost-cost <= profile["relative_cost_tolerance"]*max(1., abs(previous_cost)):
            stop_reason = "relative_cost_tolerance"
            break
    valid, clearance = task.polyline_validity(q)
    np.savez_compressed(out / "chomp_raw.npz", trajectories=np.asarray(trajectories),
                        final_clearance=distance)
    return q, dict(solver_success=valid, native_cost=cost, initial_native_cost=initial_cost,
                   iterations=len(history), stop_reason=stop_reason,
                   minimum_optimizer_sample_clearance_m=clearance, **parts)


class StompTask(GeometryTask):
    def __init__(self, model, profile, deadline, initial, out, seed, native):
        super().__init__(model, profile, deadline)
        self.initial, self.out = initial.copy(), out
        self.rng = np.random.Generator(np.random.PCG64(int(seed)))
        _, _, inverse = acceleration_metric(len(initial))
        covariance = inverse / np.max(np.diag(inverse))
        self.factor = np.linalg.cholesky(covariance)
        self.smoothing = np.asarray(native.smoothing_matrix(len(initial)))
        self.history, self.trajectories = [], [initial.copy()]
        self.noise_records, self.cost_records = [], []
        self.final_record = None
        np.savez_compressed(out / "stomp_metric.npz", covariance=covariance,
                            factor=self.factor, smoothing=self.smoothing)

    def noise(self, q, iteration, rollout):
        self.check_deadline()
        noise = np.zeros_like(q)
        noise[1:-1] = self.profile["noise_std_rad"] * self.factor @ self.rng.standard_normal((len(q)-2, 7))
        noisy = np.clip(q+noise, self.model.lower, self.model.upper)
        noisy[0], noisy[-1] = self.initial[0], self.initial[-1]
        noise = noisy-q
        self.noise_records.append((iteration, rollout, noise.copy()))
        return noisy, noise

    def cost(self, q, iteration, rollout):
        distance = self.clearance(q)
        potential, _ = obstacle_potential(distance, self.profile["obstacle_influence_m"])
        costs = potential / self.profile["obstacle_influence_m"]
        valid = bool(np.all(distance > self.profile["safety_band_m"]))
        if rollout == -1 and valid:
            valid, minimum = self.polyline_validity(q)
        else:
            minimum = float(distance.min())
        self.cost_records.append(dict(iteration=iteration, rollout=rollout,
                                      state_cost=float(costs.sum()), validity=valid,
                                      minimum_clearance_m=minimum))
        return costs, valid

    def update(self, q, delta, iteration):
        self.check_deadline()
        update = self.smoothing @ delta
        update[[0, -1]] = 0.
        update /= max(1., np.abs(update).max()/self.profile["maximum_update_rad"])
        candidate = np.clip(q+update, self.model.lower, self.model.upper)
        candidate[0], candidate[-1] = self.initial[0], self.initial[-1]
        return candidate-q

    def post(self, iteration, cost, q):
        self.history.append(dict(iteration=iteration, native_cost=cost))
        self.trajectories.append(q.copy())
        with (self.out / "stomp_iterations.jsonl").open("a") as stream:
            stream.write(json.dumps(self.history[-1], allow_nan=False)+"\n")
        np.save(self.out / "optimizer_latest.npy", q)

    def done(self, valid, iterations, cost, q):
        self.final_record = dict(solver_success=bool(valid), iterations=int(iterations),
                                 native_cost=float(cost), stop_reason="native_STOMP_return")

    def save(self):
        np.savez_compressed(self.out / "stomp_raw.npz", trajectories=np.asarray(self.trajectories),
                            noise=np.asarray([r[2] for r in self.noise_records]),
                            noise_iteration_rollout=np.asarray([r[:2] for r in self.noise_records], int))
        write(self.out / "stomp_costs.json", self.cost_records)


def _profile(method, cfg):
    defaults = CHOMP_PROFILE if method == "CHOMP" else STOMP_PROFILE
    result = dict(defaults)
    overrides = cfg.get("extended_baselines", {}).get(method.lower(), {})
    if not isinstance(overrides, dict):
        raise ValueError(f'{method} settings must be an object')
    unknown = set(overrides)-set(defaults)
    if unknown:
        raise ValueError(f"Unknown {method} settings: {sorted(unknown)}")
    result.update(overrides)
    for key, value in result.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
            raise ValueError(f"Invalid numeric {method} setting {key}")
        if value < 0 or (value == 0 and key != "iterations_after_valid"):
            raise ValueError(f"Nonpositive {method} setting {key}")
        if isinstance(defaults[key], int) and int(value) != value:
            raise ValueError(f"Expected integer {method} setting {key}")
        if isinstance(defaults[key], int):
            result[key] = int(value)
    if result["initial_knots"] < 8 or result["maximum_knots"] < result["initial_knots"]:
        raise ValueError("Invalid trajectory knot bounds")
    if method == "STOMP" and result["rollouts"] < 2:
        raise ValueError("STOMP requires at least two rollouts")
    return result


def solve(path, model, case, cfg, out, deadline, seed, method):
    if method not in ("CHOMP", "STOMP"):
        raise ValueError("Expected CHOMP or STOMP")
    if case.get("_objective", "minimum_distance") != "minimum_distance":
        raise ValueError("CHOMP/STOMP expose PWL distance evaluation only")
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    profile = _profile(method, cfg)
    initial = initial_trajectory(path, profile["initial_knots"], profile["maximum_knots"])
    if np.any(initial < model.lower-1e-10) or np.any(initial > model.upper+1e-10):
        raise ValueError("Seed violates canonical joint limits")
    record = dict(method=method, profile=profile, seed=int(seed),
                   initializer="shared validated DRM path, all vertices retained",
                   native_objective=("CHOMP: squared joint accelerations plus canonical signed-clearance potential integrated over C-space arc length"
                                     if method == "CHOMP" else "STOMP: official normalized acceleration control cost plus summed canonical signed-clearance waypoint potential"),
                   reported_benchmark_cost="Euclidean seven-joint PWL arc length in radians",
                   collision_model="unchanged Model.clearance; exact canonical primitives and filter pairs",
                   algorithm=("CHOMP fixed-endpoint acceleration-metric covariant descent; C-space line-integral potential"
                              if method == "CHOMP" else "unmodified ros-industrial STOMP core"),
                   sources=(["https://publications.ri.cmu.edu/storage/publications/pub_files/2013/5/CHOMP_IJRR.pdf"]
                            if method == "CHOMP" else [f"https://github.com/ros-industrial/stomp/tree/{STOMP_COMMIT}"]))
    write(out / "optimizer_parameters.json", record)
    np.savez_compressed(out / "optimizer_initial.npz", original_drm_path=np.asarray(path), initial=initial)
    started = time.perf_counter()
    task = GeometryTask(model, profile, deadline)
    try:
        if method == "CHOMP":
            result, status = _chomp(initial, task, out, profile)
        else:
            native, dependencies = _backend()
            write(out / "stomp_dependencies.json", dependencies)
            task = StompTask(model, profile, deadline, initial, out, seed, native)
            try:
                valid, result = native.solve(initial, task, profile["maximum_iterations"],
                    profile["iterations_after_valid"], profile["rollouts"],
                    profile["cost_sensitivity"], profile["control_cost_weight"])
            finally:
                task.save()
            status = task.final_record or dict(solver_success=False, stop_reason="native_return_without_done")
            status["solver_success"] = bool(valid and status["solver_success"])
        result = np.asarray(result, float)
        if result.shape != initial.shape or not np.isfinite(result).all():
            raise ValueError("Malformed native optimization result")
        if not np.array_equal(result[[0, -1]], initial[[0, -1]]):
            raise ValueError("Optimizer changed fixed endpoints")
        status.update(seconds=time.perf_counter()-started,
                       clearance_calls=task.clearance_calls, clearance_states=task.clearance_states,
                       geometry_seconds=task.geometry_seconds,
                       analytic_gradient_calls=task.analytic_gradient_calls,
                       candidate_changed=bool(np.max(np.abs(result-initial)) > 1e-10),
                       initial_pwl_length_rad=float(np.linalg.norm(np.diff(initial, axis=0), axis=1).sum()),
                       final_pwl_length_rad=float(np.linalg.norm(np.diff(result, axis=0), axis=1).sum()),
                       finite_sampling_is_not_continuous_certificate=True)
        write(out / "optimizer_result.json", status)
        np.savez_compressed(out / "optimizer_solution.npz", points=result)
        return dict(kind="polygon", objective="minimum_distance", points=result,
                    solver_success=status["solver_success"], native_cost=status.get("native_cost"),
                    native_objective=record["native_objective"],
                    optimizer_status=status, optimizer_provenance=record)
    except Exception as exc:
        write(out / "optimizer_failure.json", dict(method=method, exception=repr(exc),
            traceback=traceback.format_exc(), seconds=time.perf_counter()-started,
            clearance_calls=task.clearance_calls, clearance_states=task.clearance_states,
            analytic_gradient_calls=task.analytic_gradient_calls,
            geometry_seconds=task.geometry_seconds))
        raise


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["build", "probe"])
    args = parser.parse_args()
    print(json.dumps(build_stomp() if args.action == "build" else dependency_probe(), indent=2))
