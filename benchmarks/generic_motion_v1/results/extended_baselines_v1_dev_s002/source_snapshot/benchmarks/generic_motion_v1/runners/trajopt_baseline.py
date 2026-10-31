"""Author TrajOpt trust-region SQP with a canonical-model collision adapter.

The native SQP, merit updates, convexification machinery and bundled BPMPD QP
backend are unmodified joschu/trajopt sources. This file defines the local
seven-joint problem and collision collocation policy. Continuous swept-volume
constraints from the OpenRAVE integration are not used or claimed here.
"""
from __future__ import annotations

import argparse
import ctypes
import difflib
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import time
import traceback

import numpy as np

from ..roadmap import write

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
ARCHIVE = HERE / "reports/extended_baselines_v1/trajopt_author_core.tar.gz"
BRIDGE = HERE / "native/trajopt_bridge.cpp"
SAFE_IO = HERE / "native/trajopt_safe_io.hpp"
DEFAULT_BUILD = ROOT / ".local_setup/solver_probes/trajopt_native_v2"
AUTHOR_COMMIT = "9c634c060f7e2d61c4a888b0a5700d668a5d16f8"
PROFILE = dict(
    name="TrajOpt_author_SCO_canonical_collocation_v1",
    author_repository="https://github.com/joschu/trajopt",
    author_commit=AUTHOR_COMMIT,
    algorithm="author BasicTrustRegionSQP; exact quadratic joint velocity cost; linearized signed-distance inequalities; l1 hinge merit; trust region; penalty increases",
    qp_solver="author bundled BPMPD 2.21",
    implementation_scope="Unmodified author optimizer core with local canonical geometry callbacks and sampled segment constraints",
    objective="sum_i ||q[i+1]-q[i]||_2^2",
    evaluation_objective="minimum_distance: sum_i ||q[i+1]-q[i]||_2",
    initial_knots=64,
    maximum_knots=256,
    segment_collocation_subdivisions=3,
    clearance_m=0.001,
    derivative_step_rad=1e-5,
    maximum_sqp_iterations=50,
    initial_trust_box_rad=0.1,
    constraint_tolerance=1e-4,
    initial_merit_coefficient=10.0,
    maximum_merit_coefficient_increases=5,
    minimum_model_improvement=1e-4,
    smoothness_weight=1.0,
    maximum_optimizer_seconds=45.0,
    unchanged_author_parameters=dict(improve_ratio_threshold=0.25,
        minimum_trust_box_size=1e-4, trust_shrink_ratio=0.1,
        trust_expand_ratio=1.5, merit_coefficient_increase_ratio=10),
    final_independent_validation_required=True,
    continuous_collision_certificate=False,
)


def settings(cfg):
    """Strict public profile overrides; source/method claims are immutable."""
    values = dict(PROFILE)
    allowed = {"initial_knots", "maximum_knots", "segment_collocation_subdivisions",
               "clearance_m", "derivative_step_rad", "maximum_sqp_iterations",
               "initial_trust_box_rad", "constraint_tolerance", "initial_merit_coefficient",
               "maximum_merit_coefficient_increases", "minimum_model_improvement",
               "maximum_optimizer_seconds", "build_dir"}
    overrides = cfg.get("extended_baselines", {}).get("trajopt", {})
    if not isinstance(overrides, dict):
        raise ValueError("extended_baselines.trajopt must be an object")
    unknown = set(overrides)-allowed
    if unknown:
        raise ValueError("Unknown TrajOpt parameters: " + ", ".join(sorted(unknown)))
    values.update(overrides)
    integers = {"initial_knots": (3, 256), "maximum_knots": (3, 256),
                "segment_collocation_subdivisions": (1, 16),
                "maximum_sqp_iterations": (1, 1000),
                "maximum_merit_coefficient_increases": (1, 20)}
    for key, (lower, upper) in integers.items():
        value = values[key]
        if isinstance(value, bool) or not isinstance(value, int) or not lower <= value <= upper:
            raise ValueError(f"TrajOpt {key} must be an integer in [{lower}, {upper}]")
    if values["initial_knots"] > values["maximum_knots"]:
        raise ValueError("TrajOpt initial_knots exceeds maximum_knots")
    for key in allowed-integers.keys()-{"build_dir"}:
        value = values[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
            raise ValueError(f"TrajOpt {key} must be finite and positive")
    if "build_dir" in values and not isinstance(values["build_dir"], (str, Path)):
        raise ValueError("TrajOpt build_dir must be a path string")
    return values


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build_native(build_dir=DEFAULT_BUILD):
    """Build isolated author code; never alter the installed Conda runtime."""
    build_dir = Path(build_dir).resolve()
    fingerprint = dict(author_commit=AUTHOR_COMMIT, archive_sha256=_sha(ARCHIVE),
                       bridge_sha256=_sha(BRIDGE), safe_io_sha256=_sha(SAFE_IO))
    receipt_path = build_dir / "build.json"
    if receipt_path.exists():
        receipt = json.loads(receipt_path.read_text())
        if receipt.get("fingerprint") == fingerprint and all(
                (build_dir / name).exists() and _sha(build_dir / name) == digest
                for name, digest in receipt.get("binaries", {}).items()):
            return receipt
        raise RuntimeError("TrajOpt build fingerprint changed: select a fresh build directory")
    build_dir.mkdir(parents=True, exist_ok=True)
    source = build_dir / "source"
    source.mkdir(exist_ok=True)
    with tarfile.open(ARCHIVE) as archive:
        for item in archive.getmembers():
            target = (source / item.name).resolve()
            if source.resolve() not in target.parents or item.issym() or item.islnk():
                raise ValueError("Unsafe author archive member")
        archive.extractall(source)
    # Retain pristine author archive; apply only POSIX transport repairs in the
    # isolated extracted build copy. Keep the exact patch as build evidence.
    patches = []
    for name in ("bpmpd_io.hpp", "bpmpd_interface.cpp"):
        file = source / "src/sco" / name
        original = file.read_text()
        if name == "bpmpd_io.hpp":
            changed = '#include "trajopt_safe_io.hpp"\n'+original
            changed = changed.replace("write(fp,", "trajopt_io::write_full(fp,")
            changed = changed.replace("read(fp,", "trajopt_io::read_full(fp,")
        else:
            anchor = "  if (infp == NULL)"
            if original.count(anchor) != 1:
                raise RuntimeError("Unexpected author BPMPD transport source")
            changed = original.replace(anchor,
                "  close(p_stdin[READ]);\n  close(p_stdout[WRITE]);\n\n"+anchor)
        patches.extend(difflib.unified_diff(original.splitlines(True), changed.splitlines(True),
                                           fromfile="author/"+name, tofile="build/"+name))
        file.write_text(changed)
    (build_dir / "transport.patch").write_text("".join(patches))
    includes = ["-I" + str(source / "src"), "-I/usr/include/eigen3", "-I"+str(SAFE_IO.parent)]
    flags = ["g++", "-std=c++11", "-O2"]
    commands = []
    caller = build_dir / "bpmpd_caller"
    commands.append(flags + ["-no-pie"] + includes + [
        '-DBPMPD_WORKING_DIR=getenv("TRAJOPT_BPMPD_LOG_DIR")',
        str(source / "src/sco/bpmpd_caller.cpp"),
        str(source / "3rdpartylib/bpmpd_linux64.a"), "-o", str(caller)])
    sources = [source / "src/sco" / (name + ".cpp") for name in (
        "solver_interface", "modeling", "expr_ops", "expr_vec_ops",
        "optimizers", "modeling_utils", "num_diff", "bpmpd_interface")]
    sources += [source / "src/utils" / (name + ".cpp") for name in ("logging", "stl_to_string")]
    commands.append(flags + ["-shared", "-fPIC"] + includes + ["-DHAVE_BPMPD",
        '-DBPMPD_CALLER="' + str(caller) + '"', *map(str, sources), str(BRIDGE),
        "-o", str(build_dir / "libtrajopt_benchmark.so")])
    receipt = dict(fingerprint=fingerprint, commands=commands, status="building")
    receipt.update(optimizer_core_modified=False, transport_only_patch_sha256=_sha(build_dir/"transport.patch"))
    write(build_dir / "build_attempt.json", receipt)
    with (build_dir / "build.log").open("w") as log:
        for command in commands:
            completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                       check=False)
            if completed.returncode:
                receipt.update(status="failed", returncode=completed.returncode)
                write(build_dir / "build_attempt.json", receipt)
                raise RuntimeError("Native TrajOpt build failed; inspect " + str(build_dir / "build.log"))
    receipt.update(status="ready", binaries={name: _sha(build_dir / name)
        for name in ("bpmpd_caller", "libtrajopt_benchmark.so")})
    write(receipt_path, receipt)
    return receipt


def dependency_probe(build_dir=DEFAULT_BUILD):
    build_dir = Path(build_dir).resolve()
    receipt_path = build_dir / "build.json"
    result = dict(algorithm="TrajOpt", solver="author BPMPD 2.21",
                  author_commit=AUTHOR_COMMIT, build_directory=str(build_dir),
                  available=False, status="not_built")
    if receipt_path.exists():
        receipt = json.loads(receipt_path.read_text())
        expected = dict(author_commit=AUTHOR_COMMIT, archive_sha256=_sha(ARCHIVE),
                        bridge_sha256=_sha(BRIDGE), safe_io_sha256=_sha(SAFE_IO))
        result.update(build_receipt=receipt, build_receipt_sha256=_sha(receipt_path))
        result["available"] = (receipt.get("fingerprint") == expected
            and bool(receipt.get("binaries")) and all(
                (build_dir / name).exists() and _sha(build_dir / name) == digest
                for name, digest in receipt.get("binaries", {}).items()))
        result["status"] = "ready" if result["available"] else "build_fingerprint_mismatch"
    return result


def subdivide_preserving_vertices(path, minimum_waypoints):
    """Add midpoints, keeping every original DRM vertex and path segment."""
    points = np.asarray(path, float).copy()
    if points.ndim != 2 or points.shape[1] != 7 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError("TrajOpt initialization must be finite N x 7 with at least two states")
    if not isinstance(minimum_waypoints, int) or minimum_waypoints < 2:
        raise ValueError("minimum_waypoints must be an integer at least two")
    while len(points) < minimum_waypoints:
        index = int(np.argmax(np.linalg.norm(np.diff(points, axis=0), axis=1)))
        points = np.insert(points, index+1, (points[index]+points[index+1])/2, axis=0)
    return points


def collocation_matrix(waypoints, subdivisions):
    if waypoints < 2 or subdivisions < 1 or int(subdivisions) != subdivisions:
        raise ValueError("Invalid segment collocation dimensions")
    matrix = np.zeros(((waypoints-1)*subdivisions+1, waypoints))
    for segment in range(waypoints-1):
        for k in range(subdivisions):
            alpha = k/subdivisions
            matrix[segment*subdivisions+k, segment:segment+2] = [1-alpha, alpha]
    matrix[-1, -1] = 1
    return matrix


def signed_distance_jacobian(model, q, step, deadline):
    """Analytic canonical primitive derivative; finite differences for fixtures."""
    if hasattr(model, "_robot_geometries"):
        from .canonical_distance import clearance_gradient
        return clearance_gradient(model, q, deadline, finite_difference_step=step)
    q = np.asarray(q, float)
    values = np.asarray(model.clearance(q), float)
    gradient = np.empty_like(q)
    for index, state in enumerate(q):
        if time.perf_counter() >= deadline:
            raise TimeoutError("TrajOpt derivative evaluation exhausted shared budget")
        perturbation = np.eye(7)*step
        samples = np.r_[state[None, :]+perturbation, state[None, :]-perturbation]
        distances = np.asarray(model.clearance(samples), float)
        gradient[index] = (distances[:7]-distances[7:])/(2*step)
    if not np.isfinite(values).all() or not np.isfinite(gradient).all():
        raise ValueError("Nonfinite canonical collision clearance or derivative")
    return values, gradient


def solve(path, model, case, cfg, out, deadline, seed=0):
    """Run author SQP on a fixed DRM initialization and return its own iterate."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    profile = settings(cfg)
    build_dir = Path(profile.get("build_dir", DEFAULT_BUILD)).resolve()
    dependency = dependency_probe(build_dir)
    if not dependency["available"]:
        write(out / "trajopt_dependency.json", dependency)
        raise RuntimeError("TrajOpt author core is unavailable: " + dependency["status"])
    if not np.isfinite(deadline) or deadline <= started:
        raise TimeoutError("TrajOpt shared budget exhausted before solve")
    from .chomp_stomp import initial_trajectory
    points = initial_trajectory(path, profile["initial_knots"], profile["maximum_knots"])
    outer_deadline = float(deadline)
    deadline = min(deadline, started+profile["maximum_optimizer_seconds"])
    interpolation = collocation_matrix(len(points), profile["segment_collocation_subdivisions"])
    if np.any(points < np.asarray(model.lower)-1e-7) or np.any(points > np.asarray(model.upper)+1e-7):
        raise ValueError("TrajOpt initial DRM path violates frozen joint limits")
    # The outer runner owns endpoint normalization. Check it instead of changing
    # any task endpoint inside an algorithm adapter.
    endpoint_tolerance = cfg.get("endpoint_tolerance_rad", 1e-5)
    if (isinstance(endpoint_tolerance, bool) or not isinstance(endpoint_tolerance, (int, float))
            or not np.isfinite(endpoint_tolerance) or endpoint_tolerance <= 0):
        raise ValueError("TrajOpt requires a finite positive common endpoint tolerance")
    endpoint_errors = {}
    for name, point in (("q_start", points[0]), ("q_goal", points[-1])):
        if name in case:
            endpoint_errors[name] = float(np.max(np.abs(point-np.asarray(case[name], float))))
        if name in case and not np.allclose(point, case[name], atol=endpoint_tolerance, rtol=0):
            raise ValueError("TrajOpt received an altered task endpoint")
    np.savez_compressed(out / "trajopt_initial.npz", drm_path=np.asarray(path),
                        controls=points, collocation_matrix=interpolation)
    write(out / "trajopt_parameters.json", dict(profile=profile, dependency=dependency,
        seed=seed, randomness="deterministic given initialization", initial_remaining_seconds=deadline-started,
        endpoint_error_rad=endpoint_errors, endpoint_tolerance_rad=endpoint_tolerance,
        endpoint_policy="preserve supplied DRM endpoints; compare against immutable task under common numeric tolerance",
        collision_feature="minimum signed distance over all canonical static pairs and online spheres",
        constraint_sampling="fixed equally spaced points on every current PWL segment",
        derivative="analytic active primitive feature; central differences at feature ties/degeneracies"))
    trajectory_dir = out / "trajopt_iterates"
    trajectory_dir.mkdir(exist_ok=True)
    # Author BPMPD maintains one persistent subprocess per worker. Its scratch
    # log is copied after every objective evaluation; no historical run is used
    # as writable solver scratch space.
    scratch = build_dir / "scratch" / str(os.getpid())
    scratch.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(build_dir / "source/src/sco/bpmpd.par", scratch / "bpmpd.par")
    os.environ["TRAJOPT_BPMPD_LOG_DIR"] = str(scratch)
    os.environ["TRAJOPT_BPMPD_DEADLINE_MONOTONIC"] = str(deadline)
    os.environ.setdefault("TRAJOPT_LOG_THRESH", "INFO")
    library = ctypes.CDLL(str(build_dir / "libtrajopt_benchmark.so"))
    pointer = ctypes.POINTER(ctypes.c_double)
    callback_type = ctypes.CFUNCTYPE(ctypes.c_int, pointer, ctypes.c_int,
        ctypes.c_int, pointer, pointer)
    iteration_type = ctypes.CFUNCTYPE(ctypes.c_int, pointer, ctypes.c_int, ctypes.c_int)
    failures = []
    counter = {"evaluations": 0, "derivatives": 0, "iterations": 0}
    cached = {}
    rows = []
    optimized_iterates = []

    def check_deadline():
        if time.perf_counter() >= deadline:
            raise TimeoutError("TrajOpt exhausted shared online deadline")

    def evaluate(values_pointer, dimension, want_gradient, distances_pointer, jacobian_pointer):
        try:
            check_deadline()
            controls = np.ctypeslib.as_array(values_pointer, shape=(dimension,)).reshape(-1, 7).copy()
            q = interpolation @ controls
            key = controls.tobytes()
            if cached.get("key") == key and (not want_gradient or cached.get("gradient") is not None):
                values, gradient = cached["values"], cached.get("gradient")
            else:
                if want_gradient:
                    values, gradient = signed_distance_jacobian(model, q, profile["derivative_step_rad"], deadline)
                else:
                    values, gradient = np.asarray(model.clearance(q), float), None
                cached.update(key=key, values=values, gradient=gradient)
            if not np.isfinite(values).all():
                raise ValueError("Nonfinite TrajOpt collision value")
            np.ctypeslib.as_array(distances_pointer, shape=values.shape)[:] = values
            if want_gradient:
                jacobian = interpolation[:, :, None]*gradient[:, None, :]
                np.ctypeslib.as_array(jacobian_pointer, shape=(len(q)*dimension,))[:] = jacobian.ravel()
                counter["derivatives"] += 1
            index = counter["evaluations"]
            row = dict(evaluation=index, derivative=bool(want_gradient),
                       minimum_clearance_m=float(values.min()),
                       quadratic_smoothness=float(np.square(np.diff(controls, axis=0)).sum()),
                       hinge_violation=float(np.maximum(profile["clearance_m"]-values, 0).sum()),
                       elapsed_seconds=time.perf_counter()-started)
            rows.append(row)
            with (out / "trajopt_evaluations.jsonl").open("a") as stream:
                stream.write(json.dumps(row, allow_nan=False)+"\n")
            np.savez_compressed(trajectory_dir / f"evaluation_{index:04d}.npz",
                                controls=controls, collocation_clearance=values)
            if (scratch / "bpmpd.log").exists():
                shutil.copyfile(scratch / "bpmpd.log", trajectory_dir / f"qp_{index:04d}.log")
            counter["evaluations"] += 1
            check_deadline()
            return 0
        except BaseException as exc:
            failures.append((exc, traceback.format_exc()))
            return 1

    def iterate(values_pointer, dimension, iteration):
        try:
            check_deadline()
            controls = np.ctypeslib.as_array(values_pointer, shape=(dimension,)).reshape(-1, 7).copy()
            np.save(trajectory_dir / f"accepted_{iteration:04d}.npy", controls)
            counter["iterations"] = iteration+1
            if iteration > 0 and np.max(np.abs(controls-points)) > 1e-8:
                optimized_iterates.append(controls.copy())
            return 0
        except BaseException as exc:
            failures.append((exc, traceback.format_exc()))
            return 1

    distance_callback, iteration_callback = callback_type(evaluate), iteration_type(iterate)
    lower = np.tile(model.lower, (len(points), 1)).astype(float)
    upper = np.tile(model.upper, (len(points), 1)).astype(float)
    lower[[0, -1]], upper[[0, -1]] = points[[0, -1]], points[[0, -1]]
    native_settings = np.array([profile["clearance_m"], profile["maximum_sqp_iterations"],
        profile["initial_trust_box_rad"], profile["constraint_tolerance"],
        profile["initial_merit_coefficient"], profile["maximum_merit_coefficient_increases"],
        profile["minimum_model_improvement"]], dtype=float)
    solution, statistics = np.full_like(points, np.nan), np.full(8, np.nan)
    error = ctypes.create_string_buffer(4096)
    native = library.trajopt_solve
    native.argtypes = [ctypes.c_int, pointer, pointer, pointer, ctypes.c_int,
        callback_type, iteration_type, pointer, pointer, pointer, ctypes.c_char_p, ctypes.c_int]
    native.restype = ctypes.c_int
    args = lambda array: np.ascontiguousarray(array, dtype=np.float64).ctypes.data_as(pointer)
    try:
        code = native(points.size, args(points), args(lower), args(upper), len(interpolation),
            distance_callback, iteration_callback, args(native_settings), args(solution), args(statistics), error, len(error))
        if failures:
            raise failures[0][0]
        if code:
            if "transport deadline" in error.value.decode(errors="replace"):
                raise TimeoutError(error.value.decode(errors="replace"))
            raise RuntimeError("Author TrajOpt core: " + error.value.decode(errors="replace"))
        if not np.isfinite(solution).all() or not np.isfinite(statistics).all():
            raise ValueError("Nonfinite native TrajOpt output")
        status = ["CONVERGED", "SCO_ITERATION_LIMIT", "PENALTY_ITERATION_LIMIT", "FAILED", "INVALID"][int(statistics[0])]
        check_deadline()
        changed_incumbent = bool(np.max(np.abs(solution-points)) > 1e-8 and statistics[3] > 0)
        usable = (status == "CONVERGED" or (status in (
            "SCO_ITERATION_LIMIT", "PENALTY_ITERATION_LIMIT") and changed_incumbent))
        result = dict(solver="author TrajOpt BasicTrustRegionSQP / BPMPD 2.21",
            solver_success=usable, solver_status=status, native_status=status,
            solver_converged=status == "CONVERGED", termination=status,
            returned_accepted_iterate=changed_incumbent,
            quadratic_smoothness=float(np.square(np.diff(solution, axis=0)).sum()),
            path_length_rad=float(np.linalg.norm(np.diff(solution, axis=0), axis=1).sum()),
            native_statistics=statistics.tolist(), counters=counter,
            elapsed_seconds=time.perf_counter()-started, final_independent_validation_required=True)
        write(out / "trajopt_solver_result.json", result)
        np.savez_compressed(out / "trajopt_solver_raw.npz", points=solution, statistics=statistics)
        return dict(kind="polygon", objective="minimum_distance", points=solution,
                    solver=result["solver"], solver_success=result["solver_success"],
                    solver_status=status, native_status=status, termination=status,
                    solver_converged=result["solver_converged"],
                    solver_objective=result["quadratic_smoothness"],
                    native_profile=profile["name"],
                    native_objective=profile["objective"],
                    cost=result["path_length_rad"])
    except BaseException as exc:
        failure = dict(solver_success=False,
            solver_status="timeout" if isinstance(exc, TimeoutError) else "exception",
            exception=repr(exc), traceback=traceback.format_exc(),
            native_error=error.value.decode(errors="replace"), counters=counter,
            elapsed_seconds=time.perf_counter()-started)
        # The optimizer's internal cap leaves the outer validator its own time.
        # Publish only a genuinely accepted, changed SQP iterate. The original
        # DRM initialization is never a fallback candidate for an interrupted
        # optimizer, and trial QP solutions that SQP rejected are not eligible.
        if (isinstance(exc, TimeoutError) and optimized_iterates
                and deadline < outer_deadline and time.perf_counter() < outer_deadline):
            solution = optimized_iterates[-1]
            status = "INTERNAL_TIME_LIMIT_WITH_OPTIMIZED_ITERATE"
            quadratic = float(np.square(np.diff(solution, axis=0)).sum())
            length = float(np.linalg.norm(np.diff(solution, axis=0), axis=1).sum())
            failure.update(solver_success=True, solver_converged=False,
                solver_status=status, native_status="INTERRUPTED_BY_SHARED_CALLBACK_DEADLINE",
                termination="INTERNAL_TIME_LIMIT", returned_accepted_iterate=True,
                quadratic_smoothness=quadratic, path_length_rad=length,
                final_independent_validation_required=True)
            write(out / "trajopt_solver_result.json", failure)
            np.savez_compressed(out / "trajopt_solver_raw.npz", points=solution,
                                interrupted_native_statistics=statistics)
            return dict(kind="polygon", objective="minimum_distance", points=solution,
                solver="author TrajOpt BasicTrustRegionSQP / BPMPD 2.21",
                solver_success=True, solver_converged=False, solver_status=status,
                native_status="INTERRUPTED_BY_SHARED_CALLBACK_DEADLINE", termination="INTERNAL_TIME_LIMIT",
                solver_objective=quadratic, native_objective=profile["objective"],
                native_profile=profile["name"], cost=length)
        write(out / "trajopt_solver_result.json", failure)
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD)
    arguments = parser.parse_args()
    print(json.dumps(build_native(arguments.build_dir) if arguments.build
                     else dependency_probe(arguments.build_dir), indent=2))
