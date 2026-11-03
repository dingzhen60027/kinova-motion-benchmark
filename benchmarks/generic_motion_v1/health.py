"""Real environment, collision, solver and timeout checks; no planning benchmark."""

from __future__ import annotations

import argparse
from contextlib import redirect_stderr, redirect_stdout
import datetime
import fcntl
from importlib import metadata
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
import traceback

import numpy as np

from .common import BASE, ROOT, head, read, sha, write
from .dataset import load_index, resolve_problem, source_records


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def timeout_probe(log_path, timeout=0.15):
    """Run and terminate an actual sleeping process group, never a planner.

    The diagnostic tests subprocess deadline enforcement. It does not claim a
    planning timeout, nor does it contribute any benchmark run or success.
    """
    if not np.isfinite(timeout) or timeout <= 0:
        raise ValueError("Timeout must be positive and finite")
    command = [
        sys.executable,
        "-u",
        "-c",
        "import time; print('timeout diagnostic child started', flush=True); time.sleep(30)",
    ]
    started = time.perf_counter()
    timed_out = False
    with Path(log_path).open("w") as log:
        process = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
        )
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    return dict(
        passed=timed_out and process.returncode is not None,
        status="timeout" if timed_out else "unexpected_completion",
        command=command,
        timeout_seconds=timeout,
        elapsed_seconds=time.perf_counter() - started,
        exit_code=process.returncode,
        diagnostic_only=True,
        log_sha256=sha(log_path),
    )


def _environment():
    import csdecomp
    import pydrake

    # Match the public launcher/CLI: a relocated checkout may intentionally
    # reuse the already installed environment through this override.
    expected = Path(os.environ.get("CSDECOMP_CONDA_PREFIX", ROOT / ".conda/envs/csdecomp")).expanduser().resolve()
    versions = {}
    for name in (
        "numpy",
        "scipy",
        "drake",
        "clarabel",
        "cvxpy",
        "scsplanning",
        "pybezier",
    ):
        try:
            versions[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            versions[name] = None
    nvidia = subprocess.run(
        [
            "nvidia-smi",
            "--query-gpu=name,driver_version,memory.total",
            "--format=csv,noheader",
        ],
        capture_output=True,
        text=True,
        timeout=10,
    )
    print(nvidia.stdout, end="")
    if nvidia.stderr:
        print(nvidia.stderr, file=sys.stderr, end="")
    return dict(
        passed=Path(sys.prefix).resolve() == expected.resolve()
        and nvidia.returncode == 0,
        python=sys.version,
        executable=sys.executable,
        prefix=sys.prefix,
        expected_conda_prefix=str(expected),
        numpy_file=np.__file__,
        csdecomp_file=csdecomp.__file__,
        drake_file=pydrake.__file__,
        versions=versions,
        platform=platform.platform(),
        operating_system=Path("/etc/os-release").read_text(),
        gpu=nvidia.stdout.strip(),
        nvidia_exit_code=nvidia.returncode,
        thread_environment={
            key: os.environ.get(key)
            for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")
        },
    )


def _solver_checks():
    from pydrake.solvers import ClarabelSolver, MathematicalProgram

    solver = ClarabelSolver()
    feasible = MathematicalProgram()
    x = feasible.NewContinuousVariables(1, "x")
    feasible.AddQuadraticCost((x[0] - 1) ** 2)
    feasible.AddBoundingBoxConstraint(0, 2, x)
    start = time.perf_counter()
    solution = solver.Solve(feasible)
    feasible_seconds = time.perf_counter() - start
    value = float(solution.GetSolution(x)[0])
    impossible = MathematicalProgram()
    y = impossible.NewContinuousVariables(1, "y")
    impossible.AddLinearConstraint(y[0] <= 0)
    impossible.AddLinearConstraint(y[0] >= 1)
    start = time.perf_counter()
    rejected = solver.Solve(impossible)
    return dict(
        passed=solution.is_success()
        and abs(value - 1) < 1e-4
        and not rejected.is_success(),
        solver="Clarabel",
        available=solver.available(),
        enabled=solver.enabled(),
        feasible=dict(
            solver_success=solution.is_success(),
            x=value,
            target=1.0,
            status=str(solution.get_solution_result()),
            seconds=feasible_seconds,
        ),
        infeasible=dict(
            solver_success=rejected.is_success(),
            status=str(rejected.get_solution_result()),
            seconds=time.perf_counter() - start,
            diagnostic_constraints=["y <= 0", "y >= 1"],
        ),
        scope="Actual convex solver execution and infeasibility detection; end-to-end LSCS/SCSTO tested separately by smoke planning",
    )


def _load_candidate(attempt):
    candidate = read(attempt / "candidate.json")
    with np.load(attempt / "candidate.npz", allow_pickle=False) as arrays:
        candidate.update({key: arrays[key] for key in arrays.files})
    return candidate


def _collision_checks():
    from .model import Model
    from .runners.validate import validate

    records = [
        row
        for row, _, _ in source_records()
        if row["split"] == "dev" and row["validation_status"] == "valid_input"
    ]
    short = min(
        (row for row in records if row["straight_line_collision"] is False),
        key=lambda row: row["start_goal_joint_distance"],
    )
    model = Model(short["voxels_xyz"], short["voxel_radius_m"])
    case_hash_match = all(
        getattr(model, key) == short[key]
        for key in ("robot_sha256", "collision_geometry_sha256")
    )
    q = np.asarray([short["q_start"], short["q_goal"]])
    started = time.perf_counter()
    native = model.cuda(q)
    # Native API returns a copied host boolean array: completion of the CUDA
    # collision query is observed before this wall time ends.
    native_seconds = time.perf_counter() - started
    clearance = model.clearance(q)
    straight = dict(kind="polygon", points=q, objective="minimum_distance")
    short_validation = validate(straight, model, short)
    positive = dict(
        passed=case_hash_match
        and native.all()
        and np.all(clearance > model.contact_band_m)
        and short_validation["passed"],
        problem_id=short["problem_id"],
        problem_sha256=short["problem_sha256"],
        q_start=q[0],
        q_goal=q[1],
        cuda_free=native,
        drake_clearance_m=clearance,
        cuda_call_seconds=native_seconds,
        validation=short_validation,
    )

    # Use an actual saved rejected sample; no endpoint in the dataset changes.
    with np.load(BASE / short["input_arrays_file"], allow_pickle=False) as arrays:
        rejected = np.flatnonzero(
            (~arrays["endpoint_proposal_cuda_free"])
            & (
                arrays["endpoint_proposal_independent_clearance_m"]
                < -model.contact_band_m
            )
        )
        if not len(rejected):
            raise RuntimeError(
                "Chosen short-path input has no saved definite collision for negative control"
            )
        colliding = arrays["endpoint_proposal_q"][rejected[0]].copy()
    collision_case = dict(short, q_start=colliding.tolist())
    collision_candidate = dict(
        kind="polygon",
        points=np.array([colliding, short["q_goal"]]),
        objective="minimum_distance",
    )
    colliding_native = model.cuda(colliding)
    colliding_clearance = model.clearance(colliding)
    collision_validation = validate(collision_candidate, model, collision_case)
    negative_collision = dict(
        passed=not colliding_native.any()
        and np.all(colliding_clearance < -model.contact_band_m)
        and not collision_validation["passed"],
        diagnostic_only=True,
        original_problem_id=short["problem_id"],
        q_start=colliding,
        cuda_free=colliding_native,
        drake_clearance_m=colliding_clearance,
        validation=collision_validation,
    )

    outside = np.asarray(short["q_start"]).copy()
    outside[0] = model.upper[0] + 0.1
    limit_case = dict(short, q_start=outside.tolist())
    limit_validation = validate(
        dict(
            kind="polygon",
            points=np.array([outside, short["q_goal"]]),
            objective="minimum_distance",
        ),
        model,
        limit_case,
    )
    negative_limit = dict(
        passed=not limit_validation["passed"]
        and not limit_validation["joint_limits_passed"],
        diagnostic_only=True,
        original_problem_id=short["problem_id"],
        q_start=outside,
        violated_joint=0,
        upper_limit_rad=model.upper[0],
        exceedance_rad=float(outside[0] - model.upper[0]),
        validation=limit_validation,
    )

    reference = next(
        row
        for row in load_index()
        if row["split"] == "dev"
        and row["straight_line_collision"] is True
        and row.get("witness")
    )
    detour = resolve_problem(reference["problem_id"])
    model.set_voxels(detour["voxels_xyz"], detour["voxel_radius_m"])
    attempt = BASE / Path(reference["witness"]["result"]).parent
    candidate = _load_candidate(attempt)
    detour_validation = validate(candidate, model, detour)
    direct_validation = validate(
        dict(
            kind="polygon",
            points=np.array([detour["q_start"], detour["q_goal"]]),
            objective="minimum_distance",
        ),
        model,
        detour,
    )
    around_obstacle = dict(
        passed=detour_validation["passed"] and not direct_validation["collision_free"],
        problem_id=detour["problem_id"],
        problem_sha256=detour["problem_sha256"],
        historical_candidate=str(attempt.relative_to(BASE)),
        candidate_sha256=sha(attempt / "candidate.npz"),
        validation=detour_validation,
        direct_path_validation=direct_validation,
        scope="Revalidated existing trajectory with blocked direct connection; no new planning call",
    )
    return dict(
        passed=all(
            item["passed"]
            for item in (positive, negative_collision, negative_limit, around_obstacle)
        ),
        model_sha256=model.robot_sha256,
        collision_geometry_sha256=model.collision_geometry_sha256,
        legal_short_path=positive,
        colliding_start=negative_collision,
        outside_joint_limits=negative_limit,
        validated_detour=around_obstacle,
        certification="Finite numerical sampling, not continuous collision certification",
    )


def check(output_path=BASE / "results/checks.json"):
    """Append one actual health-check attempt, preserving previous attempts."""
    output_path = Path(output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    previous = read(output_path) if output_path.exists() else {}
    attempts = list(previous.get("attempts", []))
    attempt_number = len(attempts) + 1
    logs = output_path.parent / "logs"
    logs.mkdir(exist_ok=True)
    log_path = logs / f"health_{attempt_number:04d}.log"
    timeout_log = logs / f"health_{attempt_number:04d}_timeout.log"
    if log_path.exists() or timeout_log.exists():
        raise FileExistsError(
            "Health log already exists without a corresponding completed attempt"
        )
    started = time.perf_counter()
    checks = {}
    lock_path = BASE / "results/.gpu.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w") as log, redirect_stdout(log), redirect_stderr(log):
        with lock_path.open("a+") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                for name, operation in (
                    ("environment", _environment),
                    ("model_and_validation", _collision_checks),
                    ("convex_solver", _solver_checks),
                    ("subprocess_timeout", lambda: timeout_probe(timeout_log)),
                ):
                    print(f"Running {name}", flush=True)
                    try:
                        checks[name] = _jsonable(operation())
                    except Exception as exc:
                        traceback.print_exc()
                        checks[name] = dict(
                            passed=False,
                            status="infrastructure_error",
                            error_type=type(exc).__name__,
                            error=str(exc),
                        )
                    print(
                        json.dumps(dict(check=name, passed=checks[name]["passed"])),
                        flush=True,
                    )
            except BlockingIOError:
                checks["serial_gpu_lock"] = dict(
                    passed=False,
                    status="infrastructure_error",
                    error="Another benchmark job owns the GPU; no GPU diagnostic started",
                )
            finally:
                fcntl.flock(lock, fcntl.LOCK_UN)
    attempt = dict(
        attempt=attempt_number,
        passed=bool(checks) and all(item["passed"] for item in checks.values()),
        time_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        elapsed_seconds=time.perf_counter() - started,
        git_commit=head(),
        source_sha256=sha(__file__),
        checks=checks,
        log=str(log_path.relative_to(output_path.parent)),
        log_sha256=sha(log_path),
        scope="Diagnostics only; excluded from benchmark records and denominators",
    )
    attempts.append(attempt)
    output = dict(
        passed=attempt["passed"],
        checks=checks,
        latest_attempt=attempt_number,
        attempts=attempts,
    )
    write(output_path, output)
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=BASE / "results/checks.json")
    args = parser.parse_args(argv)
    result = check(args.output)
    print(
        json.dumps(
            dict(
                passed=result["passed"],
                checks={
                    key: value["passed"] for key, value in result["checks"].items()
                },
                output=str(args.output),
            ),
            indent=2,
        )
    )
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
