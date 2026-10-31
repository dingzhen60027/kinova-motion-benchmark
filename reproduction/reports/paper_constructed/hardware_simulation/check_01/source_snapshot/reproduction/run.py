"""Isolated, resumable full-factorial reconstruction with immutable raw records."""

from __future__ import annotations
import argparse
import datetime
import importlib.metadata
import itertools
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import traceback
import numpy as np
from .common import ROOT, atomic, config, digest, read, sources


def runtime_versions():
    return {
        name: importlib.metadata.version(name)
        for name in [
            "csdecomp",
            "numpy",
            "drake",
            "scipy",
            "pybezier",
            "scsplanning",
            "gurobipy",
            "cvxpy",
            "clarabel",
        ]
    }


def case_worker(root, case_id):
    from .forest import Forest
    from .pipeline import Pipeline, save_candidate, validate
    from .optimization import nto

    root = Path(root)
    manifest = read(root / "manifest.json")
    cfg = manifest["config"]
    if digest(sources()) != manifest["implementation_hash"]:
        raise ValueError("source identity changed during experiment")
    if runtime_versions() != manifest["runtime_versions"]:
        raise ValueError("runtime dependency versions changed")
    entry = next(c for c in manifest["cases"] if c["id"] == case_id)
    out = root / "cases" / case_id
    out.mkdir(parents=True, exist_ok=True)
    record = dict(
        **entry,
        status="running",
        implementation_hash=manifest["implementation_hash"],
        methods={},
        events=[],
    )
    atomic(out / "case.json", record)

    def progress(stage):
        atomic(
            out / "progress.json",
            dict(stage=stage, elapsed_seconds=time.monotonic() - started),
        )

    started = time.monotonic()
    try:
        progress("model")
        forest = Forest(cfg, entry["environment_seed"])
        atomic(
            out / "input.json",
            dict(
                case=entry,
                config_hash=manifest["config_hash"],
                centers=forest.centers,
                start=cfg["start"],
                goal=cfg["goal"],
                urdf_sha256=__import__("hashlib")
                .sha256(forest.source.encode())
                .hexdigest(),
            ),
        )
        (out / "forest.urdf").write_text(forest.source)
        progress("native_roadmap")
        roadmap, path, drm = forest.roadmap(
            entry["roadmap_seed"], entry["roadmap_size"], out
        )
        record["roadmap"] = roadmap
        atomic(out / "case.json", record)
        valid_seed = (
            roadmap["success"]
            and roadmap["validation"]["collision_free"]
            and roadmap["validation"]["endpoint_pass"]
        )
        record["drm_validated_success"] = bool(valid_seed)
        if not valid_seed:
            record["status"] = (
                "drm_failed" if not roadmap["success"] else "invalid_seed_path"
            )
            atomic(out / "case.json", record)
            return
        # Persist a warm-up separately; do not hide cold worker/setup costs.
        progress("eizo_warmup")
        tic = time.perf_counter()
        warm = Pipeline(forest, 1337)
        warm.inflate(path[0], path[1])
        del warm
        record["eizo_warmup_seconds"] = time.perf_counter() - tic
        for method in cfg["method_order"]:
            progress(method)
            dest = out / method
            dest.mkdir(exist_ok=True)
            tic = time.perf_counter()
            try:
                if method.startswith("NTO"):
                    candidate = nto(path, forest, method == "NTO_time")
                    candidate["method"] = method
                    to_seconds = time.perf_counter() - tic
                    save_candidate(dest, candidate)
                    if (
                        candidate["duration"] <= 0
                        or not np.isfinite(candidate["controls"]).all()
                    ):
                        result = dict(
                            status="optimization_failed",
                            solver_success=candidate["solver_success"],
                            solver_status=candidate["solver_status"],
                            optimization_seconds=to_seconds,
                            construction_seconds=0.0,
                            total_seconds=to_seconds,
                            cost=None,
                            collision_free=None,
                            validation_passed=False,
                            reason="nonfinite or nonpositive candidate",
                        )
                    else:
                        check_started = time.perf_counter()
                        validation = validate(candidate, forest)
                        validation_seconds = time.perf_counter() - check_started
                        samples = validation.pop("_samples", None)
                        if samples:
                            np.savez_compressed(
                                dest / "trajectory_samples.npz", **samples
                            )
                        atomic(dest / "validation.json", validation)
                        result = dict(
                            status=(
                                "success_numerical"
                                if candidate["solver_success"] and validation["passed"]
                                else (
                                    "validation_failed"
                                    if candidate["solver_success"]
                                    else "optimization_failed"
                                )
                            ),
                            solver_success=candidate["solver_success"],
                            solver_status=candidate["solver_status"],
                            optimization_seconds=to_seconds,
                            construction_seconds=0.0,
                            total_seconds=to_seconds,
                            validation_seconds=validation_seconds,
                            cost=validation["cost"],
                            solver_objective=candidate["objective"],
                            collision_free=validation["collision_free"],
                            validation_passed=validation["passed"],
                            minimum_clearance=validation["minimum_clearance"],
                            recovery_iterations=0,
                            regions=0,
                        )
                else:
                    setup = time.perf_counter()
                    pipeline = Pipeline(forest, 1337)
                    allocation_seconds = time.perf_counter() - setup
                    answer = pipeline.solve(method, path, drm, dest)
                    candidate = answer.pop("candidate")
                    regions = answer.pop("regions")
                    validation = answer.pop("validation", None)
                    if validation:
                        samples = validation.pop("_samples", None)
                        if samples:
                            np.savez_compressed(
                                dest / "trajectory_samples.npz", **samples
                            )
                        atomic(dest / "validation.json", validation)
                    result = dict(
                        **answer,
                        solver_success=candidate["solver_success"],
                        solver_status=candidate["solver_status"],
                        regions=len(regions),
                        cost=validation["cost"] if validation else None,
                        validation_passed=validation["passed"] if validation else False,
                        collision_free=(
                            validation["collision_free"] if validation else None
                        ),
                        minimum_clearance=(
                            validation["minimum_clearance"] if validation else None
                        ),
                        allocation_seconds=allocation_seconds,
                    )
            except Exception as error:
                traceback.print_exc()
                result = dict(
                    status=(
                        "blocked_license"
                        if getattr(error, "errno", None) == 10010
                        else (
                            "optimization_failed"
                            if isinstance(error, ValueError)
                            else "error"
                        )
                    ),
                    solver_success=False,
                    reason=repr(error),
                    traceback=traceback.format_exc(),
                    total_seconds=time.perf_counter() - tic,
                    cost=None,
                    validation_passed=False,
                    collision_free=None,
                )
            result.update(
                method=method,
                case_id=case_id,
                elapsed_wall_seconds=time.perf_counter() - tic,
            )
            atomic(dest / "result.json", result)
            record["methods"][method] = result
            atomic(out / "case.json", record)
        record["status"] = "completed"
        record["elapsed_seconds"] = time.monotonic() - started
    except Exception as error:
        traceback.print_exc()
        record.update(
            status="error", reason=repr(error), traceback=traceback.format_exc()
        )
    finally:
        atomic(out / "case.json", record)


def new_run(args):
    cfg = config(args.config)
    envs = args.environments or cfg["environment_seeds"]
    seeds = args.roadmap_seeds or cfg["roadmap_seeds"]
    sizes = args.sizes or cfg["roadmap_sizes"]
    for values in (envs, seeds, sizes):
        if not values or len(set(values)) != len(values):
            raise ValueError("case values must be unique")
    cases = [
        dict(
            id=f"e{e:02d}_r{r:02d}_n{n:05d}",
            environment_seed=e,
            roadmap_seed=r,
            roadmap_size=n,
        )
        for e, r, n in itertools.product(envs, seeds, sizes)
    ]
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    out = Path(args.output_root or ROOT / "reproduction/runs") / (stamp + "_forest")
    out = out.resolve()
    out.mkdir(parents=True)
    source = sources()
    provenance = dict(
        code_commit=subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        dirty=bool(
            subprocess.check_output(
                ["git", "status", "--porcelain"], cwd=ROOT, text=True
            ).strip()
        ),
        implementation_hash=digest(source),
        source_files=source,
    )
    for name in source:
        destination = out / "source_snapshot" / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / name, destination)
    manifest = dict(
        schema_version=1,
        status="pending",
        created_utc=stamp,
        paper="arXiv:2504.10783v1",
        fidelity=cfg["fidelity"],
        config=cfg,
        config_hash=digest(cfg),
        cases=cases,
        planned_cases=len(cases),
        planned_solver_calls=len(cases) * len(cfg["method_order"]),
        runtime_versions=runtime_versions(),
        threads={
            name: os.environ.get(name)
            for name in ["OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"]
        },
        **provenance,
    )
    atomic(out / "manifest.json", manifest)
    subprocess.run(
        ["nvidia-smi", "-q"], stdout=(out / "gpu_before.txt").open("w"), check=True
    )
    return out


def execute(out):
    out = Path(out).resolve()
    m = read(out / "manifest.json")
    if (
        digest(sources()) != m["implementation_hash"]
        or runtime_versions() != m["runtime_versions"]
    ):
        raise ValueError("source/runtime changed; do not mix results")
    m["status"] = "running"
    atomic(out / "manifest.json", m)
    print("RUN_DIR=" + str(out), flush=True)
    failures = 0
    completed = 0
    begin = time.monotonic()
    for case in m["cases"]:
        dest = out / "cases" / case["id"]
        record_path = dest / "case.json"
        if record_path.exists() and read(record_path).get("status") in {
            "completed",
            "drm_failed",
            "invalid_seed_path",
            "timeout",
            "error",
            "process_error",
        }:
            completed += 1
            continue
        dest.mkdir(parents=True, exist_ok=True)
        cmd = [
            sys.executable,
            "-m",
            "reproduction.run",
            "_case",
            "--run-dir",
            str(out),
            "--case-id",
            case["id"],
        ]
        with (dest / "stdout.log").open("w") as stdout, (dest / "stderr.log").open(
            "w"
        ) as stderr:
            proc = subprocess.Popen(
                cmd, cwd=ROOT, stdout=stdout, stderr=stderr, start_new_session=True
            )
            timed_out = False
            try:
                proc.wait(timeout=m["config"]["case_timeout_seconds"])
            except subprocess.TimeoutExpired:
                timed_out = True
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
            except KeyboardInterrupt:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
                m["status"] = "interrupted"
                atomic(out / "manifest.json", m)
                raise
        record = read(record_path) if record_path.exists() else dict(**case, methods={})
        if timed_out or proc.returncode != 0:
            record.update(
                status="timeout" if timed_out else "process_error",
                process_exit_code=proc.returncode,
            )
            if (dest / "progress.json").exists():
                record["failure_stage"] = read(dest / "progress.json")["stage"]
            atomic(record_path, record)
        counts = {
            name: record.get("methods", {}).get(name, {}).get("status", "not_run")
            for name in m["config"]["method_order"]
        }
        completed += 1
        failures += record["status"] in {"error", "process_error"} or any(
            v == "error" for v in counts.values()
        )
        print(
            f"{completed}/{m['planned_cases']} {case['id']} {record['status']} {counts} elapsed={time.monotonic()-begin:.1f}s",
            flush=True,
        )
        atomic(
            out / "progress.json",
            dict(
                completed_cases=completed,
                planned_cases=m["planned_cases"],
                infrastructure_error_cases=failures,
                elapsed_seconds=time.monotonic() - begin,
                last_case=case["id"],
            ),
        )
    failures = sum(
        read(out / "cases" / case["id"] / "case.json").get("status")
        in {"error", "process_error"}
        or any(
            r.get("status") == "error"
            for r in read(out / "cases" / case["id"] / "case.json")
            .get("methods", {})
            .values()
        )
        for case in m["cases"]
    )
    m["status"] = "completed"
    m["infrastructure_error_cases"] = failures
    atomic(out / "manifest.json", m)
    subprocess.run(
        ["nvidia-smi", "-q"], stdout=(out / "gpu_after.txt").open("w"), check=True
    )
    return 1 if failures else 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("command", choices=["run", "resume", "_case"])
    p.add_argument("--config")
    p.add_argument("--run-dir")
    p.add_argument("--case-id")
    p.add_argument("--output-root")
    p.add_argument("--environments", type=int, nargs="+")
    p.add_argument("--roadmap-seeds", type=int, nargs="+")
    p.add_argument("--sizes", type=int, nargs="+")
    a = p.parse_args()
    if a.command == "_case":
        case_worker(a.run_dir, a.case_id)
        return 0
    return execute(new_run(a) if a.command == "run" else a.run_dir)


if __name__ == "__main__":
    raise SystemExit(main())
