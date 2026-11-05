"""CPU-only forensic replay of the saved NTO first solve, with GDB evidence.

This is not a benchmark run: the saved, validated DRM path replaces the front
end. No CUDA query or optimizer configuration tuning is allowed. The driver
can reproduce the original deadline signal, or leave IPOPT's original CPU
limit in charge. Both modes preserve a native SIGSEGV as a crash.
"""
from __future__ import annotations

import argparse
import faulthandler
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[6]
DEFAULT_SOURCE = (
    ROOT / "benchmarks/generic_motion_v1/results/rigor_checks/dev_nto_incumbent"
    "/dev_seed0/jobs/dev-dense_clutter-001-p01-NTO_REFINED-minimum_time-4e4af77e"
    "/attempts/0001"
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")
    temporary.replace(path)


def verified_source(path):
    source = Path(path).resolve()
    receipt = json.loads((source / "receipt.json").read_text())
    failures = [name for name, digest in receipt["files"].items()
                if not (source / name).is_file() or sha(source / name) != digest]
    if failures:
        raise ValueError("Source receipt mismatch: " + repr(failures))
    return source, dict(receipt_sha256=sha(source / "receipt.json"),
                        files_checked=len(receipt["files"]), errors=failures)


def worker(args):
    sys.path.insert(0, str(ROOT))
    import numpy as np
    from benchmarks.generic_motion_v1.model import Model
    from benchmarks.generic_motion_v1.runners.paper_nto import paper_nto, warm_start

    faulthandler.enable(all_threads=True)
    source, receipt = verified_source(args.source)
    out = Path(args.output)
    case = json.loads((source / "input.json").read_text())
    case["_objective"] = "minimum_time"
    cfg = json.loads((source / "evaluation.json").read_text())
    parameters = json.loads((source / "refinement/round_00/nto_parameters.json").read_text())
    profile = parameters["profile"]
    path = np.load(source / "path.npy", allow_pickle=False)
    model = Model(case["voxels_xyz"], case["voxel_radius_m"])
    for name in ("robot_sha256", "collision_geometry_sha256"):
        if getattr(model, name) != case[name]:
            raise ValueError("Source/canonical model identity mismatch: " + name)

    def forbidden(*unused_args, **unused_kwargs):
        raise AssertionError("CPU forensic replay must not query CUDA")

    model.cuda = forbidden
    # Verify the exact frozen initial spline before executing a solver.
    initial, _, controls, order, duration = warm_start(
        path, "minimum_time", minimum_time_control_points=profile["minimum_time_control_points"])
    with np.load(source / "refinement/round_00/nto_initial.npz", allow_pickle=False) as original:
        np.testing.assert_array_equal(controls, original["controls"])
        np.testing.assert_array_equal(initial.basis().knots(), original["knots"])
        np.testing.assert_equal(duration, original["duration"])
        np.testing.assert_array_equal(path, original["original_path"])
    phase = json.loads((source / "phase.json").read_text())
    remaining = float(cfg["online_budget_seconds"]) - phase["elapsed_seconds"]
    started = time.perf_counter()
    write(out / "ready.json", dict(pid=os.getpid(), started_monotonic=started,
          deadline_monotonic=started + remaining, original_remaining_seconds=remaining,
          source=str(source.relative_to(ROOT)), source_receipt=receipt,
          profile=profile, spline_order=order, initial_spline_exactly_equal=True,
          source_original_max_cpu_time=parameters["parameters"]["max_cpu_time"],
          gpu_queries_forbidden=True,
          scope="One CPU first-solve reconstruction; no planning score or timing comparison"))
    try:
        candidate = paper_nto(path, model, case, cfg, out, started + remaining,
                             _profile=profile)
        write(out / "completion.json", dict(
            status="returned", solver_success=bool(candidate["solver_success"]),
            solver_status=candidate["solver_status"],
            wall_seconds=time.perf_counter() - started))
    except BaseException as exc:
        write(out / "completion.json", dict(status="python_exception", exception=repr(exc),
              traceback=traceback.format_exc(), wall_seconds=time.perf_counter()-started))
        raise


def driver(args):
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    source, receipt = verified_source(args.source)
    # Match the existing Conda launcher; do not alter the installed environment.
    environment = os.environ.copy()
    for name in ("PYTHONHOME", "PYTHONPATH", "LD_LIBRARY_PATH"):
        environment.pop(name, None)
    environment.update(PYTHONNOUSERSITE="1", OPENBLAS_NUM_THREADS="1",
                       MKL_NUM_THREADS="1", OMP_NUM_THREADS=environment.get("OMP_NUM_THREADS", "4"),
                       CUDA_VISIBLE_DEVICES="")
    python = ROOT / ".conda/envs/csdecomp/bin/python"
    command = ["gdb", "--batch", "--quiet", "-ex", "set pagination off",
               "-ex", "handle SIGTERM nostop noprint pass", "-ex", "run",
               "-ex", "thread apply all bt", "--args", str(python),
               str(Path(__file__).resolve()), "--worker", "--source", str(source),
               "--output", str(out)]
    started = time.perf_counter()
    state = dict(command=command, source_receipt=receipt, script_sha256=sha(__file__),
                 git_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                 runtime_source_sha256={name: sha(ROOT / name) for name in (
                     "benchmarks/generic_motion_v1/model.py",
                     "benchmarks/generic_motion_v1/runners/paper_nto.py",
                     "benchmarks/generic_motion_v1/runners/nto_collision_constraint.py")},
                 thread_environment={name: environment[name] for name in (
                     "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "OMP_NUM_THREADS", "CUDA_VISIBLE_DEVICES")},
                 source=str(source.relative_to(ROOT)), mode=args.mode,
                 signal_events=[], gpu_queries_forbidden=True,
                 scope="CPU-only forensic experiment, not a benchmark query; no score modification",
                 limitation="A non-reproduced crash does not establish absence of the original fault")
    write(out / "driver.json", state)
    with (out / "gdb.log").open("w") as log:
        process = subprocess.Popen(command, cwd=ROOT, env=environment,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        sent_term = sent_kill = False
        ready = None
        while process.poll() is None:
            if ready is None and (out / "ready.json").is_file():
                ready = json.loads((out / "ready.json").read_text())
            now = time.perf_counter()
            if args.mode == "original_watchdog" and ready is not None:
                if not sent_term and now >= ready["deadline_monotonic"]:
                    try:
                        os.kill(ready["pid"], signal.SIGTERM)
                        state["signal_events"].append(dict(signal="SIGTERM", monotonic=now))
                    except ProcessLookupError:
                        state["signal_events"].append(dict(signal="SIGTERM", already_exited=True))
                    sent_term = True
                if sent_term and not sent_kill and now >= ready["deadline_monotonic"]+2:
                    try:
                        os.kill(ready["pid"], signal.SIGKILL)
                        state["signal_events"].append(dict(signal="SIGKILL", monotonic=now))
                    except ProcessLookupError:
                        pass
                    sent_kill = True
            if now-started > 100:
                os.killpg(process.pid, signal.SIGKILL)
                state["diagnostic_outer_ceiling_exceeded"] = True
                break
            time.sleep(.02)
        process.wait()
    text = (out / "gdb.log").read_text(errors="replace")
    state.update(gdb_exit_code=process.returncode, wall_seconds=time.perf_counter()-started,
                 native_sigsegv_observed="received signal SIGSEGV" in text,
                 completion=json.loads((out / "completion.json").read_text())
                 if (out / "completion.json").is_file() else None,
                 gdb_log_sha256=sha(out / "gdb.log"))
    write(out / "driver.json", state)
    print(json.dumps({k: state[k] for k in ("mode", "gdb_exit_code", "wall_seconds",
          "native_sigsegv_observed", "completion")}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", choices=("original_watchdog", "solver_limit_only"),
                        default="original_watchdog")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    arguments = parser.parse_args()
    worker(arguments) if arguments.worker else driver(arguments)
