"""Sequential GPU jobs with isolated budgets, immutable retries and shared inputs."""
import datetime
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import traceback

from .config import case_grid, load_config, resolve_config
from .diagnostics import fingerprint
from .storage import (ROOT, atomic, attempts, digest, freeze_inputs, identifier, inventory,
                      lock, new_attempt, read, read_result, seal_attempt, selected_attempt,
                      sha, verify_attempt, verify_inputs, verify_run)


def sources(resolved):
    files = set(resolved["source_hashes"])
    for folder in ["research_platform", "reproduction/paper_constructed",
                   "reproduction/environment_reconstruction", "reproduction/alignment_diagnosis"]:
        files.update(str(p.relative_to(ROOT)) for p in (ROOT / folder).glob("*.py"))
    files.update("reproduction/" + name for name in [
        "__init__.py", "common.py", "forest.py", "pipeline.py", "optimization.py", "run.py"])
    files.update(["local/python.sh", "reproduction/requirements.lock.txt", "reproduction/conda-explicit.txt",
                  "csdecomp/tests/test_assets/table.obj", "csdecomp/tests/test_assets/table.urdf",
                  "reproduction/paper_constructed/assets/fr3_33.json"])
    return {name: sha(ROOT / name) for name in sorted(files)}


def job_graph(config):
    cases = case_grid(config)
    jobs = []
    for seed in config["environment_seeds"]:
        jobs.append(dict(id=f"scene-e{seed:03d}", kind="scene", environment_seed=seed, dependencies={}))
    for seed in config["roadmap_seeds"]:
        for size in config["roadmap_sizes"]:
            jobs.append(dict(id=f"map-r{seed:03d}-n{size:05d}", kind="roadmap",
                             roadmap_seed=seed, roadmap_size=size, dependencies={}))
    for case in cases:
        base = {k: v for k, v in case.items() if k != "id"}
        deps = dict(scene=f"scene-e{case['environment_seed']:03d}",
                    roadmap=f"map-r{case['roadmap_seed']:03d}-n{case['roadmap_size']:05d}")
        plan = f"plan-{case['id']}"
        jobs.append(dict(**base, id=plan, kind="plan", case_id=case["id"], dependencies=deps))
        for method in config["methods"]:
            jobs.append(dict(**base, id=f"method-{case['id']}-{method}", kind="method",
                             case_id=case["id"], method=method, dependencies=dict(deps, plan=plan)))
    return cases, jobs


def create_run(config_path, output=None):
    config = load_config(config_path)
    resolved = resolve_config(config)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    run = Path(output).resolve() if output else ROOT / "runs" / (stamp + "_" + config["name"])
    run.mkdir(parents=True, exist_ok=False)
    try:
        source = sources(resolved)
        cases, jobs = job_graph(config)
        for name in source:
            target = run / "source_snapshot" / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / name, target)
            if sha(target) != source[name]:
                raise ValueError("Source changed while snapshotting: " + name)
        environment = fingerprint()
        manifest = dict(schema_version=1, name=config["name"], created_utc=stamp,
            config=config, config_sha256=resolved["config_sha256"],
            resolved_sha256=resolved["resolved_sha256"], cases=cases, jobs=jobs,
            source_hashes=source, environment_sha256=digest(environment),
            git_head=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
            git_dirty=bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT)),
            command=sys.argv, semantics="All declared tasks stay in denominator; newest explicit attempt is reported; all attempts retained",
            clock="per-job monotonic wall time; shared preparation, method solver and validation reported separately")
        atomic(run / "resolved.json", resolved)
        atomic(run / "environment.json", environment)
        atomic(run / "manifest.json", manifest)
        freeze_inputs(run)
    except BaseException as exc:
        atomic(run / "creation_failure.json", dict(error=repr(exc), traceback=traceback.format_exc()))
        raise
    return run


def terminate_group(process):
    """Reap the leader and stop descendants even when the leader already exited.

    The session created by Popen owns this process group. Waiting only for its
    leader is insufficient: a native solver's child may ignore SIGTERM.
    """
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        process.wait()
        return
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        process.poll()  # Reap the leader; group members may still be alive.
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.02)
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    process.wait()


def execute_process(command, out, timeout):
    """Bound the entire subprocess, including native code, and preserve all logs."""
    out = Path(out)
    started = time.monotonic()
    outcome = "finished"
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f"Received signal {signal.Signals(signum).name}")

    previous_handler = signal.signal(signal.SIGTERM, interrupted)
    try:
        with (out / "stdout.log").open("x") as stdout, (out / "stderr.log").open("x") as stderr:
            process = subprocess.Popen(command, cwd=ROOT, stdout=stdout, stderr=stderr, start_new_session=True)
            try:
                atomic(out / "process.json", dict(pid=process.pid, command=command, timeout_seconds=timeout))
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                outcome = "timeout"
            finally:
                # Includes SIGTERM, metadata failure and orphan descendants after
                # normal exit. SIGKILL cannot run Python cleanup handlers.
                terminate_group(process)
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
    return dict(outcome=outcome, exit_code=process.returncode, wall_seconds=time.monotonic()-started)


def dependency_receipts(run, job):
    result = {}
    for dependency in job["dependencies"].values():
        path = selected_attempt(run, dependency)
        if path and (path / "receipt.json").exists():
            verify_attempt(path)
            result[dependency] = sha(path / "receipt.json")
    return result


def schedule(run, case_id=None, retry_failed=False, retry_jobs=()):
    """Default resume retries interrupted work, not completed numerical failures."""
    run = Path(run).resolve()
    with lock(run / ".run.lock"), lock(ROOT / ".research/gpu.lock"):
        manifest = verify_inputs(run, current_sources=True)
        if sources(read(run / "resolved.json")) != manifest["source_hashes"]:
            raise ValueError("Source file inventory changed; create a new run")
        if fingerprint() != read(run / "environment.json"):
            raise ValueError("Numerical runtime, thread settings or GPU identity changed; create a new run")
        # Always inspect every existing attempt, including failures and old retries.
        verify_run(run, require_complete=False)
        jobs = manifest["jobs"]
        known = {job["id"] for job in jobs}
        if set(retry_jobs) - known:
            raise ValueError("Unknown retry jobs")
        if case_id is not None and case_id not in {c["id"] for c in manifest["cases"]}:
            raise ValueError("Unknown case")
        selected = known if case_id is None else {j["id"] for j in jobs if j.get("case_id") == case_id}
        # Include transitive shared preparation needed by a single-case request.
        while True:
            closure = selected | {dep for j in jobs if j["id"] in selected for dep in j["dependencies"].values()}
            if closure == selected:
                break
            selected = closure
        for job in jobs:
            job_id = job["id"]
            if job_id not in selected:
                continue
            last = selected_attempt(run, job_id)
            if last is not None and not (last / "receipt.json").exists():
                # A live orphan worker owns this lock; never seal changing files.
                with lock(last / ".execution.lock"):
                    seal_attempt(last, "interrupted", dependency_receipts=read(last / "attempt.json").get("dependency_receipts", {})
                                 if (last / "attempt.json").exists() else {},
                                 reason="Previous process stopped before writing its receipt")
            previous = read_result(run, job_id)
            automatic = previous is None or previous["status"] in ["interrupted", "stale_dependency"]
            requested = previous is not None and previous["status"] != "success" and (
                retry_failed or job_id in retry_jobs)
            if not automatic and not requested:
                continue
            dependencies = {name: read_result(run, dep) for name, dep in job["dependencies"].items()}
            receipts = dependency_receipts(run, job)
            out = new_attempt(run, job_id)
            atomic(out / "attempt.json", dict(job=job, created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    retry_of=previous.get("attempt") if previous else None,
                    retry_reason="explicit" if requested else "resume", dependency_receipts=receipts))
            blocked = {name: None if r is None else r["status"] for name, r in dependencies.items()
                       if r is None or r["status"] != "success"}
            if blocked:
                atomic(out / "result.json", dict(status="blocked", solver_success=False,
                                                 validation_passed=False, blocked_dependencies=blocked))
                seal_attempt(out, "blocked", dependency_receipts=receipts, wall_seconds=0.0)
                print(job_id, "blocked", flush=True)
                continue
            command = [sys.executable, "-m", "research_platform.worker", "--run", str(run),
                       "--job", job_id, "--attempt", out.name]
            try:
                execution = execute_process(command, out, manifest["config"]["limits"]["job_timeout_seconds"])
            except KeyboardInterrupt as exc:
                seal_attempt(out, "interrupted", dependency_receipts=receipts, reason=str(exc) or "KeyboardInterrupt")
                raise
            result = read(out / "result.json") if (out / "result.json").exists() else {}
            if execution["outcome"] == "timeout":
                status = "timeout"
            elif execution["exit_code"] != 0 or not result:
                status = "worker_failed"
            else:
                status = result["status"]
            seal_attempt(out, status, dependency_receipts=receipts, **execution)
            print(job_id, status, flush=True)
        return verify_run(run, require_complete=False)


def parent_death_signal():
    """Linux workers terminate if the coordinator is killed, avoiding orphan GPU jobs."""
    if sys.platform.startswith("linux"):
        import ctypes
        parent = os.getppid()
        if ctypes.CDLL(None, use_errno=True).prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
            raise OSError(ctypes.get_errno(), "Unable to set parent-death signal")
        if os.getppid() != parent:
            os.kill(os.getpid(), signal.SIGTERM)


def worker(run, job_id, attempt_number):
    parent_death_signal()
    run = Path(run).resolve()
    manifest = verify_inputs(run, current_sources=True)
    identifier(job_id)
    if not attempt_number.isdecimal():
        raise ValueError("Invalid attempt")
    job = next(j for j in manifest["jobs"] if j["id"] == job_id)
    out = run / "jobs" / job_id / "attempts" / attempt_number
    if selected_attempt(run, job_id) != out or not (out / "attempt.json").exists():
        raise ValueError("Worker can only execute its newly scheduled attempt")
    if (out / "receipt.json").exists() or (out / "result.json").exists():
        raise ValueError("Refuse to overwrite attempt evidence")
    with lock(out / ".execution.lock"):
        expected = read(out / "attempt.json")["dependency_receipts"]
        if expected != dependency_receipts(run, job):
            raise ValueError("Dependencies changed after scheduling")
        dependencies = {name: selected_attempt(run, dep) for name, dep in job["dependencies"].items()}
        if any(read_result(run, dep)["status"] != "success" for dep in job["dependencies"].values()):
            raise ValueError("Worker dependencies are not successful")
        from .backend import execute_job
        started = time.monotonic()
        try:
            result = execute_job(job, read(run / "resolved.json"), dependencies, out)
            # The hash binds every method to one shared scene, IK solution, seed
            # path and validation/algorithm configuration. It is never per-method IK.
            plan = out if job["kind"] == "plan" else dependencies.get("plan")
            if plan and (plan / "input.json").exists() and (plan / "seed_path.npz").exists():
                result["input_hash"] = digest(dict(protocol=read(run / "manifest.json")["resolved_sha256"],
                    input=sha(plan / "input.json"), seed_path=sha(plan / "seed_path.npz")))
            if expected != dependency_receipts(run, job):
                raise ValueError("Dependency changed during execution")
            result.update(job_id=job_id, case_id=job.get("case_id"), method=job.get("method"),
                          worker_wall_seconds=time.monotonic()-started)
            atomic(out / "result.json", result)
        except Exception as exc:
            atomic(out / "result.json", dict(status="error", solver_success=False, validation_passed=False,
                    error=repr(exc), traceback=traceback.format_exc(), worker_wall_seconds=time.monotonic()-started))
