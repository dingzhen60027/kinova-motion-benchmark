"""Serial GPU jobs with an external deadline and immutable attempt receipts."""
from pathlib import Path
import datetime
import hashlib
import json
import os
import signal
import subprocess
import sys
import time

from .common import BASE, ROOT, cases, code_inventory, head, read, sha, write


def prepare(manifest, roadmaps, output, *, objectives, methods, selection="all", problem=None, timeout=60., config=None):
    from .roadmap import load_config
    evaluation = load_config(config)
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError("Output already exists; use resume or a new run directory")
    manifest = Path(manifest).resolve()
    data = cases(manifest)
    if any(c["split"] == "test" for c in data):
        raise ValueError("Formal test execution is locked in v1 acceptance. Test generation only.")
    if problem:
        data = [c for c in data if c["problem_id"] == problem]
        if not data:
            raise ValueError("Unknown problem identifier")
    if selection == "first-per-family":
        selected = {}
        for case in data:
            selected.setdefault(case["family"], case)
        data = list(selected.values())
    jobs = []
    roadmap_files = {}
    for roadmap in roadmaps:
        roadmap = Path(roadmap).resolve()
        metadata_path = roadmap / "metadata.json"
        if not metadata_path.is_file():
            raise FileNotFoundError(metadata_path)
        metadata = read(metadata_path)
        for file in roadmap.iterdir():
            if file.is_file():
                roadmap_files[str(file)] = sha(file)
        seed = int(metadata.get("seed", metadata.get("roadmap_seed", 0)))
        for case in data:
            pairing = next((p for p in case.get("paired_seeds", []) if p["drm_seed"] == seed), None)
            if pairing is None:
                raise ValueError(f"Roadmap seed {seed} is not declared in {case['problem_id']}")
            for objective in objectives:
                for method in methods:
                    identity = json.dumps([case["problem_id"], objective, method, str(roadmap)], separators=(",", ":"))
                    job_id = f"{case['problem_id']}-{method}-{objective}-{hashlib.sha256(identity.encode()).hexdigest()[:8]}"
                    jobs.append(dict(job_id=job_id, case_id=case["problem_id"], scene_id=case["scene_id"],
                        family=case["family"], method=method, objective=objective,
                        roadmap_id=metadata.get("roadmap_id", roadmap.name), evaluation_config=evaluation,
                        roadmap_dir=str(roadmap), method_seed=int(pairing["method_seed"]),
                        solver_seed=int(pairing["solver_seed"]), solver_seed_used=False,
                        solver_seed_note="The current solvers start deterministically from the fixed DRM path.", case=case))
    output.mkdir(parents=True)
    sources = code_inventory()
    for name in sources:
        dest = output / "source_snapshot" / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes((ROOT / name).read_bytes())
    write(output / "run_manifest.json", dict(version=1, source_code_commit=head(),
        created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), manifest_path=str(manifest),
        manifest_sha256=sha(manifest), source_hashes=sources, roadmap_files=roadmap_files,
        selection=selection, timeout_seconds=float(timeout), split=data[0]["split"] if data else None,
        formal_test_run=False, jobs=jobs,
        denominator="All declared legal tasks, including solver/front-end/timeout failures. No replacement.",
        seed_policy="Roadmap PCG64 seed paired with the exposed EI-ZO seed; deterministic solver initialization."))
    write(output / "identity.json", dict(run_manifest_sha256=sha(output / "run_manifest.json")))
    return output


def verify(run, current_sources=False):
    run = Path(run)
    manifest = read(run / "run_manifest.json")
    errors = []
    if (run / "identity.json").exists() and read(run / "identity.json")["run_manifest_sha256"] != sha(run / "run_manifest.json"):
        errors.append("run manifest changed")
    if sha(manifest["manifest_path"]) != manifest["manifest_sha256"]:
        errors.append("input manifest changed")
    for name, expected in manifest["source_hashes"].items():
        snapshot = run / "source_snapshot" / name
        if not snapshot.is_file() or sha(snapshot) != expected:
            errors.append("source snapshot changed: " + name)
        if current_sources and (not (ROOT / name).is_file() or sha(ROOT / name) != expected):
            errors.append("current source changed; use a new run: " + name)
    for name, expected in manifest["roadmap_files"].items():
        if not Path(name).is_file() or sha(name) != expected:
            errors.append("roadmap changed: " + name)
    complete = 0
    for job in manifest["jobs"]:
        path = run / "jobs" / job["job_id"] / "attempts"
        for attempt in sorted(path.glob("[0-9][0-9][0-9][0-9]")):
            if not (attempt / "receipt.json").exists():
                errors.append("unsealed attempt: " + str(attempt))
                continue
            receipt = read(attempt / "receipt.json")
            for name, expected in receipt["files"].items():
                if not (attempt / name).is_file() or sha(attempt / name) != expected:
                    errors.append("artifact changed: " + str(attempt / name))
        if any(path.glob("*/receipt.json")):
            complete += 1
    return dict(integrity_passed=not errors, complete=complete == len(manifest["jobs"]),
                completed_jobs=complete, planned_jobs=len(manifest["jobs"]), errors=errors)


def _gpu_memory(pid):
    try:
        r = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"],
                           capture_output=True, text=True, timeout=2.)
        values = [float(line.split(",")[1]) for line in r.stdout.splitlines()
                  if line.split(",")[0].strip() == str(pid)]
        return max(values) if values else None
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return None


def execute(job, run, timeout):
    attempt = Path(run) / "jobs" / job["job_id"] / "attempts" / "0001"
    attempt.mkdir(parents=True, exist_ok=False)
    write(attempt / "request.json", job)
    argv = [sys.executable, "-m", "benchmarks.generic_motion_v1", "_worker", "--attempt", str(attempt)]
    start = time.perf_counter()
    peak = None
    timed_out = False
    with (attempt / "stdout.log").open("w") as stdout, (attempt / "stderr.log").open("w") as stderr:
        process = subprocess.Popen(argv, cwd=ROOT, stdout=stdout, stderr=stderr, start_new_session=True)
        while process.poll() is None:
            elapsed = time.perf_counter() - start
            if elapsed >= timeout:
                timed_out = True
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                break
            memory = _gpu_memory(process.pid)
            if memory is not None:
                peak = memory if peak is None else max(peak, memory)
            time.sleep(min(.2, max(.001, timeout - (time.perf_counter() - start))))
    wall = time.perf_counter() - start
    write(attempt / "process.json", dict(argv=argv, exit_code=process.returncode, timeout_seconds=timeout,
        timed_out=timed_out, cold_e2e_seconds=wall, sampled_gpu_peak_mib=peak,
        gpu_peak_scope="Per-process nvidia-smi samples at approximately 0.2 s; not an exact allocator high-water mark.",
        includes="fresh process, imports, CUDA context, model/map load, warm-up, online planning, validation and output"))
    result_path = attempt / "result.json"
    result = read(result_path) if result_path.exists() else {}
    if timed_out or process.returncode != 0 or not result:
        if result:
            write(attempt / "result_before_external_stop.json", result)
        result.update(status="timeout" if timed_out else "infrastructure_error", success=False,
                      error="External job deadline" if timed_out else f"Worker exit {process.returncode}")
    result.update(problem_id=job["case_id"], scene_id=job["scene_id"], family=job["family"],
                  method=job["method"], objective=job["objective"], roadmap_id=job["roadmap_id"],
                  method_seed=job["method_seed"], cold_e2e_seconds=wall, sampled_gpu_peak_mib=peak)
    write(result_path, result)
    files = {str(p.relative_to(attempt)): sha(p) for p in sorted(attempt.rglob("*")) if p.is_file()}
    write(attempt / "receipt.json", dict(status=result["status"], files=files))
    print(json.dumps(dict(job_id=job["job_id"], status=result["status"], seconds=round(wall, 3))), flush=True)
    return result


def resume(run, max_jobs=None):
    run = Path(run).resolve()
    check = verify(run, current_sources=True)
    if not check["integrity_passed"]:
        raise ValueError(json.dumps(check))
    manifest = read(run / "run_manifest.json")
    completed = 0
    for job in manifest["jobs"]:
        if (run / "jobs" / job["job_id"] / "attempts" / "0001" / "receipt.json").exists():
            continue
        if max_jobs is not None and completed >= max_jobs:
            break
        execute(job, run, manifest["timeout_seconds"])
        completed += 1
    result = verify(run)
    write(run / "verification.json", result)
    return result


def worker(attempt):
    from .runners.csdecomp_baseline import run_case
    attempt = Path(attempt)
    job = read(attempt / "request.json")
    try:
        result = run_case(job["case"], job["roadmap_dir"], attempt, objective=job["objective"],
                          seed=job["method_seed"], method=job["method"], config=job.get("evaluation_config"))
        write(attempt / "result.json", result)
    except Exception as exc:
        import traceback
        traceback.print_exc()
        write(attempt / "result.json", dict(status="infrastructure_error", success=False,
                                            error=repr(exc), traceback=traceback.format_exc()))
        raise
