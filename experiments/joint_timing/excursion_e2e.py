"""Sealed paired 12-query Dev pilot with a cold subprocess and a shared GPU lock."""

import argparse
from collections import Counter
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.common import (
    BASE,
    ROOT,
    code_inventory,
    read,
    sha,
    write,
)
from benchmarks.generic_motion_v1.runners.results import _receipt_check
from . import verify_witnesses as witnesses

OUT = ROOT / "runs/phase_search/excursion_e2e"
DEV = ROOT / "runs/phase_search/dev/run_manifest.json"
WORKER = Path(__file__).with_name("excursion_worker.py")
ARMS = ("guided", "excursion")
THREADS = {
    "CUDA_VISIBLE_DEVICES": "0",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "4",
}


def require(ok, detail):
    if not ok:
        raise ValueError(detail)


def design():
    dev = read(DEV)
    inputs = read(witnesses.PILOT / "protocol.json")
    cases = inputs["cases"]
    job_of = {
        j["case_id"]: j
        for j in dev["jobs"]
        if j["method"] == "PHASE_SEARCH"
        and j["objective"] == "minimum_time"
        and j["replica_seed"] == 0
        and j["case_id"] in cases
    }
    require(
        len(cases) == len(set(cases)) == len(job_of) == 12,
        "Unexpected experiment cohort",
    )
    jobs = [
        {"arm": arm, "source_job": job_of[pid]}
        for i, pid in enumerate(cases)
        for arm in (ARMS if i % 2 == 0 else ARMS[::-1])
    ]
    require(code_inventory() == dev["source_hashes"], "Frozen planner sources changed")
    files = [
        Path(__file__),
        WORKER,
        Path(witnesses.__file__),
        DEV,
        witnesses.PILOT / "protocol.json",
        ROOT / "configs/evaluation.json",
    ]
    files.extend(
        WORKER.with_name(n)
        for n in ["excursion_pilot.py", "excursions.py", "cohort.py", "paired.py"]
    )
    return dict(
        jobs=jobs,
        budget_seconds=60,
        split="dev",
        threads=THREADS,
        algorithm_commit=dev["source_code_commit"],
        source_sha256={str(f.relative_to(ROOT)): sha(f) for f in files},
        reference_mode="first sealed terminal, paired problem and seed, numerical validation",
    )


def manifest(prepare=False):
    current = design()
    path = OUT / "protocol.json"
    if not path.exists() and prepare:
        OUT.mkdir(parents=True, exist_ok=True)
        with path.open("x") as f:
            json.dump(current, f, indent=2, allow_nan=False)
    require(read(path) == current, "Pilot protocol or source fingerprint differs")
    return current


def prerequisites():
    finished = ROOT / "runs/phase_search/test/verification.json"
    if not finished.exists():
        return False
    rep = read(finished)
    require(
        rep["integrity_passed"] and rep["complete"], "Test verification is incomplete"
    )
    items, _, _ = witnesses.inventory(("pilot", "dev-remainder"))
    counts = Counter()
    for _, _, key in items:
        target = witnesses.target(key)
        if not target.exists():
            return False
        status = witnesses.existing(target, key)["status"]
        require(
            status != "infrastructure_error",
            "Witness verification has an infrastructure failure",
        )
        if key["cohort"] == "pilot":
            counts[status] += 1
    require(
        counts["success_validated_numerical"] > 0,
        "No pilot witness passed canonical validation",
    )
    return True


def paths(job):
    base = OUT / "jobs" / job["arm"] / job["source_job"]["case_id"]
    return base, sorted(base.glob("[0-9][0-9][0-9][0-9]"))


def seal(attempt, result, retry=False):
    files = {
        str(p.relative_to(attempt)): sha(p) for p in attempt.rglob("*") if p.is_file()
    }
    require("receipt.json" not in files, "Refusing to replace receipt")
    write(
        attempt / "receipt.json",
        dict(status=result["status"], files=files, retry_pending=retry),
    )


def outcome(job):
    base, attempts = paths(job)
    terminal = []
    for attempt in attempts:
        if not (attempt / "receipt.json").exists():
            require(
                not (attempt / "process.json").exists(),
                "Unsealed finished process requires inspection: " + str(attempt),
            )
            continue
        receipt = read(attempt / "receipt.json")
        require(
            not _receipt_check(attempt, receipt), "Corrupt attempt: " + str(attempt)
        )
        require(
            read(attempt / "request.json") == job["source_job"],
            "Frozen source job differs",
        )
        if not receipt.get("retry_pending"):
            result = read(attempt / "result.json")
            require(
                receipt["status"] == result["status"], "Status differs from receipt"
            )
            launch = read(attempt / "launch.json")
            require(
                launch["arm"] == result.get("pilot_arm") == job["arm"]
                and launch["budget_seconds"] == 60,
                "Sealed arm identity mismatch",
            )
            terminal.append((attempt, result))
    require(len(terminal) <= 1, "Multiple terminal outcomes")
    return terminal[0] if terminal else None


def execute(job):
    base, prior = paths(job)
    for a in prior:
        if not (a / "receipt.json").exists():
            require(
                (a / "request.json").exists()
                and read(a / "request.json") == job["source_job"],
                "Interrupted attempt identity mismatch",
            )
            partial = a / "result.json"
            if partial.exists():
                partial.rename(a / "result_before_interruption.json")
            interrupted = dict(
                status="infrastructure_error",
                error_category="coordinator_interrupted",
                pilot_arm=job["arm"],
            )
            write(partial, interrupted)
            seal(a, interrupted, retry=True)
    attempt = base / f"{len(prior)+1:04d}"
    attempt.mkdir(parents=True)
    req = job["source_job"]
    write(attempt / "request.json", req)
    argv = [
        sys.executable,
        "-m",
        "experiments.joint_timing.excursion_worker",
        "--arm",
        job["arm"],
        "--attempt",
        str(attempt),
    ]
    write(attempt / "launch.json", dict(argv=argv, arm=job["arm"], budget_seconds=60))
    with (attempt / "stdout.log").open("w") as out, (attempt / "stderr.log").open(
        "w"
    ) as err:
        start = time.perf_counter()
        proc = subprocess.Popen(
            argv,
            cwd=ROOT,
            env=dict(os.environ, **THREADS),
            stdout=out,
            stderr=err,
            start_new_session=True,
        )
        timeout = False
        try:
            proc.wait(timeout=max(0.0, 60 - (time.perf_counter() - start)))
        except subprocess.TimeoutExpired:
            timeout = True
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait()
        elapsed = time.perf_counter() - start
    details = dict(
        argv=argv,
        exit_code=proc.returncode,
        timed_out=timeout,
        cold_e2e_seconds=elapsed,
        timeout_seconds=60,
    )
    path = attempt / "result.json"
    result = read(path) if path.exists() else {}
    if timeout or proc.returncode or not result:
        if result:
            write(attempt / "result_before_external_stop.json", result)
        batch._external_stop_outcome(
            result,
            timed_out=timeout,
            returncode=proc.returncode,
            attempt=attempt,
            config=req["evaluation_config"],
        )
    require(result["status"] in batch.TERMINAL_STATUSES, "Unknown worker status")
    if result["status"] == "success_validated_numerical":
        require(
            read(attempt / "validation.json")["passed"] is True
            and (attempt / "trajectory_samples.npz").exists(),
            "Successful worker lacks validation evidence",
        )
    result.update(cold_e2e_seconds=elapsed, pilot_arm=job["arm"])
    write(path, result)
    write(attempt / "process.json", details)
    seal(attempt, result)
    return result


def run():
    contract = manifest()
    print(
        "Waiting for frozen Test and all 81 excursion validation receipts", flush=True
    )
    while not prerequisites():
        time.sleep(15)
    with (BASE / "results/.gpu.lock").open("r+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        for j in contract["jobs"]:
            manifest()
            previous = outcome(j)
            r = previous[1] if previous else execute(j)
            print(j["arm"], j["source_job"]["case_id"], r["status"], flush=True)
            require(
                not batch.exception_needs_attention(r),
                "Infrastructure error preserved; inspect the attempt",
            )
        summary = status(contract)
        path = OUT / "summary.json"
        if path.exists():
            require(read(path) == summary, "Sealed summary changed")
        else:
            write(path, summary)


def status(contract):
    rows = []
    counts = {arm: Counter() for arm in ARMS}
    for j in contract["jobs"]:
        o = outcome(j)
        r = o[1] if o else {}
        counts[j["arm"]][r.get("status", "pending")] += 1
        rows.append(
            dict(
                arm=j["arm"],
                case=j["source_job"]["case_id"],
                status=r.get("status", "pending"),
                duration_s=r.get("metrics", {}).get("duration_s"),
                cold_s=r.get("cold_e2e_seconds"),
                receipt_sha256=sha(o[0] / "receipt.json") if o else None,
            )
        )
    return dict(counts=counts, records=rows)


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument("action", choices=["prepare", "run", "status"])
    cmd = a.parse_args().action
    if cmd == "run":
        run()
    else:
        contract = manifest(prepare=cmd == "prepare")
        print(json.dumps(status(contract)["counts"]))


if __name__ == "__main__":
    main()
