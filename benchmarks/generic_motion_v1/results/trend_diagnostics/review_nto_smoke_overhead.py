"""Compare saved NTO smoke evidence without evaluating models or solvers."""
from __future__ import annotations

import hashlib
import json
import re
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
RESULTS = ROOT / "results"


def read(path):
    return json.loads(path.read_text()) if path.exists() else {}


def inspect(run, problem, method="NTO_REFINED"):
    matches = list((RESULTS / run / "jobs").glob(f"{problem}-{method}-minimum_time-*"))
    if not matches:
        return None
    attempt = matches[0] / "attempts" / "0001"
    result = read(attempt / "result.json")
    process = read(attempt / "process.json")
    native = attempt / "refinement" / "round_00" if method == "NTO_REFINED" else attempt
    params = read(native / "nto_parameters.json")
    log_path = native / "ipopt.log"
    log = log_path.read_text(errors="replace") if log_path.exists() else ""
    rows = []
    for line in log.splitlines():
        fields = line.split()
        if len(fields) != 10 or not re.fullmatch(r"\d+r?", fields[0]):
            continue
        try:
            row = {"iteration": int(fields[0].rstrip("r")),
                   "restoration": fields[0].endswith("r"),
                   "objective": float(fields[1]),
                   "primal_infeasibility": float(fields[2]),
                   "dual_infeasibility": float(fields[3]),
                   "line_search_trials": int(fields[-1])}
        except ValueError:
            continue
        rows.append(row)
    hashes = {name: hashlib.sha256((attempt / name).read_bytes()).hexdigest()
              for name in ["path.npy", "nto_initial.npz"] if (attempt / name).exists()}
    if (native / "nto_initial.npz").exists():
        hashes["native_initial.npz"] = hashlib.sha256((native / "nto_initial.npz").read_bytes()).hexdigest()
    backend = read(native / "nto_collision_backend.json")
    return {"run": run, "problem_id": problem, "method": method,
            "attempt": str(attempt.relative_to(ROOT)), "status": result.get("status"),
            "timing": result.get("timing", {}),
            "process": process,
            "native_solve_started": log_path.exists(),
            "first_complete_iteration": rows[0] if rows else None,
            "last_complete_iteration": rows[-1] if rows else None,
            "complete_iteration_rows": len(rows),
            "iteration_rows_sha256": hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest() if rows else None,
            "parameters": {k: params.get(k) for k in ["warm_start_control_count", "warm_start_node_count", "warm_start_duration_seconds", "construction_seconds", "collision_pairs", "collocation_points"]},
            "solver_parameters": params.get("parameters"), "artifact_sha256": hashes,
            "native_evaluations": backend.get("native_evaluations"),
            "recovery_events": backend.get("recovery_events", [])}


def main():
    sparse = [{"problem_id": problem,
               "records": [inspect(run, problem) for run in
                           ["smoke", "rss2025_reliable_v2_smoke_s000", "repaired_smoke"]]}
              for problem in ["smoke-sparse_clutter-000-p00", "smoke-sparse_clutter-000-p01"]]
    comparable = [inspect(run, "smoke-bottleneck-000-p00", "NTO_PAPER")
                  for run in ["smoke", "repaired_smoke"]]
    before, after = comparable
    if not before["complete_iteration_rows"] or not after["complete_iteration_rows"]:
        raise ValueError("Cannot compare solver iteration histories with no parsed iteration rows")
    old_seconds = before["timing"]["trajectory_optimization_seconds"]
    new_seconds = after["timing"]["trajectory_optimization_seconds"]
    output = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scope": __doc__, "gpu_calls": 0, "solver_calls": 0,
        "microbenchmark_performed": False,
        "diagnostic_parser_correction": "IPOPT emits 10 fields per iteration. The initial diagnostic required 11 and produced empty parsed histories; corrected and rechecked against all 415 rows (iterations 0..414). Original solver logs were never changed.",
        "frozen_runtime_commit": "7284d06f75aa6ba5ed5bf2b987d9ff287a2e8ab3",
        "sparse_timeout_comparison": sparse,
        "same_input_completed_comparison": {
            "records": comparable,
            "path_bytes_equal": before["artifact_sha256"]["path.npy"] == after["artifact_sha256"]["path.npy"],
            "initial_bytes_equal": before["artifact_sha256"]["nto_initial.npz"] == after["artifact_sha256"]["nto_initial.npz"],
            "iteration_rows_equal": before["iteration_rows_sha256"] == after["iteration_rows_sha256"],
            "optimization_seconds_delta": new_seconds - old_seconds,
            "optimization_seconds_ratio": new_seconds / old_seconds,
            "caveat": "Single historical and current measurement; not an isolated performance benchmark. Timing difference cannot be assigned wholly to Python wrapper."
        },
        "normal_path_source_review": {
            "path": "benchmarks/generic_motion_v1/runners/nto_collision_constraint.py",
            "sha256": hashlib.sha256((ROOT / "benchmarks/generic_motion_v1/runners/nto_collision_constraint.py").read_bytes()).hexdigest(),
            "normal_eval_per_pair_python_scan": False,
            "normal_eval_disk_write": False,
            "native_constraint_and_gradient_used": True,
            "extra_normal_work": ["Python callback dispatch and shape check", "AutoDiff scalar type detection", "Value extraction and finiteness check", "AutoDiff gradient extraction and finiteness check", "Native evaluation counter increment"],
            "fallback_only_work": ["All collision-pair queries", "Near-contact OBB recovery", "Finite-difference gradient recovery", "Recovery-event JSON persistence"]
        },
        "conclusions": [
            "Both old sparse cases failed in frontend and never optimized. They do not provide native-vs-wrapper timing comparisons.",
            "Both new sparse cases perform over 1500 IPOPT iterations and remain in restoration with nonzero primal residual at watchdog cutoff. No backend exception or recovery loop is present in their saved evidence.",
            "NTO_PAPER bottleneck provides a matched input/control initialization and equal iteration-count comparison with observable elapsed-time increase; its single pair does not isolate the cause.",
            "Normal Python wrapper overhead exists by construction and remains unmeasured separately; do not call either timeout a wrapper performance bug or claim zero overhead."
        ]
    }
    target = RESULTS / "trend_diagnostics" / "nto_smoke_normal_path_review.json"
    target.write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps({"output": str(target.relative_to(ROOT)),
                      "matched_optimization_ratio": new_seconds / old_seconds,
                      "matched_iteration_rows_equal": output["same_input_completed_comparison"]["iteration_rows_equal"]}))


if __name__ == "__main__":
    main()
