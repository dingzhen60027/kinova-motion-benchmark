"""Read-only adapter diagnostics from selected records and original arrays.

This does not rerun a planner, change success labels, select a better attempt,
or infer an optimizer's quality from its frontend's success. Run with ``-m``.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np

from ..common import ROOT, recorded_path, sha, write

METHODS = ("CHOMP", "STOMP", "TRAJOPT", "RRTCONNECT", "RRTSTAR")
COPY_FIELDS = ("record_id", "problem_id", "scene_id", "cluster_id", "split", "category",
    "method", "objective", "planner_seed", "attempt_number", "input_valid", "status",
    "failure_stage", "failure_reason", "frontend_validated", "validation_passed",
    "solver_reported_success", "solver_converged", "cold_e2e_seconds",
    "drm_path_seconds", "trajectory_optimization_seconds", "sampling_planning_seconds",
    "result_path")


def _array(path, key):
    with np.load(path, allow_pickle=False) as archive:
        value = archive[key]
    if (value.ndim != 2 or value.shape[1] != 7 or len(value) < 2
            or not np.issubdtype(value.dtype, np.number) or not np.isfinite(value).all()):
        raise ValueError(f"Invalid seven-joint polyline in {path}:{key}")
    return np.asarray(value, float)


def path_comparison(initial, final, coordinate_tolerance=1e-10, length_tolerance=1e-6):
    """Compare saved geometry, without claiming collision validity or optimality."""
    initial, final = np.asarray(initial, float), np.asarray(final, float)
    for path in (initial, final):
        if path.ndim != 2 or path.shape[1] != 7 or len(path) < 2 or not np.isfinite(path).all():
            raise ValueError("Expected finite N x 7 polylines")
    before = float(np.linalg.norm(np.diff(initial, axis=0), axis=1).sum())
    after = float(np.linalg.norm(np.diff(final, axis=0), axis=1).sum())
    lower = float(np.linalg.norm(initial[-1]-initial[0]))
    change = (float(np.max(np.abs(final-initial))) if initial.shape == final.shape else None)
    delta = after-before
    relation = ("unchanged" if change is not None and change <= coordinate_tolerance else
                "shorter" if delta < -length_tolerance else
                "longer" if delta > length_tolerance else "changed_equal_length")
    return dict(initial_length_rad=before, final_length_rad=after, length_delta_rad=delta,
        length_relative_delta=delta/before if before > 0 else None,
        endpoint_chord_rad=lower, initial_at_endpoint_lower_bound=abs(before-lower) <= length_tolerance,
        maximum_corresponding_knot_change_rad=change, relation=relation,
        endpoint_change_rad=float(np.max(np.abs(final[[0, -1]]-initial[[0, -1]]))),
        initial_knots=len(initial), final_knots=len(final))


def diagnose(record, comparison):
    row = {key: record.get(key, "") for key in COPY_FIELDS}
    row.update(comparison=comparison, invocation_evidence=False, candidate_evidence=False,
               diagnostic_error="")
    directory = recorded_path(record["result_path"]).parent
    method = record["method"]
    if method in ("CHOMP", "STOMP"):
        paths = (directory/"optimizer_initial.npz", directory/"optimizer_solution.npz",
                 directory/"optimizer_result.json")
        keys = ("initial", "points")
        row["comparison_scope"] = "optimizer output versus supplied validated DRM initialization"
    elif method == "TRAJOPT":
        paths = (directory/"trajopt_initial.npz", directory/"trajopt_solver_raw.npz",
                 directory/"trajopt_solver_result.json")
        keys = ("controls", "points")
        row["comparison_scope"] = "optimizer output versus supplied validated DRM initialization"
    else:
        paths = (directory/"ompl_raw_path.npz", directory/"ompl_simplified_path.npz",
                 directory/"ompl_result.json")
        keys = ("points", "points")
        row["comparison_scope"] = "native OMPL vertex reduction versus native sampled path"
    digests = {}
    try:
        for path in paths:
            if path.is_file():
                digests[str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)] = sha(path)
        row["invocation_evidence"] = (paths[0].is_file() if method in METHODS[:3] else paths[2].is_file())
        row["candidate_evidence"] = paths[1].is_file()
        if paths[2].is_file():
            native = json.loads(paths[2].read_text())
            row.update(native_stop=native.get("stop_reason", native.get("solver_status", native.get("native_status"))),
                native_reported_iterations=native.get("iterations"),
                native_reported_cost=native.get("native_cost", native.get("quadratic_smoothness")),
                native_reported_converged=native.get("solver_converged"))
        history = directory / f"{method.lower()}_iterations.jsonl"
        if history.is_file():
            row["observed_iteration_callbacks"] = sum(bool(line.strip()) for line in history.open())
        if paths[1].is_file():
            if not paths[0].is_file():
                raise ValueError("Candidate evidence exists but corresponding initialization is missing")
            row.update(path_comparison(_array(paths[0], keys[0]), _array(paths[1], keys[1])))
            # The benchmark cost is recomputed from arrays, never from a native
            # smoothness cost or from the row's rounded summary alone.
            if record.get("path_length_rad"):
                saved_cost = float(record["path_length_rad"])
                row["records_length_error_rad"] = abs(saved_cost-row["final_length_rad"])
                if row["records_length_error_rad"] > 1e-6:
                    raise ValueError("records.csv length disagrees with the returned array")
    except Exception as exc:
        row["diagnostic_error"] = f"{type(exc).__name__}: {exc}"
    row["artifact_sha256"] = json.dumps(digests, sort_keys=True, separators=(",", ":"))
    return row


def summarize(rows):
    summaries = []
    for comparison, method in sorted({(r["comparison"], r["method"]) for r in rows}):
        subset = [r for r in rows if (r["comparison"], r["method"]) == (comparison, method)]
        legal = [r for r in subset if str(r["input_valid"]).lower() == "true"]
        invoked = [r for r in legal if r["invocation_evidence"]]
        candidates = [r for r in invoked if r["candidate_evidence"] and not r["diagnostic_error"]]
        succeeded = [r for r in candidates if r["status"] == "success_numerical"]
        deltas = [r["length_relative_delta"] for r in succeeded if r.get("length_relative_delta") is not None]
        summaries.append(dict(comparison=comparison, method=method, registered_runs=len(subset),
            legal_runs=len(legal), unique_legal_problems=len({r["problem_id"] for r in legal}),
            legal_scene_clusters=len({r["cluster_id"] for r in legal}),
            statuses=dict(Counter(r["status"] for r in subset)),
            failure_stages=dict(Counter(r["failure_stage"] for r in legal if r["status"] != "success_numerical")),
            invocation_evidence_count=len(invoked), saved_candidate_count=len(candidates),
            candidate_relations=dict(Counter(r["relation"] for r in candidates)),
            unchanged_already_straight=sum(r["relation"] == "unchanged" and r["initial_at_endpoint_lower_bound"] for r in candidates),
            validated_successes_after_invocation=len(succeeded),
            success_given_invocation=len(succeeded)/len(invoked) if invoked else None,
            success_only_relative_length_delta_median=float(np.median(deltas)) if deltas else None,
            native_stops=dict(Counter(str(r.get("native_stop")) for r in invoked)),
            observed_iteration_callback_counts=dict(Counter(str(r.get("observed_iteration_callbacks")) for r in invoked)),
            diagnostic_errors=sum(bool(r["diagnostic_error"]) for r in subset)))
    return summaries


def audit(records, comparisons, out):
    records, out = Path(records).resolve(), Path(out).resolve()
    comparisons = list(dict.fromkeys(comparisons))
    if not comparisons:
        raise ValueError("Select at least one declared comparison")
    destinations = (out/"adapter_records.csv", out/"adapter_summary.json")
    if any(path.exists() for path in destinations):
        raise FileExistsError("Refusing to overwrite existing diagnostic evidence")
    rows, seen = [], set()
    with records.open(newline="") as stream:
        for record in csv.DictReader(stream):
            if record["method"] not in METHODS:
                continue
            selected = json.loads(record.get("selected_comparisons") or "[]")
            for comparison in comparisons:
                if comparison not in selected:
                    continue
                key = (comparison, record["record_id"])
                if key in seen:
                    raise ValueError("Duplicate selected record identity")
                seen.add(key)
                rows.append(diagnose(record, comparison))
    if any(not any(r["comparison"] == c for r in rows) for c in comparisons):
        raise ValueError("A requested comparison has no selected supported-method records")
    out.mkdir(parents=True, exist_ok=True)
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with destinations[0].open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    summary = dict(records=str(records.relative_to(ROOT)) if records.is_relative_to(ROOT) else str(records),
        records_sha256=sha(records), comparisons=comparisons, rows=len(rows),
        diagnostic_errors=sum(bool(row["diagnostic_error"]) for row in rows),
        semantics=dict(selection="Only records explicitly selected by each declared comparison; no best-of selection",
            geometry="Saved arrays only; no new collision checks or planner runs",
            invocation="Initialization archive for local optimizers; native OMPL result file for sampling methods",
            costs="Actual unwrapped seven-joint Euclidean PWL length; native objectives may differ",
            rates="Conditional optimizer diagnostics, not replacement benchmark success rates",
            intervals="No confidence intervals or cross-method superiority claims; rows share scene layouts",
            numerical_tolerances=dict(coordinate_change_rad=1e-10, length_comparison_rad=1e-6)),
        methods=summarize(rows))
    write(destinations[1], summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--records", type=Path, default=ROOT/"results/records.csv")
    parser.add_argument("--comparison", action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    summary = audit(args.records, args.comparison, args.output)
    print(json.dumps({k: summary[k] for k in ("rows", "diagnostic_errors", "comparisons")}, indent=2))
    return int(bool(summary["diagnostic_errors"]))


if __name__ == "__main__":
    raise SystemExit(main())
