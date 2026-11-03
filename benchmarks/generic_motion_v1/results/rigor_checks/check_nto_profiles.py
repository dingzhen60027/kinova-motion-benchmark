#!/usr/bin/env python3
"""Read-only receipt audit of the completed NTO dev and test comparisons.

Run from any directory with --output pointing to a NEW JSON file. All input
artifacts remain untouched. No planner, collision query, or GPU is invoked.
"""
from __future__ import annotations

import argparse
import collections
import csv
import datetime as dt
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[4]
RESULTS = ROOT / "benchmarks/generic_motion_v1/results"
METHODS = ("NTO_PAPER", "NTO_REFINED", "NTO")
OBJECTIVES = ("minimum_distance", "minimum_time")
STATUS_NAMES = (
    "success_numerical", "invalid_input", "no_solution_within_budget", "timeout",
    "solver_failure", "validation_failed", "infrastructure_error",
)


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def counter(values):
    return dict(sorted(collections.Counter(values).items()))


def read_json(path):
    return json.loads(path.read_text())


def audit():
    errors = []
    file_count = 0
    byte_count = 0
    sources = [RESULTS / name for name in ("records.csv", "summary.csv", "selection.json")]
    sources.extend(RESULTS / f"baseline_{split}/run_manifest.json" for split in ("dev", "test"))
    source_hashes = {str(path.relative_to(ROOT)): sha(path) for path in sources}
    report = {
        "schema_version": 1,
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "script": str(Path(__file__).resolve().relative_to(ROOT)),
        "script_sha256": sha(Path(__file__)),
        "sources_sha256": source_hashes,
        "scope": "Completed baseline_dev (paired replicas 0,1,2) and baseline_test (replica 0), three NTO profiles and both objectives; all registered jobs included.",
        "notes": [
            "Numerical success requires saved independent validation, not solver convergence. It is not a mathematical continuous collision certificate.",
            "Native codes are the last saved solver result. For interrupted refinement, that may precede the interrupted solve; missing output is not a fabricated exit code.",
            "Frontend comparison checks actual receipt-bound path.npy bytes, as well as input/model/geometry/planner-seed/roadmap identities.",
            "Every receipt-listed file is rehashed, including raw arrays and logs. All inspected JSON must be included in its receipt.",
        ],
        "cohorts": [], "frontend_groups": [], "paired_status_changes": [],
    }
    with (RESULTS / "records.csv").open(newline="") as stream:
        rows = [r for r in csv.DictReader(stream)
                if r["method"] in METHODS and Path(r["run"]).name in ("baseline_dev", "baseline_test")]
    with (RESULTS / "summary.csv").open(newline="") as stream:
        summaries = list(csv.DictReader(stream))
    selection = read_json(RESULTS / "selection.json")
    data = {}
    manifests = {}
    for split in ("dev", "test"):
        run = RESULTS / f"baseline_{split}"
        manifest = read_json(run / "run_manifest.json")
        manifests[split] = manifest
        expected = {j["job_id"]: j for j in manifest["jobs"] if j["method"] in METHODS}
        # Path(r['run']) is relative to ROOT, never the caller's working directory.
        selected = [r for r in rows if (ROOT / r["run"]).resolve() == run.resolve()]
        actual = counter(r["job_id"] for r in selected)
        if set(actual) != set(expected) or any(n != 1 for n in actual.values()):
            errors.append({"kind": "job_coverage", "split": split,
                           "missing": sorted(set(expected) - set(actual)),
                           "extra": sorted(set(actual) - set(expected)),
                           "duplicate_jobs": {k: v for k, v in actual.items() if v != 1}})
        comparison_id = f"current_{split}_repaired"
        comps = [c for c in selection["comparisons"] if c["id"] == comparison_id]
        if len(comps) != 1 or comps[0].get("mode") != "end_to_end":
            errors.append({"kind": "selection_comparison", "comparison": comparison_id})
        elif len(comps[0]["runs"]) != 1 or (ROOT / comps[0]["runs"][0]["path"]).resolve() != run.resolve():
            errors.append({"kind": "selection_run", "comparison": comparison_id})
        for row in selected:
            attempt = ROOT / row["attempt_path"]
            key = row["record_id"]
            try:
                receipt_path = attempt / "receipt.json"
                if sha(receipt_path) != row["receipt_sha256"]:
                    raise ValueError("receipt SHA differs from records.csv")
                receipt = read_json(receipt_path)
                for relative, expected_sha in receipt["files"].items():
                    path = attempt / relative
                    if not path.resolve().is_relative_to(attempt.resolve()):
                        raise ValueError(f"receipt path escapes attempt: {relative}")
                    if sha(path) != expected_sha:
                        raise ValueError(f"receipt artifact SHA mismatch: {relative}")
                    file_count += 1
                    byte_count += path.stat().st_size
                def artifact(name, required=False):
                    path = attempt / name
                    if not path.exists() and not required:
                        return {}
                    if name not in receipt["files"]:
                        raise ValueError(f"inspected JSON absent from receipt: {name}")
                    return read_json(path)
                raw = {name: artifact(name, name in ("result.json", "request.json")) for name in (
                    "result.json", "request.json", "candidate.json", "validation.json",
                    "nto_solver_result.json", "phase.json", "frontend_validation/validation.json",
                )}
                result = raw["result.json"]
                if row["raw_status"] != result["status"] or receipt["status"] != result["status"]:
                    raise ValueError("raw status differs between receipt/result/index")
                if row["source_commit"] != manifest["source_code_commit"]:
                    raise ValueError("source commit differs from run manifest")
                if row["run_manifest_sha256"] != source_hashes[str((run / "run_manifest.json").relative_to(ROOT))]:
                    raise ValueError("run manifest SHA differs from records.csv")
                if raw["request.json"] != expected[row["job_id"]]:
                    raise ValueError("receipt-bound request differs from run manifest job")
                if row["attempt_number"] != "1" or row["lifecycle"] != "terminal":
                    raise ValueError("expected a single first terminal attempt")
                if comparison_id not in json.loads(row["selected_comparisons"]):
                    raise ValueError("row omitted from expected selected comparison")
                if row["frontend_path_sha256"] != receipt["files"].get("path.npy", ""):
                    raise ValueError("frontend path SHA differs from receipt")
                if row["frontend_validated"] == "True" and raw["frontend_validation/validation.json"].get("passed") is not True:
                    raise ValueError("frontend marked valid without passed validation")
                if row["status"] == "success_numerical" and raw["validation.json"].get("passed") is not True:
                    raise ValueError("numerical success lacks passed validation")
                if row["solver_reported_success"]:
                    if str(raw["candidate.json"].get("solver_success")) != row["solver_reported_success"]:
                        raise ValueError("candidate solver success differs from index")
                data[key] = raw
            except (OSError, ValueError, KeyError, TypeError) as exc:
                errors.append({"kind": "attempt_integrity", "attempt": row["attempt_path"], "error": str(exc)})
    report["integrity"] = {"registered_rows": len(rows), "verified_attempts": len(data),
                           "verified_receipt_files": file_count, "verified_bytes": byte_count}
    for split in ("dev", "test"):
        for method in METHODS:
            for objective in OBJECTIVES:
                registered = [r for r in rows if r["split"] == split and r["method"] == method and r["objective"] == objective]
                legal = [r for r in registered if r["input_valid"] == "True"]
                item = {"split": split, "method": method, "objective": objective,
                        "registered": len(registered), "legal": len(legal),
                        "replica_seeds": counter(r["replica_seed"] for r in legal),
                        "statuses": counter(r["status"] for r in legal),
                        "raw_statuses": counter(r["raw_status"] for r in legal),
                        "frontend_validated": counter(r["frontend_validated"] for r in legal),
                        "source_commits": counter(r["source_commit"] for r in legal),
                        "attempts": counter(r["attempt_number"] for r in legal),
                        "convergence_vs_status": counter(f"solver_success={r['solver_reported_success'] or 'missing'} | {r['status']}" for r in legal)}
                native, failed_validation, timeout_phase, phases, rejected_reasons = [], [], [], [], []
                for row in legal:
                    if row["record_id"] not in data:
                        continue
                    raw = data[row["record_id"]]
                    solver, validation = raw["nto_solver_result.json"], raw["validation.json"]
                    details = solver.get("native_solver_details", {})
                    code = details.get("info", details.get("status", "missing"))
                    native.append(f"{row['status']} | {code} | {solver.get('native_status_description', solver.get('solver_status', 'missing'))}")
                    if row["status"] == "validation_failed":
                        signature = {k: validation.get(k) for k in (
                            "status", "collision_free", "joint_limits_passed", "derivative_limits_passed")}
                        signature["endpoint_within_1e_5_rad"] = validation.get("endpoint_error_rad", float("inf")) <= 1e-5
                        signature["has_contact_band_samples"] = validation.get("contact_band_samples", 0) > 0
                        failed_validation.append(json.dumps(signature, sort_keys=True))
                    if row["status"] == "timeout":
                        timeout_phase.append(raw["phase.json"].get("phase", "missing"))
                    if row["status"] != "success_numerical":
                        phases.append(f"{row['status']} | recorded_failure_stage={row['failure_stage'] or 'missing'} | last_raw_phase={raw['phase.json'].get('phase', 'missing')}")
                    if row["status"] == "no_solution_within_budget":
                        rejected_reasons.append(json.dumps({k: raw["result.json"][k] for k in (
                            "status", "reason", "failure_reason", "final_stage") if k in raw["result.json"]}, sort_keys=True))
                item.update(native_code_by_status=counter(native), validation_failure_signature=counter(failed_validation),
                            timeout_phase=counter(timeout_phase), failure_stages=counter(phases), frontend_failure_reasons=counter(rejected_reasons))
                matching = [r for r in summaries if r["comparison"] == f"current_{split}_repaired"
                            and r["category"] == "all" and r["row_type"] == "status"
                            and r["method"] == method and r["objective"] == objective]
                expected_counts = dict(planned_jobs=len(registered), actual_attempts=len(registered),
                                       legal_jobs=len(legal), finished_legal_jobs=len(legal), pending_jobs=0,
                                       invalid_input_jobs=len(registered)-len(legal))
                expected_counts.update({name: item["statuses"].get(name, 0) for name in STATUS_NAMES if name != "invalid_input"})
                item["summary_counts_match"] = len(matching) == 1 and all(
                    matching[0][name] == str(value) for name, value in expected_counts.items())
                if not item["summary_counts_match"]:
                    errors.append({"kind": "summary_counts", "split": split, "method": method,
                                   "objective": objective, "expected": expected_counts, "actual": matching})
                report["cohorts"].append(item)
        grouped = collections.defaultdict(list)
        for row in rows:
            if row["split"] == split and row["input_valid"] == "True":
                grouped[row["problem_id"], row["replica_seed"]].append(row)
        missing, mismatches, valid_count, unavailable = [], [], 0, []
        identities = ("frontend_path_sha256", "frontend_validated", "input_sha256", "robot_sha256",
                      "geometry_sha256", "planner_seed", "roadmap_id")
        for key, group in grouped.items():
            if len(group) != 6:
                missing.append(list(key))
            unequal = {name: sorted({r[name] for r in group}) for name in identities if len({r[name] for r in group}) != 1}
            if unequal:
                mismatches.append({"problem_seed": list(key), "fields": unequal})
            if all(r["frontend_validated"] == "True" for r in group):
                valid_count += 1
            elif all(not r["frontend_validated"] for r in group):
                unavailable.append(" | ".join(sorted({r["status"] for r in group})))
        report["frontend_groups"].append({"split": split, "groups": len(grouped),
            "all_six_same_validated_path": valid_count if not mismatches else None,
            "all_six_without_frontend": len(unavailable), "without_frontend_statuses": counter(unavailable),
            "missing_method_track_rows": missing, "mismatches": mismatches})
        if missing or mismatches:
            errors.append({"kind": "paired_frontend_mismatch", "split": split})
        for objective in OBJECTIVES:
            by_method = {method: {(r["problem_id"], r["replica_seed"]): r for r in rows
                         if r["split"] == split and r["objective"] == objective
                         and r["method"] == method and r["input_valid"] == "True"} for method in METHODS[:2]}
            transitions = counter(f"{row['status']} -> {by_method['NTO_REFINED'][key]['status']}"
                                  for key, row in by_method["NTO_PAPER"].items() if key in by_method["NTO_REFINED"])
            report["paired_status_changes"].append({"split": split, "objective": objective,
                                                   "from": "NTO_PAPER", "to": "NTO_REFINED", "transitions": transitions})
    changed = [name for name, digest in source_hashes.items() if sha(ROOT / name) != digest]
    if changed:
        errors.append({"kind": "inputs_changed_during_audit", "files": changed})
    report.update(passed=not errors, errors=errors)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New JSON output; existing paths are never overwritten")
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error(f"output already exists: {output}")
    try:
        report = audit()
    except Exception as exc:
        report = {"passed": False, "errors": [{"kind": "audit_exception", "type": type(exc).__name__, "error": str(exc)}]}
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": str(output), "passed": report["passed"],
                      "errors": len(report["errors"]), "integrity": report.get("integrity")}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
