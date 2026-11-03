"""Verify the repair evidence and ensure historical observations are unchanged.

Run from the repository with ``bash local/python.sh
benchmarks/generic_motion_v1/results/rigor_checks/check_publication.py``.
No planner or collision checker is invoked here.
"""
from __future__ import annotations

from collections import Counter
import csv
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))

from benchmarks.generic_motion_v1.common import ROOT, read, sha, write
from benchmarks.generic_motion_v1.batch import verify

HERE = Path(__file__).resolve().parent
RESULTS = HERE.parent
BASELINE = "fa2d9baa59154b8551c13d8d086bf832de2e9724"


def table(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def passed_tests(name):
    matches = re.findall(r"(?:^|\n)(\d+) passed[^\n]*", (HERE / name).read_text())
    if len(matches) != 1:
        raise ValueError(f"Expected one successful pytest result in {name}")
    return int(matches[0])


def pinned_table(name, expected_sha):
    data = subprocess.check_output(
        ["git", "show", f"{BASELINE}:benchmarks/generic_motion_v1/results/{name}"], cwd=ROOT)
    if hashlib.sha256(data).hexdigest() != expected_sha:
        raise ValueError(f"Pre-repair {name} snapshot does not match pinned Git bytes")
    return list(csv.DictReader(io.StringIO(data.decode())))


def summary_hash(row):
    # Only coverage status prose changed. Metric and paired scopes retain
    # their exact denominator/clock meaning and must compare verbatim.
    ignored = {"p95", "frontend_excluded_jobs"}
    if row["row_type"] == "status":
        ignored.add("scope")
    return hashlib.sha256(json.dumps(
        {k: v for k, v in row.items() if k not in ignored}, sort_keys=True
    ).encode()).hexdigest()


def main():
    before = read(HERE / "summary_before.json")
    records = table(RESULTS / "records.csv")
    current = {row["record_id"]: row for row in records}
    if len(current) != len(records):
        raise ValueError("Duplicate record identifiers")
    changed, old_ids = [], set()
    for row in pinned_table("records.csv", before["records_sha256"]):
        if row["record_id"] in old_ids:
            raise ValueError("Duplicate original record identifiers")
        old_ids.add(row["record_id"])
        if current.get(row["record_id"]) != row:
            changed.append(row["record_id"])
    regression = read(HERE / "baseline_regression.json")
    regression_csv = HERE / "baseline_regression_records.csv"
    if sha(regression_csv) != regression["records_sha256"]:
        raise ValueError("Regression CSV hash does not match verified evidence")
    expected_rows = table(regression_csv)
    expected_new = {row["record_id"] for row in expected_rows}
    actual_new = set(current) - old_ids
    checks = [verify(HERE / name) for name in ("baseline_bottleneck", "baseline_dense")]
    if changed or actual_new != expected_new or len(actual_new) != 28:
        raise ValueError("Historical record changed, or new records differ from the actual regressions")
    if any(current[row["record_id"]] != row or row["selected_comparisons"] != "[]"
           for row in expected_rows):
        raise ValueError("New regression rows changed or entered a primary comparison")
    if not regression["passed"] or not all(c["integrity_passed"] and c["complete"] for c in checks):
        raise ValueError("Incomplete or invalid regression artifacts")

    summary = table(RESULTS / "summary.csv")
    original_summary = pinned_table("summary.csv", before["summary_sha256"])
    row_hashes = Counter(map(summary_hash, summary))
    missing = Counter(map(summary_hash, original_summary)) - row_hashes
    if missing:
        raise ValueError("Pre-existing statistical values changed")
    initial_refresh = read(HERE / "summary_refresh_verification.json")
    if sha(RESULTS / "summary.csv") != initial_refresh["summary_sha256"]:
        raise ValueError("Regression queries entered the primary summary")

    model = read(HERE / "model_audit_fresh.json")
    inputs = read(HERE / "generator_reaudit_receipt.json")
    health = read(HERE / "health.json")
    acceptance = read(HERE / "completed_evaluation.json")
    catalog = read(RESULTS / "methods.json")["actual_coverage_evidence"]
    for value in (model, inputs, health, acceptance):
        if value.get("passed") is not True:
            raise ValueError("A required evidence check did not pass")
    if (acceptance["records_sha256"] != before["records_sha256"] or
            acceptance["summary_sha256"] != sha(RESULTS / "summary.csv")):
        raise ValueError("Historical acceptance is not bound to the unchanged records and statistics")
    if catalog["records_sha256"] != sha(RESULTS / "records.csv") or catalog["summary_sha256"] != sha(RESULTS / "summary.csv"):
        raise ValueError("Method catalog does not bind the final indexes")
    input_count = sum(v["problem_count"] for v in inputs["splits"].values())
    proposals = sum(v["endpoint_proposals_replayed"] for v in inputs["splits"].values())
    if not all(v["input_manifest_unchanged"] for v in inputs["splits"].values()):
        raise ValueError("Frozen input manifest changed")

    runtimes = {read(HERE / name / "run_manifest.json")["source_code_commit"]
                for name in ("baseline_bottleneck", "baseline_dense")}
    if runtimes != {"84d9f8eb33b1a5501f5e947a7e3c37dc9a744b0b"}:
        raise ValueError("Regression runtime differs from the reviewed repair commit")
    test_seeds = sorted({job["replica_seed"] for job in
                         read(RESULTS / "baseline_test/run_manifest.json")["jobs"]})
    if test_seeds != [0]:
        raise ValueError("Update the documented test repetition scope")
    primary_calls = sum(v["completed_jobs"] for v in acceptance["coverage"]
                        if Path(v["run"]).name != "baseline_initial_regions")
    tests = dict(regression=passed_tests("regression_final.log"),
                 upstream=passed_tests("upstream_tests.log"))
    if tests != dict(regression=1330, upstream=3):
        raise ValueError("Test logs do not cover the declared repair regression suite")

    receipt = dict(
        passed=True, base_commit=BASELINE, planning_runtime_commit=next(iter(runtimes)),
        input_checks=dict(registered_problems=input_count, endpoint_proposals_replayed=proposals,
                          original_manifest_hashes_unchanged=True),
        model_check_status=model["status"], contact_band_m=model["contact_band_m"],
        tests=tests,
        historical_records_checked=len(old_ids), historical_records_changed=len(changed),
        new_regression_records=len(actual_new), regression_outcomes=regression["outcome_counts"],
        complete_historical_primary_calls=primary_calls,
        complete_historical_selected_calls=sum(v["completed_jobs"] for v in acceptance["coverage"]),
        completed_formal_test_seeds=test_seeds,
        historical_summary_rows=before["row_count"], current_summary_rows=len(summary),
        historical_summary_values_changed=sum(missing.values()),
        records_sha256=sha(RESULTS / "records.csv"), summary_sha256=sha(RESULTS / "summary.csv"),
        evidence_sha256={str(p.relative_to(HERE)): sha(p) for p in (
            HERE / "generator_reaudit_receipt.json", HERE / "model_audit_fresh.json",
            HERE / "health.json", HERE / "baseline_regression.json",
            HERE / "nto_saved_dev_diagnostic/summary.json", HERE / "completed_evaluation.json",
            HERE / "regression_final.log", HERE / "upstream_tests.log",
        )},
        limitations=[
            "Formal test currently has seed 0 only; seeds 1 and 2 have not been run.",
            "Collision validation is numerical; no continuous mathematical certificate.",
            "NTO_PAPER collocation can miss between-point collisions; those outputs remain failures.",
            "Regression timings overlapped CPU postprocessing and are outside primary performance comparisons.",
            "No external public benchmark was run in this repair task.",
        ],
        retained_diagnostic_errors=[
            "regression_after.log: obsolete pending-category test assertion; corrected and full regression passed.",
            "completed_evaluation.failed.json: unsupported conditional-comparison invocation; accepted primary invocation passed.",
            "nto_saved_dev_diagnostic/selection_probe_failure.log: initial report selection call omitted comparison id.",
            "baseline_regression.json: initial summary script referenced a nonexistent field; corrected using batch.verify.",
            "publication_check_import_failure.log: direct-script package search path missing; corrected before publication verification.",
        ],
    )
    write(HERE / "checks.json", receipt)
    print(json.dumps({k: v for k, v in receipt.items() if k != "evidence_sha256"}, indent=2))


if __name__ == "__main__":
    main()
