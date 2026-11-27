#!/usr/bin/env python3
"""Compare sealed candidate runs against the completed, paired benchmark."""

import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from benchmarks.generic_motion_v1.common import ROOT, read, resolve_case, sha, write
from benchmarks.generic_motion_v1.batch import _first_terminal_attempt
from benchmarks.generic_motion_v1.runners.results import (
    SUCCESS,
    attempt_row,
    _fingerprint,
)
from benchmarks.generic_motion_v1.runners.summarize import _cluster_ratio


def observed(run, replica_seed=None):
    manifest_file = run / "run_manifest.json"
    manifest = read(manifest_file)
    manifest["_index_manifest_sha256"] = sha(manifest_file)
    rows = []
    planned = Counter()
    case_cache = {}
    for job in manifest["jobs"]:
        if replica_seed is not None and job["replica_seed"] != replica_seed:
            continue
        case = resolve_case(job, case_cache)
        if case["validation_status"] != "valid_input":
            continue
        objective = job["objective"]
        planned[objective] += 1
        paths = (run / "jobs" / job["job_id"] / "attempts").glob("[0-9][0-9][0-9][0-9]")
        terminal = _first_terminal_attempt(
            sorted(p for p in paths if (p / "receipt.json").is_file())
        )
        if terminal is None:
            continue
        r = attempt_row(
            run, manifest, job, terminal, verify_files=True, case_cache=case_cache
        )
        if r["receipt_integrity"] is not True or r["status"] == "infrastructure_error":
            raise ValueError(
                f"Invalid evidence or infrastructure outcome in {terminal}"
            )
        trace_path = terminal / "phase_search.json"
        trace = read(trace_path) if trace_path.is_file() else {}
        r["candidate_source"] = trace.get("candidate_source")
        r["physical_lower_bound"] = trace.get("objective_lower_bound")
        if r["status"] == SUCCESS and (
            r["validation_passed"] is not True or r["objective_value"] is None
        ):
            raise ValueError(f"Missing numerical validation in {terminal}")
        r["common_config_sha256"] = _fingerprint(
            {k: v for k, v in job["evaluation_config"].items() if k != "phase_search"}
        )
        rows.append(r)
    return rows, planned, manifest


def compare(rows, planned, baselines):
    groups = defaultdict(list)
    for r in rows:
        for ref in baselines[(r["problem_id"], int(r["replica_seed"]), r["objective"])]:
            if ref["config_sha256"] != r["common_config_sha256"]:
                raise ValueError("Resolved benchmark configurations differ")
            for key in (
                "input_sha256",
                "validation_protocol_sha256",
                "robot_sha256",
                "geometry_sha256",
                "planner_seed",
                "budget_seconds",
                "cluster_id",
                "execution_mode",
            ):
                if str(ref[key]) != str(r[key]):
                    raise ValueError(f"Pairing mismatch for {r['problem_id']}, {key}")
            a = r["status"] == SUCCESS
            b = ref["status"] == SUCCESS
            groups[(r["objective"], ref["method"])].append(
                dict(
                    cluster_id=r["cluster_id"],
                    a=a,
                    b=b,
                    delta=(
                        float(r["objective_value"]) - float(ref["objective_value"])
                        if a and b
                        else None
                    ),
                    our_cost=float(r["objective_value"]) if a and b else None,
                    ref_cost=float(ref["objective_value"]) if a and b else None,
                    wall_delta=float(r["cold_e2e_seconds"])
                    - float(ref["cold_e2e_seconds"]),
                )
            )
    paired = []
    for (obj, method), data in sorted(groups.items()):
        common = [d for d in data if d["delta"] is not None]
        sr = _cluster_ratio(data, lambda d: int(d["a"]) - int(d["b"]), lambda d: 1)
        cost = _cluster_ratio(common, lambda d: d["delta"], lambda d: 1)
        wall = _cluster_ratio(data, lambda d: d["wall_delta"], lambda d: 1)
        paired.append(
            dict(
                objective=obj,
                reference=method,
                expected_jobs=planned[obj],
                completed_pairs=len(data),
                candidate_success=sum(d["a"] for d in data),
                reference_success=sum(d["b"] for d in data),
                common_success=len(common),
                success_rate_delta=sr["estimate"],
                success_ci95=sr["ci95"],
                common_candidate_mean=(
                    float(np.mean([d["our_cost"] for d in common])) if common else None
                ),
                common_reference_mean=(
                    float(np.mean([d["ref_cost"] for d in common])) if common else None
                ),
                common_mean_cost_delta=cost["estimate"],
                cost_ci95=cost["ci95"],
                all_mean_cold_seconds_delta=wall["estimate"],
                cold_ci95=wall["ci95"],
            )
        )
    return paired


def standings(data, baselines, method):
    """Use an identical job denominator for success and a common cost cohort."""
    successes = Counter()
    costs = defaultdict(list)
    for trial in data:
        key = (trial["problem_id"], int(trial["replica_seed"]), trial["objective"])
        panel = [trial, *baselines[key]]
        for r in panel:
            successes[r["method"]] += int(r["status"] == SUCCESS)
        if all(r["status"] == SUCCESS for r in panel):
            for r in panel:
                costs[r["method"]].append(float(r["objective_value"]))
    means = {m: float(np.mean(values)) for m, values in costs.items()}
    return dict(
        success_counts=dict(successes.most_common()),
        candidate_success_rank=(
            1 + sum(count > successes[method] for count in successes.values())
            if data
            else None
        ),
        tied_at_candidate_success_count=[
            m
            for m, count in successes.items()
            if m != method and count == successes[method]
        ],
        all_methods_common_success=len(costs.get(method, [])),
        all_methods_common_mean_cost=dict(
            sorted(means.items(), key=lambda entry: entry[1])
        ),
    )


def compare_clock_ablation(rows, manifest, control):
    control_rows, expected, control_manifest = observed(control)
    if manifest["source_hashes"] != control_manifest["source_hashes"]:
        raise ValueError("Clock ablation changes the runtime source")
    by_key = {
        (r["problem_id"], int(r["replica_seed"]), r["objective"]): r for r in rows
    }
    if any(
        j["evaluation_config"].get("phase_search") != {"joint_clocks": False}
        for j in control_manifest["jobs"]
    ):
        raise ValueError("Ablation must disable only joint clock proposals")
    if any(
        j["evaluation_config"].get("phase_search", {}).get("joint_clocks", True)
        is not True
        for j in manifest["jobs"]
    ):
        raise ValueError("The reference algorithm must enable clock proposals")
    matched = []
    controls = {}
    for off in control_rows:
        key = (off["problem_id"], int(off["replica_seed"]), off["objective"])
        if key not in by_key:
            raise ValueError("Missing matched clocks-enabled trial")
        matched.append(by_key[key])
        controls[key] = [
            dict(
                off,
                method="PHASE_SEARCH_CLOCKS_DISABLED",
                config_sha256=off["common_config_sha256"],
            )
        ]
    return compare(matched, expected, controls)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--baseline-root", type=Path, default=ROOT / "runs/end_to_end")
    parser.add_argument("--output", type=Path, help="Save summary.json and paired.csv")
    parser.add_argument(
        "--clock-ablation",
        type=Path,
        help="Optional same-source, matched run with joint clocks disabled",
    )
    parser.add_argument(
        "--replica-seed",
        type=int,
        help="Report one complete predeclared repetition cohort",
    )
    args = parser.parse_args()
    rows, planned, manifest = observed(args.run.resolve(), args.replica_seed)
    root = args.baseline_root
    if not all(
        read(root / "verification.json").get(k) is True
        for k in ("integrity_passed", "complete")
    ):
        raise ValueError("Baseline matrix is incomplete or has failed verification")
    if not rows:
        print("No sealed legal candidate outcomes yet.")
        return
    split = rows[0]["split"]
    baselines = defaultdict(list)
    seen = set()
    keys = {(r["problem_id"], int(r["replica_seed"]), r["objective"]) for r in rows}
    checked_runs = set()
    source_files = ("model.py", "runners/validate.py", "runners/ompl_baselines.py")
    with (root / "analysis/records.csv").open() as infile:
        for ref in csv.DictReader(infile):
            if "baselines_" + split not in json.loads(
                ref["selected_comparisons"] or "[]"
            ):
                continue
            key = (ref["problem_id"], int(ref["replica_seed"]), ref["objective"])
            if key not in keys:
                continue
            if ref["receipt_integrity"] != "True" or (*key, ref["method"]) in seen:
                raise ValueError("Unverified or duplicate baseline record")
            if ref["run"] not in checked_runs:
                sources = read(ROOT / ref["run"] / "run_manifest.json")["source_hashes"]
                for path in source_files:
                    name = "benchmarks/generic_motion_v1/" + path
                    if sources[name] != manifest["source_hashes"][name]:
                        raise ValueError("Shared source differs: " + name)
                checked_runs.add(ref["run"])
            seen.add((*key, ref["method"]))
            baselines[key].append(ref)
    for key in keys:
        if len(baselines[key]) != {"minimum_distance": 10, "minimum_time": 4}[key[-1]]:
            raise ValueError("Incomplete comparison panel: " + str(key))
    paired = compare(rows, planned, baselines)
    tracks = {}
    for objective, total in sorted(planned.items()):
        data = [r for r in rows if r["objective"] == objective]
        valid = [r for r in data if r["status"] == SUCCESS]
        tracks[objective] = dict(
            planned_legal=total,
            finished=len(data),
            status_counts=dict(Counter(r["status"] for r in data)),
            numerically_matches_physical_lower_bound=sum(
                abs(r["objective_value"] - r["physical_lower_bound"]) <= 1e-6
                for r in valid
            ),
            candidate_sources_on_success=dict(
                Counter(r["candidate_source"] for r in valid)
            ),
            mean_cold_s=(
                float(np.mean([r["cold_e2e_seconds"] for r in data])) if data else None
            ),
        )
        tracks[objective]["leaderboard"] = standings(data, baselines, rows[0]["method"])
    report = dict(
        split=split,
        replica_seed=args.replica_seed,
        run=str(args.run),
        commit=manifest.get("source_code_commit"),
        registered_jobs=sum(
            args.replica_seed is None or j["replica_seed"] == args.replica_seed
            for j in manifest["jobs"]
        ),
        complete=all(t["finished"] == t["planned_legal"] for t in tracks.values()),
        tracks=tracks,
        paired=paired,
    )
    if args.clock_ablation:
        report["clock_ablation"] = compare_clock_ablation(
            rows, manifest, args.clock_ablation.resolve()
        )
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        write(args.output / "summary.json", report)
        fields = (
            "problem_id",
            "scene_id",
            "cluster_id",
            "category",
            "split",
            "replica_seed",
            "planner_seed",
            "objective",
            "status",
            "failure_reason",
            "cold_e2e_seconds",
            "objective_value",
            "objective_unit",
            "validation_passed",
            "minimum_sampled_clearance_m",
            "physical_lower_bound",
            "candidate_source",
            "receipt_sha256",
            "attempt_path",
        )
        with (args.output / "legal_records.csv").open("w", newline="") as out:
            writer = csv.DictWriter(out, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
        with (args.output / "paired.csv").open("w", newline="") as out:
            writer = csv.DictWriter(out, fieldnames=list(paired[0]))
            writer.writeheader()
            writer.writerows(paired)
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
