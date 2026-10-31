"""Artifact-only metrics, full denominators and paired comparisons; never plans."""

from __future__ import annotations
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
from statistics import mean, stdev
import numpy as np
from .common import ROOT, atomic, read


def write_csv(path, rows):
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def average(values):
    values = [value for value in values if value is not None and np.isfinite(value)]
    return mean(values) if values else None


def deviation(values):
    values = [value for value in values if value is not None and np.isfinite(value)]
    return stdev(values) if len(values) > 1 else None


def cluster_interval(rows, field, repetitions=2000):
    groups = sorted({row["environment_seed"] for row in rows})
    if len(groups) < 2:
        return None
    groups_values = {
        group: [
            row[field]
            for row in rows
            if row["environment_seed"] == group and row.get(field) is not None
        ]
        for group in groups
    }
    rng = np.random.default_rng(250410783)
    values = []
    for _ in range(repetitions):
        chosen = rng.choice(groups, size=len(groups), replace=True)
        sample = [value for group in chosen for value in groups_values[group]]
        if sample:
            values.append(float(np.mean(sample)))
    return np.quantile(values, [0.025, 0.975]).tolist() if values else None


def summarize(run, output):
    run = Path(run).resolve()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = read(run / "manifest.json")
    jobs = []
    rows = []
    integrity = []
    for entry in manifest["cases"]:
        path = run / "cases" / entry["id"] / "case.json"
        case = read(path) if path.exists() else dict(status="not_run", methods={})
        if case.get("implementation_hash") not in {
            None,
            manifest["implementation_hash"],
        }:
            integrity.append(entry["id"] + ": source hash mismatch")
        roadmap = case.get("roadmap", {})
        row = dict(
            **entry,
            status=case["status"],
            drm_reported_success=roadmap.get("success"),
            drm_validated_success=case.get("drm_validated_success"),
            offline_build_seconds=roadmap.get("offline_build_seconds"),
            drm_online_seconds=roadmap.get("online_seconds"),
            drm_path_length=roadmap.get("path_length"),
            drm_segments=roadmap.get("segments"),
            actual_nodes=roadmap.get("actual_nodes"),
        )
        jobs.append(row)
        for method in manifest["config"]["method_order"]:
            result = case.get("methods", {}).get(method, {})
            status = result.get(
                "status",
                (
                    "not_run_due_to_drm"
                    if case["status"] in {"drm_failed", "invalid_seed_path"}
                    else "not_run"
                ),
            )
            record = dict(
                **entry,
                method=method,
                problem=(
                    "distance" if method in {"NTO_distance", "LSCS", "LGCS"} else "time"
                ),
                status=status,
                drm_eligible=case.get("drm_validated_success") is True,
                solver_success=result.get("solver_success"),
                validation_passed=result.get("validation_passed"),
                collision_free=result.get("collision_free"),
                cost=result.get("cost"),
                minimum_clearance=result.get("minimum_clearance"),
                total_ms=(
                    result.get("total_seconds") * 1000
                    if result.get("total_seconds") is not None
                    else None
                ),
                optimization_ms=(
                    result.get("optimization_seconds") * 1000
                    if result.get("optimization_seconds") is not None
                    else None
                ),
                construction_ms=(
                    result.get("construction_seconds") * 1000
                    if result.get("construction_seconds") is not None
                    else None
                ),
                validation_ms=(
                    result.get("validation_seconds") * 1000
                    if result.get("validation_seconds") is not None
                    else None
                ),
                wall_ms=(
                    result.get("elapsed_wall_seconds") * 1000
                    if result.get("elapsed_wall_seconds") is not None
                    else None
                ),
                regions=result.get("regions"),
                recovery_iterations=result.get("recovery_iterations"),
                reason=result.get("reason"),
            )
            if record["status"] == "success_numerical" and not (
                record["solver_success"]
                and record["validation_passed"]
                and record["collision_free"]
            ):
                integrity.append(entry["id"] + ": inconsistent success for " + method)
            rows.append(record)
    write_csv(output / "jobs.csv", jobs)
    write_csv(output / "methods.csv", rows)
    summaries = []
    statuses = []
    for method in manifest["config"]["method_order"]:
        selection = [row for row in rows if row["method"] == method]
        eligible = [row for row in selection if row["drm_eligible"]]
        attempted = [row for row in eligible if row["total_ms"] is not None]
        solved = [row for row in eligible if row["solver_success"] is True]
        passed = [row for row in eligible if row["status"] == "success_numerical"]
        counts = Counter(row["status"] for row in selection)
        for status, count in sorted(counts.items()):
            statuses.append(
                dict(
                    method=method,
                    status=status,
                    count=count,
                    requested_denominator=len(selection),
                )
            )
        summaries.append(
            dict(
                method=method,
                requested=len(selection),
                drm_eligible=len(eligible),
                attempted=len(attempted),
                solver_successes=len(solved),
                collision_free_successes=sum(
                    row["collision_free"] is True for row in solved
                ),
                collision_free_rate_given_drm=(
                    sum(row["collision_free"] is True for row in solved) / len(eligible)
                    if eligible
                    else None
                ),
                validated_successes=len(passed),
                solver_success_rate=len(solved) / len(eligible) if eligible else None,
                validated_rate_given_drm=(
                    len(passed) / len(eligible) if eligible else None
                ),
                validated_rate_requested=len(passed) / len(selection),
                collision_free_given_solver=(
                    sum(row["collision_free"] is True for row in solved) / len(solved)
                    if solved
                    else None
                ),
                cost_mean_given_validation=average([row["cost"] for row in passed]),
                cost_sd_given_validation=deviation([row["cost"] for row in passed]),
                cost_mean_given_solver_success=average([row["cost"] for row in solved]),
                total_ms_mean=average([row["total_ms"] for row in attempted]),
                total_ms_sd=deviation([row["total_ms"] for row in attempted]),
                total_ms_median=(
                    float(np.median([row["total_ms"] for row in attempted]))
                    if attempted
                    else None
                ),
                total_ms_p95=(
                    float(np.quantile([row["total_ms"] for row in attempted], 0.95))
                    if attempted
                    else None
                ),
                construction_ms_mean=average(
                    [row["construction_ms"] for row in attempted]
                ),
                optimization_ms_mean=average(
                    [row["optimization_ms"] for row in attempted]
                ),
                validation_ms_mean=average([row["validation_ms"] for row in attempted]),
                wall_ms_mean=average([row["wall_ms"] for row in attempted]),
                regions_mean=average([row["regions"] for row in attempted]),
                recovery_triggered=sum(
                    (row["recovery_iterations"] or 0) > 0 for row in attempted
                ),
                status_counts=dict(counts),
                total_ms_cluster_bootstrap_95=cluster_interval(attempted, "total_ms"),
            )
        )
    write_csv(output / "status_counts.csv", statuses)
    flat = [
        {
            key: json.dumps(value) if isinstance(value, (dict, list)) else value
            for key, value in record.items()
        }
        for record in summaries
    ]
    write_csv(output / "summary.csv", flat)
    pairs = []
    indexed = {(row["id"], row["method"]): row for row in rows}
    for alternative, baseline in [
        ("LSCS", "NTO_distance"),
        ("LGCS", "NTO_distance"),
        ("SCSTO", "NTO_time"),
        ("LGCS", "LSCS"),
    ]:
        for entry in manifest["cases"]:
            first = indexed[(entry["id"], alternative)]
            second = indexed[(entry["id"], baseline)]
            common = first["status"] == second["status"] == "success_numerical"
            pairs.append(
                dict(
                    **entry,
                    alternative=alternative,
                    baseline=baseline,
                    alternative_status=first["status"],
                    baseline_status=second["status"],
                    common_validated_success=common,
                    paired_runtime_speed_ratio=(
                        second["total_ms"] / first["total_ms"]
                        if first["total_ms"] is not None
                        and second["total_ms"] is not None
                        and first["total_ms"] > 0
                        else None
                    ),
                    cost_ratio=(
                        first["cost"] / second["cost"]
                        if common and second["cost"] > 0
                        else None
                    ),
                    cost_difference=first["cost"] - second["cost"] if common else None,
                )
            )
    write_csv(output / "paired_comparisons.csv", pairs)
    paper = read(ROOT / "reproduction/configs/paper_reference.json")
    comparisons = []
    for item in summaries:
        reference = paper["forest"][item["method"]]
        comparisons.append(
            dict(
                method=item["method"],
                paper_solver_success_rate=reference["solver_success_rate"],
                actual_solver_success_rate=item["solver_success_rate"],
                paper_collision_free_rate=reference["collision_free_rate"],
                actual_collision_free_rate=item["collision_free_rate_given_drm"],
                actual_validated_rate=item["validated_rate_given_drm"],
                paper_total_ms=reference["total_ms"],
                actual_total_ms=item["total_ms_mean"],
                paper_cost=reference["cost"],
                actual_cost_given_validation=item["cost_mean_given_validation"],
                comparison_scope="different hardware; reconstructed inputs; documented solver/transcription differences",
            )
        )
    write_csv(output / "paper_vs_actual.csv", comparisons)
    result = dict(
        run_dir=str(run),
        manifest_status=manifest["status"],
        implementation_hash=manifest["implementation_hash"],
        requested_cases=len(jobs),
        case_status_counts=dict(Counter(row["status"] for row in jobs)),
        drm_reported_successes=sum(row["drm_reported_success"] is True for row in jobs),
        drm_validated_successes=sum(
            row["drm_validated_success"] is True for row in jobs
        ),
        method_summaries=summaries,
        integrity_issues=integrity,
        statistics=dict(
            sd="sample standard deviation, ddof=1",
            bootstrap="2000 resamples clustered by environment seed; descriptive, not iid tests",
            primary_denominator="all requested cases; paper-conditional rates also retain every DRM-eligible case",
            timing="reported total = measured construction + optimization; validation, setup, allocation, offline DRM, warmup and artifact IO reported separately",
        ),
        fidelity=manifest["fidelity"],
    )
    atomic(output / "summary.json", result)
    lines = [
        "# Forest reconstruction: saved results",
        "",
        "Inputs and unspecified settings are declared in the frozen manifest. These are actual measurements, not the original authors' records.",
        "",
        "| Method | Requested | DRM eligible | Solver successes | Independently validated | Mean TO+CS [ms] | Mean validated cost |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summaries:
        fmt = lambda x: "—" if x is None else f"{x:.3f}"
        lines.append(
            f"| {row['method']} | {row['requested']} | {row['drm_eligible']} | {row['solver_successes']} | {row['validated_successes']} | {fmt(row['total_ms_mean'])} | {fmt(row['cost_mean_given_validation'])} |"
        )
    lines += [
        "",
        f"Case states: {result['case_status_counts']}. Native DRM reported {result['drm_reported_successes']}/{len(jobs)} solutions; exact seed-path validation accepted {result['drm_validated_successes']}/{len(jobs)}.",
        "",
        "All failures and absent outputs remain in methods.csv. Quality comparisons use common independently validated successes in paired_comparisons.csv. Counts from diagnostics or preliminary runs are not pooled.",
        "",
        "The original FR3 model/seeds and KUKA hardware recordings are unavailable; this report must not be described as an exact reproduction of the complete paper or its 17.1x claim.",
    ]
    (output / "summary.md").write_text("\n".join(lines) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--output", required=True)
    a = parser.parse_args()
    result = summarize(a.run_dir, a.output)
    print(
        json.dumps(
            {
                key: result[key]
                for key in [
                    "requested_cases",
                    "case_status_counts",
                    "drm_validated_successes",
                    "integrity_issues",
                ]
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
