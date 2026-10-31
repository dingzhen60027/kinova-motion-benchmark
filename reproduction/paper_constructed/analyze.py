"""Full-denominator summaries and source/raw-artifact audits; no planning calls."""

import argparse
from collections import Counter
import csv
import gzip
import hashlib
from pathlib import Path
import numpy as np
from reproduction.common import atomic, digest, read
from reproduction.analyze import cluster_interval
from .run import identity, runtime_versions


TERMINAL_CASE_STATUSES = {
    "completed", "drm_failed", "scene_failed", "offline_failed", "ik_failed",
    "error", "timeout", "process_error",
}


def source_integrity_issues(run, manifest, historical=False):
    """Check the manifest itself as well as its frozen numerical source bytes."""
    errors = []
    if digest(manifest["source_files"]) != manifest["implementation_hash"]:
        errors.append("manifest source-file map does not match implementation hash")
    if digest(manifest["config"]) != manifest["config_hash"]:
        errors.append("manifest configuration does not match configuration hash")
    for name, expected in manifest["source_files"].items():
        path = Path(run) / "source_snapshot" / name
        if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            errors.append("frozen source missing or changed: " + name)
    if not historical and digest(identity()) != manifest["implementation_hash"]:
        errors.append("current computational source drift")
    if runtime_versions() != manifest["runtime_versions"]:
        errors.append("runtime drift")
    return errors


def write_csv(path, rows, fields=None):
    fields = fields or list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def mean(values):
    values = [x for x in values if x is not None and np.isfinite(x)]
    return float(np.mean(values)) if values else None


def sd(values):
    values = [x for x in values if x is not None and np.isfinite(x)]
    return float(np.std(values, ddof=1)) if len(values) > 1 else None


def semantic_case_status(case):
    if (
        case.get("status") == "error"
        and not case.get("methods")
        and "[DRMPlanner] No collision-free path exists in roadmap."
        in case.get("reason", "")
    ):
        return "drm_failed_native_exception"
    return case["status"]


def collision_depths(run, out):
    """Post-hoc severity of saved collision witnesses; never changes success flags."""
    from .model import FR3, ForestEnvironment

    run = Path(run)
    out = Path(out)
    m = read(run / "manifest.json")
    # This operation evaluates geometry using the current Python model. An
    # archival summary may use old bytes, but geometric recomputation may not.
    errors = source_integrity_issues(run, m)
    if errors:
        raise ValueError("Cannot recompute collision depths with source drift: " + "; ".join(errors))
    family = m["environment"]
    env = None
    seed = None
    rows = []
    for entry in m["cases"]:
        directory = run / "cases" / entry["id"]
        case = read(directory / "case.json")
        if seed != entry["environment_seed"]:
            seed = entry["environment_seed"]
            if family == "fr3":
                scene = read(run / "scenes" / f"e{seed:02d}" / "scene.json")
                env = FR3(m["config"], scene["obstacles"])
            else:
                env = ForestEnvironment(m["config"], seed)
        for method in ["NTO_distance", "NTO_time"]:
            result = case.get("methods", {}).get(method, {})
            path = directory / method / "validation.json"
            v = read(path) if path.exists() else None
            minimum = None
            witnesses = [] if v is None else v["collision_points"]
            if witnesses:
                if family == "forest":
                    clearance = float(env.base.clearances(witnesses).min())
                    minimum = clearance if clearance < 0 else None
                else:
                    distances = []
                    for q in witnesses:
                        env.drake.SetPositions(env.pc, q)
                        query = env.sg.get_query_output_port().Eval(
                            env.sg.GetMyContextFromRoot(env.context)
                        )
                        distances.extend(
                            pair.distance
                            for pair in query.ComputeSignedDistancePairwiseClosestPoints(
                                0.0
                            )
                            if pair.distance < 0
                        )
                    if distances:
                        minimum = float(min(distances))
            depth = abs(minimum) if minimum is not None else None
            category = (
                "no_saved_negative_Drake_witness"
                if depth is None
                else (
                    "at_most_1um"
                    if depth <= 1e-6
                    else (
                        "1um_to_0.1mm"
                        if depth <= 1e-4
                        else "0.1mm_to_1mm" if depth <= 1e-3 else "over_1mm"
                    )
                )
            )
            rows.append(
                dict(
                    environment=family,
                    **entry,
                    method=method,
                    status=result.get("status", "not_run"),
                    solver_success=result.get("solver_success"),
                    collision_free=result.get("collision_free"),
                    saved_collision_witnesses=len(witnesses),
                    minimum_clearance_at_saved_witnesses=minimum,
                    collision_severity=category,
                )
            )
    write_csv(out / "collision_depths.csv", rows)
    summary = [
        dict(method=method, status=status, severity=severity, count=count)
        for (method, status, severity), count in sorted(
            Counter(
                (r["method"], r["status"], r["collision_severity"]) for r in rows
            ).items()
        )
    ]
    atomic(
        out / "collision_depth_summary.json",
        dict(
            scope="saved collision witnesses only; success flags unchanged; not continuous certification",
            counts=summary,
        ),
    )
    return rows


def summarize(run, out, audit=False, historical=False):
    run = Path(run).resolve()
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    m = read(run / "manifest.json")
    env = m["environment"]
    jobs = []
    rows = []
    issues = []
    for entry in m["cases"]:
        directory = run / "cases" / entry["id"]
        path = directory / "case.json"
        case = read(path) if path.exists() else dict(status="not_run", methods={})
        roadmap = case.get("roadmap", {})
        jobs.append(
            dict(
                **entry,
                environment=env,
                status=case["status"],
                semantic_status=semantic_case_status(case),
                drm_success=roadmap.get("success"),
                strict_seed_valid=case.get("drm_validated_success"),
                path_length=roadmap.get("path_length"),
                segments=roadmap.get("segments"),
                online_seconds=roadmap.get("online_seconds"),
                elapsed_seconds=case.get("elapsed_seconds"),
                failure_stage=case.get("failure_stage"),
                reason=case.get("reason"),
            )
        )
        if case.get("implementation_hash") not in [None, m["implementation_hash"]]:
            issues.append(entry["id"] + ": source identity mismatch")
        if path.exists() and case.get("implementation_hash") is None:
            issues.append(entry["id"] + ": missing case source identity")
        if m["status"] == "completed" and case["status"] not in TERMINAL_CASE_STATUSES:
            issues.append(entry["id"] + ": completed manifest has nonterminal/missing case")
        for method in m["config"]["method_order"]:
            r = case.get("methods", {}).get(method, {})
            interrupted = bool(not r and case["status"] in {"timeout", "process_error"}
                               and case.get("failure_stage") == method)
            recovery_iterations = r.get("recovery_iterations")
            recovery_triggered = bool((recovery_iterations or 0) > 0)
            if not method.startswith("NTO"):
                pre_solve = sorted((directory / method).glob("pre_solve_*.json"))
                if pre_solve:
                    recovered = read(pre_solve[-1]).get("iteration", 0)
                    recovery_iterations = max(recovery_iterations or 0, recovered)
                # Also retain a collision-triggered repair that subsequently
                # raised an exception before a normal final result was returned.
                for vp in (directory / method).glob("validation_*.json"):
                    iteration = int(vp.stem.rsplit("_", 1)[-1])
                    v = read(vp)
                    if (iteration < m["config"]["maximum_recovery_iterations"]
                            and len(v.get("collision_points", []))):
                        recovery_triggered = True
            row = dict(
                **entry,
                environment=env,
                method=method,
                problem=(
                    "distance" if method in ["LSCS", "LGCS", "NTO_distance"] else "time"
                ),
                case_status=case["status"],
                native_drm_success=roadmap.get("success") is True,
                strict_seed_valid=case.get("drm_validated_success") is True,
                status=r.get("status", ("interrupted_" if interrupted else "not_run_due_to_")
                             + semantic_case_status(case)),
                attempted=bool((r and r.get("status") != "invalid_seed_path") or interrupted),
                timing_scope=(
                    "exception_wall_fallback"
                    if r.get("status") in ["error", "blocked_license"]
                    else (
                        "construction_plus_optimization"
                        if r.get("total_seconds") is not None
                        else "interrupted_no_completed_timing" if interrupted else "not_attempted"
                    )
                ),
                solver=r.get("solver"),
                solver_success=r.get("solver_success"),
                validation_passed=r.get("validation_passed"),
                collision_free=r.get("collision_free"),
                native_cuda_collision_free=r.get("native_cuda_collision_free"),
                independent_collision_free=r.get("independent_collision_free"),
                model_disagreements=r.get("model_disagreements"),
                cost=r.get("cost"),
                total_ms=(
                    r.get("total_seconds") * 1000
                    if r.get("total_seconds") is not None
                    else None
                ),
                optimization_ms=(
                    r.get("optimization_seconds") * 1000
                    if r.get("optimization_seconds") is not None
                    else None
                ),
                construction_ms=(
                    r.get("construction_seconds") * 1000
                    if r.get("construction_seconds") is not None
                    else None
                ),
                validation_ms=(
                    r.get("validation_seconds") * 1000
                    if r.get("validation_seconds") is not None
                    else None
                ),
                wall_ms=(
                    r.get("elapsed_wall_seconds") * 1000
                    if r.get("elapsed_wall_seconds") is not None
                    else None
                ),
                recovery_iterations=recovery_iterations,
                recovery_triggered=recovery_triggered,
                regions=r.get("regions"),
                reason=r.get("reason"),
            )
            rows.append(row)
            if row["status"] == "success_numerical":
                if not (
                    row["solver_success"]
                    and row["validation_passed"]
                    and row["collision_free"]
                    and not row["model_disagreements"]
                ):
                    issues.append(entry["id"] + ": inconsistent success: " + method)
                if not list((directory / method).glob("candidate_*.npz")):
                    issues.append(entry["id"] + ": missing candidate: " + method)
            result_path = directory / method / "result.json"
            if r and (not result_path.exists() or r != read(result_path)):
                issues.append(entry["id"] + ": missing/mismatched checkpoint: " + method)
    write_csv(out / "jobs.csv", jobs)
    write_csv(out / "methods.csv", rows)
    summaries = []
    status_rows = []
    for method in m["config"]["method_order"]:
        part = [r for r in rows if r["method"] == method]
        attempted = [r for r in part if r["attempted"]]
        timed = [r for r in attempted if r["total_ms"] is not None]
        passed = [r for r in part if r["status"] == "success_numerical"]
        native = [r for r in part if r["native_drm_success"]]
        counts = Counter(r["status"] for r in part)
        status_rows.extend(
            dict(
                environment=env, method=method, status=k, count=v, denominator=len(part)
            )
            for k, v in sorted(counts.items())
        )
        summaries.append(
            dict(
                environment=env,
                method=method,
                requested=len(part),
                native_drm_successes=len(native),
                strict_seed_valid=sum(r["strict_seed_valid"] for r in part),
                validated_on_valid_seeds=sum(
                    r["status"] == "success_numerical" and r["strict_seed_valid"]
                    for r in part
                ),
                validated_rate_given_valid_seed=(
                    sum(
                        r["status"] == "success_numerical" and r["strict_seed_valid"]
                        for r in part
                    )
                    / sum(r["strict_seed_valid"] for r in part)
                    if any(r["strict_seed_valid"] for r in part)
                    else None
                ),
                attempted=len(attempted),
                attempts_with_completed_timing=len(timed),
                interrupted_attempts_without_timing=sum(r["timing_scope"] == "interrupted_no_completed_timing" for r in part),
                exception_wall_timings=sum(
                    r["timing_scope"] == "exception_wall_fallback" for r in part
                ),
                timing_environment_clusters=len(
                    {r["environment_seed"] for r in timed}
                ),
                solver_successes=sum(r["solver_success"] is True for r in part),
                solver_success_rate_given_native_drm=(sum(r["solver_success"] is True for r in native)/len(native) if native else None),
                native_cuda_cfr_given_drm=(sum(r["native_cuda_collision_free"] is True for r in native)/len(native) if native else None),
                native_cuda_free_trajectories=sum(r["native_cuda_collision_free"] is True for r in part),
                native_cuda_checked_trajectories=sum(r["native_cuda_collision_free"] is not None for r in part),
                solver_failed_but_native_cuda_free=sum(r["solver_success"] is False and r["native_cuda_collision_free"] is True for r in part),
                native_cuda_validated_successes=sum(r["solver_success"] is True and r["native_cuda_collision_free"] is True for r in part),
                solver_success_and_native_cuda_free_rate_given_drm=(sum(r["solver_success"] is True and r["native_cuda_collision_free"] is True for r in native)/len(native) if native else None),
                solver_successes_missing_native_cuda_check=sum(r["solver_success"] is True and r["native_cuda_collision_free"] is None for r in part),
                cost_mean_solver_success=mean([r["cost"] for r in part if r["solver_success"] is True]),
                cost_sd_solver_success=sd([r["cost"] for r in part if r["solver_success"] is True]),
                validated_successes=len(passed),
                validated_rate_requested=len(passed) / len(part),
                validated_rate_given_native_drm=(
                    len(passed) / len(native) if native else None
                ),
                cost_mean_validated=mean([r["cost"] for r in passed]),
                cost_sd_validated=sd([r["cost"] for r in passed]),
                total_ms_mean_all_attempts=mean([r["total_ms"] for r in attempted]),
                total_ms_sd_all_attempts=sd([r["total_ms"] for r in attempted]),
                construction_ms_mean=mean([r["construction_ms"] for r in attempted]),
                optimization_ms_mean=mean([r["optimization_ms"] for r in attempted]),
                validation_ms_mean=mean([r["validation_ms"] for r in attempted]),
                total_ms_cluster_bootstrap_95=cluster_interval(timed, "total_ms"),
                recovery_triggered=sum(r["recovery_triggered"] for r in part),
                recovery_rate_given_attempt=sum(r["recovery_triggered"] for r in attempted)/len(attempted) if attempted else None,
                recovery_rate_given_native_drm=sum(r["recovery_triggered"] for r in native)/len(native) if native else None,
                recovery_rate_given_valid_seed=(sum(r["recovery_triggered"] and r["strict_seed_valid"] for r in part)/sum(r["strict_seed_valid"] for r in part) if any(r["strict_seed_valid"] for r in part) else None),
                recovery_rate_given_solver_success=(sum(r["recovery_triggered"] and r["solver_success"] is True for r in part)/sum(r["solver_success"] is True for r in part) if any(r["solver_success"] is True for r in part) else None),
                completed_recovery_iterations=sum(r["recovery_iterations"] or 0 for r in part),
                status_counts=dict(counts),
            )
        )
    write_csv(out / "summary.csv", summaries)
    write_csv(out / "statuses.csv", status_rows)
    paired = []
    index = {(r["id"], r["method"]): r for r in rows}
    for alternative, baseline in [
        ("LSCS", "NTO_distance"),
        ("LGCS", "NTO_distance"),
        ("LGCS", "LSCS"),
        ("SCSTO", "NTO_time"),
    ]:
        for case in m["cases"]:
            a, b = index[(case["id"], alternative)], index[(case["id"], baseline)]
            common = a["status"] == b["status"] == "success_numerical"
            paired.append(
                dict(
                    environment=env,
                    **case,
                    alternative=alternative,
                    baseline=baseline,
                    common_validated=common,
                    alternative_status=a["status"],
                    baseline_status=b["status"],
                    cost_difference=a["cost"] - b["cost"] if common else None,
                    cost_ratio=a["cost"] / b["cost"] if common and b["cost"] else None,
                    time_ratio_baseline_over_alternative=(
                        b["total_ms"] / a["total_ms"]
                        if a["total_ms"] and b["total_ms"]
                        else None
                    ),
                )
            )
    write_csv(out / "paired.csv", paired)
    pair_summary = []
    for pair in sorted({(r["alternative"], r["baseline"]) for r in paired}):
        pp = [r for r in paired if (r["alternative"], r["baseline"]) == pair]
        common = [r for r in pp if r["common_validated"]]
        pair_summary.append(
            dict(
                alternative=pair[0],
                baseline=pair[1],
                common_validated=len(common),
                mean_cost_difference=mean([r["cost_difference"] for r in common]),
                mean_cost_ratio=mean([r["cost_ratio"] for r in common]),
                runtime_pairs=sum(
                    r["time_ratio_baseline_over_alternative"] is not None for r in pp
                ),
                mean_paired_time_ratio=mean(
                    [r["time_ratio_baseline_over_alternative"] for r in pp]
                ),
            )
        )
    source_errors = source_integrity_issues(run, m, historical)
    result = dict(
        environment=env,
        run_dir=str(run),
        manifest_status=m["status"],
        requested=len(jobs),
        terminal_cases=sum(r["status"] in TERMINAL_CASE_STATUSES for r in jobs),
        case_status_counts=dict(Counter(r["status"] for r in jobs)),
        semantic_case_status_counts=dict(Counter(r["semantic_status"] for r in jobs)),
        native_drm_successes=sum(r["drm_success"] is True for r in jobs),
        strict_seed_valid=sum(r["strict_seed_valid"] is True for r in jobs),
        methods=summaries,
        paired=pair_summary,
        source_integrity_issues=source_errors,
        artifact_issues=issues,
        implementation_hash=m["implementation_hash"],
        historical_snapshot_only=historical,
        fidelity=m["fidelity"],
        timing_scope="construction plus optimization for recorded solves; caught exceptions retain method wall time as an explicitly flagged fallback; alternate-map serialization/loading and alternate-path validation, plus recovery coverage-record writes, occur inside construction timing; final independent validation, offline roadmap preparation, warmup and most pre-solve/replay IO are outside TO+CS",
        statistics="sample SD ddof=1; 2000 environment-cluster bootstrap resamples; cost comparisons only on common validated pairs",
        collision_rate_definition="Native CUDA CFR = all saved CUDA checks free, divided by native DRM successes, irrespective of solver success. The solver-success conjunction and independently validated solver success are reported separately. Missing trajectories/checks remain in the DRM denominator but are not counted free.",
        recovery_rate_definition="A non-NTO collision witness requiring a subsequent correction counts as recovery triggered, including corrections later interrupted by an exception. Completed edit iterations are separate. Rates given attempted methods, native DRM success, and independently valid seeds are supplied; the paper's exact distance-method pooling denominator is not reconstructed.",
    )
    atomic(out / "summary.json", result)
    if audit:
        inventory = {}
        for path in sorted(run.rglob("*")):
            if path.is_file():
                h = hashlib.sha256()
                with path.open("rb") as stream:
                    for block in iter(lambda: stream.read(1024 * 1024), b""):
                        h.update(block)
                inventory[str(path.relative_to(run))] = dict(
                    bytes=path.stat().st_size, sha256=h.hexdigest()
                )
        import json

        blob = (json.dumps(inventory, indent=2) + "\n").encode()
        (out / "raw_inventory.json.gz").write_bytes(gzip.compress(blob, mtime=0))
        atomic(
            out / "raw_inventory_summary.json",
            dict(
                files=len(inventory),
                bytes=sum(v["bytes"] for v in inventory.values()),
                inventory_hash=digest(inventory),
            ),
        )
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run-dir", nargs="+", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--audit", action="store_true")
    p.add_argument("--collision-depths", action="store_true")
    p.add_argument(
        "--historical",
        action="store_true",
        help="Verify archived snapshot; do not require it to equal current sources",
    )
    args = p.parse_args()
    all_results = [
        summarize(
            run,
            Path(args.output) / read(Path(run) / "manifest.json")["environment"],
            args.audit,
            args.historical,
        )
        for run in args.run_dir
    ]
    atomic(Path(args.output) / "combined.json", all_results)
    if args.collision_depths:
        for result in all_results:
            collision_depths(
                result["run_dir"], Path(args.output) / result["environment"]
            )
    for result in all_results:
        print(
            result["environment"],
            result["case_status_counts"],
            [
                (r["method"], r["validated_successes"], r["requested"])
                for r in result["methods"]
            ],
        )
    return int(
        any(r["source_integrity_issues"] or r["artifact_issues"] for r in all_results)
    )


if __name__ == "__main__":
    raise SystemExit(main())
