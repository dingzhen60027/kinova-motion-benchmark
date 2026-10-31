"""CPU-only audit of the complete, sealed 360-job development run.

This report utility does not import the benchmark runtime, Drake or CSDecomp.
It refuses incomplete evidence and never launches or resumes planning. Run from
the repository with ``bash local/python.sh <this file>`` after summarization.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import tempfile

import numpy as np

BASE = Path(__file__).resolve().parents[1]
SUCCESS = "success_validated_numerical"
PENDING = {"pending", "running", "not_started", "retry_pending"}
FAMILIES = {"sparse_clutter", "dense_clutter", "bottleneck", "route_choice"}
OBJECTIVES = {"minimum_distance", "minimum_time"}


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and np.isfinite(value)


def select(*values):
    return next((value for value in values if value is not None), None)


def finite(*values):
    return next((value for value in values if number(value)), None)


def stats(values):
    values = list(values)
    a = np.asarray([value for value in values if number(value)], float)
    result = dict(count=len(a), missing_count=len(values) - len(a), mean=None,
                  p50=None, p90=None, p95=None, min=None, max=None)
    if len(a):
        result.update(mean=float(a.mean()), p50=float(np.quantile(a, .5)),
                      p90=float(np.quantile(a, .9)), p95=float(np.quantile(a, .95)),
                      min=float(a.min()), max=float(a.max()))
    return result


def cluster_ratio(rows, conditional=False, seed=45827, replicates=2000):
    # Resample actual scene IDs, keeping the paired endpoints/roadmaps together.
    groups = defaultdict(lambda: [0, 0])
    for row in rows:
        if row["input_legal"] and (not conditional or row["drm_success"] is True):
            groups[row["scene_id"]][0] += int(row["success"])
            groups[row["scene_id"]][1] += 1
    totals = np.asarray([v for _, v in sorted(groups.items())], float).reshape(-1, 2)
    n = len(totals)
    numerator = int(totals[:, 0].sum()) if n else 0
    denominator = int(totals[:, 1].sum()) if n else 0
    result = dict(numerator=numerator, denominator=denominator,
                  estimate=numerator / denominator if denominator else None,
                  independent_scene_clusters=n, ci95=None, bootstrap_seed=seed,
                  bootstrap_replicates=replicates,
                  method="Percentile scene-cluster bootstrap, all paired jobs kept together")
    if n >= 2:
        rng = np.random.Generator(np.random.PCG64(seed))
        draws = rng.integers(n, size=(replicates, n))
        counts = totals[draws].sum(axis=1)
        result["ci95"] = np.quantile(counts[:, 0] / counts[:, 1], [.025, .975]).tolist()
    else:
        result["limitation"] = "Fewer than two observed scene clusters; no CI"
    return result


def plan_and_completion(run):
    manifest = read(run / "run_manifest.json")
    jobs = manifest.get("jobs", [])
    require(manifest.get("split") == "dev" and not manifest.get("formal_test_run"),
            "Only the development run is authorized for this reporting script")
    require(len(jobs) == 360, f"Expected 360 planned jobs, found {len(jobs)}")
    require(len({j["job_id"] for j in jobs}) == 360, "Duplicate job IDs")
    require({j["method"] for j in jobs} == {"EI_ZO"}, "Expected the EI_ZO-only development run")
    require(Counter(j["objective"] for j in jobs) == Counter({o: 180 for o in OBJECTIVES}),
            "Expected 180 jobs per objective")
    require(Counter(j["family"] for j in jobs) == Counter({f: 90 for f in FAMILIES}),
            "Expected 90 jobs per obstacle family")
    require(len({j["case_id"] for j in jobs}) == 60, "Expected 60 fixed development cases")
    require(len({j["scene_id"] for j in jobs}) == 20, "Expected 20 independent development scenes")
    require(len({json.dumps(j["evaluation_config"], sort_keys=True) for j in jobs}) == 1,
            "Evaluation configuration changed within the frozen development plan")
    roadmaps = {j["roadmap_id"] for j in jobs}
    require(len(roadmaps) == 3, "Expected three offline roadmap seeds")
    by_case = defaultdict(list)
    for job in jobs:
        by_case[job["case_id"]].append(job)
    for case_id, paired in by_case.items():
        require({(j["objective"], j["roadmap_id"]) for j in paired} ==
                {(o, r) for o in OBJECTIVES for r in roadmaps}, f"Incomplete pairing: {case_id}")
        require(len({json.dumps(j["case"], sort_keys=True) for j in paired}) == 1,
                f"Paired methods changed the fixed case: {case_id}")
    selected, incomplete = [], []
    for job in jobs:
        attempts = sorted((run / "jobs" / job["job_id"] / "attempts").glob("[0-9][0-9][0-9][0-9]"))
        if not attempts or not (attempts[-1] / "receipt.json").is_file():
            incomplete.append(job["job_id"])
            continue
        attempt = attempts[-1]
        receipt = read(attempt / "receipt.json")
        result = read(attempt / "result.json")
        require(isinstance(result.get("status"), str), f"Sealed result has no status: {attempt}")
        if receipt.get("retry_pending") or result.get("status") in PENDING:
            incomplete.append(job["job_id"])
            continue
        require(receipt.get("status") == result.get("status"), f"Status/receipt disagreement: {attempt}")
        selected.append((job, attempt, result, receipt, attempts))
    require(not incomplete, f"Refusing final report: {len(selected)}/360 sealed final jobs; "
            f"{len(incomplete)} incomplete jobs, first={incomplete[:3]}")
    summary_path = run / "summary/summary.json"
    require(summary_path.is_file(), "Refusing final report: completed summary/summary.json is missing")
    summary = read(summary_path)
    require(summary.get("complete") is True and summary.get("planned_jobs") == 360 and
            summary.get("finished_jobs") == 360, "Refusing final report: summary is incomplete")
    require(summary.get("run_manifest_sha256") == sha(run / "run_manifest.json"), "Summary manifest SHA mismatch")
    require(summary.get("status_counts") == dict(Counter(item[2]["status"] for item in selected)),
            "Summary status counts do not match the sealed latest attempts")
    return manifest, summary, selected


def verify_artifacts(run, manifest, selected):
    checked = 0
    total_bytes = 0
    require(read(run / "identity.json")["run_manifest_sha256"] == sha(run / "run_manifest.json"),
            "Run manifest identity changed")
    expected_files = [(Path(manifest["manifest_path"]), manifest["manifest_sha256"])]
    expected_files += [(run / "source_snapshot" / name, digest)
                       for name, digest in manifest["source_hashes"].items()]
    expected_files += [(Path(name), digest) for name, digest in manifest["roadmap_files"].items()]
    receipts = []
    historical_attempts = []
    for job, latest, _, _, attempts in selected:
        require(read(latest / "evaluation.json") == job["evaluation_config"],
                f"Executed evaluation configuration differs from frozen plan: {latest}")
        for attempt in attempts:
            receipt_path = attempt / "receipt.json"
            require(receipt_path.is_file(), f"Historical attempt is not sealed: {attempt}")
            receipt = read(receipt_path)
            receipts.append(dict(path=str(receipt_path.relative_to(run)), sha256=sha(receipt_path)))
            for name, digest in receipt["files"].items():
                file = (attempt / name).resolve()
                require(file.is_relative_to(attempt.resolve()), f"Receipt path escaped attempt: {name}")
                expected_files.append((file, digest))
            if attempt != latest:
                historical_attempts.append(dict(job_id=job["job_id"], attempt=str(attempt.relative_to(run)),
                    status=receipt["status"], retry_pending=bool(receipt.get("retry_pending")),
                    included_in_final_job_denominator=False))
    for file, expected in expected_files:
        require(file.is_file() and sha(file) == expected, f"Saved evidence SHA mismatch: {file}")
        checked += 1
        total_bytes += file.stat().st_size
    return dict(integrity_passed=True, checked_files=checked, checked_bytes=total_bytes,
                frozen_evaluation_config_versions=1, executed_configs_matching_manifest=len(selected),
                historical_attempts_excluded_from_final_denominator=historical_attempts,
                receipt_sha256=receipts)


def extract_row(job, attempt, result):
    process = read(attempt / "process.json")
    validation = result.get("validation") or (read(attempt / "validation.json")
                 if (attempt / "validation.json").is_file() else {})
    timing, metrics = result.get("timing", {}), result.get("metrics", {})
    case = job["case"]
    require(case.get("validation_status") in {"valid_input", "invalid_endpoint"},
            f"Unrecognized input validity: {job['job_id']}")
    regions = read(attempt / "regions.json") if (attempt / "regions.json").is_file() else {}
    row = dict(job_id=job["job_id"], case_id=job["case_id"], scene_id=job["scene_id"],
        family=job["family"], objective=job["objective"], roadmap_id=job["roadmap_id"],
        method_seed=job["method_seed"], status=result["status"], success=result["status"] == SUCCESS,
        input_legal=case["validation_status"] == "valid_input",
        drm_success=select(result.get("drm_success"), result.get("frontend_success")),
        cold_e2e_seconds=process.get("cold_e2e_seconds"),
        warm_query_seconds=timing.get("warm_query_seconds"),
        sampled_gpu_peak_mib=process.get("sampled_gpu_peak_mib"), timing=timing,
        regions=finite(metrics.get("regions"), regions.get("count")),
        facets=finite(metrics.get("facets"), sum(regions["facets"]) if "facets" in regions else None),
        recovery_attempts=finite(metrics.get("recovery_attempts")),
        normalized_path_length=finite(validation.get("normalized_path_length"), metrics.get("normalized_path_length")),
        minimum_sampled_clearance_m=finite(validation.get("minimum_sampled_clearance_m"), metrics.get("minimum_sampled_clearance_m")),
        cost=finite(validation.get("cost"), metrics.get("cost")) if result["status"] == SUCCESS else None,
        cost_unit=select(validation.get("cost_unit"), metrics.get("cost_unit")),
        failed_stage=result.get("failed_stage"), final_stage=result.get("final_stage"),
        error_category=result.get("error_category"), process_exit_code=process.get("exit_code"),
        timed_out=process.get("timed_out", False),
        attempt=str(attempt), has_saved_candidate=(attempt / "candidate.npz").is_file())
    if row["success"]:
        require(validation.get("passed") is True, f"Success has no passing validation: {attempt}")
        require((attempt / "trajectory_samples.npz").is_file(), f"Success has no sampled trajectory: {attempt}")
    return row


def group_rows(rows, objective, family):
    partitions = {"all_finished": rows,
                  "success": [r for r in rows if r["success"]],
                  "failure": [r for r in rows if not r["success"]]}
    numeric = ["cold_e2e_seconds", "warm_query_seconds", "sampled_gpu_peak_mib", "regions", "facets",
               "recovery_attempts", "normalized_path_length", "minimum_sampled_clearance_m"]
    stages = sorted({key for r in rows for key, value in r["timing"].items() if number(value)})
    return dict(objective=objective, family=family, planned_jobs=len(rows),
        legal_input_jobs=sum(r["input_legal"] for r in rows),
        invalid_input_jobs=sum(not r["input_legal"] for r in rows),
        independent_scene_clusters=len({r["scene_id"] for r in rows}),
        status_counts=dict(sorted(Counter(r["status"] for r in rows).items())),
        unconditional_success=cluster_ratio(rows), conditional_on_drm_success=cluster_ratio(rows, True),
        drm_status_unknown_jobs=sum(r["drm_success"] is None for r in rows),
        recorded_statistics={field: {label: stats(r[field] for r in sample)
            for label, sample in partitions.items()} for field in numeric},
        stage_timing={stage: {label: stats(r["timing"].get(stage) for r in sample)
            for label, sample in partitions.items()} for stage in stages},
        success_cost=stats(r["cost"] for r in partitions["success"]))


def coverage_audit(selected):
    records = []
    for job, attempt, _, receipt, _ in selected:
        limit = job["evaluation_config"]["region_coverage_tolerance_rad"]
        for name in sorted(receipt["files"]):
            if name.startswith("coverage/") and name.endswith("/coverage.json"):
                data = read(attempt / name)
                require(data.get("normal_matrix_changed") is False and data.get("fixed_endpoints_changed") is False,
                        f"Coverage report records a forbidden geometry/endpoint change: {attempt / name}")
                require(data["allowed_normalized_correction"] == limit, "Coverage tolerance differs from frozen config")
                maximum = data.get("maximum_normalized_correction")
                if data["status"] == "accepted":
                    require(number(maximum) and maximum <= limit, f"Accepted over-budget coverage correction: {attempt / name}")
                    require((attempt / Path(name).parent / "after.npz").is_file(), "Accepted coverage lacks saved corrected region")
                records.append(dict(job_id=job["job_id"], objective=job["objective"], family=job["family"],
                    status=data["status"], stage=Path(name).parts[1],
                    maximum_normalized_correction=maximum, allowed_normalized_correction=limit,
                    corrected_planes=data.get("corrected_planes", 0), artifact=str(attempt / name)))
    groups = []
    for objective in sorted(OBJECTIVES):
        for family in ["all", *sorted(FAMILIES)]:
            subset = [r for r in records if r["objective"] == objective and (family == "all" or r["family"] == family)]
            accepted = [r for r in subset if r["status"] == "accepted"]
            groups.append(dict(objective=objective, family=family, calls=len(subset),
                accepted=len(accepted), rejected=sum(r["status"] != "accepted" for r in subset),
                jobs_with_any_correction=len({r["job_id"] for r in accepted if r["corrected_planes"] > 0}),
                corrected_planes_total=sum(r["corrected_planes"] for r in accepted),
                maximum_normalized_correction=stats(r["maximum_normalized_correction"] for r in subset)["max"],
                maximum_accepted_normalized_correction=stats(r["maximum_normalized_correction"] for r in accepted)["max"]))
    maximum_record = max((r for r in records if number(r["maximum_normalized_correction"])),
                         key=lambda r: r["maximum_normalized_correction"], default=None)
    return dict(total_records=len(records), accepted_records=sum(r["status"] == "accepted" for r in records),
        rejected_records=sum(r["status"] != "accepted" for r in records),
        maximum_record=maximum_record, groups=groups,
        rejection_records=[r for r in records if r["status"] != "accepted"], records=records,
        note="Counts are recorded handoff checks, including repeated/reused regions; not independent geometric experiments. A and fixed endpoints stay unchanged. Sampling after bounded b relaxation is not a continuous safety certificate.")


def classify_stderr(text, exit_code, timed_out=False):
    categories = []
    if re.search(r"Assertion\s+[`'\"].*failed|assertion.*failed", text, re.I):
        categories.append("native_assertion")
    if "GetClosestNonCollidingConfigurationInRoadMap" in text and "closest_node >= 0" in text:
        categories.append("drm_no_connectable_node_assertion")
    if re.search(r"CUDA\s+(?:error|Error)|cudaError|device-side assert|illegal memory access", text, re.I):
        categories.append("cuda_runtime_error_text")
    if "terminate called after throwing" in text:
        categories.append("uncaught_native_exception")
    if number(exit_code) and exit_code < 0:
        categories.append("supervisor_timeout_termination" if timed_out else "process_signal")
    return categories


def stderr_audit(selected):
    records = []
    for job, attempt, result, _, _ in selected:
        process = read(attempt / "process.json")
        file = attempt / "stderr.log"
        text = file.read_text(errors="replace") if file.is_file() else ""
        categories = classify_stderr(text, process.get("exit_code"), process.get("timed_out", False))
        if not categories:
            continue
        lines = text.splitlines()
        evidence = [dict(line=i + 1, text=line[:1800]) for i, line in enumerate(lines)
                    if re.search(r"assert|CUDA|cudaError|terminate called|what\(\)", line, re.I)]
        records.append(dict(job_id=job["job_id"], family=job["family"], objective=job["objective"],
            status=result["status"], error_category=result.get("error_category"),
            exit_code=process.get("exit_code"), categories=categories, stderr=str(file),
            supervisor_timed_out=process.get("timed_out", False),
            evidence_lines=evidence, signal_number=-process["exit_code"] if process.get("exit_code", 0) < 0 else None))
    return dict(category_counts=dict(sorted(Counter(c for r in records for c in r["categories"]).items())),
        affected_jobs=len(records), records=records,
        note="Categories can overlap and retain the recorded terminal status. Supervisor timeout terminations are distinguished from unexpected native process signals. A DRM assertion does not prove the original continuous planning problem infeasible.")


def offline_audit(manifest, summary):
    entries = []
    dirs = sorted({j["roadmap_dir"] for j in manifest["jobs"]})
    for directory in dirs:
        file = Path(directory) / "metadata.json"
        data = read(file)
        entries.append(dict(roadmap_id=data["roadmap_id"], metadata_path=str(file), metadata_sha256=sha(file),
            offline_seconds=data["offline_seconds"], requested_nodes=data["requested_nodes"],
            actual_nodes=data["actual_nodes"], directed_edges=data["directed_edges"],
            native_map_bytes=data["native_map_bytes"], native_map_sha256=data["native_map_sha256"],
            timing=data["timing"], seed=data["seed"]))
    require({r["roadmap_id"] for r in entries} == {r["roadmap_id"] for r in summary["offline_roadmaps"]},
            "Summary and manifest disagree on offline roadmap membership")
    return dict(roadmaps=entries, total_offline_seconds=sum(r["offline_seconds"] for r in entries),
        total_native_map_bytes=sum(r["native_map_bytes"] for r in entries),
        note="Each offline roadmap is counted once; no offline cost is folded into online per-query timing. Stage timers need not sum to total wall time because initialization, serialization and other work are included in the total.")


def fmt(value, digits=4):
    return "—" if value is None else f"{value:.{digits}g}"


def rate(value):
    ci = value["ci95"]
    interval = "CI unavailable" if ci is None else f"95% CI {ci[0]:.1%}–{ci[1]:.1%}"
    estimate = "undefined" if value["estimate"] is None else f"{value['estimate']:.1%}"
    return f"{value['numerator']}/{value['denominator']} ({estimate}; {interval})"


def render(report):
    lines = ["# Sealed development benchmark results", "",
        f"Run: `{report['run_directory']}`; source commit `{report['source_code_commit']}`.", "",
        "All 360 planned EI-ZO jobs are sealed and their saved-file SHA-256 receipts were verified. "
        "This is development evidence from 60 fixed cases in 20 independent obstacle scenes, with three paired roadmap seeds and two objectives. "
        "It is not a formal test-set result or an exact Franka/FR3 reproduction.", "",
        "Missing stage durations and absent candidate metrics stay missing. Finite recorded values are summarized without zero imputation; "
        "failure metrics describe saved candidates, not valid solutions. Success means the saved trajectory passed finite numerical checks, "
        "not a mathematical continuous collision-free certificate.", "",
        "## Status and scene-cluster uncertainty", "",
        "| Objective | Family | Legal/planned | Success / legal | Success / recorded DRM success | Terminal statuses |",
        "|---|---|---:|---|---|---|"]
    for group in report["groups"]:
        statuses = "; ".join(f"{k}: {v}" for k, v in group["status_counts"].items())
        lines.append(f"| {group['objective']} | {group['family']} | {group['legal_input_jobs']}/{group['planned_jobs']} | "
                     f"{rate(group['unconditional_success'])} | {rate(group['conditional_on_drm_success'])} | {statuses} |")
    lines += ["", "The percentile bootstrap resamples physical scene IDs with all endpoint/roadmap pairs intact (2,000 replicates, PCG64 seed 45827). "
              "Each family has only five independent scenes. Seed repetitions are not new environments. Conditional rates include only recorded DRM successes; "
              "unknown/unreached DRM outcomes remain outside that conditional denominator.", "", "## Timing and saved-candidate statistics", "",
              "Cold time is measured by the parent process and includes imports, model/CUDA initialization, planning, validation and output. "
              "Warm query time is the recorded online block within that same execution, after setup; it is not a separate resident-service throughput trial. "
              "Timeouts retain measured wall time, including supervisor overhead; durations are not clipped to 60 seconds.", ""]
    for group in report["groups"]:
        lines += [f"### {group['objective']} / {group['family']}", "",
                  "| Field | Outcome stratum | Recorded / missing | Mean | p50 | p90 | p95 | Min | Max |",
                  "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
        for field, strata in group["recorded_statistics"].items():
            for label, stat in strata.items():
                lines.append(f"| {field} | {label} | {stat['count']} / {stat['missing_count']} | " +
                             " | ".join(fmt(stat[k]) for k in ["mean", "p50", "p90", "p95", "min", "max"]) + " |")
        lines += ["", "| Saved stage timer | All mean / n / missing | Success mean / n / missing | Failure mean / n / missing |",
                  "|---|---:|---:|---:|"]
        for stage, strata in group["stage_timing"].items():
            lines.append(f"| {stage} | " + " | ".join(
                f"{fmt(strata[label]['mean'])} / {strata[label]['count']} / {strata[label]['missing_count']}"
                for label in ["all_finished", "success", "failure"]) + " |")
        lines.append("")
    lines += ["## Offline roadmap cost", "", "| Roadmap | Requested / retained nodes | Directed edges | Wall seconds | Native map bytes |",
              "|---|---:|---:|---:|---:|"]
    for item in report["offline"]["roadmaps"]:
        lines.append(f"| {item['roadmap_id']} | {item['requested_nodes']} / {item['actual_nodes']} | "
                     f"{item['directed_edges']} | {item['offline_seconds']:.6f} | {item['native_map_bytes']} |")
    lines += ["", f"Three builds total: {report['offline']['total_offline_seconds']:.6f} seconds; "
              f"native serialized maps total {report['offline']['total_native_map_bytes']} bytes. " + report["offline"]["note"], "",
              "## Bounded numerical region coverage repair", "",
              "The fixed handoff cap is 1e-6 rad after normalizing each inequality row. The implementation keeps the normal matrix, "
              "fixed manifest endpoints and collision models unchanged; it only increases b by the required bounded positive anchor residual. "
              "Over-cap corrections are refused. Counts below include reused and recovery regions.", "",
              "| Objective | Family | Saved checks | Accepted | Rejected | Jobs with correction | Max accepted normalized correction | Max attempted correction |",
              "|---|---|---:|---:|---:|---:|---:|---:|"]
    for g in report["coverage"]["groups"]:
        lines.append(f"| {g['objective']} | {g['family']} | {g['calls']} | {g['accepted']} | {g['rejected']} | "
                     f"{g['jobs_with_any_correction']} | {fmt(g['maximum_accepted_normalized_correction'], 8)} | "
                     f"{fmt(g['maximum_normalized_correction'], 8)} |")
    lines += ["", "Every correction record and each rejected artifact path are retained in `development_results.json`.", "",
              "## Native stderr diagnostics", "", "Categories overlap; the original terminal failure statuses remain unchanged.", "",
              "| Category | Affected jobs |", "|---|---:|"]
    for key, count in report["native_stderr"]["category_counts"].items():
        lines.append(f"| {key} | {count} |")
    if not report["native_stderr"]["records"]:
        lines.append("| No matched native assertion/error/signal | 0 |")
    for rec in report["native_stderr"]["records"]:
        lines += ["", f"- `{rec['job_id']}`: {rec['status']}, exit {rec['exit_code']}; `{rec['stderr']}`."]
        for evidence in rec["evidence_lines"]:
            lines.append(f"  - Line {evidence['line']}: `{evidence['text'].replace('`', chr(39))}`")
    lines += ["", "A failed roadmap connection or native assertion is not evidence that the original continuous problem is globally infeasible. "
              "No failed endpoint, obstacle, radius or task objective was changed for this report.", "", "## Interpretation limits and provenance", "",
              "Ordered region/facet counts can include repeated regions; they are not unique inflation-call counts. "
              "Native pre-shortcut paths and collision-query counters are not exposed. Therefore no equal collision-query-budget claim is justified. "
              "The wall-clock budget is shared; the open-source solver route and numerical validation are described in `baseline_implementation_audit.md`. "
              "Development success-subset costs are descriptive and are not rankings against another method.", "",
              f"Verified {report['verification']['checked_files']} files ({report['verification']['checked_bytes']} bytes) against frozen hashes. "
              "All 360 executed evaluation configurations exactly match the single frozen manifest configuration. "
              f"Run manifest SHA-256: `{report['sources_sha256']['run_manifest']}`. "
              f"Summary SHA-256: `{report['sources_sha256']['summary']}`.", "",
              "Reproduce this CPU-only report from the repository:", "", "```bash",
              "bash local/python.sh benchmarks/generic_motion_v1/reports/development_result_analysis.py",
              "```", ""]
    return "\n".join(lines)


def analyze(run):
    run = Path(run).resolve()
    manifest, summary, selected = plan_and_completion(run)
    verification = verify_artifacts(run, manifest, selected)
    rows = [extract_row(job, attempt, result) for job, attempt, result, _, _ in selected]
    groups = []
    for objective in sorted(OBJECTIVES):
        for family in ["all", *sorted(FAMILIES)]:
            subset = [r for r in rows if r["objective"] == objective and (family == "all" or r["family"] == family)]
            group = group_rows(subset, objective, family)
            prior = next(g for g in summary["groups"] if g["method"] == "EI_ZO" and
                         g["objective"] == objective and g["family"] == family)
            for field in ["planned_jobs", "legal_input_jobs", "invalid_input_jobs", "status_counts"]:
                require(group[field] == prior[field], f"Summary {field} disagrees for {objective}/{family}")
            for field in ["unconditional_success", "conditional_on_drm_success"]:
                for key in ["numerator", "denominator", "estimate", "ci95", "independent_scene_clusters"]:
                    require(group[field][key] == prior[field][key], f"Recomputed {field}/{key} differs from summary")
            groups.append(group)
    return dict(schema_version=1, created_utc=datetime.now(timezone.utc).isoformat(),
        report_kind="complete_sealed_development_only", no_model_or_gpu_calls=True,
        run_directory=str(run), source_code_commit=manifest["source_code_commit"],
        planned_jobs=360, complete=True, legal_input_jobs=sum(r["input_legal"] for r in rows),
        fixed_cases=60, independent_scenes=20, roadmap_seeds=3,
        status_counts=summary["status_counts"], timeout_seconds=manifest["timeout_seconds"],
        verification=verification, groups=groups, coverage=coverage_audit(selected),
        native_stderr=stderr_audit(selected), offline=offline_audit(manifest, summary), rows=rows,
        sources_sha256=dict(run_manifest=sha(run / "run_manifest.json"), summary=sha(run / "summary/summary.json"),
            reporter=sha(__file__)),
        statistical_notes=["Scene cluster bootstrap, not independent seed bootstrap",
            "Missing values are never imputed to zero; failures remain in the legal planned denominator",
            "Region/facet counts use final metrics or last saved regions.json when final metrics are absent",
            "Candidate clearance/length on failures is descriptive, not a valid solution",
            "Cold external process wall time differs from warm query block within the same execution",
            "No formal test cases are evaluated or read by this report"])


def self_test():
    s = stats([1, 2, None, float("nan")])
    require(s["count"] == 2 and s["missing_count"] == 2 and s["mean"] == 1.5, "Missing-value regression")
    rows = [dict(scene_id=scene, input_legal=True, success=success, drm_success=True)
            for scene, success in [("a", True), ("a", True), ("b", False), ("b", False)]]
    ratio = cluster_ratio(rows)
    require(ratio["estimate"] == .5 and ratio["independent_scene_clusters"] == 2 and
            ratio["ci95"] == [0., 1.], "Scene pairing/bootstrap regression")
    require(cluster_ratio([])["estimate"] is None, "Zero denominator regression")
    require(cluster_ratio(rows[:2])["ci95"] is None, "One scene must not produce a CI")
    text = "python: drm_planner.cpp:249: GetClosestNonCollidingConfigurationInRoadMap: Assertion `closest_node >= 0' failed."
    require(set(classify_stderr(text, -6)) == {"native_assertion", "drm_no_connectable_node_assertion", "process_signal"},
            "Native assertion classification regression")
    require(not classify_stderr("Drake warning only", 0), "Warning wrongly classified as native failure")
    require(classify_stderr("", -15, True) == ["supervisor_timeout_termination"],
            "Budget termination wrongly classified as an unexpected native signal")
    with tempfile.TemporaryDirectory() as directory:
        run = Path(directory)
        (run / "run_manifest.json").write_text(json.dumps(dict(split="dev", formal_test_run=False, jobs=[])))
        try:
            plan_and_completion(run)
        except ValueError as exc:
            require("Expected 360" in str(exc), "Incorrect incomplete-run refusal")
        else:
            raise AssertionError("Incomplete run was accepted")
    print("PASS: finite/missing statistics, clustered pairing, zero/one-scene cases, native classification, incomplete-run refusal")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=BASE / "results/dev_eizo_v2")
    parser.add_argument("--output", type=Path, default=BASE / "reports")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    report = analyze(args.run)
    # Complete verification precedes either output. No partial report is emitted.
    args.output.mkdir(parents=True, exist_ok=True)
    for name, text in [("development_results.json", json.dumps(report, indent=2, allow_nan=False) + "\n"),
                       ("development_results.md", render(report))]:
        target = args.output / name
        temporary = target.with_suffix(target.suffix + ".tmp")
        temporary.write_text(text)
        temporary.replace(target)
    print(json.dumps(dict(complete=True, planned_jobs=360, status_counts=report["status_counts"],
        checked_files=report["verification"]["checked_files"], coverage_records=report["coverage"]["total_records"],
        coverage_rejections=report["coverage"]["rejected_records"],
        native_stderr_categories=report["native_stderr"]["category_counts"], output=str(args.output)), indent=2))


if __name__ == "__main__":
    main()
