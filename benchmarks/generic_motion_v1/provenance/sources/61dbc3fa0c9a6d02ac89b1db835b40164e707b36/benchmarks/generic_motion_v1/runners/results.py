"""One index of actual attempts, with explicitly selected comparable cohorts.

This is a reader of existing evidence: it never runs a planner, selects the best
retry, rewrites an attempt, or infers a first-feasible time from final latency.
The selection file names runs and comparison modes; old and current protocols
remain separate unless their compatibility is explicitly documented there.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
from functools import lru_cache
import hashlib
import itertools
import json
import math
import os
from pathlib import Path

from ..common import BASE, ROOT, read, sha, write, resolve_case
from .summarize import _cluster_ratio


SUCCESS = "success_numerical"
STATUS = {
    "success_validated_numerical": SUCCESS,
    "success_numerical": SUCCESS,
    "invalid_endpoint": "invalid_input",
    "invalid_scene_geometry": "invalid_input",
    "invalid_input": "invalid_input",
    "generation_failed": "invalid_input",
    "frontend_failed": "no_solution_within_budget",
    "planner_failed": "no_solution_within_budget",
    "planner_timeout": "no_solution_within_budget",
    "no_solution_within_budget": "no_solution_within_budget",
    "timeout": "timeout",
    "solver_failed": "solver_failure",
    "solver_failure": "solver_failure",
    "collision_failed": "validation_failed",
    "limit_failed": "validation_failed",
    "validation_undetermined": "validation_failed",
    "validation_failed": "validation_failed",
    "model_consistency_failed": "infrastructure_error",
    "unsupported": "infrastructure_error",
    "infrastructure_error": "infrastructure_error",
}
LABELS = {
    ("EI_ZO", "minimum_distance"): "DRM + EI-ZO + LSCS",
    ("EI_ZO", "minimum_time"): "DRM + EI-ZO + SCSTO",
    ("LGCS", "minimum_distance"): "DRM + EI-ZO + LGCS",
    ("NTO_PAPER", "minimum_distance"): "NTO path",
    ("NTO_PAPER", "minimum_time"): "NTO time",
    ("NTO_REFINED", "minimum_distance"): "NTO refined path",
    ("NTO_REFINED", "minimum_time"): "NTO refined time",
    ("NTO", "minimum_distance"): "NTO IPOPT adaptation path",
    ("NTO", "minimum_time"): "NTO IPOPT adaptation time",
    ("RRTCONNECT", "minimum_distance"): "OMPL RRTConnect",
    ("RRTSTAR", "minimum_distance"): "OMPL RRT*",
    ("CHOMP", "minimum_distance"): "CHOMP adaptation (C-space functional)",
    ("STOMP", "minimum_distance"): "STOMP (official core, canonical task)",
    ("TRAJOPT", "minimum_distance"): "TrajOpt adaptation (sampled collision)",
}
RECORD_FIELDS = (
    "record_id", "run", "job_id", "attempt_number", "previous_attempt", "lifecycle",
    "selected_comparisons", "problem_id", "scene_id", "cluster_id", "split", "category",
    "method", "method_label", "objective", "planner_seed", "replica_seed", "roadmap_id",
    "input_valid", "status", "raw_status", "failure_reason", "failure_stage",
    "validation_passed", "validation_kind", "solver_reported_success", "solver_converged",
    "budget_seconds", "cold_e2e_seconds", "worker_seconds", "within_call_query_seconds",
    "first_valid_seconds", "first_valid_time_evidence", "observed_first_valid_adapter_seconds",
    "progress_time_origin", "progress_path", "path_length_rad", "trajectory_duration_s",
    "objective_value", "objective_unit", "native_objective", "minimum_sampled_clearance_m",
    "model_setup_seconds", "gpu_initialization_warmup_seconds", "roadmap_load_seconds",
    "online_collision_map_seconds", "drm_path_seconds", "drm_path_validation_seconds",
    "sampling_planning_seconds", "eizo_inflation_seconds", "trajectory_optimization_seconds",
    "collision_recovery_seconds", "validation_seconds", "backend_seconds",
    "timing_json", "offline_roadmap_seconds", "roadmap_nodes", "roadmap_bytes",
    "frontend_success", "frontend_validated", "frontend_path_sha256", "initial_regions_sha256",
    "source_commit", "source_fingerprint", "config_sha256", "validation_protocol_sha256", "input_sha256", "robot_sha256",
    "geometry_sha256", "run_manifest_sha256", "receipt_sha256", "receipt_integrity",
    "candidate_path", "trajectory_samples_path", "regions_path", "validation_path",
    "stdout_path", "stderr_path", "result_path", "attempt_path",
)
SUMMARY_FIELDS = (
    "comparison", "mode", "split", "objective", "category", "row_type", "method", "method_b",
    "planned_jobs", "actual_attempts", "legal_jobs", "finished_legal_jobs", "pending_jobs",
    "invalid_input_jobs", "scene_clusters", "success_numerical", "no_solution_within_budget",
    "timeout", "solver_failure", "validation_failed", "infrastructure_error",
    "success_rate", "success_ci_low", "success_ci_high", "metric", "count", "mean", "median",
    "p90", "ci_low", "ci_high", "unit", "scope",
)
STAGE_KEYS = tuple(k for k in RECORD_FIELDS if k.endswith("_seconds") and k not in {
    "budget_seconds", "cold_e2e_seconds", "worker_seconds", "within_call_query_seconds",
    "first_valid_seconds", "observed_first_valid_adapter_seconds", "backend_seconds", "offline_roadmap_seconds"})
VALIDATION_KEYS = ("validation_step_rad", "validation_minimum_samples", "validation_maximum_samples",
                   "endpoint_tolerance_rad", "feasibility_tolerance", "velocity_limit_rad_s",
                   "acceleration_limit_rad_s2")
BACKEND_KEYS = ("eizo_inflation_seconds", "alternate_drm_path_seconds",
                "trajectory_optimization_seconds", "collision_recovery_seconds", "validation_seconds")


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _relative(path):
    if path is None:
        return None
    path = Path(path).resolve()
    return str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)


def _path(value):
    path = Path(value)
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def _load(path, default=None):
    return read(path) if Path(path).is_file() else default


@lru_cache(maxsize=128)
def _roadmap_metadata(path):
    return _load(path, {})


@lru_cache(maxsize=128)
def _roadmap_bytes(path):
    return sum(p.stat().st_size for p in path.parent.iterdir() if p.is_file()) if path.is_file() else None


def _existing(path):
    return _relative(path) if path.is_file() else None


def _number(*values):
    return next((v for v in values if _finite(v)), None)


def _csv(path, rows, fields):
    """Atomic derived output replacement; the raw evidence is never edited."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({k: (json.dumps(v, separators=(",", ":"), sort_keys=True)
                                  if isinstance(v, (dict, list, tuple)) else v)
                             for k, v in row.items()})
    os.replace(temporary, path)


def discover(roots):
    """Only manifest-backed runs are benchmark runs; ad-hoc probes stay probes."""
    found = set()
    for root in map(_path, roots):
        if (root / "run_manifest.json").is_file():
            found.add(root)
        elif root.is_dir():
            found.update(p.parent.resolve() for p in root.rglob("run_manifest.json")
                         if "source_snapshot" not in p.parts)
    return sorted(found)


def _receipt_check(attempt, receipt):
    if receipt is None:
        return None
    errors = []
    for name, digest in receipt.get("files", {}).items():
        file = (attempt / name).resolve()
        if (not file.is_relative_to(attempt.resolve()) or not file.is_file()
                or sha(file) != digest):
            errors.append(name)
    if "result.json" not in receipt.get("files", {}):
        errors.append("missing_result_binding")
    actual = {str(p.relative_to(attempt)) for p in attempt.rglob("*")
              if p.is_file() and p.name != "receipt.json" and "replay" not in p.relative_to(attempt).parts}
    if actual - receipt.get("files", {}).keys():
        errors.append("unsealed_artifacts")
    return errors


def attempt_row(run, manifest, job, attempt, *, verify_files=True, case_cache=None):
    """Read one actual attempt; no last/best attempt selection happens here."""
    result = _load(attempt / "result.json", {})
    process = _load(attempt / "process.json", {})
    receipt = _load(attempt / "receipt.json")
    validation = _load(attempt / "validation.json", {})
    candidate = _load(attempt / "candidate.json", {})
    case = resolve_case(job, case_cache)
    timing = result.get("timing", result.get("timings", {})) or {}
    raw = result.get("status", process.get("status"))
    lifecycle = "unsealed" if receipt is None else "retry_pending" if receipt.get("retry_pending") else "terminal"
    valid = case.get("validation_status") == "valid_input"
    status = STATUS.get(raw, "infrastructure_error") if lifecycle == "terminal" else None
    if lifecycle == "terminal" and valid and raw == "invalid_endpoint":
        status = "validation_failed"  # The fixed input is legal; the candidate missed it.
    integrity_errors = _receipt_check(attempt, receipt) if verify_files else None
    if receipt and verify_files and receipt.get("status") != raw:
        integrity_errors.append("receipt/result status mismatch")
    validation_passed = validation.get("passed")
    reason = result.get("error", result.get("exception"))
    if status == SUCCESS and (not valid or validation_passed is not True):
        status = "validation_failed"
        reason = "Recorded success lacks legal input and standalone passing validation"
    if integrity_errors:
        status = "infrastructure_error"
        reason = "Receipt artifact mismatch: " + ", ".join(integrity_errors)
    if status == SUCCESS and (process.get("exit_code") != 0 or process.get("timed_out") is not False):
        status = "infrastructure_error"
        reason = "Recorded success contradicts external process evidence"
    objective = job.get("objective", result.get("objective"))
    cost = _number(validation.get("cost")) if status == SUCCESS else None
    expected_unit = {"minimum_distance": "rad", "minimum_time": "s"}.get(objective)
    if status == SUCCESS and (cost is None or cost < 0 or validation.get("cost_unit") != expected_unit):
        status = "validation_failed"
        reason = "Recorded success has invalid objective or unit"
        cost = None
    success = status == SUCCESS
    family = case.get("family", job.get("family"))
    scene = case.get("scene_id", job.get("scene_id"))
    frontend = _load(attempt / "frontend_validation/validation.json", {})
    metadata_path = _path(job["roadmap_dir"]) / "metadata.json" if job.get("roadmap_dir") else None
    metadata = _roadmap_metadata(metadata_path) if metadata_path else {}
    roadmap_used = result.get("roadmap_used", job.get("roadmap_used", True))
    replica = job.get("replica_seed", metadata.get("seed"))
    if replica is None:
        pairing = [p["drm_seed"] for p in case.get("paired_seeds", [])
                   if p.get("method_seed") == job.get("method_seed")]
        replica = pairing[0] if len(pairing) == 1 else None
    number = int(attempt.name)
    prior = sorted(p for p in attempt.parent.glob("[0-9][0-9][0-9][0-9]") if int(p.name) < number)
    backend_complete = (candidate and validation and not result.get("failed_stage")
                        and not result.get("exception") and lifecycle == "terminal"
                        and status not in {"timeout", "infrastructure_error"})
    backend_keys = BACKEND_KEYS if roadmap_used else ("sampling_planning_seconds", "validation_seconds")
    required = ("trajectory_optimization_seconds", "validation_seconds") if roadmap_used else backend_keys
    backend_seconds = (sum(timing.get(k, 0) for k in backend_keys)
                       if backend_complete and all(_finite(timing.get(k)) for k in required) else None)
    # A final output timestamp is NOT a first-valid timestamp. Only an explicit
    # independently validated event with a documented origin can supply it.
    first = result.get("first_validated_solution")
    first_valid = (first.get("elapsed_seconds") if isinstance(first, dict)
                   and first.get("validation_passed") is True
                   and first.get("clock_origin") == "external_query_start"
                   and _finite(first.get("elapsed_seconds")) else None)
    progress = _load(attempt / "validated_progress/summary.json", {})
    adapter_first = (_number(progress.get("first_valid_seconds"))
                     if progress.get("validated_observations", 0) > 0
                     and progress.get("validation_charged_to_budget") is True
                     and str(progress.get("time_origin", "")).startswith("OMPL adapter entry") else None)
    path = attempt / "path.npy"
    initial_region_name = {"EI_ZO": "rounds/000/regions.npz",
                           "LGCS": "first_corridor/regions.npz"}.get(job.get("method"))
    initial_regions = attempt / initial_region_name if initial_region_name else None
    output = dict(
        record_id=_fingerprint(_relative(attempt)), run=_relative(run), job_id=job["job_id"],
        attempt_number=number, previous_attempt=_relative(prior[-1]) if prior else None,
        lifecycle=lifecycle, selected_comparisons=[], problem_id=case.get("problem_id", job.get("case_id")),
        scene_id=scene, cluster_id="open_control_shared_base_world" if family == "open_control" else scene,
        split=case.get("split", manifest.get("split")), category=family, method=job.get("method"),
        method_label=LABELS.get((job.get("method"), objective), job.get("method")), objective=objective,
        planner_seed=job.get("method_seed"), replica_seed=replica, roadmap_id=job.get("roadmap_id"),
        input_valid=valid, status=status, raw_status=raw, failure_reason=reason,
        failure_stage=result.get("failed_stage", result.get("final_stage")) if not success else None,
        validation_passed=validation_passed, validation_kind=validation.get("check"),
        solver_reported_success=candidate.get("solver_success"), solver_converged=candidate.get("solver_converged"),
        budget_seconds=manifest.get("timeout_seconds"),
        cold_e2e_seconds=_number(process.get("cold_e2e_seconds"), result.get("cold_e2e_seconds")),
        worker_seconds=_number(timing.get("cold_e2e_seconds")),
        within_call_query_seconds=_number(timing.get("warm_query_seconds")),
        first_valid_seconds=first_valid, first_valid_time_evidence=first if first_valid is not None else None,
        observed_first_valid_adapter_seconds=adapter_first, progress_time_origin=progress.get("time_origin"),
        progress_path=_existing(attempt / "validated_progress/records.jsonl"),
        path_length_rad=_number(validation.get("path_length_rad")) if success else None,
        trajectory_duration_s=_number(validation.get("duration_s")) if success and objective == "minimum_time" else None,
        objective_value=cost, objective_unit=expected_unit,
        native_objective=candidate.get("native_objective", result.get("native_objective")),
        minimum_sampled_clearance_m=_number(validation.get("minimum_sampled_clearance_m")),
        backend_seconds=backend_seconds, timing_json=timing,
        offline_roadmap_seconds=result.get("offline_roadmap_seconds") if roadmap_used else None,
        roadmap_nodes=metadata.get("actual_nodes") if roadmap_used else None,
        roadmap_bytes=_roadmap_bytes(metadata_path) if roadmap_used and metadata_path else None,
        frontend_success=result.get("frontend_success") if roadmap_used else None,
        frontend_validated=frontend.get("passed") if roadmap_used else None,
        frontend_path_sha256=sha(path) if roadmap_used and path.is_file() else None,
        initial_regions_sha256=sha(initial_regions) if initial_regions is not None and initial_regions.is_file() else None,
        source_commit=manifest.get("source_code_commit"), source_fingerprint=_fingerprint(manifest.get("source_hashes")),
        config_sha256=_fingerprint(job.get("evaluation_config")),
        validation_protocol_sha256=_fingerprint({k: (job.get("evaluation_config") or {}).get(k) for k in VALIDATION_KEYS}),
        input_sha256=case.get("problem_sha256"),
        robot_sha256=case.get("robot_sha256"), geometry_sha256=case.get("collision_geometry_sha256"),
        run_manifest_sha256=manifest.get("_index_manifest_sha256") or sha(run / "run_manifest.json"),
        receipt_sha256=sha(attempt / "receipt.json") if receipt else None,
        receipt_integrity=(not integrity_errors) if verify_files and receipt else None,
        candidate_path=_existing(attempt / "candidate.npz"), trajectory_samples_path=_existing(attempt / "trajectory_samples.npz"),
        regions_path=_existing(attempt / "regions.npz"), validation_path=_existing(attempt / "validation.json"),
        stdout_path=_existing(attempt / "stdout.log"), stderr_path=_existing(attempt / "stderr.log"),
        result_path=_existing(attempt / "result.json"), attempt_path=_relative(attempt),
    )
    output.update({key: _number(timing.get(key)) for key in STAGE_KEYS})
    return output


def collect(runs, *, verify_files=True):
    _roadmap_metadata.cache_clear()
    _roadmap_bytes.cache_clear()
    rows, inventory = [], []
    for run in runs:
        manifest = read(run / "run_manifest.json")
        manifest["_index_manifest_sha256"] = sha(run / "run_manifest.json")
        case_cache = {}
        counts = Counter()
        statuses = Counter()
        configs, seeds, methods = set(), set(), set()
        start = len(rows)
        for job in manifest.get("jobs", []):
            methods.add((job["method"], job["objective"]))
            configs.add(_fingerprint(job.get("evaluation_config")))
            attempts = sorted((run / "jobs" / job["job_id"] / "attempts").glob("[0-9][0-9][0-9][0-9]"))
            for attempt in attempts:
                row = attempt_row(run, manifest, job, attempt, verify_files=verify_files, case_cache=case_cache)
                rows.append(row)
                counts[row["lifecycle"]] += 1
                statuses[row["status"] or row["lifecycle"]] += 1
                seeds.add(row["replica_seed"])
        sample = rows[start:]
        inventory.append(dict(run=_relative(run), split=manifest.get("split"),
            planned_jobs=len(manifest.get("jobs", [])), actual_attempts=len(sample),
            actual_legal_attempts=sum(r["input_valid"] for r in sample), lifecycle=dict(counts), status_counts=dict(statuses),
            method_objectives=[dict(method=m, objective=o) for m, o in sorted(methods)],
            replica_seeds=sorted(seeds, key=str), source_commit=manifest.get("source_code_commit"),
            source_fingerprint=_fingerprint(manifest.get("source_hashes")), config_hashes=sorted(configs),
            budget_seconds=manifest.get("timeout_seconds"), manifest_sha256=sha(run / "run_manifest.json")))
    return rows, inventory


def _selected_rows(rows, spec, verified_runs=None):
    """Choose the first terminal attempt, never the fastest/cheapest/successful."""
    selected = []
    verified_runs = {} if verified_runs is None else verified_runs
    planned = []
    for item in spec["runs"]:
        item = {"path": item} if isinstance(item, str) else item
        run = _relative(_path(item["path"]))
        if not (_path(run) / "run_manifest.json").is_file() and item.get("allow_pending") is True:
            continue
        methods = set(item.get("methods", []))
        objectives = set(item.get("objectives", []))
        manifest = read(_path(run) / "run_manifest.json")
        from ..batch import verify
        if run not in verified_runs:
            verified_runs[run] = verify(_path(run), allow_unsealed=True)
        integrity = verified_runs[run]
        if not integrity["integrity_passed"]:
            raise ValueError("Selected run provenance is invalid: " + run + ": " + json.dumps(integrity["errors"]))
        jobs = [j for j in manifest["jobs"] if (not methods or j["method"] in methods)
                and (not objectives or j["objective"] in objectives)]
        allowed = {j["job_id"] for j in jobs}
        by_job = defaultdict(list)
        for row in rows:
            if row["run"] == run and row["job_id"] in allowed:
                by_job[row["job_id"]].append(row)
        case_cache = {}
        for job in jobs:
            attempts = sorted(by_job[job["job_id"]], key=lambda row: row["attempt_number"])
            terminal = [r for r in attempts if r["lifecycle"] == "terminal"]
            chosen = terminal[0] if terminal else None
            planned.append(dict(run=run, job=job, case=resolve_case(job, case_cache),
                                row=chosen, actual_attempts=len(attempts)))
            if chosen:
                if chosen["receipt_integrity"] is not True:
                    raise ValueError(f"Selected evidence lacks verified receipt: {chosen['attempt_path']}")
                selected.append(chosen)
    keys = [(r["problem_id"], r["replica_seed"], r["method"], r["objective"]) for r in selected]
    if len(keys) != len(set(keys)):
        raise ValueError("Duplicate observation in comparison; select one run per fixed task/seed/method/objective")
    budgets = {r["budget_seconds"] for r in selected}
    models = {(r["robot_sha256"], r["geometry_sha256"]) for r in selected}
    if len(budgets) > 1 or len(models) > 1:
        raise ValueError("A comparison cannot mix budgets or robot/collision models")
    if len({r["validation_protocol_sha256"] for r in selected}) > 1:
        raise ValueError("A comparison cannot mix numerical validation or motion-limit protocols")
    if ((len({r["source_fingerprint"] for r in selected}) > 1
         or len({r["config_sha256"] for r in selected}) > 1)
            and not spec.get("compatibility_note")):
        raise ValueError("Different source/configuration fingerprints require an explicit compatibility_note")
    inputs = defaultdict(set)
    for row in selected:
        inputs[row["problem_id"]].add(row["input_sha256"])
    if any(len(values) > 1 for values in inputs.values()):
        raise ValueError("The same problem_id refers to different frozen inputs")
    if spec.get("mode", "end_to_end") == "fixed_frontend":
        groups = defaultdict(list)
        for row in selected:
            groups[(row["problem_id"], row["replica_seed"], row["objective"])].append(row)
        for group in groups.values():
            eligible = [r for r in group if r["frontend_validated"] is True]
            if eligible and (len(eligible) != len(group) or
                             len({r["frontend_path_sha256"] for r in eligible}) != 1 or
                             eligible[0]["frontend_path_sha256"] is None):
                raise ValueError("Fixed-front-end methods do not share the identical validated path")
            if spec.get("require_shared_initial_regions") and eligible and (
                    len({r["initial_regions_sha256"] for r in eligible}) != 1
                    or eligible[0]["initial_regions_sha256"] is None):
                raise ValueError("Shared initial regions are requested but not established by saved evidence")
        selected = [r for r in selected if r["frontend_validated"] is True]
    elif spec.get("mode", "end_to_end") != "end_to_end":
        raise ValueError("mode must be end_to_end or fixed_frontend")
    for row in selected:
        row["selected_comparisons"].append(spec["id"])
    return selected, planned


def _stats(rows, field):
    usable = [r for r in rows if _finite(r.get(field))]
    values = sorted(r[field] for r in usable)
    n = len(values)
    if not n:
        return dict(count=0, mean=None, median=None, p90=None, ci_low=None, ci_high=None)
    import numpy as np
    bootstrap = _cluster_ratio(usable, lambda r: r[field], lambda r: 1)
    ci = bootstrap["ci95"] or (None, None)
    return dict(count=n, mean=sum(values) / n, median=float(np.quantile(values, .5)),
                p90=float(np.quantile(values, .9)), ci_low=ci[0], ci_high=ci[1])


def summarize(rows, selection):
    output = []
    verified_runs = {}
    for spec in selection.get("comparisons", []):
        selected, planned = _selected_rows(rows, spec, verified_runs)
        mode = spec.get("mode", "end_to_end")
        if not planned:
            output.append(dict(comparison=spec["id"], mode=mode, row_type="coverage",
                               scope="Run not yet prepared; no planning observations or scores are claimed."))
        blocks = sorted({(p["case"]["split"], p["job"]["objective"]) for p in planned})
        for split, objective in blocks:
            block = [r for r in selected if (r["split"], r["objective"]) == (split, objective)]
            categories = ["all"] + sorted({r["category"] for r in block})
            for category in categories:
                pool = [r for r in block if category == "all" or r["category"] == category]
                base = dict(comparison=spec["id"], mode=mode, split=split, objective=objective, category=category)
                methods = sorted({p["job"]["method"] for p in planned if
                                  (p["case"]["split"], p["job"]["objective"]) == (split, objective)})
                for method in methods:
                    group = [r for r in pool if r["method"] == method]
                    declarations = [p for p in planned if p["job"]["method"] == method
                                    and (p["case"]["split"], p["job"]["objective"]) == (split, objective)
                                    and (category == "all" or p["case"]["family"] == category)]
                    legal = [r for r in group if r["input_valid"]]
                    success = [r for r in legal if r["status"] == SUCCESS]
                    planned_legal = sum(p["case"].get("validation_status") == "valid_input" for p in declarations)
                    stat = Counter(r["status"] for r in legal)
                    rate = _cluster_ratio(legal, lambda r: r["status"] == SUCCESS, lambda r: 1)
                    ci = rate["ci95"] or (None, None)
                    output.append(dict(base, row_type="status", method=method,
                        planned_jobs=len(declarations), actual_attempts=sum(p["actual_attempts"] for p in declarations),
                        legal_jobs=planned_legal, finished_legal_jobs=len(legal),
                        pending_jobs=sum(p["row"] is None for p in declarations),
                        invalid_input_jobs=len(declarations) - planned_legal,
                        scene_clusters=rate["independent_scene_clusters"], **{s: stat[s] for s in set(STATUS.values()) if s != "invalid_input"},
                        success_rate=rate["estimate"], success_ci_low=ci[0], success_ci_high=ci[1],
                        scope="First terminal attempt per declared job. Rate denominator is finished legal jobs; pending coverage is reported separately."
                              if mode == "end_to_end" else "Conditional on the identical validated frontend path; frontend failures excluded from backend denominator."))
                    metrics = [("objective_value", success, "rad" if objective == "minimum_distance" else "s", "validated_success_only_descriptive"),
                               ("path_length_rad", success, "rad", "validated_success_only_descriptive"),
                               ("trajectory_duration_s", success, "s", "validated_success_only_descriptive"),
                               ("first_valid_seconds", legal, "s", "explicit_validated_event_only"),
                               ("observed_first_valid_adapter_seconds", legal, "s", "observed_upper_bound_from_OMPL_adapter_entry_not_cold_query_time"),
                               ("cold_e2e_seconds" if mode == "end_to_end" else "backend_seconds", legal, "s", "all_finished_legal"),
                               ("within_call_query_seconds", legal, "s", "derived_within_cold_call_not_independent_warm_run")]
                    for field, sample, unit, scope in metrics:
                        if field == "trajectory_duration_s" and objective != "minimum_time":
                            continue
                        output.append(dict(base, row_type="metric", method=method, metric=field, unit=unit,
                                           scope=scope, **_stats(sample, field)))
                # Matched task and algorithm repetition; roadmap is an input to
                # DRM methods, not a requirement imposed on independent OMPL.
                for a, b in itertools.combinations(methods, 2):
                    by_a = {(r["problem_id"], r["replica_seed"]): r for r in pool if r["method"] == a and r["input_valid"]}
                    by_b = {(r["problem_id"], r["replica_seed"]): r for r in pool if r["method"] == b and r["input_valid"]}
                    pairs = [(by_a[k], by_b[k]) for k in sorted(by_a.keys() & by_b.keys(), key=str)]
                    for left, right in pairs:
                        if left["planner_seed"] != right["planner_seed"] or left["input_sha256"] != right["input_sha256"]:
                            raise ValueError("Paired rows disagree on task or planner seed")
                    fields = [("objective_value", "common_success_a_minus_b", "rad" if objective == "minimum_distance" else "s"),
                              ("cold_e2e_seconds" if mode == "end_to_end" else "backend_seconds", "paired_finished_a_minus_b", "s"),
                              ("success", "paired_success_rate_a_minus_b", "fraction")]
                    for field, scope, unit in fields:
                        differences = []
                        for left, right in pairs:
                            if field == "objective_value" and not (left["status"] == right["status"] == SUCCESS):
                                continue
                            x = int(left["status"] == SUCCESS) if field == "success" else left.get(field)
                            y = int(right["status"] == SUCCESS) if field == "success" else right.get(field)
                            if _finite(x) and _finite(y):
                                differences.append(dict(cluster_id=left["cluster_id"], difference=x - y))
                        output.append(dict(base, row_type="paired", method=a, method_b=b, metric=field,
                                           scope=scope, unit=unit, **_stats(differences, "difference")))
    return output


def export(output=None, *, roots=None, selection=None, verify_files=True):
    output = _path(output or "results")
    roots = roots or [BASE / "results"]
    selection_value = read(_path(selection)) if selection else {"comparisons": []}
    # Include explicitly selected new runs even when they live outside the
    # historical run root. Rebuilding the index discovers appended attempts.
    extra = [item if isinstance(item, str) else item["path"]
             for spec in selection_value.get("comparisons", []) for item in spec["runs"]]
    runs = discover(list(roots) + extra)
    rows, inventory = collect(runs, verify_files=verify_files)
    summary = summarize(rows, selection_value)
    _csv(output / "records.csv", rows, RECORD_FIELDS)
    _csv(output / "summary.csv", summary, SUMMARY_FIELDS)
    write(output / "inventory.json", dict(
        scope="Every actual attempt belonging to a run_manifest; standalone diagnostic probes are not planning queries.",
        actual_attempts=len(rows), runs=inventory,
        summary_selection=_relative(_path(selection)) if selection else None,
        summary_selection_sha256=sha(_path(selection)) if selection else None,
        pending_run_paths=sorted({_relative(_path(item["path"]))
            for spec in selection_value.get("comparisons", []) for item in spec["runs"]
            if isinstance(item, dict) and item.get("allow_pending") and not (_path(item["path"]) / "run_manifest.json").is_file()}),
        mapped_status_counts=dict(Counter(r["status"] or r["lifecycle"] for r in rows)),
        receipt_files_verified=verify_files,
        attempt_policy="All attempts indexed; first terminal attempt only per selected job; no best-of.",
        first_valid_time_policy="Blank without an explicit validated solution event timed from external query start.",
        warm_time_policy="within_call_query_seconds subtracts setup from a cold run; it is not an independently timed warm experiment.",
        interval_policy="95% scene-cluster bootstrap; shared open_control base world is one cluster; no CI for one cluster.",
    ))
    _index_links(output, runs)
    _figures(output, rows, summary)
    return dict(output=_relative(output), runs=len(runs), actual_attempts=len(rows), summary_rows=len(summary))


def _index_links(output, runs):
    """One link per run, not a copy of every array or a link per tiny file."""
    for folder in ("trajectories", "logs"):
        target = output / folder
        target.mkdir(parents=True, exist_ok=True)
        for run in runs:
            link = target / run.name
            source = run / "jobs"
            if not source.is_dir():
                continue
            relative = os.path.relpath(source, target)
            if link.is_symlink():
                if link.resolve() != source.resolve():
                    raise ValueError("Result index link points at a different run: " + str(link))
            elif link.exists():
                raise ValueError("Refusing to replace a non-link result artifact: " + str(link))
            else:
                link.symlink_to(relative, target_is_directory=True)


def _figures(output, rows, summary):
    """Compact evidence plots; unsupported intermediate curves stay absent."""
    panels = sorted({(r["comparison"], r["objective"]) for r in summary
                     if r["row_type"] == "status" and r["mode"] == "end_to_end"
                     and r["category"] == "all" and r["comparison"].startswith("current_")})
    if not panels:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    directory = output / "figures"
    directory.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(len(panels), 1, figsize=(11, max(3.3, len(panels) * 3.3)), squeeze=False)
    for axis, (comparison, objective) in zip(axes[:, 0], panels):
        group = [r for r in summary if r["row_type"] == "status" and r["category"] == "all"
                 and (r["comparison"], r["objective"], r["mode"]) == (comparison, objective, "end_to_end")]
        group = [r for r in group if r["success_rate"] is not None]
        positions = np.arange(len(group))
        rates = np.asarray([r["success_rate"] for r in group]) * 100
        axis.barh(positions, rates, color="#277da1")
        for y, r, rate in zip(positions, group, rates):
            if r["success_ci_low"] is not None:
                axis.plot([100*r["success_ci_low"], 100*r["success_ci_high"]], [y, y], color="black", lw=1)
            axis.text(min(rate + 1, 97), y, f"{r['success_numerical']}/{r['finished_legal_jobs']}", va="center", fontsize=8)
        axis.set_yticks(positions, [r["method"] for r in group])
        axis.set_xlim(0, 112)
        axis.set_title(f"{comparison} / {objective}: numerical validation pass rate")
        axis.set_xlabel("Finished legal queries (%) — lines: 95% scene-cluster bootstrap")
    fig.tight_layout()
    fig.savefig(directory / "success_rate.png", dpi=160)
    plt.close(fig)
    # All components below refer to the SAME successful rows. Missing genuine
    # stage measurements exclude a row; inapplicable method stages are omitted.
    stage_groups = [("setup", ("model_setup_seconds", "gpu_initialization_warmup_seconds", "roadmap_load_seconds")),
                    ("front end", ("online_collision_map_seconds", "drm_path_seconds", "drm_path_validation_seconds")),
                    ("sampling", ("sampling_planning_seconds",)),
                    ("regions", ("eizo_inflation_seconds", "alternate_drm_path_seconds", "collision_recovery_seconds")),
                    ("optimization", ("trajectory_optimization_seconds",)),
                    ("final validation", ("validation_seconds",))]
    fig, axes = plt.subplots(len(panels), 1, figsize=(11, max(3.3, len(panels) * 3.3)), squeeze=False)
    for axis, (comparison, objective) in zip(axes[:, 0], panels):
        eligible = [r for r in rows if comparison in r["selected_comparisons"] and r["objective"] == objective
                    and r["status"] == SUCCESS and _finite(r["backend_seconds"]) and _finite(r["cold_e2e_seconds"])]
        methods = sorted({r["method"] for r in eligible})
        positions = np.arange(len(methods))
        values = [[] for _ in range(len(stage_groups) + 1)]
        counts = []
        for method in methods:
            entries = []
            for row in eligible:
                if row["method"] != method:
                    continue
                timing = row["timing_json"]
                components = [sum(timing.get(k, 0) for k in keys) for _, keys in stage_groups]
                if all(_finite(v) and v >= 0 for v in components) and sum(components) <= row["cold_e2e_seconds"] + 1e-6:
                    entries.append(components + [max(0., row["cold_e2e_seconds"] - sum(components))])
            counts.append(len(entries))
            for index in range(len(values)):
                values[index].append(float(np.mean([r[index] for r in entries])) if entries else 0.)
        left = np.zeros(len(methods))
        for label, value in zip([name for name, _ in stage_groups] + ["other/startup"], values):
            axis.barh(positions, value, left=left, label=label)
            left += value
        axis.set_yticks(positions, [f"{m} (n={n})" for m, n in zip(methods, counts)])
        axis.set_title(f"{comparison} / {objective}: successful-query mean cold time")
        axis.set_xlabel("Seconds; descriptive success subsets, not a paired speed ranking")
    axes[0, 0].legend(ncol=4, fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(directory / "time_breakdown.png", dpi=160)
    plt.close(fig)
    progress_rows = [r for r in rows if any(c.startswith("current_") for c in r["selected_comparisons"])
                     and r["progress_path"] and r["receipt_integrity"] is True]
    representatives = {}
    for row in sorted(progress_rows, key=lambda r: (r["problem_id"], r["replica_seed"], r["method"])):
        records = [json.loads(line) for line in _path(row["progress_path"]).read_text().splitlines() if line.strip()]
        usable = [v for v in records if v.get("within_budget") and v.get("validation_status") == "success_validated_numerical"
                  and _finite(v.get("evidence_available_seconds", v.get("validation_completed_seconds")))
                  and _finite(v.get("best_validated_cost_rad"))]
        if usable:
            representatives.setdefault((row["split"], row["category"]), (row, usable))
    if representatives:
        fig, axes = plt.subplots(len(representatives), 1, figsize=(10, 2.8*len(representatives)), squeeze=False)
        for axis, (row, curve) in zip(axes[:, 0], representatives.values()):
            axis.step([p.get("evidence_available_seconds", p.get("validation_completed_seconds")) for p in curve], [p["best_validated_cost_rad"] for p in curve], where="post", marker="o")
            axis.set_title(f"{row['problem_id']} / seed {row['replica_seed']} / {row['method']}")
            axis.set_ylabel("Best validated length (rad)")
            axis.set_xlabel("Seconds from OMPL adapter entry; includes observation validation")
        fig.suptitle("Measured representative progress: first available case per split/category, no inferred curve")
        fig.tight_layout()
        fig.savefig(directory / "validated_progress.png", dpi=160)
        plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="results")
    parser.add_argument("--roots", nargs="+")
    parser.add_argument("--selection", help="JSON specifying comparison ids, modes and exact run paths")
    args = parser.parse_args(argv)
    print(json.dumps(export(args.output, roots=args.roots, selection=args.selection), indent=2))


if __name__ == "__main__":
    main()
