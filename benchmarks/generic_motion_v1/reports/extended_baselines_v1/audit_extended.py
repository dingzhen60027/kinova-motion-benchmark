"""Independent, read-only audit of the sealed extended-baseline study.

This file deliberately imports no benchmark implementation, optimizer, Drake,
CUDA, or report aggregator. It recomputes numerical identities from saved
arrays; it does not rerun collision checking or claim continuous safety.

Run only after the study and metrics finish:
  bash local/python.sh benchmarks/generic_motion_v1/reports/extended_baselines_v1/audit_extended.py
The --self-test command uses synthetic CPU fixtures and never opens run data.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import tempfile

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = Path(__file__).resolve().parents[4]
BASE = ROOT / "benchmarks/generic_motion_v1"
LABELS = dict(EI_ZO="LSCS", RRTCONNECT="RRTConnect", RRTSTAR="RRT*",
              CHOMP="CHOMP", STOMP="STOMP", TRAJOPT="TrajOpt")
INDEPENDENT = {"RRTCONNECT", "RRTSTAR"}
SUCCESS = "success_validated_numerical"
TERMINAL = {SUCCESS, "timeout", "invalid_endpoint", "invalid_scene_geometry",
            "model_consistency_failed", "collision_failed", "limit_failed",
            "solver_failed", "frontend_failed", "validation_undetermined",
            "infrastructure_error", "unsupported", "planner_timeout", "planner_failed"}


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def close(actual, expected):
    if actual is None or expected is None:
        return actual is expected
    if isinstance(actual, bool) or isinstance(expected, bool):
        return type(actual) is type(expected) and actual == expected
    try:
        a, b = np.asarray(actual), np.asarray(expected)
        return a.shape == b.shape and bool(np.allclose(a, b, rtol=1e-10, atol=1e-10, equal_nan=False))
    except (TypeError, ValueError):
        return actual == expected


def resample(points, step, cap):
    """Independent reconstruction of the declared PWL sampling protocol."""
    p = np.asarray(points, float)
    if p.ndim != 2 or p.shape[1] != 7 or len(p) < 2 or not np.isfinite(p).all():
        raise ValueError("PWL controls must be finite N x 7 with N >= 2")
    lengths = np.sqrt(np.square(p[1:] - p[:-1]).sum(axis=1))
    counts = np.maximum(2, np.ceil(lengths / step) + 1)
    if not np.isfinite(counts).all() or counts.sum() > cap or np.any(counts > cap):
        raise ValueError("PWL exceeds declared sample cap")
    counts = counts.astype(int)
    q = np.vstack([np.linspace(a, b, n) for a, b, n in zip(p[:-1], p[1:], counts)])
    segments = np.repeat(np.arange(len(p) - 1), counts)
    parameter = np.concatenate([np.linspace(i, i + 1, n) / (len(p) - 1)
                                for i, n in enumerate(counts)])
    return q, parameter, segments, float(lengths.sum())


def derive_validation(points, saved, case, config, robot, region_residual=0.):
    """Recompute validator arithmetic from saved checker outputs, not its code."""
    q, parameter, segments, length = resample(points, config["validation_step_rad"],
                                             config["validation_maximum_samples"])
    p = np.asarray(points, float)
    native = np.asarray(saved["native"])
    independent = np.asarray(saved["independent"])
    clearance = np.asarray(saved["clearance"])
    if (native.dtype != np.bool_ or independent.dtype != np.bool_ or
            native.shape != (len(q),) or independent.shape != (len(q),) or
            clearance.shape != (len(q),) or not np.isfinite(clearance).all()):
        raise ValueError("Malformed saved checker arrays")
    lower, upper = np.asarray(robot["lower"]), np.asarray(robot["upper"])
    endpoint = float(np.linalg.norm(q[[0, -1]] - np.asarray([case["q_start"], case["q_goal"]]), axis=1).max())
    limits = bool(np.all(p >= lower - config["feasibility_tolerance"]) and
                  np.all(p <= upper + config["feasibility_tolerance"]))
    independent_recomputed = clearance > 0
    ambiguous = np.abs(clearance) <= robot.get("contact_band_m", 2e-6)
    collisions = (~native) | (~independent_recomputed)
    max_step = float(np.linalg.norm(q[1:] - q[:-1], axis=1).max())
    status = SUCCESS
    if endpoint > config["endpoint_tolerance_rad"]:
        status = "invalid_endpoint"
    elif not limits or region_residual > config["feasibility_tolerance"]:
        status = "limit_failed"
    elif np.any(collisions & ~ambiguous):
        status = "collision_failed"
    elif np.any(ambiguous) or max_step > config["validation_step_rad"] * (1 + 1e-9) + 1e-12:
        status = "validation_undetermined"
    expected = dict(status=status, passed=status == SUCCESS, collision_free=not bool(collisions.any()),
                    endpoint_error_rad=endpoint, joint_limits_passed=limits, derivative_limits_passed=True,
                    derivative_bounds={}, region_residual=region_residual, samples=len(q),
                    sample_step_bound_rad=config["validation_step_rad"],
                    actual_maximum_sample_joint_step_rad=max_step,
                    native_collisions=int((~native).sum()), independent_collisions=int((~independent_recomputed).sum()),
                    model_disagreements=int((native != independent_recomputed).sum()),
                    contact_band_samples=int(ambiguous.sum()), minimum_sampled_clearance_m=float(clearance.min()),
                    cost=length, path_length_rad=float(np.linalg.norm(q[1:] - q[:-1], axis=1).sum()),
                    normalized_path_length=float(np.linalg.norm((q[1:] - q[:-1]) / (upper - lower), axis=1).sum()),
                    duration_s=None, cost_unit="rad", time_axis="normalized_path_parameter")
    return expected, {"q": q, "time": parameter, "segment": segments,
                      "independent": independent_recomputed,
                      "collision_points": q[collisions], "collision_regions": segments[collisions]}


def arrays_identical(first, second):
    """Exact serialized-array identity, independent of ZIP timestamps."""
    with np.load(first, allow_pickle=False) as a, np.load(second, allow_pickle=False) as b:
        return set(a.files) == set(b.files) and all(
            a[key].dtype == b[key].dtype and np.array_equal(a[key], b[key]) for key in a.files)


def bound_region_file(attempt, points, validation_path, samples_path):
    """Bind a completed EI-ZO validation to its own optimization corridor.

    EI-ZO writes the root regions again after recovery. A later solve may then
    fail, leaving root candidate/validation from the previous complete round.
    Use the last fully matching round, never those subsequently edited regions.
    """
    rounds = sorted((attempt / "rounds").glob("[0-9][0-9][0-9]"), reverse=True)
    if not rounds:
        root_file = attempt / "regions.npz"
        return root_file if root_file.exists() else None
    validation = read(validation_path)
    for directory in rounds:
        needed = [directory / name for name in
                  ("candidate.npz", "validation.json", "trajectory_samples.npz", "regions.npz")]
        if not all(path.is_file() for path in needed):
            continue
        with np.load(directory / "candidate.npz", allow_pickle=False) as candidate:
            same_points = "points" in candidate and np.array_equal(candidate["points"], points)
        if (same_points and read(directory / "validation.json") == validation and
                arrays_identical(directory / "trajectory_samples.npz", samples_path)):
            return directory / "regions.npz"
    raise ValueError("No completed optimization round binds the root candidate, validation and sample arrays")


class Audit:
    def __init__(self):
        self.errors = []
        self.checks = defaultdict(lambda: dict(checks=0, passed=0, failed=0))
        self.counts = Counter()
        self.hash_cache = {}
        self.observations = []
        self.by_method = {label: dict(registered=0, legal=0, invalid=0, statuses=Counter(), successes=0, candidates=0)
                          for label in LABELS.values()}

    def check(self, condition, category, scope, detail):
        condition = bool(condition)
        item = self.checks[category]
        item["checks"] += 1
        item["passed" if condition else "failed"] += 1
        if not condition:
            self.errors.append(dict(category=category, scope=str(scope), detail=detail))
        return condition

    def digest(self, path):
        path = Path(path).resolve()
        if path not in self.hash_cache:
            self.hash_cache[path] = sha(path)
        return self.hash_cache[path]

    def file_hash(self, path, expected, category):
        path = Path(path)
        return self.check(path.is_file() and self.digest(path) == expected, category, path,
                          "File is absent or SHA256 differs from frozen evidence")

    def fields(self, actual, expected, category, scope):
        for key, value in expected.items():
            self.check(key in actual and close(actual.get(key), value), category, scope,
                       f"Field {key}: recorded {actual.get(key)!r}, expected {value!r}")

    def numeric(self, attempt, points, case, cfg, robot, *, frontend=False):
        directory = attempt / "frontend_validation" if frontend else attempt
        label = "frontend_numeric" if frontend else "candidate_numeric"
        observation = dict(attempt=str(attempt), frontend=frontend)
        # A failed solver is permitted to leave a malformed candidate. It must
        # never be promoted to success, and its failure evidence is retained.
        try:
            p = np.asarray(points, float)
            observation.update(points=int(len(p)), finite=bool(np.isfinite(p).all()))
            if p.ndim != 2 or p.shape[1] != 7 or len(p) < 2 or not np.isfinite(p).all():
                raise ValueError("Nonfinite or malformed saved PWL candidate")
            observation.update(path_length_rad=float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()),
                endpoint_error_rad=float(np.linalg.norm(p[[0, -1]] - np.asarray([case["q_start"], case["q_goal"]]), axis=1).max()),
                joint_limits_passed=bool(np.all(p >= np.asarray(robot["lower"]) - cfg["feasibility_tolerance"]) and
                                         np.all(p <= np.asarray(robot["upper"]) + cfg["feasibility_tolerance"])))
            validation_path, samples_path = directory / "validation.json", directory / "trajectory_samples.npz"
            self.check(validation_path.exists() == samples_path.exists(), label, directory,
                       "A completed numerical validation must have report and recorded samples")
            if not validation_path.exists() or not samples_path.exists():
                observation["validation"] = "not_completed"
                return observation
            residual = 0.
            regions_path = None if frontend else bound_region_file(attempt, p, validation_path, samples_path)
            if regions_path is not None:
                observation["bound_regions_file"] = str(regions_path)
                with np.load(regions_path, allow_pickle=False) as regions:
                    akeys = sorted(k for k in regions.files if k.startswith("A_"))
                    self.check(len(akeys) == len(p) - 1, label, regions_path, "Region / PWL segment count mismatch")
                    if len(akeys) == len(p) - 1:
                        residual = max(float(np.max(regions[key] @ p[i:i+2].T - regions["b_" + key[2:]][:, None]))
                                       for i, key in enumerate(akeys))
            with np.load(samples_path, allow_pickle=False) as samples:
                expected, identities = derive_validation(p, samples, case, cfg, robot, residual)
                for key in ("q", "time", "segment", "independent"):
                    self.check(key in samples and close(samples[key], identities[key]), label, samples_path,
                               f"Saved {key} differs from independently reconstructed PWL sample evidence")
            validation = read(validation_path)
            self.fields(validation, expected, label, validation_path)
            for key in ("collision_points", "collision_regions"):
                actual = np.asarray(validation.get(key, []))
                exp = identities[key]
                self.check((actual.size == exp.size == 0) or close(actual, exp), label, validation_path,
                           f"Saved {key} does not match the saved checker outputs")
            observation.update(validation=expected["status"], samples=expected["samples"])
            self.counts["frontend_validations" if frontend else "candidate_validations"] += 1
            if not frontend:
                result = read(attempt / "result.json")
                for key in ("cost", "path_length_rad", "normalized_path_length", "duration_s", "cost_unit",
                            "minimum_sampled_clearance_m"):
                    if key in result.get("metrics", {}):
                        self.fields(result["metrics"], {key: expected[key]}, "result_numeric", attempt)
            return observation
        except Exception as exc:
            observation["exception"] = f"{type(exc).__name__}: {exc}"
            result = read(attempt / "result.json")
            self.check(result.get("status") != SUCCESS and not (directory / "validation.json").exists(),
                       label, directory, "Numeric reconstruction failed: " + observation["exception"])
            return observation
        finally:
            self.observations.append(observation)


def preflight(index, metrics_path):
    """Do not publish a final audit of a study still in progress."""
    missing = []
    for block in index["blocks"]:
        run = Path(block["run"])
        for job in read(run / "run_manifest.json")["jobs"]:
            attempts = sorted((run / "jobs" / job["job_id"] / "attempts").glob("[0-9][0-9][0-9][0-9]"))
            if not attempts or any(not (a / "receipt.json").is_file() for a in attempts):
                missing.append(job["job_id"])
    if missing:
        raise RuntimeError(f"Study is not sealed: {len(missing)} slots are incomplete; no final audit written")
    if not metrics_path.is_file() or not read(metrics_path).get("complete"):
        raise RuntimeError("Completed metrics are absent; no final audit written")
    completed_path = OUT / "completed_blocks.json"
    if not completed_path.exists() or not read(completed_path).get("complete"):
        raise RuntimeError("Coordinator has not sealed all study blocks; no final audit written")


def runtime_paths():
    files = [BASE / name for name in ("common.py", "batch.py", "model.py", "roadmap.py", "study_protocol.py", "__main__.py")]
    for folder in ("generator", "runners", "configs"):
        files += list((BASE / folder).glob("*.json" if folder == "configs" else "*.py"))
    files += [p for p in (BASE / "native").rglob("*") if p.is_file() and
              (p.suffix in {".cpp", ".c", ".h", ".hpp", ".json"} or p.name.startswith(("LICENSE", "COPYING")))]
    files += [OUT / "trajopt_author_core.tar.gz", ROOT / "reproduction/paper_completion/optimization.py",
              ROOT / "reproduction/alignment_diagnosis/geometric_solvers.py"]
    return {str(p.relative_to(ROOT)) for p in files if p.is_file()}


def audit_attempt(audit, attempt, job, freeze, robot):
    scope = str(attempt)
    receipt, result, process = (read(attempt / name) for name in ("receipt.json", "result.json", "process.json"))
    status = result.get("status")
    audit.counts["receipts"] += 1
    actual_files = {str(p.relative_to(attempt)) for p in attempt.rglob("*")
                    if p.is_file() and p.name != "receipt.json"}
    audit.check(set(receipt["files"]) == actual_files, "receipts", scope, "Receipt does not bind every raw artifact")
    for name, digest in receipt["files"].items():
        path = (attempt / name).resolve()
        if audit.check(path.is_relative_to(attempt.resolve()) and name != "receipt.json", "receipts", scope,
                       "Invalid path in receipt: " + name):
            audit.file_hash(path, digest, "raw_hashes")
            if path.is_file():
                audit.counts["raw_files"] += 1
                audit.counts["raw_bytes"] += path.stat().st_size
    audit.check({"stdout.log", "stderr.log", "process.json", "request.json", "result.json", "runtime_sources.json"}
                <= set(receipt["files"]), "receipts", scope, "Mandatory logs or process/source evidence are absent")
    audit.check(status in TERMINAL and receipt.get("status") == status and not receipt.get("retry_pending"),
                "classification", scope, "Unsealed, inconsistent or nonterminal result")
    audit.check(read(attempt / "request.json") == job, "input_identity", scope, "Worker request differs from frozen job")
    audit.fields(result, {"problem_id": job["case_id"], "scene_id": job["scene_id"], "family": job["family"],
        "method": job["method"], "objective": job["objective"], "method_seed": job["method_seed"],
        "roadmap_id": job["roadmap_id"], "source_code_commit": freeze["source_commit"]}, "result_identity", scope)
    runtime = read(attempt / "runtime_sources.json")
    audit.check(runtime.get("before") == runtime.get("after") == freeze["runtime_source_hashes"]
                and runtime.get("unchanged") is True, "runtime_identity", scope, "Source changed before/during worker")
    audit.fields(process, {"timeout_seconds": 60.}, "timing", scope)
    audit.fields(result, {"cold_e2e_seconds": process.get("cold_e2e_seconds")}, "timing", scope)
    audit.check(isinstance(process.get("cold_e2e_seconds"), (float, int)) and
                np.isfinite(process["cold_e2e_seconds"]) and process["cold_e2e_seconds"] >= 0,
                "timing", scope, "Cold process latency is missing or invalid")
    if process.get("timed_out"):
        audit.check(status == "timeout", "classification", scope, "External watchdog timeout claimed another status")
    if process.get("exit_code") != 0:
        audit.check(status in {"timeout", "infrastructure_error"}, "classification", scope, "Nonzero worker exit claimed a normal result")
    for name, expected in (("evaluation.json", freeze["evaluation_config"]),
                           ("input.json", {**job["case"], "_objective": job["objective"]})):
        if (attempt / name).exists():
            audit.check(read(attempt / name) == expected, "input_identity", scope, name + " differs from frozen input")
        else:
            audit.check(status in {"timeout", "infrastructure_error", "unsupported"}, "input_identity", scope, name + " absent")
    independent = job["method"] in INDEPENDENT
    audit.check(result.get("roadmap_used") is (not independent), "frontend_pairing", scope, "Wrong roadmap usage declaration")
    if independent:
        audit.check(not (attempt / "path.npy").exists() and all(result.get(k) is None for k in
                    ("frontend_success", "frontend_native_success", "drm_success", "offline_roadmap_seconds")),
                    "frontend_pairing", scope, "Independent OMPL improperly carries DRM evidence")
    candidate_path, arrays_path = attempt / "candidate.json", attempt / "candidate.npz"
    candidate = read(candidate_path) if candidate_path.exists() else {}
    validation = read(attempt / "validation.json") if (attempt / "validation.json").exists() else {}
    audit.check(candidate_path.exists() == arrays_path.exists(), "candidate_evidence", scope, "Candidate metadata/array pair incomplete")
    if candidate_path.exists():
        audit.counts["candidates"] += 1
        audit.fields(candidate, {"kind": "polygon", "objective": "minimum_distance"}, "candidate_evidence", scope)
        with np.load(arrays_path, allow_pickle=False) as arrays:
            audit.check("points" in arrays, "candidate_evidence", scope, "Missing PWL points")
            if "points" in arrays:
                audit.numeric(attempt, arrays["points"], job["case"], freeze["evaluation_config"], robot)
    if (attempt / "frontend_validation/validation.json").exists():
        audit.check((attempt / "path.npy").exists(), "frontend_evidence", scope, "Validated frontend has no raw path")
        if (attempt / "path.npy").exists():
            audit.numeric(attempt, np.load(attempt / "path.npy", allow_pickle=False), job["case"],
                          freeze["evaluation_config"], robot, frontend=True)
    if status == SUCCESS:
        audit.counts["validated_successes"] += 1
        audit.check(candidate.get("solver_success") is True and validation.get("passed") is True
                    and validation.get("status") == SUCCESS and not process.get("timed_out")
                    and process.get("exit_code") == 0 and job["case"]["validation_status"] == "valid_input",
                    "success_truth", scope, "Success lacks solver, legal-input, process or independent validation evidence")
    if job["case"]["validation_status"] != "valid_input":
        audit.check(status == "invalid_endpoint" and not candidate and not validation,
                    "invalid_inputs", scope, "Frozen generation failure was dropped, repaired or planned")
    return dict(job=job, result=result, process=process, candidate=candidate, validation=validation,
                attempt=str(attempt), path_sha256=audit.digest(attempt / "path.npy") if (attempt / "path.npy").exists() else None)


def selected_scope(row, scope):
    if scope == "all":
        return True
    if scope == "obstacle_main":
        return row["family"] != "open_control"
    if scope == "open_control":
        return row["family"] == "open_control"
    kind, value = scope.split(":", 1)
    return row["family"] != "open_control" and str(row[{"family": "family", "straight_line": "straight_line_collision",
                                                       "distance_bin": "distance_bin"}[kind]]) == value


def audit_metrics(audit, metrics, evidence, freeze):
    audit.fields(metrics, {"complete": True, "comparable": True, "requested_jobs": 1140,
                          "expected_method_labels": list(LABELS.values()), "pairing_differences": []}, "metrics", "metrics.json")
    rows = metrics["rows"]
    keys = [(str(Path(r["run"]).resolve()), r["job_id"]) for r in rows]
    audit.check(len(keys) == len(set(keys)) == 1140 and set(keys) == set(evidence), "metrics", "rows",
                "Metric rows omit, duplicate or add registered success/failure slots")
    for row, key in zip(rows, keys):
        if key not in evidence:
            continue
        e = evidence[key]
        j, result, candidate, validation, process = (e[k] for k in ("job", "result", "candidate", "validation", "process"))
        c, legal, independent = j["case"], j["case"]["validation_status"] == "valid_input", j["method"] in INDEPENDENT
        cluster = "open_control_shared_base_world" if c["family"] == "open_control" else c.get("scene_cluster_id", c.get("cluster_id", c["scene_id"]))
        expected = dict(case_id=j["case_id"], scene_id=j["scene_id"], cluster_id=cluster, method=j["method"],
            label=LABELS[j["method"]], family=j["family"], objective=j["objective"], input_legal=legal,
            status=result["status"], success=result["status"] == SUCCESS, q_start=c["q_start"], q_goal=c["q_goal"],
            robot_sha256=c["robot_sha256"], geometry_sha256=c["collision_geometry_sha256"], input_sha256=c["problem_sha256"],
            split=c["split"], method_seed=j["method_seed"], roadmap_seed=j["replica_seed"], roadmap_id=j["roadmap_id"],
            budget_seconds=60., path_sha256=e["path_sha256"], final_validation_passed=validation.get("passed"),
            solver_reported_success=candidate.get("solver_success"), solver_converged=candidate.get("solver_converged"),
            attempt=e["attempt"], cold_e2e_seconds=process["cold_e2e_seconds"],
            cost=validation.get("cost") if result["status"] == SUCCESS else None,
            roadmap_used=not independent, drm_conditional_metrics_applicable=not independent,
            drm_success=None if independent else result.get("frontend_success"),
            native_drm_success=None if independent else result.get("frontend_native_success"))
        audit.fields(row, expected, "metric_rows", key)
        if validation:
            audit.fields(row, {k: validation.get(k) for k in ("normalized_path_length", "minimum_sampled_clearance_m", "cost_unit")},
                         "metric_rows", key)
        if "native_objective" in candidate:
            audit.fields(row, {"native_objective": candidate["native_objective"]}, "metric_rows", key)
        if result["status"] == SUCCESS:
            audit.check(row["cost"] is not None and row["cost"] >= c["start_goal_joint_distance"] - 2e-5,
                        "metric_rows", key, "Successful PWL cost falls below its endpoint distance")
    group_keys = []
    for group in metrics["groups"]:
        group_key = (group["split"], group["roadmap_nodes"], group["scope"], group["label"])
        group_keys.append(group_key)
        pool = [r for r in rows if r["split"] == group["split"] and r["roadmap_nodes"] == group["roadmap_nodes"]
                and r["label"] == group["label"] and selected_scope(r, group["scope"])]
        legal = [r for r in pool if r["input_legal"]]
        success = [r for r in legal if r["success"]]
        audit.fields(group, dict(requested_jobs=len(pool), legal_jobs=len(legal), invalid_jobs=len(pool)-len(legal),
            status_counts=dict(Counter(r["status"] for r in pool)), legal_status_counts=dict(Counter(r["status"] for r in legal)),
            legal_scene_clusters=len({r["cluster_id"] for r in legal})), "metric_groups", group_key)
        audit.fields(group["valid_success"], dict(numerator=len(success), denominator=len(legal),
            estimate=len(success)/len(legal) if legal else None), "metric_groups", group_key)
        clustered = defaultdict(lambda: [0, 0])
        for row in legal:
            clustered[row["cluster_id"]][0] += int(row["success"])
            clustered[row["cluster_id"]][1] += 1
        units = np.asarray([clustered[k] for k in sorted(clustered)], float).reshape(-1, 2)
        interval = None
        if len(units) >= 2:
            rng = np.random.Generator(np.random.PCG64(45827))
            totals = units[rng.integers(len(units), size=(2000, len(units)))].sum(axis=1)
            interval = np.quantile(totals[:, 0] / totals[:, 1], [.025, .975]).tolist()
        audit.fields(group["valid_success"], dict(independent_scene_clusters=len(units), bootstrap_seed=45827,
            bootstrap_replicates=2000, ci95=interval), "metric_cluster_intervals", group_key)
        for field, stats in group["statistics"].items():
            for subset, sample in (("all_legal", legal), ("validated_success", success),
                                   ("failed_legal", [r for r in legal if not r["success"]])):
                values = np.asarray([r[field] for r in sample if isinstance(r.get(field), (int, float))
                                     and not isinstance(r[field], bool) and np.isfinite(r[field])], float)
                expected = dict(count=len(values))
                expected.update({name: float(fn(values)) if len(values) else None for name, fn in
                    (("mean", np.mean), ("p50", lambda x: np.quantile(x, .5)), ("p90", lambda x: np.quantile(x, .9)),
                     ("p95", lambda x: np.quantile(x, .95)), ("min", np.min), ("max", np.max))})
                audit.fields(stats[subset], expected, "metric_statistics", (*group_key, field, subset))
    audit.check(len(group_keys) == len(set(group_keys)), "metric_groups", "groups", "Repeated metric stratum")
    for split in ("smoke", "dev"):
        for label in LABELS.values():
            audit.check((split, 6000, "all", label) in set(group_keys), "metric_groups", "groups", "Missing unconditional method denominator")


def run_audit():
    freeze_path, index_path, metrics_path = OUT / "execution_freeze.json", OUT / "run_index.json", OUT / "metrics/metrics.json"
    freeze, index = read(freeze_path), read(index_path)
    preflight(index, metrics_path)
    audit = Audit()
    audit.fields(freeze, dict(planned_registered_slots=1140, test_planner_calls=0, timeout_seconds=60., gpu_workers=1,
                             methods=list(LABELS), expected_labels=list(LABELS.values())), "freeze", freeze_path)
    audit.fields(index, dict(slots=1140, test_planner_calls=0), "freeze", index_path)
    completed = read(OUT / "completed_blocks.json")
    audit.fields(completed, dict(complete=True, completed_jobs=1140, total_jobs=1140),
                 "completion", "completed_blocks.json")
    audit.check({b["name"] for b in completed["blocks"]} == {b["name"] for b in index["blocks"]}
                and len(completed["blocks"]) == 4, "completion", "completed_blocks.json", "Completed block matrix differs from index")
    for block in completed["blocks"]:
        audit.fields(block["verification"], dict(integrity_passed=True, complete=True),
                     "completion", block["name"])
    audit.check(runtime_paths() == set(freeze["runtime_source_hashes"]), "source_inventory", "current runtime",
                "Runtime file inventory differs from freeze (including added files)")
    for name, digest in freeze["runtime_source_hashes"].items():
        audit.file_hash(ROOT / name, digest, "runtime_hashes")
    for name, digest, category in (("development_protocol.json", freeze["declaration_sha256"], "protocol"),
                                    ("run_extended_study.py", freeze["execution_script_sha256"], "protocol")):
        audit.file_hash(OUT / name, digest, category)
    audit.file_hash(BASE / "configs/extended_baselines.json", freeze["config_file_sha256"], "protocol")
    audit.check(read(OUT / "development_protocol.json") == freeze["declaration"], "protocol", "declaration", "Embedded declaration mismatch")
    for category in ("data_files", "roadmap_files", "external_dependency_files"):
        for path, digest in freeze[category].items():
            audit.file_hash(path, digest, category)
    audit.counts["dependency_files"] = len(freeze["external_dependency_files"])
    audit.counts["frozen_data_files"] = len(freeze["data_files"])
    audit.counts["frozen_roadmap_files"] = len(freeze["roadmap_files"])
    robot = read(BASE / "configs/robot.json")
    for path, digest in robot["source_files_sha256"].items():
        audit.file_hash(ROOT / path, digest, "model_assets")
    audit.counts["model_asset_files"] = len(robot["source_files_sha256"])
    blocks, evidence, pairing = [], {}, defaultdict(list)
    frozen_blocks = {b["name"]: b for b in freeze["blocks"]}
    audit.check(len(index["blocks"]) == 4 and {b["name"] for b in index["blocks"]} == set(frozen_blocks), "matrix", "blocks", "Block matrix changed")
    for block in index["blocks"]:
        run = Path(block["run"]).resolve()
        manifest = read(run / "run_manifest.json")
        audit.check({k: v for k, v in block.items() if k != "run_manifest_sha256"} == frozen_blocks[block["name"]],
                    "matrix", run, "Index block differs from execution freeze")
        audit.file_hash(run / "run_manifest.json", block["run_manifest_sha256"], "run_identity")
        audit.fields(read(run / "identity.json"), {"run_manifest_sha256": block["run_manifest_sha256"]}, "run_identity", run)
        audit.fields(manifest, dict(source_code_commit=freeze["source_commit"], source_hashes=freeze["runtime_source_hashes"],
            timeout_seconds=60., split=block["split"], formal_test_run=False, selection="all",
            manifest_sha256=audit.digest(block["manifest"]), omitted_unsupported_combinations=[]), "run_identity", run)
        audit.check(Path(manifest["manifest_path"]).resolve() == Path(block["manifest"]).resolve(),
                    "run_identity", run, "Frozen manifest path differs")
        snapshot_files = {str(p.relative_to(run / "source_snapshot")) for p in (run / "source_snapshot").rglob("*") if p.is_file()}
        audit.check(snapshot_files == set(manifest["source_hashes"]), "source_inventory", run,
                    "Source snapshot file inventory differs from bound source hashes")
        for path, digest in manifest["roadmap_files"].items():
            audit.check(freeze["roadmap_files"].get(str(Path(path).resolve())) == digest, "roadmap_identity", run,
                        "Manifest roadmap hash differs from execution freeze: " + path)
        for path, digest in manifest["source_hashes"].items():
            audit.file_hash(run / "source_snapshot" / path, digest, "snapshot_hashes")
            audit.counts["source_snapshot_files"] += 1
        data = [json.loads(line) for line in Path(block["manifest"]).read_text().splitlines() if line.strip()]
        cases = {case["problem_id"]: case for case in data}
        metadata = read(Path(block["roadmap"]) / "metadata.json")
        audit.fields(metadata, {"robot_sha256": robot["robot_sha256"],
                                "collision_geometry_sha256": robot["collision_geometry_sha256"],
                                "requested_nodes": 6000}, "roadmap_identity", block["roadmap"])
        audit.check(len(cases) == block["registered_inputs"] and sum(c["validation_status"] == "valid_input" for c in data) == block["legal_inputs"],
                    "matrix", run, "Registered/legal input counts changed")
        expected = {(c["problem_id"], method, "minimum_distance") for c in data for method in LABELS}
        observed = [(j["case_id"], j["method"], j["objective"]) for j in manifest["jobs"]]
        audit.check(len(observed) == len(set(observed)) == block["planned_jobs"] and set(observed) == expected,
                    "matrix", run, "Must contain all six jobs per frozen case exactly once")
        audit.check({p.name for p in (run / "jobs").iterdir() if p.is_dir()} == {j["job_id"] for j in manifest["jobs"]},
                    "matrix", run, "Extra or missing job directories")
        block_status = Counter()
        for job in manifest["jobs"]:
            c = cases[job["case_id"]]
            seed = next(p for p in c["paired_seeds"] if p["drm_seed"] == metadata["seed"])
            audit.check(job["case"] == c and job["evaluation_config"] == freeze["evaluation_config"],
                        "input_identity", job["job_id"], "Case/config changed or endpoints resampled")
            audit.fields(job, dict(scene_id=c["scene_id"], family=c["family"], roadmap_id=metadata["roadmap_id"],
                method_seed=seed["method_seed"], solver_seed=seed["solver_seed"], solver_seed_used=False,
                replica_seed=metadata["seed"], roadmap_used=job["method"] not in INDEPENDENT), "pairing_seeds", job["job_id"])
            audit.check(Path(job["roadmap_dir"]).resolve() == Path(block["roadmap"]).resolve(), "pairing_seeds", job["job_id"], "Roadmap path differs")
            audit.fields(c, {"robot_sha256": robot["robot_sha256"], "collision_geometry_sha256": robot["collision_geometry_sha256"]},
                         "model_identity", job["job_id"])
            attempts = sorted((run / "jobs" / job["job_id"] / "attempts").glob("[0-9][0-9][0-9][0-9]"))
            audit.check(len(attempts) == 1 and attempts[0].name == "0001", "attempts", job["job_id"], "Unexpected retries: no best-attempt selection permitted")
            for attempt in attempts:
                try:
                    e = audit_attempt(audit, attempt, job, freeze, robot)
                    evidence[(str(run), job["job_id"])] = e
                    pairing[(c["problem_id"], metadata["seed"])].append(e)
                    status = e["result"]["status"]
                    block_status[status] += 1
                    legal = c["validation_status"] == "valid_input"
                    method = audit.by_method[LABELS[job["method"]]]
                    method["registered"] += 1
                    method["legal" if legal else "invalid"] += 1
                    method["statuses"][status] += 1
                    method["successes"] += status == SUCCESS
                    method["candidates"] += bool(e["candidate"])
                    audit.counts["registered_slots"] += 1
                    audit.counts["legal_slots" if legal else "invalid_slots"] += 1
                except Exception as exc:
                    audit.check(False, "attempt_exception", attempt, f"{type(exc).__name__}: {exc}")
        blocks.append(dict(name=block["name"], slots=len(manifest["jobs"]), receipts=sum(block_status.values()),
                           statuses=dict(block_status), run_manifest_sha256=audit.digest(run / "run_manifest.json")))
    for key, group in pairing.items():
        shared = [e for e in group if e["job"]["method"] not in INDEPENDENT]
        signatures = {(e["path_sha256"], e["result"].get("frontend_success"), e["result"].get("frontend_native_success")) for e in shared}
        audit.check(len(group) == 6 and len(shared) == 4 and len(signatures) == 1, "frontend_pairing", key,
                    "Four local methods do not share exactly the same DRM path/outcome")
        audit.check(all(e["path_sha256"] is not None for e in shared if e["result"].get("frontend_success") is True),
                    "frontend_pairing", key, "Frontend success lacks a saved path")
    metrics = read(metrics_path)
    metric_run_ids = {str(Path(item["run"]).resolve()): item for item in metrics["runs"]}
    audit.check(len(metrics["runs"]) == len(metric_run_ids) == 4 and set(metric_run_ids) ==
                {str(Path(b["run"]).resolve()) for b in index["blocks"]}, "metrics", "runs",
                "Metric run inventory differs from frozen four-block matrix")
    for block in index["blocks"]:
        run_key = str(Path(block["run"]).resolve())
        if run_key in metric_run_ids:
            audit.fields(metric_run_ids[run_key], dict(manifest_sha256=block["run_manifest_sha256"],
                source_commit=freeze["source_commit"]), "metrics", run_key)
            audit.fields(metric_run_ids[run_key]["integrity"], dict(integrity_passed=True, complete=True),
                         "metrics", run_key)
    audit_metrics(audit, metrics, evidence, freeze)
    # Protect the frozen test partition and inspect saved execution evidence.
    # This detects newly saved test calls, not unlogged external processes.
    frozen_time = datetime.fromisoformat(freeze["frozen_utc"]).timestamp()
    newer_test = []
    test_runs = 0
    for path in (BASE / "results").glob("*/run_manifest.json"):
        m = read(path)
        if m.get("split") != "test":
            continue
        test_runs += 1
        created = m.get("created_utc")
        if created and datetime.fromisoformat(created).timestamp() >= frozen_time:
            newer_test.append(str(path))
        newer_test += [str(p) for p in path.parent.glob("jobs/*/attempts/*/process.json") if p.stat().st_mtime >= frozen_time]
    audit.check(not newer_test, "test_protection", "saved test executions", "New test execution evidence after freeze: " + repr(newer_test))
    audit.counts["preexisting_test_runs"] = test_runs
    audit.counts["new_test_execution_records"] = len(newer_test)
    audit.fields(dict(audit.counts), dict(registered_slots=1140, legal_slots=1128, invalid_slots=12, receipts=1140), "totals", "study")
    audit.counts["checks"] = sum(c["checks"] for c in audit.checks.values())
    return dict(schema_version=1, passed=not audit.errors, audited_utc=datetime.now(timezone.utc).isoformat(),
        source_commit=freeze["source_commit"], audit_script_sha256=sha(__file__),
        metrics_path=str(metrics_path), metrics_sha256=sha(metrics_path), execution_freeze_sha256=sha(freeze_path),
        run_index_sha256=sha(index_path), checks=dict(audit.checks), check_count=audit.counts["checks"], errors=audit.errors,
        counts=dict(audit.counts), by_method=audit.by_method, blocks=blocks, numeric_observations=audit.observations,
        scope="All six registered methods on smoke and dev; four local methods share DRM, independent OMPL does not.",
        limitations=["Collision checks are audited from saved native boolean and signed-distance arrays; no fresh geometry or GPU calls are made.",
                     "Finite numerical collision sampling is not a proof of continuous collision freedom.",
                     "No-new-test-call check binds frozen test input hashes and saved process evidence; it cannot detect unlogged external executions.",
                     "Native solver convergence is separate from valid returned trajectory success; a validated nonconverged incumbent may qualify."])


def self_test():
    cfg = dict(validation_step_rad=.05, validation_maximum_samples=100000,
               endpoint_tolerance_rad=1e-5, feasibility_tolerance=1e-5)
    robot = dict(lower=[-3.]*7, upper=[3.]*7, contact_band_m=2e-6)
    p = np.zeros((3, 7)); p[1, 0] = .2; p[2, 0] = .5
    case = dict(q_start=p[0].tolist(), q_goal=p[-1].tolist())
    q, t, segment, length = resample(p, .05, 100000)
    saved = dict(native=np.ones(len(q), bool), independent=np.ones(len(q), bool), clearance=np.full(len(q), .1))
    expected, arrays = derive_validation(p, saved, case, cfg, robot)
    assert close(length, .5) and expected["passed"] is True and close(expected["normalized_path_length"], 1/12)
    assert close(arrays["q"], q) and close(arrays["time"], t) and close(arrays["segment"], segment)
    saved["clearance"][3] = -.01
    assert derive_validation(p, saved, case, cfg, robot)[0]["status"] == "collision_failed"
    saved["clearance"][3] = 0.
    assert derive_validation(p, saved, case, cfg, robot)[0]["status"] == "validation_undetermined"
    shifted = {**case, "q_goal": [.6, 0, 0, 0, 0, 0, 0]}
    assert derive_validation(p, saved, shifted, cfg, robot)[0]["status"] == "invalid_endpoint"
    assert derive_validation(p, saved, case, cfg, robot, .1)[0]["status"] == "limit_failed"
    try:
        resample(p, .05, 2)
    except ValueError:
        pass
    else:
        raise AssertionError("Sampling cap must reject")
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "fixture.txt"; path.write_text("fixed")
        audit = Audit(); assert audit.file_hash(path, sha(path), "fixture")
        assert not audit.file_hash(path, "0" * 64, "fixture")
        assert len(audit.errors) == 1
        attempt = Path(directory) / "attempt"; attempt.mkdir()
        current_round = attempt / "rounds/000"; current_round.mkdir(parents=True)
        later_round = attempt / "rounds/001"; later_round.mkdir()
        for folder in (attempt, current_round):
            np.savez(folder / "candidate.npz", points=p)
            np.savez(folder / "trajectory_samples.npz", q=q, time=t, segment=segment, **saved)
            (folder / "validation.json").write_text(json.dumps(expected))
        # Recovery changes the top-level corridor before a failed next solve.
        np.savez(current_round / "regions.npz", A_0000=np.ones((1, 7)), b_0000=np.ones(1))
        np.savez(attempt / "regions.npz", A_0000=np.ones((1, 7)), b_0000=-np.ones(1))
        np.savez(later_round / "regions.npz", A_0000=np.ones((1, 7)), b_0000=-np.ones(1))
        assert bound_region_file(attempt, p, attempt / "validation.json", attempt / "trajectory_samples.npz") == current_round / "regions.npz"
        # A mismatched root candidate must not borrow any round's corridor.
        try:
            bound_region_file(attempt, p + .01, attempt / "validation.json", attempt / "trajectory_samples.npz")
        except ValueError:
            pass
        else:
            raise AssertionError("Unbound candidate must reject round lookup")
    assert close(None, None) and not close(None, 0) and not close(True, 1) and not close(float("nan"), float("nan"))
    print(json.dumps(dict(passed=True, self_test="PWL length/sample/endpoint/limits/contact/collision/cap/hash/type and matching-round fixtures", gpu_calls=0)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--output-prefix", default="final_audit", help="New report filename prefix; existing reports are never overwritten")
    args = parser.parse_args()
    if args.self_test:
        self_test(); return 0
    if Path(args.output_prefix).name != args.output_prefix:
        parser.error("--output-prefix must be a filename")
    json_path, md_path = OUT / (args.output_prefix + ".json"), OUT / (args.output_prefix + ".md")
    if json_path.exists() or md_path.exists():
        parser.error("Report already exists; choose a new --output-prefix to preserve earlier evidence")
    try:
        result = run_audit()
    except Exception as exc:
        print(f"Audit refused: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    json_path.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    lines = ["# Extended baseline independent final audit", "", f"Passed: **{result['passed']}**", "",
             f"Source commit: `{result['source_commit']}`", "",
             f"Checks: {result['check_count']}; errors: {len(result['errors'])}.", "",
             "| Quantity | Count |", "|---|---:|"]
    lines += [f"| {key} | {value} |" for key, value in sorted(result["counts"].items())]
    lines += ["", "| Method | Registered | Legal | Invalid | Success | Candidate | Statuses |",
              "|---|---:|---:|---:|---:|---:|---|"]
    for name, v in result["by_method"].items():
        lines.append(f"| {name} | {v['registered']} | {v['legal']} | {v['invalid']} | {v['successes']} | {v['candidates']} | {dict(v['statuses'])} |")
    lines += ["", "| Check category | Checks | Failed |", "|---|---:|---:|"]
    lines += [f"| {key} | {value['checks']} | {value['failed']} |" for key, value in sorted(result["checks"].items())]
    lines += ["", "## Evidence bindings", "", f"Metrics SHA256: `{result['metrics_sha256']}`",
              f"Execution freeze SHA256: `{result['execution_freeze_sha256']}`", f"Run index SHA256: `{result['run_index_sha256']}`",
              "", "## Errors", ""]
    lines += [f"- {e['category']}: `{e['scope']}` — {e['detail']}" for e in result["errors"]] or ["None."]
    lines += ["", "## Audit limits", ""] + ["- " + item for item in result["limitations"]]
    md_path.write_text("\n".join(lines) + "\n")
    print(json.dumps({k: result[k] for k in ("passed", "check_count", "counts", "metrics_sha256")}, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
