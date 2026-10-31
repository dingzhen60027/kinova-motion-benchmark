"""Independent sealed-run CPU audit using SciPy; no Drake/runtime/GPU imports.

Use --run PATH --output FILE.json, with output outside the sealed run.
Collision statistics audit saved checker arrays, not a new collision experiment.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
import scipy
from scipy.interpolate import BSpline

SUCCESS = "success_validated_numerical"
INTERRUPTED = ("timeout", "infrastructure_error")


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def npz(path):
    with np.load(path, allow_pickle=False) as values:
        return {key: values[key].copy() for key in values.files}


def clean(value):
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, np.generic):
        return clean(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def audit(run):
    run = Path(run).resolve()
    manifest = read(run / "run_manifest.json")
    issues, warnings, records, source_checks = [], [], [], []
    receipts, sealed_files = 0, 0
    statuses, solver_counts = Counter(), Counter()
    path_groups = defaultdict(list)

    def check(condition, location, message):
        if not bool(condition):
            issues.append(dict(location=location, message=message))

    def close(actual, expected, location, message, atol=1e-9, rtol=1e-9):
        a, b = np.asarray(actual), np.asarray(expected)
        check(a.shape == b.shape and np.allclose(a, b, atol=atol, rtol=rtol), location, message)

    for relative, expected in manifest["source_hashes"].items():
        path = run / "source_snapshot" / relative
        actual = sha(path) if path.is_file() else None
        source_checks.append(dict(path=relative, sha256=actual, expected_sha256=expected))
        check(actual == expected, relative, "Frozen source snapshot hash differs")

    for job in manifest["jobs"]:
        jid = job["job_id"]
        attempts = sorted((run / "jobs" / jid / "attempts").glob("[0-9][0-9][0-9][0-9]"))
        if not attempts or not (attempts[-1] / "receipt.json").is_file():
            check(False, jid, "Unfinished latest attempt: no sealed receipt")
            continue
        attempt = attempts[-1]
        receipt = read(attempt / "receipt.json")
        check(not receipt.get("retry_pending"), jid, "Latest attempt has a pending retry")
        receipts += 1
        for relative, expected in receipt["files"].items():
            path = attempt / relative
            actual = sha(path) if path.is_file() else None
            check(actual == expected, f"{jid}/{relative}", "Sealed artifact SHA256 mismatch or missing file")
            sealed_files += 1
        result = read(attempt / "result.json")
        check(receipt["status"] == result["status"], jid, "Receipt/result status differs")
        task = {key: job["case"].get(key) for key in (
            "problem_id", "q_start", "q_goal", "scene_sha256", "robot_sha256",
            "collision_geometry_sha256", "voxels_xyz", "voxel_radius_m",
            "velocity_limits_rad_s", "acceleration_limits_rad_s2", "endpoint_velocity_rad_s")}
        path_groups[(job["case_id"], job["roadmap_id"])].append(dict(
            job_id=jid, method=job["method"], objective=job["objective"],
            task_sha256=digest(task),
            path_sha256=sha(attempt / "path.npy") if (attempt / "path.npy").is_file() else None,
            accepted_frontend=bool(result.get("frontend_success", False))))
        if job["method"] != "NTO_PAPER":
            continue
        statuses[result["status"]] += 1
        timed = job["objective"] == "minimum_time"
        solver, order = ("IPOPT", 4) if timed else ("SNOPT", 2)
        row = dict(job_id=jid, case_id=job["case_id"], objective=job["objective"],
                   status=result["status"], attempt=str(attempt.relative_to(run)),
                   accepted_frontend=bool(result.get("frontend_success", False)),
                   result_sha256=sha(attempt / "result.json"))
        records.append(row)
        if not (attempt / "nto_parameters.json").is_file():
            check(result["status"] != SUCCESS, jid, "Successful NTO lacks parameters")
            row["artifacts"] = "not_reached_or_interrupted_during_construction"
            continue
        check(row["accepted_frontend"], jid, "NTO constructed without accepted shared DRM seed")
        p = read(attempt / "nto_parameters.json")
        initial = npz(attempt / "nto_initial.npz")
        path = np.load(attempt / "path.npy", allow_pickle=False)
        equal_path = np.array_equal(initial["original_path"], path)
        check(equal_path, jid, "NTO initialization changed native DRM path")
        expected_controls = np.vstack([path[0], path, path[-1]]) if timed else path
        check(np.array_equal(initial["controls"], expected_controls), jid, "Warm start pruned, inserted or resampled DRM nodes")
        check(p["spline_order"] == order and p["warm_start_node_count"] == len(path), jid, "Incorrect spline order/node count")
        check(p["warm_start_control_count"] == len(expected_controls), jid, "Incorrect control count")
        check(p["warm_start_endpoint_copies"] == (2 if timed else 0), jid, "Unexpected endpoint copies")
        check(p["warm_start_resampled"] is False and p["legacy_cfg_nto_used"] is False,
              jid, "Legacy/strengthened NTO knobs applied")
        check(p["solver"] == solver, jid, "Incorrect published Franka solver")
        check(p["collocation_parameter_Nc"] == 150 and p["collocation_points"] == 149,
              jid, "Nc/count differs from declared author-comment convention")
        close(p["collocation_locations"], np.arange(1, 150) / 150, jid, "Collocation locations differ", atol=0, rtol=0)
        for key, value in {"clearance_m": 0., "influence_distance_m": .01,
                           "maximum_iterations": 2000, "velocity_limit_rad_s": 1.,
                           "acceleration_limit_rad_s2": 1., "maximum_optimizer_seconds": 60.}.items():
            check(p["profile"].get(key) == value, jid, f"Profile {key} differs")
        options = p["parameters"]
        if timed:
            check(options["max_iter"] == 2000 and options["tol"] == 1e-6, jid, "IPOPT settings differ")
            limit = options["max_cpu_time"]
        else:
            check(options["Major iterations limit"] == 2000 and options["Major feasibility tolerance"] == 1e-6
                  and options["Major optimality tolerance"] == 1e-6, jid, "SNOPT settings differ")
            limit = options["Time limit"]
        check(limit <= min(60., job["evaluation_config"]["optimizer_budget_seconds"]), jid, "Solver exceeds budget")
        check(p["collision_pairs"] == 137 + 15 * len(job["case"]["voxels_xyz"]), jid, "Canonical pair count differs")
        duration0 = float(initial["duration"])
        expected_duration = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum() + 2.) if timed else 1.
        close(duration0, expected_duration, jid, "Initial duration differs")
        # Build the clamped uniform knot sequence independently of Drake.
        breaks = np.linspace(0., duration0, len(expected_controls) - order + 2)
        expected_knots = np.r_[np.repeat(0., order - 1), breaks, np.repeat(duration0, order - 1)]
        close(initial["knots"], expected_knots, jid, "Initial knot sequence differs")
        row.update(initial_original_path_equals_native_path=bool(equal_path),
                   native_path_sha256=sha(attempt / "path.npy"), warm_start_nodes=len(path),
                   warm_start_controls=len(expected_controls), order=order,
                   collocation_bindings=149, solver=solver, permitted_solver_seconds=limit)
        if not (attempt / "nto_solver_result.json").is_file():
            check(result["status"] in INTERRUPTED, jid, "Missing solver completion outside interrupted case")
            row["artifacts"] = "solver_interrupted_without_result"
            continue
        native = read(attempt / "nto_solver_result.json")
        row.update(native_solver_success=native["solver_success"], native_solver_status=native["solver_status"])
        solver_counts[(solver, native["solver_status"])] += 1
        if native["solver_status"] == "exception":
            check(result["status"] != SUCCESS, jid, "Optimizer exception counted as success")
            continue
        raw = npz(attempt / "nto_solver_raw.npz")
        check("decision_variables" in raw and raw["decision_variables"].size > 0, jid, "Missing raw decisions")
        if not (attempt / "candidate.json").is_file():
            check(result["status"] in INTERRUPTED, jid, "Completed solve lacks candidate")
            continue
        metadata, arrays = read(attempt / "candidate.json"), npz(attempt / "candidate.npz")
        controls, knots = arrays["controls"], arrays["knots"]
        check(metadata["kind"] == "bspline" and metadata["objective"] == job["objective"], jid,
              "Candidate kind/objective differs from requested NTO task")
        check(np.array_equal(controls, raw["controls"], equal_nan=True), jid, "Candidate/raw controls differ")
        with np.errstate(invalid="ignore", over="ignore"):
            scaled_knots = raw["normalized_knots"] * raw["duration"]
        check(np.array_equal(knots, scaled_knots, equal_nan=True), jid, "Candidate/raw scaled knots differ")
        check(metadata["solver_success"] == native["solver_success"] and metadata["solver_status"] == native["solver_status"],
              jid, "Candidate/native solver status differs")
        check(metadata["spline_order"] == order and len(controls) == len(expected_controls), jid, "Optimized order/control count changed")
        if not native["solver_success"]:
            check(result["status"] in ("solver_failed",) + INTERRUPTED, jid, "Unsuccessful solver misclassified")
        else:
            check(result["status"] != "solver_failed", jid, "Successful solver classified as solver failure")
        finite = np.isfinite(controls).all() and np.isfinite(knots).all() and len(knots) == len(controls) + order
        finite = finite and np.all(np.diff(knots) >= 0) and knots[order - 1] < knots[-order]
        row["finite_reconstructable_candidate"] = bool(finite)
        if not finite:
            check(result["status"] != SUCCESS, jid, "Malformed candidate counted as success")
            continue
        spline = BSpline(knots, controls, order - 1, extrapolate=False, axis=0)
        start, end = float(knots[order - 1]), float(knots[-order])
        requested = np.asarray([job["case"]["q_start"], job["case"]["q_goal"]])
        endpoint_error = float(np.linalg.norm(spline([start, end]) - requested, axis=1).max())
        active_breaks = np.unique(knots[(knots >= start) & (knots <= end)])
        cost = end - start if timed else float(np.linalg.norm(np.diff(spline(active_breaks), axis=0), axis=1).sum())
        row.update(scipy_cost=cost, scipy_endpoint_error_rad=endpoint_error)
        if native.get("solver_objective") is not None:
            close(cost, native["solver_objective"], jid, "Independent cost differs from native solver objective")
        derivative_bounds = {}
        if timed:
            velocity, acceleration = spline.derivative(1), spline.derivative(2)
            # SciPy derivative .c arrays have trailing padding; select only
            # the len(t)-k-1 actual B-spline coefficients.
            v = velocity.c[:len(velocity.t) - velocity.k - 1]
            a = acceleration.c[:len(acceleration.t) - acceleration.k - 1]
            derivative_bounds = dict(velocity_per_joint=np.max(np.abs(v), axis=0),
                acceleration_per_joint=np.max(np.abs(a), axis=0),
                endpoint_velocity=float(np.max(np.abs(velocity([start, end])))))
            row["scipy_derivative_control_bounds"] = derivative_bounds
        if not (attempt / "validation.json").is_file():
            check(result["status"] in INTERRUPTED, jid, "Finite candidate lacks completed validation")
            row["artifacts"] = "finite_candidate_validation_interrupted"
            continue
        validation = read(attempt / "validation.json")
        samples = npz(attempt / "trajectory_samples.npz")
        observed = spline(samples["time"])
        max_error = float(np.max(np.abs(observed - samples["q"])))
        close(observed, samples["q"], jid, "SciPy curve differs from saved Drake samples", atol=1e-10, rtol=1e-10)
        close(cost, validation["cost"], jid, "Independent objective differs")
        close(endpoint_error, validation["endpoint_error_rad"], jid, "Endpoint residual differs")
        for key, value in derivative_bounds.items():
            close(value, validation["derivative_bounds"][key], jid, f"Derivative bound {key} differs")
        check(np.array_equal(samples["independent"], samples["clearance"] > 0), jid, "Saved collision labels/clearances differ")
        disagreements = int(np.count_nonzero(samples["native"] != samples["independent"]))
        check(disagreements == validation["model_disagreements"], jid, "Model disagreement count differs")
        check(int(np.count_nonzero(~samples["native"])) == validation["native_collisions"], jid, "Native collision count differs")
        check(int(np.count_nonzero(~samples["independent"])) == validation["independent_collisions"], jid, "Independent collision count differs")
        close(float(samples["clearance"].min()), validation["minimum_sampled_clearance_m"], jid, "Minimum clearance differs")
        step = float(np.linalg.norm(np.diff(observed, axis=0), axis=1).max())
        close(step, validation["actual_maximum_sample_joint_step_rad"], jid, "Sample step differs")
        row.update(artifacts="finite_candidate_and_validation_audited",
                   validation_status=validation["status"], samples=len(observed),
                   scipy_maximum_sample_error_rad=max_error, scipy_maximum_sample_step_rad=step,
                   recorded_native_collisions=validation["native_collisions"],
                   recorded_independent_collisions=validation["independent_collisions"],
                   recorded_model_disagreements=disagreements,
                   recorded_minimum_clearance_m=validation["minimum_sampled_clearance_m"])
        if native["solver_success"] and result["status"] not in INTERRUPTED:
            check(result["status"] == validation["status"], jid, "Final status differs from successful-solver validation")
        if result["status"] == SUCCESS:
            check(native["solver_success"] and validation["passed"], jid, "Claimed success lacks optimizer/validator success")
            check(endpoint_error <= job["evaluation_config"]["endpoint_tolerance_rad"], jid, "Successful endpoint error exceeds bound")
            check(step <= job["evaluation_config"]["validation_step_rad"] * (1 + 1e-9) + 1e-12, jid, "Successful sample step exceeds bound")
            if timed:
                tol = job["evaluation_config"]["feasibility_tolerance"]
                check(np.max(derivative_bounds["velocity_per_joint"]) <= 1 + tol
                      and np.max(derivative_bounds["acceleration_per_joint"]) <= 1 + tol
                      and derivative_bounds["endpoint_velocity"] <= tol, jid, "Successful dynamic limits violated")
    shared_paths = []
    for (case_id, roadmap), entries in sorted(path_groups.items()):
        tasks = {x["task_sha256"] for x in entries}
        accepted_paths = {x["path_sha256"] for x in entries if x["accepted_frontend"]}
        check(len(tasks) == 1, f"{case_id}/{roadmap}", "Methods received different frozen tasks")
        if len(accepted_paths) > 1:
            warnings.append(dict(location=f"{case_id}/{roadmap}",
                message="Separate native DRM calls returned different accepted paths; disclose paired-comparison limitation"))
        shared_paths.append(dict(case_id=case_id, roadmap_id=roadmap,
            fixed_task_hashes=sorted(tasks), accepted_seed_path_hashes=sorted(accepted_paths), entries=entries))
    check(receipts == len(manifest["jobs"]), "run", "Not every planned job has a terminal receipt")
    return clean(dict(audit_type="Independent CPU artifact audit with SciPy BSpline; no GPU/solver invocation",
        created_unix_seconds=time.time(), run=str(run), run_manifest_sha256=sha(run / "run_manifest.json"),
        frozen_planning_commit=manifest["source_code_commit"], audit_script_sha256=sha(__file__),
        environment=dict(numpy=np.__version__, scipy=scipy.__version__, python=sys.version),
        passed=not issues, issues=issues, warnings=warnings, planned_jobs=len(manifest["jobs"]),
        receipts_verified=receipts, sealed_files_verified=sealed_files, source_snapshot_checks=source_checks,
        nto_jobs=len(records), nto_status_counts=dict(statuses),
        native_solver_status_counts=[dict(solver=s, status=t, count=n) for (s, t), n in sorted(solver_counts.items())],
        reconstructed_finite_candidates=sum(r.get("finite_reconstructable_candidate", False) for r in records),
        numerical_validation_artifacts=sum("validation_status" in r for r in records),
        recorded_model_disagreements=sum(r.get("recorded_model_disagreements", 0) for r in records),
        maximum_scipy_sample_error_rad=max((r["scipy_maximum_sample_error_rad"] for r in records
                                         if "scipy_maximum_sample_error_rad" in r), default=None),
        records=records, shared_fixed_input_and_seed_path_checks=shared_paths,
        limitations=["Audits existing collision arrays/hashes; does not recompute geometry collisions.",
                     "Failed solvers remain failures even if their numerical candidate validation passes.",
                     "Nc150 uses149 internal k/Nc locations from unpublished TeX comments; paper prose says150 points.",
                     "Kinova adaptation; not original FR3 or hardware reproduction.",
                     "Finite collision sampling is not a mathematical continuous-safety certificate."]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.run.resolve() == args.output.resolve() or args.run.resolve() in args.output.resolve().parents:
        parser.error("Audit output must be outside the sealed run directory")
    report = audit(args.run)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
    print(json.dumps({k: report[k] for k in ("passed", "planned_jobs", "receipts_verified", "nto_jobs",
          "nto_status_counts", "reconstructed_finite_candidates", "recorded_model_disagreements",
          "maximum_scipy_sample_error_rad", "issues", "warnings")}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
