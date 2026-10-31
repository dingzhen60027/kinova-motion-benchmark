"""CPU-only extraction from sealed paired development evidence; no planning."""
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np

from benchmarks.generic_motion_v1.batch import verify


BASE = Path(__file__).resolve().parents[1]
RUN = BASE / "results/dev_paired_v2"


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def derivative_controls(controls, knots, order):
    denominator = knots[order:order + len(controls) - 1] - knots[1:len(controls)]
    if np.any(denominator <= 0):
        raise ValueError("Nonpositive derivative knot interval")
    return (order - 1) * np.diff(controls, axis=0) / denominator[:, None], knots[1:-1]


def main():
    verification = verify(RUN)
    assert verification["complete"] and verification["integrity_passed"], verification
    manifest = read(RUN / "run_manifest.json")
    summary = read(RUN / "summary/summary.json")
    acceptance = read(BASE / "reports/resume_and_pairing_acceptance.json")
    grouped, failures, cases, replay = defaultdict(list), Counter(), {}, []
    maximum_derivative_recompute_error = 0.
    maximum_endpoint_recompute_error = 0.
    for job in manifest["jobs"]:
        case = job["case"]
        cases.setdefault(case["problem_id"], dict(problem_id=case["problem_id"],
            scene_id=case["scene_id"], family=case["family"],
            distance_bin=case["endpoint_sampling"]["distance_bin"],
            distance_rad=case["start_goal_joint_distance"],
            normalized_distance=case["normalized_start_goal_joint_distance"],
            straight_line_collision=case["straight_line_collision"],
            scene_sha256=case["scene_sha256"], geometry_sha256=case["obstacle_geometry_sha256"]))
        attempts = sorted((RUN / "jobs" / job["job_id"] / "attempts").glob("[0-9][0-9][0-9][0-9]"))
        attempt = attempts[-1]
        receipt = read(attempt / "receipt.json")
        assert not receipt.get("retry_pending")
        result = read(attempt / "result.json")
        if job["roadmap_id"].endswith("-s0") and (result["status"] == "success_validated_numerical"
                or (job["method"] == "EI_ZO" and job["objective"] == "minimum_distance")):
            replay.append(dict(job_id=job["job_id"], status=result["status"],
                attempt=str(attempt.relative_to(BASE)), reason="Fixed seed-0 paired successful cases and one frontend failure per unsuccessful family"))
        if result["status"] != "success_validated_numerical":
            failures[(job["family"], result["status"], result.get("failed_stage"), result.get("exception"))] += 1
            continue
        validation = read(attempt / "validation.json")
        assert validation["passed"] and validation["joint_limits_passed"] and validation["derivative_limits_passed"]
        candidate = read(attempt / "candidate.json")
        with np.load(attempt / "candidate.npz") as data:
            arrays = {name: data[name] for name in data.files}
        with np.load(attempt / "trajectory_samples.npz") as data:
            q = data["q"]
            assert data["native"].all() and data["independent"].all()
            assert np.min(data["clearance"]) > 2e-6
        endpoint_error = float(np.max(np.linalg.norm(q[[0, -1]] - [case["q_start"], case["q_goal"]], axis=1)))
        maximum_endpoint_recompute_error = max(maximum_endpoint_recompute_error,
            abs(endpoint_error - validation["endpoint_error_rad"]))
        derivative = validation["derivative_bounds"]
        if derivative:
            if candidate["kind"] == "bezier":
                controls, durations = arrays["controls"], arrays["durations"]
                degree = controls.shape[1] - 1
                velocity = degree * np.diff(controls, axis=1) / durations[:, None, None]
                acceleration = (degree - 1) * np.diff(velocity, axis=1) / durations[:, None, None]
                vbound, abound = np.max(np.abs(velocity), axis=(0, 1)), np.max(np.abs(acceleration), axis=(0, 1))
            else:
                velocity, knots = derivative_controls(arrays["controls"], arrays["knots"], candidate["spline_order"])
                acceleration, _ = derivative_controls(velocity, knots, candidate["spline_order"] - 1)
                vbound, abound = np.max(np.abs(velocity), axis=0), np.max(np.abs(acceleration), axis=0)
            maximum_derivative_recompute_error = max(maximum_derivative_recompute_error,
                float(np.max(np.abs(vbound - derivative["velocity_per_joint"]))),
                float(np.max(np.abs(abound - derivative["acceleration_per_joint"]))))
        grouped[(job["method"], job["objective"])].append(dict(job_id=job["job_id"],
            validation=validation, path_shape=list(np.load(attempt / "path.npy").shape),
            euclidean_endpoint_distance_rad=case["start_goal_joint_distance"]))
    assert maximum_endpoint_recompute_error < 1e-13
    assert maximum_derivative_recompute_error < 1e-12
    worst = []
    for (method, objective), group in sorted(grouped.items()):
        vals = [r["validation"] for r in group]
        derivatives = [v["derivative_bounds"] for v in vals if v["derivative_bounds"]]
        worst.append(dict(method=method, objective=objective, successes=len(vals),
            maximum_endpoint_error_rad=max(v["endpoint_error_rad"] for v in vals),
            maximum_velocity_bound_rad_s=max(max(d["velocity_per_joint"]) for d in derivatives) if derivatives else None,
            maximum_acceleration_bound_rad_s2=max(max(d["acceleration_per_joint"]) for d in derivatives) if derivatives else None,
            maximum_endpoint_velocity_rad_s=max(d["endpoint_velocity"] for d in derivatives) if derivatives else None,
            maximum_continuity_residual=max(d["continuity_residual"] for d in derivatives) if derivatives else None,
            minimum_sampled_clearance_m=min(v["minimum_sampled_clearance_m"] for v in vals),
            maximum_sample_step_rad=max(v["actual_maximum_sample_joint_step_rad"] for v in vals),
            samples_total=sum(v["samples"] for v in vals),
            sample_count_range=[min(v["samples"] for v in vals), max(v["samples"] for v in vals)],
            collision_counts={key: sum(v[key] for v in vals) for key in
                ["native_collisions", "independent_collisions", "model_disagreements", "contact_band_samples"]},
            path_shapes=[r["path_shape"] for r in group],
            maximum_distance_cost_minus_endpoint_distance_rad=max(abs(r["validation"]["cost"] - r["euclidean_endpoint_distance_rad"]) for r in group)
                if objective == "minimum_distance" else None))
    result = dict(run=str(RUN.relative_to(BASE)), source_code_commit=manifest["source_code_commit"],
        verification=verification, selection=manifest["selection"], planned_jobs=len(manifest["jobs"]),
        timeout_seconds=manifest["timeout_seconds"], cases=list(cases.values()),
        summary_groups=[g for g in summary["groups"] if g["family"] == "all"],
        paired_comparisons=summary["paired_comparisons"], successful_validation_worst_cases=worst,
        recomputed_from_saved_arrays=dict(no_model_or_gpu_calls=True,
            maximum_endpoint_error_difference=maximum_endpoint_recompute_error,
            maximum_derivative_bound_difference=maximum_derivative_recompute_error),
        failures=[dict(family=key[0], status=key[1], stage=key[2], error=key[3], count=count)
                  for key, count in sorted(failures.items())],
        resume_acceptance=dict(initial_two_attempts_preserved=acceptance["initial_two_attempts_preserved"],
            completed_run_no_artifact_changes=acceptance["resume_completed_run_no_artifact_changes"],
            pairs_with_both_paths=sum(p["both_have_path"] for p in acceptance["paired_paths"]),
            identical_path_pairs=sum(p["identical_path"] is True for p in acceptance["paired_paths"]),
            paired_presence_matches=sum(p["same_presence"] for p in acceptance["paired_paths"])),
        sources_sha256={str(path.relative_to(BASE)): sha(path) for path in
            [RUN / "run_manifest.json", RUN / "summary/summary.json",
             BASE / "reports/resume_and_pairing_acceptance.json", Path(__file__)]},
        replay_selection=replay, replay_selection_status="planned only; no new replay or Model call in this CPU audit")
    replay_acceptance = BASE / "reports/replay_acceptance.json"
    if replay_acceptance.exists():
        rendered = read(replay_acceptance)
        generated_ids = {r["job_id"] for r in rendered.get("outputs", []) if r.get("validation_passed")}
        if rendered.get("complete") and all(r["job_id"] in generated_ids for r in replay):
            result["replay_selection_status"] = "All ten frozen selections generated separately after timing runs; see replay_acceptance.json. This extraction remains CPU-only and does not construct Model."
            result["sources_sha256"][str(replay_acceptance.relative_to(BASE))] = sha(replay_acceptance)
    out = BASE / "reports/paired_baseline_audit.json"
    out.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(dict(output=str(out), verification=verification,
        recomputation=result["recomputed_from_saved_arrays"], worst=worst), indent=2))


if __name__ == "__main__":
    main()
