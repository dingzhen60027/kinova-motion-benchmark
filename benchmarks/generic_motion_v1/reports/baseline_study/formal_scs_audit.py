"""Formal-study copy with explicit expected-source binding; independent LSCS/SCSTO audit: NumPy and SciPy BPoly only.

No runtime validator, Drake, GPU, planner or optimizer is imported. Geometry
collision signs are audited from saved arrays, not newly computed collisions.
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
import re
from pathlib import Path
import sys
import time
import numpy as np
import scipy
from scipy.interpolate import BPoly

SUCCESS = "success_validated_numerical"
ORIGINAL_AUDITOR_SOURCE = "72984027030175b7bca78e2dcf592b38f0c1864b"
ORIGINAL_AUDITOR_SHA256 = "3c2cb64f50ecb52cad34e5ef53afa147b9c8fc59cd1798978722ba333abc537e"
TERMINAL = {SUCCESS, "timeout", "invalid_endpoint", "invalid_scene_geometry", "model_consistency_failed",
            "collision_failed", "limit_failed", "solver_failed", "frontend_failed",
            "validation_undetermined", "infrastructure_error", "unsupported"}


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def arrays(path):
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key].copy() for key in data.files}


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


def audit(run, expected_source):
    if not isinstance(expected_source, str) or re.fullmatch(r"[0-9a-f]{40}", expected_source) is None:
        raise ValueError("Expected source must be an explicit full 40-character Git commit")
    run = Path(run).resolve()
    manifest_path = run / "run_manifest.json"
    manifest = read(manifest_path)
    issues, records, source_checks = [], [], []
    receipts, sealed_files, latest_jobs = 0, 0, 0
    status_counts = Counter()

    def check(condition, location, message):
        if not bool(condition):
            issues.append(dict(location=location, message=message))

    def close(actual, expected, location, message, atol=2e-9, rtol=1e-9):
        a, b = np.asarray(actual), np.asarray(expected)
        check(a.shape == b.shape and np.allclose(a, b, atol=atol, rtol=rtol), location, message)

    check(read(run / "identity.json")["run_manifest_sha256"] == sha(manifest_path), "run", "Manifest identity changed")
    check(manifest["source_code_commit"] == expected_source, "run", "Unexpected planning source commit")
    check(len({j["job_id"] for j in manifest["jobs"]}) == len(manifest["jobs"]), "run", "Duplicate job IDs")
    input_manifest = Path(manifest["manifest_path"])
    check(input_manifest.is_file() and sha(input_manifest) == manifest["manifest_sha256"],
          "run", "Frozen input manifest changed")
    for relative, expected in manifest["source_hashes"].items():
        path = run / "source_snapshot" / relative
        actual = sha(path) if path.is_file() else None
        check(actual == expected, relative, "Frozen source snapshot changed")
        source_checks.append(dict(path=relative, sha256=actual, expected_sha256=expected))
    for name, expected in manifest["roadmap_files"].items():
        path = Path(name)
        check(path.is_file() and sha(path) == expected, name, "Frozen roadmap artifact changed")
    robot = read(run / "source_snapshot/benchmarks/generic_motion_v1/configs/robot.json")
    lower, upper = np.asarray(robot["lower"]), np.asarray(robot["upper"])
    band = float(robot["contact_band_m"])

    def audit_candidate(directory, job):
        loc = str(directory.relative_to(run))
        meta, data = read(directory / "candidate.json"), arrays(directory / "candidate.npz")
        cfg, case = job["evaluation_config"], job["case"]
        timed = job["objective"] == "minimum_time"
        check(meta["kind"] == ("bezier" if timed else "polygon"), loc, "Wrong SCS candidate representation")
        check(meta["objective"] == job["objective"] and meta["solver"] == "Clarabel", loc, "Wrong objective or solver")
        row = dict(directory=loc, objective=job["objective"], solver_success=meta["solver_success"],
                   solver_status=meta["solver_status"], candidate_sha256=sha(directory / "candidate.npz"))
        if not (directory / "validation.json").is_file():
            row["scope"] = "saved_candidate_without_completed_validation"
            return row
        validation = read(directory / "validation.json")
        saved = arrays(directory / "trajectory_samples.npz")
        q, times, segments = saved["q"], saved["time"], saved["segment"]
        check(q.ndim == 2 and q.shape[1] == 7 and np.isfinite(q).all(), loc, "Malformed saved positions")
        check(times.shape == segments.shape == (len(q),) and segments.dtype.kind in "iu", loc, "Malformed sample indexing")
        regions = arrays(directory / "regions.npz")
        degree = None
        derivative_bounds = {}
        if timed:
            controls, durations = data["controls"], data["durations"]
            check(controls.ndim == 3 and controls.shape[2] == 7 and durations.shape == (len(controls),)
                  and np.isfinite(controls).all() and np.isfinite(durations).all() and np.all(durations > 0),
                  loc, "Malformed Bezier controls/durations")
            degree = controls.shape[1] - 1
            check(degree == cfg["bezier_degree"], loc, "Bezier degree differs from frozen evaluation")
            edges = np.r_[0., np.cumsum(durations)]
            # BPoly's axes are coefficient, interval, coordinate. Evaluate each
            # saved segment separately so duplicate boundary samples retain the
            # original left/right segment attribution and expose discontinuity.
            curves = [BPoly(controls[i, :, None, :], edges[i:i+2], extrapolate=False)
                      for i in range(len(controls))]
            velocity = [curve.derivative(1) for curve in curves]
            acceleration = [curve.derivative(2) for curve in curves]
            derivative_bounds = dict(
                velocity_per_joint=np.max(np.abs(np.concatenate([v.c[:, 0, :] for v in velocity])), axis=0),
                acceleration_per_joint=np.max(np.abs(np.concatenate([a.c[:, 0, :] for a in acceleration])), axis=0),
                endpoint_velocity=float(np.max(np.abs(np.r_[velocity[0](edges[0]), velocity[-1](edges[-1])]))),
                continuity_residual=max([0.] +
                    [float(np.max(np.abs(curves[i](edges[i+1])-curves[i+1](edges[i+1])))) for i in range(len(curves)-1)] +
                    [float(np.max(np.abs(velocity[i](edges[i+1])-velocity[i+1](edges[i+1])))) for i in range(len(curves)-1)]))
            count = len(curves)
            cost = float(durations.sum())
            candidate_endpoints = np.asarray([curves[0](edges[0]), curves[-1](edges[-1])])
            positions = controls.reshape(-1, 7)
            reconstructed = np.empty_like(q)
            for i, curve in enumerate(curves):
                mask = segments == i
                check(np.any(mask), loc, f"Missing Bezier segment {i} samples")
                check(np.all(times[mask] >= edges[i]-1e-12) and np.all(times[mask] <= edges[i+1]+1e-12),
                      loc, f"Sample times escape segment {i}")
                reconstructed[mask] = curve(times[mask])
                close(times[mask][[0,-1]], edges[i:i+2], loc, f"Segment {i} endpoints omitted")
            residual = max(float(np.max(regions[f"A_{i:04d}"] @ controls[i].T - regions[f"b_{i:04d}"][:,None]))
                           for i in range(count))
            close(cost, validation["duration_s"], loc, "Bezier physical duration differs")
            for key, value in derivative_bounds.items():
                close(value, validation["derivative_bounds"][key], loc, f"Independent BPoly derivative {key} differs")
        else:
            points = data["points"]
            check(points.ndim == 2 and points.shape[1] == 7 and len(points) >= 2 and np.isfinite(points).all(),
                  loc, "Malformed polygon")
            count = len(points)-1
            reconstructed = np.empty_like(q)
            for i in range(count):
                mask = segments == i
                check(np.any(mask), loc, f"Missing polygon segment {i} samples")
                parameter = times[mask] * count - i
                check(np.all(parameter >= -1e-12) and np.all(parameter <= 1+1e-12), loc, "PWL parameter outside segment")
                reconstructed[mask] = points[i] + parameter[:,None]*(points[i+1]-points[i])
                close(parameter[[0,-1]], [0.,1.], loc, f"Polygon segment {i} endpoints omitted")
            cost = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
            candidate_endpoints, positions = points[[0,-1]], points
            residual = max(float(np.max(regions[f"A_{i:04d}"] @ points[i:i+2].T - regions[f"b_{i:04d}"][:,None]))
                           for i in range(count))
            check(validation["duration_s"] is None and validation["derivative_bounds"] == {}, loc, "PWL has invented dynamics")
        check(np.all((segments >= 0) & (segments < count)) and np.all(np.diff(segments) >= 0), loc, "Out-of-order/invalid region mapping")
        check(set(regions) == {f"{prefix}_{i:04d}" for prefix in ("A","b") for i in range(count)},
              loc, "Region count differs from ordered trajectory segments")
        maximum_error = float(np.max(np.abs(reconstructed-q)))
        close(reconstructed, q, loc, "Independent interpolation differs from saved samples", atol=2e-10, rtol=1e-10)
        requested = np.asarray([case["q_start"], case["q_goal"]])
        endpoint_error = float(np.linalg.norm(candidate_endpoints-requested, axis=1).max())
        path_length = float(np.linalg.norm(np.diff(reconstructed,axis=0),axis=1).sum())
        normalized_length = float(np.linalg.norm(np.diff(reconstructed,axis=0)/(upper-lower),axis=1).sum())
        step = float(np.linalg.norm(np.diff(reconstructed,axis=0),axis=1).max())
        limits = bool(np.all(positions >= lower-cfg["feasibility_tolerance"]) and np.all(positions <= upper+cfg["feasibility_tolerance"]))
        for value, expected, message in (
            (cost,validation["cost"],"cost"), (endpoint_error,validation["endpoint_error_rad"],"endpoint residual"),
            (residual,validation["region_residual"],"region membership residual"),
            (path_length,validation["path_length_rad"],"sampled path length"),
            (normalized_length,validation["normalized_path_length"],"normalized path length"),
            (step,validation["actual_maximum_sample_joint_step_rad"],"maximum sample step")):
            close(value,expected,loc,f"Independent {message} differs")
        check(validation["joint_limits_passed"] == limits, loc, "Joint limit flag differs")
        check(validation["cost_unit"] == ("s" if timed else "rad"), loc, "Incorrect objective units")
        native, independent, clearance = saved["native"], saved["independent"], saved["clearance"]
        check(native.dtype.kind == independent.dtype.kind == "b" and native.shape == independent.shape == clearance.shape == (len(q),),
              loc, "Malformed saved collision labels")
        check(np.isfinite(clearance).all() and np.array_equal(independent,clearance>0), loc, "Independent collision signs differ")
        collisions = ~native | ~independent
        ambiguous = np.abs(clearance) <= band
        counts = dict(native_collisions=int((~native).sum()), independent_collisions=int((~independent).sum()),
                      model_disagreements=int((native != independent).sum()), contact_band_samples=int(ambiguous.sum()))
        for key, value in counts.items():
            check(validation[key] == value,loc,f"Saved collision count {key} differs")
        check(validation["collision_free"] == (not np.any(collisions)),loc,"Collision-free flag differs")
        close(float(clearance.min()), validation["minimum_sampled_clearance_m"],loc,"Clearance minimum differs")
        close(q[collisions], np.asarray(validation["collision_points"]).reshape(-1,7),loc,"Collision witness positions differ")
        check(np.array_equal(segments[collisions],np.asarray(validation["collision_regions"],int)),loc,"Collision witness segment mapping differs")
        vmax, amax, end_velocity = (np.asarray(case[k]) for k in
                                  ("velocity_limits_rad_s", "acceleration_limits_rad_s2", "endpoint_velocity_rad_s"))
        check(np.array_equal(vmax,np.ones(7)*cfg["velocity_limit_rad_s"]) and
              np.array_equal(amax,np.ones(7)*cfg["acceleration_limit_rad_s2"]) and np.array_equal(end_velocity,np.zeros(7)),
              loc,"Frozen motion limits differ")
        tol = cfg["feasibility_tolerance"]
        dynamic = (not timed or (np.all(derivative_bounds["velocity_per_joint"] <= vmax+tol) and
                    np.all(derivative_bounds["acceleration_per_joint"] <= amax+tol) and
                    derivative_bounds["endpoint_velocity"] <= tol and derivative_bounds["continuity_residual"] <= tol))
        check(bool(dynamic) == validation["derivative_limits_passed"],loc,"Derivative-limit flag differs")
        if endpoint_error > cfg["endpoint_tolerance_rad"]:
            status = "invalid_endpoint"
        elif not limits or not dynamic or residual > tol:
            status = "limit_failed"
        elif np.any(collisions & ~ambiguous):
            status = "collision_failed"
        elif np.any(ambiguous) or step > cfg["validation_step_rad"]*(1+1e-9)+1e-12:
            status = "validation_undetermined"
        else:
            status = SUCCESS
        check(status == validation["status"] and validation["passed"] == (status == SUCCESS),loc,"Validation decision conflicts with independently checked evidence")
        row.update(scope="candidate_and_validation_audited", reconstructed_kind=meta["kind"],degree=degree,
                   independently_reconstructed_cost=cost, endpoint_error_rad=endpoint_error,
                   maximum_sample_error_rad=maximum_error, maximum_sample_joint_step_rad=step,
                   normalized_path_length=normalized_length, region_residual=residual,
                   derivative_control_bounds=derivative_bounds, validation_status=status,
                   samples=len(q), minimum_sampled_clearance_m=float(clearance.min()), **counts)
        return row

    for job in manifest["jobs"]:
        jid = job["job_id"]
        check(Path(jid).name == jid and jid not in (".",".."),jid,"Invalid job directory name")
        attempts = sorted((run/"jobs"/jid/"attempts").glob("[0-9][0-9][0-9][0-9]"))
        if not attempts or not (attempts[-1]/"receipt.json").is_file():
            check(False,jid,"Incomplete run: latest attempt is not sealed")
            continue
        for attempt in attempts:
            loc = str(attempt.relative_to(run))
            try:
                receipt = read(attempt/"receipt.json")
                receipts += 1
                files = receipt["files"]
                check("result.json" in files,loc,"Receipt lacks result")
                for relative, expected in files.items():
                    path = (attempt/relative).resolve()
                    safe = path.is_relative_to(attempt.resolve()) and relative != "receipt.json"
                    check(safe and path.is_file() and sha(path) == expected,loc+"/"+relative,"Sealed file missing/changed/unsafe")
                    sealed_files += 1
                actual_files = {str(p.relative_to(attempt)) for p in attempt.rglob("*")
                                if p.is_file() and p.name != "receipt.json" and "replay" not in p.relative_to(attempt).parts}
                check(actual_files == set(files),loc,"Unsealed or absent artifacts")
                result = read(attempt/"result.json")
                check(result["status"] in TERMINAL and receipt["status"] == result["status"],loc,"Nonterminal/mismatched status")
                latest = attempt == attempts[-1]
                if latest:
                    check(not receipt.get("retry_pending"),loc,"Latest attempt still pending retry")
                    latest_jobs += int(not receipt.get("retry_pending",False))
                if not receipt.get("retry_pending"):
                    check(read(attempt/"request.json") == job,loc,"Request differs from frozen job")
                    process = read(attempt/"process.json")
                    if result["status"] == SUCCESS:
                        check(process["exit_code"] == 0 and process["timed_out"] is False,loc,"Success contradicts process")
                    runtime = read(attempt/"runtime_sources.json")
                    check(runtime["before"] == manifest["source_hashes"],loc,"Runtime sources differ from snapshot")
                    unchanged = runtime["before"] == runtime["after"]
                    check(runtime["unchanged"] == unchanged,loc,"Runtime source-change flag incorrect")
                    if not unchanged:
                        check(result["status"] == "infrastructure_error",loc,"Source drift counted as valid experiment")
                if job["method"] != "EI_ZO":
                    continue
                row = dict(job_id=jid,attempt=attempt.name,latest=latest,status=result["status"],
                           objective=job["objective"],family=job["family"],rounds=[],root_candidate_rounds=[])
                records.append(row)
                if latest:
                    status_counts[(job["objective"],result["status"])] += 1
                if (attempt/"input.json").is_file():
                    task=read(attempt/"input.json");task.pop("_objective",None)
                    check(task == job["case"],loc,"Saved fixed task differs")
                check(job["case"]["robot_sha256"] == robot["robot_sha256"] and
                      job["case"]["collision_geometry_sha256"] == robot["collision_geometry_sha256"],loc,"Canonical model hashes differ")
                candidate_rounds = sorted((attempt/"rounds").glob("[0-9][0-9][0-9]"))
                for rd in candidate_rounds:
                    if not (rd/"candidate.json").is_file():
                        row["rounds"].append(dict(directory=str(rd.relative_to(run)),scope="optimizer_input_only_no_returned_candidate"))
                        continue
                    rr = audit_candidate(rd,job)
                    row["rounds"].append(rr)
                    if ((attempt/"candidate.npz").is_file() and (attempt/"candidate.json").is_file() and
                        sha(attempt/"candidate.npz") == sha(rd/"candidate.npz") and
                        read(attempt/"candidate.json") == read(rd/"candidate.json")):
                        row["root_candidate_rounds"].append(rd.name)
                if (attempt/"candidate.json").is_file():
                    check(bool(row["root_candidate_rounds"]),loc,"Root candidate does not match any sealed optimizer round")
                if result["status"] == SUCCESS:
                    check(bool(row["root_candidate_rounds"]),loc,"Success lacks candidate")
                    final = attempt/"rounds"/row["root_candidate_rounds"][-1]
                    v=read(final/"validation.json");c=read(final/"candidate.json")
                    check(v["passed"] and v["status"] == SUCCESS and c["solver_success"] is True,loc,"Success lacks optimizer AND numerical acceptance")
                    check(read(attempt/"validation.json") == v,loc,"Successful root validation differs from final candidate")
                    check(sha(attempt/"trajectory_samples.npz") == sha(final/"trajectory_samples.npz"),loc,"Successful final samples differ")
                    check(sha(attempt/"regions.npz") == sha(final/"regions.npz"),loc,"Successful final ordered pool differs")
                    close(result["metrics"]["cost"],v["cost"],loc,"Success metric cost differs")
                    row["successful_final_round"] = final.name
                else:
                    row["root_validation_scope"] = "Last completed validation may precede an interrupted/failed later solve; never upgraded to success"
            except (OSError,ValueError,KeyError,TypeError,IndexError) as error:
                check(False,loc,"Artifact inspection failed: "+repr(error))
    check(latest_jobs == len(manifest["jobs"]),"run","Not every planned job has a sealed terminal latest attempt")
    rounds = [r for row in records for r in row["rounds"] if r.get("scope") == "candidate_and_validation_audited"]
    return clean(dict(audit_type="Independent CPU LSCS/SCSTO artifact audit using NumPy and SciPy BPoly; no GPU/solver/runtime-validator invocation",
        created_unix_seconds=time.time(),run=str(run),run_manifest_sha256=sha(manifest_path),
        frozen_planning_commit=manifest["source_code_commit"],audit_script_sha256=sha(__file__),
        expected_source_commit=expected_source,original_auditor_source_commit=ORIGINAL_AUDITOR_SOURCE,
        original_auditor_sha256=ORIGINAL_AUDITOR_SHA256,
        passed=not issues,complete=latest_jobs == len(manifest["jobs"]),issues=issues,
        planned_jobs=len(manifest["jobs"]),completed_latest_jobs=latest_jobs,receipts_verified=receipts,
        sealed_files_verified=sealed_files,source_snapshot_checks=source_checks,
        environment=dict(numpy=np.__version__,scipy=scipy.__version__,python=sys.version),
        scs_attempts=len(records),scs_latest_jobs=sum(r["latest"] for r in records),
        scs_status_counts=[dict(objective=o,status=s,count=n) for (o,s),n in sorted(status_counts.items())],
        audited_candidate_validation_rounds=len(rounds),
        audited_polygon_rounds=sum(r["reconstructed_kind"] == "polygon" for r in rounds),
        audited_bezier_rounds=sum(r["reconstructed_kind"] == "bezier" for r in rounds),
        maximum_independent_sample_error_rad=max((r["maximum_sample_error_rad"] for r in rounds),default=None),
        records=records,limitations=[
            "Existing collision signs/clearances are checked for accounting and interpolation consistency; no new geometry collision queries.",
            "Derivative control-hull bounds are sufficient numerical bounds; finite collision samples are not a continuous collision proof.",
            "All returned rounds, including colliding/failed candidates, are kept; partial interrupted optimizer inputs are explicitly labeled.",
            "Author LSCS/SCSTO return exposes no separate native solver iteration receipt; solver-success flag is checked against the preserved candidate and final independent numerical validation.",
            "Kinova adaptation, not original FR3 geometry or hardware reproduction."]))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--expected-source",required=True)
    args=parser.parse_args()
    if args.output.resolve() == args.run.resolve() or args.run.resolve() in args.output.resolve().parents:
        parser.error("Audit output must be outside the sealed run")
    report=audit(args.run,args.expected_source)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,sort_keys=True,allow_nan=False)+"\n")
    print(json.dumps({k:report[k] for k in ("passed","complete","planned_jobs","receipts_verified",
        "sealed_files_verified","scs_latest_jobs","audited_candidate_validation_rounds",
        "maximum_independent_sample_error_rad","issues")},indent=2))
    return 0 if report["passed"] and report["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
