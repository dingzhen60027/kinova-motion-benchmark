"""Read-only sealed-study audit. Imports NumPy/SciPy, never runtime or Drake.

Checks receipts for all methods, and reconstructs refined NTO with SciPy.
Existing collision arrays are audited, not recomputed. Old and new profiles
remain separate paired observations; this script never selects a best result.
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
PROFILE = "rss2025_nto_numerical_refinement_v1"
TERMINAL = {SUCCESS, "timeout", "invalid_endpoint", "invalid_scene_geometry",
            "model_consistency_failed", "collision_failed", "limit_failed",
            "solver_failed", "frontend_failed", "validation_undetermined",
            "infrastructure_error", "unsupported"}
METHOD_PAIRS = {("EI_ZO", "minimum_distance"), ("EI_ZO", "minimum_time"),
                ("LGCS", "minimum_distance"), ("NTO_REFINED", "minimum_distance"),
                ("NTO_REFINED", "minimum_time")}


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def arrays(path):
    with np.load(path, allow_pickle=False) as archive:
        return {k: archive[k].copy() for k in archive.files}


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


class Audit:
    def __init__(self):
        self.issues, self.notes = [], []
        self.receipts = self.files = self.curves = 0
        self.maximum_curve_error = 0.

    def check(self, condition, where, message):
        if not bool(condition):
            self.issues.append(dict(location=str(where), message=message))

    def close(self, actual, expected, where, message, atol=1e-9, rtol=1e-9):
        a, b = np.asarray(actual), np.asarray(expected)
        self.check(a.shape == b.shape and np.allclose(a, b, atol=atol, rtol=rtol,
                   equal_nan=True), where, message)

    def receipt(self, attempt):
        receipt = read(attempt / "receipt.json")
        self.receipts += 1
        for relative, expected in receipt["files"].items():
            path = attempt / relative
            self.check(not Path(relative).is_absolute() and ".." not in Path(relative).parts,
                       path, "Unsafe receipt file path")
            self.check(path.is_file() and sha(path) == expected, path, "Receipt SHA mismatch or file missing")
            self.files += 1
        actual = {str(p.relative_to(attempt)) for p in attempt.rglob("*")
                  if p.is_file() and p.name != "receipt.json"}
        self.check(actual == set(receipt["files"]), attempt, "Receipt does not cover every saved artifact")
        result = read(attempt / "result.json")
        self.check(receipt["status"] == result["status"], attempt, "Receipt and final status differ")
        return result


def curve(candidate, controls, knots):
    controls, knots = np.asarray(controls), np.asarray(knots)
    order = candidate["spline_order"]
    if (controls.ndim != 2 or controls.shape[1] != 7 or
            not np.isfinite(controls).all() or not np.isfinite(knots).all() or
            knots.shape != (len(controls)+order,) or np.any(np.diff(knots) < 0) or
            knots[order-1] >= knots[-order]):
        raise ValueError("Candidate cannot be independently reconstructed")
    return BSpline(knots, controls, order-1, extrapolate=False, axis=0)


def expected_additions(locations, parameters, clearances):
    """Independent implementation of the predeclared interval-minimum policy."""
    points = []
    bounds = [0., *locations, 1.]
    for a, b in zip(bounds[:-1], bounds[1:]):
        eligible = [(float(d), float(t), i) for i, (t, d) in enumerate(zip(parameters, clearances))
                    if a+1e-12 < t < b-1e-12 and d < .0005]
        if eligible:
            # The implementation's argmin resolves an exact distance tie by
            # first sample index, not by smallest parameter.
            d, t, _ = min(eligible, key=lambda entry: (entry[0], entry[2]))
            points.append((d, t))
    points.sort()
    return np.unique([t for _, t in points[:max(0, 600-len(locations))]])


def check_profile(audit, parameters, job, where):
    p = parameters["profile"]
    values = dict(name=PROFILE, clearance_m=.001, refinement_guard_m=.0005,
                  maximum_native_solves=4, maximum_collocation_points=600,
                  maximum_iterations=2000, snopt_total_iterations=100000,
                  snopt_minor_iterations=500, influence_distance_m=.01,
                  solver_feasibility_tolerance=1e-6, warm_start_resampled=False,
                  maximum_optimizer_seconds=60.)
    for key, value in values.items():
        audit.check(p.get(key) == value, where, f"Refinement policy changed: {key}")
    timed = job["objective"] == "minimum_time"
    opts = parameters["parameters"]
    expected = (dict(max_iter=2000, tol=1e-6) if timed else
                {"Iterations limit":100000, "Major iterations limit":2000,
                 "Minor iterations limit":500, "Major feasibility tolerance":1e-6,
                 "Major optimality tolerance":1e-6})
    for key, value in expected.items():
        audit.check(opts.get(key) == value, where, f"Native solver option changed: {key}")
    audit.check(parameters["solver"] == ("IPOPT" if timed else "SNOPT"), where, "Wrong objective solver")
    audit.check(parameters["spline_order"] == (4 if timed else 2), where, "Spline order changed")
    audit.check(parameters["collision_pairs"] == 137+15*len(job["case"]["voxels_xyz"]), where, "Collision pair count changed")
    audit.check(parameters["legacy_cfg_nto_used"] is False, where, "Legacy knobs silently applied")
    return float(opts["max_cpu_time" if timed else "Time limit"])


def check_refined_attempt(audit, attempt, job, result, bounds=None):
    """Audit all completed and interrupted inner rounds without re-solving."""
    where = job["job_id"]
    cfg = job["evaluation_config"]
    timed = job["objective"] == "minimum_time"
    info = dict(rounds=[], reconstructed_final=False)
    if not (attempt/"nto_refinement.json").exists():
        audit.check(result["status"] != SUCCESS, where, "Success without refinement record")
        return info
    record = read(attempt/"nto_refinement.json")
    raw_path = np.load(attempt/"path.npy", allow_pickle=False)
    expected_controls = np.vstack([raw_path[0], raw_path, raw_path[-1]]) if timed else raw_path
    audit.check(result.get("frontend_success") is True, where, "Refinement started before shared seed acceptance")
    audit.check(record["final_validation_required"] and not record["geometry_model_changed"]
                and not record["controls_inserted"], where, "Frozen model or validation policy changed")
    audit.check(record["profile"]["name"] == PROFILE, where, "Wrong refinement profile")
    rows = record["rounds"]
    directories = sorted((attempt/"refinement").glob("round_*"))
    audit.check(0 <= len(rows) <= 4 and len(directories) == len(rows)
                and (len(rows)>0 or result["status"]=="timeout"), where, "Wrong solve/directory count")
    audit.check(record["initial_remaining_seconds"] <= min(60., cfg["optimizer_budget_seconds"])
                and (record["initial_remaining_seconds"]>0 or result["status"]=="timeout"),
                where, "Initial remaining budget reset or invalid")
    previous_locations = previous_added = previous_controls = previous_knots = None
    previous_remaining = record["initial_remaining_seconds"]
    final_native = final_raw = final_candidate = None
    accounted_seconds = 0.
    for i, row in enumerate(rows):
        target = attempt/"refinement"/f"round_{i:02d}"
        label = f"{where}/round_{i:02d}"
        audit.check(row["index"] == i and row["directory"] == str(target.relative_to(attempt)), label, "Round order changed")
        remaining = row["remaining_seconds"]
        audit.check(0 < remaining <= previous_remaining+1e-6, label, "A refinement round reset/increased the budget")
        previous_remaining = remaining
        locations = np.asarray(row["collocation_locations"], float)
        audit.check(149 <= len(locations) <= 600 and row["collocation_points"] == len(locations)
                    and np.isfinite(locations).all() and np.all(np.diff(locations)>0)
                    and np.all((locations>0)&(locations<1)), label, "Invalid collision point count/order")
        if i == 0:
            audit.close(locations, np.arange(1,150)/150, label, "Initial 149 collision points changed", atol=0, rtol=0)
        else:
            audit.close(locations, np.unique(np.r_[previous_locations, previous_added]), label,
                        "Refinement removed/changed points or added undeclared points", atol=0, rtol=0)
        previous_locations = locations
        previous_added = row.get("added_collocation_locations", [])
        entry = dict(index=i, collocation_points=len(locations), status=row["status"])
        info["rounds"].append(entry)
        final_native = final_raw = final_candidate = None
        params_path = target/"nto_parameters.json"
        if not params_path.exists():
            audit.check(result["status"] in {"timeout","infrastructure_error","solver_failed"}, label, "Completed round missing parameters")
            continue
        parameters = read(params_path)
        limit = check_profile(audit, parameters, job, label)
        audit.check(parameters["warm_start_node_count"]==len(raw_path)
                    and parameters["warm_start_control_count"]==len(expected_controls)
                    and parameters["warm_start_endpoint_copies"]==(2 if timed else 0)
                    and parameters["warm_start_resampled"] is False,label,"Warm-start metadata changed")
        audit.check(limit <= remaining+1e-6, label, "Solver budget exceeds remaining round budget")
        audit.close(parameters["collocation_locations"], locations, label, "Native bindings differ from manifest", atol=0, rtol=0)
        initial = arrays(target/"nto_initial.npz")
        audit.close(initial["original_path"], raw_path, label, "Native seed path changed", atol=0, rtol=0)
        audit.check(initial["controls"].shape == expected_controls.shape, label, "Control points inserted or pruned")
        if i == 0:
            audit.close(initial["controls"], expected_controls, label, "First warm start differs from raw DRM", atol=0, rtol=0)
            duration = float(np.linalg.norm(np.diff(raw_path,axis=0),axis=1).sum()+2) if timed else 1.
            order = 4 if timed else 2
            breaks = np.linspace(0.,duration,len(expected_controls)-order+2)
            audit.close(initial["knots"], np.r_[np.repeat(0.,order-1),breaks,np.repeat(duration,order-1)], label, "Initial knots changed")
            audit.close(initial["duration"], duration, label, "Initial duration changed")
            top_initial = arrays(attempt/"nto_initial.npz")
            audit.close(top_initial["controls"], expected_controls, label, "Top-level first initial controls changed", atol=0,rtol=0)
        elif previous_controls is not None:
            audit.close(initial["controls"], previous_controls, label, "Next solve did not start from previous candidate", atol=0,rtol=0)
            audit.close(initial["knots"], previous_knots, label, "Next solve changed previous knots", atol=0,rtol=0)
        native_path = target/"nto_solver_result.json"
        final_native = final_raw = final_candidate = None
        if not native_path.exists():
            audit.check(result["status"] != SUCCESS, label, "Success inherited from an interrupted last solver")
            continue
        final_native = read(native_path)
        entry.update(native_solver_success=final_native["solver_success"], native_solver_status=final_native["solver_status"])
        if final_native["solver_status"] == "exception":
            audit.check(result["status"] != SUCCESS, label, "Native exception inherited a successful result")
            continue
        final_raw = arrays(target/"nto_solver_raw.npz")
        audit.close(final_raw["normalized_knots"],initial["knots"]/float(initial["duration"]),
                    label,"Solver changed the fixed normalized knot basis",atol=1e-12,rtol=1e-12)
        audit.check(final_raw["controls"].shape==expected_controls.shape,label,"Solver changed the control count")
        previous_controls = final_raw["controls"]
        previous_knots = final_raw["normalized_knots"] * float(final_raw["duration"])
        if not (target/"candidate.json").exists():
            audit.check(result["status"] != SUCCESS, label, "Success missing last candidate")
            continue
        final_candidate = read(target/"candidate.json")
        saved = arrays(target/"candidate.npz")
        audit.close(saved["controls"], previous_controls, label, "Round candidate differs from raw controls", atol=0,rtol=0)
        audit.close(saved["knots"], previous_knots, label, "Round candidate differs from raw knots", atol=0,rtol=0)
        audit.check(final_candidate["solver_success"] == final_native["solver_success"], label, "Round candidate changed native success")
        try:
            spline = curve(final_candidate, previous_controls, previous_knots)
        except ValueError:
            audit.check(result["status"] != SUCCESS, label, "Malformed native values counted as success")
            continue
        if not (target/"refinement_geometry.json").exists():
            audit.check(result["status"] != SUCCESS, label, "Success missing geometry guard")
            continue
        guard = read(target/"refinement_geometry.json")
        geometry = arrays(target/"refinement_geometry.npz")
        reconstructed = spline(geometry["parameter"] * float(final_raw["duration"]))
        error = float(np.max(np.abs(reconstructed-geometry["q"])))
        audit.maximum_curve_error = max(audit.maximum_curve_error,error)
        audit.close(reconstructed, geometry["q"], label, "SciPy differs from guard samples", atol=1e-10,rtol=1e-10)
        accounted_seconds += row.get("solve_and_construction_seconds",0.)+guard["seconds"]
        if not np.isfinite(geometry["clearance"]).all():
            audit.check(result["status"] != SUCCESS and "exception" in guard, label, "Nonfinite completed guard")
            continue
        d = geometry["clearance"]
        audit.close(d.min(), guard["minimum_clearance_m"], label, "Guard minimum differs")
        audit.check(guard["passed"] == bool(np.all(d>=.0005)), label, "Guard success wrong")
        audit.check(guard["below_guard_samples"] == int(np.sum(d<.0005)), label, "Guard count wrong")
        expected = expected_additions(locations,geometry["parameter"],d)
        audit.close(row["proposed_collocation_locations"], expected, label, "Refinement policy selected different points", atol=0,rtol=0)
        if i+1<len(rows):
            audit.close(previous_added,expected,label,"Actual added points differ",atol=0,rtol=0)
        else:
            audit.check(len(previous_added)==0, label, "Last stopped round records unused points as actually added")
        entry.update(guard_passed=guard["passed"],minimum_clearance_m=float(d.min()),samples=len(d))
    info.update(native_solves=len(rows),stopping_reason=record["stopping_reason"])
    if "elapsed_seconds" in record:
        audit.check(accounted_seconds <= record["elapsed_seconds"]+1e-5, where, "Inner timings exceed refinement wall time")
        if result["status"] == SUCCESS:
            audit.check(record["elapsed_seconds"] <= record["initial_remaining_seconds"], where, "Success exceeded refinement budget")
    if not (attempt/"candidate.json").exists():
        audit.check(result["status"] != SUCCESS, where, "Success without common candidate")
        return info
    candidate, saved = read(attempt/"candidate.json"), arrays(attempt/"candidate.npz")
    audit.check(final_native is not None and final_raw is not None, where, "Final candidate inherited an older native result")
    if final_native is None or final_raw is None:
        return info
    audit.check(candidate["solver_success"] == final_native["solver_success"] and
                candidate["solver_status"] == final_native["solver_status"], where, "Final/native solver status changed")
    audit.close(saved["controls"],final_raw["controls"],where,"Final candidate inherited older controls",atol=0,rtol=0)
    audit.close(saved["knots"],final_raw["normalized_knots"]*float(final_raw["duration"]),where,"Final candidate inherited older knots",atol=0,rtol=0)
    audit.check(candidate["refinement_native_solves"] == len(rows), where, "Returned solve count differs")
    audit.check(candidate["profile"] == PROFILE and candidate["objective"] == job["objective"], where, "Final candidate objective/profile changed")
    if result["status"] == SUCCESS:
        audit.check(final_native["solver_success"] is True,where,"Failed solver falsely inherited success")
    try:
        spline = curve(candidate,saved["controls"],saved["knots"])
    except ValueError:
        audit.check(result["status"] != SUCCESS,where,"Malformed final candidate counted as success")
        return info
    audit.curves += 1
    info["reconstructed_final"] = True
    if not (attempt/"validation.json").exists():
        audit.check(result["status"] != SUCCESS,where,"Success without independent final validation")
        return info
    validation = read(attempt/"validation.json")
    samples = arrays(attempt/"trajectory_samples.npz")
    observed = spline(samples["time"])
    error=float(np.max(np.abs(observed-samples["q"])))
    audit.maximum_curve_error=max(audit.maximum_curve_error,error)
    audit.close(observed,samples["q"],where,"Independent SciPy curve differs from final samples",atol=1e-10,rtol=1e-10)
    lower,upper=saved["knots"][candidate["spline_order"]-1],saved["knots"][-candidate["spline_order"]]
    endpoint_error=float(np.linalg.norm(spline([lower,upper])-np.asarray([job["case"]["q_start"],job["case"]["q_goal"]]),axis=1).max())
    breaks=np.unique(saved["knots"][(saved["knots"]>=lower)&(saved["knots"]<=upper)])
    cost=float(upper-lower) if timed else float(np.linalg.norm(np.diff(spline(breaks),axis=0),axis=1).sum())
    audit.close(endpoint_error,validation["endpoint_error_rad"],where,"Endpoint error differs")
    audit.close(cost,validation["cost"],where,"Objective differs from independent reconstruction")
    audit.close(cost,final_native["solver_objective"],where,"Objective differs from native solver")
    disagreements=int(np.sum(samples["native"]!=samples["independent"]))
    audit.check(np.array_equal(samples["independent"],samples["clearance"]>0),where,"Saved independent collision labels changed")
    audit.check(disagreements==validation["model_disagreements"],where,"Checker disagreement count differs")
    if bounds is not None:
        position_pass=bool(np.all(saved["controls"]>=np.asarray(bounds["lower"])-cfg["feasibility_tolerance"])
                           and np.all(saved["controls"]<=np.asarray(bounds["upper"])+cfg["feasibility_tolerance"]))
        audit.check(position_pass==validation["joint_limits_passed"],where,"Position control limits differ from frozen model")
        if result["status"]==SUCCESS:
            audit.check(position_pass,where,"Successful curve violates canonical joint limits")
    if timed:
        for order,name in ((1,"velocity_per_joint"),(2,"acceleration_per_joint")):
            derivative=spline.derivative(order)
            bound=np.max(np.abs(derivative.c[:len(derivative.t)-derivative.k-1]),axis=0)
            audit.close(bound,validation["derivative_bounds"][name],where,f"Derivative control bound differs: {name}")
            if result["status"]==SUCCESS:
                limit=cfg["velocity_limit_rad_s"] if order==1 else cfg["acceleration_limit_rad_s2"]
                audit.check(np.max(bound)<=limit+cfg["feasibility_tolerance"],where,f"Successful derivative bound violated: {name}")
        audit.close(np.max(np.abs(spline.derivative(1)([lower,upper]))),validation["derivative_bounds"]["endpoint_velocity"],where,"Endpoint derivative differs")
        if result["status"]==SUCCESS:
            audit.check(np.max(np.abs(spline.derivative(1)([lower,upper])))<=cfg["feasibility_tolerance"],where,"Successful curve does not start/stop at rest")
    if result["status"] == SUCCESS:
        audit.check(validation["passed"] and validation["status"]==SUCCESS and
                    endpoint_error<=cfg["endpoint_tolerance_rad"] and
                    np.all(samples["native"]) and np.all(samples["independent"])
                    and np.all(samples["clearance"]>2e-6),where,"Final success violates saved numerical checks")
        audit.check(np.linalg.norm(np.diff(observed,axis=0),axis=1).max() <= cfg["validation_step_rad"]*(1+1e-9)+1e-12,
                    where,"Successful candidate sample step exceeds policy")
    if result["status"] not in {"timeout","infrastructure_error"}:
        expected_status=validation["status"] if final_native["solver_success"] else "solver_failed"
        audit.check(result["status"]==expected_status,where,"Final classification does not follow last solver and validator")
    info.update(cost=cost,endpoint_error_rad=endpoint_error,maximum_curve_error_rad=error,
                validation_status=validation["status"],checker_disagreements=disagreements,
                minimum_clearance_m=float(samples["clearance"].min()),
                native_solver_success=final_native["solver_success"],native_solver_status=final_native["solver_status"])
    return info


def check_shared_groups(audit, groups):
    output=[]
    for key, records in sorted(groups.items()):
        identities={(r["method"],r["objective"]) for r in records}
        audit.check(len(identities)==len(records),key,"Duplicate case/method/objective/roadmap")
        audit.check(identities==METHOD_PAIRS,key,"Incomplete five-method pairing")
        audit.check(len({r["input_digest"] for r in records})==1,key,"Methods received different endpoints/model/scenes")
        accepted={r["path_sha256"] for r in records if r["frontend_accepted"]}
        audit.check(None not in accepted and len(accepted)<=1,key,"Methods changed accepted DRM seed path")
        output.append(dict(key=key,accepted_path_hashes=sorted(x for x in accepted if x),entries=records))
    return output


def load_run(audit, run, method=None):
    """Verify every receipt, including failed/retried historical attempts."""
    manifest=read(run/"run_manifest.json")
    audit.check(read(run/"identity.json")["run_manifest_sha256"]==sha(run/"run_manifest.json"),run,"Run manifest identity changed")
    audit.check(manifest["split"] in {"smoke","dev"},run,"Formal test data is forbidden in this audit")
    for relative,expected in manifest["source_hashes"].items():
        p=run/"source_snapshot"/relative
        audit.check(p.is_file() and sha(p)==expected,p,"Source snapshot hash differs")
    loaded=[]
    seen=set()
    for job in manifest["jobs"]:
        if method and job["method"]!=method:
            continue
        jid=job["job_id"]
        audit.check(jid not in seen,run,"Repeated job ID")
        seen.add(jid)
        attempts=sorted((run/"jobs"/jid/"attempts").glob("[0-9][0-9][0-9][0-9]"))
        if not attempts or not (attempts[-1]/"receipt.json").exists():
            audit.check(False,jid,"Latest attempt incomplete; run audit only after study completion")
            continue
        for attempt in attempts:
            audit.check((attempt/"receipt.json").exists(),attempt,"Unsealed historical attempt")
            if (attempt/"receipt.json").exists():
                result=audit.receipt(attempt)
        final=attempts[-1]
        result=read(final/"result.json")
        audit.check(result["status"] in TERMINAL,jid,"Unknown final status")
        audit.check(not read(final/"receipt.json").get("retry_pending"),jid,"Final attempt still retry-pending")
        case=read(final/"input.json")
        audit.check({k:v for k,v in case.items() if k!="_objective"}==job["case"],jid,"Saved task differs from frozen job")
        audit.check(case.get("_objective",job["objective"])==job["objective"],jid,"Saved objective changed")
        for name,expected in (("problem_id",job["case_id"]),("scene_id",job["scene_id"]),
                              ("method",job["method"]),("objective",job["objective"]),
                              ("roadmap_id",job["roadmap_id"]),("method_seed",job["method_seed"])):
            audit.check(result.get(name)==expected,jid,f"Final result identity changed: {name}")
        process=read(final/"process.json")
        audit.check(process["timeout_seconds"]==60.==manifest["timeout_seconds"],jid,"External watchdog differs from 60s")
        audit.check(not process.get("timed_out") or result["status"]=="timeout",jid,"Watchdog expiry counted as another status")
        if result["status"]==SUCCESS:
            audit.check(process["cold_e2e_seconds"]<=60.+.05,jid,"Successful cold execution exceeded timeout")
        loaded.append((job,final,result))
    return manifest,loaded


def audit_study(index_path, old_dev):
    audit=Audit()
    index=read(index_path)
    freeze_path=Path(index_path).with_name("execution_freeze.json")
    freeze=read(freeze_path) if freeze_path.exists() else None
    if freeze:
        audit.check(index["slots"]==freeze["planned_registered_slots"],"study","Index slot count differs from execution freeze")
        audit.check({b["run"] for b in index["blocks"]}=={b["run"] for b in freeze["blocks"]},"study","Indexed blocks differ from execution freeze")
    records=[]
    groups=defaultdict(list)
    seen=set()
    planned=0
    for block in index["blocks"]:
        run=Path(block["run"])
        audit.check(sha(run/"run_manifest.json")==block["run_manifest_sha256"],run,"Indexed manifest SHA differs")
        manifest,loaded=load_run(audit,run)
        bounds=read(run/"source_snapshot/benchmarks/generic_motion_v1/configs/robot.json")
        if freeze:
            audit.check(manifest["source_hashes"]==freeze["runtime_source_hashes"] and
                        manifest["source_code_commit"]==freeze["source_commit"],run,"Run source differs from study freeze")
        planned+=len(manifest["jobs"])
        audit.check(len(loaded)==block["planned_jobs"],run,"Block not complete")
        for job,attempt,result in loaded:
            if freeze:
                audit.check(job["evaluation_config"]==freeze["evaluation_config"],job["job_id"],"Job configuration differs from frozen study")
            audit.check(job["case"]["robot_sha256"]==bounds["robot_sha256"] and
                        job["case"]["collision_geometry_sha256"]==bounds["collision_geometry_sha256"],
                        job["job_id"],"Case model hashes differ from frozen canonical model")
            key=(manifest["split"],job["case_id"],job["roadmap_id"],job["method"],job["objective"])
            audit.check(key not in seen,job["job_id"],"Repeated logical slot across blocks")
            seen.add(key)
            row=dict(run=str(run),attempt=str(attempt),job_id=job["job_id"],
                     split=manifest["split"],case_id=job["case_id"],scene_id=job["scene_id"],
                     family=job["family"],roadmap_id=job["roadmap_id"],method=job["method"],
                     objective=job["objective"],method_seed=job["method_seed"],status=result["status"],
                     blocked=bool(job["case"]["straight_line_collision"]),
                     frontend_accepted=bool(result.get("frontend_success")),
                     input_digest=digest(job["case"]),
                     path_sha256=sha(attempt/"path.npy") if (attempt/"path.npy").exists() else None)
            groups[(row["split"],row["case_id"],row["roadmap_id"])].append(row)
            if job["method"]=="NTO_REFINED":
                try:
                    row["refinement_audit"]=check_refined_attempt(audit,attempt,job,result,bounds=bounds)
                except Exception as exc:
                    audit.check(False,job["job_id"],f"Malformed/incomplete NTO artifacts: {type(exc).__name__}: {exc}")
                records.append(row)
    audit.check(planned==index["slots"] and len(seen)==index["slots"],"study","Declared slot denominator differs")
    shared=check_shared_groups(audit,groups)
    old_manifest,old_loaded=load_run(audit,Path(old_dev),method="NTO_PAPER")
    audit.check(old_manifest["split"]=="dev","old run","Historical comparison must be development only")
    old={}
    for job,attempt,result in old_loaded:
        key=(job["case_id"],job["roadmap_id"],job["objective"])
        audit.check(key not in old,"historical","Repeated old logical NTO slot")
        old[key]=(job,attempt,result)
    transitions=Counter()
    paired=[]
    for row in records:
        if row["split"]!="dev":
            continue
        key=(row["case_id"],row["roadmap_id"],row["objective"])
        audit.check(key in old,row["job_id"],"Missing historical dev pair")
        if key not in old:
            continue
        job,attempt,result=old[key]
        audit.check(row["input_digest"]==digest(job["case"]) and row["method_seed"]==job["method_seed"],row["job_id"],"Old/new paired input or method seed differs")
        old_hash=sha(attempt/"path.npy") if (attempt/"path.npy").exists() else None
        same_path=old_hash==row["path_sha256"] if old_hash and row["path_sha256"] else None
        entry=dict(case_id=row["case_id"],roadmap_id=row["roadmap_id"],objective=row["objective"],
                   blocked=row["blocked"],old_status=result["status"],new_status=row["status"],
                   old_profile="NTO_PAPER",new_profile="NTO_REFINED",old_attempt=str(attempt),
                   new_attempt=row["attempt"],same_saved_path=same_path,
                   old_frontend_accepted=bool(result.get("frontend_success")),
                   new_frontend_accepted=row["frontend_accepted"])
        if same_path is False and entry["old_frontend_accepted"] and entry["new_frontend_accepted"]:
            audit.notes.append(dict(case_id=row["case_id"],roadmap_id=row["roadmap_id"],message="Old/new accepted paths differ; retain paired query but do not attribute change solely to NTO refinement"))
        paired.append(entry)
        transitions[(row["objective"],row["blocked"],result["status"],row["status"])]+=1
    summary=Counter((r["split"],r["objective"],r["blocked"],r["status"]) for r in records)
    return clean(dict(passed=not audit.issues,errors=audit.issues,issues=audit.issues,notes=audit.notes,
        audit="Independent NumPy/SciPy sealed artifact audit; no solver, Drake or GPU calls",
        created_unix_seconds=time.time(),script_sha256=sha(__file__),index_sha256=sha(index_path),
        source_index=str(index_path),historical_dev=str(old_dev),planned_new_slots=planned,
        new_nto_slots=len(records),paired_dev_nto_slots=len(paired),receipts_verified=audit.receipts,
        receipt_files_verified=audit.files,reconstructed_final_candidates=audit.curves,
        maximum_scipy_sample_error_rad=audit.maximum_curve_error,
        environment=dict(python=sys.version,numpy=np.__version__,scipy=scipy.__version__),
        new_status_counts=[dict(split=s,objective=o,blocked=b,status=t,count=n) for (s,o,b,t),n in sorted(summary.items())],
        paired_transitions=[dict(objective=o,blocked=b,old_status=u,new_status=v,count=n) for (o,b,u,v),n in sorted(transitions.items())],
        records=records,paired_dev=paired,shared_method_checks=shared,
        limitations=["Historical NTO_PAPER and new NTO_REFINED are distinct parameter/source protocols, never pooled.",
                     "Old/new frontend implementations also differ; changed accepted paths are separately disclosed.",
                     "This audits existing collision arrays, not fresh geometry or continuous collision freedom.",
                     "Repeated roadmap seeds are paired queries, not independent random scenes.",
                     "All failures and unresolved candidates remain in the denominator; no best-of profile selection."]))


def markdown(report):
    lines=["# NTO_REFINED 独立结果审计", "",
           f"审计结论：{'通过' if report['passed'] else '未通过'}；错误 {len(report['errors'])} 项。",
           f"新研究 {report['planned_new_slots']} 个方法槽位；NTO {report['new_nto_slots']} 个；与旧dev配对 {report['paired_dev_nto_slots']} 个。",
           f"共核对 {report['receipts_verified']} 份新旧receipt、{report['receipt_files_verified']} 个封存文件。SciPy独立重建 {report['reconstructed_final_candidates']} 条最终曲线，最大采样差 {report['maximum_scipy_sample_error_rad']:.3g} rad。", "",
           "本脚本独立检查四轮/600点上限、固定1mm裕度、原生solver选项、轮间初值、共享预算、全部最终状态、最后一轮原生成功标志以及公共验证记录。按所有方法核对固定输入与已接受DRM路径；未调用Drake、GPU、碰撞查询或求解器。", "",
           "## 新配置状态计数", "", "| split | 目标 | 原始直连碰撞 | 最终状态 | 数量 |", "|---|---|---|---|---:|"]
    lines += [f"| {r['split']} | {r['objective']} | {r['blocked']} | {r['status']} | {r['count']} |" for r in report['new_status_counts']]
    lines += ["", "## 旧/新dev状态转移", "", "旧NTO_PAPER与新NTO_REFINED是不同参数/源码协议，下面仅报告同case/roadmap配对转移，不混合成绩，也不逐题择优。原始直连碰撞标签不等于已证明不可行。", "",
              "| 目标 | 原始直连碰撞 | 旧状态 | 新状态 | 数量 |", "|---|---|---|---|---:|"]
    lines += [f"| {r['objective']} | {r['blocked']} | {r['old_status']} | {r['new_status']} | {r['count']} |" for r in report['paired_transitions']]
    changes=[p for p in report['paired_dev'] if p['same_saved_path'] is False and p['old_frontend_accepted'] and p['new_frontend_accepted']]
    lines += ["", f"新旧均已接受但保存路径不同的配对：{len(changes)}；这些任务不能把所有变化单独归因于NTO细化。详细case、seed、路径和失败记录见同名JSON。", "",
              "重复roadmap seed是配对查询，不是独立场景。碰撞数组核查和有限分辨率数值验证不构成连续安全证明。后端异常、超时和未决均保留在分母中。"]
    if report['errors']:
        lines += ["", "## 审计错误", ""]+[f"- `{e['location']}`：{e['message']}" for e in report['errors']]
    return "\n".join(lines)+"\n"


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index",type=Path,required=True)
    parser.add_argument("--old-dev",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    for block in read(args.index)["blocks"]:
        if Path(block["run"]).resolve() in args.output.resolve().parents:
            parser.error("Write audit outputs outside sealed runs")
    result=audit_study(args.index,args.old_dev)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+"\n")
    args.output.with_suffix(".md").write_text(markdown(result))
    print(json.dumps({k:result[k] for k in ("passed","planned_new_slots","new_nto_slots","paired_dev_nto_slots",
        "receipts_verified","receipt_files_verified","reconstructed_final_candidates","maximum_scipy_sample_error_rad","issues","notes")},indent=2))
    return 0 if result["passed"] else 1


if __name__=="__main__":
    raise SystemExit(main())
