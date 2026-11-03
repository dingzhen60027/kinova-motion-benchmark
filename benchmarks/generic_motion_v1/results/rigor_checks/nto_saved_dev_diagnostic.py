"""Read sealed DEV NTO returns; reevaluate collocation and saved witnesses on CPU.

No optimizer, CUDA collision call, parameter search, or historical write occurs.
Run from repo root with local/python.sh. Output must be a fresh directory.
"""
import argparse
from collections import Counter
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import write
from benchmarks.generic_motion_v1.runners.nto_collision_constraint import RobustMinimumDistanceConstraint
from benchmarks.generic_motion_v1.runners.results import collect, _selected_rows


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sealed(attempt, name):
    receipt = json.loads((attempt / "receipt.json").read_text())
    path = attempt / name
    if sha(path) != receipt["files"][name]:
        raise ValueError(f"Sealed artifact mismatch: {path}")
    return path


def read(attempt, name):
    return json.loads(sealed(attempt, name).read_text())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, default=ROOT / "benchmarks/generic_motion_v1/results/baseline_dev")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    report = dict(scope="official results.collect/_selected_rows first terminal attempt and saved-output acceptance in the current DEV run",
                  run=str(args.run.resolve()), script_sha256=sha(Path(__file__)),
                  optimizer_calls=0, cuda_calls=0, historical_writes=0,
                  collision_selection="Every official-selected NTO_PAPER DEV validation failure with raw collision_failed status",
                  status="running", states=[], unusable_raw_results=[], collision_cases=[])
    manifest = json.loads((args.run / "run_manifest.json").read_text())
    if Path(manifest["manifest_path"]).name != "dev.jsonl":
        raise ValueError("This diagnostic is DEV only")
    report["run_manifest_sha256"] = sha(args.run / "run_manifest.json")
    report["drake_version"] = importlib.metadata.version("drake")
    report["runtime_source_sha256"] = {name: sha(ROOT / name) for name in (
        "benchmarks/generic_motion_v1/model.py", "benchmarks/generic_motion_v1/runners/nto_collision_constraint.py",
        "benchmarks/generic_motion_v1/runners/results.py", "benchmarks/generic_motion_v1/outcomes.py")}
    all_rows, _ = collect([args.run.resolve()])
    official, _ = _selected_rows(all_rows, dict(id="nto_saved_dev_diagnostic", runs=[dict(
        path=str(args.run.resolve()), methods=["NTO", "NTO_PAPER", "NTO_REFINED"])]))
    report["all_run_attempt_lifecycles"] = dict(Counter(r["lifecycle"] for r in all_rows))
    report["official_secondary_accepted_outputs"] = [dict(
        attempt=r["attempt_path"], problem_id=r["problem_id"], method=r["method"], objective=r["objective"],
        raw_status=r["raw_status"], status=r["status"], status_basis=r["status_basis"],
        solver_reported_success=r["solver_reported_success"], validation_passed=r["validation_passed"])
        for r in official if r["raw_status"] != "success_validated_numerical" and r["status"] == "success_numerical"]
    states, raw_states, native_statuses, raws, selected = Counter(), Counter(), Counter(), 0, []
    for normalized in sorted(official, key=lambda r: r["attempt_path"]):
        a = ROOT / normalized["attempt_path"]
        r = read(a, "result.json")
        states[r["method"], r["objective"], normalized["status"]] += 1
        raw_states[r["method"], r["objective"], r["status"]] += 1
        for raw_path in a.glob("**/nto_solver_raw.npz"):
            sealed(a, str(raw_path.relative_to(a)))
            with np.load(raw_path, allow_pickle=False) as raw:
                if (any(not np.isfinite(raw[k]).all() for k in ("controls", "duration", "normalized_knots"))
                        or float(raw["duration"]) <= 0):
                    report["unusable_raw_results"].append(str(raw_path))
            raws += 1
        if r["method"] == "NTO_PAPER" and (a / "nto_solver_result.json").is_file():
            native = read(a, "nto_solver_result.json")
            d = native.get("native_solver_details", {})
            native_statuses[r["objective"], r["status"], native["solver_status"], str(d.get("info", d.get("status")))] += 1
        if (r["method"] == "NTO_PAPER" and r["status"] == "collision_failed"
                and normalized["status"] == "validation_failed"):
            selected.append((a, r))
    report.update(raw_files_checked=raws,
                  states=[dict(method=m, objective=o, status=s, count=n) for (m,o,s),n in sorted(states.items())],
                  raw_states=[dict(method=m, objective=o, status=s, count=n) for (m,o,s),n in sorted(raw_states.items())],
                  native_statuses=[dict(objective=o, final_status=f, solver_status=s, code=c, count=n)
                                   for (o,f,s,c),n in sorted(native_statuses.items())])
    write(args.out / "summary.json", report)
    from pydrake.all import BsplineBasis, BsplineTrajectory
    began = time.perf_counter()
    for index, (a, result) in enumerate(selected):
        target = args.out / f"case_{index:03d}"
        target.mkdir()
        row = dict(attempt=str(a.resolve()), problem_id=result["problem_id"],
                   objective=result["objective"], roadmap_id=result["roadmap_id"], status="running")
        report["collision_cases"].append(row)
        try:
            case = read(a, "input.json")
            parameters = read(a, "nto_parameters.json")
            native_status = read(a, "nto_solver_result.json")
            profile = parameters["profile"]
            with np.load(sealed(a, "nto_solver_raw.npz"), allow_pickle=False) as raw:
                duration = float(raw["duration"])
                curve = BsplineTrajectory(BsplineBasis(parameters["spline_order"], raw["normalized_knots"] * duration),
                                          [q[:, None] for q in raw["controls"]])
            locations = np.asarray(parameters["collocation_locations"])
            q = curve.vector_values(locations * duration).T
            model = Model(case["voxels_xyz"], case["voxel_radius_m"])
            if any(getattr(model, k) != case[k] for k in ("robot_sha256", "collision_geometry_sha256")):
                raise ValueError("Model identity differs from sealed input")
            backend = RobustMinimumDistanceConstraint(model.nto_plant(), profile["clearance_m"],
                                                       profile["influence_distance_m"], target)
            values = np.asarray([float(backend.evaluate(x)[0]) for x in q])
            clearance = model.clearance(q)
            lower, upper = backend.original.lower_bound()[0], backend.original.upper_bound()[0]
            residual = np.maximum(np.maximum(lower - values, values - upper), 0.)
            with np.load(sealed(a, "trajectory_samples.npz"), allow_pickle=False) as dense:
                saved_q, saved_t, saved_clearance = dense["q"], dense["time"], dense["clearance"]
                witness_index = int(np.argmin(saved_clearance))
                witness = saved_q[witness_index]
                witness_parameter = float(saved_t[witness_index] / duration)
                reconstruction_error = float(np.max(np.abs(curve.vector_values(saved_t).T - saved_q)))
                witness_recomputed = float(model.clearance(witness[None, :])[0])
                saved_minimum = float(saved_clearance[witness_index])
                dense_count = len(saved_q)
            np.savez_compressed(target / "cpu_values.npz", collocation_parameter=locations,
                                collocation_q=q, constraint_value=values, constraint_violation=residual,
                                collocation_clearance_m=clearance, witness_q=witness,
                                witness_parameter=witness_parameter, witness_clearance_m=witness_recomputed)
            row.update(status="completed", native_solver_success=native_status["solver_success"],
                       collocation_count=len(locations), constraint_upper_bound=float(upper),
                       maximum_constraint_violation=float(residual.max()),
                       configured_solver_tolerance=profile["solver_feasibility_tolerance"],
                       collocation_within_configured_tolerance=bool(np.all(residual <= profile["solver_feasibility_tolerance"])),
                       minimum_collocation_clearance_m=float(clearance.min()),
                       saved_dense_sample_count=dense_count, saved_minimum_dense_clearance_m=saved_minimum,
                       recomputed_worst_dense_clearance_m=witness_recomputed,
                       worst_dense_parameter=witness_parameter,
                       distance_to_nearest_collocation=float(np.min(np.abs(locations-witness_parameter))),
                       saved_curve_reconstruction_max_error_rad=reconstruction_error,
                       known_boxbox_recovery_events=len(backend.events), evidence=str(target.relative_to(args.out)))
        except Exception as exc:
            row.update(status="error", exception=repr(exc))
        write(args.out / "summary.json", report)
        print(json.dumps(row), flush=True)
    report["collocation_summary"] = []
    for objective in ("minimum_distance", "minimum_time"):
        rows = [r for r in report["collision_cases"] if r["objective"] == objective and r["status"] == "completed"]
        report["collocation_summary"].append(dict(objective=objective, cases=len(rows),
            within_configured_tolerance=sum(r["collocation_within_configured_tolerance"] for r in rows),
            maximum_constraint_violation=max((r["maximum_constraint_violation"] for r in rows), default=None),
            worst_dense_clearance_range_m=[min((r["recomputed_worst_dense_clearance_m"] for r in rows), default=None),
                                           max((r["recomputed_worst_dense_clearance_m"] for r in rows), default=None)],
            maximum_reconstruction_error_rad=max((r["saved_curve_reconstruction_max_error_rad"] for r in rows), default=None)))
    report.update(status="completed", seconds=time.perf_counter()-began,
                  limitation="Finite numerical reevaluation; no continuous collision certificate or claim about original FR3 experiments")
    write(args.out / "summary.json", report)


if __name__ == "__main__":
    main()
