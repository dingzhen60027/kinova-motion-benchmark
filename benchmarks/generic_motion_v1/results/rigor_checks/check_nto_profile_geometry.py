"""Recheck sealed NTO validation failures using CPU geometry only.

The selected cohort is every first-terminal ``validation_failed`` return for
the requested methods in one saved DEV or TEST run. This includes contact-band
``validation_undetermined`` returns. No optimization, CUDA collision call,
parameter selection, or historical artifact modification is performed.

Each returned curve is evaluated at its own saved collision collocation
locations. The saved densest validation witness is independently reevaluated;
the complete saved sample array is checked against the reconstructed curve,
but its collision distances are not all recomputed. This is a finite numerical
audit, not continuous certification. One JSON and one combined NPZ retain the
results, including errors and contact ambiguity.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import importlib.metadata
import json
from pathlib import Path
import shlex
import sys
import time
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.model import Model, digest
from benchmarks.generic_motion_v1.roadmap import write
from benchmarks.generic_motion_v1.runners.nto_collision_constraint import RobustMinimumDistanceConstraint
from benchmarks.generic_motion_v1.runners.results import collect, _selected_rows

METHODS = ("NTO", "NTO_PAPER", "NTO_REFINED")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class SealedAttempt:
    """Check each file against its original receipt before consuming it."""

    def __init__(self, path):
        self.path = path.resolve()
        self.receipt = json.loads((self.path / "receipt.json").read_text())
        self.checked = {}

    def file(self, name):
        path = (self.path / name).resolve()
        if not path.is_relative_to(self.path):
            raise ValueError("Artifact reference escapes its attempt")
        expected = self.receipt["files"][name]
        actual = sha(path)
        if actual != expected:
            raise ValueError(f"Sealed artifact SHA256 mismatch: {path}")
        self.checked[name] = actual
        return path

    def json(self, name):
        return json.loads(self.file(name).read_text())


def saved_profile(method, parameters, evaluation):
    """Never substitute current method defaults for saved solve settings."""
    if "profile" in parameters:
        profile = parameters["profile"]
        answer = dict(clearance_m=profile["clearance_m"],
                      influence_distance_m=profile["influence_distance_m"],
                      solver_tolerance=profile["solver_feasibility_tolerance"],
                      source="sealed nto_parameters.json/profile")
    elif method == "NTO":
        # Legacy parameters record clearance and native solver options directly;
        # influence distance was saved in evaluation.json, not duplicated there.
        answer = dict(clearance_m=parameters["clearance_m"],
                      influence_distance_m=evaluation["nto"]["influence_distance_m"],
                      solver_tolerance=parameters["parameters"]["tol"],
                      source="sealed nto_parameters.json and evaluation.json/nto/influence_distance_m")
    else:
        raise ValueError("Published/refined return has no saved profile")
    values = np.asarray([answer[k] for k in ("clearance_m", "influence_distance_m", "solver_tolerance")], float)
    if not np.isfinite(values).all() or values[0] < 0 or np.any(values[1:] <= 0):
        raise ValueError("Malformed saved collision profile")
    return answer


def recheck(row, frozen, cache, arrays, index):
    from pydrake.all import BsplineBasis, BsplineTrajectory
    artifact = None
    result = dict(index=index, array_prefix=f"case_{index:04d}",
                  attempt=row["attempt_path"], record_id=row["record_id"],
                  problem_id=row["problem_id"], method=row["method"],
                  objective=row["objective"], raw_status=row["raw_status"],
                  normalized_status=row["status"], status="running")
    try:
        artifact = SealedAttempt(ROOT / row["attempt_path"])
        result["receipt_sha256"] = sha(artifact.path / "receipt.json")
        if row["receipt_integrity"] is not True:
            raise ValueError("Official record does not have a verified receipt")
        case = artifact.json("input.json")
        original = dict(case)
        objective = original.pop("_objective")
        if objective != row["objective"] or original != frozen[row["problem_id"]]:
            raise ValueError("Saved input differs from its hash-bound original manifest record")
        result["frozen_input_equal"] = True
        parameters = artifact.json("nto_parameters.json")
        evaluation = artifact.json("evaluation.json")
        native_status = artifact.json("nto_solver_result.json")
        validation = artifact.json("validation.json")
        candidate = artifact.json("candidate.json")
        profile = saved_profile(row["method"], parameters, evaluation)
        result["saved_profile"] = profile
        with np.load(artifact.file("nto_solver_raw.npz"), allow_pickle=False) as raw:
            controls = raw["controls"].copy()
            duration = float(raw["duration"])
            normalized_knots = raw["normalized_knots"].copy()
        if (controls.ndim != 2 or controls.shape[1] != 7 or not np.isfinite(controls).all()
                or not np.isfinite(normalized_knots).all() or not np.isfinite(duration) or duration <= 0):
            raise ValueError("Saved optimizer curve has malformed or nonfinite variables")
        order = candidate["spline_order"]
        if order != parameters.get("spline_order", 4 if objective == "minimum_time" else 2):
            raise ValueError("Candidate and native parameters have different spline order")
        knots = normalized_knots * duration
        with np.load(artifact.file("candidate.npz"), allow_pickle=False) as output:
            if not np.array_equal(output["controls"], controls) or not np.array_equal(output["knots"], knots):
                raise ValueError("Published candidate differs from saved native solve")
        curve = BsplineTrajectory(BsplineBasis(order, knots), [q[:, None] for q in controls])
        locations = np.asarray(parameters["collocation_locations"], float)
        if (locations.ndim != 1 or not len(locations) or not np.isfinite(locations).all()
                or np.any(locations <= 0) or np.any(locations >= 1) or np.any(np.diff(locations) <= 0)):
            raise ValueError("Malformed saved collocation locations")
        q = curve.vector_values(locations * duration).T
        key = digest(dict(voxels=case["voxels_xyz"], radius=case["voxel_radius_m"]))
        if key not in cache:
            model = Model(case["voxels_xyz"], case["voxel_radius_m"])
            # The added-sphere model asserts exact collision pair identities.
            cache[key] = model, model.nto_plant()
        model, native = cache[key]
        for name in ("robot_sha256", "collision_geometry_sha256"):
            if getattr(model, name) != case[name]:
                raise ValueError(f"Current canonical model differs from sealed input: {name}")
        result["canonical_model_hashes_equal"] = True
        if parameters["collision_pairs"] != native.collision_pair_count:
            raise ValueError("Rebuilt native collision pair count differs from saved solve")
        backend = RobustMinimumDistanceConstraint(native, profile["clearance_m"], profile["influence_distance_m"])
        values = np.asarray([float(backend.evaluate(x)[0]) for x in q])
        clearance = model.clearance(q)
        lower, upper = float(backend.original.lower_bound()[0]), float(backend.original.upper_bound()[0])
        residual = np.maximum(np.maximum(lower - values, values - upper), 0.)
        with np.load(artifact.file("trajectory_samples.npz"), allow_pickle=False) as dense:
            saved_q, saved_t, saved_clearance = dense["q"], dense["time"], dense["clearance"]
            if (saved_q.ndim != 2 or saved_q.shape[1] != 7 or len(saved_q) == 0
                    or saved_t.shape != (len(saved_q),) or saved_clearance.shape != (len(saved_q),)
                    or not all(np.isfinite(a).all() for a in (saved_q, saved_t, saved_clearance))):
                raise ValueError("Malformed saved validation arrays")
            witness_index = int(np.argmin(saved_clearance))
            witness = saved_q[witness_index].copy()
            witness_parameter = float(saved_t[witness_index] / duration)
            reconstruction_error = float(np.max(np.abs(curve.vector_values(saved_t).T - saved_q)))
            if reconstruction_error > 1e-12:
                raise ValueError("Saved dense trajectory differs from reconstructed candidate")
            witness_recomputed = float(model.clearance(witness[None, :])[0])
            saved_minimum = float(saved_clearance[witness_index])
            dense_count = len(saved_q)
        if not all(np.isfinite(a).all() for a in (values, clearance, residual, [witness_recomputed])):
            raise ValueError("Nonfinite CPU collision reevaluation")
        contact_band = model.contact_band_m
        prefix = result["array_prefix"]
        for name, value in dict(controls=controls, knots=knots,
                                collocation_parameter=locations, collocation_q=q,
                                constraint_value=values, constraint_violation=residual,
                                collocation_clearance_m=clearance, witness_q=witness,
                                witness_parameter=witness_parameter,
                                witness_clearance_m=witness_recomputed).items():
            arrays[f"{prefix}_{name}"] = value
        result.update(status="completed", native_solver_success=native_status["solver_success"],
                      final_validation_status=validation["status"],
                      collocation_count=len(locations), constraint_upper_bound=upper,
                      maximum_constraint_violation=float(residual.max()),
                      collocation_within_configured_tolerance=bool(np.all(residual <= profile["solver_tolerance"])),
                      minimum_collocation_clearance_m=float(clearance.min()),
                      saved_dense_sample_count=dense_count, saved_minimum_dense_clearance_m=saved_minimum,
                      recomputed_worst_dense_clearance_m=witness_recomputed,
                      worst_dense_clearance_recheck_error_m=abs(witness_recomputed-saved_minimum),
                      worst_witness_in_contact_band=bool(abs(witness_recomputed) <= contact_band),
                      worst_witness_penetrates_outside_contact_band=bool(witness_recomputed < -contact_band),
                      contact_band_m=contact_band, worst_dense_parameter=witness_parameter,
                      distance_to_nearest_collocation=float(np.min(np.abs(locations-witness_parameter))),
                      saved_curve_reconstruction_max_error_rad=reconstruction_error,
                      known_boxbox_recovery_events=backend.events)
    except Exception as exc:
        result.update(status="error", exception=repr(exc), traceback=traceback.format_exc())
    result["checked_artifact_sha256"] = {} if artifact is None else artifact.checked
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--methods", choices=METHODS, nargs="+", default=list(METHODS))
    parser.add_argument("--output", type=Path, required=True, help="New directory; never overwrite existing results")
    args = parser.parse_args()
    if len(set(args.methods)) != len(args.methods):
        parser.error("--methods must be distinct")
    run = args.run.resolve()
    if args.output.resolve().is_relative_to(run):
        parser.error("--output must be outside the saved run being audited")
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(status="running", command=shlex.join([sys.executable, *sys.argv]),
                  run=str(run), methods=args.methods, script_sha256=sha(__file__),
                  selection="Every first-terminal normalized validation_failed in this saved run, including contact-band failures",
                  optimizer_calls=0, cuda_calls=0, historical_writes=0,
                  drake_version=importlib.metadata.version("drake"), cases=[])
    report["runtime_source_sha256"] = {name: sha(ROOT / name) for name in (
        "benchmarks/generic_motion_v1/model.py", "benchmarks/generic_motion_v1/runners/nto_collision_constraint.py",
        "benchmarks/generic_motion_v1/runners/results.py", "benchmarks/generic_motion_v1/outcomes.py")}
    arrays, cache = {}, {}
    began = time.perf_counter()
    exit_code = 0
    try:
        manifest = json.loads((run / "run_manifest.json").read_text())
        original_path = Path(manifest["manifest_path"])
        # Old receipts contain an absolute path. Relocation still requires the
        # exact frozen manifest bytes, never a newly generated substitute.
        if not original_path.is_file():
            original_path = ROOT / "benchmarks/generic_motion_v1/manifests" / original_path.name
        if original_path.name not in ("dev.jsonl", "test.jsonl"):
            raise ValueError("This audit accepts saved DEV or TEST runs only")
        if sha(original_path) != manifest["manifest_sha256"]:
            raise ValueError("Frozen input manifest SHA256 differs from the saved run")
        frozen_rows = [json.loads(line) for line in original_path.read_text().splitlines() if line.strip()]
        frozen = {row["problem_id"]: row for row in frozen_rows}
        if len(frozen) != len(frozen_rows):
            raise ValueError("Duplicate original problem IDs")
        report.update(run_manifest_sha256=sha(run / "run_manifest.json"),
                      input_manifest_sha256=sha(original_path), input_manifest=str(original_path))
        all_rows, _ = collect([run])
        official, _ = _selected_rows(all_rows, dict(id="nto_profile_geometry", runs=[dict(
            path=str(run), methods=args.methods)]))
        selected = sorted((r for r in official if r["status"] == "validation_failed"),
                          key=lambda r: (r["method"], r["objective"], r["attempt_path"]))
        report.update(official_rows=len(official), selected_cases=len(selected),
                      official_status_counts=[dict(method=m, objective=o, status=s, count=n)
                          for (m, o, s), n in sorted(Counter(
                              (r["method"], r["objective"], r["status"]) for r in official).items())])
        write(args.output / "summary.json", report)
        for index, row in enumerate(selected):
            checked = recheck(row, frozen, cache, arrays, index)
            report["cases"].append(checked)
            write(args.output / "summary.json", report)
            print(json.dumps({k: checked[k] for k in ("index", "problem_id", "method", "objective", "status")}), flush=True)
        report["groups"] = []
        for method in args.methods:
            for objective in ("minimum_distance", "minimum_time"):
                rows = [r for r in report["cases"] if r["method"] == method and r["objective"] == objective]
                completed = [r for r in rows if r["status"] == "completed"]
                report["groups"].append(dict(method=method, objective=objective,
                    cases=len(rows), completed=len(completed), errors=len(rows)-len(completed),
                    within_configured_tolerance=sum(r["collocation_within_configured_tolerance"] for r in completed),
                    contact_band_witnesses=sum(r["worst_witness_in_contact_band"] for r in completed),
                    penetrating_witnesses=sum(r["worst_witness_penetrates_outside_contact_band"] for r in completed),
                    maximum_constraint_violation=max((r["maximum_constraint_violation"] for r in completed), default=None),
                    worst_dense_clearance_range_m=[min((r["recomputed_worst_dense_clearance_m"] for r in completed), default=None),
                                                   max((r["recomputed_worst_dense_clearance_m"] for r in completed), default=None)],
                    maximum_witness_recheck_error_m=max((r["worst_dense_clearance_recheck_error_m"] for r in completed), default=None),
                    maximum_reconstruction_error_rad=max((r["saved_curve_reconstruction_max_error_rad"] for r in completed), default=None)))
        report["status"] = "completed" if all(r["status"] == "completed" for r in report["cases"]) else "completed_with_errors"
        exit_code = int(report["status"] != "completed")
    except Exception as exc:
        report.update(status="error", exception=repr(exc), traceback=traceback.format_exc())
        exit_code = 1
    finally:
        np.savez_compressed(args.output / "cpu_values.npz", **arrays)
        report.update(seconds=time.perf_counter()-began, numerical_arrays="cpu_values.npz",
                      numerical_arrays_sha256=sha(args.output / "cpu_values.npz"),
                      rebuilt_scene_models=len(cache),
                      limitation="Only saved failures and finite numerical collocation/witness checks; no continuous-safety certificate, new planning outcome, parameter tuning or score replacement")
        write(args.output / "summary.json", report)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
