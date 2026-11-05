"""Recompute two saved EI-ZO execution failures without changing their evidence.

Run from any working directory using the project's existing Conda Python.
This diagnostic evaluates original Bezier derivatives and saved MuJoCo arrays;
it does not integrate again, change a threshold, or update any result/receipt.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.simulation.reference import load_reference


CASES = ("smoke-sparse_clutter-000-p01", "smoke-bottleneck-000-p00")


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def relative(path):
    path = Path(path).resolve()
    return str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)


def source_path(stored):
    path = Path(stored)
    if path.exists():
        return path.resolve()
    # Retain usability when the complete repository was relocated.
    if "benchmarks" in path.parts:
        index = path.parts.index("benchmarks")
        moved = ROOT.joinpath(*path.parts[index:])
        if moved.exists():
            return moved.resolve()
    raise FileNotFoundError(f"Original source attempt is missing: {stored}")


def diagnose(directory):
    directory = Path(directory).resolve()
    reference_metadata = read(directory / "reference.json")
    attempt = source_path(reference_metadata["attempt_path"])
    for name, expected in reference_metadata["source_files_sha256"].items():
        if sha(attempt / name) != expected:
            raise ValueError(f"Original reference hash mismatch: {name}")
    reference = load_reference(attempt)
    config = read(directory / "config.json")
    result = read(directory / "result.json")
    case = read(attempt / "input.json")
    mapping = read(directory / "model/model_metadata.json")
    qi, vi = np.asarray(mapping["qpos_ids"], int), np.asarray(mapping["dof_ids"], int)
    with np.load(directory / "rollout.npz", allow_pickle=False) as source:
        arrays = {name: source[name].copy() for name in source.files}
    with np.load(attempt / "candidate.npz", allow_pickle=False) as source:
        controls, durations = source["controls"].copy(), source["durations"].copy()
    if reference.metadata["kind"] != "bezier":
        raise ValueError("This diagnosis expects the original piecewise Bezier trajectory")
    degree = controls.shape[1] - 1
    velocity_controls = degree * np.diff(controls, axis=1) / durations[:, None, None]
    acceleration_controls = (degree - 1) * np.diff(velocity_controls, axis=1) / durations[:, None, None]
    boundaries = np.r_[0., np.cumsum(durations)]
    time = arrays["time"]
    dt = np.diff(time)
    if np.any(dt <= 0):
        raise ValueError("Saved physical timestamps are not strictly increasing")
    q, velocity, acceleration = arrays["qpos"][:, qi], arrays["qvel"][:, vi], arrays["qacc"][:, vi]
    qref, vref, aref = arrays["qref"], arrays["qvel_ref"], arrays["qacc_ref"]
    exact = np.asarray([reference.at(float(t)) for t in time])
    exact_residuals = dict(position_rad=float(np.max(np.abs(exact[:, 0] - qref))),
                           velocity_rad_s=float(np.max(np.abs(exact[:, 1] - vref))),
                           acceleration_rad_s2=float(np.max(np.abs(exact[:, 2] - aref))))
    row, joint = np.unravel_index(np.argmax(np.abs(acceleration)), acceleration.shape)
    position_feedback = config["kp_s2"] * (qref - q)
    velocity_feedback = config["kd_s"] * (vref - velocity)
    requested_acceleration = aref + position_feedback + velocity_feedback
    internal = boundaries[1:-1]
    segment = int(np.argmin(np.abs(internal - time[row])))
    boundary = float(internal[segment])
    held = time > reference.duration
    neighborhood = []
    for i in range(max(0, row - 5), min(len(time), row + 6)):
        neighborhood.append(dict(index=i, time_s=float(time[i]),
                                 actual_acceleration_rad_s2=float(acceleration[i, joint]),
                                 reference_acceleration_rad_s2=float(aref[i, joint]),
                                 position_error_rad=float(q[i, joint] - qref[i, joint]),
                                 velocity_error_rad_s=float(velocity[i, joint] - vref[i, joint])))
    input_files = [directory / name for name in ("rollout.npz", "config.json", "reference.json", "result.json")]
    input_files += [directory / "model/model_metadata.json", directory / "model/model.xml"]
    input_files += [attempt / name for name in ("candidate.json", "candidate.npz", "input.json", "validation.json", "receipt.json")]
    hashes = {relative(path): sha(path) for path in input_files}
    acceleration_limit = np.asarray(case["acceleration_limits_rad_s2"])
    excess = float(max(0., np.max(np.max(np.abs(acceleration), axis=0) - acceleration_limit)))
    if not np.isclose(excess, result["acceleration_excess_rad_s2"], atol=1e-12, rtol=0):
        raise ValueError("Recomputed actual acceleration excess differs from the saved result")
    return dict(
        problem_id=case["problem_id"], execution_directory=relative(directory),
        source_attempt=relative(attempt), source_files_sha256=hashes,
        original_execution_status=result["status"], original_planning_status=result["original_planning_status"],
        original_execution_checks=result["checks"], source_reference_unchanged=True,
        controller=config["controller"], kp_s2=config["kp_s2"], kd_s=config["kd_s"], timestep_s=config["timestep_s"],
        source_reference_duration_s=reference.duration, source_segments=int(len(durations)),
        source_bezier_degree=int(degree), source_segment_durations_s=durations.tolist(),
        source_segment_boundaries_s=boundaries.tolist(),
        source_derivative_control_max_abs_acceleration_rad_s2=float(np.max(np.abs(acceleration_controls))),
        source_position_continuity_max_abs_residual_rad=float(np.max(np.abs(controls[:-1, -1] - controls[1:, 0]), initial=0)),
        source_velocity_continuity_max_abs_residual_rad_s=float(np.max(np.abs(velocity_controls[:-1, -1] - velocity_controls[1:, 0]), initial=0)),
        recomputed_exact_reference_vs_saved_max_abs_residual=exact_residuals,
        peak=dict(index=int(row), joint_one_based=int(joint + 1), time_s=float(time[row]),
                  actual_acceleration_rad_s2=float(acceleration[row, joint]),
                  reference_acceleration_rad_s2=float(aref[row, joint]),
                  position_feedback_rad_s2=float(position_feedback[row, joint]),
                  velocity_feedback_rad_s2=float(velocity_feedback[row, joint]),
                  acceleration_limit_rad_s2=float(acceleration_limit[joint]),
                  acceleration_excess_rad_s2=excess, neighborhood=neighborhood),
        nearest_internal_boundary=dict(time_s=boundary, peak_offset_s=float(time[row] - boundary),
                                      first_saved_time_after_boundary_s=float(time[np.searchsorted(time, boundary, side="right")]),
                                      left_acceleration_rad_s2=acceleration_controls[segment, -1].tolist(),
                                      right_acceleration_rad_s2=acceleration_controls[segment + 1, 0].tolist(),
                                      peak_joint_acceleration_jump_rad_s2=float(acceleration_controls[segment + 1, 0, joint] - acceleration_controls[segment, -1, joint])),
        dynamics_timing_residuals=dict(
            saved_qacc_vs_current_feedback_max_abs_rad_s2=float(np.max(np.abs(acceleration - requested_acceleration))),
            forward_velocity_difference_vs_saved_qacc_max_abs_rad_s2=float(np.max(np.abs(np.diff(velocity, axis=0) / dt[:, None] - acceleration[:-1]))),
            semiimplicit_position_difference_vs_next_velocity_max_abs_rad_s=float(np.max(np.abs(np.diff(q, axis=0) / dt[:, None] - velocity[1:])))),
        post_endpoint_hold_max_abs_acceleration_rad_s2=float(np.max(np.abs(acceleration[held]), initial=0)),
        conclusion="Peak occurs at the first saved integration state after an original C1 Bezier segment acceleration jump. Exact reference derivatives and current-time qacc agree; no one-frame timing bug was found. Discrete tracking feedback produces actual acceleration above the source reference bound. Preserve execution_failed and the separate original planning success.",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--jobs", type=Path, default=HERE / "eizo/jobs")
    parser.add_argument("--output", type=Path, default=HERE / "tracking_failure_diagnosis.json")
    args = parser.parse_args()
    diagnoses = []
    for identity in CASES:
        matches = sorted(args.jobs.glob(identity + "-EI_ZO-minimum_time-*"))
        if len(matches) != 1:
            raise ValueError(f"Expected exactly one saved execution for {identity}, found {len(matches)}")
        diagnoses.append(diagnose(matches[0]))
    report = dict(
        schema="mujoco_saved_tracking_failure_diagnosis_v1", diagnostic_script_sha256=sha(__file__),
        recompute_command="bash local/python.sh results/mujoco_smoke/diagnose_tracking.py",
        scope="Read-only recalculation of saved executions and original coefficients; no new integration and no result/receipt modifications.",
        cases=diagnoses,
        recommended_separate_sensitivity_check="Keep source trajectory, original assets, gains and thresholds fixed; compare 2, 1, 0.5 and optionally 0.25 ms. Boundary/grid phase can make individual peaks nonmonotonic. Preserve the original 2 ms failures regardless of smaller-step results.",
    )
    # Verify all evidence bytes remained unchanged during recomputation.
    for row in diagnoses:
        for name, expected in row["source_files_sha256"].items():
            path = Path(name) if Path(name).is_absolute() else ROOT / name
            if sha(path) != expected:
                raise ValueError(f"Source evidence changed during diagnosis: {name}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, sort_keys=True, indent=2, allow_nan=False) + "\n")
    print(json.dumps(dict(output=str(args.output.resolve()), cases=len(diagnoses),
                          original_statuses=[row["original_execution_status"] for row in diagnoses],
                          source_evidence_unchanged=True), sort_keys=True))


if __name__ == "__main__":
    main()
