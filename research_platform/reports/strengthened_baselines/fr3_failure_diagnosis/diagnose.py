"""Read-only numerical audit of the frozen e011 distance-baseline failure.

No Solve, CUDA query, parameter search or benchmark task is executed here.
Only saved inputs/iterates and the declared Drake constraints are evaluated.
"""
from pathlib import Path
import hashlib
import json
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from pydrake.all import BsplineBasis, BsplineTrajectory, KinematicTrajectoryOptimization, MinimumDistanceLowerBoundConstraint
from pydrake.autodiffutils import InitializeAutoDiff, ExtractGradient
from research_platform.backend import _environment

OUT = Path(__file__).resolve().parent
RUN = ROOT / "runs/v1_20261027_fr3"
JOBS = RUN / "jobs"
STRONG = JOBS / "method-e011_r000_n03000-NTO_distance_strong/attempts/0001"
LEGACY = JOBS / "method-e011_r000_n03000-NTO_distance/attempts/0001"
PLAN = JOBS / "plan-e011_r000_n03000/attempts/0001"


def read(path):
    return json.loads(path.read_text())


def serial(value):
    if isinstance(value, np.ndarray):
        return serial(value.tolist())
    if isinstance(value, np.generic):
        return serial(value.item())
    if isinstance(value, dict):
        return {k: serial(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serial(v) for v in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write(path, value):
    path.write_text(json.dumps(serial(value), indent=2, allow_nan=False) + "\n")


def trajectory(controls, knots):
    return BsplineTrajectory(BsplineBasis(2, knots.tolist()), [q[:, None] for q in controls])


def clearance_profile(curve, env, parameters):
    distances, pairs = [], []
    inspector = env.sg.model_inspector()
    for q in curve.vector_values(parameters).T:
        env.drake.SetPositions(env.pc, q)
        query = env.sg.get_query_output_port().Eval(env.sg.GetMyContextFromRoot(env.context))
        near = query.ComputeSignedDistancePairwiseClosestPoints(.05)
        if near:
            minimum = min(near, key=lambda item: item.distance)
            distances.append(minimum.distance)
            pairs.append([inspector.GetName(minimum.id_A), inspector.GetName(minimum.id_B)])
        else:
            distances.append(np.inf)
            pairs.append(None)
    distances = np.asarray(distances)
    worst = int(np.argmin(distances))
    return dict(minimum_sampled_clearance_m=float(distances[worst]),
        minimum_parameter=float(parameters[worst]), minimum_pair=pairs[worst],
        samples=len(parameters), negative_count=int(np.count_nonzero(distances < 0)),
        below_1mm_count=int(np.count_nonzero(distances < .001)),
        query_threshold_m=.05, check="finite sampled Drake distances, not continuous certificate"), distances


def build_program(initial, env, locations, clearance):
    k = KinematicTrajectoryOptimization(initial)
    program = k.get_mutable_prog()
    k.AddPositionBounds(env.lower, env.upper)
    k.AddPathPositionConstraint(env.start, env.start, 0.)
    k.AddPathPositionConstraint(env.goal, env.goal, 1.)
    k.AddPathLengthCost()
    k.AddDurationConstraint(1., 1.)
    distance = MinimumDistanceLowerBoundConstraint(env.drake, clearance, env.pc,
        influence_distance_offset=env.protocol["nto_influence_distance"])
    bindings = [k.AddPathPositionConstraint(distance, float(s)) for s in locations]
    return k, program, bindings


def residuals(program, bindings):
    values = []
    for binding in bindings:
        x = program.GetInitialGuess(binding.variables())
        result = binding.evaluator().Eval(x)
        values.append(float(np.maximum(binding.evaluator().lower_bound() - result,
                                       result - binding.evaluator().upper_bound()).max()))
    return np.asarray(values)


def gradient_check(program, binding):
    x = program.GetInitialGuess(binding.variables())
    f = binding.evaluator()
    ad = ExtractGradient(f.Eval(InitializeAutoDiff(x)))[0]
    finite_difference = np.empty_like(ad)
    h = 1e-6
    for j in range(len(x)):
        v = np.zeros_like(x)
        v[j] = h
        finite_difference[j] = (f.Eval(x + v)[0] - f.Eval(x - v)[0]) / (2*h)
    return dict(finite_difference_step=h, variables=len(x),
        autodiff=ad, finite_difference=finite_difference,
        max_absolute_error=float(np.max(np.abs(ad-finite_difference))),
        relative_inf_error=float(np.max(np.abs(ad-finite_difference))/max(1., np.max(np.abs(ad)))),
        gradient_norm=float(np.linalg.norm(ad)))


def main():
    started = time.perf_counter()
    used = [RUN / "resolved.json", PLAN / "input.json", PLAN / "seed_path.npz",
            LEGACY / "candidate.json", LEGACY / "candidate.npz"]
    for directory in sorted((STRONG / "refinements").glob("round_*")):
        used.extend(directory / name for name in ["warm_start.npz", "candidate.npz", "candidate.json", "round.json"])
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in used}
    resolved = read(RUN / "resolved.json")
    prepared = read(PLAN / "input.json")
    env = _environment(resolved["protocol"], prepared["scene"], prepared)
    common_path = np.load(PLAN / "seed_path.npz")["path"]
    dense = np.linspace(0., 1., 5001)
    previous_curve = None
    rounds = []
    for index, directory in enumerate(sorted((STRONG / "refinements").glob("round_*"))):
        warm, saved = np.load(directory / "warm_start.npz"), np.load(directory / "candidate.npz")
        metadata, history = read(directory / "candidate.json"), read(directory / "round.json")
        initial = trajectory(warm["controls"], warm["knots"])
        locations = warm["collocation_locations"]
        k, program, bindings = build_program(initial, env, locations, .001)
        actual_guess = program.GetInitialGuess(k.control_points())
        initial_residuals = residuals(program, bindings)
        initial_all_residuals = residuals(program, program.GetAllConstraints())
        initial_profile, initial_distances = clearance_profile(initial, env, dense)
        initial_collocation_profile, initial_collocation_distances = clearance_profile(initial, env, locations)
        candidate = trajectory(saved["controls"], saved["knots"])
        candidate_profile, candidate_distances = clearance_profile(candidate, env, dense)
        _, candidate_program, candidate_bindings = build_program(candidate, env, locations, .001)
        candidate_residuals = residuals(candidate_program, candidate_bindings)
        worst = int(np.argmax(candidate_residuals))
        gradient = gradient_check(candidate_program, candidate_bindings[worst])
        warm_worst = int(np.argmax(initial_residuals))
        warm_gradient = gradient_check(program, bindings[warm_worst])
        objective_gradients = [gradient_check(program, binding) for binding in program.GetAllCosts()]
        record = dict(round=index, control_points=len(warm["controls"]), collocation_points=len(locations),
            solver_success=metadata["solver_success"], solver_status=metadata["solver_status"],
            initial_guess_is_finite=bool(np.isfinite(actual_guess).all()),
            initial_guess_max_control_error=float(abs(actual_guess-warm["controls"].T).max()),
            initial_guess_duration=float(program.GetInitialGuess(k.duration())),
            initial_endpoint_error=float(max(np.linalg.norm(warm["controls"][0]-env.start),
                                              np.linalg.norm(warm["controls"][-1]-env.goal))),
            initial_vs_shared_nodes_max_error=float(abs(warm["controls"]-common_path).max()) if index == 0 else None,
            previous_failed_curve_warm_start_max_error=(float(abs(initial.vector_values(dense)-previous_curve.vector_values(dense)).max())
                                                      if previous_curve is not None else None),
            initial_max_collision_constraint_violation=float(initial_residuals.max()),
            initial_max_all_constraint_violation=float(max(0., initial_all_residuals.max())),
            initial_violated_all_constraints=int(np.count_nonzero(initial_all_residuals > 1e-6)),
            initial_path_length=float(np.linalg.norm(np.diff(warm["controls"], axis=0), axis=1).sum()),
            initial_violated_collision_constraints=int(np.count_nonzero(initial_residuals > 1e-6)),
            initial_sampled_clearance=initial_profile, initial_collocation_clearance=initial_collocation_profile,
            candidate_max_collision_constraint_violation=float(candidate_residuals.max()),
            saved_candidate_max_collision_constraint_violation=metadata["maximum_collision_constraint_violation"],
            candidate_sampled_clearance=candidate_profile, candidate_worst_collocation_parameter=float(locations[worst]),
            initial_worst_binding_gradient=warm_gradient, candidate_worst_binding_gradient=gradient,
            initial_objective_gradients=objective_gradients,
            optimization_seconds=history["optimization_seconds"])
        rounds.append(record)
        np.savez_compressed(OUT / f"round_{index:03d}_evaluations.npz", parameters=dense,
            warm_clearances=initial_distances, candidate_clearances=candidate_distances,
            collocation_locations=locations, warm_collocation_clearances=initial_collocation_distances,
            warm_constraint_residuals=initial_residuals, candidate_constraint_residuals=candidate_residuals)
        write(OUT / "round_audit.json", rounds)
        previous_curve = candidate
        print(json.dumps(serial({k: record[k] for k in ["round", "initial_max_collision_constraint_violation",
            "candidate_max_collision_constraint_violation", "initial_sampled_clearance", "candidate_sampled_clearance"]})), flush=True)
    legacy_data = np.load(LEGACY / "candidate.npz")
    legacy_meta = read(LEGACY / "candidate.json")
    legacy_curve = trajectory(legacy_data["controls"], legacy_data["knots"])
    legacy_locations = np.asarray(legacy_meta["collocation_locations"])
    legacy_profile, legacy_distances = clearance_profile(legacy_curve, env, dense)
    legacy_collocation_profile, legacy_collocation_distances = clearance_profile(legacy_curve, env, legacy_locations)
    _, legacy_program, legacy_bindings = build_program(legacy_curve, env, legacy_locations, 0.)
    legacy_residuals = residuals(legacy_program, legacy_bindings)
    np.savez_compressed(OUT / "legacy_evaluations.npz", parameters=dense, clearances=legacy_distances,
        collocation_locations=legacy_locations, collocation_clearances=legacy_collocation_distances,
        constraint_residuals=legacy_residuals)
    summary = dict(command="bash local/python.sh research_platform/reports/strengthened_baselines/fr3_failure_diagnosis/diagnose.py",
        scope="read-only evaluation; no numerical Solve, CUDA query, source modification or benchmark rerun",
        raw_run="runs/v1_20261027_fr3", case="e011_r000_n03000", shared_nodes=len(common_path),
        optimizer_clearance=.001, rounds=rounds,
        legacy=dict(solver_success=legacy_meta["solver_success"], solver_status=legacy_meta["solver_status"],
                    optimization_clearance=0., sampled_clearance=legacy_profile,
                    collocation_clearance=legacy_collocation_profile,
                    max_collision_constraint_violation=float(legacy_residuals.max())),
        input_sha256=hashes, inputs_unchanged=all(hashlib.sha256((ROOT/path).read_bytes()).hexdigest() == digest for path, digest in hashes.items()),
        elapsed_seconds=time.perf_counter()-started)
    write(OUT / "diagnosis.json", summary)
    print(json.dumps(serial({"legacy": summary["legacy"], "inputs_unchanged": summary["inputs_unchanged"],
                            "elapsed_seconds": summary["elapsed_seconds"]})), flush=True)


if __name__ == "__main__":
    main()
