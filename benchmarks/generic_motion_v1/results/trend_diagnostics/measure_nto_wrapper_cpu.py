"""CPU-only fixed-input Eval microbenchmark, permitted only when smoke is idle.

This does not run an optimizer, alter a model, or query CUDA. It takes the
existing exclusive GPU-run lock as an idle-window coordination primitive.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--evaluations-per-block", type=int, default=200)
    args = parser.parse_args()
    run = ROOT / "results/repaired_smoke"
    manifest = json.loads((run / "run_manifest.json").read_text())
    pending = [job["job_id"] for job in manifest["jobs"]
               if not any((run / "jobs" / job["job_id"] / "attempts").glob("*/process.json"))]
    if pending:
        raise SystemExit(f"Refusing measurement: {len(pending)} smoke jobs are not terminal")
    with (ROOT / "results/.gpu.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise SystemExit("Refusing measurement: planning lock is currently held") from error
        # Deferred imports prevent accidental CPU load during active evaluation.
        import numpy as np
        from pydrake.all import (MathematicalProgram, KinematicTrajectoryOptimization,
                                BsplineBasis, BsplineTrajectory, KnotVectorType)
        from pydrake.autodiffutils import AutoDiffXd, ExtractGradient, ExtractValue, InitializeAutoDiff
        from benchmarks.generic_motion_v1.model import Model
        from benchmarks.generic_motion_v1.runners.nto_collision_constraint import RobustMinimumDistanceConstraint

        Model.cuda = lambda *a, **k: (_ for _ in ()).throw(AssertionError("GPU forbidden in CPU Eval microbenchmark"))
        began = time.perf_counter()
        attempt = next((run / "jobs").glob("smoke-bottleneck-000-p00-NTO_PAPER-minimum_time-*")) / "attempts/0001"
        case = json.loads((attempt / "input.json").read_text())
        model = Model(case["voxels_xyz"], case["voxel_radius_m"])
        native = model.nto_plant()
        wrapper = RobustMinimumDistanceConstraint(native, .001, .01)
        q = np.asarray(case["q_start"], dtype=float)
        setup_seconds = time.perf_counter() - began

        bare_program = MathematicalProgram()
        bare_x = bare_program.NewContinuousVariables(len(q))
        bare = bare_program.AddConstraint(lambda x: wrapper.original.Eval(x),
                                         wrapper.original.lower_bound(),
                                         wrapper.original.upper_bound(), bare_x).evaluator()

        def prevalidation_only(value):
            # Exact normal validation prefix from the frozen implementation,
            # with no native distance computation or counter increment.
            from pydrake.autodiffutils import AutoDiffXd, ExtractGradient, ExtractValue
            value = np.asarray(value)
            if value.shape not in ((wrapper.num_positions,), (wrapper.num_positions, 1)):
                raise ValueError("Collision constraint expects a joint vector or column vector")
            value = value.reshape(-1)
            autodiff = any(isinstance(x, AutoDiffXd) for x in value)
            positions = np.asarray(ExtractValue(value) if autodiff else value, float).ravel()
            if not np.isfinite(positions).all():
                raise ValueError("Collision constraint joint positions must be finite")
            if autodiff and not np.isfinite(ExtractGradient(value)).all():
                raise ValueError("Collision constraint AutoDiff derivatives must be finite")
            return positions

        variants = {"direct_native_Eval": wrapper.original.Eval,
                    "bare_PyFunctionConstraint_Eval": bare.Eval,
                    "robust_PyFunctionConstraint_Eval": wrapper.constraint.Eval,
                    "robust_callback_direct": wrapper.evaluate,
                    "prevalidation_only": prevalidation_only}
        saved_path = np.load(attempt / "path.npy")
        path_indices = np.linspace(0, len(saved_path)-1, min(8, len(saved_path)), dtype=int)
        inputs = {}
        for pattern, points in [("fixed", [q]), ("saved_path_cycle", saved_path[path_indices])]:
            inputs[pattern+"/float"] = [point for point in points]
            inputs[pattern+"/autodiff_identity_7x7"] = [InitializeAutoDiff(point).ravel() for point in points]
            inputs[pattern+"/autodiff_local_cubic_7x28"] = [InitializeAutoDiff(point, np.tile(np.eye(7), (1, 4))).ravel() for point in points]
            inputs[pattern+"/autodiff_wide_7x113"] = [InitializeAutoDiff(point, np.tile(np.eye(7), (1, 17))[:, :113]).ravel() for point in points]
        rows, accuracy = [], []
        measure_began = time.perf_counter()
        for kind, sequence in inputs.items():
            autodiff = "/autodiff_" in kind
            for point_index, value in enumerate(sequence):
                expected = wrapper.original.Eval(value)
                expected_value = np.asarray(ExtractValue(expected), float).ravel()
                expected_gradient = ExtractGradient(expected) if autodiff else None
                if not np.isfinite(expected_value).all() or (kind.startswith("fixed/") and float(expected_value[0]) > 1.):
                    raise AssertionError("Fixed endpoint is not a finite feasible minimum-distance input")
                for name, function in variants.items():
                    for _ in range(10):
                        actual = function(value)
                    if name != "prevalidation_only":
                        value_error = float(np.max(np.abs(np.asarray(ExtractValue(actual)).ravel() - expected_value)))
                        gradient_error = float(np.max(np.abs(ExtractGradient(actual) - expected_gradient))) if autodiff else None
                        assert value_error == 0. and (gradient_error is None or gradient_error == 0.)
                        accuracy.append(dict(input_kind=kind, point_index=point_index, variant=name,
                                             native_constraint_value=float(expected_value[0]),
                                             value_error=value_error, gradient_error=gradient_error))
            names = list(variants)
            for repetition in range(args.repetitions):
                # Rotate and reverse order to distribute thermal/order bias.
                order = names[repetition % len(names):] + names[:repetition % len(names)]
                if repetition % 2:
                    order.reverse()
                for name in order:
                    function = variants[name]
                    start = time.perf_counter()
                    for evaluation in range(args.evaluations_per_block):
                        function(sequence[evaluation % len(sequence)])
                    elapsed = time.perf_counter() - start
                    rows.append(dict(input_kind=kind, variant=name, repetition=repetition,
                                     evaluations=args.evaluations_per_block, elapsed_seconds=elapsed,
                                     seconds_per_evaluation=elapsed / args.evaluations_per_block))
        summaries = []
        for kind in inputs:
            for name in variants:
                values = [r["seconds_per_evaluation"] for r in rows if r["input_kind"] == kind and r["variant"] == name]
                summaries.append(dict(input_kind=kind, variant=name, median_seconds_per_evaluation=statistics.median(values),
                                      minimum_seconds_per_evaluation=min(values), maximum_seconds_per_evaluation=max(values)))
        assert not wrapper.events, "Fixed safe point unexpectedly entered exceptional recovery"
        measurement_seconds = time.perf_counter()-measure_began
        # API-only probe: replace the exact collision bindings without solving
        # or changing any initial decision values, bounds or sample locations.
        constant = BsplineTrajectory(BsplineBasis(4, 4, KnotVectorType.kClampedUniform, 0., 1.),
                                     [q[:, None] for _ in range(4)])
        trajectory = KinematicTrajectoryOptimization(constant)
        program = trajectory.get_mutable_prog()
        baseline_constraints = len(program.GetAllConstraints())
        locations = [.25, .75]
        bindings = [trajectory.AddPathPositionConstraint(wrapper.original, s) for s in locations]
        initial_variables = program.GetInitialGuess(program.decision_variables()).copy()
        native_values = [program.EvalBinding(binding, initial_variables) for binding in bindings]
        removed = [program.RemoveConstraint(binding) for binding in bindings]
        assert removed == [1, 1]
        replacements = [trajectory.AddPathPositionConstraint(wrapper.constraint, s) for s in locations]
        assert len(program.GetAllConstraints()) == baseline_constraints + len(bindings)
        np.testing.assert_array_equal(program.GetInitialGuess(program.decision_variables()), initial_variables)
        binding_errors = [float(np.max(np.abs(program.EvalBinding(binding, initial_variables)-value)))
                          for binding, value in zip(replacements, native_values)]
        assert max(binding_errors) == 0.
        output = dict(created_utc=datetime.now(timezone.utc).isoformat(), scope=__doc__,
                      source_commit=manifest["source_code_commit"],
                      source_sha256=hashlib.sha256((ROOT / "benchmarks/generic_motion_v1/runners/nto_collision_constraint.py").read_bytes()).hexdigest(),
                      input_json=str((attempt / "input.json").relative_to(ROOT)), q=q.tolist(),
                      saved_path_sha256=hashlib.sha256((attempt / "path.npy").read_bytes()).hexdigest(),
                      saved_path_cycle_indices=path_indices.tolist(),
                      collision_pairs=len(wrapper.pairs), clearance_m=.001, influence_distance_m=.01,
                      gpu_calls=0, optimizer_calls=0, smoke_jobs_terminal=len(manifest["jobs"]),
                      exclusive_planning_lock_held=True, setup_seconds=setup_seconds,
                      measurement_seconds=measurement_seconds,
                      fixed_endpoint_native_value=np.asarray(ExtractValue(wrapper.original.Eval(q))).tolist(),
                      accuracy=accuracy, measurements=rows, summary=summaries,
                      recovery_events=wrapper.events,
                      native_first_binding_api_probe=dict(binding_type=str(type(bindings[0])),
                                                          removed_counts=removed, baseline_constraints=baseline_constraints,
                                                          replacement_constraints=len(program.GetAllConstraints()),
                                                          initial_guess_changed=False, sample_locations=locations,
                                                          maximum_value_error=max(binding_errors), solver_calls=0),
                      caveats=["Fixed-q and cycling saved-path measurements are separated; every variant receives the same order within each pattern.",
                               "The 7x113 seed is an explicit wide-AD sensitivity probe, not a claim that KTO always supplies this exact derivative dimension.",
                               "Timing excludes model setup and performs no optimizer; it estimates call overhead rather than whole-query speedup."])
        target = ROOT / "results/trend_diagnostics/nto_wrapper_cpu_timing.json"
        target.write_text(json.dumps(output, indent=2) + "\n")
        print(json.dumps({"output": str(target.relative_to(ROOT)), "setup_seconds": setup_seconds,
                          "measurement_seconds": output["measurement_seconds"], "summary": summaries}, indent=2))


if __name__ == "__main__":
    main()
