"""CPU regressions for the distinct refinement policy and its failure records."""
import json
import re
import time
from types import SimpleNamespace

import numpy as np
import pytest

from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import HERE, load_config
from benchmarks.generic_motion_v1.runners.paper_nto import PROFILE as ORIGINAL
from benchmarks.generic_motion_v1.runners.paper_nto import paper_nto, warm_start
from benchmarks.generic_motion_v1.runners.validate import sample_candidate
from benchmarks.generic_motion_v1.runners.paper_nto_refined import (
    PROFILE, paper_nto_refined, refinement_locations,
)


def test_refinement_uses_interval_minima_and_preserves_global_cap():
    existing = np.array([.25, .5, .75])
    t = np.array([0., .1, .2, .25, .3, .4, .5, .6, .7, .9, 1.])
    d = np.array([-1., .0004, -.01, -1., .002, .003, -1., -.02, -.01, .004, -1.])
    np.testing.assert_array_equal(refinement_locations(existing, t, d), [.2, .6])
    existing = np.arange(1, 600) / 600
    t = (np.r_[0., existing] + np.r_[existing, 1.]) / 2
    added = refinement_locations(existing, t, -np.arange(len(t), dtype=float))
    assert len(existing) + len(added) == 600
    assert added[0] == t[-1]


@pytest.mark.parametrize("objective", ["minimum_distance", "minimum_time"])
def test_real_cpu_refined_solver_artifacts_and_unmodified_original(tmp_path, objective):
    model = Model()
    frozen = json.loads((HERE / "manifests/dev.jsonl").read_text().splitlines()[0])
    start = np.array(frozen["q_start"])
    goal = start.copy()
    goal[0] += .02
    case = dict(q_start=start.tolist(), q_goal=goal.tolist(), _objective=objective)
    candidate = paper_nto_refined(np.array([start, goal]), model, case, load_config(),
                                 tmp_path, time.perf_counter()+30)
    assert candidate["solver_success"]
    assert candidate["refinement_final_geometry_guard_passed"]
    assert candidate["refinement_native_solves"] == 1
    assert candidate["profile"] == PROFILE["name"]
    assert ORIGINAL["clearance_m"] == 0
    assert "snopt_total_iterations" not in ORIGINAL
    record = json.loads((tmp_path / "nto_refinement.json").read_text())
    assert record["final_validation_required"]
    assert record["rounds"][0]["collocation_points"] == 149
    assert (tmp_path / "refinement/round_00/candidate.npz").exists()
    assert (tmp_path / "refinement/round_00/refinement_geometry.npz").exists()
    params = json.loads((tmp_path / "nto_parameters.json").read_text())
    if objective == "minimum_distance":
        assert params["parameters"]["Iterations limit"] == 100000
        assert params["parameters"]["Minor iterations limit"] == 500
        assert re.search(r"Iteration limit\.+\s+100000\b", (tmp_path / "snopt.log").read_text())
    else:
        assert len(candidate["controls"]) == 16
        assert params["minimum_time_control_points"] == 16
        assert params["warm_start_inserted_knots"] == 12
        assert record["controls_inserted"] and record["initial_curve_preserved"]
        # Same real canonical collision model and fixed endpoints/limits. The
        # original four-control cubic has avoidable derivative-hull slack.
        original = paper_nto(np.array([start, goal]), model, case, load_config(),
                             tmp_path / "unrefined", time.perf_counter()+30)
        assert original["solver_success"]
        assert candidate["duration"] < original["duration"] * .95
        samples = sample_candidate(candidate)
        assert model.clearance(samples["q"]).min() > .01
        assert np.max(samples["derivatives"]["velocity_per_joint"]) <= 1 + 1e-5
        assert np.max(samples["derivatives"]["acceleration_per_joint"]) <= 1 + 1e-5
        assert samples["derivatives"]["endpoint_velocity"] < 1e-6


def test_expired_refinement_budget_records_failure_before_model_construction(tmp_path):
    model = SimpleNamespace(nto_plant=lambda: pytest.fail("Must not construct expired solve"))
    with pytest.raises(TimeoutError, match="shared online budget"):
        paper_nto_refined(np.zeros((2, 7)), model, {}, load_config(), tmp_path, time.perf_counter()-1)
    record = json.loads((tmp_path / "nto_refinement.json").read_text())
    assert record["stopping_reason"] == "exception"
    assert record["rounds"] == []


def test_actual_failed_native_candidate_is_not_relabelled_as_success(tmp_path):
    model = Model()
    start = np.array(json.loads((HERE / "manifests/dev.jsonl").read_text().splitlines()[0])["q_start"])
    goal = start.copy()
    goal[0] += .02
    upper = model.upper.copy()
    upper[0] = start[0] - .1
    impossible = SimpleNamespace(lower=model.lower, upper=upper, nto_plant=model.nto_plant,
                                 clearance=model.clearance)
    case = dict(q_start=start.tolist(), q_goal=goal.tolist(), _objective="minimum_distance")
    candidate = paper_nto_refined(np.array([start, goal]), impossible, case, load_config(),
                                 tmp_path, time.perf_counter()+30)
    assert not candidate["solver_success"]
    saved = json.loads((tmp_path / "nto_solver_result.json").read_text())
    assert not saved["solver_success"]
    with np.load(tmp_path / "nto_solver_raw.npz") as arrays:
        np.testing.assert_array_equal(arrays["controls"], candidate["controls"])


@pytest.mark.parametrize("count", [2, 5, 18])
def test_time_knot_insertion_preserves_initial_curve_and_derivatives(count):
    path = np.zeros((count, 7))
    path[:, 0] = np.linspace(0, 2, count)
    path[:, 1] = np.sin(np.linspace(0, 2, count))
    base, _, _, _, duration = warm_start(path, "minimum_time")
    refined, original, controls, _, refined_duration = warm_start(
        path, "minimum_time", minimum_time_control_points=16)
    np.testing.assert_array_equal(original, path)
    assert len(controls) == max(16, count+2)
    assert duration == refined_duration
    samples = np.linspace(0, duration, 1001)
    for derivative in (0, 1, 2):
        left = base if derivative == 0 else base.MakeDerivative(derivative)
        right = refined if derivative == 0 else refined.MakeDerivative(derivative)
        np.testing.assert_allclose(left.vector_values(samples), right.vector_values(samples), atol=1e-11, rtol=1e-11)
    _, _, distance_controls, _, _ = warm_start(path, "minimum_distance", minimum_time_control_points=16)
    np.testing.assert_array_equal(distance_controls, path)


def test_actual_rest_to_rest_parameterization_control_analytical_bound():
    """An obstacle-free numerical control isolates hull conservatism from geometry."""
    from pydrake.all import KinematicTrajectoryOptimization, IpoptSolver
    path = np.zeros((2, 7))
    path[1, 0] = 2.0
    durations = []
    for count in (4, 16):
        initial, _, _, _, _ = warm_start(path, "minimum_time", minimum_time_control_points=count)
        traj = KinematicTrajectoryOptimization(initial)
        traj.AddDurationCost(1.0)
        traj.AddDurationConstraint(.05, 100.0)
        traj.AddPathPositionConstraint(path[0], path[0], 0.)
        traj.AddPathPositionConstraint(path[1], path[1], 1.)
        traj.AddVelocityBounds(-np.ones(7), np.ones(7))
        traj.AddAccelerationBounds(-np.ones(7), np.ones(7))
        traj.AddPathVelocityConstraint(np.zeros(7), np.zeros(7), 0.)
        traj.AddPathVelocityConstraint(np.zeros(7), np.zeros(7), 1.)
        solver = IpoptSolver()
        prog = traj.get_mutable_prog()
        prog.SetSolverOption(solver.solver_id(), "tol", 1e-6)
        prog.SetSolverOption(solver.solver_id(), "print_level", 0)
        result = solver.Solve(prog)
        assert result.is_success()
        curve = traj.ReconstructTrajectory(result)
        durations.append(curve.end_time())
        # The exact rest-to-rest lower bound is 3s for displacement2, v=a=1.
        assert curve.end_time() >= 3.0-1e-5
        for order in (1, 2):
            assert np.max(np.abs(curve.MakeDerivative(order).control_points())) <= 1+1e-5
    assert durations[0] == pytest.approx(6.0, abs=1e-5)
    assert durations[1] < 3.1


@pytest.mark.parametrize("value", [True, False, 3, 16.0, "16", None, float("inf")])
def test_refined_control_count_rejects_invalid_config(value):
    with pytest.raises(ValueError, match="minimum_time_control_points"):
        load_config({"nto_refined": {"minimum_time_control_points": value}})


def test_refined_config_override_and_second_native_round(tmp_path):
    for unknown in ({}, {"minimum_control_points": 16}, {"minimum_time_control_points": 16, "solver": "SNOPT"}):
        with pytest.raises(ValueError, match="nto_refined"):
            load_config({"nto_refined": unknown})
    cfg = load_config({"nto_refined": {"minimum_time_control_points": 4}})
    model = Model()
    start = np.array(json.loads((HERE / "manifests/dev.jsonl").read_text().splitlines()[0])["q_start"])
    goal = start.copy()
    goal[0] += .02
    path = np.array([start, goal])
    case = dict(q_start=start.tolist(), q_goal=goal.tolist(), _objective="minimum_time")
    four = paper_nto_refined(path, model, case, cfg, tmp_path / "four", time.perf_counter()+30)
    assert len(four["controls"]) == 4
    assert PROFILE["minimum_time_control_points"] == 16
    sixteen = paper_nto_refined(path, model, case, load_config(), tmp_path / "sixteen", time.perf_counter()+30)
    second = paper_nto(path, model, case, load_config(), tmp_path / "second", time.perf_counter()+30,
                       _profile=PROFILE, _initial_candidate=sixteen)
    assert second["solver_success"] and len(second["controls"]) == 16
    assert json.loads((tmp_path / "second/nto_parameters.json").read_text())["refinement_warm_start"]
