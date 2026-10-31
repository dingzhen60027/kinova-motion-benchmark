"""Small real CPU solves for the strengthened baseline and its failure contract."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from research_platform import strong_nto


def environment(centers=(), radius=.25, start=(0., 0.), goal=(1., 0.), timed_solver="SNOPT"):
    from pydrake.all import RobotDiagramBuilder
    from reproduction.forest import urdf
    centers = np.asarray(centers, dtype=float).reshape(-1, 2)
    cfg = dict(domain_lower=[-3., -3.], domain_upper=[3., 3.],
               point_radius=.01, obstacle_radius=radius - .01)
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    builder.parser().AddModelsFromString(urdf(cfg, centers), "urdf")
    plant.Finalize()
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    return SimpleNamespace(kind="forest", dimension=2, start=np.asarray(start), goal=np.asarray(goal),
        lower=np.array(cfg["domain_lower"]), upper=np.array(cfg["domain_upper"]),
        cfg=dict(cfg, feasibility_tolerance=1e-5),
        base=SimpleNamespace(centers=centers, radius=radius), drake=plant, diagram=diagram,
        context=context, pc=plant.GetMyMutableContextFromRoot(context),
        protocol={"optimizer_time_limit_seconds": 20., "nto_major_iterations": 2000,
                  "nto_solvers": {"forest": {"distance": "SNOPT", "time": timed_solver}},
                  "velocity_limit": 1., "acceleration_limit": 1., "nto_influence_distance": .1})


def validate(candidate, env):
    """CPU reference: exact PWL geometry, dense cubic geometry, derivative hulls."""
    from reproduction.alignment_diagnosis.forest_oracle import distance_value_jacobian
    curve = strong_nto._trajectory(candidate)
    times = np.linspace(curve.start_time(), curve.end_time(), 2001)
    q = curve.vector_values(times).T
    if len(env.base.centers):
        clearance = np.linalg.norm(q[:, None] - env.base.centers[None, :], axis=2).min(axis=1) - env.base.radius
        minimum = float(clearance.min())
        if candidate["spline_order"] == 2:
            minimum = float(distance_value_jacobian(candidate["controls"].ravel(), env.base.centers)[0].min() - env.base.radius)
    else:
        clearance = np.full(len(q), np.inf)
        minimum = None
    collision_free = bool(np.all(clearance >= 0.) and (minimum is None or minimum >= 0.))
    endpoint_error = max(np.linalg.norm(q[0] - env.start), np.linalg.norm(q[-1] - env.goal))
    derivative_pass = True
    if candidate["method"] == "NTO_time":
        derivative_pass = bool(max(np.abs(curve.MakeDerivative(1).control_points()).ravel()) <= 1. + 1e-5
            and max(np.abs(curve.MakeDerivative(2).control_points()).ravel()) <= 1. + 1e-5
            and np.max(np.abs(curve.MakeDerivative(1).vector_values(times[[0, -1]]))) <= 1e-5)
    free = clearance >= 0.
    return dict(passed=collision_free and endpoint_error < 1e-5 and derivative_pass,
                collision_free=collision_free, minimum_clearance=minimum,
                derivative_pass=derivative_pass, endpoint_error=float(endpoint_error),
                _samples={"q": q, "time": times, "cuda": free, "independent": free})


def test_real_richer_time_spline_improves_four_control_transcription(tmp_path):
    env = environment()
    path = np.asarray([env.start, env.goal])
    sparse = strong_nto.solve(path, env, minimum_time=True,
        options={"minimum_control_points": 4, "maximum_refinements": 0},
        out=tmp_path / "four", validate_callback=validate)
    rich = strong_nto.solve(path, env, minimum_time=True,
        options={"minimum_control_points": 16, "maximum_refinements": 0},
        out=tmp_path / "sixteen", validate_callback=validate)
    assert sparse["status"] == rich["status"] == "success"
    # The rest-to-rest unit-distance analytic lower bound is 2 s. A single
    # cubic's velocity hull forces 3 s; richer controls approach the lower bound.
    assert sparse["candidate"]["duration"] == pytest.approx(3., abs=2e-5)
    assert 2. - 1e-5 <= rich["candidate"]["duration"] < 2.15
    assert rich["candidate"]["warm_start_shape_max_error"] < 1e-12
    assert np.array_equal(rich["candidate"]["warm_start_original_nodes"], path)
    assert rich["validation"]["derivative_pass"]
    assert (tmp_path / "sixteen/refinements/round_000/snopt.log").exists()


def test_real_exact_forest_segment_constraints_and_saved_evidence(tmp_path):
    env = environment([[0., 0.]], start=(-1., 0.), goal=(1., 0.))
    path = np.array([env.start, [0., .7], env.goal])
    answer = strong_nto.solve(path, env, minimum_time=False, options={"maximum_refinements": 0},
                              out=tmp_path, validate_callback=validate)
    assert answer["status"] == "success"
    assert answer["candidate"]["exact_segment_circle_constraints"] == 2
    assert answer["candidate"]["collocation_points"] == 0
    assert answer["validation"]["minimum_clearance"] >= .001 - 1e-6
    assert np.allclose(answer["candidate"]["controls"][[0, -1]], path[[0, -1]], atol=1e-12)
    for filename in ("candidate.npz", "candidate.json", "validation.json", "solver_details.json", "round.json"):
        assert (tmp_path / "refinements/round_000" / filename).exists()
    assert json.loads((tmp_path / "strong_nto_result.json").read_text())["status"] == "success"


def test_actual_collision_witness_refinement_repairs_missed_circle(tmp_path):
    from research_platform.continuous_checks import forest_curve_check
    env = environment([[.41, .03]], radius=.05)

    def continuous_reference(candidate, environment):
        checked = validate(candidate, environment)
        continuous = forest_curve_check(candidate, environment)
        checked.update(minimum_clearance=continuous["minimum_clearance"],
            normalized_collision_parameters=continuous["normalized_collision_parameters"],
            normalized_minimum_parameters=continuous["normalized_minimum_parameters"],
            clearance_witnesses=continuous["clearance_witnesses"],
            passed=checked["passed"] and continuous["resolved"] and continuous["collision_free"])
        return checked

    answer = strong_nto.solve(np.array([env.start, env.goal]), env, minimum_time=True,
        options={"initial_collocation_points": 1, "maximum_refinements": 4, "maximum_new_points": 16},
        out=tmp_path, validate_callback=continuous_reference)
    assert answer["history"][0]["solver_success"]
    assert not answer["history"][0]["validation_passed"]
    assert answer["history"][0]["minimum_clearance"] < -.019
    assert answer["status"] == "success"
    assert 2 <= answer["rounds"] <= 5
    assert answer["validation"]["minimum_clearance"] >= .001 - 1e-6
    assert answer["candidate"]["duration"] >= 2. - 1e-5


def test_actual_ipopt_rich_time_control(tmp_path):
    env = environment(timed_solver="IPOPT")
    answer = strong_nto.solve(np.array([env.start, env.goal]), env, minimum_time=True,
        options={"initial_collocation_points": 1, "maximum_refinements": 0},
        out=tmp_path, validate_callback=validate)
    assert answer["status"] == "success"
    assert 2. - 1e-5 <= answer["candidate"]["duration"] < 2.15
    assert (tmp_path / "refinements/round_000/ipopt.log").exists()


def test_fixed_endpoint_margin_is_reported_without_displacement(tmp_path):
    env = environment([[0., 0.]], start=(.2505, 0.), goal=(1., 0.))
    path = np.array([env.start, env.goal])
    answer = strong_nto.solve(path, env, minimum_time=False, out=tmp_path, validate_callback=validate)
    assert answer["status"] == "optimization_clearance_infeasible"
    assert not answer["candidate"]["solver_success"]
    assert answer["rounds"] == 0
    assert np.array_equal(answer["candidate"]["controls"], path)


def test_refinement_is_bounded_and_adds_explicit_witnesses(tmp_path):
    env = environment()
    calls = []

    def rejecting_reference(candidate, environment):
        result = validate(candidate, environment)
        result.update(passed=False, normalized_collision_parameters=[.371 + .01 * len(calls)])
        calls.append(candidate["control_points"])
        return result

    answer = strong_nto.solve(np.array([env.start, env.goal]), env, minimum_time=True,
        options={"maximum_refinements": 2, "maximum_new_points": 1,
                 "initial_collocation_points": 5, "maximum_collocation_points": 7},
        out=tmp_path, validate_callback=rejecting_reference)
    assert answer["status"] == "refinement_exhausted"
    assert answer["rounds"] == len(calls) == 3
    assert answer["candidate"]["collocation_points"] == 7
    assert not answer["candidate"]["strong_nto_completed"]
    assert answer["history"][0]["added_collocation_locations"] == pytest.approx([.371])
    assert all(record["refinement_shape_max_error"] < 1e-10 for record in answer["history"][:-1])
    budget_limits = [json.loads((tmp_path / f"refinements/round_{i:03d}/solver_parameters.json").read_text())["Time limit"]
                     for i in range(3)]
    assert budget_limits[0] > budget_limits[1] > budget_limits[2] > 0


def test_real_impossible_fixed_two_node_path_is_not_accepted(tmp_path):
    env = environment([[0., 0.]], start=(-1., 0.), goal=(1., 0.))
    answer = strong_nto.solve(np.array([env.start, env.goal]), env, minimum_time=False,
        options={"maximum_refinements": 0}, out=tmp_path, validate_callback=validate)
    assert answer["status"] == "solver_failed"
    assert not answer["candidate"]["solver_success"]
    assert not answer["validation"]["passed"]
    assert (tmp_path / "refinements/round_000/candidate.npz").exists()


def test_cumulative_budget_includes_validation_and_never_resets(tmp_path, monkeypatch):
    env = environment()
    env.protocol["optimizer_time_limit_seconds"] = 5.
    clock = [0.]
    monkeypatch.setattr(strong_nto.time, "perf_counter", lambda: clock[0])

    def delayed_reference(candidate, environment):
        result = validate(candidate, environment)
        clock[0] = 6.
        return result

    answer = strong_nto.solve(np.array([env.start, env.goal]), env, minimum_time=False,
                              out=tmp_path, validate_callback=delayed_reference)
    assert answer["candidate"]["solver_success"] and answer["validation"]["passed"]
    assert answer["status"] == "budget_exhausted"
    assert not answer["candidate"]["strong_nto_completed"]
    assert answer["rounds"] == 1


def test_feasible_failed_solver_iterate_never_becomes_success(tmp_path, monkeypatch):
    env = environment()
    original = strong_nto._solve_round

    def failed_flag(*args, **kwargs):
        candidate, timing = original(*args, **kwargs)
        candidate.update(solver_success=False, solver_status="test_injected_nonconvergence")
        return candidate, timing

    monkeypatch.setattr(strong_nto, "_solve_round", failed_flag)
    answer = strong_nto.solve(np.array([env.start, env.goal]), env, minimum_time=False,
        options={"maximum_refinements": 1}, out=tmp_path, validate_callback=validate)
    assert answer["validation"]["passed"]
    assert answer["rounds"] == 2
    assert answer["status"] == "solver_failed"
    assert not answer["candidate"]["solver_success"]
    assert not answer["candidate"]["strong_nto_completed"]


def test_new_round_validation_exception_does_not_reuse_previous_check(tmp_path, monkeypatch):
    env = environment()
    calls = []

    def failing_reference(candidate, environment):
        if calls:
            raise ValueError("reference failed on second curve")
        calls.append(True)
        result = validate(candidate, environment)
        result["passed"] = False
        return result

    answer = strong_nto.solve(np.array([env.start, env.goal]), env, minimum_time=True,
        options={"maximum_refinements": 2}, out=tmp_path, validate_callback=failing_reference)
    assert answer["status"] == "numerical_error"
    assert answer["validation"] is None
    assert "reference failed" in answer["history"][-1]["exception"]


def test_witness_filtering_retains_real_interior_parameters():
    candidate = {"knots": np.array([2., 2., 4., 4.]), "spline_order": 2, "duration": 2.}
    validation = {"normalized_collision_parameters": [np.nan, -.1, 0., .25, 1., 1.1],
                  "minimum_clearance": -.1, "normalized_minimum_parameters": [.7],
                  "clearance_witnesses": [{"clearance": .0005, "normalized_parameter": .8},
                                           {"clearance": .1, "normalized_parameter": .9}],
                  "_samples": {"time": np.array([2., 3., np.nan]),
                               "cuda": np.array([True, False, False]),
                               "independent": np.array([True, True, False])}}
    assert strong_nto._witnesses(validation, candidate, .001) == pytest.approx([.25, .5, .7, .8])


@pytest.mark.parametrize("options", [{"maximum_refinements": -1}, {"maximum_new_points": 0},
    {"minimum_control_points": 3}, {"clearance": float("nan")}, {"unknown": 1},
    {"minimum_control_points": 65}, {"initial_collocation_points": 1001}])
def test_invalid_options_rejected(options):
    with pytest.raises(ValueError):
        strong_nto._options(options)
