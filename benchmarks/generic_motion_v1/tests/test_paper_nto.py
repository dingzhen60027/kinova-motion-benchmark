"""Real CPU solver regressions for the declared paper NTO reconstruction."""
import json
import re
import time
from types import SimpleNamespace

import numpy as np
import pytest

from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import HERE, load_config
from benchmarks.generic_motion_v1.runners.paper_nto import (
    dependency_probe, paper_nto, warm_start,
)
from benchmarks.generic_motion_v1.runners.validate import sample_candidate


@pytest.fixture(scope="module")
def canonical():
    # No CUDA collision checker or planner is called in this test module.
    return Model()


@pytest.fixture
def local_task(canonical):
    frozen = json.loads((HERE / "manifests/dev.jsonl").read_text().splitlines()[0])
    start = np.array(frozen["q_start"])
    goal = start.copy()
    goal[0] += 0.02
    path = np.array([start, goal])
    assert canonical.clearance(np.linspace(start, goal, 17)).min() > 0.01
    # A synthetic small motion is a CPU regression, never a benchmark case.
    return path, dict(q_start=start.tolist(), q_goal=goal.tolist())


@pytest.mark.parametrize("objective", ["minimum_distance", "minimum_time"])
def test_actual_published_solver_dependency_probe(tmp_path, objective):
    record = dependency_probe(tmp_path, objective)
    assert record["probe_success"]
    assert record["probe_solution"] == pytest.approx(1.0, abs=1e-6)
    assert record["solver"] == ("SNOPT" if objective == "minimum_distance" else "IPOPT")
    assert json.loads((tmp_path / "paper_nto_dependencies.json").read_text())["probe_success"]


@pytest.mark.parametrize("count", [2, 3, 17])
@pytest.mark.parametrize("objective", ["minimum_distance", "minimum_time"])
def test_no_drm_nodes_are_resampled_pruned_or_inserted(count, objective):
    path = np.zeros((count, 7))
    path[:, 0] = np.linspace(0, 1, count)
    path[:, 1] = np.sin(np.linspace(0, 4, count))
    original = path.copy()
    initial, saved, controls, order, duration = warm_start(path, objective)
    np.testing.assert_array_equal(path, original)
    np.testing.assert_array_equal(saved, original)
    if objective == "minimum_time":
        np.testing.assert_array_equal(controls[1:-1], original)
        np.testing.assert_array_equal(controls[0], original[0])
        np.testing.assert_array_equal(controls[-1], original[-1])
        assert len(initial.control_points()) == count + 2
        assert order == 4
        derivative = initial.MakeDerivative(1)
        np.testing.assert_allclose(derivative.value(0), 0, atol=1e-13)
        np.testing.assert_allclose(derivative.value(duration), 0, atol=1e-13)
    else:
        np.testing.assert_array_equal(controls, original)
        assert len(initial.control_points()) == count
        assert order == 2 and duration == 1.0


@pytest.mark.parametrize("objective", ["minimum_distance", "minimum_time"])
def test_real_native_collision_transcription_solves_and_preserves_artifacts(canonical, local_task, tmp_path, objective):
    path, case = local_task
    case["_objective"] = objective
    cfg = load_config()
    # These legacy adapter knobs must not silently strengthen this method.
    cfg["nto"].update(minimum_control_points=100, collocation_points=3,
                      maximum_iterations=1, clearance_m=100, solver="unavailable")
    candidate = paper_nto(path, canonical, case, cfg, tmp_path, time.perf_counter() + 30)
    assert candidate["solver_success"], candidate
    assert candidate["solver"] == ("SNOPT" if objective == "minimum_distance" else "IPOPT")
    assert len(candidate["controls"]) == (2 if objective == "minimum_distance" else 4)
    samples = sample_candidate(candidate, load_config())
    np.testing.assert_allclose(samples["q"][[0, -1]], path, atol=1e-7, rtol=0)
    assert canonical.clearance(samples["q"][::max(1, len(samples["q"]) // 20)]).min() > 0.01
    if objective == "minimum_time":
        # A single cubic with duplicated endpoints has q'' extrema 6D/T^2.
        # For D=.02 and |q''|<=1, the exact minimum is sqrt(.12).
        assert candidate["duration"] == pytest.approx(np.sqrt(0.12), abs=1e-5)
        assert samples["derivatives"]["endpoint_velocity"] < 1e-6
        assert np.max(samples["derivatives"]["acceleration_per_joint"]) <= 1.0 + 1e-5
    else:
        assert samples["cost"] == pytest.approx(0.02, abs=1e-6)
    parameters = json.loads((tmp_path / "nto_parameters.json").read_text())
    assert parameters["collocation_parameter_Nc"] == 150
    assert parameters["collocation_points"] == 149
    np.testing.assert_array_equal(parameters["collocation_locations"], np.arange(1, 150) / 150)
    assert parameters["legacy_cfg_nto_used"] is False
    assert parameters["collision_pairs"] == 137
    assert parameters["profile"]["clearance_m"] == 0.0
    assert parameters["profile"]["maximum_iterations"] == 2000
    assert "149 bindings" in parameters["profile"]["sources"]["collocation"]
    with np.load(tmp_path / "nto_solver_raw.npz", allow_pickle=False) as saved:
        np.testing.assert_array_equal(saved["controls"], candidate["controls"])
        np.testing.assert_array_equal(saved["normalized_knots"] * saved["duration"], candidate["knots"])
        assert saved["decision_variables"].size > 0
    assert (tmp_path / ("ipopt.log" if objective == "minimum_time" else "snopt.log")).is_file()
    if objective == "minimum_distance":
        # Exercise the real solver: a file's existence alone previously passed
        # even though the direct Print file option wrote only a 121-byte header.
        log = (tmp_path / "snopt.log").read_text()
        assert "No. of iterations" in log and "No. of major iterations" in log
        for label, value in (("Iteration limit", 10000),
                             ("Major iterations limit", 2000),
                             ("Minor iterations limit", 500)):
            assert re.search(re.escape(label) + r"\.+\s+" + str(value) + r"\b", log)
        assert "SNOPTA INFO" in log
        assert "Print file" not in parameters["parameters"]
        assert "Iterations limit" not in parameters["parameters"]
        assert "Minor iterations limit" not in parameters["parameters"]
        assert parameters["parameters"]["Major iterations limit"] == 2000
        assert parameters["parameters"]["Major feasibility tolerance"] == 1e-6
        assert parameters["parameters"]["Major optimality tolerance"] == 1e-6
        assert parameters["solver_logging"]["interface"] == "SolverOptions/CommonSolverOption.kPrintFileName"
        observation = parameters["reported_iteration_limits"]
        assert observation["values"] == {"Iterations limit": 10000,
                                          "Major iterations limit": 2000,
                                          "Minor iterations limit": 500}
        assert observation["explicitly_configured"] == ["Major iterations limit"]
        assert observation["defaults_overridden"] is False
        assert "not a fresh runtime query" in observation["scope"]
        assert observation["evidence"].endswith("common_file_rows164/snopt.log")
    else:
        assert parameters["reported_iteration_limits"] is None
        assert parameters["solver_logging"]["interface"] == "IPOPT output_file"


def test_real_infeasible_snopt_keeps_failed_native_candidate(canonical, local_task, tmp_path):
    path, case = local_task
    case["_objective"] = "minimum_distance"
    upper = canonical.upper.copy()
    upper[0] = path[0, 0] - 0.1  # Intentionally inconsistent synthetic unit test.
    impossible = SimpleNamespace(lower=canonical.lower, upper=upper, nto_plant=canonical.nto_plant)
    candidate = paper_nto(path, impossible, case, load_config(), tmp_path, time.perf_counter() + 30)
    assert candidate["solver_success"] is False
    assert candidate["solver_status"] != "SolutionResult.kSolutionFound"
    assert candidate["controls"].shape == (2, 7)
    assert np.isfinite(candidate["controls"]).all()
    with np.load(tmp_path / "nto_solver_raw.npz", allow_pickle=False) as raw:
        assert raw["decision_variables"].size > 0
        np.testing.assert_array_equal(raw["controls"], candidate["controls"])
    result = json.loads((tmp_path / "nto_solver_result.json").read_text())
    assert result["solver_success"] is False
    assert "info" in result["native_solver_details"]
    # Logging must also preserve the unsuccessful solver's native termination.
    log = (tmp_path / "snopt.log").read_text()
    assert "SNOPTA INFO" in log and "No. of iterations" in log


def test_expired_online_budget_preserves_initial_and_parameters(canonical, local_task, tmp_path):
    path, case = local_task
    case["_objective"] = "minimum_distance"
    with pytest.raises(TimeoutError, match="exhausted"):
        paper_nto(path, canonical, case, load_config(), tmp_path, time.perf_counter() - 1)
    assert (tmp_path / "nto_initial.npz").is_file()
    parameters = json.loads((tmp_path / "nto_parameters.json").read_text())
    assert parameters["parameters"]["Time limit"] < 0
    assert not (tmp_path / "nto_solver_raw.npz").exists()


@pytest.mark.parametrize("path", [np.zeros((1, 7)), np.zeros((2, 6)),
                                 np.full((2, 7), np.nan), [[False] * 7] * 2])
def test_malformed_warm_start_is_rejected(path):
    with pytest.raises(ValueError):
        warm_start(path, "minimum_time")


def test_moving_endpoints_or_frozen_limits_is_rejected_before_native_construction(local_task, tmp_path):
    path, case = local_task
    case["_objective"] = "minimum_distance"
    def forbidden():
        raise AssertionError("Invalid task must not construct native optimizer geometry")
    model = SimpleNamespace(nto_plant=forbidden)
    bad = path.copy()
    bad[0, 1] += 0.1
    with pytest.raises(ValueError, match="endpoints differ"):
        paper_nto(bad, model, case, load_config(), tmp_path, time.perf_counter() + 30)
    cfg = load_config()
    cfg["velocity_limit_rad_s"] = 2.0
    with pytest.raises(ValueError, match="frozen unit"):
        paper_nto(path, model, case, cfg, tmp_path, time.perf_counter() + 30)
