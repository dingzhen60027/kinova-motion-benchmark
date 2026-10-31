"""Numerical method checks, native execution, and geometry-contract regressions."""
import json
import time

import numpy as np
import pytest

from benchmarks.generic_motion_v1.runners import chomp_stomp as m


class QuadraticClearance:
    lower = np.full(7, -3.)
    upper = np.full(7, 3.)

    def clearance(self, q):
        return .025 + .002*np.sum(np.asarray(q)**2, axis=1)


def seed_path():
    path = np.zeros((4, 7))
    path[:, 0] = [-.8, -.2, .3, .9]
    path[:, 1] = [0., .3, -.2, 0.]
    return path


def test_seed_resampling_preserves_every_vertex_and_exact_length():
    path = seed_path()
    result = m.initial_trajectory(path)
    assert len(result) >= 64
    np.testing.assert_array_equal(result[[0, -1]], path[[0, -1]])
    for vertex in path:
        assert np.min(np.linalg.norm(result-vertex, axis=1)) < 1e-14
    assert np.linalg.norm(np.diff(result, axis=0), axis=1).sum() == pytest.approx(
        np.linalg.norm(np.diff(path, axis=0), axis=1).sum(), abs=1e-12)


@pytest.mark.parametrize("path", [np.zeros((1, 7)), np.zeros((2, 6)),
                                    np.zeros((2, 7)), np.full((2, 7), np.nan),
                                    [[True]*7, [False]*7]])
def test_seed_invalid_input_rejected(path):
    with pytest.raises(ValueError):
        m.initial_trajectory(path)


def test_acceleration_covariance_and_boundary_conditions():
    d2, metric, inverse = m.acceleration_metric(14)
    assert np.linalg.eigvalsh(metric).min() > 0
    np.testing.assert_allclose(metric@inverse, np.eye(12), atol=1e-11)
    assert np.linalg.norm(d2@np.linspace(0, 1, 14)) < 1e-14
    assert np.linalg.norm(d2@np.arange(14)**2-np.full(12, 2.)) < 1e-14


def test_chomp_potential_values_slopes_and_contact_continuity():
    values, slope = m.obstacle_potential(np.array([-.02, 0., .025, .05, .08]), .05)
    np.testing.assert_allclose(values, [.045, .025, .00625, 0., 0.])
    np.testing.assert_allclose(slope, [-1., -1., -.5, 0., 0.])
    for location in [0., .05]:
        h = 1e-7
        vp = m.obstacle_potential([location+h], .05)[0][0]
        vm = m.obstacle_potential([location-h], .05)[0][0]
        analytic = m.obstacle_potential([location], .05)[1][0]
        assert (vp-vm)/(2*h) == pytest.approx(analytic, abs=1e-5)


def test_chomp_whole_objective_gradient_against_independent_finite_differences():
    rng = np.random.default_rng(73)
    q = rng.normal(0, .2, (10, 7))
    model = QuadraticClearance()
    cost, analytic, _ = m.discrete_cost_gradient(q, model.clearance(q), .004*q, m.CHOMP_PROFILE)
    numeric = np.zeros_like(q)
    step = 1e-6
    for i in range(len(q)):
        for j in range(7):
            plus, minus = q.copy(), q.copy()
            plus[i, j] += step
            minus[i, j] -= step
            fp = m.discrete_cost_gradient(plus, model.clearance(plus), None, m.CHOMP_PROFILE)[0]
            fm = m.discrete_cost_gradient(minus, model.clearance(minus), None, m.CHOMP_PROFILE)[0]
            numeric[i, j] = (fp-fm)/(2*step)
    assert np.isfinite(cost)
    np.testing.assert_allclose(analytic, numeric, rtol=1e-7, atol=2e-8)


def test_clearance_gradient_keeps_canonical_callback():
    model = QuadraticClearance()
    q = seed_path()
    task = m.GeometryTask(model, m.CHOMP_PROFILE, time.perf_counter()+5)
    gradient = task.gradient(q)
    np.testing.assert_allclose(gradient, .004*q, atol=1e-11)
    assert task.clearance_states == 14*len(q)


@pytest.mark.parametrize("method", ["CHOMP", "STOMP"])
def test_real_optimizers_produce_evidence_and_preserve_boundary(method, tmp_path):
    cfg = {"extended_baselines": {method.lower(): dict(initial_knots=16, maximum_iterations=3)}}
    result = m.solve(seed_path(), QuadraticClearance(), {"_objective": "minimum_distance"},
                     cfg, tmp_path, time.perf_counter()+10, 98231, method)
    np.testing.assert_array_equal(result["points"][[0, -1]], seed_path()[[0, -1]])
    assert result["solver_success"]
    assert np.isfinite(result["native_cost"])
    record = json.loads((tmp_path / "optimizer_result.json").read_text())
    assert record["iterations"] > 0
    assert record["clearance_states"] > len(result["points"])
    assert record["candidate_changed"]
    if method == "STOMP":
        raw = np.load(tmp_path / "stomp_raw.npz")
        assert len(raw["noise"]) >= 12
        assert np.max(np.abs(raw["noise"][:, [0, -1]])) == 0
        assert json.loads((tmp_path / "stomp_dependencies.json").read_text())["upstream_core_modified"] is False
    else:
        assert record["native_cost"] < record["initial_native_cost"]


def test_native_stomp_seed_replay_is_deterministic(tmp_path):
    cfg = {"extended_baselines": {"stomp": dict(initial_knots=16, maximum_iterations=3)}}
    a = m.solve(seed_path(), QuadraticClearance(), {}, cfg, tmp_path / "a", time.perf_counter()+10, 194, "STOMP")
    b = m.solve(seed_path(), QuadraticClearance(), {}, cfg, tmp_path / "b", time.perf_counter()+10, 194, "STOMP")
    np.testing.assert_array_equal(a["points"], b["points"])
    ra, rb = np.load(tmp_path / "a/stomp_raw.npz"), np.load(tmp_path / "b/stomp_raw.npz")
    np.testing.assert_array_equal(ra["noise"], rb["noise"])


@pytest.mark.parametrize("method", ["CHOMP", "STOMP"])
def test_deadline_is_failure_with_saved_reason(method, tmp_path):
    with pytest.raises(TimeoutError):
        m.solve(seed_path(), QuadraticClearance(), {}, {}, tmp_path, time.perf_counter()-1, 1, method)
    record = json.loads((tmp_path / "optimizer_failure.json").read_text())
    assert "TimeoutError" in record["exception"]
    assert (tmp_path / "optimizer_initial.npz").exists()


@pytest.mark.parametrize("settings", [{"maximum_iterations": 0}, {"rollouts": 1},
                                        {"noise_std_rad": float("nan")}, {"bogus": 3},
                                        {"initial_knots": 2}, {"rollouts": True},
                                        {"maximum_iterations": 2.2}])
def test_invalid_profiles_rejected(settings):
    with pytest.raises(ValueError):
        m._profile("STOMP", {"extended_baselines": {"stomp": settings}})


def test_wrong_objective_rejected_before_solver(tmp_path):
    with pytest.raises(ValueError, match="distance"):
        m.solve(seed_path(), QuadraticClearance(), {"_objective": "minimum_time"}, {}, tmp_path,
                time.perf_counter()+10, 1, "CHOMP")
