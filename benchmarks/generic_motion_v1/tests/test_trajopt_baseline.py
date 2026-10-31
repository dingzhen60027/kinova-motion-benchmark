"""Actual native SQP fixtures and mathematical collision-adapter checks."""
import json
import time
from pathlib import Path

import numpy as np
import pytest

from benchmarks.generic_motion_v1.runners import trajopt_baseline as trajopt


class EmptyGeometry:
    lower = np.full(7, -3.)
    upper = np.full(7, 3.)

    def clearance(self, q):
        return np.full(len(q), .5)


def cfg(**values):
    return {"extended_baselines": {"trajopt": values}}


@pytest.mark.parametrize("value", [True, 0, -1, 1.5, "24", 257])
def test_reject_bad_knot_count(value):
    with pytest.raises(ValueError, match="initial_knots"):
        trajopt.settings(cfg(initial_knots=value))


@pytest.mark.parametrize("value", [False, 0, -1, float("nan"), float("inf"), "0.1"])
def test_reject_bad_trust_box(value):
    with pytest.raises(ValueError, match="initial_trust_box_rad"):
        trajopt.settings(cfg(initial_trust_box_rad=value))


def test_cannot_override_method_identity_or_unknown_setting():
    for key in ("algorithm", "author_commit", "typo"):
        with pytest.raises(ValueError, match="Unknown"):
            trajopt.settings(cfg(**{key: "changed"}))


def test_collocation_interpolates_fixed_endpoints_and_every_segment():
    interpolation = trajopt.collocation_matrix(4, 3)
    q = np.arange(28).reshape(4, 7)
    assert interpolation.shape == (10, 4)
    assert np.allclose(interpolation @ q, np.linspace(q[0], q[-1], 10))
    assert np.all(interpolation >= 0)
    assert np.allclose(interpolation.sum(axis=1), 1)


def test_subdivision_retains_original_corners():
    q = np.zeros((3, 7)); q[1, :2] = [1, 1]; q[-1, 0] = 2
    result = trajopt.subdivide_preserving_vertices(q, 9)
    assert len(result) == 9
    for vertex in q:
        assert np.any(np.all(result == vertex, axis=1))
    assert np.isclose(np.linalg.norm(np.diff(result, axis=0), axis=1).sum(),
                      np.linalg.norm(np.diff(q, axis=0), axis=1).sum())


def test_numeric_callback_derivative_matches_known_function():
    class Quadratic(EmptyGeometry):
        def clearance(self, q):
            return .1+np.square(q).sum(axis=1)
    q = np.arange(21).reshape(3, 7)*.01
    d, g = trajopt.signed_distance_jacobian(Quadratic(), q, 1e-5, time.perf_counter()+1)
    assert np.allclose(d, .1+np.square(q).sum(axis=1))
    assert np.allclose(g, 2*q, atol=1e-10)


def test_expired_derivative_deadline_is_failure():
    with pytest.raises(TimeoutError):
        trajopt.signed_distance_jacobian(EmptyGeometry(), np.zeros((2, 7)), 1e-5, 0)


def test_native_author_sqp_reduces_quadratic_cost_and_fixes_endpoints(tmp_path):
    if not trajopt.dependency_probe()["available"]:
        pytest.skip("Run explicit isolated author TrajOpt build before native tests")
    q = np.zeros((3, 7)); q[1, :2] = [.4, .5]; q[-1, 0] = 1
    case = dict(q_start=q[0].tolist(), q_goal=q[-1].tolist())
    result = trajopt.solve(q, EmptyGeometry(), case,
        cfg(initial_knots=4, segment_collocation_subdivisions=1),
        tmp_path, time.perf_counter()+15, 1)
    raw = json.loads((tmp_path/"trajopt_solver_result.json").read_text())
    assert result["solver_success"] and result["solver_status"] == "CONVERGED"
    assert np.array_equal(result["points"][[0, -1]], q[[0, -1]])
    assert result["solver_objective"] < .4
    assert np.isclose(result["cost"], 1, atol=1e-6)
    assert raw["native_statistics"][3] >= 1
    assert list((tmp_path/"trajopt_iterates").glob("accepted_*.npy"))
    assert result["native_objective"] != result["objective"]


def test_wrong_endpoint_never_silently_replaced(tmp_path):
    if not trajopt.dependency_probe()["available"]:
        pytest.skip("Native source build unavailable")
    q = np.zeros((2, 7)); q[-1, 0] = 1
    with pytest.raises(ValueError, match="altered task endpoint"):
        trajopt.solve(q, EmptyGeometry(), dict(q_start=q[0], q_goal=q[-1]+.1),
                     cfg(initial_knots=4), tmp_path, time.perf_counter()+5)


def test_iteration_limit_keeps_only_actual_optimized_incumbent(tmp_path):
    if not trajopt.dependency_probe()["available"]:
        pytest.skip("Native source build unavailable")
    q = np.zeros((3, 7)); q[1, :2] = [.4, .5]; q[-1, 0] = 1
    result = trajopt.solve(q, EmptyGeometry(), {}, cfg(initial_knots=4,
        maximum_sqp_iterations=1, segment_collocation_subdivisions=1),
        tmp_path, time.perf_counter()+5)
    assert result["solver_success"] and not result["solver_converged"]
    assert result["native_status"] == "SCO_ITERATION_LIMIT"
    initial = np.load(tmp_path/"trajopt_initial.npz")["controls"]
    assert not np.array_equal(result["points"], initial)
    assert np.array_equal(result["points"][[0, -1]], q[[0, -1]])


def test_native_roundtrip_endpoint_uses_common_tolerance(tmp_path):
    if not trajopt.dependency_probe()["available"]:
        pytest.skip("Native source build unavailable")
    q = np.zeros((2, 7)); q[-1, 0] = 1
    goal = q[-1].copy(); goal[0] += 1.2e-7
    config = cfg(initial_knots=4, segment_collocation_subdivisions=1)
    config["endpoint_tolerance_rad"] = 1e-5
    result = trajopt.solve(q, EmptyGeometry(), dict(q_start=q[0], q_goal=goal),
                          config, tmp_path, time.perf_counter()+5)
    assert np.array_equal(result["points"][-1], q[-1])
    record = json.loads((tmp_path/"trajopt_parameters.json").read_text())
    assert record["endpoint_error_rad"]["q_goal"] > 1e-7
    assert record["endpoint_tolerance_rad"] == 1e-5


def test_callback_failure_is_saved_not_success(tmp_path):
    if not trajopt.dependency_probe()["available"]:
        pytest.skip("Native source build unavailable")
    class BrokenGeometry(EmptyGeometry):
        def clearance(self, q):
            raise RuntimeError("intentional geometry failure fixture")
    q = np.zeros((2, 7)); q[-1, 0] = 1
    with pytest.raises(RuntimeError, match="intentional geometry failure"):
        trajopt.solve(q, BrokenGeometry(), {}, cfg(initial_knots=4),
                      tmp_path, time.perf_counter()+5)
    saved = json.loads((tmp_path/"trajopt_solver_result.json").read_text())
    assert not saved["solver_success"] and saved["solver_status"] == "exception"
    assert not (tmp_path/"trajopt_solver_raw.npz").exists()


def test_canonical_analytic_gradient_matches_real_geometry_cpu():
    from benchmarks.generic_motion_v1.model import Model
    from benchmarks.generic_motion_v1.runners.canonical_distance import clearance_gradient
    model = Model()
    rng = np.random.Generator(np.random.PCG64(623))
    q = rng.uniform(model.lower*.8, model.upper*.8, (5, 7))
    model.set_voxels([[.5, .2, .4], [.3, -.4, .8], [.4, .1, .2]])
    values, gradient = clearance_gradient(model, q)
    direct = model.clearance(q)
    finite = np.stack([(model.clearance(q+np.eye(7)[j]*1e-6)
                       -model.clearance(q-np.eye(7)[j]*1e-6))/2e-6
                      for j in range(7)], axis=1)
    assert np.array_equal(values, direct)
    assert np.allclose(gradient, finite, atol=2e-6, rtol=2e-4)


def test_saved_online_gradient_probe_covers_penetrating_voxels():
    path = trajopt.HERE/"reports/extended_baselines_v1/trajopt_gradient_online_probe_v2.json"
    rows = json.loads(path.read_text())
    assert len(rows) == 24
    assert any(r["distance"] < -.01 for r in rows)
    assert max(r["distance_error"] for r in rows) < 1e-12
    assert max(r["gradient_error"] for r in rows) < 2e-6
