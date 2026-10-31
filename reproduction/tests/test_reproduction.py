"""Numerical and native-GPU checks needed before measuring paper-style runs."""

from pathlib import Path
import numpy as np
from pydrake.geometry.optimization import HPolyhedron
from reproduction.common import config
from reproduction.forest import Forest
from reproduction.pipeline import Pipeline, validate
from reproduction.optimization import nto, lscs, lgcs, bezier_values
import csdecomp as csd


def test_cuda_matches_independent_forest_geometry():
    f = Forest(config(), 3)
    points = np.random.default_rng(9).uniform(-5, 5, (20000, 2))
    cuda = np.array(csd.CheckCollisionFreeCuda(points.T, f.minimal), bool)
    analytic = f.clearances(points) >= 0
    assert np.array_equal(cuda, analytic)


def test_analytic_segment_checker_catches_unsampled_circle():
    c = config()
    c["obstacles"] = 1
    c["obstacle_center_lower"] = [0, 0]
    c["obstacle_center_upper"] = [0, 0]
    f = Forest(c, 0)
    distances, closest = f.segment_clearances(np.array([[-4, 0], [4, 0]]))
    assert abs(distances[0] + f.radius) < 1e-12
    assert np.allclose(closest[0], [0, 0])
    assert (f.clearances([[-4, 0], [4, 0]]) > 0).all()


def test_both_conic_planners_match_straight_line_optimum():
    region = HPolyhedron.MakeBox([-5, -5], [5, 5])
    start = np.array([-4.0, -4.0])
    goal = np.array([4.0, 3.0])
    expected = np.linalg.norm(goal - start)
    for solver in (lscs, lgcs):
        result = solver([region], start, goal)
        assert result["solver_success"]
        assert abs(result["objective"] - expected) < 1e-5
        assert np.allclose(result["points"][0], start, atol=1e-6)
        assert np.allclose(result["points"][-1], goal, atol=1e-6)


def test_lgcs_handles_alternative_corridors_without_relaxation():
    # Two top/bottom routes around a missing central strip.
    regions = [
        HPolyhedron.MakeBox([-5, -5], [-1, 5]),
        HPolyhedron.MakeBox([-5, 1], [5, 5]),
        HPolyhedron.MakeBox([-5, -5], [5, -1]),
        HPolyhedron.MakeBox([1, -5], [5, 5]),
    ]
    r = lgcs(regions, np.array([-4.0, 0]), np.array([4.0, 0]))
    assert r["solver_success"] and r["gap"] <= 1e-6
    assert len(r["selected_regions"]) == 3
    assert abs(r["objective"] - (2 * np.sqrt(10) + 2)) < 1e-4


def test_bezier_derivatives_include_segment_durations():
    controls = np.array(
        [np.linspace([0, 0], [2, 0], 7), np.linspace([2, 0], [2, 4], 7)]
    )
    q, index = bezier_values(
        controls, np.array([2.0, 4.0]), np.array([0.0, 1.0, 2.0, 4.0, 6.0])
    )
    v, _ = bezier_values(controls, np.array([2.0, 4.0]), np.array([1.0, 4.0]), 1)
    assert np.allclose(q, [[0, 0], [1, 0], [2, 0], [2, 2], [2, 4]])
    assert np.allclose(v, [[1, 0], [0, 1]])


def test_native_eizo_recovery_excludes_known_collision(tmp_path):
    cfg = config()
    cfg["obstacles"] = 1
    cfg["obstacle_center_lower"] = [0, 0]
    cfg["obstacle_center_upper"] = [0, 0]
    cfg["start"] = [-4.0, 0.0]
    cfg["goal"] = [4.0, 0.0]
    f = Forest(cfg, 0)
    path = np.array([[-4.0, 0.0], [-1.0, 1.0], [1.0, 1.0], [4.0, 0.0]])

    class DeliberatelyUnsafeInitialRegion(Pipeline):
        def initial(self, path):
            return [HPolyhedron.MakeBox([-5, -5], [5, 5])], [(path[0], path[1])]

    result = DeliberatelyUnsafeInitialRegion(f, 13).solve("LSCS", path, None, tmp_path)
    assert result["recovery_iterations"] > 0
    assert result["history"][0]["collisions"] > 0
    assert result["status"] == "success_numerical"
    assert result["validation"]["minimum_clearance"] >= 0


def test_nto_free_space_respects_real_time_derivative_bounds():
    cfg = config()
    cfg["obstacles"] = 1
    cfg["obstacle_center_lower"] = [-3, 3]
    cfg["obstacle_center_upper"] = [-3, 3]
    f = Forest(cfg, 0)
    path = np.array([cfg["start"], cfg["goal"]])
    candidate = nto(path, f, True)
    candidate["method"] = "NTO_time"
    result = validate(candidate, f)
    assert candidate["solver_success"] and result["passed"]
    # Bang-bang lower bound for an 8m displacement, unit v/a, rest endpoints.
    assert 9 - 1e-5 <= result["duration"] < 10
    assert result["derivative_bounds"]["velocity"] <= 1 + 1e-5
    assert result["derivative_bounds"]["acceleration"] <= 1 + 1e-5
