"""CPU regressions for whole-curve Forest distance checks and failure evidence."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.interpolate import BSpline

from research_platform import continuous_checks as checks


def environment(centers=((.4137, 0.),), obstacle_radius=.0001, point_radius=.000001):
    return SimpleNamespace(kind="forest", cfg={"obstacle_radius": obstacle_radius,
        "point_radius": point_radius, "feasibility_tolerance": 1e-5},
        base=SimpleNamespace(centers=np.asarray(centers, dtype=float),
                             radius=obstacle_radius+point_radius))


def cubic_line(kind="bspline", start=0., end=1., y=0.):
    controls = np.column_stack([np.linspace(0., 1., 4), np.full(4, y)])
    if kind == "bspline":
        return dict(kind=kind, spline_order=4, controls=controls,
                    knots=np.r_[np.full(4, start), np.full(4, end)], duration=end-start)
    return dict(kind=kind, controls=controls[None, :, :], durations=np.array([end-start]))


@pytest.mark.parametrize("kind", ["bspline", "bezier"])
def test_cubic_thin_obstacle_between_sample_grid_is_found(kind):
    env = environment()
    samples = np.column_stack([np.linspace(0., 1., 101), np.zeros(101)])
    assert np.min(np.linalg.norm(samples-env.base.centers[0], axis=1)) > env.base.radius
    result = checks.forest_curve_check(cubic_line(kind), env)
    assert result["resolved"] and not result["collision_free"]
    assert result["status"] == "collision"
    assert result["minimum_clearance"] == pytest.approx(-env.base.radius, abs=1e-12)
    assert result["normalized_collision_parameters"] == pytest.approx([.4137], abs=1e-12)
    assert np.allclose(result["collision_points"], [[.4137, 0.]], atol=1e-12)
    assert result["method"] == "piecewise_polynomial_stationary_points_float64"
    assert "not_interval_certificate" in result["guarantee"]


@pytest.mark.parametrize("kind", ["bspline", "bezier"])
def test_clear_cubic_curve_and_positive_margin_witness(kind):
    env = environment(obstacle_radius=.2, point_radius=.01)
    candidate = cubic_line(kind, y=1.)
    before = deepcopy(candidate)
    result = checks.forest_curve_check(candidate, env)
    assert result["resolved"] and result["collision_free"]
    assert result["minimum_clearance"] == pytest.approx(.79, abs=1e-12)
    assert result["minimum_squared_center_distance"] == pytest.approx(1.)
    assert result["normalized_collision_parameters"] == []
    assert result["clearance_witnesses"][0]["normalized_parameter"] == pytest.approx(.4137)
    assert np.array_equal(candidate["controls"], before["controls"])


@pytest.mark.parametrize("kind", ["bspline", "bezier"])
def test_genuinely_curved_cubic_thin_crossing_between_grid_points(kind):
    # q(u) = (u, .3 u^2 + .2 u^3), represented in a cubic Bernstein basis.
    controls = np.array([[0., 0.], [1/3, 0.], [2/3, .1], [1., .5]])
    parameter = .4137
    center = (parameter, .3*parameter**2+.2*parameter**3)
    env = environment(centers=(center,), obstacle_radius=1e-5, point_radius=1e-6)
    grid = np.linspace(0., 1., 101)
    samples = np.column_stack([grid, .3*grid**2+.2*grid**3])
    assert np.min(np.linalg.norm(samples-center, axis=1)) > env.base.radius
    candidate = cubic_line(kind)
    candidate["controls"] = controls if kind == "bspline" else controls[None]
    result = checks.forest_curve_check(candidate, env)
    assert result["resolved"] and not result["collision_free"]
    assert result["minimum_clearance"] == pytest.approx(-env.base.radius, abs=1e-12)
    assert result["normalized_collision_parameters"] == pytest.approx([parameter], abs=1e-12)


def test_polygon_matches_independent_projection_with_duplicate_waypoints():
    points = np.array([[-1., 0.], [-1., 0.], [1., 0.], [1., 2.]])
    env = environment(centers=((.3, .1), (2., 3.)), obstacle_radius=.2, point_radius=.03)
    first, delta = points[:-1], np.diff(points, axis=0)
    length2 = np.sum(delta**2, axis=1)
    projection = np.sum((env.base.centers[None]-first[:, None])*delta[:, None], axis=2)
    fractions = np.clip(projection/np.maximum(length2[:, None], 1e-30), 0., 1.)
    closest = first[:, None]+fractions[:, :, None]*delta[:, None]
    expected = np.min(np.linalg.norm(closest-env.base.centers[None], axis=2))-env.base.radius
    result = checks.forest_curve_check(dict(kind="polygon", points=points), env)
    assert result["resolved"]
    assert result["minimum_clearance"] == pytest.approx(expected, abs=1e-12)
    assert result["collision_points"] == [[pytest.approx(.3), 0.]]
    assert result["normalized_collision_parameters"] == pytest.approx([(1+.65)/3])
    assert result["diagnostics"]["pieces"] == 3


def test_shifted_bspline_active_interval_and_internal_repeated_knots():
    # A piecewise linear B-spline with nonuniform spans and shifted time.
    candidate = dict(kind="bspline", controls=np.array([[0., 0.], [1., 0.], [2., 0.]]),
        spline_order=2, knots=np.array([5., 5., 7., 9., 9.]), duration=4.)
    env = environment(centers=((1.25, 0.),))
    result = checks.forest_curve_check(candidate, env)
    assert result["resolved"] and not result["collision_free"]
    assert result["parameter_domain"] == [5., 9.]
    assert result["normalized_collision_parameters"] == pytest.approx([.625])
    assert result["diagnostics"]["pieces"] == 2


def test_shifted_cubic_bspline_parameter_is_normalized_not_raw_time():
    result = checks.forest_curve_check(cubic_line(start=23., end=27.), environment())
    assert result["resolved"]
    assert result["normalized_collision_parameters"] == pytest.approx([.4137], abs=1e-12)


def test_bezier_nonuniform_durations_determine_global_witness_parameter():
    first = np.column_stack([np.linspace(0., 1., 4), np.zeros(4)])
    second = np.column_stack([np.linspace(1., 2., 4), np.zeros(4)])
    candidate = dict(kind="bezier", controls=np.array([first, second]), durations=np.array([1., 3.]))
    result = checks.forest_curve_check(candidate, environment(centers=((1.25, 0.),)))
    assert result["resolved"]
    assert result["normalized_collision_parameters"] == pytest.approx([.4375])


@pytest.mark.parametrize("kind", ["polygon", "bezier", "bspline"])
def test_constant_zero_degree_or_zero_length_curve_is_handled(kind):
    candidate = {
        "polygon": dict(kind="polygon", points=np.array([[2., 0.], [2., 0.]])),
        "bezier": dict(kind="bezier", controls=np.array([[[2., 0.]]]), durations=np.array([1.])),
        "bspline": dict(kind="bspline", controls=np.array([[2., 0.]]), spline_order=1,
                        knots=np.array([3., 7.]), duration=4.),
    }[kind]
    result = checks.forest_curve_check(candidate, environment(centers=((0., 0.),), obstacle_radius=.2))
    assert result["resolved"] and result["collision_free"]
    assert result["minimum_clearance"] == pytest.approx(2.-.200001)
    assert result["diagnostics"]["stationary_roots"] == 0


def test_tangency_allowed_but_solver_tolerance_does_not_hide_penetration():
    env = environment(centers=((.5, 0.),), obstacle_radius=.125, point_radius=.125)
    tangent = checks.forest_curve_check(cubic_line(y=.25), env)
    assert tangent["resolved"] and tangent["collision_free"] and tangent["near_contact"]
    assert tangent["minimum_clearance"] == 0.
    penetrating = checks.forest_curve_check(cubic_line(y=.25-1e-7), env)
    assert penetrating["resolved"] and not penetrating["collision_free"]
    assert penetrating["minimum_clearance"] < 0
    assert abs(penetrating["minimum_clearance"]) < env.cfg["feasibility_tolerance"]


def test_curved_cubic_matches_dense_minimum_and_finds_an_interior_stationary_point():
    controls = np.array([[0., 0.], [.1, 1.3], [.9, -1.1], [1., .2]])
    candidate = dict(kind="bspline", controls=controls, spline_order=4,
                     knots=np.r_[np.zeros(4), np.ones(4)], duration=1.)
    env = environment(centers=((.35, .1), (.7, -.1)), obstacle_radius=.08)
    result = checks.forest_curve_check(candidate, env)
    curve = BSpline(candidate["knots"], controls, 3)(np.linspace(0., 1., 100001))
    dense_min = np.min(np.linalg.norm(curve[:, None]-env.base.centers[None], axis=2))-env.base.radius
    assert result["resolved"]
    assert result["minimum_clearance"] <= dense_min+1e-12
    assert result["minimum_clearance"] == pytest.approx(dense_min, abs=1e-8)


@pytest.mark.parametrize("change", ["nonfinite", "duration", "negative_duration", "high_degree"])
def test_invalid_input_is_unresolved_and_never_accepted(change):
    candidate = cubic_line()
    if change == "nonfinite":
        candidate["controls"][1, 0] = np.nan
    elif change == "duration":
        candidate["duration"] = 2.
    elif change == "negative_duration":
        candidate = cubic_line("bezier")
        candidate["durations"][0] = -1.
    else:
        candidate = dict(kind="bezier", controls=np.zeros((1, 20, 2)), durations=np.array([1.]))
    result = checks.forest_curve_check(candidate, environment())
    assert not result["resolved"] and not result["collision_free"]
    assert result["status"] == "unresolved" and result["diagnostics"]["issues"]


def test_root_failure_is_preserved_as_unresolved(monkeypatch):
    def fail(*args, **kwargs):
        raise np.linalg.LinAlgError("injected eigensolver failure")
    monkeypatch.setattr(checks.poly, "polyroots", fail)
    result = checks.forest_curve_check(cubic_line(y=1.), environment())
    assert not result["resolved"] and not result["collision_free"]
    assert "injected eigensolver failure" in result["diagnostics"]["issues"][0]["exception"]


def test_nearly_real_root_is_not_silently_discarded(monkeypatch):
    monkeypatch.setattr(checks.poly, "polyroots", lambda polynomial: np.array([.5+1e-6j]))
    result = checks.forest_curve_check(cubic_line(y=1.), environment())
    assert not result["resolved"] and not result["collision_free"]
    assert any(issue["reason"] == "ambiguous_near_real_stationary_root"
               for issue in result["diagnostics"]["issues"])


def test_disconnected_bezier_pieces_are_retained_as_unresolved():
    candidate = dict(kind="bezier", controls=np.array([[[0., 1.]], [[1., 1.]]]),
                     durations=np.array([1., 1.]))
    result = checks.forest_curve_check(candidate, environment())
    assert not result["resolved"] and not result["collision_free"]
    assert result["diagnostics"]["continuity_residual"] == 1.
    assert any(issue["reason"] == "discontinuous_curve" for issue in result["diagnostics"]["issues"])


def test_geometry_radius_disagreement_is_not_silently_accepted():
    env = environment()
    env.base.radius += .1
    result = checks.forest_curve_check(cubic_line(y=1.), env)
    assert not result["resolved"] and not result["collision_free"]
    assert "effective radius disagrees" in result["diagnostics"]["issues"][0]["exception"]
