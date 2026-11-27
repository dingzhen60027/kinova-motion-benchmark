import numpy as np
import pytest
from scipy.optimize import minimize

from benchmarks.generic_motion_v1.runners.phase_search import clock_state, rest_times
from experiments.joint_timing.witness_clocks import clock_sensitivity, propose


def setup():
    start = np.zeros(7)
    goal = np.array([3.5, -1.5, 0.5, -0.25, 0.0, 1.0, 0.1])
    base, ramp = rest_times(goal - start, np.ones(7), np.ones(7))
    return start, goal, max(base), base, ramp


def sample(args, width, shift, t):
    start, goal, T, base, ramp = args
    span = np.where(base > 0, base + width * (T - base), 0.0)
    delay = shift * (T - span)
    r = np.divide(ramp, base, out=np.zeros(7), where=base > 0) * span
    return clock_state(t, start, goal - start, delay, span, r)[0]


@pytest.mark.parametrize("t", [0.01, 0.7, 1.5, 2.25, 3.31, 4.48])
def test_clock_jacobian_matches_central_difference(t):
    args = setup()
    w = np.linspace(0.2, 0.6, 7)
    s = np.linspace(0.3, 0.7, 7)
    dw, ds = clock_sensitivity(*args, w, s, t)
    for i in range(7):
        # At acceleration switches the centered difference has a first-order error.
        epsilon = 2.0e-8
        e = np.eye(7)[i] * epsilon
        for a, b, analytic in [(e, 0, dw), (0, e, ds)]:
            finite = (sample(args, w + a, s + b, t) - sample(args, w - a, s - b, t)) / (
                2 * epsilon
            )
            np.testing.assert_allclose(finite, np.eye(7)[i] * analytic[i], atol=2e-8)
    assert dw[0] == ds[0] == dw[4] == ds[4] == 0.0


def test_box_projection_is_minimum_norm():
    args = setup()
    w = np.linspace(0.02, 0.3, 7)
    s = np.linspace(0.3, 0.9, 7)
    t = 2.3
    grad = np.array([0.01, 0.1, -0.3, 0.2, 0.01, 0.16, 0.35])
    cw, cs = clock_sensitivity(*args, w, s, t)
    g = np.r_[grad * cw, grad * cs]
    x = np.r_[w, s]
    budget = 0.032
    actual = (
        np.concatenate(propose(*args, w, s, t, grad, clearance_m=0.002 - budget)) - x
    )
    limits = list(zip(np.maximum(-0.35, -x), np.minimum(0.35, 1 - x)))
    result = minimize(
        lambda y: 0.5 * (y @ y),
        np.zeros_like(x),
        jac=lambda y: y,
        constraints=[dict(type="ineq", fun=lambda y: g @ y - budget, jac=lambda y: g)],
        bounds=limits,
        method="SLSQP",
        options=dict(ftol=1e-12),
    )
    assert result.success and g @ actual == pytest.approx(budget)
    np.testing.assert_allclose(actual, result.x, atol=2e-7)


@pytest.mark.parametrize(
    "gradient, clearance", [(np.zeros(7), -1.0), (np.ones(7), 0.05)]
)
def test_no_direction_or_clearance_already_met(gradient, clearance):
    args = setup()
    w = np.full(7, 0.3)
    s = np.full(7, 0.7)
    nw, ns = propose(*args, w, s, 2.0, gradient, clearance_m=clearance)
    np.testing.assert_equal([nw, ns], [w, s])


def test_unreachable_linearized_target_stays_inside_box_and_preserves_critical_axes():
    args = setup()
    w = np.linspace(0.01, 0.99, 7)
    s = w[::-1].copy()
    g = np.ones(7)
    nw, ns = propose(*args, w, s, 2.3, g, clearance_m=-1000.0)
    delta = np.r_[nw - w, ns - s]
    a, b = clock_sensitivity(*args, w, s, 2.3)
    assert np.r_[a, b] @ delta > 0
    assert np.max(np.abs(delta)) <= 0.35000001
    assert ((np.r_[nw, ns] >= 0) & (np.r_[nw, ns] <= 1)).all()
    np.testing.assert_equal([nw[0], ns[0], nw[4], ns[4]], [w[0], s[0], w[4], s[4]])
