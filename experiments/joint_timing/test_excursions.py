import numpy as np
import pytest
from benchmarks.generic_motion_v1.runners.phase_search import rest_times
from benchmarks.generic_motion_v1.runners.validate import sample_candidate
from experiments.joint_timing.excursions import proposals, radius, via_intervals


def test_radius_and_critical_axis():
    limit = np.ones(7)
    np.testing.assert_allclose(radius(1.0, limit, limit), 0.25)
    np.testing.assert_allclose(radius(2.0, limit, limit), 1.0)
    np.testing.assert_allclose(radius(4.0, limit, limit), 3.0)
    a = np.zeros(7)
    b = np.array([3.0, 0.0, 0.2, -0.5, 0.3, 0.0, 0.0])
    lo, hi = via_intervals(a, b, 4.0, 2.0, -3 * limit, 3 * limit, limit, limit)
    assert lo[0] > hi[0] and lo[1] == -1.0 and hi[1] == 1.0
    with pytest.raises(ValueError):
        via_intervals(a, b, 4.0, 0.0, -3 * limit, 3 * limit, limit, limit)


def test_exact_reversal_bounds_and_endpoints():
    limit = np.ones(7)
    a = np.zeros(7)
    b = np.array([3.0, 0.0, 0.2, -0.5, 0.3, 0.0, 0.0])
    base, ramp = rest_times(b - a, limit, limit)
    rng = np.random.default_rng(123)
    shifts = rng.random((25, 7))
    width = rng.random((25, 7))
    seen = False
    for m in proposals(
        a,
        b,
        4.0,
        1.63,
        -3 * limit,
        3 * limit,
        limit,
        limit,
        limit,
        base,
        ramp,
        width,
        shifts,
        rng,
    ):
        via, v = m.state([m.t])
        np.testing.assert_allclose(via[0, m.active], m.via[m.active], atol=1e-9)
        np.testing.assert_allclose(v[0, m.active], 0.0, atol=1e-9)
        q = sample_candidate(m.curve())
        np.testing.assert_allclose(q["q"][[0, -1]], [a, b], atol=1e-9)
        d = q["derivatives"]
        assert d["endpoint_velocity"] < 1e-8 and d["continuity_residual"] < 1e-8
        assert (
            max(d["velocity_per_joint"]) < 1.00001
            and max(d["acceleration_per_joint"]) < 1.00001
        )
        predicted, _ = m.state(q["time"])
        np.testing.assert_allclose(predicted, q["q"], atol=2e-10)
        assert not m.active[0]
        seen |= np.any(
            (via[0] < np.minimum(a, b) - 0.001) | (via[0] > np.maximum(a, b) + 0.001)
        )
    assert seen


def test_feasible_edge_preserves_coincident_switches():
    # Moving the analytically attainable via edge inward by 1e-8 rad
    # separates two joint switches by 1e-8 s and corrupts floating
    # Bézier second differences when absolute angles are nonzero.
    limit = np.ones(7)
    start = np.array([-0.5, 0.75, 1.32, -0.9, 0.2, 0.5, 0.25])
    goal = start + np.array([4.0, 0.15, 0.0, 0.3, 0.0, 0.0, 0.0])
    base, ramp = rest_times(goal - start, limit, limit)
    zeros = np.zeros((25, 7))
    for m in proposals(
        start,
        goal,
        5.0,
        2.5,
        -5 * limit,
        5 * limit,
        limit,
        limit,
        limit,
        base,
        ramp,
        zeros,
        zeros,
        np.random.default_rng(23),
    ):
        curve = m.curve()
        d = sample_candidate(curve)["derivatives"]
        assert max(d["acceleration_per_joint"]) <= 1.00001
