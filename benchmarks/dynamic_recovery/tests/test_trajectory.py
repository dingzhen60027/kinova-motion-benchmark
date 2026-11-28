import numpy as np
import pytest

from benchmarks.dynamic_recovery.trajectory import (Plan, Segment, braking, check_submission, hold,
                                                     point_to_point)

V, A = np.full(7, 1.0), np.full(7, 2.5)
LIMITS = dict(lower=np.full(7, -3.0), upper=np.full(7, 3.0), velocity=V, acceleration=A, tolerance=1e-6,
              maximum_duration_s=60.0, maximum_pieces=4000)
QA = np.array([0.0, 0.5, 0.0, 1.8, 0.0, 0.8, 0.0])
QB = np.array([0.9, 0.3, 0.2, 1.5, 0.1, 0.9, -0.2])


def test_point_to_point_is_rest_to_rest_and_within_limits():
    motion, arrivals = point_to_point([QA, QB, QA], V, A, arrivals=True)
    segment = Segment(0.0, motion)
    for t, q in zip(arrivals, (QA, QB, QA)):
        position, velocity, _ = segment.at(t)
        assert np.allclose(position, q, atol=1e-9) and np.allclose(velocity, 0, atol=1e-9)
    plan = Plan([Segment(0.0, hold(QA, 0.1))])
    _, reason, details = check_submission(plan, 0.0, motion, LIMITS, 0.0)
    assert reason == "accepted", details


def test_braking_stops_on_the_tangent_line_within_limits():
    q, v = QA, np.array([0.4, -0.8, 0.2, 0.0, 0.5, -0.1, 0.3])
    stop = braking(q, v, A)
    segment = Segment(1.0, stop)
    assert np.allclose(segment.at(1.0)[1], v) and np.allclose(segment.at(segment.end)[1], 0, atol=1e-12)
    for t in np.linspace(1.0, segment.end, 7):
        offset = segment.at(t)[0] - q
        assert np.linalg.norm(offset - (offset @ v) / (v @ v) * v) < 1e-12
    assert np.max(np.abs(segment.at(1.01)[2])) <= 2.5 + 1e-12


def test_splice_replaces_the_future_and_keeps_the_past():
    plan = Plan([Segment(-0.5, point_to_point([QA, QB], V, A))])
    q, v, _ = plan.at(0.3)
    spliced = plan.spliced(Segment(0.3, braking(q, v, A)))
    assert np.allclose(spliced.at(0.2)[0], plan.at(0.2)[0])
    assert np.allclose(spliced.at(0.3)[0], q) and np.allclose(spliced.at(0.3)[1], v)
    assert len(spliced.future(0.31)) == 1


def _moving_case(case):
    """Submissions from a moving reference (t = 0.5 is mid-cruise of a 1.3 s straight motion)."""
    plan = Plan([Segment(0.0, point_to_point([QA, QB], V, A))])
    q, v, _ = plan.at(0.5)
    assert np.max(np.abs(v)) > 0.5
    earliest, candidate = 0.0, braking(q, v, A)
    if case == "late":
        earliest = 0.6
    elif case == "jump":
        candidate = braking(q + 0.1, v, A)
    elif case == "moving_end":  # leaves with the current velocity but ends at twice it
        candidate = dict(kind="bezier", controls=np.stack([q, q + 0.1 * v, q + 0.3 * v])[None], durations=[0.2])
    elif case == "garbage":
        candidate = dict(kind="bezier", controls="nonsense", durations=[1.0])
    return plan, candidate, earliest


def _resting_case(case):
    """Submissions from rest at QA."""
    plan = Plan([Segment(0.0, hold(QA, 0.1))])
    if case == "fast":
        candidate = point_to_point([QA, QB], 2 * V, A)
    elif case == "harsh":
        candidate = point_to_point([QA, QB], V, 2 * A)
    else:
        candidate = point_to_point([QA, QA + np.array([3.5, 0, 0, 0, 0, 0, 0])], V, A)
    return plan, candidate, 0.0


@pytest.mark.parametrize("case,expected", [
    ("late", "deadline_miss"), ("jump", "discontinuous"), ("moving_end", "not_at_rest"), ("garbage", "malformed"),
    ("fast", "velocity_limits"), ("harsh", "acceleration_limits"), ("outside", "joint_limits")])
def test_each_rule_rejects_its_violation(case, expected):
    plan, candidate, earliest = (_moving_case if case in ("late", "jump", "moving_end", "garbage")
                                 else _resting_case)(case)
    start = 0.5 if case in ("late", "jump", "moving_end", "garbage") else 0.2
    segment, reason, details = check_submission(plan, start, candidate, LIMITS, earliest)
    assert (reason, segment) == (expected, None), details
