import time

import numpy as np
import pytest

from benchmarks.generic_motion_v1.runners import phase_search as planner
from benchmarks.generic_motion_v1.runners.phase_refinement import refine_time
from benchmarks.generic_motion_v1.runners.validate import sample_candidate


class OpenModel:
    lower = np.full(7, -3.0)
    upper = -lower

    def cuda(self, q):
        return np.ones(len(q), dtype=bool)

    def clearance(self, q):
        return np.ones(len(q))


class BlockedChord(OpenModel):
    def clearance(self, q):
        return np.hypot(q[:, 0] - 1.0, q[:, 1] - 0.5) - 0.16

    def cuda(self, q):
        return self.clearance(q) > 0


def check_motion(candidate, start, goal, velocity, acceleration):
    sample = sample_candidate(candidate)
    np.testing.assert_allclose(sample["q"][[0, -1]], [start, goal], atol=1e-9)
    limits = sample["derivatives"]
    assert np.all(limits["velocity_per_joint"] <= velocity + 1e-5)
    assert np.all(limits["acceleration_per_joint"] <= acceleration + 1e-5)
    assert limits["continuity_residual"] < 1e-5
    assert limits["endpoint_velocity"] < 1e-5
    return sample


@pytest.mark.parametrize("distance, expected", [(0.25, 1.0), (1.0, 2.0), (2.0, 3.0)])
def test_triangular_and_trapezoidal_lower_bound(distance, expected):
    start = np.zeros(7)
    goal = np.array([distance, -0.2 * distance, 0.0, 0.0, 0.0, 0.0, 0.0])
    limits = np.ones(7)
    candidate = planner.line_curve(start, goal, limits, limits)
    sample = check_motion(candidate, start, goal, limits, limits)
    assert sample["duration"] == pytest.approx(expected)
    np.testing.assert_allclose(sample["q"][:, 1], -0.2 * sample["q"][:, 0], atol=1e-12)


def test_same_duration_can_avoid_an_obstacle_on_the_synchronous_chord():
    model = BlockedChord()
    start = np.zeros(7)
    goal = np.array([2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    limits = np.ones(7)
    line = np.linspace(start, goal, 33)
    assert not model.cuda(line).all()
    _, grad = planner.collision_witness(model, line, model.cuda(line))
    trials = planner.schedules(
        model,
        start,
        goal,
        limits,
        limits,
        np.random.default_rng(42),
        grad,
        {},
        time.perf_counter() + 5.0,
    )
    curve, _, scale = next(trials)
    sample = check_motion(curve, start, goal, limits, limits)
    assert scale == 1
    assert sample["duration"] == pytest.approx(3.0)
    assert model.cuda(sample["q"]).all()


def test_projection_keeps_an_incumbent_and_checks_motion_limits():
    start = np.zeros(7)
    goal = np.array([2.0, 0.2, -0.3, 0.0, 0.3, 0.5, 0.6])
    points = np.array([start, [-1.0, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0], goal])
    model = OpenModel()
    limits = np.ones(7)
    deadline = time.perf_counter() + 3.0
    candidate = planner.time_path(model, points, limits, limits, {}, deadline)
    improved, history = refine_time(
        model, points, candidate, 3.0, limits, limits, {}, deadline
    )
    assert any(step["accepted"] for step in history)
    sample = check_motion(improved, start, goal, limits, limits)
    assert 3.0 - 1e-6 <= sample["duration"] < candidate["durations"].sum()
    unchanged, history = refine_time(
        model, points, candidate, 3.0, limits, limits, {}, 0.0
    )
    assert unchanged is candidate and not history
