"""R3 toolkit invariants and smoke runs of R3 and the baselines (B3-B6) on the hand-built fixture tasks."""
from pathlib import Path

import numpy as np
import pytest

from benchmarks.dynamic_recovery.harness import WorkerPlanner, load_config, run_episode
from benchmarks.dynamic_recovery.policies import REGISTRY


@pytest.fixture(scope="module")
def arm(model):
    from benchmarks.dynamic_recovery.recovery import ArmModel
    return ArmModel(model)


def _problem(task):
    from benchmarks.dynamic_recovery.harness import public_problem
    from benchmarks.dynamic_recovery.world import World
    return public_problem(task, World.from_task(task), load_config(), 0.002)


def _random_configurations(arm, count, seed=0):
    rng = np.random.default_rng(seed)
    return rng.uniform(arm.lower * 0.9, arm.upper * 0.9, (count, 7))


def test_batched_kinematics_match_drake(arm):
    Q = _random_configurations(arm, 25)
    rotations, centres = arm.boxes(Q)
    for q, R, c in zip(Q, rotations, centres):
        R_ref, c_ref, _ = arm.checker.poses(q)
        assert np.abs(R - R_ref).max() < 1e-9 and np.abs(c - c_ref).max() < 1e-9


def test_covering_spheres_contain_every_box_corner(arm):
    Q = _random_configurations(arm, 10, seed=1)
    rotations, centres = arm.boxes(Q)
    spheres = arm.spheres(Q)
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)
    for n in range(len(Q)):
        for g, half in enumerate(arm.half):
            corners = centres[n, g] + (signs * half) @ rotations[n, g].T
            own = arm.sphere_owner == g
            d = np.linalg.norm(corners[:, None] - spheres[n, own][None], axis=-1) - arm.sphere_radius[own][None]
            assert d.min(axis=1).max() <= 1e-9


def test_fast_static_distance_is_conservative(arm):
    from benchmarks.dynamic_recovery.recovery import StaticScene
    rng = np.random.default_rng(2)
    voxels = rng.uniform([-0.6, -0.6, 0.0], [0.6, 0.6, 1.0], (400, 3))
    scene = StaticScene(voxels, [])
    Q = _random_configurations(arm, 40, seed=3)
    fast = scene.distance(arm.spheres(Q), arm.sphere_radius).min(axis=1)
    exact = np.array([arm.checker.clearance(q, voxels, fixed=False) for q in Q])
    assert np.all(fast <= exact + 1e-6)


def test_brake_and_rest_to_rest_pieces_pass_the_submission_check(arm, make_task):
    from benchmarks.dynamic_recovery.recovery import Limits, Pieces, brake, validate
    from benchmarks.dynamic_recovery.trajectory import Plan
    task = make_task(False)
    plan = Plan.from_list(task["public"]["initial_plan"])
    limits = Limits(_problem(task), arm.lower, arm.upper)
    t_s = 0.3
    qs, vs, _ = plan.at(t_s)
    assert np.abs(vs).max() > 0.1
    pieces = Pieces(qs).add(brake(qs, vs, limits.acceleration)).to(qs + 0.2, limits).wait(0.3)
    reason, _ = validate(plan, t_s, pieces.candidate(), limits, t_s)
    assert reason == "accepted"


def test_acceleration_pieces_with_braking_tail_splice_and_end_at_rest(arm, make_task):
    from benchmarks.dynamic_recovery.recovery import Limits, accel_pieces, bezier_candidate, brake, validate
    from benchmarks.dynamic_recovery.trajectory import Plan
    task = make_task(False)
    plan = Plan.from_list(task["public"]["initial_plan"])
    limits = Limits(_problem(task), arm.lower, arm.upper)
    qs, vs, _ = plan.at(0.3)
    accelerations = np.clip(np.random.default_rng(4).normal(0, 1.0, (6, 7)), -2.0, 2.0)
    controls, durations, q1, v1 = accel_pieces(qs, vs, accelerations, 0.05)
    tail_c, tail_d = brake(q1, v1, limits.acceleration)
    candidate = bezier_candidate(np.concatenate([controls, tail_c]), np.concatenate([durations, tail_d]))
    reason, _ = validate(plan, 0.3, candidate, limits, 0.3)
    assert reason == "accepted"


def test_vpsto_minimum_duration_saturates_the_derivative_hull(make_task):
    from benchmarks.dynamic_recovery.harness import public_problem
    from benchmarks.dynamic_recovery.vpsto import VPSTO
    from benchmarks.dynamic_recovery.world import World
    from benchmarks.generic_motion_v1.runners.validate import sample_candidate
    task = make_task(False)
    method = VPSTO()
    method.setup(public_problem(task, World.from_task(task), load_config(), 0.002))
    rng = np.random.default_rng(5)
    qs, vs = np.asarray(task["public"]["initial_state"]["q"]), np.asarray(task["public"]["initial_state"]["v"])
    via = method._initial_mean(qs)[None] + rng.normal(0, 0.2, (8, method.n_via, 7))
    T = method._durations(qs, vs, via)
    c = method._coefficients(qs, vs, via, T)
    for k in range(len(via)):
        bounds = sample_candidate(dict(kind="bspline", objective="minimum_time", controls=c[k],
                                       knots=method.knots * T[k], spline_order=4))["derivatives"]
        ratio = max(np.max(bounds["velocity_per_joint"] / method.limits.velocity),
                    np.max(bounds["acceleration_per_joint"] / method.limits.acceleration))
        assert 0.95 <= ratio <= 1.0 + 1e-6


def _fabrics_available():
    from benchmarks.dynamic_recovery.fabrics_bridge import FabricsBridge
    return Path(FabricsBridge().python).exists()


METHODS = ["r3", "snapshot", "mppi", "vpsto", "cc_vpsto", "fabrics"]


@pytest.mark.parametrize("method", METHODS)
def test_method_completes_the_undisturbed_fixture(method, make_task, tmp_path):
    if method == "fabrics" and not _fabrics_available():
        pytest.skip("fabrics environment (.fabrics.venv or $FABRICS_PYTHON) not installed")
    planner = WorkerPlanner(REGISTRY[method], {}, tmp_path / "work")
    result = run_episode(make_task(False), planner, tmp_path / "episode", method=dict(name=method))
    assert result["status"] == "success", result.get("error", result["status"])
    assert result["planner"]["deadline_misses"] == 0


def _slow_crossing(make_task, model):
    """The fixture route with a slower crossing (peak about 0.47 m/s) that braking in place cannot avoid."""
    from benchmarks.dynamic_recovery.trajectory import Plan
    from benchmarks.dynamic_recovery.world import DynamicObstacle, GeometryChecker, Motion
    task = make_task(False)
    plan = Plan.from_list(task["public"]["initial_plan"])
    t_c, half_time = 1.2, 1.8
    _, translations, _ = GeometryChecker(model).poses(plan.at(t_c)[0])
    point, direction = translations[6], np.array([np.cos(0.6), np.sin(0.6), 0.0])
    motion = Motion([t_c - half_time, t_c + half_time], [point - 0.45 * direction, point + 0.45 * direction])
    obstacle = DynamicObstacle("o0", dict(shape="sphere", radius_m=0.06), motion)
    task = dict(task, task_id="fixture-slow-crossing")
    task["oracle"] = dict(task["oracle"], obstacles=[obstacle.to_dict()])
    return task


@pytest.mark.parametrize("method,avoids", [("brake_wait", False), ("r3", True)])
def test_r3_avoids_a_crossing_that_braking_in_place_does_not(method, avoids, make_task, model, tmp_path):
    planner = WorkerPlanner(REGISTRY[method], {}, tmp_path / "work")
    result = run_episode(_slow_crossing(make_task, model), planner, tmp_path / "episode", method=dict(name=method))
    assert result["collision"]["occurred"] is (not avoids)
    if avoids:
        assert result["status"] == "success" and result["interventions"]["count"] == 0
