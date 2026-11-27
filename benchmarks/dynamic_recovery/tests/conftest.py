"""Shared fixtures: the canonical model and a small hand-built task independent of the frozen data."""
import warnings

import numpy as np
import pytest

warnings.filterwarnings("ignore", category=DeprecationWarning)


@pytest.fixture(scope="session")
def model():
    pytest.importorskip("pydrake")
    pytest.importorskip("csdecomp")
    from benchmarks.generic_motion_v1.model import Model
    return Model()


@pytest.fixture(scope="session")
def make_task(model):
    """Build a task that sweeps joint 1 between two free poses, optionally with a crossing obstacle."""
    from benchmarks.dynamic_recovery.generator import public_robot
    from benchmarks.dynamic_recovery.harness import load_config
    from benchmarks.dynamic_recovery.trajectory import Plan, Segment, point_to_point, serializable
    from benchmarks.dynamic_recovery.world import DynamicObstacle, GeometryChecker, Motion

    cfg = load_config()
    v = np.full(7, cfg["robot"]["velocity_limit_rad_s"])
    a = np.full(7, cfg["robot"]["acceleration_limit_rad_s2"])
    start = np.array([0.0, 0.5, 0.0, 1.8, 0.0, 0.8, 0.0])
    goal = np.array([1.2, 0.5, 0.0, 1.8, 0.0, 0.8, 0.0])
    candidate, arrivals = point_to_point([start, goal], v, a, arrivals=True)
    offset = 0.2
    plan = Plan([Segment(-offset, candidate)])
    q0, v0, _ = plan.at(0.0)

    def build(crossing=False):
        obstacles = []
        if crossing:
            t_c = 0.9
            _, translations, _ = GeometryChecker(model).poses(plan.at(t_c)[0])
            point = translations[6]  # bracelet (checker order starts at the shoulder)
            direction = np.array([np.cos(0.6), np.sin(0.6), 0.0])
            motion = Motion([t_c - 0.9, t_c + 0.9], [point - 0.45 * direction, point + 0.45 * direction])
            obstacles.append(DynamicObstacle("o0", dict(shape="sphere", radius_m=0.06), motion))
        return dict(task_id="fixture-crossing" if crossing else "fixture-free", layout_id="fixture", split="fixture",
                    workspace="fixture", family="fixture", benchmark="kinova7_dynamic_recovery_v1",
                    public=dict(robot=public_robot(model), scene=dict(static_primitives=[]),
                                goal=dict(q=goal.tolist()),
                                nominal=dict(waypoints=[start.tolist(), goal.tolist()],
                                             arrival_times_s=(arrivals - offset).tolist(), sites=[]),
                                initial_plan=[dict(start_time_s=-offset, trajectory=serializable(candidate))],
                                initial_state=dict(q=q0.tolist(), v=v0.tolist())),
                    oracle=dict(obstacles=[o.to_dict() for o in obstacles], disturbance={}))

    build.nominal_end = float(arrivals[-1] - offset)
    return build
