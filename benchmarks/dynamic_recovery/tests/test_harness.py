"""Closed-loop protocol: latency charging, splice rules, collisions, isolation and replay."""
import json
import time

import numpy as np
import pytest

from benchmarks.dynamic_recovery.harness import (InProcessPlanner, WorkerPlanner, load_config, public_problem,
                                                 replay, run_episode)
from benchmarks.dynamic_recovery.policies import REGISTRY, Nominal
from benchmarks.dynamic_recovery.trajectory import Plan, braking, point_to_point
from benchmarks.dynamic_recovery.world import World

V, A = np.full(7, 1.0), np.full(7, 2.5)


class SlowStopThenGo:
    """Sleeps on its first call, then stops at ``lead`` after the observation; later resumes."""

    def __init__(self, sleep_s, lead_s):
        self.sleep, self.lead, self.calls = sleep_s, lead_s, 0

    def setup(self, problem):
        self.goal = np.asarray(problem["goal"]["q"])

    def step(self, observation):
        self.calls += 1
        plan, t = Plan.from_list(observation["plan"]), observation["time_s"]
        if self.calls == 1:
            time.sleep(self.sleep)
            q, v, _ = plan.at(t + self.lead)
            return dict(start_time_s=t + self.lead, trajectory=braking(q, v, A))
        q, v, _ = plan.at(t)
        if np.max(np.abs(v)) < 1e-9 and np.max(np.abs(q - self.goal)) > 1e-3:
            return dict(start_time_s=t + 0.05, trajectory=point_to_point([q, self.goal], V, A))
        return None


class Violations:
    """Submits one invalid trajectory per call."""

    def setup(self, problem):
        self.calls = 0

    def step(self, observation):
        self.calls += 1
        plan, t = Plan.from_list(observation["plan"]), observation["time_s"] + 0.05
        q, v, _ = plan.at(t)
        if self.calls == 1:
            return dict(start_time_s=t, trajectory=braking(q + 0.2, v, A))
        if self.calls == 2:
            return dict(start_time_s=t, trajectory=dict(kind="bezier", controls=np.stack([q, q + 0.1 * v, q + 0.3 * v])[None],
                                                        durations=[0.2]))
        if self.calls == 3:
            return dict(start_time_s=t, trajectory=braking(q, v, 4 * A))
        return None


class ResumeAfterStop:
    """Tries to override protective stops (must be rejected), then resumes once the obstacle has left."""

    def setup(self, problem):
        self.goal = np.asarray(problem["goal"]["q"])

    def step(self, observation):
        t, plan = observation["time_s"], Plan.from_list(observation["plan"])
        q, v, _ = plan.at(t + 0.05)
        if observation["supervisor"]["protective_stop"]:
            return dict(start_time_s=t + 0.05, trajectory=braking(q, v, A))
        if t > 2.6 and np.max(np.abs(v)) < 1e-9 and np.max(np.abs(q - self.goal)) > 1e-3:
            return dict(start_time_s=t + 0.05, trajectory=point_to_point([q, self.goal], V, A))
        return None


def submissions(directory):
    return [e for e in json.loads((directory / "events.json").read_text()) if e["type"] == "submission"]


def test_undisturbed_nominal_completes_on_time(make_task, tmp_path):
    result = run_episode(make_task(), InProcessPlanner(Nominal()), tmp_path / "e")
    assert result["status"] == "success"
    assert result["completion_time_s"] == pytest.approx(make_task.nominal_end, abs=0.25)
    assert result["interventions"]["count"] == 0 and result["planner"]["submissions"] == 0
    assert result["collision"]["checker_disagreement_steps"] == 0


def test_latency_is_charged_to_simulation_time(make_task, tmp_path):
    late = run_episode(make_task(), InProcessPlanner(SlowStopThenGo(0.25, 0.1)), tmp_path / "late")
    first = submissions(tmp_path / "late")[0]
    assert first["status"] == "deadline_miss" and first["ready_s"] >= 0.25
    assert late["planner"]["deadline_misses"] == 1
    ok = run_episode(make_task(), InProcessPlanner(SlowStopThenGo(0.25, 0.4)), tmp_path / "ok")
    first = submissions(tmp_path / "ok")[0]
    assert first["status"] == "accepted" and first["ready_s"] >= 0.25 and first["details"]["start_time_s"] == 0.4
    assert ok["status"] == "success" and ok["completion_time_s"] > make_task.nominal_end + 0.2
    rollout = np.load(tmp_path / "ok" / "rollout.npz")
    stopped = rollout["time"][np.max(np.abs(rollout["vref"]), axis=1) < 1e-9]
    assert stopped.min() > 0.4  # the commanded motion continued while the method computed


def test_invalid_submissions_never_change_the_reference(make_task, tmp_path):
    result = run_episode(make_task(), InProcessPlanner(Violations()), tmp_path / "e")
    assert [s["status"] for s in submissions(tmp_path / "e")] == ["discontinuous", "not_at_rest", "acceleration_limits"]
    assert result["status"] == "success"
    assert result["completion_time_s"] == pytest.approx(make_task.nominal_end, abs=0.25)


def test_crossing_obstacle_is_a_collision_for_blind_execution(make_task, tmp_path):
    result = run_episode(make_task(crossing=True), InProcessPlanner(Nominal()), tmp_path / "e")
    assert result["status"] == "collision"
    assert result["collision"]["mujoco_penetration_steps"] >= 1
    assert result["collision"]["independent_penetration_steps"] >= 1
    assert result["clearance"]["minimum_dynamic_m"] < 0


def test_supervisor_stops_an_approach_and_blocks_overrides(make_task, model, tmp_path):
    from benchmarks.dynamic_recovery.world import DynamicObstacle, GeometryChecker, Motion
    task = make_task()
    plan = Plan.from_list(task["public"]["initial_plan"])
    checker = GeometryChecker(type(model)())
    point = checker.poses(plan.at(0.9)[0])[1][6]
    ahead = checker.poses(plan.at(1.0)[0])[1][6] - point
    ahead /= np.linalg.norm(ahead)
    # An obstacle comes toward the arm, pauses just short of its path, then leaves.
    motion = Motion([0.1, 0.8, 1.8, 2.5], [point + 0.7 * ahead, point + 0.2 * ahead, point + 0.2 * ahead, point + 0.9 * ahead])
    task["oracle"]["obstacles"] = [DynamicObstacle("o0", dict(shape="sphere", radius_m=0.05), motion).to_dict()]
    result = run_episode(task, InProcessPlanner(ResumeAfterStop()), tmp_path / "e")
    events = json.loads((tmp_path / "e" / "events.json").read_text())
    assert result["interventions"]["count"] >= 1 and not result["collision"]["occurred"]
    assert any(e.get("status") == "protective_stop_active" for e in events)
    assert result["status"] == "completed_with_intervention"


def test_worker_isolation_and_bit_identical_replay(make_task, tmp_path):
    task = make_task(crossing=True)
    problem = public_problem(task, World.from_task(task), load_config(), 0.002)
    text = json.dumps(problem, default=lambda x: x.tolist())
    assert "oracle" not in text and "times_s" not in text and "witness" not in text
    planner = WorkerPlanner(REGISTRY["brake_wait"], {}, tmp_path / "work")
    original = run_episode(task, planner, tmp_path / "e", method=dict(name="brake_wait"))
    assert original["planner"]["calls"] > 0 and original["status"] != "infrastructure_error"
    comparison = replay(task, tmp_path / "e", tmp_path / "r")
    assert comparison["identical_states"] and comparison["identical_outcome"]
