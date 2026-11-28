"""B3: snapshot replanning (global replanning against the latest obstacle snapshot).

The standard reactive use of a fast global motion generator (for example cuRobo's MotionGen with a world
model refreshed from perception): obstacles are treated as static at their latest observed pose, inflated by a
margin, and no obstacle motion is predicted. Whenever the commanded plan comes within the margin of the snapshot
inside the look-ahead, the method brakes; from the resting point it plans to the goal around the latest snapshot
and executes the path at the robot's limits. While stopped short of the goal it keeps planning at every
observation.

On this CPU-only machine the planner is OMPL RRT-Connect with OMPL's path simplification (the repository's
``generic_motion_v1`` adapter), so planning happens from rest after braking; ``backend="curobo"`` is reserved
for the GPU machine, where planning from the braking point while still moving fits in the latency budget.
"""
from __future__ import annotations

import time

import numpy as np

from .recovery import GOAL_TOLERANCE_RAD, Obstacles, Pieces, brake, common_setup, monitor_margins, plan_state, validate
from .trajectory import Plan


class SnapshotReplan:
    def __init__(self, allowance_s=0.04, planning_s=0.4, horizon_s=2.0, sample_s=0.05, margin_m=0.05,
                 static_margin_m=0.005, backend="ompl", seed=0):
        if backend != "ompl":
            raise NotImplementedError("Only the OMPL backend is available on this machine; cuRobo needs CUDA")
        self.allowance, self.planning, self.horizon, self.sample = allowance_s, planning_s, horizon_s, sample_s
        self.margin, self.static_margin, self.seed = margin_m, static_margin_m, seed

    def setup(self, problem):
        common_setup(self, problem)
        self.tau = np.arange(0.0, self.horizon + 1e-9, self.sample)
        self.calls = 0
        return None

    def _clearance(self, plan, t0, obstacles):
        """Worst clearance of the plan to the frozen snapshot over the look-ahead, minus the margin."""
        if not len(obstacles):
            return np.inf
        Q, V = plan_state(plan, t0 + self.tau)
        return float(monitor_margins(self.arm, obstacles, Q[None], V[None], self.tau, self.supervisor,
                                     self.limits.acceleration, ("hold",), self.margin, protective=False).min())

    def _plan(self, q, observation):
        from .world import ompl_path
        self.calls += 1
        moving = [np.asarray(o["voxels_xyz"], float) for o in observation["obstacles"]]
        moving = np.vstack(moving) if moving else np.empty((0, 3))
        return ompl_path(self.arm.checker, q, self.goal, self.voxels, moving, self.static_margin, self.margin,
                         self.planning, self.seed * 1000 + self.calls, f"ompl_{self.calls:03d}")

    def step(self, observation):
        start = time.perf_counter()
        t = observation["time_s"]
        if observation["supervisor"]["protective_stop"]:
            return None
        plan = Plan.from_list(observation["plan"])
        q_now = np.asarray(observation["robot"]["q"], float)
        to_goal = np.max(np.abs(plan.final_configuration() - self.goal)) <= 1e-6
        if to_goal and np.max(np.abs(q_now - self.goal)) <= GOAL_TOLERANCE_RAD:
            return None
        obstacles = Obstacles(observation["obstacles"], rho0=0.0, sigma_v=0.0, cap=0.0)
        t_s = t + self.allowance
        qs, vs, _ = plan.at(t_s)
        moving = np.max(np.abs(vs)) > 1e-6
        if to_goal:
            if self._clearance(plan, t_s, obstacles) >= 0.0:
                return None
            if moving:
                stop = Pieces(qs).add(brake(qs, vs, self.limits.acceleration)).candidate()
                reason, _ = validate(plan, t_s, stop, self.limits, t + time.perf_counter() - start)
                return dict(start_time_s=t_s, trajectory=stop) if reason == "accepted" else None
        # Braking or stopped short of the goal: plan from the resting point around the latest snapshot.
        if moving:
            q_rest, rest_time = np.asarray(plan.final_configuration(), float), plan.segments[-1].end
        else:
            q_rest, rest_time = np.asarray(qs, float), t_s
        path = self._plan(q_rest, observation)
        if path is None or len(path) < 2:
            return None
        candidate = Pieces(q_rest).route([np.asarray(w, float) for w in path[1:]], self.limits).candidate()
        elapsed = time.perf_counter() - start
        start_time = max(t + elapsed + 0.02, rest_time + 1e-3)
        reason, _ = validate(plan, start_time, candidate, self.limits, t + elapsed)
        if reason != "accepted":
            return None
        return dict(start_time_s=start_time, trajectory=candidate)
