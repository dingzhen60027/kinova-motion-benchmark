"""Method interface and reference behaviours for benchmark validation.

A method is a class with ``setup(problem)`` and ``step(observation)``. It runs
in its own process and receives only the public problem and observations.
``step`` returns ``None`` to keep the commanded plan, or
``{"start_time_s": t_s, "trajectory": candidate}`` to replace it from ``t_s``
(see ``harness``). ``t_s`` must not precede the time at which the response
becomes available, i.e. the observation time plus the measured call latency.

The reference behaviours below are standard compositions of existing parts
(protective braking, waiting, OMPL RRTConnect from ``generic_motion_v1``). They
exist to check that the benchmark separates behaviours; they are not proposed
methods and are not tuned on test tasks.
"""
from __future__ import annotations

import time

import numpy as np

from .trajectory import Plan, Segment, braking, point_to_point
from .world import GeometryChecker, ompl_path

REGISTRY = {
    "nominal": "benchmarks.dynamic_recovery.policies:Nominal",
    "brake_wait": "benchmarks.dynamic_recovery.policies:BrakeWait",
    "brake_replan": "benchmarks.dynamic_recovery.policies:BrakeReplan",
    "snapshot": "benchmarks.dynamic_recovery.snapshot:SnapshotReplan",
    "mppi": "benchmarks.dynamic_recovery.mppi:MPPI",
    "vpsto": "benchmarks.dynamic_recovery.vpsto:VPSTO",
    "cc_vpsto": "benchmarks.dynamic_recovery.vpsto:CCVPSTO",
    "fabrics": "benchmarks.dynamic_recovery.fabrics_bridge:FabricsBridge",
    "r3": "benchmarks.dynamic_recovery.r3:R3",
}
# R3 ablations, one component switched off each (see ``r3.ABLATIONS``): ``r3-no_refuge`` and so on.
REGISTRY.update({f"r3-{name}": f"benchmarks.dynamic_recovery.r3:R3_{name}" for name in (
    "no_refuge", "no_escape", "no_retreat", "no_reroute", "no_ompl", "no_improve", "no_fallback", "no_accel",
    "always_wait", "reroute_now", "timeout", "randomized", "cv_only", "clearance_only", "no_latency")})


class Nominal:
    """Executes the initial plan and never replans (disturbance validity control)."""

    def setup(self, problem):
        return None

    def step(self, observation):
        return None


class BrakeWait:
    """Stop on a predicted conflict; whenever stopped short of the goal, resume once the route is clear.

    Obstacles are extrapolated at their observed velocity over ``horizon_s``.
    The route is the nominal waypoint polyline; the robot never leaves it.
    Stops imposed by the external supervisor are handled like the method's own.
    """

    def __init__(self, margin_m=0.05, horizon_s=1.0, sample_s=0.05, latency_allowance_s=0.04):
        self.margin, self.horizon, self.sample, self.allowance = margin_m, horizon_s, sample_s, latency_allowance_s

    def setup(self, problem):
        from benchmarks.generic_motion_v1.model import Model
        robot = problem["robot"]
        self.velocity = np.full(7, robot["velocity_limit_rad_s"])
        self.acceleration = np.full(7, robot["acceleration_limit_rad_s2"])
        self.goal = np.asarray(problem["goal"]["q"], float)
        self.tolerance = problem["protocol"]["goal_position_tolerance_rad"]
        self.static_xyz = np.asarray(problem["scene"]["static_voxels_xyz"], float)
        self.checker = GeometryChecker(Model())
        self.path = [np.asarray(w, float) for w in problem["nominal_waypoints"]]
        self.braking_until = -np.inf
        return None

    def predicted(self, observation, dt):
        parts = [np.asarray(o["voxels_xyz"], float) + np.asarray(o["velocity_m_s"], float) * dt
                 for o in observation["obstacles"]]
        return np.vstack(parts) if parts else np.empty((0, 3))

    def conflict(self, observation, plan, t0, t1):
        """True if the plan comes within the margin of predicted obstacles on [t0, t1]."""
        now = observation["time_s"]
        for tau in np.arange(t0, t1 + 1e-9, self.sample):
            xyz = self.predicted(observation, min(tau - now, self.horizon))
            if len(xyz) and self.checker.clearance(plan.at(tau)[0], xyz, fixed=False,
                                                   cutoff=self.margin + 0.05) < self.margin:
                return True
        return False

    def blocked(self, observation, route):
        """Route against the current obstacle snapshot, sampled every 0.02 rad."""
        xyz = self.predicted(observation, 0.0)
        if not len(xyz):
            return False
        for a, b in zip(route[:-1], route[1:]):
            for q in np.linspace(a, b, max(2, int(np.ceil(np.linalg.norm(b - a) / 0.02)) + 1)):
                if self.checker.clearance(q, xyz, fixed=False, cutoff=self.margin + 0.05) < self.margin:
                    return True
        return False

    def ahead(self, q):
        """Remaining route from q: drop the waypoints of the polyline already passed."""
        best, index = np.inf, 1
        for i, (a, b) in enumerate(zip(self.path[:-1], self.path[1:])):
            d = b - a
            s = np.clip((q - a) @ d / max(d @ d, 1e-12), 0.0, 1.0)
            error = np.linalg.norm(a + s * d - q)
            if error < best - 1e-9:
                best, index = error, i + 1
        return [w for w in self.path[index:]]

    def step(self, observation):
        start = time.perf_counter()
        t = observation["time_s"]
        if observation["supervisor"]["protective_stop"]:
            return None
        plan = Plan.from_list(observation["plan"])
        q, v, _ = plan.at(t)
        moving = np.max(np.abs(v)) > 1e-6 or np.max(np.abs(plan.at(t + self.allowance)[1])) > 1e-6
        if moving:
            if t < self.braking_until or not self.conflict(observation, plan, t + self.allowance, t + self.horizon):
                return None  # already stopping, or no predicted conflict
            t_s = t + (time.perf_counter() - start) + self.allowance
            qs, vs, _ = plan.at(t_s)
            stop = braking(qs, vs, self.acceleration)
            self.braking_until = t_s + float(np.sum(stop["durations"]))
            return dict(start_time_s=t_s, trajectory=stop)
        if np.max(np.abs(q - self.goal)) <= self.tolerance:
            return None
        return self.resume(observation, plan, np.asarray(q, float), start)

    def resume(self, observation, plan, q, start):
        route = [q] + self.ahead(q)
        if self.blocked(observation, route):
            return None
        motion = point_to_point(route, self.velocity, self.acceleration)
        t_s = observation["time_s"] + (time.perf_counter() - start) + self.allowance
        if self.conflict(observation, plan.spliced(Segment(t_s, motion)), t_s, t_s + self.horizon):
            return None
        self.path = route
        return dict(start_time_s=t_s, trajectory=motion)


class BrakeReplan(BrakeWait):
    """BrakeWait plus an OMPL RRTConnect detour when a stationary obstacle keeps the route blocked."""

    def __init__(self, patience_s=1.0, planning_s=4.0, static_speed_m_s=0.02, **kwargs):
        super().__init__(**kwargs)
        self.patience, self.planning, self.static_speed = patience_s, planning_s, static_speed_m_s
        self.blocked_since, self.calls = None, 0

    def resume(self, observation, plan, q, start):
        response = super().resume(observation, plan, q, start)
        t = observation["time_s"]
        moving = any(np.linalg.norm(o["velocity_m_s"]) > self.static_speed for o in observation["obstacles"])
        if response is not None or moving:
            self.blocked_since = None
            return response
        self.blocked_since = t if self.blocked_since is None else self.blocked_since
        if t - self.blocked_since < self.patience:
            return None
        path = self.detour(observation, q)
        if path is None:
            return None
        motion = point_to_point(path, self.velocity, self.acceleration)
        t_s = t + (time.perf_counter() - start) + self.allowance
        self.path, self.blocked_since = path, None
        return dict(start_time_s=t_s, trajectory=motion)

    def detour(self, observation, q):
        """Plan to the goal around the observed obstacles; shrink the margin if already close to them."""
        self.calls += 1
        obstacles = self.predicted(observation, 0.0)
        near = min(self.checker.clearance(x, obstacles, fixed=False, cutoff=1.0) for x in (q, self.goal))
        margin = max(0.005, min(self.margin, 0.5 * near))
        return ompl_path(self.checker, q, self.goal, self.static_xyz, obstacles, 0.005, margin,
                         self.planning, self.calls, f"ompl_{self.calls:03d}")
