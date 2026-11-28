"""B4: sampling-based model predictive control (MPPI) in joint space.

Model Predictive Path Integral control (Williams et al., 2017) over piecewise-constant joint accelerations of a
double integrator with the robot's velocity and acceleration limits. Each observation samples ``samples``
perturbed control sequences around the warm-started mean, rolls them out over ``steps`` x ``dt``, and updates
the mean with the path-integral weights exp(-(S - S_min) / lambda). Costs:

* route tracking (deviation from the nominal route) and progress along it, and a terminal distance-to-go term;
* static scene: being closer to the static scene than the nominal route is at the same progress;
* moving obstacles: separation under constant-velocity prediction below a margin (or, with
  ``monitor_aware=True``, below the monitor's protective distance);
* joint limits and control effort.

The executed segment is the first ``commit_steps`` of the updated mean followed by a braking tail, so every
submission ends at rest as the protocol requires and the robot stops if the planner falls silent. Near the goal
the motion is handed back to the nominal route (``RouteHandoff``), as also when MPPI stalls.
"""
from __future__ import annotations

import time

import numpy as np

from .recovery import (GOAL_TOLERANCE_RAD, Obstacles, RouteHandoff, RouteReference, accel_pieces,
                       bezier_candidate, brake, common_setup, monitor_margins, validate)
from .trajectory import Plan


class MPPI(RouteHandoff):
    def __init__(self, budget_s=0.1, samples=128, steps=12, dt=0.1, sigma=1.0, temperature=0.1, iterations=2,
                 commit_steps=2, w_track=10.0, w_progress=3.0, w_goal=4.0, w_obstacle=3000.0, w_static=3000.0,
                 w_contact=50.0,
                 w_limits=100.0, w_effort=0.01, obstacle_margin_m=0.05, monitor_aware=False, finish_rad=0.12,
                 finish_arc_rad=0.6, seed=0):
        self.budget, self.samples, self.steps, self.dt = budget_s, samples, steps, dt
        self.sigma, self.temperature, self.iterations, self.commit = sigma, temperature, iterations, commit_steps
        self.w = dict(track=w_track, progress=w_progress, goal=w_goal, obstacle=w_obstacle, static=w_static,
                      limits=w_limits, effort=w_effort, contact=w_contact)
        self.margin, self.monitor_aware, self.finish, self.seed = obstacle_margin_m, monitor_aware, finish_rad, seed
        self.finish_arc = finish_arc_rad

    def setup(self, problem):
        common_setup(self, problem)
        self.reference = RouteReference(self.route, self.arm, self.scene)
        self.rng = np.random.default_rng(self.seed + int.from_bytes(problem["task_id"].encode()[-4:], "little"))
        self.mean = np.zeros((self.steps, 7))
        self.tau = self.dt * np.arange(1, self.steps + 1)
        self.handoff_setup(finish_arc=self.finish_arc)
        return None

    def _rollout(self, q0, v0, A):
        """Integrate accelerations A (N, K, 7) with velocity clipping; returns Q, V and the applied A."""
        n = len(A)
        Q = np.empty((n, self.steps, 7))
        V = np.empty_like(Q)
        applied = np.empty_like(A)
        q = np.broadcast_to(q0, (n, 7)).copy()
        v = np.broadcast_to(v0, (n, 7)).copy()
        vmax = self.limits.velocity
        for k in range(self.steps):
            v_next = np.clip(v + A[:, k] * self.dt, -vmax, vmax)
            a = (v_next - v) / self.dt
            q = q + v * self.dt + 0.5 * a * self.dt ** 2
            v = v_next
            Q[:, k], V[:, k], applied[:, k] = q, v, a
        return Q, V, applied

    def _costs(self, Q, V, A, obstacles, s0):
        w = self.w
        e, s, ref = self.reference.project(Q)
        cost = w["track"] * np.square(e).sum(axis=1)
        cost -= w["progress"] * (s[:, -1] - s0)
        cost += w["goal"] * np.linalg.norm(Q[:, -1] - self.goal, axis=1)
        centres = self.arm.spheres(Q.reshape(-1, 7)).reshape(Q.shape[:-1] + (-1, 3))
        static = self.scene.distance(centres, self.arm.sphere_radius).min(axis=-1)
        cost += w["static"] * np.square(np.maximum(ref - static, 0.0)).sum(axis=1)
        cost += w["contact"] * (static < ref - 0.05).sum(axis=1)
        low = np.maximum(self.limits.lower + 0.02 - Q, 0.0) + np.maximum(Q - self.limits.upper + 0.02, 0.0)
        cost += w["limits"] * np.square(low).sum(axis=(1, 2))
        cost += w["effort"] * np.square(A).sum(axis=(1, 2))
        if len(obstacles):
            m = monitor_margins(self.arm, obstacles, Q, V, self.tau, self.supervisor, self.limits.acceleration,
                                ("cv",), self.margin, centres=centres, protective=self.monitor_aware)[0]
            cost += w["obstacle"] * np.square(np.minimum(m, 0.0)).sum(axis=1)
            cost += w["contact"] * (m < -self.margin).sum(axis=1)  # predicted contact (margin fully used)
        return cost

    def step(self, observation):
        start = time.perf_counter()
        t = observation["time_s"]
        if observation["supervisor"]["protective_stop"]:
            return None
        plan = Plan.from_list(observation["plan"])
        q_now = np.asarray(observation["robot"]["q"], float)
        if np.max(np.abs(q_now - self.goal)) <= GOAL_TOLERANCE_RAD and self.handed_off:
            return None
        t_s = t + self.budget
        qs, vs, _ = plan.at(t_s)
        obstacles = Obstacles(observation["obstacles"])
        action, response = self.handoff(observation, plan, t_s, qs, vs, obstacles, start, self.margin,
                                        self.monitor_aware)
        if action == "keep":
            return None
        if action == "submit":
            return response
        _, s0, _ = self.reference.project(qs[None])
        # Warm start: shift the previous mean by the elapsed observation period.
        shift = max(1, int(round(self.period / self.dt)))
        self.mean = np.vstack([self.mean[shift:], np.zeros((shift, 7))])
        amax = self.limits.acceleration
        best, last = None, 0.0
        for _ in range(self.iterations):
            if best is not None and time.perf_counter() - start + 1.2 * last > 0.9 * self.budget:
                break
            began = time.perf_counter()
            noise = self.rng.normal(0.0, self.sigma, (self.samples, self.steps, 7))
            noise[0] = 0.0  # keep the current mean among the samples
            A = np.clip(self.mean[None] + noise, -amax, amax)
            Q, V, applied = self._rollout(qs, vs, A)
            cost = self._costs(Q, V, applied, obstacles, float(s0[0]))
            spread = max(float(np.median(cost) - cost.min()), 1e-6)
            weights = np.exp(-(cost - cost.min()) / (self.temperature * spread))
            weights /= weights.sum()
            self.mean = np.clip(np.einsum("n,nkj->kj", weights, applied), -amax, amax)
            best = float(cost.min())
            last = time.perf_counter() - began
        if best is None:
            return None
        self.info = dict(t=round(t, 2), s=round(float(s0[0]), 3), length=round(self.reference.length, 3),
                 cost=round(best, 2), speed=round(float(np.abs(vs).max()), 2))
        _, _, applied = self._rollout(qs, vs, self.mean[None])
        controls, durations, q1, v1 = accel_pieces(qs, vs, applied[0, :self.commit], self.dt)
        tail_c, tail_d = brake(q1, v1, amax)
        if len(tail_d):
            controls, durations = np.concatenate([controls, tail_c]), np.concatenate([durations, tail_d])
        candidate = bezier_candidate(controls, durations)
        elapsed = time.perf_counter() - start
        if elapsed > self.budget - 0.002:
            return None
        reason, _ = validate(plan, t_s, candidate, self.limits, t + elapsed)
        return dict(start_time_s=t_s, trajectory=candidate) if reason == "accepted" else None
