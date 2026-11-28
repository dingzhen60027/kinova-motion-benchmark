"""B6: dynamic optimization fabrics (Spahn, Wisse and Alonso-Mora, T-RO 2023) through a process bridge.

The ``fabrics`` package (0.10.0) needs numpy < 2 and casadi, so it runs in its own environment
(``.fabrics.venv`` at the repository root, or ``$FABRICS_PYTHON``) as ``fabrics_worker.py``; this class is the
benchmark method and talks to it over JSON lines. The fabric is a reactive second-order policy: a joint-space
attractor towards a carrot on the nominal route (``lookahead_rad`` ahead of the robot, the goal at the end),
joint-limit avoidance, the ``static_obstacles`` static cover spheres nearest to the arm and the moving obstacles
as spheres with their observed velocity and estimated acceleration. Collision geometry is the benchmark's covering
sphere model (the many small gripper spheres are merged into ``gripper_spheres`` bounding spheres). Each
observation integrates the fabric for ``steps`` x ``dt``; the executed segment is that rollout followed by a
braking tail (every submission ends at rest). Near the goal, or when the fabric stalls in a local minimum, the
motion is handed back to the nominal route (``RouteHandoff``, as for MPPI). Fabrics does
not model the external monitor, self-collision or the static scene beyond the nearest spheres; that is the method
as published and as typically deployed.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np

from .recovery import (GOAL_TOLERANCE_RAD, VOXEL_RADIUS_M, Obstacles, RouteHandoff, RouteReference, accel_pieces,
                       bezier_candidate, brake, common_setup, validate)
from .trajectory import Plan

ROOT = Path(__file__).resolve().parents[2]
WORKER = Path(__file__).resolve().with_name("fabrics_worker.py")


def _clusters(points, radii, count, rng):
    """Bounding spheres of ``count`` k-means clusters of spheres (centres (N,3), radii (N,))."""
    centres = points[rng.choice(len(points), count, replace=False)]
    for _ in range(50):
        label = np.argmin(np.linalg.norm(points[:, None] - centres[None], axis=-1), axis=1)
        centres = np.asarray([points[label == k].mean(axis=0) if np.any(label == k) else centres[k]
                              for k in range(count)])
    out = []
    for k in range(count):
        members = label == k
        if np.any(members):
            r = float(np.max(np.linalg.norm(points[members] - centres[k], axis=1) + radii[members]))
            out.append((centres[k], r))
    return out


class FabricsBridge(RouteHandoff):
    def __init__(self, budget_s=0.1, dt=0.02, steps=15, static_obstacles=8, dynamic_obstacles=1, lookahead_rad=0.3,
                 goal_weight=8.0, finish_rad=0.1, finish_arc_rad=0.6, gripper_spheres=6, python=None, seed=0):
        self.budget, self.dt, self.steps = budget_s, dt, steps
        self.n_static, self.n_dynamic = static_obstacles, dynamic_obstacles
        self.lookahead, self.goal_weight, self.finish = lookahead_rad, goal_weight, finish_rad
        self.finish_arc = finish_arc_rad
        self.gripper_spheres, self.seed = gripper_spheres, seed
        self.python = python or os.environ.get("FABRICS_PYTHON") or str(ROOT / ".fabrics.venv" / "bin" / "python")

    # -- robot description for the worker
    def _description(self):
        arm = self.arm
        centres = arm.spheres(np.zeros(7))[0]
        origins = np.vstack([np.zeros(3), arm.points])  # frame origin of link k (k joints) at q = 0
        spheres = []
        last = int(arm.sphere_chain.max())
        for link in range(last + 1):
            members = np.flatnonzero(arm.sphere_chain == link)
            if not len(members):
                continue
            if link == last and len(members) > self.gripper_spheres:
                groups = _clusters(centres[members], arm.sphere_radius[members], self.gripper_spheres,
                                   np.random.default_rng(0))
            else:
                groups = [(centres[i], float(arm.sphere_radius[i])) for i in members]
            for centre, radius in groups:
                spheres.append(dict(link=link, offset=(centre - origins[link]).tolist(), radius=radius))
        return dict(points=arm.points.tolist(), axes=arm.axes.tolist(), spheres=spheres,
                    lower=self.limits.lower.tolist(), upper=self.limits.upper.tolist(),
                    static_obstacles=self.n_static, dynamic_obstacles=self.n_dynamic)

    def _call(self, message):
        self.process.stdin.write(json.dumps(message) + "\n")
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            self.log.flush()
            raise RuntimeError("fabrics worker exited: " + Path(self.log.name).read_text()[-2000:])
        reply = json.loads(line)
        if not reply.get("ok"):
            raise RuntimeError(f"fabrics worker error: {reply.get('error')}\n{reply.get('trace', '')}")
        return reply

    def setup(self, problem):
        common_setup(self, problem)
        self.reference = RouteReference(self.route, self.arm, self.scene)
        description = self._description()
        key = hashlib.sha256((json.dumps(description, sort_keys=True) + WORKER.read_text()).encode()).hexdigest()[:20]
        description.update(cmd="setup", cache=str(ROOT / "runs" / "dynamic_recovery" / ".fabrics_cache" / f"{key}.pkl"))
        self.sphere_count = len(description["spheres"])
        env = {k: v for k, v in os.environ.items() if not k.startswith(("PYTHON", "VIRTUAL_ENV"))}
        self.log = open("fabrics_worker.log", "w")
        self.process = subprocess.Popen([self.python, "-I", str(WORKER)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=self.log, text=True, env=env)
        reply = self._call(description)
        self.previous_velocity = {}
        self.tau = np.arange(0.1, 1.2 + 1e-9, 0.1)
        self.handoff_setup(finish_arc=self.finish_arc)
        return dict(fabrics_setup_s=reply["seconds"], loaded_from_cache=reply["loaded"], spheres=self.sphere_count)

    def __del__(self):
        process = getattr(self, "process", None)
        if process is not None and process.poll() is None:
            try:
                process.stdin.write(json.dumps(dict(cmd="close")) + "\n")
                process.stdin.flush()
                process.wait(timeout=2)
            except Exception:
                process.kill()

    # -- per-step inputs
    def _carrot(self, qs):
        _, s, _ = self.reference.project(qs[None])
        target = float(s[0]) + self.lookahead
        if target >= self.reference.length:
            return self.goal
        return self.reference.points[int(np.searchsorted(self.reference.arc, target))]

    def _static(self, qs):
        if not len(self.voxels):
            return [[100.0, 100.0, 100.0, VOXEL_RADIUS_M]] * self.n_static
        centres = self.arm.spheres(qs[None])[0]
        d = np.linalg.norm(self.voxels[:, None] - centres[None], axis=-1).min(axis=1)
        nearest = self.voxels[np.argsort(d)[: self.n_static]]
        rows = [[*p, VOXEL_RADIUS_M] for p in nearest]
        return rows + [[100.0, 100.0, 100.0, VOXEL_RADIUS_M]] * (self.n_static - len(rows))

    def _dynamic(self, observation, qs):
        t = observation["time_s"]
        rows = []
        centres = self.arm.spheres(qs[None])[0]
        for o in observation["obstacles"]:
            p, v = np.asarray(o["position_m"], float), np.asarray(o["velocity_m_s"], float)
            previous = self.previous_velocity.get(o["id"])
            a = (v - previous[1]) / (t - previous[0]) if previous is not None and t > previous[0] else np.zeros(3)
            self.previous_velocity[o["id"]] = (t, v)
            prim = o["primitive"]
            radius = float(prim["radius_m"]) if prim["shape"] == "sphere" else float(np.linalg.norm(prim["size_m"]) / 2)
            distance = float(np.linalg.norm(centres - p, axis=1).min())
            rows.append((distance, [*p, *v, *a, radius + VOXEL_RADIUS_M]))
        rows = [r for _, r in sorted(rows, key=lambda x: x[0])][: self.n_dynamic]
        return rows + [[100.0, 100.0, 100.0, 0, 0, 0, 0, 0, 0, 0.05]] * (self.n_dynamic - len(rows))

    def step(self, observation):
        start = time.perf_counter()
        t = observation["time_s"]
        if observation["supervisor"]["protective_stop"]:
            return None
        plan = Plan.from_list(observation["plan"])
        q_now = np.asarray(observation["robot"]["q"], float)
        if self.handed_off and np.max(np.abs(q_now - self.goal)) <= GOAL_TOLERANCE_RAD:
            return None
        t_s = t + self.budget
        qs, vs, _ = plan.at(t_s)
        dynamic = self._dynamic(observation, qs)
        obstacles = Obstacles(observation["obstacles"])
        action, response = self.handoff(observation, plan, t_s, qs, vs, obstacles, start, 0.05, False)
        if action == "keep":
            return None
        if action == "submit":
            return response
        reply = self._call(dict(cmd="rollout", q=qs.tolist(), qdot=vs.tolist(), goal=self._carrot(qs).tolist(),
                                goal_weight=self.goal_weight, static=self._static(qs), dynamic=dynamic,
                                steps=self.steps, dt=self.dt, vmax=self.limits.velocity.tolist(),
                                amax=self.limits.acceleration.tolist()))
        controls, durations, q1, v1 = accel_pieces(qs, vs, np.asarray(reply["acc"]), self.dt)
        tail_c, tail_d = brake(q1, v1, self.limits.acceleration)
        if len(tail_d):
            controls, durations = np.concatenate([controls, tail_c]), np.concatenate([durations, tail_d])
        candidate = bezier_candidate(controls, durations)
        elapsed = time.perf_counter() - start
        if elapsed > self.budget - 0.002:
            return None
        reason, _ = validate(plan, t_s, candidate, self.limits, t + elapsed)
        return dict(start_time_s=t_s, trajectory=candidate) if reason == "accepted" else None
