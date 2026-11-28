"""Shared machinery for the online recovery methods (R3 and the baselines B3-B6).

Everything here uses only what a method is given: the public problem at setup and the observations
during the episode. The module provides

- batched forward kinematics of the canonical robot collision boxes (product of exponentials calibrated
  against the Drake plant; tests check agreement with Drake),
- a conservative sphere model that covers every robot box,
- distances to the static scene (a distance field of the static cover spheres plus the fixed equipment
  boxes) and to moving obstacles under constant-velocity, stop and hold hypotheses,
- the protective distance the external speed-and-separation monitor will demand,
- rest-to-rest, braking and hold trajectory pieces (quadratic Bezier, the benchmark's timed format) and
  their batched evaluation,
- a static roadmap built at setup and searched online.

Scoring uses the conservative sphere model; the candidate a method finally submits can be confirmed with
the exact canonical checker (``ArmModel.exact_clearance``).
"""
from __future__ import annotations

import hashlib
import heapq
import json
import time
from pathlib import Path

import numpy as np

from .trajectory import TIMED, Plan, check_submission
from .world import VOXEL_RADIUS_M, GeometryChecker

GOAL_TOLERANCE_RAD = 0.02


# ----------------------------------------------------------------------------- problem helpers

class Limits:
    """Joint position, velocity and acceleration limits from the public problem."""

    def __init__(self, problem, lower, upper):
        robot, protocol = problem["robot"], problem["protocol"]
        self.velocity = np.full(7, float(robot["velocity_limit_rad_s"]))
        self.acceleration = np.full(7, float(robot["acceleration_limit_rad_s2"]))
        self.lower, self.upper = np.asarray(lower, float), np.asarray(upper, float)
        self.protocol = protocol
        self.check = dict(lower=self.lower, upper=self.upper, velocity=self.velocity, acceleration=self.acceleration,
                          tolerance=float(protocol["splice_tolerance"]),
                          maximum_duration_s=float(protocol["maximum_trajectory_duration_s"]),
                          maximum_pieces=int(protocol["maximum_trajectory_pieces"]))

    def clip(self, q):
        return np.clip(q, self.lower + 1e-4, self.upper - 1e-4)


def remaining_route(path, q):
    """Waypoints of the polyline ``path`` still ahead of configuration ``q`` (closest-segment rule)."""
    best, index = np.inf, 1
    for i, (a, b) in enumerate(zip(path[:-1], path[1:])):
        d = b - a
        s = np.clip((q - a) @ d / max(d @ d, 1e-12), 0.0, 1.0)
        error = np.linalg.norm(a + s * d - q)
        if error < best - 1e-9:
            best, index = error, i + 1
    return [np.asarray(w, float) for w in path[index:]]


# ----------------------------------------------------------------------------- kinematics

def _rodrigues(axis, angles):
    """(N, 3, 3) rotations about a unit ``axis`` by ``angles`` (N,)."""
    x, y, z = axis
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    s, c = np.sin(angles)[:, None, None], np.cos(angles)[:, None, None]
    return np.eye(3)[None] + s * K[None] + (1.0 - c) * (K @ K)[None]


class ArmModel:
    """Batched kinematics of the canonical collision boxes, a covering sphere model and exact checks."""

    def __init__(self, model=None, sphere_inflation=1.1):
        from benchmarks.generic_motion_v1.model import Model
        self.checker = GeometryChecker(model if model is not None else Model())
        self.model = self.checker.model
        plant, context = self.model.plant, self.model.plant_context
        instance = plant.GetModelInstanceByName("kinova")
        self.joints = [plant.GetJointByName(f"joint_{k}", instance) for k in range(1, 8)]
        zero = np.zeros(plant.num_positions())
        plant.SetPositions(context, zero)
        self.axes, self.points = [], []
        for joint in self.joints:
            X_WF = joint.frame_on_parent().CalcPoseInWorld(context)
            self.axes.append(X_WF.rotation().matrix() @ np.asarray(joint.revolute_axis(), float))
            self.points.append(X_WF.translation().copy())
        self.axes, self.points = np.asarray(self.axes), np.asarray(self.points)
        rotations, translations, _ = self.checker.poses(np.zeros(7))
        self.box_R0, self.box_p0 = rotations, translations
        self.half = self.checker.half
        self.box_joints = []
        for body in self.checker.bodies:
            affected = [k for k, joint in enumerate(self.joints)
                        if body.index() in plant.GetBodiesKinematicallyAffectedBy([joint.index()])]
            self.box_joints.append(max(affected) + 1 if affected else 0)
        self.box_joints = np.asarray(self.box_joints)
        ee = plant.GetBodyByName("end_effector_link", instance)
        X = plant.EvalBodyPoseInWorld(context, ee)
        self.ee_R0, self.ee_p0 = X.rotation().matrix(), X.translation().copy()
        self.lower, self.upper = np.asarray(self.checker.lower, float), np.asarray(self.checker.upper, float)
        self._spheres(sphere_inflation)

    # -- spheres covering each box
    def _spheres(self, inflation):
        centres, radii, owner = [], [], []
        for g, h in enumerate(self.half):
            order = np.argsort(h)[::-1]
            long_axis, a, b = order
            rho = float(np.hypot(h[a], h[b]))
            radius = inflation * rho
            gap = 2.0 * np.sqrt(max(radius ** 2 - rho ** 2, 1e-12))
            count = max(1, int(np.ceil(2.0 * h[long_axis] / gap)))
            step = 2.0 * h[long_axis] / count
            for i in range(count):
                c = np.zeros(3)
                c[long_axis] = -h[long_axis] + step * (i + 0.5)
                # Every box point lies within sqrt((step / 2)^2 + rho^2) <= radius of some centre.
                r = float(np.sqrt((step / 2) ** 2 + rho ** 2))
                centres.append(c)
                radii.append(max(r, 1e-3))
                owner.append(g)
        self.sphere_local = np.asarray(centres)
        self.sphere_radius = np.asarray(radii)
        self.sphere_owner = np.asarray(owner)
        self.sphere_chain = self.box_joints[self.sphere_owner]
        per_box = np.bincount(self.sphere_owner, minlength=len(self.half))
        k = int(per_box.max())
        self._padded = np.zeros((len(self.half), k, 3))
        self._flat = np.zeros(len(self.sphere_owner), int)
        slot = np.zeros(len(self.half), int)
        for i, g in enumerate(self.sphere_owner):
            self._padded[g, slot[g]] = self.sphere_local[i]
            self._flat[i] = g * k + slot[g]
            slot[g] += 1

    # -- batched transforms
    def chain(self, Q):
        """Cumulative joint transforms (rotation (N,8,3,3), translation (N,8,3)); index k = after k joints."""
        Q = np.atleast_2d(np.asarray(Q, float))
        n = len(Q)
        R = np.empty((n, 8, 3, 3))
        t = np.empty((n, 8, 3))
        R[:, 0], t[:, 0] = np.eye(3), 0.0
        for k in range(7):
            Rk = _rodrigues(self.axes[k], Q[:, k])
            tk = self.points[k][None] - np.einsum("nij,j->ni", Rk, self.points[k])
            R[:, k + 1] = R[:, k] @ Rk
            t[:, k + 1] = np.einsum("nij,nj->ni", R[:, k], tk) + t[:, k]
        return R, t

    def boxes(self, Q):
        """World poses of the 15 collision boxes: rotations (N,G,3,3), centres (N,G,3)."""
        R, t = self.chain(Q)
        Rc, tc = R[:, self.box_joints], t[:, self.box_joints]
        rotations = Rc @ self.box_R0[None]
        centres = np.einsum("ngij,gj->ngi", Rc, self.box_p0) + tc
        return rotations, centres

    def spheres(self, Q):
        """World centres (N,S,3) of the covering spheres; radii are ``sphere_radius``."""
        rotations, centres = self.boxes(Q)
        world = np.einsum("ngij,gkj->ngki", rotations, self._padded) + centres[:, :, None, :]
        return world.reshape(len(world), -1, 3)[:, self._flat]

    def tool(self, Q, offset):
        R, t = self.chain(Q)
        Rk, tk = R[:, 7], t[:, 7]
        p = np.einsum("nij,j->ni", Rk, self.ee_p0) + tk
        z = np.einsum("nij,j->ni", Rk, self.ee_R0[:, 2])
        return p + offset * z

    # -- exact canonical checks (slow; for final confirmation)
    def exact_clearance(self, Q, xyz=(), fixed=True, cutoff=0.2):
        return np.asarray([self.checker.clearance(q, xyz, fixed=fixed, cutoff=cutoff) for q in np.atleast_2d(Q)])

    def self_and_fixed_clearance(self, Q):
        out = []
        for q in np.atleast_2d(Q):
            _, _, query = self.checker.poses(q)
            out.append(self.checker.fixed_clearance(query))
        return np.asarray(out)


# ----------------------------------------------------------------------------- static scene

def box_distance(points, centre, half, yaw=0.0):
    """Signed distance from points (..., 3) to a z-yaw box."""
    c, s = np.cos(yaw), np.sin(yaw)
    d = np.asarray(points, float) - np.asarray(centre, float)
    local = np.stack([c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1], d[..., 2]], axis=-1)
    q = np.abs(local) - np.asarray(half, float)
    outside = np.sqrt(np.square(np.maximum(q, 0.0)).sum(axis=-1))
    return outside + np.minimum(q.max(axis=-1), 0.0)


class StaticScene:
    """Distances from robot spheres to the static cover spheres and the fixed equipment boxes."""

    def __init__(self, voxels_xyz, fixed_boxes, resolution=0.01, padding=0.35):
        from scipy import ndimage
        xyz = np.asarray(voxels_xyz, float).reshape(-1, 3)
        self.boxes = list(fixed_boxes)
        self.resolution = resolution
        if len(xyz):
            low, high = xyz.min(axis=0) - padding, xyz.max(axis=0) + padding
            shape = np.ceil((high - low) / resolution).astype(int) + 1
            grid = np.ones(shape, dtype=bool)
            index = np.round((xyz - low) / resolution).astype(int)
            grid[tuple(index.T)] = False
            # Distance from each cell centre to the nearest voxel centre, in metres.
            self.field = ndimage.distance_transform_edt(grid, sampling=resolution).astype(np.float32)
            self.low, self.shape = low, shape
            self.padding = padding
            # A query point lies within half a cell diagonal of its cell centre.
            self.slack = 0.5 * np.sqrt(3.0) * resolution
        else:
            self.field = None

    def distance(self, centres, radii):
        """Conservative clearance of spheres (..., S, 3) with radii (S,) to the static scene."""
        centres = np.asarray(centres, float)
        best = np.full(centres.shape[:-1], np.inf)
        if self.field is not None:
            index = np.round((centres - self.low) / self.resolution).astype(int)
            inside = np.all((index >= 0) & (index < self.shape), axis=-1)
            clipped = np.clip(index, 0, self.shape - 1)
            d = self.field[clipped[..., 0], clipped[..., 1], clipped[..., 2]].astype(float)
            d = np.where(inside, d - self.slack, self.padding)
            best = np.minimum(best, d - VOXEL_RADIUS_M)
        for box in self.boxes:
            best = np.minimum(best, box_distance(centres, box["center_m"], np.asarray(box["size_m"]) / 2,
                                                 box.get("yaw_rad", 0.0)))
        return best - np.asarray(radii, float)


def fixed_equipment(model):
    """Canonical fixed boxes the robot can hit (table, tripods, wall); the robot base is excluded."""
    from .generator import fixed_primitives
    return [p for p in fixed_primitives(model) if p["shape"] == "box" and not p["name"].startswith("kinova::")]


# ----------------------------------------------------------------------------- moving obstacles

HYPOTHESES = ("cv", "stop", "hold")


class Obstacles:
    """Observed moving obstacles and their predicted motion under several hypotheses."""

    def __init__(self, observed, deceleration=1.0, rho0=0.005, sigma_v=0.03, cap=0.08, rest_speed=0.02,
                 accelerations=None, accel_horizon=0.6, speed_cap=1.6):
        self.items = []
        for o in observed:
            prim = o["primitive"]
            item = dict(id=o["id"], shape=prim["shape"], p=np.asarray(o["position_m"], float),
                        v=np.asarray(o["velocity_m_s"], float), yaw=float(prim.get("yaw_rad", 0.0)),
                        a=np.asarray((accelerations or {}).get(o["id"], np.zeros(3)), float))
            if prim["shape"] == "sphere":
                item["radius"] = float(prim["radius_m"])
            else:
                item["half"] = np.asarray(prim["size_m"], float) / 2
            self.items.append(item)
        self.deceleration, self.rho0, self.sigma_v, self.cap = deceleration, rho0, sigma_v, cap
        self.rest_speed, self.accel_horizon, self.speed_cap = rest_speed, accel_horizon, speed_cap

    def inflation(self, tau, j=None):
        """Envelope radius added to every prediction: grows with look-ahead (capped) for a moving obstacle.

        A resting obstacle keeps the base radius: if it starts to move, that is observed within one period.
        """
        tau = np.asarray(tau, float)
        if j is not None and self.speed(j) < self.rest_speed:
            return np.full(tau.shape, self.rho0)
        return np.minimum(self.rho0 + self.sigma_v * tau, self.cap)

    def subset(self, indices):
        out = Obstacles([])
        out.__dict__.update(self.__dict__)
        out.items = [self.items[j] for j in indices]
        return out

    def __len__(self):
        return len(self.items)

    def speed(self, j):
        return float(np.linalg.norm(self.items[j]["v"]))

    def predict(self, j, tau, hypothesis):
        """Predicted centre (T,3) and velocity (T,3) of obstacle j at look-ahead times tau (T,)."""
        item = self.items[j]
        tau = np.asarray(tau, float)
        p, v = item["p"], item["v"]
        if hypothesis == "hold":
            return np.broadcast_to(p, tau.shape + (3,)), np.zeros(tau.shape + (3,))
        if hypothesis == "cv":
            return self._accelerated(item, tau)
        speed = np.linalg.norm(v)
        if speed < 1e-9:
            return np.broadcast_to(p, tau.shape + (3,)), np.zeros(tau.shape + (3,))
        stop = speed / self.deceleration
        s = np.minimum(tau, stop)
        direction = v / speed
        travel = speed * s - 0.5 * self.deceleration * s ** 2
        velocity = np.maximum(speed - self.deceleration * tau, 0.0)
        return p + travel[:, None] * direction, velocity[:, None] * direction

    def _accelerated(self, item, tau):
        """Constant velocity, preceded by the estimated acceleration along the motion for ``accel_horizon``.

        Deceleration stops at rest (no reversal) and speed-up is capped at ``speed_cap``.
        """
        p, v, a = item["p"], item["v"], item["a"]
        speed = float(np.linalg.norm(v))
        if speed < 1e-6 or not np.any(a):
            return p + tau[:, None] * v, np.broadcast_to(v, tau.shape + (3,))
        u = v / speed
        along = float(a @ u)
        if along < 0.0:
            ramp = min(self.accel_horizon, speed / -along)
        elif along > 0.0:
            ramp = min(self.accel_horizon, max(self.speed_cap - speed, 0.0) / along)
        else:
            ramp = 0.0
        s = np.minimum(tau, ramp)
        travel = speed * s + 0.5 * along * s ** 2
        final = speed + along * ramp
        travel = travel + final * np.maximum(tau - ramp, 0.0)
        velocity = np.where(tau < ramp, speed + along * tau, final)
        return p + travel[:, None] * u, velocity[:, None] * u

    def surface_distance(self, j, centres, predicted):
        """Signed distance of points (..., T, S, 3) to obstacle j placed at ``predicted`` (T, 3)."""
        item = self.items[j]
        rel = centres - predicted[..., None, :]
        if item["shape"] == "sphere":
            return np.linalg.norm(rel, axis=-1) - item["radius"]
        return box_distance(rel, np.zeros(3), item["half"], item["yaw"])

    def clearance(self, centres, radii, tau, hypotheses=HYPOTHESES):
        """Per hypothesis, the conservative clearance (H, T, S) of robot spheres over look-ahead ``tau``."""
        out = []
        for h in hypotheses:
            best = np.full(centres.shape[:-1], np.inf)
            for j in range(len(self.items)):
                predicted, _ = self.predict(j, tau, h)
                d = self.surface_distance(j, centres, predicted) - VOXEL_RADIUS_M
                d = d - self.inflation(tau, j)[:, None]
                best = np.minimum(best, d)
            out.append(best - radii[None, :])
        return np.asarray(out)


def protective_distance(obstacle_approach, robot_approach, stopping_time, supervisor):
    """Simplified ISO/TS 15066 protective distance used by the benchmark's external monitor."""
    reaction = supervisor["reaction_time_s"]
    return (obstacle_approach * (reaction + stopping_time) + robot_approach * reaction
            + 0.5 * robot_approach * stopping_time + supervisor["intrusion_distance_m"])


def monitor_margins(arm, obstacles, Q, V, tau, supervisor, acceleration, hypotheses=HYPOTHESES, buffer=0.01,
                    centres=None, protective=True):
    """Separation minus the monitor's protective distance, per hypothesis and sample (H, ..., T).

    ``Q``/``V`` are (..., T, 7) configurations and joint velocities at look-ahead times ``tau`` (T,). The
    robot point velocity is the finite difference of the sphere centres along the last time axis. The monitor
    never stops a robot that is already at rest, so samples at rest only need clearance (no protective distance).
    """
    Q = np.asarray(Q, float)
    shape = Q.shape[:-1]
    if not len(obstacles):
        return np.full((len(hypotheses),) + shape, np.inf)
    tau = np.asarray(tau, float)
    if centres is None:
        centres = arm.spheres(Q.reshape(-1, 7)).reshape(shape + (-1, 3))
    if len(tau) > 1:
        point_velocity = np.gradient(centres, tau, axis=-3)
    else:
        point_velocity = np.zeros_like(centres)
    stopping = np.max(np.abs(V) / acceleration, axis=-1)
    moving = (np.max(np.abs(V), axis=-1) > supervisor.get("moving_threshold_rad_s", 0.01))[..., None]
    out = []
    for h in hypotheses:
        worst = np.full(shape, np.inf)
        for j in range(len(obstacles)):
            predicted, velocity = obstacles.predict(j, tau, h)
            rel = centres - predicted[..., None, :]
            d = obstacles.surface_distance(j, centres, predicted) - VOXEL_RADIUS_M - arm.sphere_radius
            normal = -rel / np.maximum(np.linalg.norm(rel, axis=-1, keepdims=True), 1e-9)  # robot -> obstacle
            robot_app = np.maximum(np.sum(point_velocity * normal, axis=-1), 0.0)
            obstacle_app = np.maximum(-np.sum(velocity[..., None, :] * normal, axis=-1), 0.0)
            required = (protective_distance(obstacle_app, robot_app, stopping[..., None], supervisor)
                        if protective else 0.0)
            inflation = obstacles.inflation(tau, j)[:, None]
            margin = d - inflation - buffer - np.where(moving, required, 0.0)
            margin = np.where(d < supervisor["cutoff_m"], margin, np.inf)
            worst = np.minimum(worst, margin.min(axis=-1))
        out.append(worst)
    return np.asarray(out)


def monitor_margin(arm, obstacles, Q, V, tau, supervisor, acceleration, hypotheses=HYPOTHESES, buffer=0.01):
    """Worst margin over time per hypothesis (H,) for one trajectory sampled at ``tau``."""
    return monitor_margins(arm, obstacles, Q, V, tau, supervisor, acceleration, hypotheses, buffer).min(axis=-1)


# ----------------------------------------------------------------------------- trajectory pieces

def _trapezoid(delta, velocity, acceleration):
    """Synchronized straight-line rest-to-rest timing on s in [0, 1] (as ``line_curve``)."""
    moving = np.abs(delta) > 1e-12
    if not np.any(moving):
        return 0.0, 0.0, 0.0
    speed = float(np.min(velocity[moving] / np.abs(delta[moving])))
    accel = float(np.min(acceleration[moving] / np.abs(delta[moving])))
    ramp = min(np.sqrt(1.0 / accel), speed / accel)
    duration = ramp + 1.0 / (accel * ramp)
    return duration, ramp, accel


def p2p(a, b, velocity, acceleration):
    """Rest-to-rest straight joint motion a -> b as quadratic Bezier pieces (controls (P,3,7), durations (P,))."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    delta = b - a
    duration, ramp, accel = _trapezoid(delta, velocity, acceleration)
    if duration <= 0.0:
        return np.empty((0, 3, 7)), np.empty(0)
    peak = accel * ramp
    cruise = duration - 2.0 * ramp
    s1 = 0.5 * accel * ramp ** 2
    knots = [(0.0, 0.0, 0.0), (ramp, s1, peak)]
    if cruise > 1e-9:
        knots.append((ramp + cruise, s1 + peak * cruise, peak))
    knots.append((duration, 1.0, 0.0))
    controls, durations = [], []
    for (t0, s0, v0), (t1, s1_, _) in zip(knots[:-1], knots[1:]):
        d = t1 - t0
        if d <= 1e-9:
            continue
        p = np.array([s0, s0 + 0.5 * d * v0, s1_])
        controls.append(a[None] + p[:, None] * delta[None])
        durations.append(d)
    return np.asarray(controls), np.asarray(durations)


def brake(q, v, acceleration):
    """Uniform tangent deceleration to rest (as ``trajectory.braking``)."""
    q, v = np.asarray(q, float), np.asarray(v, float)
    duration = float(np.max(np.abs(v) / acceleration))
    if duration < 1e-6:
        return np.empty((0, 3, 7)), np.empty(0)
    end = q + 0.5 * duration * v
    return np.stack([q, end, end])[None], np.array([duration])


def hold(q, duration):
    if duration <= 1e-6:
        return np.empty((0, 3, 7)), np.empty(0)
    return np.broadcast_to(np.asarray(q, float), (1, 3, 7)).copy(), np.array([float(duration)])


class Pieces:
    """A candidate under construction: quadratic Bezier pieces starting at relative time 0."""

    def __init__(self, q):
        self.controls, self.durations = [np.empty((0, 3, 7))], [np.empty(0)]
        self.q = np.asarray(q, float)
        self.labels = []

    def copy(self):
        out = Pieces(self.q)
        out.controls, out.durations, out.labels = list(self.controls), list(self.durations), list(self.labels)
        return out

    @property
    def duration(self):
        return float(sum(np.sum(d) for d in self.durations))

    def add(self, piece, label=None):
        c, d = piece
        if len(d):
            self.controls.append(c)
            self.durations.append(d)
            self.q = np.asarray(c[-1, -1], float)
        if label:
            self.labels.append((label, self.duration))
        return self

    def to(self, target, limits, label=None):
        return self.add(p2p(self.q, target, limits.velocity, limits.acceleration), label)

    def route(self, waypoints, limits, label=None):
        for w in waypoints:
            if np.max(np.abs(np.asarray(w) - self.q)) > 1e-9:
                self.to(w, limits)
        if label:
            self.labels.append((label, self.duration))
        return self

    def wait(self, duration, label=None):
        return self.add(hold(self.q, duration), label)

    def arrays(self):
        c = np.concatenate(self.controls) if len(self.controls) > 1 else np.empty((0, 3, 7))
        d = np.concatenate(self.durations) if len(self.durations) > 1 else np.empty(0)
        if not len(d):
            c, d = hold(self.q, 1e-3)
        return c, d

    def candidate(self):
        c, d = self.arrays()
        return dict(kind="bezier", objective=TIMED, controls=c.tolist(), durations=d.tolist())


def evaluate(controls, durations, times):
    """Positions and velocities (T,7) of Bezier pieces at relative ``times``; the end pose is held."""
    controls, durations = np.asarray(controls, float), np.asarray(durations, float)
    times = np.asarray(times, float)
    ends = np.cumsum(durations)
    starts = ends - durations
    k = np.clip(np.searchsorted(ends, times, side="right"), 0, len(durations) - 1)
    s = np.clip((times - starts[k]) / durations[k], 0.0, 1.0)
    P = controls[k]
    after = times >= ends[-1]
    s = np.where(after, 1.0, s)
    q = ((1 - s) ** 2)[:, None] * P[:, 0] + (2 * s * (1 - s))[:, None] * P[:, 1] + (s ** 2)[:, None] * P[:, 2]
    v = (2.0 / durations[k])[:, None] * ((1 - s)[:, None] * (P[:, 1] - P[:, 0]) + s[:, None] * (P[:, 2] - P[:, 1]))
    v[after] = 0.0
    return q, v


def plan_state(plan, times):
    """Positions and velocities (T,7) of a benchmark Plan at absolute ``times``."""
    states = [plan.at(t) for t in times]
    return np.asarray([s[0] for s in states]), np.asarray([s[1] for s in states])


def path_samples(controls, durations, step=0.03):
    """Configurations along a candidate spaced at most ``step`` rad apart (for static checks)."""
    controls = np.asarray(controls, float)
    out = [controls[0, 0]]
    for P in controls:
        length = np.linalg.norm(P[1] - P[0]) + np.linalg.norm(P[2] - P[1])
        n = max(1, int(np.ceil(length / step)))
        s = np.linspace(0, 1, n + 1)[1:]
        out.append(((1 - s) ** 2)[:, None] * P[0] + (2 * s * (1 - s))[:, None] * P[1] + (s ** 2)[:, None] * P[2])
    return np.vstack([np.atleast_2d(o) for o in out])


def validate(plan, start_time, candidate, limits, earliest):
    """The benchmark's own submission check (continuity, rest, limits)."""
    _, reason, details = check_submission(plan, start_time, candidate, limits.check, earliest)
    return reason, details


def accel_pieces(q0, v0, accelerations, dt):
    """Quadratic Bezier pieces for piecewise-constant accelerations (K, 7) of duration ``dt`` from (q0, v0).

    Returns (controls (K, 3, 7), durations (K,), final q, final v).
    """
    q, v = np.asarray(q0, float).copy(), np.asarray(v0, float).copy()
    controls = []
    for a in np.asarray(accelerations, float).reshape(-1, 7):
        q1 = q + v * dt + 0.5 * a * dt * dt
        controls.append(np.stack([q, q + 0.5 * dt * v, q1]))
        q, v = q1, v + a * dt
    return np.asarray(controls).reshape(-1, 3, 7), np.full(len(controls), float(dt)), q, v


def bezier_candidate(controls, durations):
    return dict(kind="bezier", objective=TIMED, controls=np.asarray(controls).tolist(),
                durations=np.asarray(durations).tolist())


class RouteReference:
    """The nominal route sampled densely, with arc length and fast static clearance at every sample.

    Used by the baselines that track the route with soft costs (MPPI, VP-STO): deviation and progress come from
    the nearest sample, and the static cost penalizes being closer to the static scene than the route itself is
    at that point (the fast sphere model is conservative, so tight but valid route sections are not penalized).
    """

    def __init__(self, route, arm, scene, step=0.02, margin=0.03):
        points = [np.asarray(route[0], float)]
        for a, b in zip(route[:-1], route[1:]):
            a, b = np.asarray(a, float), np.asarray(b, float)
            n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
            points += list(a + (b - a) * (np.arange(1, n + 1) / n)[:, None])
        self.points = np.asarray(points)
        self.arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.points, axis=0), axis=1))]
        self.length = float(self.arc[-1])
        self.clearance = scene.distance(arm.spheres(self.points), arm.sphere_radius).min(axis=1)
        self.reference = np.minimum(self.clearance, margin)

    def project(self, Q):
        """Nearest route sample of configurations Q (..., 7): (deviation, arc length, reference clearance)."""
        flat = np.asarray(Q, float).reshape(-1, 7)
        d2 = (np.square(flat).sum(1)[:, None] - 2.0 * flat @ self.points.T + np.square(self.points).sum(1)[None])
        k = np.argmin(d2, axis=1)
        shape = np.asarray(Q).shape[:-1]
        e = np.sqrt(np.maximum(d2[np.arange(len(flat)), k], 0.0))
        return e.reshape(shape), self.arc[k].reshape(shape), self.reference[k].reshape(shape)


class RouteHandoff:
    """Hand a local method's motion back to the nominal route near the goal or when it stalls.

    MPPI and fabrics are local controllers; as deployed they track a global path. Here the nominal route is that
    path: once the robot is within ``finish_arc`` (path length) of the goal and close to the route, or has made no
    progress along the route for ``stall_s``, the method brakes, joins the nearest route sample (if that segment is
    statically clear) and follows the rest of the route. Control returns to the local method if a moving obstacle
    conflicts with the hand-off plan.
    """

    def handoff_setup(self, finish_arc=0.6, near_deviation=0.25, stall_s=2.0, stall_progress=0.05,
                      join_deviation=0.5):
        self.finish_arc, self.near_deviation, self.stall_s = finish_arc, near_deviation, stall_s
        self.stall_progress, self.join_deviation = stall_progress, join_deviation
        self.handed_off, self.best_progress = False, (-np.inf, -np.inf)

    def handoff(self, observation, plan, t_s, qs, vs, obstacles, start, margin, protective):
        """Returns ("keep", None), ("submit", response) or ("control", None) (the local method decides)."""
        t = observation["time_s"]
        e0, s0, _ = self.reference.project(qs[None])
        e0, s0 = float(e0[0]), float(s0[0])
        if self.handed_off:
            Q, V = plan_state(plan, t_s + self.tau)
            clear = monitor_margins(self.arm, obstacles, Q[None], V[None], self.tau, self.supervisor,
                                    self.limits.acceleration, ("cv",), margin, protective=protective)
            if not len(obstacles) or float(clear.min()) >= 0.0:
                return "keep", None
            self.handed_off, self.best_progress = False, (s0, t)
            return "control", None
        if s0 > self.best_progress[0] + self.stall_progress:
            self.best_progress = (s0, t)
        near = self.reference.length - s0 < self.finish_arc and e0 < self.near_deviation
        stalled = t - self.best_progress[1] > self.stall_s and e0 < self.join_deviation
        near = near or np.max(np.abs(qs - self.goal)) <= 0.12
        if not ((near or stalled) and np.max(np.abs(vs)) < 0.6):
            return "control", None
        stop = brake(qs, vs, self.limits.acceleration)
        q_b = np.asarray(stop[0][-1, -1], float) if len(stop[1]) else np.asarray(qs, float)
        _, s_b, _ = self.reference.project(q_b[None])
        k = min(int(np.searchsorted(self.reference.arc, float(s_b[0]))), len(self.reference.points) - 1)
        join = self.reference.points[k]
        samples = np.linspace(q_b, join, max(2, int(np.ceil(np.max(np.abs(join - q_b)) / 0.04)) + 1))
        if self.scene.distance(self.arm.spheres(samples), self.arm.sphere_radius).min() < min(
                float(self.reference.clearance[k]), 0.0) - 0.03:
            self.best_progress = (s0, t)  # joining would hit the static scene; let the local method continue
            return "control", None
        pieces = Pieces(qs).add(stop).route([join] + (remaining_route(self.route, join) or [self.goal]), self.limits)
        candidate = pieces.candidate()
        reason, _ = validate(plan, t_s, candidate, self.limits, t + time.perf_counter() - start)
        if reason != "accepted":
            return "control", None
        self.handed_off = True
        return "submit", dict(start_time_s=t_s, trajectory=candidate)


def common_setup(method, problem, arm=None):
    """Shared setup of the baselines: kinematics, limits, static scene, protocol, goal and nominal route."""
    method.arm = arm or ArmModel()
    method.limits = Limits(problem, method.arm.lower, method.arm.upper)
    method.voxels = np.asarray(problem["scene"]["static_voxels_xyz"], float)
    method.scene = StaticScene(method.voxels, fixed_equipment(method.arm.model))
    method.supervisor = problem["protocol"]["supervisor"]
    method.period = float(problem["protocol"]["observation_period_s"])
    method.goal = np.asarray(problem["goal"]["q"], float)
    route = [np.asarray(w, float) for w in problem["nominal_waypoints"]]
    if np.max(np.abs(route[-1] - method.goal)) > 1e-9:
        route.append(method.goal)
    method.route = route


# ----------------------------------------------------------------------------- roadmap

class Roadmap:
    """Static roadmap over the public scene, built at setup and searched online.

    Nodes are the nominal route (waypoints and samples, statically valid by construction) and random
    configurations around it. A node or straight edge is kept if the conservative sphere model clears the
    static scene, or, when that model is borderline, if the exact canonical checker does. Edge samples are kept
    so that edges blocked by a moving obstacle are found online with one batched distance query.
    """

    def __init__(self, nodes, edges, lengths, samples, owners, route_count):
        self.nodes, self.edges, self.lengths = nodes, edges, lengths
        self.samples, self.owners, self.route_count = samples, owners, int(route_count)
        self.adjacency = [[] for _ in range(len(nodes))]
        for e, (a, b) in enumerate(edges):
            self.adjacency[a].append((b, e))
            self.adjacency[b].append((a, e))
        self.sample_spheres, self.box_sphere_index = None, None

    @classmethod
    def build(cls, arm, scene, voxels, route, limits, rng, random_nodes=450, neighbours=10, step=0.04,
              margin=0.01, exact_margin=0.005, borderline=0.05, spread=0.6, exact_budget=1500, self_check=True):
        route = [np.asarray(w, float) for w in route]
        checker_self = arm.self_and_fixed_clearance if self_check else None
        trusted = [route[0]]
        for a, b in zip(route[:-1], route[1:]):
            count = max(1, int(np.ceil(np.max(np.abs(b - a)) / 0.12)))
            trusted += list(np.linspace(a, b, count + 1)[1:])
        trusted = np.asarray(trusted)
        anchors = trusted
        extra = limits.clip(anchors[rng.integers(len(anchors), size=random_nodes)] + rng.normal(0.0, spread, (random_nodes, 7)))
        fast = scene.distance(arm.spheres(extra), arm.sphere_radius).min(axis=1)
        keep, budget = [], exact_budget
        for q, f in zip(extra, fast):
            if f < -borderline:
                continue
            exact = arm.exact_clearance(q, voxels, fixed=True, cutoff=0.1)[0] if (f < margin and budget > 0) else None
            if exact is not None:
                budget -= 1
            if (f >= margin and arm.self_and_fixed_clearance(q)[0] > exact_margin) or (exact is not None and exact >= exact_margin):
                keep.append(q)
        Q = np.vstack([trusted] + ([np.asarray(keep)] if keep else []))
        n_route = len(trusted)
        edges = [(i, i + 1) for i in range(n_route - 1)]
        weights = np.linalg.norm(Q[:, None, :] - Q[None, :, :], axis=-1)
        pending = set()
        for i in range(len(Q)):
            for j in np.argsort(weights[i])[1:neighbours + 1]:
                a, b = min(i, int(j)), max(i, int(j))
                if not (b == a + 1 and b < n_route):
                    pending.add((a, b))
        pending = sorted(pending)
        segments = [np.linspace(Q[a], Q[b], max(2, int(np.ceil(np.max(np.abs(Q[b] - Q[a])) / step)) + 1))
                    for a, b in pending]
        if segments:
            flat = np.vstack(segments)
            clearance = scene.distance(arm.spheres(flat), arm.sphere_radius).min(axis=1)
            offset = 0
            for (a, b), seg in zip(pending, segments):
                worst = clearance[offset:offset + len(seg)].min()
                offset += len(seg)
                ok = worst >= margin
                if not ok and worst >= -borderline and budget > 0:
                    sparse = seg[::3] if len(seg) > 3 else seg
                    budget -= len(sparse)
                    ok = arm.exact_clearance(sparse, voxels, fixed=True, cutoff=0.1).min() >= exact_margin
                if ok:
                    edges.append((a, b))
        if checker_self is not None and len(edges) > n_route - 1:
            # Self-collision and fixed equipment (exact Drake query) on the interior of every non-route edge.
            keep_edges = edges[:n_route - 1]
            for a, b in edges[n_route - 1:]:
                seg = np.linspace(Q[a], Q[b], max(2, int(np.ceil(np.max(np.abs(Q[b] - Q[a])) / step)) + 1))[1:-1]
                if not len(seg) or checker_self(seg).min() > exact_margin:
                    keep_edges.append((a, b))
            edges = keep_edges
        lengths, samples, owners = [], [], []
        for e, (a, b) in enumerate(edges):
            lengths.append(float(np.linalg.norm(Q[b] - Q[a])))
            seg = np.linspace(Q[a], Q[b], max(2, int(np.ceil(np.max(np.abs(Q[b] - Q[a])) / step)) + 1))
            samples.append(seg)
            owners.append(np.full(len(seg), e))
        return cls(Q, np.asarray(edges, int).reshape(-1, 2), np.asarray(lengths), np.vstack(samples),
                   np.concatenate(owners), n_route)

    def prepare(self, arm):
        """Cache edge-sample box centres and covering spheres (float32) and per-edge bounding boxes."""
        if self.sample_spheres is None:
            chunks = np.array_split(self.samples, max(1, len(self.samples) // 4000))
            self.sample_spheres = np.concatenate([arm.boxes(chunk)[1] for chunk in chunks]).astype(np.float32)
            self.sample_cover = np.concatenate([arm.spheres(chunk) for chunk in chunks]).astype(np.float32)
            self.box_reach = np.linalg.norm(arm.half, axis=1).astype(np.float32)
            reach = float(self.box_reach.max())
            lo = self.sample_spheres.min(axis=1) - reach
            hi = self.sample_spheres.max(axis=1) + reach
            self.edge_low = np.full((len(self.edges), 3), np.inf)
            self.edge_high = np.full((len(self.edges), 3), -np.inf)
            np.minimum.at(self.edge_low, self.owners, lo)
            np.maximum.at(self.edge_high, self.owners, hi)
        return self

    def edge_clearance(self, arm, obstacles, positions, reach_margin):
        """Per-edge clearance of the covering spheres to the obstacles at ``positions`` (inf beyond reach).

        ``positions`` maps obstacle index to (P, 3) predicted centres. A first stage bounds robot boxes and
        obstacles by spheres (box circumradius) and keeps only samples within ``reach_margin``; the second
        stage measures the cached covering spheres against the obstacle primitive itself.
        """
        if self.box_sphere_index is None:
            per_box = [np.flatnonzero(arm.sphere_owner == g) for g in range(len(arm.half))]
            width = max(len(ids) for ids in per_box)
            self.box_sphere_index = np.full((len(per_box), width), -1)
            for g, ids in enumerate(per_box):
                self.box_sphere_index[g, :len(ids)] = ids
        clearance = np.full(len(self.edges), np.inf)
        for j, centres in positions.items():
            item = obstacles.items[j]
            extent = item["radius"] if item["shape"] == "sphere" else float(np.linalg.norm(item["half"]))
            for p in np.atleast_2d(centres).astype(float):
                gap = np.maximum(np.maximum(self.edge_low - p, p - self.edge_high), 0.0)
                near = np.linalg.norm(gap, axis=1) < extent + reach_margin + VOXEL_RADIUS_M
                if not near.any():
                    continue
                rows = np.flatnonzero(near[self.owners])
                # Stage 1: each robot box bounded by its circumsphere, against the obstacle primitive itself.
                d = obstacles.surface_distance(j, self.sample_spheres[rows][None], p[None])[0] - self.box_reach
                r, g = np.nonzero(d < reach_margin + VOXEL_RADIUS_M)
                if not len(r):
                    continue
                # Stage 2: only the covering spheres of the boxes that came close.
                ids = self.box_sphere_index[g]
                valid = ids >= 0
                r = np.broadcast_to(r[:, None], ids.shape)[valid]
                ids = ids[valid]
                points = self.sample_cover[rows[r], ids]
                dist = obstacles.surface_distance(j, points[None], p[None])[0] - VOXEL_RADIUS_M - arm.sphere_radius[ids]
                np.minimum.at(clearance, self.owners[rows[r]], dist)
        return clearance

    def blocked(self, arm, obstacles, margin, positions):
        """Edges with any sample whose covering spheres come within ``margin`` of an obstacle at ``positions``."""
        return self.edge_clearance(arm, obstacles, positions, margin) < margin

    def goal_tree(self, goal):
        """Next hop towards ``goal`` from every node (static shortest paths, -1 if unreachable)."""
        dist = np.full(len(self.nodes), np.inf)
        nxt = np.full(len(self.nodes), -1)
        dist[goal] = 0.0
        heap = [(0.0, int(goal))]
        while heap:
            d, u = heapq.heappop(heap)
            if d > dist[u]:
                continue
            for w, e in self.adjacency[u]:
                nd = d + self.lengths[e]
                if nd < dist[w]:
                    dist[w], nxt[w] = nd, u
                    heapq.heappush(heap, (nd, w))
        self.to_goal, self.goal_distance = nxt, dist
        return self

    def path_to_goal(self, node):
        if not np.isfinite(self.goal_distance[node]):
            return None
        out = [int(node)]
        while self.to_goal[out[-1]] != -1:
            out.append(int(self.to_goal[out[-1]]))
        return out

    def nearest(self, q, count=8):
        d = np.linalg.norm(self.nodes - np.asarray(q)[None], axis=1)
        return np.argsort(d)[:count]

    def search(self, start, goal, blocked=None):
        """Shortest node path (list of node ids) from ``start`` to ``goal`` avoiding ``blocked`` edges."""
        blocked = np.zeros(len(self.edges), bool) if blocked is None else blocked
        dist = np.full(len(self.nodes), np.inf)
        prev = np.full(len(self.nodes), -1)
        dist[start] = 0.0
        heap = [(0.0, int(start))]
        while heap:
            d, u = heapq.heappop(heap)
            if u == goal:
                break
            if d > dist[u]:
                continue
            for w, e in self.adjacency[u]:
                if blocked[e]:
                    continue
                nd = d + self.lengths[e]
                if nd < dist[w]:
                    dist[w], prev[w] = nd, u
                    heapq.heappush(heap, (nd, w))
        if not np.isfinite(dist[goal]):
            return None
        path, u = [], int(goal)
        while u != -1:
            path.append(u)
            u = int(prev[u])
        return path[::-1]

    def save(self, path):
        np.savez_compressed(path, nodes=self.nodes, edges=self.edges, lengths=self.lengths, samples=self.samples,
                            owners=self.owners, route_count=self.route_count)

    @classmethod
    def load(cls, path):
        z = np.load(path)
        return cls(z["nodes"], z["edges"], z["lengths"], z["samples"], z["owners"], int(z["route_count"]))


def scene_key(problem, extra):
    """Cache key of a roadmap: the public static scene, the route and the build settings."""
    payload = json.dumps(dict(static=problem["scene"]["static_primitives"], route=problem["nominal_waypoints"],
                              extra=extra), sort_keys=True, default=float)
    return hashlib.sha256(payload.encode()).hexdigest()[:20]


def cached_roadmap(problem, arm, scene, limits, settings, cache_dir="roadmaps"):
    """Build the roadmap once per public scene and route (deterministic seed); reuse it across episodes."""
    key = scene_key(problem, dict(settings, version=2))
    path = Path(cache_dir) / f"{key}.npz"
    if path.exists():
        try:
            return Roadmap.load(path).prepare(arm)
        except Exception:
            pass
    rng = np.random.default_rng(int(key[:8], 16))
    route = [np.asarray(w, float) for w in problem["nominal_waypoints"]]
    if np.max(np.abs(route[-1] - np.asarray(problem["goal"]["q"], float))) > 1e-9:
        route.append(np.asarray(problem["goal"]["q"], float))
    voxels = np.asarray(problem["scene"]["static_voxels_xyz"], float)
    roadmap = Roadmap.build(arm, scene, voxels, route, limits, rng, **settings)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.stem + f".{time.time_ns()}.tmp.npz")
    roadmap.save(tmp)
    tmp.replace(path)
    return roadmap.prepare(arm)


def current_plan(observation):
    return Plan.from_list(observation["plan"])
