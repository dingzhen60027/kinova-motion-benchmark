"""Task generation with method-independent acceptance.

A task combines a static workspace layout, semantic pick/place endpoints solved
by inverse kinematics, the nominal motion the robot is already executing at
``t = 0``, independently moving obstacles and a privileged completion witness.
Acceptance uses geometry, the nominal motion and the witness only; no evaluated
method is consulted. Rejected proposals are counted by reason, and a task that
exhausts its attempts stays registered as a generation failure.

Disturbance validity. A disturbance obstacle is at least 0.25 m from the robot
at ``t = 0`` and first comes within 5 cm of the nominal motion no earlier than
1.0 s after its observable onset (its departure, or ``t = 0`` if it is already
moving when the episode starts). Executing the nominal motion unchanged must
collide with it. The witness follows the nominal motion until at least the
onset, so no task demands a reaction before the disturbance is observable.
"""
from __future__ import annotations

import json
from pathlib import Path
import time
import traceback

import numpy as np

from benchmarks.generic_motion_v1.generator.scene import domain_seed, rng as make_rng
from benchmarks.generic_motion_v1.model import Model

from . import BENCHMARK
from .harness import (InProcessPlanner, clean, digest, load_config, motion_limits, protective_separation,
                      run_episode, source_hashes, write_json)
from .trajectory import Plan, Segment, braking, concatenate, hold, point_to_point, serializable
from .world import DynamicObstacle, GeometryChecker, Motion, World, ompl_path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DATA = HERE / "data"
WORKSPACES = ("shelf", "workstation", "machine")
DOWN = np.array([0.0, 0.0, -1.0])
CONFLICT_LINKS = ("forearm_link", "spherical_wrist_1_link", "spherical_wrist_2_link", "bracelet_link",
                  "robotiq_85_base_link")  # disturbances aim at these moving boxes


def link_index(ctx, name):
    return next(i for i, n in enumerate(ctx.checker.names) if f"::{name}_collision" in n)


class Rejected(Exception):
    """A proposal failed an acceptance rule; the reason is counted."""


def rz(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def unit(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise Rejected("degenerate_direction")
    return v / n


def uniform(random, bounds):
    return float(random.uniform(bounds[0], bounds[1]))


def place(origin, yaw, local, size, name):
    centre = np.asarray(origin, float) + rz(yaw) @ np.asarray(local, float)
    return dict(shape="box", center_m=centre.tolist(), size_m=[float(s) for s in size], yaw_rad=float(yaw), name=name)


# ----------------------------------------------------------------------------- primitive geometry

def _sphere_box_distance(centre, box):
    R = rz(box.get("yaw_rad", 0.0))
    local = R.T @ (np.asarray(centre, float) - np.asarray(box["center_m"], float))
    d = np.abs(local) - np.asarray(box["size_m"], float) / 2
    return float(np.linalg.norm(np.maximum(d, 0.0)) + min(d.max(), 0.0))


def overlap(a, b, margin=0.0):
    """Exact overlap test for z-yaw boxes and spheres, inflated by ``margin``."""
    if a["shape"] == "sphere" and b["shape"] == "sphere":
        return np.linalg.norm(np.subtract(a["center_m"], b["center_m"])) < a["radius_m"] + b["radius_m"] + margin
    if a["shape"] == "sphere" or b["shape"] == "sphere":
        sphere, box = (a, b) if a["shape"] == "sphere" else (b, a)
        return _sphere_box_distance(sphere["center_m"], box) < sphere["radius_m"] + margin
    ca, cb = np.asarray(a["center_m"], float), np.asarray(b["center_m"], float)
    ha, hb = np.asarray(a["size_m"], float) / 2, np.asarray(b["size_m"], float) / 2
    if abs(ca[2] - cb[2]) >= ha[2] + hb[2] + margin:
        return False
    axes_a, axes_b = rz(a.get("yaw_rad", 0.0))[:2, :2], rz(b.get("yaw_rad", 0.0))[:2, :2]
    delta = (cb - ca)[:2]
    for axis in np.hstack([axes_a, axes_b]).T:
        ra = np.sum(np.abs(axis @ axes_a) * ha[:2])
        rb = np.sum(np.abs(axis @ axes_b) * hb[:2])
        if abs(delta @ axis) >= ra + rb + margin:
            return False
    return True


def bottom(primitive):
    c = primitive["center_m"][2]
    return c - (primitive["radius_m"] if primitive["shape"] == "sphere" else primitive["size_m"][2] / 2)


def fixed_primitives(model):
    """Canonical fixed equipment (robot base, table, tripods, wall) as world primitives."""
    out = []
    inventory = model.inventory
    for g, pose in zip(inventory["geometries"], inventory["zero_configuration_geometry_world_poses"]):
        if g["moving_robot"]:
            continue
        T = np.asarray(pose)
        out.append(dict(shape=g["shape"], center_m=T[:3, 3].tolist(), size_m=g["dimensions_m"],
                        yaw_rad=float(np.arctan2(T[1, 0], T[0, 0])), name=g["name"],
                        table=g["name"].startswith("table::")))
    return out


def plausible(primitive, static, fixed, margin=0.005):
    """Source primitive above the table and clear of furniture and fixed equipment."""
    if bottom(primitive) < -1e-9:
        return False
    return not any(overlap(primitive, other, margin) for other in static) and \
        not any(overlap(primitive, other, margin) for other in fixed if not other["table"])


# ----------------------------------------------------------------------------- kinematics

class Kinematics:
    """Tool point and inverse kinematics on the canonical Drake plant."""

    def __init__(self, model, tool_offset, ik):
        self.model, self.plant, self.context = model, model.plant, model.plant_context
        self.ee = self.plant.GetBodyByName("end_effector_link", self.plant.GetModelInstanceByName("kinova"))
        self.offset, self.ik = float(tool_offset), ik

    def tool(self, q):
        self.plant.SetPositions(self.context, np.asarray(q, float))
        X = self.plant.EvalBodyPoseInWorld(self.context, self.ee)
        R = X.rotation().matrix()
        return X.translation() + self.offset * R[:, 2], R[:, 2].copy()

    def solve(self, position, approach, seed, angle_deg=None, preferred=None, level_fingers=False):
        """IK from ``seed`` with a joint-space cost toward ``preferred`` (default: the seed).

        ``level_fingers`` keeps the finger opening axis (end-effector x) horizontal.
        """
        from pydrake.multibody.inverse_kinematics import InverseKinematics
        from pydrake.solvers import Solve
        tol = self.ik["position_tolerance_m"]
        angle = np.radians(self.ik["angle_tolerance_deg"] if angle_deg is None else angle_deg)
        problem = InverseKinematics(self.plant)
        frame = self.ee.body_frame()
        problem.AddPositionConstraint(frame, [0, 0, self.offset], self.plant.world_frame(),
                                      np.asarray(position) - tol, np.asarray(position) + tol)
        problem.AddAngleBetweenVectorsConstraint(self.plant.world_frame(), unit(approach), frame, [0, 0, 1], 0, angle)
        if level_fingers:
            problem.AddAngleBetweenVectorsConstraint(self.plant.world_frame(), [0, 0, 1], frame, [1, 0, 0],
                                                     np.radians(80), np.radians(100))
        program, q = problem.get_mutable_prog(), problem.q()
        program.AddQuadraticErrorCost(0.01 * np.eye(7), seed if preferred is None else preferred, q)
        program.SetInitialGuess(q, seed)
        result = Solve(program)
        return np.asarray(result.GetSolution(q)) if result.is_success() else None

    def best(self, position, approach, reference, random, accept, angle_deg=None, extra_seeds=(), level_fingers=False):
        """Accepted IK solution closest to ``reference`` (seeded near it and from ``extra_seeds``)."""
        lower, upper = self.model.lower, self.model.upper
        seeds = [np.clip(reference, lower, upper)] + [np.clip(s, lower, upper) for s in extra_seeds]
        seeds += [np.clip(reference + random.normal(0, 0.5, 7), lower, upper)
                  for _ in range(max(0, self.ik["attempts"] - len(seeds)))]
        found = []
        for seed in seeds:
            q = self.solve(position, approach, seed, angle_deg, preferred=reference, level_fingers=level_fingers)
            if q is not None and accept(q):
                found.append(q)
        return min(found, key=lambda q: float(np.linalg.norm(q - reference))) if found else None

    def continuation(self, start, target, approach, steps, level_fingers=False):
        """Follow the straight tool line from configuration ``start`` to ``target`` by small IK steps."""
        q = np.asarray(start, float)
        origin = self.tool(q)[0]
        for k in range(1, steps + 1):
            q = self.solve(origin + (np.asarray(target) - origin) * k / steps, approach, q, preferred=q,
                           level_fingers=level_fingers)
            if q is None:
                return None
        return q

    def deviation(self, a, b, samples=21):
        """Largest tool-point distance from the straight line while interpolating joints a -> b."""
        pa, pb = self.tool(a)[0], self.tool(b)[0]
        axis = unit(pb - pa) if np.linalg.norm(pb - pa) > 1e-9 else np.zeros(3)
        worst = 0.0
        for s in np.linspace(0, 1, samples):
            d = self.tool(a + s * (np.asarray(b) - a))[0] - pa
            worst = max(worst, float(np.linalg.norm(d - (d @ axis) * axis)))
        return worst


def reference_posture(position):
    """Elbow-up seed facing the site."""
    return np.array([np.arctan2(position[1], position[0]), 0.55, 0.0, 1.9, 0.0, 0.8, 0.0])


# ----------------------------------------------------------------------------- layouts

def _tote(random, cfg, bearing):
    p = cfg["generator"]["workspaces"]["tote"]
    side = float(random.choice([-1.0, 1.0]))
    angle = bearing + side * np.radians(uniform(random, p["side_bearing_deg"]))
    radius = uniform(random, p["radius_m"])
    size = [uniform(random, s) for s in p["size_m"]]
    centre = np.array([radius * np.cos(angle), radius * np.sin(angle), size[2] / 2])
    primitive = dict(shape="box", center_m=centre.tolist(), size_m=size, yaw_rad=float(angle), name="tote")
    site = centre + np.array([0, 0, size[2] / 2 + uniform(random, p["approach_height_m"])])
    return primitive, site


def shelf_layout(random, cfg):
    p = cfg["generator"]["workspaces"]["shelf"]
    yaw = np.radians(uniform(random, p["bearing_deg"]))
    origin = uniform(random, p["front_distance_m"]) * np.array([np.cos(yaw), np.sin(yaw), 0.0])
    W, D, t = uniform(random, p["width_m"]), uniform(random, p["depth_m"]), p["panel_m"]
    zb, H = uniform(random, p["plinth_m"]), uniform(random, p["compartment_m"])
    z = [zb + k * (t + H) for k in range(3)]
    top = z[2] + t
    prims = [place(origin, yaw, [D / 2, 0, zb / 2], [D, W, zb], "shelf_plinth")]
    prims += [place(origin, yaw, [D / 2, 0, z[k] + t / 2], [D, W, t], f"shelf_board_{k}") for k in range(3)]
    prims += [place(origin, yaw, [D / 2, s * (W / 2 - t / 2), (zb + top) / 2], [D, t, top - zb], "shelf_side")
              for s in (-1, 1)]
    prims.append(place(origin, yaw, [D - t / 2, 0, (zb + top) / 2], [t, W - 2 * t, top - zb], "shelf_back"))
    tote, start = _tote(random, cfg, yaw)
    compartments = [dict(level=k, z_low=z[k] + t, z_high=z[k + 1], lateral=[-W / 2 + t + 0.15, W / 2 - t - 0.15])
                    for k in range(2)]
    return dict(workspace="shelf", origin=origin.tolist(), yaw_rad=float(yaw), static_primitives=prims + [tote],
                start_site=start.tolist(), compartments=compartments,
                parameters=dict(width_m=W, depth_m=D, plinth_m=zb, compartment_m=H))


def machine_layout(random, cfg):
    p = cfg["generator"]["workspaces"]["machine"]
    yaw = np.radians(uniform(random, p["bearing_deg"]))
    origin = uniform(random, p["front_distance_m"]) * np.array([np.cos(yaw), np.sin(yaw), 0.0])
    W, D, t = uniform(random, p["width_m"]), uniform(random, p["depth_m"]), p["panel_m"]
    zf, H = uniform(random, p["stand_m"]), uniform(random, p["interior_m"])
    wo, ho = uniform(random, p["opening_width_m"]), uniform(random, p["opening_height_m"])
    yo = uniform(random, [-(W / 2 - t - wo / 2 - 0.02), W / 2 - t - wo / 2 - 0.02])
    zo = zf + uniform(random, [0.02, max(0.03, H - ho - 0.02)])
    mid = zf + H / 2
    prims = [place(origin, yaw, [D / 2, 0, zf / 2], [D, W, zf], "machine_stand"),
             place(origin, yaw, [D / 2, 0, zf + H + t / 2], [D, W, t], "machine_roof"),
             place(origin, yaw, [D - t / 2, 0, mid], [t, W - 2 * t, H], "machine_back")]
    prims += [place(origin, yaw, [D / 2, s * (W / 2 - t / 2), mid], [D, t, H], "machine_side") for s in (-1, 1)]
    left, right = yo + wo / 2, yo - wo / 2
    prims.append(place(origin, yaw, [t / 2, (left + W / 2) / 2, mid], [t, W / 2 - left, H], "machine_front_left"))
    prims.append(place(origin, yaw, [t / 2, (right - W / 2) / 2, mid], [t, right + W / 2, H], "machine_front_right"))
    if zo - zf > 0.005:
        prims.append(place(origin, yaw, [t / 2, yo, (zf + zo) / 2], [t, wo, zo - zf], "machine_front_bottom"))
    if zf + H - zo - ho > 0.005:
        prims.append(place(origin, yaw, [t / 2, yo, (zo + ho + zf + H) / 2], [t, wo, zf + H - zo - ho], "machine_front_top"))
    tote, start = _tote(random, cfg, yaw)
    centre = np.asarray(origin) + rz(yaw) @ np.array([0.0, yo, 0.0]) + np.array([0, 0, zo + ho / 2])
    return dict(workspace="machine", origin=origin.tolist(), yaw_rad=float(yaw), static_primitives=prims + [tote],
                start_site=start.tolist(),
                opening=dict(centre_m=centre.tolist(), width_m=wo, height_m=ho, lateral=float(yo)),
                parameters=dict(width_m=W, depth_m=D, stand_m=zf, interior_m=H, opening_width_m=wo, opening_height_m=ho))


def workstation_layout(random, cfg):
    """Pick bin and assembly fixture on opposite sides of a cluttered table area."""
    p = cfg["generator"]["workspaces"]["workstation"]
    yaw = np.radians(uniform(random, p["bearing_deg"]))
    radius = uniform(random, p["centre_distance_m"])
    centre = radius * np.array([np.cos(yaw), np.sin(yaw), 0.0])
    spread = np.radians(uniform(random, p["spread_deg"]))
    items = []
    for name, sizes, sign in (("bin", p["bin_m"], 1.0), ("fixture", p["fixture_m"], -1.0)):
        size = [uniform(random, s) for s in sizes]
        angle = yaw + sign * spread
        r = radius + uniform(random, [-0.04, 0.04])
        items.append(dict(shape="box", center_m=[r * np.cos(angle), r * np.sin(angle), size[2] / 2], size_m=size,
                          yaw_rad=float(angle), name=name))
    for k in range(int(random.integers(p["clutter"][0], p["clutter"][1] + 1))):
        for _ in range(30):
            size = [uniform(random, p["clutter_m"]), uniform(random, p["clutter_m"]), uniform(random, p["clutter_height_m"])]
            angle = yaw + uniform(random, [-0.7, 0.7]) * spread
            r = radius + uniform(random, [-0.12, 0.15])
            candidate = dict(shape="box", center_m=[r * np.cos(angle), r * np.sin(angle), size[2] / 2], size_m=size,
                             yaw_rad=float(angle + random.uniform(-0.6, 0.6)), name=f"clutter_{k}")
            if not any(overlap(candidate, other, 0.03) for other in items):
                items.append(candidate)
                break
    sites = []
    for item in items[:2]:
        top = np.asarray(item["center_m"]) + np.array([0, 0, item["size_m"][2] / 2])
        sites.append((top + np.array([0, 0, uniform(random, p["approach_height_m"])])).tolist())
    transit = max(np.asarray(i["center_m"])[2] + i["size_m"][2] / 2 for i in items) + 0.15
    return dict(workspace="workstation", origin=centre.tolist(), yaw_rad=float(yaw), static_primitives=items,
                sites=sites, transit_height_m=float(transit), parameters=dict(spread_deg=float(np.degrees(spread))))


LAYOUTS = dict(shelf=shelf_layout, machine=machine_layout, workstation=workstation_layout)


def make_layout(workspace, random, cfg, fixed):
    for attempt in range(50):
        layout = LAYOUTS[workspace](random, cfg)
        prims = layout["static_primitives"]
        if any(not plausible(p, [], fixed) for p in prims):
            continue
        if workspace != "workstation" and any(overlap(prims[-1], p, 0.05) for p in prims[:-1]):
            continue  # the pick tote stays clear of the furniture
        # Footprints stay clear of the robot base and on the table.
        corners = []
        for p in prims:
            h = np.asarray(p["size_m"]) / 2
            for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
                corners.append(np.asarray(p["center_m"])[:2] + rz(p["yaw_rad"])[:2, :2] @ (h[:2] * [sx, sy]))
        corners = np.asarray(corners)
        if np.min(np.linalg.norm(corners, axis=1)) < 0.22 or np.any(np.abs(corners[:, 1]) > 1.2) or np.any(corners[:, 0] < -0.45):
            continue
        layout["proposals"] = attempt + 1
        return layout
    raise Rejected("layout_geometry")


# ----------------------------------------------------------------------------- task context

class Context:
    """Shared, deterministic generation state for one layout."""

    def __init__(self, layout, cfg, model=None):
        self.cfg, self.layout = cfg, layout
        self.model = model or Model()
        self.checker = GeometryChecker(self.model)
        self.kin = Kinematics(self.model, cfg["robot"]["tool_offset_m"], cfg["generator"]["ik"])
        self.world = World(layout["static_primitives"])
        self.fixed = fixed_primitives(self.model)
        self.limits = motion_limits(cfg, self.model)
        g = cfg["generator"]
        self.nominal_cfg, self.disturbance_cfg, self.witness_cfg = g["nominal"], g["disturbance"], g["witness"]

    def static_ok(self, q, margin=None):
        margin = self.nominal_cfg["static_margin_m"] if margin is None else margin
        return self.checker.clearance(q, self.world.static_xyz, cutoff=0.1) >= margin

    def segment_ok(self, a, b, margin):
        step = self.nominal_cfg["path_step_rad"]
        count = max(2, int(np.ceil(np.linalg.norm(np.subtract(b, a)) / step)) + 1)
        samples = np.linspace(a, b, count)
        for q in samples:
            if self.checker.clearance(q, self.world.static_xyz, cutoff=0.1) < margin:
                return False
        return True


def sites_for_task(ctx, random):
    """Semantic tool sites for one pick-and-place operation.

    Each site is (name, position, approach, mode, level_fingers). ``linear``
    sites are reached along a straight tool line from the previous site (lift,
    descent and insertion); ``free`` sites are joint-space transfers.
    """
    layout, n = ctx.layout, ctx.nominal_cfg
    if layout["workspace"] == "workstation":
        order = [0, 1] if random.random() < 0.5 else [1, 0]
        start, goal = (np.asarray(layout["sites"][i]) for i in order)
        transit = layout["transit_height_m"]
        names = ("bin" if order[0] == 0 else "fixture", "fixture" if order[1] == 1 else "bin")
        return [("pick_" + names[0], start, DOWN, "free", False),
                ("lift", np.r_[start[:2], transit], DOWN, "linear", False),
                ("transfer", np.r_[goal[:2], transit], DOWN, "free", False),
                ("place_" + names[1], goal, DOWN, "linear", False)], "place_" + names[1]
    start = np.asarray(layout["start_site"])
    lift = start + np.array([0, 0, n["lift_m"]])
    R = rz(layout["yaw_rad"])
    inward = R[:, 0]
    origin = np.asarray(layout["origin"])
    if layout["workspace"] == "shelf":
        compartment = layout["compartments"][int(random.integers(0, len(layout["compartments"])))]
        depth = uniform(random, ctx.cfg["generator"]["workspaces"]["shelf"]["inside_m"])
        height = compartment["z_low"] + (compartment["z_high"] - compartment["z_low"]) * uniform(random, [0.42, 0.58])
        goal = origin + R @ np.array([depth, uniform(random, compartment["lateral"]), 0.0]) + np.array([0, 0, height])
        name = f"place_shelf_level_{compartment['level']}"
    else:
        opening = layout["opening"]
        depth = uniform(random, ctx.cfg["generator"]["workspaces"]["machine"]["inside_m"])
        goal = np.asarray(opening["centre_m"]) + depth * inward
        name = "place_in_machine"
    pre = goal - (depth + ctx.cfg["generator"]["workspaces"][layout["workspace"]]["pre_entry_m"]) * inward
    return [("pick_tote", start, DOWN, "free", False), ("lift", lift, DOWN, "linear", False),
            ("pre_entry", pre, inward, "free", True), ("goal", goal, inward, "linear", True)], name


def nominal_motion(ctx, random):
    sites, goal_name = sites_for_task(ctx, random)
    margin = ctx.nominal_cfg["static_margin_m"]
    waypoints, previous = [], None
    for name, position, approach, mode, level in sites:
        if mode == "linear" and previous is not None:
            q = ctx.kin.continuation(previous, position, approach, ctx.nominal_cfg["linear_steps"], level)
            if q is not None and (not ctx.static_ok(q, margin)
                                  or ctx.kin.deviation(previous, q) > ctx.nominal_cfg["linear_deviation_m"]):
                q = None
        else:
            reference = reference_posture(position) if previous is None else previous
            q = ctx.kin.best(position, approach, reference, random, lambda q: ctx.static_ok(q, margin),
                             extra_seeds=[reference_posture(position)], level_fingers=level)
        if q is None:
            raise Rejected("ik_" + name)
        waypoints.append(q)
        previous = q
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        if not ctx.segment_ok(a, b, margin):
            raise Rejected("nominal_static_collision")
    sites = [(name, position, approach) for name, position, approach, _, _ in sites]
    # The programmed motion runs at a speed override; recovery motions may use the full limits.
    candidate, arrivals = point_to_point(waypoints, ctx.limits["velocity"] * ctx.nominal_cfg["speed_scale"],
                                         ctx.limits["acceleration"] * ctx.nominal_cfg["acceleration_scale"], arrivals=True)
    # The episode starts while the robot lifts the part out of the tote or bin.
    offset = arrivals[1] * uniform(random, ctx.nominal_cfg["initial_lift_fraction"])
    plan = Plan([Segment(-offset, candidate)])
    q0, v0, _ = plan.at(0.0)
    return dict(sites=[(n, p.tolist(), a.tolist()) for n, p, a in sites], goal_name=goal_name,
                waypoints=np.asarray(waypoints), arrivals=arrivals - offset, candidate=candidate,
                offset=float(offset), plan=plan, q0=q0, v0=v0, end=float(arrivals[-1] - offset))


# ----------------------------------------------------------------------------- disturbances

def link_state(ctx, plan, t, link, h=0.01):
    _, p0, _ = ctx.checker.poses(plan.at(t - h)[0])
    _, p1, _ = ctx.checker.poses(plan.at(t + h)[0])
    _, p, _ = ctx.checker.poses(plan.at(t)[0])
    return p[link].copy(), (p1[link] - p0[link]) / (2 * h)


def crossing_direction(random, velocity, tilt_deg):
    horizontal = np.array([velocity[0], velocity[1], 0.0])
    if np.linalg.norm(horizontal) > 0.05:
        direction = np.cross([0.0, 0.0, 1.0], horizontal)
    else:
        angle = random.uniform(-np.pi, np.pi)
        direction = np.array([np.cos(angle), np.sin(angle), 0.0])
    direction = unit(direction) * float(random.choice([-1.0, 1.0]))
    direction[2] = np.tan(np.radians(tilt_deg)) * random.uniform(-1, 1)
    return unit(direction)


def moving_shape(random, d):
    if random.random() < 0.6:
        return dict(shape="box", size_m=[uniform(random, d["box_size_m"]) for _ in range(3)],
                    yaw_rad=float(random.uniform(-np.pi, np.pi)))
    return dict(shape="sphere", radius_m=uniform(random, d["sphere_radius_m"]))


def conflict_window(ctx, nominal):
    lo, hi = ctx.disturbance_cfg["conflict_window_s"]
    hi = min(hi, nominal["end"] - 0.3)
    if hi <= lo:
        raise Rejected("nominal_too_short")
    return lo, hi


def transient_crossing(ctx, nominal, random, distractor=False):
    family = "distractor" if distractor else "transient_crossing"
    d = ctx.disturbance_cfg[family]
    lo, hi = conflict_window(ctx, nominal)
    t_c = uniform(random, [lo, hi])
    link = link_index(ctx, CONFLICT_LINKS[int(random.integers(len(CONFLICT_LINKS)))])
    point, velocity = link_state(ctx, nominal["plan"], t_c, link)
    if distractor:
        # Pass beside or above the route, never through it.
        offset = np.r_[random.normal(size=2), random.uniform(0.3, 1.0)]
        point = point + unit(offset) * uniform(random, d["offset_m"])
    direction = crossing_direction(random, velocity, ctx.disturbance_cfg["transient_crossing"]["tilt_deg"])
    shape = moving_shape(random, dict(d, sphere_radius_m=ctx.disturbance_cfg["transient_crossing"]["sphere_radius_m"]))
    speed, half = uniform(random, d["speed_m_s"]), uniform(random, d["half_travel_m"])
    duration = 2 * half / speed
    t0 = t_c + (0.0 if distractor else uniform(random, d["pass_offset_s"])) - duration / 2
    motion = Motion([t0, t0 + duration], [point - half * direction, point + half * direction])
    info = dict(family=family, conflict_time_s=t_c, conflict_link=ctx.checker.names[link],
                conflict_point_m=point.tolist(), direction=direction.tolist(), mean_speed_m_s=speed,
                peak_speed_m_s=motion.peak_speed())
    return [DynamicObstacle("o0", shape, motion, family)], info


def persistent_blockage(ctx, nominal, random):
    """An object moves into the middle-to-late transfer route and stays there.

    It settles ``lead`` (1-2 s) before the robot would reach it, at a distal
    link's position, so the robot faces a visible, settled blocker; it may
    already be moving when the episode starts. It either stands on the
    table reaching above the route (a stack or cart) or is held in place (by a
    person or another machine); its rest pose must leave the goal pose free.
    """
    d, sep = ctx.disturbance_cfg["persistent_blockage"], ctx.disturbance_cfg
    lo, _ = conflict_window(ctx, nominal)
    travel, speed, lead = (uniform(random, d[k]) for k in ("travel_m", "speed_m_s", "lead_s"))
    transfer = nominal["arrivals"][1:3]
    lo = max(lo, transfer[0] + 0.3 * (transfer[1] - transfer[0]))
    hi = transfer[0] + 0.85 * (transfer[1] - transfer[0])
    if hi <= lo:
        raise Rejected("nominal_too_short")
    footprint = [uniform(random, d["footprint_m"]), uniform(random, d["footprint_m"])]
    yaw = float(random.uniform(-np.pi, np.pi))
    goal, start_boxes = nominal["waypoints"][-1], ctx.checker.poses(nominal["q0"])[:2]

    def options(point):
        height = point[2] + uniform(random, d["top_above_m"])
        standing = ("standing", dict(shape="box", size_m=footprint + [float(height)], yaw_rad=yaw),
                    np.array([point[0], point[1], height / 2]))
        held = ("held", dict(shape="box", size_m=footprint + [uniform(random, d["footprint_m"])], yaw_rad=yaw), point.copy())
        return [standing, held] if random.random() < 0.5 else [held, standing]

    chosen = None
    for _ in range(16):
        t_c = uniform(random, [lo, hi])
        link = link_index(ctx, CONFLICT_LINKS[int(random.integers(3, len(CONFLICT_LINKS)))])  # bracelet or gripper
        point, velocity = link_state(ctx, nominal["plan"], t_c, link)
        for support, shape, final in options(point):
            cover = DynamicObstacle("probe", shape, Motion([0.0], [final])).voxels(0.0)
            if (plausible(dict(shape, center_m=final.tolist()), ctx.layout["static_primitives"], ctx.fixed)
                    and ctx.checker.clearance(goal, cover, fixed=False, cutoff=1.0) >= sep["goal_separation_m"]):
                chosen = (t_c, link, point, velocity, support, shape, final, cover - final)
                break
        if chosen:
            break
    if chosen is None:
        raise Rejected("obstacle_rests_at_goal_pose")
    t_c, link, point, velocity, support, shape, final, local = chosen
    # Approach along the first direction whose path clears furniture and fixed equipment and starts
    # away from the robot, preferring the far side of the workspace; held objects may be lowered.
    outward = unit(np.array([final[0], final[1], 0.0]))
    candidates = [rz(k * float(random.choice([1, -1])) * np.pi / 6) @ outward for k in (0, 1, -1, 2, -2, 3, -3)]
    candidates += [np.array([0.0, 0.0, 1.0])] if support == "held" else []
    rotations, translations = start_boxes
    for direction in candidates:
        start = final + travel * direction
        path = [dict(shape, center_m=(final + s * travel * direction).tolist()) for s in np.linspace(0, 1, 16)]
        if (float(ctx.checker.sphere_distances(rotations, translations, local + start).min()) >= sep["initial_separation_m"]
                and all(plausible(p, ctx.layout["static_primitives"], ctx.fixed) for p in path)):
            break
    else:
        raise Rejected("no_clear_approach")
    arrival = t_c - lead
    motion = Motion([arrival - travel / speed, arrival], [start, final])
    info = dict(family="persistent_blockage", support=support, conflict_time_s=t_c,
                conflict_link=ctx.checker.names[link], conflict_point_m=point.tolist(), arrival_s=arrival,
                mean_speed_m_s=speed, peak_speed_m_s=motion.peak_speed())
    return [DynamicObstacle("o0", shape, motion, "persistent_blockage")], info


def entrance_blockage(ctx, nominal, random):
    d = ctx.disturbance_cfg["entrance_blockage"]
    opening = ctx.layout["opening"]
    R = rz(ctx.layout["yaw_rad"])
    inward, lateral = R[:, 0], R[:, 1]
    front = np.asarray(opening["centre_m"])
    times = np.arange(0.0, nominal["end"], 0.01)
    depth = np.asarray([(ctx.kin.tool(nominal["plan"].at(t)[0])[0] - front) @ inward for t in times])
    crossing = np.flatnonzero((depth[:-1] < 0) & (depth[1:] >= 0))
    if not len(crossing):
        raise Rejected("no_entry")
    entry = float(times[crossing[0] + 1])
    width, height = opening["width_m"] + 2 * d["overlap_m"], opening["height_m"] + 2 * d["overlap_m"]
    shape = dict(shape="box", size_m=[d["thickness_m"], width, height], yaw_rad=ctx.layout["yaw_rad"])
    centre = front - (d["thickness_m"] / 2 + d["standoff_m"]) * inward
    speed = uniform(random, d["speed_m_s"])
    t1 = entry - uniform(random, d["lead_s"])
    t2 = t1 + uniform(random, d["dwell_s"])
    # A guard door parked above the window drops to close it and rises to clear it; if it cannot
    # park there, a shutter slides in from one side and leaves to either side.
    lift = height + 0.04
    if plausible(dict(shape, center_m=(centre + [0, 0, lift]).tolist()), ctx.layout["static_primitives"], ctx.fixed):
        kind, rest, leave, slide = "vertical_door", centre + [0, 0, lift], centre + [0, 0, lift], lift / speed
    else:
        offset = opening["width_m"] / 2 + d["overlap_m"] + width / 2 + 0.04
        side, exit_side = float(random.choice([-1.0, 1.0])), float(random.choice([-1.0, 1.0]))
        kind, rest, leave = "sliding_shutter", centre + side * offset * lateral, centre + exit_side * offset * lateral
        slide = offset / speed
    motion = Motion([t1 - slide, t1, t2, t2 + slide], [rest, centre, centre, leave])
    info = dict(family="entrance_blockage", kind=kind, entry_time_s=entry, blocked_s=[t1, t2], mean_speed_m_s=speed,
                peak_speed_m_s=motion.peak_speed())
    return [DynamicObstacle("o0", shape, motion, "entrance_blockage")], info


DISTURBANCES = dict(transient_crossing=transient_crossing, persistent_blockage=persistent_blockage,
                    entrance_blockage=entrance_blockage,
                    distractor=lambda ctx, nominal, random: transient_crossing(ctx, nominal, random, True))


def dynamic_clearances(ctx, world, plan, times):
    return np.asarray([ctx.checker.clearance(plan.at(t)[0], world.dynamic_xyz(t), fixed=False, cutoff=1.0)
                       for t in times])


def accept_obstacles(ctx, nominal, obstacles, family):
    """Method-independent validity rules for one disturbance proposal."""
    d = ctx.disturbance_cfg
    world = World(ctx.layout["static_primitives"], obstacles)
    probe = d["probe_period_s"]
    # Observable onset: departure time, or t = 0 for an obstacle already moving when the episode starts.
    onset = max(0.0, min(o.motion.moving_interval()[0] for o in obstacles))
    settle = max(o.motion.moving_interval()[1] for o in obstacles)
    for obstacle in obstacles:
        for t in np.arange(0.0, settle + probe, 0.05):
            primitive = dict(obstacle.primitive, center_m=obstacle.motion.at(t)[0].tolist())
            if not plausible(primitive, ctx.layout["static_primitives"], ctx.fixed):
                raise Rejected("obstacle_intersects_scene")
    horizon = max(nominal["end"], settle) + 1.0
    times = np.arange(0.0, horizon, probe)
    blind = dynamic_clearances(ctx, world, nominal["plan"], times)
    # Nothing appears next to the robot: at t = 0 every obstacle is at least the initial separation away.
    if blind[0] < d["initial_separation_m"]:
        raise Rejected("obstacle_starts_near_robot")
    # A completed task stays completed: the final obstacle poses leave the goal pose free.
    goal = nominal["waypoints"][-1]
    if ctx.checker.clearance(goal, world.dynamic_xyz(settle), fixed=False, cutoff=1.0) < d["goal_separation_m"]:
        raise Rejected("obstacle_rests_at_goal_pose")
    near = np.flatnonzero(blind < 0.05)
    if family == "distractor":
        if np.min(blind) < ctx.disturbance_cfg["distractor"]["separation_m"]:
            raise Rejected("distractor_conflicts")
        return world, dict(blind_minimum_clearance_m=float(np.min(blind)), onset_s=onset, settle_s=settle)
    if np.min(blind) >= 0.0:
        raise Rejected("nominal_not_disturbed")
    if times[near[0]] < onset + d["warning_min_s"]:
        raise Rejected("insufficient_warning")
    return world, dict(blind_minimum_clearance_m=float(np.min(blind)), first_conflict_s=float(times[near[0]]),
                       first_collision_s=float(times[np.flatnonzero(blind < 0)[0]]), onset_s=onset, settle_s=settle)


# ----------------------------------------------------------------------------- witness

class Screen:
    """Kinematic screen of a reference: supervisor rule while moving, clearance while at rest."""

    def __init__(self, ctx, world):
        self.ctx, self.world = ctx, world
        self.sup = ctx.cfg["supervisor"]
        self.w = ctx.witness_cfg

    def dynamic(self, plan, t0, t1):
        """First time in [t0, t1] that violates the rule, else None (details in ``last_failure``)."""
        checker, world = self.ctx.checker, self.world
        self.last_failure = None
        for t in np.arange(t0, t1 + 1e-9, self.w["check_period_s"]):
            q, v, _ = plan.at(t)
            xyz = world.dynamic_xyz(t)
            if np.max(np.abs(v)) > self.sup["moving_threshold_rad_s"]:
                proximity = checker.proximity(q, v, xyz)
                velocity = world.obstacles[int(world.owner[proximity["sphere"]])].motion.at(t)[1]
                required = protective_separation(proximity, velocity, v, self.sup, self.ctx.limits["acceleration"])[0]
                if proximity["distance"] < required + self.w["margin_m"]:
                    self.last_failure = dict(time_s=float(t), moving=True, distance_m=proximity["distance"],
                                             required_m=required)
                    return float(t)
            else:
                distance = checker.clearance(q, xyz, fixed=False, cutoff=1.0)
                if distance < self.w["rest_clearance_m"]:
                    self.last_failure = dict(time_s=float(t), moving=False, distance_m=distance)
                    return float(t)
        return None


def via_candidates(ctx, q_stop, target, random):
    p_s, approach = ctx.kin.tool(q_stop)
    p_t, _ = ctx.kin.tool(target)
    middle = (p_s + p_t) / 2
    try:
        lateral = unit(np.cross([0.0, 0.0, 1.0], p_t - p_s))
    except Rejected:
        lateral = np.array([0.0, 1.0, 0.0])
    positions = [middle + [0, 0, dz] for dz in (0.15, 0.25, 0.35)]
    positions += [middle + s * lateral + [0, 0, 0.08] for s in (0.2, -0.2, 0.3, -0.3)]
    vias = []
    margin = ctx.witness_cfg["static_margin_m"]
    for position in positions:
        q = ctx.kin.best(position, approach, q_stop, random, lambda q: ctx.static_ok(q, margin), angle_deg=35.0)
        if q is not None:
            vias.append(q)
    centre = (q_stop + target) / 2
    for _ in range(ctx.witness_cfg["random_vias"]):
        q = np.clip(centre + random.normal(0, ctx.witness_cfg["via_sigma_rad"], 7), ctx.model.lower, ctx.model.upper)
        if ctx.static_ok(q, margin):
            vias.append(q)
    return vias


def witness_candidates(ctx, nominal, world, random):
    """Yield privileged completion candidates in a fixed order (generator only)."""
    w, dwell = ctx.witness_cfg, ctx.cfg["protocol"]["goal_dwell_s"]
    screen = Screen(ctx, world)
    plan, waypoints, arrivals = nominal["plan"], nominal["waypoints"], nominal["arrivals"]
    nominal_failure = screen.dynamic(plan, 0.0, nominal["end"] + dwell + 0.3)
    if nominal_failure is None:
        yield dict(strategy="continue", submission=None, completion_estimate_s=nominal["end"])
        return
    onset = max(0.0, min(o.motion.moving_interval()[0] for o in world.obstacles))
    vlim, alim = ctx.limits["velocity"], ctx.limits["acceleration"]
    margin = w["static_margin_m"]
    latest = w["latest_completion_s"]
    settled = max(o.motion.moving_interval()[1] for o in world.obstacles)
    resting = world.dynamic_xyz(settled)

    def staged(stop, q_stop, t_stop, stages):
        """Brake, then rest-to-rest motions at the given start times with holds in between."""
        pieces, t, q = [stop], t_stop, q_stop
        for start, motion in stages:
            if start - t > 1e-6:
                pieces.append(hold(q, start - t))
            pieces.append(motion)
            t = start + float(np.sum(motion["durations"]))
            q = np.asarray(motion["controls"])[-1, -1]
        return concatenate(pieces)

    for attempt, delay in enumerate(w["brake_delays_s"]):
        t_b = onset + delay
        if t_b >= min(nominal["end"] - 0.05, nominal_failure):
            continue
        q_b, v_b, _ = plan.at(t_b)
        stop = braking(q_b, v_b, alim)
        t_stop = t_b + float(np.sum(stop["durations"]))
        if screen.dynamic(plan.spliced(Segment(t_b, stop)), t_b, t_stop) is not None:
            continue
        q_stop = np.asarray(stop["controls"])[-1, -1]
        index = int(np.searchsorted(arrivals, t_b, side="right"))

        def scan(name, prefix, forward):
            """Run ``prefix`` from the stop, wait, then start ``forward`` at the earliest safe time."""
            base = plan.spliced(Segment(t_b, staged(stop, q_stop, t_stop, prefix)))
            ready = t_stop + sum(float(np.sum(m["durations"])) for _, m in prefix)
            failure = screen.dynamic(base, t_stop, latest)
            if failure is not None and failure < ready:
                return None
            duration = float(np.sum(forward["durations"]))
            last_go = min(latest if failure is None else failure, latest - duration)
            for t_go in np.arange(ready, last_go + 1e-9, w["resume_step_s"]):
                submission = staged(stop, q_stop, t_stop, prefix + [(float(t_go), forward)])
                candidate = plan.spliced(Segment(t_b, submission))
                if screen.dynamic(candidate, t_go, t_go + duration + dwell + 0.3) is None:
                    return dict(strategy=name, brake_s=float(t_b), resume_s=float(t_go),
                                completion_estimate_s=float(t_go + duration),
                                submission=dict(start_time_s=float(t_b), trajectory=serializable(submission)))
                if t_go >= settled:
                    return None  # every obstacle is at rest: waiting longer changes nothing
            return None

        def clear_of_rest_poses(route):
            """Route against the obstacles' final poses (prunes detours through a settled blocker)."""
            for a, b in zip(route[:-1], route[1:]):
                count = max(2, int(np.ceil(np.linalg.norm(b - a) / 0.01)) + 1)
                if np.min(ctx.checker.path_clearance(np.linspace(a, b, count), resting, fixed=False,
                                                     cutoff=0.1)) < w["margin_m"]:
                    return False
            return True

        found = scan("brake_wait_resume", [], point_to_point([q_stop] + list(waypoints[index:]), vlim, alim))
        if found is None and np.max(np.abs(waypoints[index - 1] - q_stop)) > 1e-6:
            back = point_to_point([q_stop, waypoints[index - 1]], vlim, alim)
            found = scan("brake_retreat_resume", [(t_stop, back)],
                         point_to_point(list(waypoints[index - 1:]), vlim, alim))
        if found is None:
            for via in via_candidates(ctx, q_stop, waypoints[-2], random):
                for m in (len(waypoints) - 2, len(waypoints) - 1):
                    route = [q_stop, via] + list(waypoints[m:])
                    if not (clear_of_rest_poses(route) and ctx.segment_ok(q_stop, via, margin)
                            and ctx.segment_ok(via, waypoints[m], margin)):
                        continue
                    found = scan("brake_detour", [], point_to_point([q_stop, via] + list(waypoints[m:]), vlim, alim))
                    if found is not None:
                        break
                if found is not None:
                    break
        if found is None and attempt < w["planned_detour_attempts"]:
            # Last resort: the existing OMPL RRTConnect against the scene and the obstacles' rest poses.
            import tempfile
            with tempfile.TemporaryDirectory() as workdir:
                path = ompl_path(ctx.checker, q_stop, waypoints[-1], ctx.world.static_xyz, resting,
                                 w["static_margin_m"], w["planned_detour_margin_m"], w["planning_s"],
                                 int(random.integers(1, 2 ** 31)), workdir)
            if path is not None:
                found = scan("brake_planned_detour", [], point_to_point(path, vlim, alim))
        if found is not None:
            yield found


class ScheduledSubmission:
    """Privileged witness replay: submits the witness once at the first observation."""

    def __init__(self, submission):
        self.submission, self.sent = submission, False

    def setup(self, problem):
        return None

    def step(self, observation):
        if self.sent or self.submission is None:
            return None
        self.sent = True
        return self.submission


# ----------------------------------------------------------------------------- tasks and splits

def public_robot(model):
    return dict(joint_names=model.joint_names, lower=model.lower.tolist(), upper=model.upper.tolist(),
                robot_sha256=model.robot_sha256, collision_geometry_sha256=model.collision_geometry_sha256)


def assemble(split, layout_id, layout, family, nominal, obstacles, disturbance, ctx):
    return dict(
        task_id=f"{layout_id}-{family}", split=split, layout_id=layout_id, workspace=layout["workspace"],
        family=family, benchmark=BENCHMARK,
        public=dict(robot=public_robot(ctx.model), scene=dict(static_primitives=layout["static_primitives"]),
                    goal=dict(q=nominal["waypoints"][-1].tolist(), site=nominal["goal_name"],
                              tool_position_m=nominal["sites"][-1][1], approach=nominal["sites"][-1][2]),
                    nominal=dict(waypoints=nominal["waypoints"].tolist(), arrival_times_s=nominal["arrivals"].tolist(),
                                 sites=nominal["sites"]),
                    initial_plan=[dict(start_time_s=-nominal["offset"], trajectory=serializable(nominal["candidate"]))],
                    initial_state=dict(q=nominal["q0"].tolist(), v=nominal["v0"].tolist())),
        oracle=dict(obstacles=[o.to_dict() for o in obstacles], disturbance=disturbance))


def generate_task(split, layout_id, layout, family, ctx, seed, validation_dir):
    import shutil
    random = make_rng(domain_seed(seed, f"{BENCHMARK}/task", layout_id, family))
    attempts = ctx.cfg["generator"]["attempts_per_task"]
    reasons = {}
    evidence = Path(validation_dir) / f"{layout_id}-{family}"
    if evidence.exists():
        shutil.rmtree(evidence)  # regenerated deterministically below
    for attempt in range(attempts):
        try:
            nominal = nominal_motion(ctx, random)
            obstacles, info = DISTURBANCES[family](ctx, nominal, random)
            world, checks = accept_obstacles(ctx, nominal, obstacles, family)
            info.update(checks)
            task = assemble(split, layout_id, layout, family, nominal, obstacles, info, ctx)
            for k, candidate in enumerate(witness_candidates(ctx, nominal, world, random)):
                if k >= ctx.witness_cfg["closed_loop_candidates"]:
                    break
                if family == "distractor" and candidate["strategy"] != "continue":
                    raise Rejected("distractor_requires_reaction")  # a control must not need any reaction
                task["oracle"]["witness"] = candidate
                directory = Path(validation_dir) / task["task_id"] / f"attempt{attempt:02d}_witness{k}"
                result = run_episode(task, InProcessPlanner(ScheduledSubmission(candidate["submission"])),
                                     directory, ctx.cfg, method=dict(name="witness", privileged=True))
                if result["status"] == "success":
                    task["oracle"]["witness_validation"] = dict(
                        status=result["status"], completion_time_s=result["completion_time_s"],
                        minimum_clearance_m=result["clearance"]["minimum_m"],
                        minimum_dynamic_clearance_m=result["clearance"]["minimum_dynamic_m"],
                        checker_disagreement_steps=result["collision"]["checker_disagreement_steps"],
                        interventions=result["interventions"]["count"], result_sha256=digest(result),
                        evidence=str(directory.relative_to(ROOT)) if directory.is_relative_to(ROOT) else str(directory))
                    task["generation"] = dict(status="accepted", attempts=attempt + 1, rejections=reasons)
                    return task
                reasons["witness_closed_loop_" + result["status"]] = reasons.get("witness_closed_loop_" + result["status"], 0) + 1
            raise Rejected("no_witness")
        except Rejected as exc:
            reasons[str(exc)] = reasons.get(str(exc), 0) + 1
    return dict(task_id=f"{layout_id}-{family}", split=split, layout_id=layout_id, workspace=layout["workspace"],
                family=family, benchmark=BENCHMARK, generation=dict(status="generation_failed", attempts=attempts,
                                                                    rejections=reasons))


def reachable(ctx, random, probes=6):
    """A layout is usable if some sampled pick-and-place nominal motion exists (geometry only)."""
    for _ in range(probes):
        try:
            nominal_motion(ctx, random)
            return True
        except Rejected:
            continue
    return False


def generate_layout(job):
    """One layout and all its tasks (independent process; deterministic)."""
    split, workspace, index, seed, validation_dir, families = job
    cfg = load_config()
    layout_id = f"{split}-{workspace}-{index:03d}"
    started = time.perf_counter()
    try:
        model = Model()
        random = make_rng(domain_seed(seed, f"{BENCHMARK}/{split}/{workspace}/layout", index))
        fixed = fixed_primitives(model)
        for proposal in range(cfg["generator"]["layout_proposals"]):
            layout = make_layout(workspace, random, cfg, fixed)
            ctx = Context(layout, cfg, model)
            if reachable(ctx, make_rng(domain_seed(seed, f"{BENCHMARK}/reachability", layout_id, proposal))):
                break
        else:
            raise Rejected("layout_unreachable")
        layout["reachability_proposals"] = proposal + 1
        tasks = [generate_task(split, layout_id, layout, family, ctx, seed, validation_dir) for family in families]
        status = "valid"
    except Rejected as exc:
        layout, tasks, status = None, [], str(exc)
    except Exception:
        layout, tasks, status = None, [], "infrastructure_error: " + traceback.format_exc()
    record = dict(layout_id=layout_id, split=split, workspace=workspace, index=index, status=status,
                  layout_sha256=digest(clean(layout["static_primitives"])) if layout else None,
                  wall_seconds=time.perf_counter() - started)
    return record, [clean(t) for t in tasks]


def generate(splits, output=DATA, validation_dir=None, workers=1, families=None):
    """Generate the requested splits and write ``{split}.jsonl`` plus a manifest entry."""
    import multiprocessing
    cfg = load_config()
    g = cfg["generator"]
    validation_dir = Path(validation_dir or ROOT / "runs/dynamic_recovery/generation").resolve()
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else dict(splits={})
    for split in splits:
        jobs = [(split, workspace, i, g["seed"], str(validation_dir / split), (families or g["families"])[workspace])
                for workspace in WORKSPACES for i in range(g["splits"][split])]
        if workers > 1:
            with multiprocessing.get_context("spawn").Pool(workers, maxtasksperchild=1) as pool:  # fresh OMPL RNG per layout
                results = pool.map(generate_layout, jobs, chunksize=1)
        else:
            results = [generate_layout(job) for job in jobs]
        layouts = [r for r, _ in results]
        tasks = sorted((t for _, items in results for t in items), key=lambda t: t["task_id"])
        path = output / f"{split}.jsonl"
        path.write_text("".join(json.dumps(t, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n" for t in tasks))
        accepted = [t for t in tasks if t["generation"]["status"] == "accepted"]
        manifest["splits"][split] = dict(
            file=path.name, sha256=_sha(path), registered_tasks=len(tasks), accepted_tasks=len(accepted),
            generation_failures=len(tasks) - len(accepted), layouts=layouts,
            families={f: sum(t["family"] == f for t in accepted) for f in sorted({t["family"] for t in tasks})},
            workspaces={w: sum(t["workspace"] == w for t in accepted) for w in WORKSPACES})
    manifest.update(benchmark=BENCHMARK, config_sha256=_sha(HERE / "config.json"), source_hashes=source_hashes(),
                    selection_policy="Layouts and tasks are accepted by geometric rules, the nominal motion and a privileged witness only; no evaluated method is consulted.")
    write_json(manifest_path, manifest)
    return manifest


def _sha(path):
    import hashlib
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_tasks(split, path=DATA, accepted_only=True):
    tasks = [json.loads(line) for line in (Path(path) / f"{split}.jsonl").read_text().splitlines() if line.strip()]
    return [t for t in tasks if not accepted_only or t["generation"]["status"] == "accepted"]
