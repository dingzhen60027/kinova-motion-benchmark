"""World geometry: static workspaces, moving obstacles and their collision models.

Every non-robot obstacle uses the frozen ``generic_motion_v1`` representation,
a conservative cover of 0.04 m spheres around each source box or sphere
(``generator.voxelize``). The canonical Drake checker (``Model.clearance``),
CSDecomp collision checks on a GPU machine and the MuJoCo contact model built
here therefore consume the same sphere centres. Source primitives are kept for
display and for planners that prefer exact shapes.

A moving obstacle is a rigid cover translated along a rest-to-rest
minimum-jerk path through timed waypoints. Its position is C2 in time, it never
reacts to the robot, and the observation interface exposes only its current
pose and velocity.
"""
from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from benchmarks.generic_motion_v1.generator.voxelize import voxelize
from benchmarks.generic_motion_v1.model import Model, sha256

VOXEL_RADIUS_M = 0.04


def quantize(xyz):
    """Float64 copies of the float32 sphere centres consumed by CSDecomp."""
    return np.asarray(xyz, dtype=float).reshape(-1, 3).astype(np.float32).astype(float)


def cover(primitives, prefix="s"):
    """Conservative sphere cover of world-frame boxes/spheres (frozen v1 voxelizer)."""
    objects = [dict(p, obstacle_id=p.get("obstacle_id", f"{prefix}{i:03d}"))
               for i, p in enumerate(primitives)]
    if not objects:
        return np.empty((0, 3)), dict(voxel_count=0, obstacles=[])
    xyz, report = voxelize(objects, VOXEL_RADIUS_M, VOXEL_RADIUS_M)
    return quantize(xyz), report


class Motion:
    """Rest-to-rest minimum-jerk translation through timed waypoints.

    Each interval uses p0 + (p1 - p0)(10s^3 - 15s^4 + 6s^5), so velocity and
    acceleration vanish at every waypoint and the whole path is C2. The
    obstacle rests at the first waypoint before it and at the last one after it.
    """

    def __init__(self, times, points):
        times = np.asarray(times, dtype=float)
        points = np.asarray(points, dtype=float)
        if (times.ndim != 1 or len(times) < 1 or points.shape != (len(times), 3)
                or not np.isfinite(times).all() or not np.isfinite(points).all()
                or np.any(np.diff(times) <= 0)):
            raise ValueError("Motion needs strictly increasing finite times and matching 3D points")
        self.times, self.points = times, points

    def at(self, t):
        t = float(t)
        times, points = self.times, self.points
        if len(times) == 1 or t <= times[0]:
            return points[0].copy(), np.zeros(3), np.zeros(3)
        if t >= times[-1]:
            return points[-1].copy(), np.zeros(3), np.zeros(3)
        i = int(np.searchsorted(times, t, side="right")) - 1
        span = times[i + 1] - times[i]
        s = (t - times[i]) / span
        delta = points[i + 1] - points[i]
        position = points[i] + delta * (s ** 3 * (10 - 15 * s + 6 * s * s))
        velocity = delta / span * (30 * s * s * (1 - s) ** 2)
        acceleration = delta / span ** 2 * (60 * s - 180 * s * s + 120 * s ** 3)
        return position, velocity, acceleration

    def moving_interval(self):
        """First departure and last arrival time; ``None`` for a static obstacle."""
        moving = np.flatnonzero(np.any(np.diff(self.points, axis=0) != 0, axis=1))
        if not len(moving):
            return None
        return float(self.times[moving[0]]), float(self.times[moving[-1] + 1])

    def peak_speed(self):
        if len(self.times) < 2:
            return 0.0
        return float(np.max(1.875 * np.linalg.norm(np.diff(self.points, axis=0), axis=1) / np.diff(self.times)))

    def to_dict(self):
        return dict(kind="rest_to_rest_minimum_jerk", times_s=self.times.tolist(),
                    points_m=self.points.tolist())

    @classmethod
    def from_dict(cls, value):
        if value.get("kind") != "rest_to_rest_minimum_jerk":
            raise ValueError("Unsupported obstacle motion")
        return cls(value["times_s"], value["points_m"])


class DynamicObstacle:
    """A rigid sphere cover whose reference point follows a ``Motion``."""

    def __init__(self, obstacle_id, primitive, motion, role="disturbance"):
        primitive = dict(primitive)
        if primitive.get("shape") not in ("box", "sphere"):
            raise ValueError("Moving obstacles are boxes or spheres")
        primitive["center_m"] = [0.0, 0.0, 0.0]
        self.id, self.primitive, self.motion, self.role = str(obstacle_id), primitive, motion, role
        self.local_xyz, self.cover_report = cover([dict(primitive, obstacle_id=self.id)])
        if not len(self.local_xyz):
            raise ValueError("Moving obstacle cover is empty")

    def voxels(self, t):
        return quantize(self.local_xyz + self.motion.at(t)[0])

    def state(self, t):
        position, velocity, _ = self.motion.at(t)
        return dict(id=self.id, primitive=dict(self.primitive, center_m=position.tolist()),
                    position_m=position.tolist(), velocity_m_s=velocity.tolist())

    def to_dict(self):
        return dict(id=self.id, role=self.role, primitive=self.primitive, motion=self.motion.to_dict())

    @classmethod
    def from_dict(cls, value):
        return cls(value["id"], value["primitive"], Motion.from_dict(value["motion"]), value.get("role", "disturbance"))


class World:
    """Static workspace plus independently moving obstacles."""

    def __init__(self, static_primitives, obstacles=()):
        self.static_primitives = [dict(p) for p in static_primitives]
        self.static_xyz, self.static_cover = cover(self.static_primitives, "s")
        self.obstacles = list(obstacles)
        counts = [len(o.local_xyz) for o in self.obstacles]
        self.owner = np.repeat(np.arange(len(counts)), counts).astype(int)
        self.offsets = np.r_[0, np.cumsum(counts)].astype(int)

    @classmethod
    def from_task(cls, task, with_obstacles=True):
        obstacles = [DynamicObstacle.from_dict(o) for o in task["oracle"]["obstacles"]] if with_obstacles else []
        return cls(task["public"]["scene"]["static_primitives"], obstacles)

    def dynamic_xyz(self, t):
        if not self.obstacles:
            return np.empty((0, 3))
        return np.vstack([o.voxels(t) for o in self.obstacles])

    def obstacle_states(self, t):
        return [o.state(t) for o in self.obstacles]


class GeometryChecker:
    """Vectorized signed distances from the moving robot boxes to sphere sets.

    Robot box poses come from the canonical Drake plant, and fixed-equipment
    and self-collision pairs use the canonical Drake query. The box-to-sphere
    expression equals ``Model._voxel_clearance``; tests require agreement with
    ``Model.clearance``. Used by the harness supervisor, the task generator and
    reference policies; final acceptance uses ``Model.clearance`` itself.
    """

    def __init__(self, model=None):
        from pydrake.multibody.tree import JacobianWrtVariable
        self.model = model if model is not None else Model()
        self.model.set_voxels()
        geometries = self.model._robot_geometries
        if any(kind != "box" for _, kind, _ in geometries):
            raise ValueError("Canonical moving robot geometry is expected to be boxes")
        self.geometry_ids = [g for g, _, _ in geometries]
        self.half = np.asarray([np.asarray(d, float) / 2 for _, _, d in geometries])
        plant, inspector = self.model.plant, self.model.inspector
        self.bodies = [plant.GetBodyFromFrameId(inspector.GetFrameId(g)) for g in self.geometry_ids]
        self.names = [inspector.GetName(g) for g in self.geometry_ids]
        self._wrt = JacobianWrtVariable.kV
        self.radius = VOXEL_RADIUS_M

    @property
    def lower(self):
        return self.model.lower

    @property
    def upper(self):
        return self.model.upper

    def poses(self, q):
        model = self.model
        model.plant.SetPositions(model.plant_context, np.asarray(q, dtype=float))
        query = model._query()
        transforms = [query.GetPoseInWorld(g) for g in self.geometry_ids]
        rotations = np.stack([X.rotation().matrix() for X in transforms])
        translations = np.stack([X.translation() for X in transforms])
        return rotations, translations, query

    def sphere_distances(self, rotations, translations, xyz):
        """(G, N) signed distances between robot boxes and spheres of the frozen radius."""
        xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
        if not len(xyz):
            return np.full((len(rotations), 0), np.inf)
        local = np.matmul(xyz[None, :, :] - translations[:, None, :], rotations)
        d = np.abs(local) - self.half[:, None, :]
        outside = np.sqrt(np.square(np.maximum(d, 0.0)).sum(axis=2))
        inside = np.minimum(d.max(axis=2), 0.0)
        return outside + inside - self.radius

    def nearby(self, rotations, translations, xyz, cutoff):
        """Spheres that can lie within ``cutoff`` of any robot box (AABB prefilter)."""
        xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
        if not len(xyz) or not np.isfinite(cutoff):
            return xyz
        extent = np.abs(rotations) @ self.half[:, :, None]
        low = (translations - extent[:, :, 0]).min(axis=0) - cutoff - self.radius
        high = (translations + extent[:, :, 0]).max(axis=0) + cutoff + self.radius
        return xyz[np.all((xyz >= low) & (xyz <= high), axis=1)]

    @staticmethod
    def fixed_clearance(query):
        """Canonical fixed-equipment and self-collision pairs (Drake signed distance)."""
        return min((r.distance for r in query.ComputeSignedDistancePairwiseClosestPoints()), default=np.inf)

    def clearance(self, q, xyz=(), fixed=True, cutoff=np.inf):
        """Minimum signed distance; values at or above ``cutoff`` are reported as ``cutoff``."""
        rotations, translations, query = self.poses(q)
        spheres = self.sphere_distances(rotations, translations, self.nearby(rotations, translations, xyz, cutoff))
        value = min(float(spheres.min()) if spheres.size else np.inf, cutoff)
        return min(value, self.fixed_clearance(query)) if fixed else value

    def path_clearance(self, configurations, xyz=(), fixed=True, cutoff=np.inf):
        return np.asarray([self.clearance(q, xyz, fixed, cutoff)
                           for q in np.asarray(configurations, float).reshape(-1, 7)])

    def proximity(self, q, v, xyz):
        """Closest robot-box/sphere pair, its outward normal and the robot point velocity."""
        rotations, translations, _ = self.poses(q)
        distances = self.sphere_distances(rotations, translations, xyz)
        g, n = np.unravel_index(int(np.argmin(distances)), distances.shape)
        centre = np.asarray(xyz, float).reshape(-1, 3)[n]
        local = rotations[g].T @ (centre - translations[g])
        point = rotations[g] @ np.clip(local, -self.half[g], self.half[g]) + translations[g]
        direction = centre - point
        if np.linalg.norm(direction) < 1e-12:
            direction = centre - translations[g]
        normal = direction / max(np.linalg.norm(direction), 1e-12)
        plant, context = self.model.plant, self.model.plant_context
        body = self.bodies[g]
        X_WB = plant.EvalBodyPoseInWorld(context, body)
        p_B = X_WB.inverse().multiply(point)
        jacobian = plant.CalcJacobianTranslationalVelocity(
            context, self._wrt, body.body_frame(), p_B, plant.world_frame(), plant.world_frame())
        return dict(distance=float(distances[g, n]), geometry=int(g), sphere=int(n), point=point,
                    normal=normal, point_velocity=jacobian @ np.asarray(v, float))


class CollisionModel:
    """The fast checker behind the ``Model.cuda`` interface that the existing OMPL adapter calls.

    Static geometry keeps a small margin (goal poses sit close to furniture by
    design); the separately supplied obstacle spheres keep a larger margin.
    """

    def __init__(self, checker, static_xyz, moving_xyz, static_margin, moving_margin):
        model = checker.model
        self.checker, self.static_xyz, self.moving_xyz = checker, static_xyz, moving_xyz
        self.static_margin, self.moving_margin = static_margin, moving_margin
        self.lower, self.upper = model.lower, model.upper
        self.robot_sha256, self.collision_geometry_sha256 = model.robot_sha256, model.collision_geometry_sha256

    def cuda(self, q):
        check = self.checker.clearance
        return np.asarray([check(x, self.static_xyz, cutoff=self.static_margin + 0.01) >= self.static_margin
                           and check(x, self.moving_xyz, fixed=False, cutoff=self.moving_margin + 0.01) >= self.moving_margin
                           for x in np.asarray(q, float).reshape(-1, 7)])


def ompl_path(checker, start, goal, static_xyz, moving_xyz, static_margin, moving_margin, seconds, seed, workdir):
    """OMPL RRTConnect (existing ``generic_motion_v1`` adapter) on the fast checker; None if no path."""
    from benchmarks.generic_motion_v1.roadmap import load_config
    from benchmarks.generic_motion_v1.runners import ompl_baselines
    import time
    cfg = load_config()
    cfg["ompl"] = dict(planning_seconds=float(seconds), validation_reserve_seconds=0.1,
                       simplification_seconds=0.3, motion_step_rad=0.01)
    model = CollisionModel(checker, static_xyz, moving_xyz, static_margin, moving_margin)
    case = dict(q_start=[float(x) for x in start], q_goal=[float(x) for x in goal], _objective="minimum_distance")
    try:
        result = ompl_baselines.solve(model, case, cfg, Path(workdir), time.perf_counter() + seconds + 1.0,
                                      int(seed), "RRTCONNECT")
    except Exception:
        return None
    return [np.asarray(p, float) for p in result["points"]]


def build_simulation(world, output_dir, config, identity):
    """Extend the canonical MuJoCo export with workspace and moving-obstacle covers.

    The canonical exporter (inertias, joints, torque motors, robot/fixed boxes
    and the exact canonical pair list) is reused unchanged. Static cover
    spheres become world geoms and each moving obstacle becomes a mocap body
    carrying its cover. Moving robot boxes collide with every cover sphere
    through contype/conaffinity bits; covers never collide with each other or
    with fixed equipment. Source primitives are display-only geoms.
    """
    import mujoco
    from benchmarks.generic_motion_v1.simulation.mujoco_model import build

    out = Path(output_dir).resolve()
    canonical = build(dict(voxels_xyz=[], voxel_radius_m=VOXEL_RADIUS_M, obstacle_geometry=[], **identity),
                      out, config)
    tree = ET.parse(canonical["xml_path"])
    root = tree.getroot()
    names = canonical["canonical_geom_names"]
    robot_names = {names[i] for i in canonical["robot_canonical_geom_indices"]}
    for geom in root.iter("geom"):
        if geom.get("name") in robot_names:
            geom.set("contype", "1")
            geom.set("conaffinity", "0")
    worldbody = root.find("worldbody")

    def text(values):
        return " ".join(format(float(x), ".17g") for x in np.asarray(values, float).reshape(-1))

    def visual(parent, name, primitive, rgba, centre):
        args = dict(name=name, group="2", rgba=rgba, contype="0", conaffinity="0", pos=text(centre))
        if primitive["shape"] == "box":
            yaw = float(primitive.get("yaw_rad", 0.0))
            args.update(type="box", size=text(np.asarray(primitive["size_m"]) / 2),
                        quat=text([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]))
        else:
            args.update(type="sphere", size=text([primitive["radius_m"]]))
        ET.SubElement(parent, "geom", **args)

    static_names = []
    for i, primitive in enumerate(world.static_primitives):
        visual(worldbody, f"static_visual_{i:03d}", primitive, "0.62 0.58 0.52 1", primitive["center_m"])
    for i, centre in enumerate(world.static_xyz):
        name = f"static_cover_{i:05d}"
        ET.SubElement(worldbody, "geom", name=name, type="sphere", size=text([VOXEL_RADIUS_M]),
                      pos=text(centre), contype="0", conaffinity="1", group="4", rgba="0.5 0.5 0.5 .12")
        static_names.append(name)
    obstacle_bodies, obstacle_geoms = [], []
    for k, obstacle in enumerate(world.obstacles):
        body_name = f"obstacle_{k:02d}"
        body = ET.SubElement(worldbody, "body", name=body_name, mocap="true",
                             pos=text(obstacle.motion.at(0.0)[0]))
        visual(body, f"obstacle_{k:02d}_visual", obstacle.primitive, "0.91 0.37 0.11 1", [0, 0, 0])
        geoms = []
        for i, centre in enumerate(obstacle.local_xyz):
            name = f"obstacle_{k:02d}_cover_{i:04d}"
            ET.SubElement(body, "geom", name=name, type="sphere", size=text([VOXEL_RADIUS_M]),
                          pos=text(centre), contype="0", conaffinity="1", group="4", rgba="0.91 0.37 0.11 .15")
            geoms.append(name)
        obstacle_bodies.append(body_name)
        obstacle_geoms.append(geoms)
    ET.indent(root, space="  ")
    xml_path = out / "world.xml"
    tree.write(xml_path, encoding="unicode", xml_declaration=True)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    ids = lambda kind, items: [int(mujoco.mj_name2id(model, kind, n)) for n in items]
    robot_ids = ids(mujoco.mjtObj.mjOBJ_GEOM, sorted(robot_names))
    static_ids = ids(mujoco.mjtObj.mjOBJ_GEOM, static_names)
    obstacle_ids = [ids(mujoco.mjtObj.mjOBJ_GEOM, g) for g in obstacle_geoms]
    cover_ids = set(static_ids).union(*map(set, obstacle_ids)) if obstacle_ids else set(static_ids)
    if model.nq != 7 or model.nv != 7 or model.nu != 7 or model.nmocap != len(world.obstacles):
        raise ValueError("Dynamic MuJoCo world must keep seven torque-driven joints and one mocap body per obstacle")
    for geom in range(model.ngeom):
        expected = (1, 0) if geom in robot_ids else (0, 1) if geom in cover_ids else (0, 0)
        if (int(model.geom_contype[geom]), int(model.geom_conaffinity[geom])) != expected:
            raise ValueError("Unexpected MuJoCo collision mask for geom " + str(geom))
    if model.npair != canonical["collision_pair_count"]:
        raise ValueError("Canonical explicit collision pairs changed")
    mocap = [int(model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b)]) for b in obstacle_bodies]
    metadata = dict(canonical, schema="kinova_dynamic_recovery_mujoco_v1", xml_path=str(xml_path),
                    xml_sha256=sha256(xml_path), canonical_xml_path=canonical["xml_path"],
                    canonical_xml_sha256=canonical["xml_sha256"], robot_geom_ids=robot_ids,
                    static_cover_geom_ids=static_ids, obstacle_mocap_ids=mocap,
                    obstacle_cover_geom_ids=obstacle_ids, cover_radius_m=VOXEL_RADIUS_M,
                    dynamic_contact_policy="Moving robot boxes (contype 1) against every static and moving cover sphere (conaffinity 1); canonical explicit pairs unchanged.")
    (out / "world_metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
    return metadata
