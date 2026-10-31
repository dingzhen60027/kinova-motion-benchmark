"""Independent qualification probes; never rewrites canonical config or inputs.

Manual URDF forward kinematics and a fresh Drake scene provide additional
oracles. The benchmark's analytic sphere distance is not reused by the oracle.
No planning or performance evaluation is performed.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import warnings
import xml.etree.ElementTree as ET

import numpy as np

from benchmarks.generic_motion_v1.model import Model, HERE, ROOT, ASSETS, CONTACT_BAND_M

OUT = Path(__file__).resolve().parent
SEED = 20261027931


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rotation(axis, angle):
    axis = np.asarray(axis, float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(angle) * cross + (1 - np.cos(angle)) * (cross @ cross)


def origin(element):
    X = np.eye(4)
    if element is not None:
        X[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        X[:3, :3] = rotation([0., 0., 1.], y) @ rotation([0., 1., 0.], p) @ rotation([1., 0., 0.], r)
    return X


class ManualURDF:
    """URDF XML + explicit frozen directive welds, no Drake/CSD FK calls."""
    sources = {
        "kinova": "kinova_sens.urdf", "robotiq": "robotiq_2f85_description_box_welded_fingers.urdf",
        "table": "table.urdf", "cam_tripod_0": "camera_and_tripod.urdf",
        "cam_tripod_1": "camera_and_tripod.urdf", "box_to_protect_pete": "box.urdf"}

    def __init__(self):
        self.joints, self.geometry, self.links = [], {}, ["world"]
        self.active = []
        for model, source in self.sources.items():
            root = ET.parse(ASSETS / source).getroot()
            for link in root.findall("link"):
                link_name = model + "::" + link.get("name")
                self.links.append(link_name)
                for collision in link.findall("collision"):
                    # Drake assigns the primitive type to these singleton
                    # unnamed table/wall collisions; the XML pose/size remains
                    # parsed independently here.
                    name = model + "::" + (collision.get("name") or "Box")
                    box = collision.find("geometry/box")
                    assert box is not None, name
                    self.geometry[name] = dict(link=link_name, dimensions=np.fromstring(box.get("size"), sep=" "),
                                               X=origin(collision.find("origin")))
            for joint in root.findall("joint"):
                name, kind = joint.get("name"), joint.get("type")
                assert kind in ("fixed", "revolute")
                axis = joint.find("axis")
                item = dict(name=model + "_" + name, kind=kind,
                    parent=model + "::" + joint.find("parent").get("link"),
                    child=model + "::" + joint.find("child").get("link"),
                    X=origin(joint.find("origin")),
                    axis=np.fromstring(axis.get("xyz"), sep=" ") if axis is not None else None)
                if kind == "revolute":
                    limit = joint.find("limit")
                    item["lower"], item["upper"] = float(limit.get("lower")), float(limit.get("upper"))
                    self.active.append(item)
                self.joints.append(item)
        for parent, child, translation in [
            ("world", "kinova::base_link", [0, 0, 0]),
            ("kinova::end_effector_link", "robotiq::robotiq_85_base_link", [0, 0, 0]),
            ("world", "table::table_body", [.4, 0, 0]),
            ("world", "cam_tripod_0::camera_tripod_link", [.6, .66, 0]),
            ("world", "cam_tripod_1::camera_tripod_link", [.54, -1.04, 0]),
            ("world", "box_to_protect_pete::box_link", [-.5, 0, 0])]:
            X = np.eye(4); X[:3, 3] = translation
            self.joints.append(dict(kind="fixed", parent=parent, child=child, X=X))

    def fk(self, q):
        values = {joint["name"]: float(value) for joint, value in zip(self.active, q)}
        poses, remaining = {"world": np.eye(4)}, list(self.joints)
        while remaining:
            next_round = []
            for joint in remaining:
                if joint["parent"] not in poses:
                    next_round.append(joint); continue
                motion = np.eye(4)
                if joint["kind"] == "revolute":
                    motion[:3, :3] = rotation(joint["axis"], values[joint["name"]])
                poses[joint["child"]] = poses[joint["parent"]] @ joint["X"] @ motion
            assert len(next_round) < len(remaining), "Disconnected XML graph"
            remaining = next_round
        return poses

    def filter_pairs(self, names):
        components = {name: name for name in self.links}
        def root(name):
            while components[name] != name:
                name = components[name]
            return name
        adjacent = set()
        for j in self.joints:
            adjacent.add(frozenset((j["parent"], j["child"])))
            if j["kind"] == "fixed":
                components[root(j["child"])] = root(j["parent"])
        world = {name for name in self.links if root(name) == root("world")}
        checked, excluded = [], []
        for a, b in itertools.combinations(range(len(names)), 2):
            la, lb = self.geometry[names[a]]["link"], self.geometry[names[b]]["link"]
            reasons = []
            if la in world and lb in world: reasons.append("world_world")
            if root(la) == root(lb): reasons.append("same_rigid_component")
            if frozenset((la, lb)) in adjacent: reasons.append("directly_adjacent")
            (excluded if reasons else checked).append(dict(pair=[a, b], reasons=reasons))
        return checked, excluded, world


class FreshDrake:
    """Independent construction, with online spheres genuinely in SceneGraph."""
    def __init__(self, centres=()):
        from pydrake.all import RobotDiagramBuilder, LoadModelDirectives, ProcessModelDirectives, RigidTransform, Sphere, CoulombFriction, Role
        b = RobotDiagramBuilder(); p = b.parser(); self.plant = b.plant(); self.sg = b.scene_graph()
        for name, path in [("test_assets", ASSETS), ("kortex_description", ASSETS / "kinova/kortex_description"),
                           ("robotiq_arg85_description", ASSETS / "robotiq")]:
            p.package_map().Add(name, str(path))
        ProcessModelDirectives(LoadModelDirectives(str(ASSETS / "directives/kinova_sens_on_table.yml")), self.plant, p)
        for index, point in enumerate(centres):
            self.plant.RegisterCollisionGeometry(self.plant.world_body(), RigidTransform(point), Sphere(.04),
                                                f"qualification_voxel_{index}", CoulombFriction(.5, .5))
        self.plant.Finalize(); self.diagram = b.Build(); self.context = self.diagram.CreateDefaultContext()
        self.pc = self.plant.GetMyContextFromRoot(self.context); self.sc = self.sg.GetMyContextFromRoot(self.context)
        self.inspector = self.query().inspector()
        self.ids = {self.inspector.GetName(g): g for f in self.inspector.GetAllFrameIds()
                    for g in self.inspector.GetGeometries(f, Role.kProximity)}

    def query(self):
        return self.sg.get_query_output_port().Eval(self.sc)

    def distance(self, q):
        self.plant.SetPositions(self.pc, q)
        return min(d.distance for d in self.query().ComputeSignedDistancePairwiseClosestPoints())


class SphereBoxOracle:
    """Separate native Drake box/sphere distance engine with no Model formula."""
    def __init__(self, geometry):
        from pydrake.all import SceneGraph, GeometryFrame, GeometryInstance, RigidTransform, Box, Sphere, ProximityProperties
        self.sg = SceneGraph(); self.source = self.sg.RegisterSource("qualification")
        self.boxes = {}
        for name, item in geometry.items():
            frame = self.sg.RegisterFrame(self.source, GeometryFrame(name))
            gid = self.sg.RegisterGeometry(self.source, frame, GeometryInstance(RigidTransform(), Box(*item["dimensions"]), name))
            self.sg.AssignRole(self.source, gid, ProximityProperties())
            self.boxes[name] = frame, gid
        self.sphere_frame = self.sg.RegisterFrame(self.source, GeometryFrame("sphere"))
        self.sphere_id = self.sg.RegisterGeometry(self.source, self.sphere_frame,
            GeometryInstance(RigidTransform(), Sphere(.04), "sphere"))
        self.sg.AssignRole(self.source, self.sphere_id, ProximityProperties())
        self.context = self.sg.CreateDefaultContext()

    def distances(self, poses, centre, names):
        from pydrake.all import RigidTransform, RotationMatrix, FramePoseVector
        frames = FramePoseVector()
        for name, (frame, _) in self.boxes.items():
            X = poses[name]
            frames.set_value(frame, RigidTransform(RotationMatrix(X[:3, :3]), X[:3, 3]))
        frames.set_value(self.sphere_frame, RigidTransform(centre))
        self.sg.get_source_pose_port(self.source).FixValue(self.context, frames)
        query = self.sg.get_query_output_port().Eval(self.context)
        return np.asarray([query.ComputeSignedDistancePairClosestPoints(self.boxes[name][1], self.sphere_id).distance for name in names])


def main(output=OUT / "model_audit_current.json"):
    output = Path(output).resolve()
    raw_path = output.with_name(output.stem + "_raw.npz")
    if output.exists() or raw_path.exists():
        raise FileExistsError("Preserve prior audit evidence; choose a new --output path")
    output.parent.mkdir(exist_ok=True, parents=True)
    guarded = [HERE / "model.py", HERE / "configs/robot.json"]
    unchanged = {str(p.relative_to(ROOT)): sha(p) for p in guarded}
    model, manual, base = Model(), ManualURDF(), FreshDrake()
    rng = np.random.Generator(np.random.PCG64(SEED))
    q = rng.uniform(model.lower + .02, model.upper - .02, (512, 7))
    edges = rng.uniform(model.lower + .02, model.upper - .02, (12, 2, 7))
    edge_q = np.vstack([np.linspace(a, b, 25) for a, b in edges])
    raw = dict(random_q=q, edge_endpoints=edges, edge_q=edge_q)
    names = [g["name"] for g in model.inventory["geometries"]]
    moving = [names[i] for i in model.robot_geometry_ids]
    fixed = [name for name in names if name not in moving]
    expected, excluded, world_links = manual.filter_pairs(names)
    pair_names = {frozenset((names[a], names[b])) for a, b in model.pairs}
    drake_pairs = {frozenset(base.inspector.GetName(g) for g in pair) for pair in base.inspector.GetCollisionCandidates()}
    manual_pairs = {frozenset((names[r["pair"][0]], names[r["pair"][1]])) for r in expected}
    assert pair_names == drake_pairs == manual_pairs
    assert moving == [name for name in names if manual.geometry[name]["link"] not in world_links]
    assert model.joint_names == [j["name"] + "_q" for j in manual.active] == list(base.plant.GetPositionNames())
    tree = model.csd_plant.getKinematicTree()
    errors = dict(manual_vs_drake_geometry=0., manual_vs_csd_geometry=0., manual_vs_drake_all_links=0.,
                  manual_vs_csd_all_links=0., manual_vs_drake_tcp=0., manual_vs_csd_tcp=0., dimensions=0., local_transforms=0.,
                  csd_vs_drake_geometry_rotation_matrix=0., csd_vs_drake_geometry_translation_norm_m=0.,
                  csd_rotation_orthogonality_residual=0., csd_vs_drake_polar_rotation_angle_rad=0.)
    for entry in model.inventory["geometries"]:
        source = manual.geometry[entry["name"]]
        errors["dimensions"] = max(errors["dimensions"], float(np.max(np.abs(source["dimensions"] - entry["dimensions_m"]))))
        errors["local_transforms"] = max(errors["local_transforms"], float(np.max(np.abs(source["X"] - entry["local_transform"]))))
    fk_q = np.vstack([q[:64], np.zeros(7), model.lower, model.upper])
    for state in fk_q:
        M = manual.fk(state); C = np.asarray(tree.compute_link_frame_to_world_transforms(state))
        base.plant.SetPositions(base.pc, state); query = base.query()
        for index, link in enumerate(tree.get_links()):
            if link.name == "world": body = base.plant.world_body()
            else:
                instance, name = link.name.split("::")
                body = base.plant.GetBodyByName(name, base.plant.GetModelInstanceByName(instance))
            D = base.plant.EvalBodyPoseInWorld(base.pc, body).GetAsMatrix4()
            errors["manual_vs_drake_all_links"] = max(errors["manual_vs_drake_all_links"], float(np.max(np.abs(M[link.name] - D))))
            errors["manual_vs_csd_all_links"] = max(errors["manual_vs_csd_all_links"], float(np.max(np.abs(M[link.name] - C[index]))))
            if link.name == model.tcp_name:
                errors["manual_vs_drake_tcp"] = max(errors["manual_vs_drake_tcp"], float(np.max(np.abs(M[link.name] - D))))
                errors["manual_vs_csd_tcp"] = max(errors["manual_vs_csd_tcp"], float(np.max(np.abs(M[link.name] - C[index]))))
        for index, name in enumerate(names):
            g = manual.geometry[name]; W = M[g["link"]] @ g["X"]
            native = model.mplant.scene_collision_geometries[index]
            D = query.GetPoseInWorld(base.ids[name]).GetAsMatrix4()
            CW = C[native.link_index] @ native.X_L_B
            errors["manual_vs_drake_geometry"] = max(errors["manual_vs_drake_geometry"], float(np.max(np.abs(W - D))))
            errors["manual_vs_csd_geometry"] = max(errors["manual_vs_csd_geometry"], float(np.max(np.abs(W - CW))))
            errors["csd_vs_drake_geometry_rotation_matrix"] = max(errors["csd_vs_drake_geometry_rotation_matrix"], float(np.max(np.abs(CW[:3, :3] - D[:3, :3]))))
            errors["csd_vs_drake_geometry_translation_norm_m"] = max(errors["csd_vs_drake_geometry_translation_norm_m"], float(np.linalg.norm(CW[:3, 3] - D[:3, 3])))
            errors["csd_rotation_orthogonality_residual"] = max(errors["csd_rotation_orthogonality_residual"], float(np.max(np.abs(CW[:3, :3].T @ CW[:3, :3] - np.eye(3)))))
            U, _, V = np.linalg.svd(CW[:3, :3].astype(float)); relative = (U @ V).T @ D[:3, :3]
            skew = np.array([relative[2, 1] - relative[1, 2], relative[0, 2] - relative[2, 0], relative[1, 0] - relative[0, 1]])
            angle = float(np.arctan2(np.linalg.norm(skew) / 2, (np.trace(relative) - 1) / 2))
            errors["csd_vs_drake_polar_rotation_angle_rad"] = max(errors["csd_vs_drake_polar_rotation_angle_rad"], angle)
    report = dict(seed=SEED, random_generator="numpy.random.PCG64", frozen_sources_before=unchanged,
        model_identity=dict(robot_sha256=model.robot_sha256, collision_geometry_sha256=model.collision_geometry_sha256,
                            inventory_matches_frozen_json=model.inventory == json.loads((HERE / "configs/robot.json").read_text())),
        geometry=dict(active_joint_names=model.joint_names, all_links=len(manual.links), primitives=len(names),
            moving=len(moving), fixed=len(fixed), checked_pairs=len(expected), excluded_pairs=len(excluded),
            manual_drake_csd_filter_sets_equal=True, exclusions=excluded, fk_states=len(fk_q), max_abs_errors=errors,
            lower_limit_float32_difference=float(np.max(np.abs(model.lower - [j["lower"] for j in manual.active]))),
            upper_limit_float32_difference=float(np.max(np.abs(model.upper - [j["upper"] for j in manual.active])))))
    def check(label, states, oracle):
        distance = np.asarray([oracle.distance(state) for state in states])
        actual = model.clearance(states)
        native = model.cuda(states)
        diff = native != (distance > 0)
        raw[label + "_q"] = states; raw[label + "_native_free"] = native
        raw[label + "_oracle_distance"] = distance; raw[label + "_adapter_distance"] = actual
        report[label] = dict(count=len(states), native_free=int(native.sum()), oracle_free=int((distance > 0).sum()),
            disagreements=int(diff.sum()), disagreements_outside_band=int(np.sum(diff & (np.abs(distance) > CONTACT_BAND_M))),
            cuda_false_free_count=int(np.sum(native & (distance <= 0))),
            cuda_false_collision_count=int(np.sum(~native & (distance > 0))),
            maximum_false_free_oracle_penetration_m=float(np.max(np.where(native & (distance <= 0), -distance, 0.))),
            maximum_false_collision_oracle_clearance_m=float(np.max(np.where(~native & (distance > 0), distance, 0.))),
            adapter_vs_oracle_max_distance_error=float(np.max(np.abs(actual - distance))),
            mismatch_indices=np.flatnonzero(diff).tolist())
        print(label, report[label], flush=True)
        return distance
    base_d = check("base_random", q, base)
    check("base_edges", edge_q, base)
    valid_states = q[base_d > 0]
    tcp, bounds = model.tcp(valid_states), model.robot_bounds(valid_states)
    raw["base_free_tcp_xyz"], raw["base_free_robot_bounds"] = tcp, bounds
    report["workspace"] = dict(sampled_valid_configurations=len(valid_states),
        tcp_min=tcp.min(axis=0).tolist(), tcp_max=tcp.max(axis=0).tolist(),
        tcp_quantiles=np.quantile(tcp, [.05, .5, .95], axis=0).tolist(),
        moving_collision_geometry_min=bounds[:, 0].min(axis=0).tolist(),
        moving_collision_geometry_max=bounds[:, 1].max(axis=0).tolist(),
        note="Fresh oracle-free configuration samples audit the obstacle ROI; they do not prove reachability of every point.")
    centres = rng.uniform([-.3, -.75, .1], [.9, .75, 1.0], (24, 3)).astype(np.float32).astype(float)
    model.set_voxels(centres); with_spheres = FreshDrake(centres)
    assert len(with_spheres.inspector.GetCollisionCandidates()) == len(model.pairs) + 15 * len(centres)
    raw["online_centres"] = centres
    check("online_random", q, with_spheres); check("online_edges", edge_q, with_spheres)
    model.set_voxels()
    # Locate true static collision transitions using an oracle-only bisection,
    # then test both sides. These are checker probes, never dataset screening.
    free = q[base_d > 1e-4]; colliding = q[base_d < -1e-4]
    contact_q, contact_meta = [], []
    offsets = np.array([-1e-4, -1e-5, -1e-6, -1e-7, 0., 1e-7, 1e-6, 1e-5, 1e-4])
    for index in range(12):
        a, b = free[index], colliding[index]
        lo, hi = 0., 1.
        for _ in range(48):
            mid = (lo + hi) / 2
            if base.distance(a + mid * (b - a)) > 0: lo = mid
            else: hi = mid
        root = (lo + hi) / 2
        for offset in offsets:
            state = a + (root + offset / np.linalg.norm(b - a)) * (b - a)
            contact_q.append(state); contact_meta.append(dict(edge=index, joint_offset_rad=float(offset)))
    check("static_contact", np.asarray(contact_q), base)
    report["static_contact"]["probes"] = contact_meta
    rounded_distance = np.asarray([base.distance(state.astype(np.float32).astype(float)) for state in contact_q])
    raw["static_contact_float32_q_oracle_distance"] = rounded_distance
    report["static_contact"]["joint_rounding_max_clearance_change_m"] = float(np.max(np.abs(rounded_distance - raw["static_contact_oracle_distance"])))
    # Face, edge and corner probes for every moving primitive. The native
    # SceneGraph pair oracle also distinguishes targeted contacts masked by a
    # different body, something the original aggregate-only audit did not do.
    pair_oracle = SphereBoxOracle(manual.geometry)
    q0 = free[0]; link_poses = manual.fk(q0)
    box_poses = {name: link_poses[g["link"]] @ g["X"] for name, g in manual.geometry.items()}
    boundary, sphere_centres = [], []
    for name in moving:
        g = manual.geometry[name]; X = box_poses[name]
        for feature, raw_direction in [("face", [1., 0, 0]), ("edge", [1., 1., 0]), ("corner", [1., 1., 1.])]:
            for sign in (-1, 1):
                direction = sign * np.asarray(raw_direction) / np.linalg.norm(raw_direction)
                contact = np.sign(direction) * g["dimensions"] / 2
                for offset in (-1e-3, -1e-5, -1e-6, 0., 1e-6, 1e-5, 1e-3):
                    centre = (X[:3, :3] @ (contact + direction * (.04 + offset)) + X[:3, 3]).astype(np.float32).astype(float)
                    exact_pairs = pair_oracle.distances(box_poses, centre, moving)
                    target = float(exact_pairs[moving.index(name)])
                    oracle_distance = min(float(exact_pairs.min()), float(base_d[np.flatnonzero(base_d > 1e-4)[0]]))
                    model.set_voxels([centre]); native = bool(model.cuda(q0)[0]); adapter = float(model.clearance(q0)[0])
                    boundary.append(dict(geometry=name, feature=feature, sign=sign, offset_m=offset,
                        target_native_drake_distance_m=target, oracle_distance_m=oracle_distance,
                        targeted_boundary_is_global_min=bool(abs(target - oracle_distance) < 1e-10),
                        adapter_distance_m=adapter, native_free=native, oracle_free=oracle_distance > 0,
                        outside_contact_band=abs(oracle_distance) > CONTACT_BAND_M))
                    sphere_centres.append(centre)
    raw["sphere_boundary_configuration"] = q0; raw["sphere_boundary_centres"] = sphere_centres
    report["sphere_boundaries"] = dict(count=len(boundary), geometries=len(moving), features=["face", "edge", "corner"],
        exposed_target_probes=sum(x["targeted_boundary_is_global_min"] for x in boundary),
        masked_target_probes=sum(not x["targeted_boundary_is_global_min"] for x in boundary),
        disagreements=sum(x["native_free"] != x["oracle_free"] for x in boundary),
        disagreements_outside_band=sum(x["native_free"] != x["oracle_free"] and x["outside_contact_band"] for x in boundary),
        cuda_false_free_count=sum(x["native_free"] and not x["oracle_free"] for x in boundary),
        cuda_false_collision_count=sum(not x["native_free"] and x["oracle_free"] for x in boundary),
        maximum_false_free_oracle_penetration_m=max([0.] + [-x["oracle_distance_m"] for x in boundary if x["native_free"] and not x["oracle_free"]]),
        maximum_false_collision_oracle_clearance_m=max([0.] + [x["oracle_distance_m"] for x in boundary if not x["native_free"] and x["oracle_free"]]),
        adapter_vs_oracle_max_distance_error=max(abs(x["adapter_distance_m"] - x["oracle_distance_m"]) for x in boundary), probes=boundary)
    # Deliberately invalid input scenes expose the differing CPU binding; they
    # are NOT benchmark cases and do not change any retained task or model.
    cpu = []
    for centre in ([0., 0., .02], [0., 0., .04], [0., 0., .08]):
        model.set_voxels([centre])
        cpu.append(dict(centre=centre, cuda_free=bool(model.cuda(q0)[0]),
            adapter_distance_m=float(model.clearance(q0)[0]),
            fixed_geometry_clearance_m=float(model.fixed_geometry_sphere_clearance([centre])[0]),
            upstream_cpu_wrapper_free=bool(model.csd.CheckCollisionFreeVoxels(q0.astype(np.float32), model.voxels, .04, model.mplant, model.robot_geometry_ids))))
    report["upstream_cpu_voxel_wrapper_reproduction"] = cpu
    model.set_voxels()
    # Fixed-world obstacle validity is outside the CUDA robot-pair predicate;
    # independently check its separate adapter against native Drake distances.
    fixed_probes = []
    for name in fixed:
        g = manual.geometry[name]; X = box_poses[name]
        for direction in (np.zeros(3), np.array([1., 0, 0]), np.array([-1., 0, 0]), np.array([0, 0, 1.])):
            local = direction * (g["dimensions"] / 2 + .041)
            centre = X[:3, :3] @ local + X[:3, 3]
            measured = float(pair_oracle.distances(box_poses, centre, fixed).min())
            observed = float(model.fixed_geometry_sphere_clearance([centre])[0])
            fixed_probes.append(dict(geometry=name, centre=centre.tolist(), oracle_distance_m=measured, adapter_distance_m=observed))
    report["fixed_geometry_sphere_adapter"] = dict(count=len(fixed_probes),
        max_abs_distance_error=max(abs(x["oracle_distance_m"] - x["adapter_distance_m"]) for x in fixed_probes), probes=fixed_probes)
    # Real frozen development scenes, not just artificial random obstacles:
    # first scene per family, fixed endpoint edge, and fresh uniform states.
    dev_path = HERE / "manifests/dev.jsonl"
    selected = {}
    for line in dev_path.read_text().splitlines():
        case = json.loads(line)
        selected.setdefault(case["family"], case)
    report["real_dev_scenes"] = []
    for family, case in selected.items():
        centres = np.asarray(case["voxels_xyz"], float)
        model.set_voxels(centres)
        oracle = FreshDrake(centres)
        random_q = rng.uniform(model.lower + .02, model.upper - .02, (32, 7))
        line_q = np.linspace(case["q_start"], case["q_goal"], 33)
        label = "dev_" + family
        check(label, np.vstack([random_q, line_q]), oracle)
        raw[label + "_voxels"] = centres
        report["real_dev_scenes"].append(dict(family=family, problem_id=case["problem_id"], scene_id=case["scene_id"],
            scene_sha256=case["scene_sha256"], voxel_count=len(centres),
            centres_min=centres.min(axis=0).tolist(), centres_max=centres.max(axis=0).tolist(),
            fresh_random_states=32, fixed_endpoint_edge_states=33,
            expected_native_drake_pairs=len(model.pairs) + len(moving) * len(centres), result_key=label))
    report["real_dev_manifest_sha256"] = sha(dev_path)
    # A finite float64 centre can overflow during the adapter's float32 cast.
    # Do not invoke GPU for this malformed scene; record the accepted invalid
    # representation for the parent to fix at the input boundary.
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        try:
            model.set_voxels([[1e300, 0., 0.]])
            overflow = dict(rejected=False, stored_centres_finite=bool(np.isfinite(model.voxels_xyz).all()))
        except (ValueError, RuntimeError, OverflowError) as error:
            overflow = dict(rejected=True, error=repr(error))
        overflow["warnings"] = [str(w.message) for w in caught]
        overflow["gpu_invoked"] = False
    report["finite_float64_to_float32_overflow_guard"] = overflow
    model.set_voxels()
    old = json.loads((HERE / "reports/model_audit/audit.json").read_text())
    probes = old["boundary"]["samples"]
    report["previous_audit_scope"] = dict(geometry=old["boundary"]["tested_geometry"], total=len(probes),
        positive_clearance=sum(s["clearance_m"] > 0 for s in probes),
        negative_clearance=sum(s["clearance_m"] < 0 for s in probes),
        limitation="One geometry; face offsets only. Aggregate label may be masked by another collision. No independent XML FK.")
    np.savez_compressed(raw_path, **raw)
    report["frozen_sources_after"] = {str(p.relative_to(ROOT)): sha(p) for p in guarded}
    assert unchanged == report["frozen_sources_after"]
    report["source_files_sha256"] = {str(p.relative_to(ROOT)): sha(p) for p in
        [Path(__file__), ASSETS / "directives/kinova_sens_on_table.yml",
         ROOT / "csdecomp/src/pybind/csdecomp/collision_checker_bindings.h",
         ROOT / "csdecomp/src/pybind/csdecomp/drake_csd_bridge.py"]}
    report["raw_file"] = str(raw_path.relative_to(ROOT))
    report["raw_sha256"] = sha(raw_path)
    report["passed_scoped_model_checks"] = bool(report["model_identity"]["inventory_matches_frozen_json"]
        and errors["manual_vs_csd_geometry"] < CONTACT_BAND_M
        and all(report[label]["disagreements_outside_band"] == 0 for label in
                ["base_random", "base_edges", "online_random", "online_edges", "static_contact", "sphere_boundaries"]
                + ["dev_" + family for family in selected]))
    report["independence_limit"] = "Auditor previously implemented the adapter; this is additional independent oracle testing, not independent institutional replication. Both loaders share source URDFs; primitive physical fidelity and exhaustiveness are not established."
    report["model"] = model.inventory
    report["model_source_sha256"] = sha(HERE / "model.py")
    report["passed"] = bool(report["passed_scoped_model_checks"] and overflow["rejected"])
    report["status"] = "passed_with_explicit_contact_band" if report["passed"] else "failed"
    report["contact_band_m"] = CONTACT_BAND_M
    report["contact_band_limit"] = "Empirical acceptance policy, not an analytic floating-point bound. All near-contact false-free and false-collision observations are retained, including those inside this band."
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({key: value for key, value in report.items() if key not in ["sphere_boundaries", "static_contact", "geometry"]}, indent=2))
    print("sphere_boundaries", {k: v for k, v in report["sphere_boundaries"].items() if k != "probes"})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUT / "model_audit_current.json")
    main(parser.parse_args().output)
