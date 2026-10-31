"""Independent official-mesh/box audit of the constructed FR3 sphere model.

Prepare fixes all samples before loading the constructed asset. Validate uses
Drake Convex hulls of official collision STL meshes and exact finger boxes.
Hull separation certifies mesh separation; hull penetration alone does not
certify contact of the original non-convex meshes. No CUDA or solver is used.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys
import time
import traceback
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from pydrake.all import (Box, CollisionFilterDeclaration, Convex, CoulombFriction,
                        GeometrySet, RigidTransform, RobotDiagramBuilder,
                        RollPitchYaw, Sphere)
from reproduction.common import ROOT, atomic, read

REVISION = "39f8c0bcea9bc48ae5334edcb2aa2a504f2d2c85"
SOURCE = ROOT / "reproduction/vendor/franka_description"
DEFAULT_OUT = ROOT / "reproduction/reports/paper_constructed/geometry_audit"
REFERENCE_CONFIG = ROOT / "reproduction/paper_completion/config.json"
ERA_AUDIT = ROOT / "reproduction/reports/paper_completion/fr3_geometry_era_comparison.json"
OLD_SCAN = ROOT / "reproduction/reports/paper_completion/diagnostics/pipeline/mesh_comparison/results.json"
OFFICIAL_FILES = ["robots/fr3/kinematics.yaml", "robots/fr3/joint_limits.yaml",
                  "end_effectors/common/franka_hand.xacro", "robots/common/franka_arm.srdf.xacro",
                  "end_effectors/franka_hand/franka_hand.srdf.xacro"]
INTERPRETATION = (
    "The reference uses enclosing convex hulls of official collision meshes and exact official finger boxes. "
    "Positive hull distance proves separation of those original meshes; negative hull distance may be caused "
    "by filled concavities. False negatives relative to this reference are candidates, not automatically "
    "proven original-mesh collisions. Strict sign uses distance < 0 without collision-threshold relaxation."
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def numbers(text):
    # Only the three literal pi expressions present in the pinned finger xacro.
    replacement = {"${pi/6}": str(np.pi / 6), "${-pi/6}": str(-np.pi / 6), "${pi}": str(np.pi)}
    for key, value in replacement.items():
        text = text.replace(key, value)
    return np.array([float(value) for value in text.split()])


def official_sources(out):
    records = []
    for relative in OFFICIAL_FILES:
        target = out / "official" / relative
        url = f"https://raw.githubusercontent.com/frankarobotics/franka_description/{REVISION}/{relative}"
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            with urllib.request.urlopen(url, timeout=45) as response:
                raw = response.read()
            target.write_bytes(raw)
        records.append(dict(path=str(target), upstream_path=relative, url=url, sha256=sha(target)))
    era = read(ERA_AUDIT)
    for entry in era:
        if not entry["current_path"].endswith(".stl"):
            continue
        path = SOURCE / entry["current_path"]
        actual = sha(path)
        if not entry["identical"] or actual != entry["previous_sha256"]:
            raise ValueError("Official prepublication mesh identity mismatch: " + str(path))
        records.append(dict(path=str(path), upstream_path=entry["previous_path"], url=entry["url"], sha256=actual))
    return records


def description(out):
    official = out / "official"
    kinematics = yaml.safe_load((official / "robots/fr3/kinematics.yaml").read_text())
    limits = yaml.safe_load((official / "robots/fr3/joint_limits.yaml").read_text())
    disabled = []
    for relative in OFFICIAL_FILES[-2:]:
        for elem in ET.parse(official / relative).iter("disable_collisions"):
            pair = [elem.attrib[key].replace("${prefix}", "fr3_") for key in ("link1", "link2")]
            if any("${" in name for name in pair):
                raise ValueError("Unresolved SRDF name: " + repr(pair))
            disabled.append(dict(first=pair[0], second=pair[1], reason=elem.attrib["reason"]))
    boxes = {}
    xml = ET.parse(official / "end_effectors/common/franka_hand.xacro")
    for link in xml.iter("link"):
        name = link.attrib["name"].replace("${ee_prefix}", "fr3_")
        if name not in ("fr3_leftfinger", "fr3_rightfinger"):
            continue
        boxes[name] = []
        for collision in link.findall("collision"):
            origin = collision.find("origin")
            boxes[name].append(dict(center=numbers(origin.attrib["xyz"]).tolist(),
                                    rpy=numbers(origin.attrib.get("rpy", "0 0 0")).tolist(),
                                    size=numbers(collision.find("geometry/box").attrib["size"]).tolist()))
    if sorted(map(len, boxes.values())) != [4, 4]:
        raise ValueError("Expected exactly four official boxes on each finger")
    return dict(kinematics=kinematics, limits=limits, disabled=disabled, boxes=boxes)


def bare_chain(desc):
    """Kinematic-only preparation chain from the official pinned files."""
    names = [f"fr3_link{i}" for i in range(9)] + ["fr3_hand", "fr3_leftfinger", "fr3_rightfinger", "fr3_hand_tcp"]
    pieces = ['<robot name="fr3">'] + [f'<link name="{name}"/>' for name in names]
    pieces.append('<joint name="base_weld" type="fixed"><parent link="world"/><child link="fr3_link0"/></joint>')
    for i in range(1, 9):
        origin = desc["kinematics"][f"joint{i}"]["kinematic"]
        xyz = " ".join(str(origin[key]) for key in ("x", "y", "z"))
        rpy = " ".join(str(origin[key]) for key in ("roll", "pitch", "yaw"))
        kind = "revolute" if i < 8 else "fixed"
        pieces.append(f'<joint name="fr3_joint{i}" type="{kind}"><parent link="fr3_link{i-1}"/><child link="fr3_link{i}"/><origin xyz="{xyz}" rpy="{rpy}"/>')
        if i < 8:
            lim = desc["limits"][f"joint{i}"]["limit"]
            pieces.append(f'<axis xyz="0 0 1"/><limit lower="{lim["lower"]}" upper="{lim["upper"]}" velocity="{lim["velocity"]}" effort="{lim["effort"]}"/>')
        pieces.append('</joint>')
    pieces.append(f'<joint name="hand_weld" type="fixed"><parent link="fr3_link8"/><child link="fr3_hand"/><origin rpy="0 0 {-np.pi/4}"/></joint>')
    for name, xyz, yaw in [("fr3_leftfinger", "0 0.04 0.0584", 0), ("fr3_rightfinger", "0 -0.04 0.0584", np.pi), ("fr3_hand_tcp", "0 0 0.1034", 0)]:
        pieces.append(f'<joint name="{name}_weld" type="fixed"><parent link="fr3_hand"/><child link="{name}"/><origin xyz="{xyz}" rpy="0 0 {yaw}"/></joint>')
    return "".join(pieces) + '</robot>'


def mesh_obj(path, target):
    raw = path.read_bytes()
    count = int.from_bytes(raw[80:84], "little")
    if len(raw) != 84 + 50 * count:
        raise ValueError("Expected binary STL: " + str(path))
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    triangles = np.frombuffer(raw, dtype=dtype, offset=84)["vertices"].astype(float)
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    text = ''.join('v %.17g %.17g %.17g\n' % tuple(v) for v in vertices)
    text += ''.join('f %d %d %d\n' % tuple(face + 1) for face in inverse.reshape(-1, 3))
    target.parent.mkdir(parents=True, exist_ok=True)
    if not target.exists():
        target.write_text(text)
    elif target.read_text() != text:
        raise ValueError("Frozen OBJ differs: " + str(target))
    return target


class GeometryOracle:
    def __init__(self, desc, out, chain, scene=None, obstacles=(), official=True):
        xml = ET.fromstring(chain)
        if official:
            for link in xml.findall("link"):
                for collision in link.findall("collision"):
                    link.remove(collision)
        builder = RobotDiagramBuilder()
        self.plant, self.sg = builder.plant(), builder.scene_graph()
        self.instance = builder.parser().AddModelsFromString(ET.tostring(xml, encoding="unicode"), "urdf")[0]
        self.kinds = {}
        if official:
            mesh_paths = {f"fr3_link{i}": SOURCE / f"meshes/robots/fr3/collision/link{i}.stl" for i in range(8)}
            mesh_paths["fr3_hand"] = SOURCE / "meshes/robot_ee/franka_hand_white/collision/hand.stl"
            for name, source in mesh_paths.items():
                obj = mesh_obj(source, out / "mesh_obj" / (name + '.obj'))
                gid = self.plant.RegisterCollisionGeometry(self.plant.GetBodyByName(name, self.instance), RigidTransform(), Convex(str(obj)), name + "_official_convex_hull", CoulombFriction(.9, .8))
                self.kinds[gid] = "official_mesh_convex_hull"
            for name, boxes in desc["boxes"].items():
                for i, box in enumerate(boxes):
                    gid = self.plant.RegisterCollisionGeometry(self.plant.GetBodyByName(name, self.instance), RigidTransform(RollPitchYaw(box["rpy"]), box["center"]), Box(*box["size"]), f"{name}_official_box_{i}", CoulombFriction(.9, .8))
                    self.kinds[gid] = "exact_box"
        if scene is not None:
            self.plant.RegisterCollisionGeometry(self.plant.world_body(), RigidTransform(scene["table_center"]), Box(*scene["table_size"]), "table", CoulombFriction(.9, .8))
        obstacle_ids = []
        if scene is not None:
            for i, center in enumerate(obstacles):
                obstacle_ids.append(self.plant.RegisterCollisionGeometry(self.plant.world_body(), RigidTransform(center), Sphere(scene["obstacle_radius"]), f"obstacle_{i}", CoulombFriction(.9, .8)))
        self.plant.Finalize()
        for pair in desc["disabled"]:
            groups = [GeometrySet(self.plant.GetCollisionGeometriesForBody(self.plant.GetBodyByName(pair[key], self.instance))) for key in ("first", "second")]
            self.sg.collision_filter_manager().Apply(CollisionFilterDeclaration().ExcludeBetween(*groups))
        if obstacle_ids:
            robot_ids = [gid for index in self.plant.GetBodyIndices(self.instance)
                         for gid in self.plant.GetCollisionGeometriesForBody(self.plant.get_body(index))]
            # Online obstacles must test all robot geometry, including the
            # anchored base. Keep base/table and all SRDF self filters intact.
            self.sg.collision_filter_manager().Apply(CollisionFilterDeclaration().AllowBetween(
                GeometrySet(robot_ids), GeometrySet(obstacle_ids)))
        self.diagram = builder.Build()
        self.context = self.diagram.CreateDefaultContext()
        self.pc = self.plant.GetMyMutableContextFromRoot(self.context)
        self.sc = self.sg.GetMyContextFromRoot(self.context)
        self.inspector = self.sg.get_query_output_port().Eval(self.sc).inspector()
        self.tcp = self.plant.GetFrameByName("fr3_hand_tcp", self.instance)
        links = {}
        for body_index in [self.plant.world_body().index(), *self.plant.GetBodyIndices(self.instance)]:
            body = self.plant.get_body(body_index)
            for gid in self.plant.GetCollisionGeometriesForBody(body):
                # Static geometries share the world body but remain separate named objects.
                name = self.inspector.GetName(gid)
                links[gid] = name if body.name() == "world" else body.name()
                if gid not in self.kinds:
                    self.kinds[gid] = "sphere" if isinstance(self.inspector.GetShape(gid), Sphere) else "exact_box"
        self.pairs = sorted(self.inspector.GetCollisionCandidates(), key=lambda pair: tuple(sorted(self.inspector.GetName(gid) for gid in pair)))
        self.pair_index = {frozenset(pair): i for i, pair in enumerate(self.pairs)}
        self.pair_metadata = [dict(first_geometry=self.inspector.GetName(a), second_geometry=self.inspector.GetName(b), first_link=links[a], second_link=links[b], first_kind=self.kinds[a], second_kind=self.kinds[b]) for a, b in self.pairs]

    def pose(self, q):
        self.plant.SetPositions(self.pc, q)
        return self.plant.CalcRelativeTransform(self.pc, self.plant.world_frame(), self.tcp).GetAsMatrix4()

    def distances(self, q):
        self.plant.SetPositions(self.pc, q)
        query = self.sg.get_query_output_port().Eval(self.sc)
        distances = np.full(len(self.pairs), np.nan)
        for pair in query.ComputeSignedDistancePairwiseClosestPoints(np.inf):
            distances[self.pair_index[frozenset((pair.id_A, pair.id_B))]] = pair.distance
        if not np.all(np.isfinite(distances)):
            raise ValueError("Missing or nonfinite candidate-pair distance")
        return distances


def prepare(out):
    prereg_path = out / "preregistration.json"
    if prereg_path.exists():
        prereg = read(prereg_path)
        for name, expected in prereg["frozen_files"].items():
            if sha(out / name) != expected:
                raise ValueError("Frozen sample/source changed: " + name)
        for source in prereg["sources"]:
            if sha(source["path"]) != source["sha256"]:
                raise ValueError("Pinned official source changed: " + source["path"])
        print(json.dumps(dict(status="existing_preregistration_verified", samples=prereg["samples"], path=str(prereg_path))))
        return prereg
    sources = official_sources(out)
    desc = description(out)
    cfg = read(REFERENCE_CONFIG)
    scene = cfg["fr3_scene"]
    if scene["finger_opening"] != .04:
        raise ValueError("Fixed open-finger preparation requires 0.04m")
    atomic(out / "official_description.json", desc)
    atomic(out / "scene_configuration.json", scene)
    oracle = GeometryOracle(desc, out, bare_chain(desc))
    lower, upper = oracle.plant.GetPositionLowerLimits(), oracle.plant.GetPositionUpperLimits()
    old = read(OLD_SCAN)
    scan = [row["configuration"] for row in old["rows"] if row["label"] == "node2922_q7_scan"]
    if len(scan) != 31:
        raise ValueError("Missing original fixed q7 scan")
    rows = [dict(sample_id="home", group="home", q=[0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, 0])]
    rows += [dict(sample_id=f"q7_{i:02}", group="old_q7_scan", q=q) for i, q in enumerate(scan)]
    random_seed = [250410783, 20261026, 601]
    rng = np.random.default_rng(np.random.SeedSequence(random_seed))
    rows += [dict(sample_id=f"joint_{i:04}", group="uniform_joint", q=q) for i, q in enumerate(rng.uniform(lower, upper, (1024, 7)))]
    workspace_seed = [250410783, 20261026, 602]
    rng = np.random.default_rng(np.random.SeedSequence(workspace_seed))
    accepted = {"left": [], "right": []}
    pool, xyzs = [], []
    for i in range(65536):
        q = rng.uniform(lower, upper)
        xyz = oracle.pose(q)[:3, 3]
        pool.append(q)
        xyzs.append(xyz)
        for side in accepted:
            if len(accepted[side]) < 64 and np.all(xyz >= scene[side + "_ee_lower"]) and np.all(xyz <= scene[side + "_ee_upper"]):
                accepted[side].append(i)
                rows.append(dict(sample_id=f"workspace_{side}_{len(accepted[side])-1:02}", group="workspace_" + side, q=q, workspace_pool_index=i))
        if all(len(indices) == 64 for indices in accepted.values()):
            break
    obstacles = np.random.default_rng(np.random.SeedSequence([250410783, 0])).uniform(scene["obstacle_lower"], scene["obstacle_upper"], (20, 3))
    np.savez_compressed(out / "samples.npz", configurations=np.array([row["q"] for row in rows]), groups=np.array([row["group"] for row in rows]), sample_ids=np.array([row["sample_id"] for row in rows]), obstacles=obstacles, workspace_pool=np.array(pool), workspace_tcp_xyz=np.array(xyzs), lower=lower, upper=upper)
    atomic(out / "sample_metadata.json", rows)
    frozen = {str(path.relative_to(out)): sha(path) for path in [out / "samples.npz", out / "sample_metadata.json", out / "official_description.json", out / "scene_configuration.json", *sorted((out / "official").rglob("*"))] if path.is_file()}
    prereg = dict(status="prepared", prepared_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), script_sha256=sha(Path(__file__)), source_revision=REVISION, sources=sources, frozen_files=frozen, samples=len(rows), group_counts=dict(Counter(row["group"] for row in rows)), uniform_joint_seed=random_seed, workspace_seed=workspace_seed, workspace_selection="first 64 each side within fixed TCP boxes; no collision filtering or sphere-model reads", workspace_candidate_limit=65536, workspace_candidates=len(pool), workspace_complete=all(len(x) == 64 for x in accepted.values()), constructed_asset_read=False, reference_config=dict(path=str(REFERENCE_CONFIG), sha256=sha(REFERENCE_CONFIG)), old_scan=dict(path=str(OLD_SCAN), sha256=sha(OLD_SCAN)), finger_opening=.04, threshold=0, interpretation=INTERPRETATION, drake_version=importlib.metadata.version("drake"))
    atomic(prereg_path, prereg)
    print(json.dumps(dict(status="prepared", samples=len(rows), groups=prereg["group_counts"], workspace_candidates=len(pool))), flush=True)
    return prereg


def summarize_rows(rows):
    result = {}
    for group in ["all", *sorted(set(row["group"] for row in rows))]:
        selected = rows if group == "all" else [row for row in rows if row["group"] == group]
        counts = Counter(row["classification"] for row in selected)
        result[group] = dict(samples=len(selected), counts=dict(counts), sphere_collision_rate_given_evaluated=sum(row["sphere_collision"] for row in selected)/len(selected) if selected else None, reference_collision_rate_given_evaluated=sum(row["reference_collision"] for row in selected)/len(selected) if selected else None, exact_primitive_false_negatives=sum(row["exact_primitive_false_negative"] for row in selected), near_contact_reference=sum(row["reference_near_contact"] for row in selected), near_contact_spheres=sum(row["sphere_near_contact"] for row in selected), out_of_domain=sum(not row["within_joint_limits"] for row in selected))
    return result


def validate(out, result_dir):
    prereg = prepare(out)
    desc, scene = read(out / "official_description.json"), read(out / "scene_configuration.json")
    config_path = ROOT / "reproduction/paper_constructed/config.json"
    if read(config_path)["fr3_scene"] != scene:
        raise ValueError("Constructed scene differs from preregistered geometry inputs")
    # Import only after preparation has fixed the samples, and use the new
    # model's exact URDF chain, replacing collision geometry for the oracle.
    from reproduction.paper_constructed.model import fr3_urdf, ASSETS
    asset_path = ASSETS / "fr3_33.json"
    asset_identity = sha(asset_path)
    asset = read(asset_path)
    if sum(len(balls) for balls in asset["spheres"].values()) != 33:
        raise ValueError("Constructed comparison requires exactly 33 spheres")
    expected_filters = {frozenset((row["first"], row["second"])) for row in desc["disabled"]}
    if {frozenset(pair) for pair in asset["disabled_collision_links"]} != expected_filters:
        raise ValueError("Constructed filters differ from independently parsed official SRDF")
    if asset["kinematics"] != desc["kinematics"] or asset["limits"] != desc["limits"]:
        raise ValueError("Constructed chain/limits differ from official pinned source")
    chain = fr3_urdf()
    if sha(asset_path) != asset_identity:
        raise ValueError("Constructed asset changed while building its chain")
    samples = np.load(out / "samples.npz")
    q_values = samples["configurations"]
    bare = GeometryOracle(desc, out, bare_chain(desc))
    results = {}
    result_dir.mkdir(parents=True, exist_ok=False)
    atomic(result_dir / "input_identity.json", dict(asset_path=str(asset_path), asset_sha256=asset_identity, constructed_config_path=str(config_path), constructed_config_sha256=sha(config_path), geometry_configuration_matches_preregistered=True, preregistration_sha256=sha(out / "preregistration.json"), sample_sha256=sha(out / "samples.npz"), script_sha256=sha(Path(__file__)), command=sys.argv, interpretation=INTERPRETATION, online_policy="all robot geometries versus each obstacle, including fixed base; default welded base/table filter retained"))
    for condition, obstacles in [("static_table", []), ("online_20_spheres", samples["obstacles"])]:
        reference = GeometryOracle(desc, out, chain, scene, obstacles, official=True)
        spheres = GeometryOracle(desc, out, chain, scene, obstacles, official=False)
        r_distances = np.full((len(q_values), len(reference.pairs)), np.nan)
        s_distances = np.full((len(q_values), len(spheres.pairs)), np.nan)
        rows = []
        def body_pair_groups(metadata):
            groups = {}
            for j, pair in enumerate(metadata):
                groups.setdefault(tuple(sorted((pair["first_link"], pair["second_link"]))), []).append(j)
            return groups
        r_groups = body_pair_groups(reference.pair_metadata)
        s_groups = body_pair_groups(spheres.pair_metadata)
        if set(r_groups) != set(s_groups):
            raise ValueError("Official/sphere body-pair policies differ")
        failures = []
        for i, q in enumerate(q_values):
            try:
                pose_error = float(np.max(abs(reference.pose(q) - bare.pose(q))))
                if pose_error > 1e-12:
                    raise ValueError("Constructed chain differs from independent preparation FK")
                r, s = reference.distances(q), spheres.distances(q)
                r_distances[i], s_distances[i] = r, s
                rc, sc = bool(np.min(r) < 0), bool(np.min(s) < 0)
                classification = "both_collide" if rc and sc else "both_free" if not rc and not sc else "sphere_false_positive" if sc else "reference_false_negative_candidate"
                def collisions(distances, metadata):
                    return [dict(metadata[j], distance=float(distances[j])) for j in np.argsort(distances) if distances[j] < 0]
                reference_collisions = collisions(r, reference.pair_metadata)
                pair_differences = []
                for pair, r_indices in r_groups.items():
                    rd, sd = float(np.min(r[r_indices])), float(np.min(s[s_groups[pair]]))
                    if (rd < 0) != (sd < 0):
                        pair_differences.append(dict(links=pair, sphere_distance=sd, reference_distance=rd,
                                                    classification="sphere_false_positive_pair" if sd < 0 else "reference_false_negative_pair_candidate"))
                exact_miss = not sc and any("official_mesh_convex_hull" not in (pair["first_kind"], pair["second_kind"]) for pair in reference_collisions)
                rows.append(dict(sample_id=str(samples["sample_ids"][i]), group=str(samples["groups"][i]), configuration=q, within_joint_limits=bool(np.all(q >= samples["lower"]) and np.all(q <= samples["upper"])), classification=classification, sphere_collision=sc, reference_collision=rc, sphere_minimum_distance=float(np.min(s)), reference_minimum_distance=float(np.min(r)), sphere_deepest_pair=spheres.pair_metadata[int(np.argmin(s))], reference_deepest_pair=reference.pair_metadata[int(np.argmin(r))], sphere_collisions=collisions(s, spheres.pair_metadata), reference_collisions=reference_collisions, body_pair_disagreements=pair_differences, exact_primitive_false_negative=exact_miss, reference_near_contact=bool(np.any(abs(r) <= 1e-8)), sphere_near_contact=bool(np.any(abs(s) <= 1e-8)), fk_error=pose_error))
            except Exception as error:
                failures.append(dict(sample_id=str(samples["sample_ids"][i]), exception=repr(error), traceback=traceback.format_exc()))
        np.savez_compressed(result_dir / (condition + "_pair_distances.npz"), reference=np.array(r_distances), spheres=np.array(s_distances), configurations=q_values, sample_ids=samples["sample_ids"])
        atomic(result_dir / (condition + "_pair_metadata.json"), dict(reference=reference.pair_metadata, spheres=spheres.pair_metadata))
        atomic(result_dir / (condition + "_rows.json"), rows)
        atomic(result_dir / (condition + "_failures.json"), failures)
        with (result_dir / (condition + "_rows.csv")).open("w") as stream:
            fields = ["sample_id", "group", "within_joint_limits", "classification", "sphere_minimum_distance", "reference_minimum_distance"]
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows({key: row[key] for key in fields} for row in rows)
        results[condition] = dict(requested=len(q_values), evaluated=len(rows), failures=len(failures), summary=summarize_rows(rows), reference_pairs=len(reference.pairs), sphere_pairs=len(spheres.pairs))
        atomic(result_dir / "progress.json", results)
        print(condition, json.dumps(results[condition]), flush=True)
    if sha(asset_path) != asset_identity:
        raise ValueError("Constructed asset changed during validation; retain partial records and rerun")
    atomic(result_dir / "summary.json", dict(status="completed" if all(not row["failures"] for row in results.values()) else "completed_with_failures", conditions=results, interpretation=INTERPRETATION, preregistration_sha256=sha(out / "preregistration.json"), asset_sha256=asset_identity))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("prepare", "validate"))
    parser.add_argument("--output", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--result-dir", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    try:
        if args.action == "prepare":
            prepare(args.output)
        else:
            target = args.result_dir or args.output / ("validation_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()))
            validate(args.output, target)
    except BaseException as error:
        atomic(args.output / ("failure_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + ".json"), dict(command=sys.argv, exception=repr(error), traceback=traceback.format_exc()))
        raise


if __name__ == "__main__":
    main()
