"""Audit a public iiwa + WSG assembly; this is not the paper's hardware asset.

The downloaded author URDFs provide 8 + 3 box geometries. The gripper weld
and optional table are explicit reconstruction assumptions. No hardware IO.
Run via ``local/python.sh -m reproduction.probe_public_iiwa``.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import urllib.request

import csdecomp as csd
import numpy as np
from pydrake.all import RobotDiagramBuilder, Role

from reproduction.paper_protocol.model import matched_native

BASE = Path(__file__).resolve().parent / "reports/paper_completion/source_evidence"
SOURCES = [
    ("iiwa7_box_collision.urdf", "https://raw.githubusercontent.com/wernerpe/iiwa_description/967cf36204325302bf6726c5c6e65a70f87d6a04/urdf_output/iiwa7_box_collision.urdf", "49683af3d03522be81795cb1a1b23e04c2cbdd9595f9a02d8eb4c6ca59cfe7c1"),
    ("wsg50_110_welded_fingers_box_collision.urdf", "https://raw.githubusercontent.com/wernerpe/schunk_wsg50_model/7b3f5615bbaab066d29803e9386db0ddf1fd92c0/urdf/wsg50_110_welded_fingers_box_collision.urdf", "cae1db4f5219acae7e6e941ae019f0822d42ddeccdf9602cfecf95fc06a5fd19"),
]


def fetch_assets(asset_dir):
    asset_dir.mkdir(parents=True, exist_ok=True)
    for name, url, expected in SOURCES:
        path = asset_dir / name
        content = path.read_bytes() if path.exists() else urllib.request.urlopen(url, timeout=30).read()
        if hashlib.sha256(content).hexdigest() != expected:
            raise ValueError(f"Source hash mismatch: {path}")
        if not path.exists():
            path.write_bytes(content)


def assemble(asset_dir, table):
    arm = ET.parse(asset_dir / "iiwa7_box_collision.urdf").getroot()
    gripper = ET.parse(asset_dir / "wsg50_110_welded_fingers_box_collision.urdf").getroot()
    arm.set("name", "public_iiwa7_wsg_probe")
    for item in list(gripper):
        arm.append(copy.deepcopy(item))
    for link in arm.findall("link"):
        for visual in link.findall("visual"):
            link.remove(visual)
        for i, collision in enumerate(link.findall("collision")):
            collision.set("name", f"{link.attrib['name']}_collision_{i}")
    arm.append(ET.fromstring('<joint name="base_weld" type="fixed"><parent link="world"/><child link="iiwa_link_0"/></joint>'))
    # The author URDF's fingers extend along -y, unlike Drake's WSG frame.
    # Rx(-pi/2) maps -y to the arm flange's +z. The 9 cm weld translation
    # is a declared reconstruction choice, not an author calibration.
    arm.append(ET.fromstring('<joint name="wsg_weld" type="fixed"><parent link="iiwa_link_7"/><child link="wsg50_gripper_link"/><origin xyz="0 0 0.09" rpy="-1.5707963267948966 0 1.5707963267948966"/></joint>'))
    if table:
        arm.append(ET.fromstring('<link name="probe_table"><collision name="probe_table"><geometry><box size="1.4 1.6 0.1"/></geometry></collision></link>'))
        arm.append(ET.fromstring('<joint name="table_weld" type="fixed"><parent link="world"/><child link="probe_table"/><origin xyz="0.5 0 -0.05"/></joint>'))
    return ET.tostring(arm, encoding="unicode")


def check(urdf, out, samples):
    out.mkdir(parents=True, exist_ok=True)
    (out / "assembled.urdf").write_text(urdf)
    builder = RobotDiagramBuilder()
    plant, sg = builder.plant(), builder.scene_graph()
    instance = builder.parser().AddModelsFromString(urdf, "urdf")[0]
    plant.Finalize()
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    pc = plant.GetMyMutableContextFromRoot(context)
    inspector = sg.get_query_output_port().Eval(sg.GetMyContextFromRoot(context)).inspector()
    parts = diagram, context, plant, pc, sg, inspector, instance
    native = matched_native(parts)
    lower, upper = plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits()
    points = np.random.default_rng(250410783).uniform(lower, upper, (samples, 7))
    gpu = np.asarray(csd.CheckCollisionFreeCuda(points.astype(np.float32).T, native.getMinimalPlant()), bool)
    independent, minimum, worst_pairs = [], [], []
    for q in points:
        plant.SetPositions(pc, q)
        distances = sg.get_query_output_port().Eval(sg.GetMyContextFromRoot(context)).ComputeSignedDistancePairwiseClosestPoints()
        d = min((x.distance for x in distances), default=float("inf"))
        independent.append(d >= 0)
        minimum.append(d)
        if distances and len(worst_pairs) < 10:
            closest = min(distances, key=lambda x:x.distance)
            worst_pairs.append(dict(a=inspector.GetName(closest.id_A), b=inspector.GetName(closest.id_B), distance=float(closest.distance)))
    independent = np.asarray(independent)
    tree = native.getKinematicTree()
    fk_errors = []
    for q in points[:30]:
        plant.SetPositions(pc, q)
        native_fk = tree.compute_link_frame_to_world_transforms(q.astype(np.float32))
        for i, link in enumerate(tree.get_links()):
            if link.name == "world":
                continue
            drake_fk = plant.EvalBodyPoseInWorld(pc, plant.GetBodyByName(link.name.split("::")[-1], instance)).GetAsMatrix4()
            fk_errors.append(float(np.max(np.abs(native_fk[i] - drake_fk))))
    pairs = [dict(a=inspector.GetName(a), b=inspector.GetName(b)) for a, b in inspector.GetCollisionCandidates()]
    np.savez_compressed(out / "collision_probe.npz", q=points, cuda_free=gpu, drake_free=independent, minimum_signed_distance=minimum)
    data = dict(positions=plant.num_positions(), geometries=inspector.NumGeometriesWithRole(Role.kProximity), active_native_geometries=len(native.getRobotGeometryIds()), static_candidate_pairs=len(pairs), pair_names=pairs, example_closest_pairs=worst_pairs, random_seed=250410783, samples=samples, cuda_free=int(gpu.sum()), drake_free=int(independent.sum()), disagreements=int(np.count_nonzero(gpu != independent)), fk_test_configurations=min(30,samples), fk_max_abs_native_float32_vs_drake_float64=max(fk_errors), source_asset_recovered=False, paper_hardware_reproduced=False, model_usable_for_planning=bool(gpu.any()))
    (out / "probe.json").write_text(json.dumps(data, indent=2) + "\n")
    if not np.array_equal(gpu, independent):
        raise ValueError("CUDA / Drake collision disagreement; inspect saved arrays")
    if max(fk_errors) > 2e-6:
        raise ValueError("Native / Drake FK differs beyond float32 tolerance")
    return data


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--asset-dir", type=Path, default=BASE / "hardware_assets")
    p.add_argument("--output", type=Path, default=BASE / "hardware_probe")
    p.add_argument("--samples", type=int, default=2000)
    p.add_argument("--fetch-only", action="store_true")
    args = p.parse_args()
    fetch_assets(args.asset_dir)
    if args.fetch_only:
        print(json.dumps([dict(name=n, url=u, sha256=h) for n,u,h in SOURCES], indent=2))
        return
    args.output.mkdir(parents=True, exist_ok=True)
    report = dict(scope="public_model_assembly_verification_only", original_hardware_logs_available=False, robot_box_count=11, gripper_mount=dict(xyz=[0,0,0.09], rpy=[-np.pi/2,0,np.pi/2], status="explicit_URDF_frame_adaptation_not_paper_calibration"), table=dict(size=[1.4,1.6,.1], center=[.5,0,-.05], status="explicit_reconstruction_assumption"), sources=[dict(name=f.name, sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in sorted(args.asset_dir.glob("*.urdf"))], variants={})
    for table in [False, True]:
        name = "robot_only" if not table else "robot_plus_example_table"
        report["variants"][name] = check(assemble(args.asset_dir, table), args.output / name, args.samples)
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k:{a:b for a,b in v.items() if a != "pair_names"} for k,v in report["variants"].items()}, indent=2))


if __name__ == "__main__":
    main()
