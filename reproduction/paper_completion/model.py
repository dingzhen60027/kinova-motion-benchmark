"""Matched Drake/native FR3 plants and paper Forest, with genuine Drake distance constraints."""

from pathlib import Path
import numpy as np
import csdecomp as csd
from pydrake.all import (
    RobotDiagramBuilder,
    RigidTransform,
    Sphere,
    Box,
    CoulombFriction,
    Role,
    CollisionFilterDeclaration,
    GeometrySet,
    InverseKinematics,
    RotationMatrix,
    SnoptSolver,
)
from pydrake.multibody.inverse_kinematics import MinimumDistanceLowerBoundConstraint
from reproduction.common import ROOT, atomic, read
from reproduction.forest import Forest

ASSETS = Path(__file__).parent / "assets"


def fr3_urdf():
    asset = read(ASSETS / "fr3_33.json")
    text = ['<robot name="fr3">']
    for name in [f"fr3_link{i}" for i in range(9)] + [
        "fr3_hand",
        "fr3_leftfinger",
        "fr3_rightfinger",
        "fr3_hand_tcp",
    ]:
        text.append(f'<link name="{name}">')
        for i, ball in enumerate(asset["spheres"].get(name, [])):
            xyz = " ".join(map(str, ball["center"]))
            text.append(
                f'<collision name="{name}_sphere_{i}"><origin xyz="{xyz}"/><geometry><sphere radius="{ball["radius"]}"/></geometry></collision>'
            )
        text.append("</link>")
    text.append(
        '<joint name="base_weld" type="fixed"><parent link="world"/><child link="fr3_link0"/></joint>'
    )
    for i in range(1, 9):
        origin = asset["kinematics"][f"joint{i}"]["kinematic"]
        xyz = " ".join(str(origin[x]) for x in ["x", "y", "z"])
        rpy = " ".join(str(origin[x]) for x in ["roll", "pitch", "yaw"])
        kind = "revolute" if i < 8 else "fixed"
        limit = asset["limits"][f"joint{i}"]["limit"] if i < 8 else None
        text.append(
            f'<joint name="fr3_joint{i}" type="{kind}"><parent link="fr3_link{i-1}"/><child link="fr3_link{i}"/><origin xyz="{xyz}" rpy="{rpy}"/>'
        )
        if limit:
            text.append(
                f'<axis xyz="0 0 1"/><limit lower="{limit["lower"]}" upper="{limit["upper"]}" velocity="{limit["velocity"]}" effort="{limit["effort"]}"/>'
            )
        text.append("</joint>")
    text.append(
        f'<joint name="hand_weld" type="fixed"><parent link="fr3_link8"/><child link="fr3_hand"/><origin rpy="0 0 {-np.pi/4}"/></joint>'
    )
    for name, xyz, yaw in [
        ("fr3_leftfinger", "0 0.04 0.0584", 0),
        ("fr3_rightfinger", "0 -0.04 0.0584", np.pi),
        ("fr3_hand_tcp", "0 0 0.1034", 0),
    ]:
        text.append(
            f'<joint name="{name}_weld" type="fixed"><parent link="fr3_hand"/><child link="{name}"/><origin xyz="{xyz}" rpy="0 0 {yaw}"/></joint>'
        )
    return "".join(text) + "</robot>"


def drake_fr3(cfg, obstacles=None):
    builder = RobotDiagramBuilder()
    plant, sg = builder.plant(), builder.scene_graph()
    scene = cfg["fr3_scene"]
    # Keep world itself free of geometry: the released native bridge requires
    # every geometry-bearing link to have a parent joint, including static links.
    extra = f'<link name="table"><collision name="table"><geometry><box size="{" ".join(map(str,scene["table_size"]))}"/></geometry></collision></link><joint name="table_weld" type="fixed"><parent link="world"/><child link="table"/><origin xyz="{" ".join(map(str,scene["table_center"]))}"/></joint>'
    for i, center in enumerate([] if obstacles is None else obstacles):
        extra += f'<link name="obstacle_{i}"><collision name="obstacle_{i}"><geometry><sphere radius="{scene["obstacle_radius"]}"/></geometry></collision></link><joint name="obstacle_{i}_weld" type="fixed"><parent link="world"/><child link="obstacle_{i}"/><origin xyz="{" ".join(map(str,center))}"/></joint>'
    instance = builder.parser().AddModelsFromString(
        fr3_urdf().replace("</robot>", extra + "</robot>"), "urdf"
    )[0]
    plant.Finalize()
    # Use the official robot's SRDF filters, with no outcome-based extra exclusions.
    for first, second in read(ASSETS / "fr3_33.json")["disabled_collision_links"]:
        ids = []
        for name in [first, second]:
            body = plant.GetBodyByName(name, instance)
            ids.append(GeometrySet(plant.GetCollisionGeometriesForBody(body)))
        sg.collision_filter_manager().Apply(
            CollisionFilterDeclaration().ExcludeBetween(*ids)
        )
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    pc = plant.GetMyMutableContextFromRoot(context)
    sc = sg.GetMyContextFromRoot(context)
    inspector = sg.get_query_output_port().Eval(sc).inspector()
    return diagram, context, plant, pc, sg, inspector, instance


def matched_native(parts):
    diagram, context, plant, pc, sg, inspector, instance = parts
    converted = csd.convert_drake_plant_to_csd_plant(plant, pc, inspector)
    # The released bridge rebuilds default filters. Restore exactly the explicit
    # Drake candidate pairs in both the full native Plant and its MinimalPlant.
    ids = [
        g
        for frame in inspector.GetAllFrameIds()
        for g in inspector.GetGeometries(frame, Role.kProximity)
    ]
    assert len(ids) == len(converted.getSceneCollisionGeometries())
    lookup = {g: i for i, g in enumerate(ids)}
    pairs = sorted(
        (min(lookup[a], lookup[b]), max(lookup[a], lookup[b]))
        for a, b in inspector.GetCollisionCandidates()
    )
    si = converted.getSceneInspector()
    si.robot_to_world_collision_pairs = pairs
    native = csd.Plant(
        converted.getKinematicTree(), si, converted.getSceneCollisionGeometries()
    )
    assert native.getMinimalPlant().num_collision_pairs == len(pairs)
    return native


class FR3:
    kind = "fr3"
    dimension = 7

    def __init__(self, protocol, centers=None):
        self.protocol = protocol
        self.parts = drake_fr3(protocol, centers)
        (
            self.diagram,
            self.context,
            self.drake,
            self.pc,
            self.sg,
            self.inspector,
            self.instance,
        ) = self.parts
        self.native = (
            matched_native(drake_fr3(protocol))
            if centers is not None
            else matched_native(self.parts)
        )
        self.minimal = self.native.getMinimalPlant()
        self.geometry_ids = self.native.getRobotGeometryIds()
        self.lower = self.drake.GetPositionLowerLimits()
        self.upper = self.drake.GetPositionUpperLimits()
        self.domain = csd.HPolyhedron()
        self.domain.MakeBox(self.lower, self.upper)
        self.voxels = csd.Voxels(
            np.empty((3, 0), np.float32)
            if centers is None
            else np.asarray(centers, np.float32).T
        )
        self.voxel_radius = protocol["fr3_scene"]["obstacle_radius"]
        self.cfg = dict(
            protocol,
            eizo=protocol["eizo"]["fr3"],
            domain_lower=self.lower.tolist(),
            domain_upper=self.upper.tolist(),
        )
        self.tcp = self.drake.GetFrameByName("fr3_hand_tcp", self.instance)

    def cuda(self, points):
        return np.asarray(
            csd.CheckCollisionFreeVoxelsCuda(
                np.asarray(points, np.float32).T,
                self.voxels,
                self.voxel_radius,
                self.minimal,
                self.geometry_ids,
            ),
            bool,
        )

    def independent(self, points):
        values = []
        for q in points:
            self.drake.SetPositions(self.pc, q)
            query = self.sg.get_query_output_port().Eval(
                self.sg.GetMyContextFromRoot(self.context)
            )
            distances = query.ComputeSignedDistancePairwiseClosestPoints(0.0)
            values.append(not any(x.distance < 0.0 for x in distances))
        return np.asarray(values, bool)

    def constraint(self):
        return MinimumDistanceLowerBoundConstraint(
            self.drake,
            self.protocol["nto_collision_bound"],
            self.pc,
            influence_distance_offset=self.protocol["nto_influence_distance"],
        )

    def pose(self, q):
        self.drake.SetPositions(self.pc, q)
        return self.drake.CalcRelativeTransform(
            self.pc, self.drake.world_frame(), self.tcp
        ).GetAsMatrix4()

    def ik(self, target, initial):
        ik = InverseKinematics(self.drake, self.pc)
        ik.AddPositionConstraint(
            self.tcp,
            np.zeros(3),
            self.drake.world_frame(),
            target[:3, 3] - 1e-5,
            target[:3, 3] + 1e-5,
        )
        ik.AddOrientationConstraint(
            self.drake.world_frame(),
            RotationMatrix(target[:3, :3]),
            self.tcp,
            RotationMatrix(),
            1e-4,
        )
        ik.AddMinimumDistanceLowerBoundConstraint(
            self.protocol.get("endpoint_collision_margin", 1e-5),
            self.protocol["nto_influence_distance"],
        )
        ik.prog().AddQuadraticErrorCost(np.eye(7), initial, ik.q())
        ik.prog().SetInitialGuess(ik.q(), initial)
        result = SnoptSolver().Solve(ik.prog())
        return (
            result.is_success(),
            result.GetSolution(ik.q()),
            str(result.get_solution_result()),
        )


class ForestEnvironment:
    kind = "forest"
    dimension = 2

    def __init__(self, protocol, seed):
        from reproduction.common import config

        cfg = config()
        cfg.update({k: v for k, v in protocol.items() if k in cfg})
        cfg["eizo"] = protocol["eizo"]["forest"]
        self.base = Forest(cfg, seed)
        self.cfg = cfg
        self.protocol = protocol
        self.lower = np.array(cfg["domain_lower"])
        self.upper = np.array(cfg["domain_upper"])
        # Offline DRM contains only the moving point. Obstacles enter online;
        # the full Drake model remains available for independent/NTO checks.
        from reproduction.forest import urdf
        parser = csd.URDFParser()
        if not parser.parse_urdf_string(urdf(cfg, [])):
            raise ValueError("Empty-workspace Forest URDF rejected")
        self.native = parser.build_plant()
        self.minimal = self.native.getMinimalPlant()
        self.geometry_ids = self.native.getRobotGeometryIds()
        self.domain = self.base.domain
        self.voxels = csd.Voxels(np.vstack([
            self.base.centers.T, np.zeros(len(self.base.centers))
        ]).astype(np.float32))
        self.voxel_radius = cfg["obstacle_radius"]
        builder = RobotDiagramBuilder()
        self.drake = builder.plant()
        self.sg = builder.scene_graph()
        builder.parser().AddModelsFromString(self.base.source, "urdf")
        self.drake.Finalize()
        self.diagram = builder.Build()
        self.context = self.diagram.CreateDefaultContext()
        self.pc = self.drake.GetMyMutableContextFromRoot(self.context)

    def cuda(self, q):
        return np.asarray(
            csd.CheckCollisionFreeVoxelsCuda(
                np.asarray(q, np.float32).T, self.voxels,
                self.voxel_radius, self.minimal, self.geometry_ids
            ), bool
        )

    def independent(self, q):
        return self.base.clearances(q) >= 0

    def constraint(self):
        return MinimumDistanceLowerBoundConstraint(
            self.drake,
            self.protocol["nto_collision_bound"],
            self.pc,
            influence_distance_offset=self.protocol["nto_influence_distance"],
        )
