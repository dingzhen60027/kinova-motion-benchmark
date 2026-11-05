"""MuJoCo dynamics model exported from the benchmark's canonical Drake plant.

The original assets remain read-only. Illustration meshes have no contacts;
canonical primitive boxes/spheres and the frozen voxel spheres carry contacts.
The explicit MuJoCo pair list is exactly the benchmark collision pair list.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from ..model import ASSETS, Model, ROOT, digest, sha256


def _text(values):
    return " ".join(format(float(x), ".17g") for x in np.asarray(values).reshape(-1))


def _pose(transform):
    return dict(pos=_text(transform.translation()),
                quat=_text(transform.rotation().ToQuaternion().wxyz()))


def _write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def _configuration(config):
    config = dict(config or {})
    timestep = float(config.get("timestep_s", config.get("timestep", .002)))
    gravity = np.asarray(config.get("gravity_m_s2", [0., 0., -9.81]), dtype=float)
    if not np.isfinite(timestep) or not 0 < timestep <= .01:
        raise ValueError("MuJoCo timestep_s must be finite and in (0, 0.01].")
    if gravity.shape != (3,) or not np.isfinite(gravity).all():
        raise ValueError("MuJoCo gravity_m_s2 must be three finite values.")
    return dict(timestep_s=timestep, gravity_m_s2=gravity.tolist(),
                integrator="implicitfast", friction=[.5, .5, .005, .0001, .0001],
                contact_solref=[.02, 1.], contact_margin_m=0.,
                joint_damping_nm_s=0., joint_armature_kg_m2=0.)


def build(case, output_dir, config=None):
    """Export and compile one existing case; return JSON-serializable metadata.

    ``case`` is an original benchmark manifest row (or a scene with no endpoints).
    ``qpos_ids`` and ``dof_ids`` map the fixed benchmark seven-joint order to
    MuJoCo; ``actuator_ids`` and torque limits use this same order. The returned
    ``xml_path`` is absolute, while mesh references within XML are relative to
    the generated XML and point to the original read-only repository assets.
    """
    import mujoco
    from pydrake.all import Box, Sphere, Cylinder, Mesh, RotationMatrix
    from pydrake.multibody.tree import JointIndex

    out = Path(output_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    settings = _configuration(config)
    xyz = case.get("voxels_xyz", [])
    radius = case.get("voxel_radius_m", .04)
    canonical = Model(xyz, radius)
    for key, actual in (("robot_sha256", canonical.robot_sha256),
                        ("collision_geometry_sha256", canonical.collision_geometry_sha256)):
        if key in case and case[key] != actual:
            raise ValueError(f"Case {key} differs from the canonical robot.")
    plant = canonical.plant
    plant.SetPositions(canonical.plant_context, np.zeros(7))

    doc = ET.Element("mujoco", model="kinova_motion_benchmark")
    ET.SubElement(doc, "compiler", angle="radian", inertiafromgeom="false",
                  fusestatic="false", autolimits="true", strippath="false")
    ET.SubElement(doc, "option", timestep=_text([settings["timestep_s"]]),
                  gravity=_text(settings["gravity_m_s2"]), integrator=settings["integrator"],
                  cone="elliptic", iterations="100", tolerance="1e-10")
    ET.SubElement(doc, "size", memory="128M")
    visual = ET.SubElement(doc, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="720")
    ET.SubElement(visual, "quality", offsamples="4", shadowsize="2048")
    ET.SubElement(visual, "headlight", ambient="0.35 0.35 0.35", diffuse="0.65 0.65 0.65")
    defaults = ET.SubElement(doc, "default")
    ET.SubElement(defaults, "geom", contype="0", conaffinity="0", margin="0", gap="0")
    ET.SubElement(defaults, "joint", damping="0", armature="0", frictionloss="0")
    asset = ET.SubElement(doc, "asset")
    ET.SubElement(asset, "texture", name="neutral_sky", type="skybox", builtin="gradient",
                  rgb1="0.80 0.84 0.88", rgb2="0.96 0.97 0.98", width="512", height="3072")
    world = ET.SubElement(doc, "worldbody")
    ET.SubElement(world, "light", name="key_light", pos="1 -1 3", dir="-0.3 0.3 -1", directional="true")
    ET.SubElement(world, "camera", name="overview", pos="2.2 -2.4 1.9", xyaxes="0.737 0.676 0 -0.307 0.335 0.891")
    ET.SubElement(world, "geom", name="visual_floor", type="plane", size="3 3 .05",
                  pos="0 0 -0.73", rgba="0.86 0.88 0.90 1", group="2")

    body_xml = {0: world}
    body_names = {}
    pending = {int(plant.get_joint(JointIndex(i)).child_body().index()): plant.get_joint(JointIndex(i))
               for i in range(plant.num_joints())}
    joints = {}
    inertia_sources = []
    while pending:
        progress = False
        for child, joint in list(pending.items()):
            parent = int(joint.parent_body().index())
            if parent not in body_xml:
                continue
            body = joint.child_body()
            name = f"body_{child}_{plant.GetModelInstanceName(body.model_instance())}_{body.name()}"
            X_PC = plant.CalcRelativeTransform(canonical.plant_context,
                                              joint.parent_body().body_frame(), body.body_frame())
            node = ET.SubElement(body_xml[parent], "body", name=name, **_pose(X_PC))
            body_xml[child] = node
            body_names[child] = name
            spatial = body.CalcSpatialInertiaInBodyFrame(canonical.plant_context)
            mass = float(spatial.get_mass())
            if mass > 0:
                com = spatial.get_com()
                central = spatial.Shift(com).CalcRotationalInertia().CopyToFullMatrix3()
                eigenvalues, eigenvectors = np.linalg.eigh(central)
                if np.linalg.det(eigenvectors) < 0:
                    eigenvectors[:, 0] *= -1
                if np.any(eigenvalues <= 0):
                    raise ValueError(f"Source body {name} has nonpositive central inertia.")
                # Explicit principal axes avoid MuJoCo's approximate fullinertia eigensolver.
                # This is the same source central tensor, reconstructed to float64 precision.
                ET.SubElement(node, "inertial", mass=_text([mass]), pos=_text(com),
                              diaginertia=_text(eigenvalues),
                              quat=_text(RotationMatrix(eigenvectors).ToQuaternion().wxyz()))
                inertia_sources.append(dict(drake_body_index=child, body=name, mass_kg=mass,
                                             com_m=com.tolist(), central_inertia_kg_m2=central.tolist()))
            if joint.num_positions():
                if joint.type_name() != "revolute" or joint.num_positions() != 1:
                    raise ValueError("Canonical MuJoCo exporter supports fixed and revolute joints only.")
                order = joint.position_start()
                X_CJ = joint.frame_on_child().GetFixedPoseInBodyFrame()
                name_joint = f"kinova_joint_{order + 1}"
                axis = X_CJ.rotation().matrix() @ joint.revolute_axis()
                ET.SubElement(node, "joint", name=name_joint, type="hinge", axis=_text(axis),
                              pos=_text(X_CJ.translation()), limited="true",
                              range=_text([canonical.lower[order], canonical.upper[order]]))
                joints[order] = name_joint
            del pending[child]
            progress = True
        if not progress:
            raise ValueError("Canonical plant body graph cannot be traversed from world.")

    canonical_geom_names = []
    for i, gid in enumerate(canonical._drake_ids):
        body = plant.GetBodyFromFrameId(canonical.inspector.GetFrameId(gid))
        node = body_xml[int(body.index())]
        name = f"canonical_collision_{i:03d}"
        shape = canonical.inspector.GetShape(gid)
        args = dict(name=name, group="3", rgba="0.12 0.45 0.8 .22",
                    **_pose(canonical.inspector.GetPoseInFrame(gid)))
        if isinstance(shape, Box):
            args.update(type="box", size=_text(shape.size() / 2))
        elif isinstance(shape, Sphere):
            args.update(type="sphere", size=_text([shape.radius()]))
        else:
            raise ValueError(f"Unexpected canonical collision shape {type(shape).__name__}.")
        ET.SubElement(node, "geom", **args)
        canonical_geom_names.append(name)

    mesh_assets = {}
    mesh_sources = {}
    visual_count = 0
    illustrated_bodies = set()
    for gid in canonical.inspector.GetAllGeometryIds():
        props = canonical.inspector.GetIllustrationProperties(gid)
        if props is None:
            continue
        body = plant.GetBodyFromFrameId(canonical.inspector.GetFrameId(gid))
        node = body_xml[int(body.index())]
        illustrated_bodies.add(int(body.index()))
        shape = canonical.inspector.GetShape(gid)
        rgba = [0.65, 0.68, 0.72, 1.]
        if props.HasProperty("phong", "diffuse"):
            rgba = np.asarray(props.GetProperty("phong", "diffuse").rgba).tolist()
        args = dict(name=f"visual_{visual_count:03d}", group="2", rgba=_text(rgba),
                    **_pose(canonical.inspector.GetPoseInFrame(gid)))
        if isinstance(shape, Mesh):
            source = Path(shape.source().path()).resolve()
            if source.suffix.lower() not in (".obj", ".stl", ".msh"):
                raise ValueError(f"MuJoCo visual mesh conversion required for {source.suffix}.")
            scale = np.asarray(shape.scale3()) if hasattr(shape, "scale3") else np.repeat(shape.scale(), 3)
            key = (str(source), tuple(scale))
            if key not in mesh_assets:
                mesh_name = f"mesh_{len(mesh_assets):03d}"
                ET.SubElement(asset, "mesh", name=mesh_name,
                              file=os.path.relpath(source, out), scale=_text(scale))
                mesh_assets[key] = mesh_name
                mesh_sources[str(source.relative_to(ROOT))] = sha256(source)
            args.update(type="mesh", mesh=mesh_assets[key])
        elif isinstance(shape, Box):
            args.update(type="box", size=_text(shape.size() / 2))
        elif isinstance(shape, Sphere):
            args.update(type="sphere", size=_text([shape.radius()]))
        elif isinstance(shape, Cylinder):
            args.update(type="cylinder", size=_text([shape.radius(), shape.length() / 2]))
        else:
            raise ValueError(f"Unhandled canonical visual shape {type(shape).__name__}.")
        ET.SubElement(node, "geom", **args)
        visual_count += 1

    # A fixed object with no illustration must still be visible to the user.
    # These are display-only copies of the original boxes, never contact geoms.
    fixed_collision_visuals = []
    for i, gid in enumerate(canonical._drake_ids):
        body = plant.GetBodyFromFrameId(canonical.inspector.GetFrameId(gid))
        body_index = int(body.index())
        if i in canonical.robot_geometry_ids or body_index in illustrated_bodies:
            continue
        shape = canonical.inspector.GetShape(gid)
        args = dict(name=f"fixed_collision_visual_{i:03d}", group="2",
                    rgba="0.40 0.44 0.48 0.65", contype="0", conaffinity="0",
                    **_pose(canonical.inspector.GetPoseInFrame(gid)))
        if isinstance(shape, Box):
            args.update(type="box", size=_text(shape.size() / 2))
        elif isinstance(shape, Sphere):
            args.update(type="sphere", size=_text([shape.radius()]))
        else:
            raise ValueError("Fixed visual fallback requires canonical box or sphere.")
        ET.SubElement(body_xml[body_index], "geom", **args)
        fixed_collision_visuals.append(dict(name=args["name"], canonical_geom_index=i,
                                            drake_name=canonical.inspector.GetName(gid),
                                            display_only=True))

    for i, obj in enumerate(case.get("obstacle_geometry", [])):
        args = dict(name=f"obstacle_visual_{i:04d}", pos=_text(obj["center_m"]),
                    group="2", rgba="0.91 0.37 0.11 1")
        if obj["shape"] == "box":
            yaw = float(obj.get("yaw_rad", 0.))
            args.update(type="box", size=_text(np.asarray(obj["size_m"]) / 2),
                        quat=_text([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]))
        elif obj["shape"] == "sphere":
            args.update(type="sphere", size=_text([obj["radius_m"]]))
        else:
            raise ValueError("Frozen obstacle shape must be box or sphere.")
        ET.SubElement(world, "geom", **args)
    voxel_names = []
    for i, centre in enumerate(canonical.voxels_xyz):
        name = f"voxel_collision_{i:05d}"
        ET.SubElement(world, "geom", name=name, type="sphere", pos=_text(centre),
                      size=_text([canonical.voxel_radius]), group="4", rgba="0.93 0.5 0.1 .16")
        voxel_names.append(name)

    all_geom_names = canonical_geom_names + voxel_names
    pair_indices = canonical.pairs.tolist()
    pair_indices.extend([int(r), len(canonical_geom_names) + v]
                        for r in canonical.robot_geometry_ids for v in range(len(voxel_names)))
    contact = ET.SubElement(doc, "contact")
    for i, (a, b) in enumerate(pair_indices):
        ET.SubElement(contact, "pair", name=f"canonical_pair_{i:06d}",
                      geom1=all_geom_names[a], geom2=all_geom_names[b],
                      condim="3", friction=_text(settings["friction"]),
                      solref=_text(settings["contact_solref"]), margin="0", gap="0")
    limits = {}
    urdf = ET.parse(ASSETS / "kinova_sens.urdf").getroot()
    for node in urdf.findall("joint"):
        element = node.find("limit")
        if node.attrib.get("type") in ("revolute", "continuous") and element is not None:
            effort = float(element.attrib["effort"])
            if not np.isfinite(effort) or effort <= 0:
                raise ValueError("Canonical URDF torque limits must be positive.")
            limits[node.attrib["name"]] = effort
    actuator = ET.SubElement(doc, "actuator")
    efforts, actuator_names = [], []
    for order in range(7):
        name = joints[order]
        effort = limits[f"joint_{order + 1}"]
        actuator_name = f"torque_{name}"
        ET.SubElement(actuator, "motor", name=actuator_name, joint=name, gear="1",
                      ctrllimited="true", ctrlrange=_text([-effort, effort]),
                      forcelimited="true", forcerange=_text([-effort, effort]))
        efforts.append(effort)
        actuator_names.append(actuator_name)
    ET.indent(doc, space="  ")
    xml_path = out / "model.xml"
    ET.ElementTree(doc).write(xml_path, encoding="unicode", xml_declaration=True)
    mjmodel = mujoco.MjModel.from_xml_path(str(xml_path))
    joint_names = [joints[i] for i in range(7)]
    joint_ids = [mujoco.mj_name2id(mjmodel, mujoco.mjtObj.mjOBJ_JOINT, n) for n in joint_names]
    ids = lambda names, kind: [int(mujoco.mj_name2id(mjmodel, kind, n)) for n in names]
    if mjmodel.nq != 7 or mjmodel.nv != 7 or mjmodel.nu != 7:
        raise ValueError("MuJoCo compiled model must preserve seven positions, velocities and torque motors.")
    tcp_body_index = int(canonical._tcp_body.index())
    metadata = dict(
        schema="kinova_motion_mujoco_v1", xml_path=str(xml_path), xml_sha256=sha256(xml_path),
        metadata_path=str(out / "model_metadata.json"), problem_id=case.get("problem_id"),
        scene_id=case.get("scene_id"), problem_sha256=case.get("problem_sha256"),
        robot_sha256=canonical.robot_sha256, collision_geometry_sha256=canonical.collision_geometry_sha256,
        voxel_geometry_sha256=digest(dict(xyz=canonical.voxels_xyz.tolist(), radius_m=canonical.voxel_radius)),
        canonical_source_files_sha256=canonical.sources, visual_mesh_files_sha256=mesh_sources,
        settings=settings, mujoco_version=mujoco.__version__,
        official_documentation=["https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#body-inertial",
                                "https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#contact-pair",
                                "https://mujoco.readthedocs.io/en/3.3.7/computation/index.html#collision-detection"],
        benchmark_joint_names=canonical.joint_names, joint_names=joint_names, joint_ids=joint_ids,
        qpos_ids=mjmodel.jnt_qposadr[joint_ids].astype(int).tolist(),
        dof_ids=mjmodel.jnt_dofadr[joint_ids].astype(int).tolist(),
        actuator_ids=ids(actuator_names, mujoco.mjtObj.mjOBJ_ACTUATOR),
        effort_limits_nm=efforts, effort_limit_source="Read verbatim from canonical kinova_sens.urdf; not manufacturer calibration.",
        lower=canonical.lower.tolist(), upper=canonical.upper.tolist(),
        tcp_body_id=int(mujoco.mj_name2id(mjmodel, mujoco.mjtObj.mjOBJ_BODY, body_names[tcp_body_index])),
        tcp_body_name=body_names[tcp_body_index], tcp_local_transform=np.eye(4).tolist(),
        canonical_geom_ids=ids(canonical_geom_names, mujoco.mjtObj.mjOBJ_GEOM),
        canonical_geom_names=canonical_geom_names,
        canonical_geom_drake_names=[canonical.inspector.GetName(g) for g in canonical._drake_ids],
        robot_canonical_geom_indices=[int(i) for i in canonical.robot_geometry_ids],
        voxel_geom_ids=ids(voxel_names, mujoco.mjtObj.mjOBJ_GEOM),
        pair_indices=pair_indices,
        collision_geom_ids=ids(all_geom_names, mujoco.mjtObj.mjOBJ_GEOM),
        collision_pair_count=len(pair_indices), static_pair_count=len(canonical.pairs),
        visual_geom_count=visual_count + len(fixed_collision_visuals) + len(case.get("obstacle_geometry", [])) + 1,
        fixed_collision_visuals=fixed_collision_visuals,
        inertia_sources=inertia_sources,
        contact_policy="Explicit canonical geometry pairs plus moving-robot versus every frozen voxel sphere. Source obstacle and mesh visuals have no contacts.",
        inertia_policy="Canonical parsed URDF masses/COM/full central inertias; no density inference, mass rescaling, armature or damping.",
        physical_assumptions="Rigid links; fixed gripper; ideal torque motors with source URDF limits; uncalibrated friction/soft contact; no motor electronics, flexibility or external calibration.",
        body_id_map={str(k): int(mujoco.mj_name2id(mjmodel, mujoco.mjtObj.mjOBJ_BODY, n)) for k, n in body_names.items()},
    )
    actual_pairs = sorted(tuple(sorted((int(a), int(b)))) for a, b in zip(mjmodel.pair_geom1, mjmodel.pair_geom2))
    expected_pairs = sorted(tuple(sorted((metadata["collision_geom_ids"][a], metadata["collision_geom_ids"][b]))) for a, b in pair_indices)
    if actual_pairs != expected_pairs:
        raise ValueError("MuJoCo compiler changed the canonical collision pair identities.")
    if np.any(mjmodel.geom_contype) or np.any(mjmodel.geom_conaffinity):
        raise ValueError("Unexpected dynamic collision-mask pairs in MuJoCo model.")
    _write_json(out / "model_metadata.json", metadata)
    return metadata


def pair_distances(model, data, metadata, maximum_distance=.1):
    """Raw MuJoCo distances, clipped at cutoff; use Drake for acceptance.

    MuJoCo 3.3.7 nativeCCD can report zero for separated convex pairs (upstream
    issue #3383). These diagnostic values must never replace canonical validation.
    The small cutoff reduces exposure to distant-pair degeneracy.
    """
    import mujoco
    ids = metadata["collision_geom_ids"]
    return np.asarray([mujoco.mj_geomDistance(model, data, ids[a], ids[b], maximum_distance, None)
                       for a, b in metadata["pair_indices"]], dtype=float)


def cross_check(case, metadata, configurations, output_path=None):
    """Numerically compare FK, inertia, limits and collision sign with Drake.

    Positive primitive separation, capped at 0.1 m, should agree to 1e-7 m. Deep overlap signed
    penetration depths are engine dependent; their signs are compared outside
    the benchmark's two-micrometre contact ambiguity band.
    """
    import mujoco
    q = np.asarray(configurations, dtype=float)
    if q.ndim != 2 or q.shape[1] != 7 or not np.isfinite(q).all():
        raise ValueError("cross_check requires finite N x 7 configurations.")
    canonical = Model(case.get("voxels_xyz", []), case.get("voxel_radius_m", .04))
    model = mujoco.MjModel.from_xml_path(metadata["xml_path"])
    data = mujoco.MjData(model)
    pose_error = rotation_error = mass_matrix_error = gravity_error = 0.
    drake_clearances, mujoco_clearances, collisions, contact_mismatches = [], [], [], []
    positive_distance_errors = []
    native_pair_contacts_outside_protocol = set()
    allowed = {tuple(sorted((metadata["collision_geom_ids"][a], metadata["collision_geom_ids"][b])))
               for a, b in metadata["pair_indices"]}
    for configuration in q:
        data.qpos[metadata["qpos_ids"]] = configuration
        data.qvel[:] = 0.
        mujoco.mj_forward(model, data)
        canonical.plant.SetPositions(canonical.plant_context, configuration)
        query = canonical._query()
        for index, gid in enumerate(canonical._drake_ids):
            X = query.GetPoseInWorld(gid)
            mjid = metadata["canonical_geom_ids"][index]
            pose_error = max(pose_error, float(np.max(np.abs(X.translation() - data.geom_xpos[mjid]))))
            rotation_error = max(rotation_error, float(np.max(np.abs(X.rotation().matrix() - data.geom_xmat[mjid].reshape(3, 3)))))
        matrix = np.zeros((model.nv, model.nv))
        mujoco.mj_fullM(model, matrix, data.qM)
        reordered = matrix[np.ix_(metadata["dof_ids"], metadata["dof_ids"])]
        drake_matrix = canonical.plant.CalcMassMatrixViaInverseDynamics(canonical.plant_context)
        mass_matrix_error = max(mass_matrix_error, float(np.max(np.abs(reordered - drake_matrix))))
        dg = canonical.plant.CalcGravityGeneralizedForces(canonical.plant_context)
        # MuJoCo bias includes the compensation sign (qvel=0); Drake gives applied gravity.
        if np.allclose(model.opt.gravity, [0, 0, -9.81], atol=0, rtol=0):
            gravity_error = max(gravity_error, float(np.max(np.abs(data.qfrc_bias[metadata["dof_ids"]] + dg))))
        md = float(np.min(pair_distances(model, data, metadata)))
        dd = float(canonical.clearance(configuration)[0])
        mujoco_clearances.append(md)
        drake_clearances.append(dd)
        if md > canonical.contact_band_m and dd > canonical.contact_band_m:
            positive_distance_errors.append(abs(md - min(dd, .1)))
        if abs(md) > canonical.contact_band_m and abs(dd) > canonical.contact_band_m and (md > 0) != (dd > 0):
            collisions.append(dict(q=configuration.tolist(), drake_m=dd, mujoco_m=md))
        penetrating_contacts = [c for c in data.contact[:data.ncon] if float(c.dist) < -canonical.contact_band_m]
        if abs(dd) > canonical.contact_band_m and bool(penetrating_contacts) != (dd < 0):
            contact_mismatches.append(dict(q=configuration.tolist(), drake_m=dd,
                                           penetrating_contacts=len(penetrating_contacts)))
        for contact in data.contact[:data.ncon]:
            pair = tuple(sorted((int(contact.geom1), int(contact.geom2))))
            if pair not in allowed:
                native_pair_contacts_outside_protocol.add(pair)
    ids = metadata["joint_ids"]
    limits_error = float(np.max(np.abs(model.jnt_range[ids] - np.column_stack((canonical.lower, canonical.upper)))))
    max_positive_error = max(positive_distance_errors, default=0.)
    passed = (pose_error < 1e-10 and rotation_error < 1e-10 and mass_matrix_error < 1e-9
              and gravity_error < 1e-8 and limits_error < 1e-12 and max_positive_error < 1e-7
              and not collisions and not contact_mismatches and not native_pair_contacts_outside_protocol)
    result = dict(passed=bool(passed), configurations=len(q),
                  geometry_position_max_abs_error_m=pose_error, geometry_rotation_max_abs_error=rotation_error,
                  mass_matrix_max_abs_error_kg_m2=mass_matrix_error, gravity_compensation_max_abs_error_nm=gravity_error,
                  joint_limits_max_abs_error_rad=limits_error, positive_clearance_max_abs_error_m=max_positive_error,
                  collision_sign_mismatches=collisions, contact_sign_mismatches=contact_mismatches,
                  distance_cutoff_m=.1,
                  distance_api_caution="MuJoCo 3.3.7 nativeCCD raw distance can report zero for far separated convex pairs; issue #3383. Collision acceptance uses canonical Drake, and actual physics contacts are cross-checked separately.",
                  unexpected_contact_pairs=[list(x) for x in sorted(native_pair_contacts_outside_protocol)],
                  drake_min_clearance_m=drake_clearances, mujoco_min_clearance_m=mujoco_clearances,
                  explicit_pair_count=metadata["collision_pair_count"],
                  collision_comparison="Numerical pair-sign comparison outside 2 um; engine penetration depth is not assumed identical.",
                  canonical_sources_sha256=canonical.sources, model_xml_sha256=sha256(metadata["xml_path"]))
    if output_path is not None:
        _write_json(output_path, result)
    return result
