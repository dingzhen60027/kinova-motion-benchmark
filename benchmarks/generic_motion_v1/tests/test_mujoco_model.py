"""Real MuJoCo/Drake model and collision contracts, without planning GPU work."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
from benchmarks.generic_motion_v1.model import HERE, sha256
from benchmarks.generic_motion_v1.simulation.mujoco_model import build, cross_check, pair_distances


@pytest.fixture(scope="module")
def case():
    return json.loads((HERE / "manifests/smoke.jsonl").read_text().splitlines()[0])


@pytest.fixture(scope="module")
def exported(case, tmp_path_factory):
    return build(case, tmp_path_factory.mktemp("mujoco_canonical"))


def test_compiled_joint_mapping_effort_and_limits(exported):
    model = mujoco.MjModel.from_xml_path(exported["xml_path"])
    assert (model.nq, model.nv, model.nu) == (7, 7, 7)
    assert exported["benchmark_joint_names"] == [f"kinova_joint_{i}_q" for i in range(1, 8)]
    assert exported["effort_limits_nm"] == [9, 39, 39, 39, 9, 9, 9]
    np.testing.assert_array_equal(model.jnt_range[exported["joint_ids"]],
                                  np.column_stack((exported["lower"], exported["upper"])))
    np.testing.assert_array_equal(model.actuator_ctrlrange[exported["actuator_ids"]],
                                  np.column_stack((-np.asarray(exported["effort_limits_nm"]), exported["effort_limits_nm"])))
    np.testing.assert_array_equal(model.actuator_forcerange, model.actuator_ctrlrange)


def test_explicit_collision_pairs_and_visuals_have_no_extra_contacts(exported, case):
    model = mujoco.MjModel.from_xml_path(exported["xml_path"])
    expected = 137 + 15 * len(case["voxels_xyz"])
    assert model.npair == exported["collision_pair_count"] == expected
    assert exported["static_pair_count"] == 137
    assert not np.any(model.geom_contype)
    assert not np.any(model.geom_conaffinity)
    collision_ids = set(exported["collision_geom_ids"])
    assert all(int(i) in collision_ids for i in model.pair_geom1)
    assert all(int(i) in collision_ids for i in model.pair_geom2)
    assert len(exported["canonical_geom_ids"]) == 20
    assert len(exported["voxel_geom_ids"]) == len(case["voxels_xyz"])
    assert model.ngeom > len(collision_ids)


def test_mesh_sources_are_readonly_relative_references(exported):
    xml = ET.parse(exported["xml_path"]).getroot()
    for mesh in xml.findall("asset/mesh"):
        assert not Path(mesh.attrib["file"]).is_absolute()
        assert (Path(exported["xml_path"]).parent / mesh.attrib["file"]).is_file()
    root = HERE.parents[1]
    assert exported["visual_mesh_files_sha256"]
    for source, before in exported["visual_mesh_files_sha256"].items():
        assert sha256(root / source) == before
    for source, before in exported["canonical_source_files_sha256"].items():
        assert sha256(root / source) == before


def test_original_inertia_has_no_added_damping_or_armature(exported):
    model = mujoco.MjModel.from_xml_path(exported["xml_path"])
    assert not np.any(model.dof_damping)
    assert not np.any(model.dof_armature)
    assert not np.any(model.dof_frictionloss)
    assert len(exported["inertia_sources"]) >= 18  # Includes fixed gripper payload bodies.
    for entry in exported["inertia_sources"]:
        bid = exported["body_id_map"][str(entry["drake_body_index"])]
        assert model.body_mass[bid] == entry["mass_kg"]
        np.testing.assert_allclose(model.body_ipos[bid], entry["com_m"], atol=1e-15, rtol=0)


def test_fk_dynamics_and_real_contact_cross_check(exported, case, tmp_path):
    random = np.random.Generator(np.random.PCG64(343))
    q = np.vstack((case["q_start"], case["q_goal"],
                   random.uniform(exported["lower"], exported["upper"], (64, 7))))
    result = cross_check(case, exported, q, tmp_path / "cross_check.json")
    assert result["passed"], result
    assert result["configurations"] == 66
    assert result["geometry_position_max_abs_error_m"] < 1e-12
    assert result["mass_matrix_max_abs_error_kg_m2"] < 1e-12
    assert result["gravity_compensation_max_abs_error_nm"] < 1e-12
    assert min(result["drake_min_clearance_m"]) < -.01  # Actual collision negative examples.
    assert max(result["drake_min_clearance_m"]) > .001
    assert result["collision_sign_mismatches"] == result["contact_sign_mismatches"] == []
    assert result["unexpected_contact_pairs"] == []


def test_torque_actuator_saturation_is_enforced(exported, case):
    model = mujoco.MjModel.from_xml_path(exported["xml_path"])
    data = mujoco.MjData(model)
    data.qpos[exported["qpos_ids"]] = case["q_start"]
    data.ctrl[exported["actuator_ids"]] = np.asarray(exported["effort_limits_nm"]) * 10
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.actuator_force[exported["actuator_ids"]], exported["effort_limits_nm"], atol=1e-12)


def test_gravity_and_unactuated_motion_are_real(exported, case):
    model = mujoco.MjModel.from_xml_path(exported["xml_path"])
    data = mujoco.MjData(model)
    initial = np.asarray(case["q_start"])
    data.qpos[exported["qpos_ids"]] = initial
    mujoco.mj_forward(model, data)
    for _ in range(20):
        mujoco.mj_step(model, data)
    assert data.time == pytest.approx(.04)
    assert np.max(np.abs(data.qpos[exported["qpos_ids"]] - initial)) > .001
    assert np.isfinite(data.qpos).all()


@pytest.mark.parametrize("config", [{"timestep_s": 0}, {"timestep_s": float("nan")},
                                     {"gravity_m_s2": [0, 0]}, {"gravity_m_s2": [0, 0, float("inf")]}])
def test_invalid_simulation_settings_rejected(case, tmp_path, config):
    with pytest.raises(ValueError):
        build(case, tmp_path, config)


def test_wrong_canonical_identity_rejected(case, tmp_path):
    with pytest.raises(ValueError, match="canonical robot"):
        build(dict(case, robot_sha256="wrong"), tmp_path)


def test_changed_voxel_radius_rejected(case, tmp_path):
    with pytest.raises(ValueError, match="freezes sphere radius"):
        build(dict(case, voxel_radius_m=.039), tmp_path)


def test_crosscheck_bad_configuration_shape_rejected(exported, case):
    with pytest.raises(ValueError, match="N x 7"):
        cross_check(case, exported, np.zeros((2, 6)))


def test_neutral_skybox_and_antialiasing_are_visual_only(exported):
    xml = ET.parse(exported["xml_path"]).getroot()
    sky = xml.find("asset/texture[@type='skybox']")
    assert sky is not None and sky.attrib["builtin"] == "gradient"
    assert xml.find("visual/quality").attrib["offsamples"] == "4"
    model = mujoco.MjModel.from_xml_path(exported["xml_path"])
    assert model.ntex == 1
    assert model.vis.quality.offsamples == 4
    assert not np.any(model.geom_contype)
    assert not np.any(model.geom_conaffinity)


def test_missing_fixed_illustrations_are_visible_without_physics_change(exported):
    xml = ET.parse(exported["xml_path"]).getroot()
    names = {entry["name"] for entry in exported["fixed_collision_visuals"]}
    indices = {entry["canonical_geom_index"] for entry in exported["fixed_collision_visuals"]}
    assert indices == {17, 18, 19}  # Both camera boxes and the fixed protective box.
    assert len(names) == 3
    model = mujoco.MjModel.from_xml_path(exported["xml_path"])
    for entry in exported["fixed_collision_visuals"]:
        fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, entry["name"])
        cid = exported["canonical_geom_ids"][entry["canonical_geom_index"]]
        assert model.geom_group[fid] == 2
        assert model.geom_contype[fid] == model.geom_conaffinity[fid] == 0
        assert model.geom_rgba[fid, 3] < 1
        np.testing.assert_array_equal(model.geom_pos[fid], model.geom_pos[cid])
        np.testing.assert_array_equal(model.geom_quat[fid], model.geom_quat[cid])
        np.testing.assert_array_equal(model.geom_size[fid], model.geom_size[cid])
        assert fid not in model.pair_geom1 and fid not in model.pair_geom2
    for parent in xml.iter():
        for child in list(parent):
            if child.tag == "geom" and child.attrib.get("name") in names:
                parent.remove(child)
    before_path = Path(exported["xml_path"]).parent / "without_fixed_visuals.xml"
    ET.ElementTree(xml).write(before_path, encoding="unicode")
    before = mujoco.MjModel.from_xml_path(str(before_path))
    for field in ("body_mass", "body_ipos", "body_iquat", "body_inertia", "body_pos",
                  "body_quat", "jnt_range", "actuator_ctrlrange", "actuator_forcerange"):
        np.testing.assert_array_equal(getattr(model, field), getattr(before, field))
    # Geom IDs can move; compare the canonical name maps and pair identities.
    for name in exported["canonical_geom_names"]:
        a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        b = mujoco.mj_name2id(before, mujoco.mjtObj.mjOBJ_GEOM, name)
        for field in ("geom_pos", "geom_quat", "geom_size", "geom_type", "geom_bodyid"):
            np.testing.assert_array_equal(getattr(model, field)[a], getattr(before, field)[b])
    def pair_names(m):
        return sorted(tuple(sorted((mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(a)),
                                    mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(b)))))
                      for a, b in zip(m.pair_geom1, m.pair_geom2))
    assert pair_names(model) == pair_names(before)
