"""Additional CPU oracles and invalid-input regression; no CUDA invocation."""
import warnings

import numpy as np
import pytest

from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.reports.qualification.model_audit import ManualURDF, FreshDrake, SphereBoxOracle


@pytest.fixture(scope="module")
def model():
    return Model()


def test_manual_xml_kinematics_matches_separately_loaded_drake_geometry():
    manual, drake = ManualURDF(), FreshDrake()
    for q in (np.zeros(7), np.array([.2, -.7, .4, 1., -.8, .1, .9])):
        expected = manual.fk(q)
        drake.plant.SetPositions(drake.pc, q)
        query = drake.query()
        for name, geometry in manual.geometry.items():
            actual = query.GetPoseInWorld(drake.ids[name]).GetAsMatrix4()
            np.testing.assert_allclose(actual, expected[geometry["link"]] @ geometry["X"], atol=1e-12, rtol=0)


def test_native_pair_oracle_resolves_face_edge_corner_and_interior_signs():
    manual = ManualURDF()
    links = manual.fk(np.array([.2, -.7, .4, 1., -.8, .1, .9]))
    poses = {name: links[g["link"]] @ g["X"] for name, g in manual.geometry.items()}
    oracle = SphereBoxOracle(manual.geometry)
    name = "kinova::half_arm_1_link_collision"
    X, half = poses[name], manual.geometry[name]["dimensions"] / 2
    for direction in ([1., 0., 0.], [1., 1., 0.], [1., 1., 1.]):
        unit = np.array(direction) / np.linalg.norm(direction)
        for distance in (-.005, .005):
            point = np.sign(unit) * half + unit * (.04 + distance)
            world = X[:3, :3] @ point + X[:3, 3]
            measured = oracle.distances(poses, world, [name])[0]
            np.testing.assert_allclose(measured, distance, atol=1e-12, rtol=0)
    interior = oracle.distances(poses, X[:3, 3], [name])[0]
    np.testing.assert_allclose(interior, -np.min(half) - .04, atol=1e-12, rtol=0)


@pytest.mark.parametrize("value", [1e300, -1e300])
def test_finite_voxel_coordinates_that_overflow_float32_are_rejected(model, value):
    model.set_voxels([[.123456789, .2, .3]])
    previous = model.voxels_xyz.copy()
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        with pytest.raises(ValueError, match="finite|representable|float32|overflow"):
            model.set_voxels([[value, 0., 0.]])
    np.testing.assert_array_equal(model.voxels_xyz, previous)


@pytest.mark.parametrize("radius", [np.nan, np.inf, -np.inf, 0., -.04, .039, 1e300, [.04], None, "invalid"])
def test_radius_must_be_finite_scalar_frozen_value(model, radius):
    previous = model.voxels_xyz.copy()
    with pytest.raises(ValueError, match="freezes sphere radius"):
        model.set_voxels([[0., .2, .3]], radius)
    np.testing.assert_array_equal(model.voxels_xyz, previous)


@pytest.mark.parametrize("xyz", [[[np.nan, 0, 0]], [[np.inf, 0, 0]], [1., 2., 3.],
                                [[1., 2.]], np.zeros((2, 6)), np.zeros((1, 1, 3)),
                                [[1., 2., 3.], [1., 2.]], "invalid"])
def test_voxel_shape_and_nonfinite_inputs_rejected_without_state_change(model, xyz):
    previous = model.voxels_xyz.copy()
    with pytest.raises(ValueError, match="finite N x 3"):
        model.set_voxels(xyz)
    np.testing.assert_array_equal(model.voxels_xyz, previous)


def test_unrepresentable_joint_state_rejected_before_any_cuda_call(model, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("Invalid state must not reach CUDA")
    monkeypatch.setattr(model.csd, "CheckCollisionFreeCuda", forbidden)
    monkeypatch.setattr(model.csd, "CheckCollisionFreeVoxelsCuda", forbidden)
    with pytest.raises(ValueError, match="representable as float32"):
        model.cuda(np.full(7, 1e300))


def test_valid_voxel_quantization_and_empty_scene_are_preserved(model):
    xyz = np.array([[.123456789, -.234567891, .345678912]])
    model.set_voxels(xyz)
    np.testing.assert_array_equal(model.voxels_xyz, xyz.astype(np.float32).astype(float))
    for empty in (None, [], np.zeros((0, 3))):
        model.set_voxels(empty)
        assert model.voxels_xyz.shape == (0, 3)
