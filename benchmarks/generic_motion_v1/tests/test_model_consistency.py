"""Canonical-model contracts; GPU cross checks live in the saved G0 audit."""
import json
from pathlib import Path

import numpy as np
import pytest

from benchmarks.generic_motion_v1.model import Model, HERE, digest


@pytest.fixture(scope='module')
def model():
    return Model()


def test_model_identity_and_joint_order(model):
    assert model.dimension == 7
    assert model.joint_names == [f'kinova_joint_{i}_q' for i in range(1, 8)]
    assert model.mplant.num_scene_geometries == 20
    assert len(model.robot_geometry_ids) == 15
    assert len(model.pairs) == 137
    assert len(model.inventory['checked_pairs']) + len(model.inventory['excluded_pairs']) == 190
    assert np.isfinite(model.lower).all() and np.all(model.upper > model.lower)
    np.testing.assert_allclose(model.lower, -model.upper, atol=0, rtol=0)


def test_model_rebuild_has_stable_hashes(model):
    rebuilt = Model()
    assert rebuilt.robot_sha256 == model.robot_sha256
    assert rebuilt.collision_geometry_sha256 == model.collision_geometry_sha256
    assert rebuilt.inventory == model.inventory


def test_voxel_float32_centres_frozen_and_radius_fixed(model):
    xyz = np.array([[.123456789123, .3, .8]])
    model.set_voxels(xyz)
    np.testing.assert_array_equal(model.voxels_xyz, xyz.astype(np.float32).astype(float))
    with pytest.raises(ValueError, match='freezes sphere radius'):
        model.set_voxels(xyz, .039)
    with pytest.raises(ValueError, match='finite'):
        model.set_voxels([[np.nan, 0, 0]])
    model.set_voxels()


def test_independent_query_and_tcp_reproducibility(model):
    samples = np.load(HERE / 'reports/model_audit/raw_samples.npz')
    q = samples['random_states'][:16]
    model.set_voxels()
    np.testing.assert_array_equal(model.independent(q), samples['base_random_independent_free'][:16])
    np.testing.assert_allclose(model.clearance(q), samples['base_random_clearance'][:16], atol=1e-12)
    assert model.tcp(q).shape == (16, 3)
    assert model.robot_bounds(q).shape == (16, 2, 3)


def test_independent_voxel_contact_probe(model):
    receipt = json.loads((HERE / 'reports/model_audit/audit.json').read_text())
    q = np.load(HERE / 'reports/model_audit/raw_samples.npz')['boundary_configuration']
    for sample in receipt['boundary']['samples'][::10]:
        model.set_voxels([sample['centre']])
        np.testing.assert_allclose(model.clearance(q), [sample['clearance_m']], atol=1e-12)
    model.set_voxels()


def test_fixed_geometry_penetration_audit(model):
    # table top is z = 0; world box has centre at z = -0.1.
    result = model.fixed_geometry_sphere_clearance([[.8, 0, -.1], [.8, 0, .1]])
    assert result[0] < 0 < result[1]


def test_bad_configuration_shape_rejected(model):
    with pytest.raises(ValueError, match='N x 7'):
        model.independent(np.zeros((7, 1)))
    with pytest.raises(ValueError, match='finite'):
        model.tcp(np.full(7, np.inf))
    assert model.clearance(np.zeros((0, 7))).shape == (0,)


def test_native_nto_constraint_has_identical_online_spheres(model):
    from pydrake.all import MinimumDistanceLowerBoundConstraint
    model.set_voxels([[.3, .4, .7], [.5, -.4, .5]])
    native = model.nto_plant()
    assert native.collision_pair_count == 137 + 15 * 2
    constraint = MinimumDistanceLowerBoundConstraint(native.plant, .001,
                                                    native.plant_context,
                                                    influence_distance_offset=.01)
    q = np.load(HERE / 'reports/model_audit/raw_samples.npz')['random_states'][:8]
    expected = model.clearance(q)
    measured = []
    for configuration in q:
        native.plant.SetPositions(native.plant_context, configuration)
        query = native.scene_graph.get_query_output_port().Eval(native.scene_context)
        measured.append(min(x.distance for x in query.ComputeSignedDistancePairwiseClosestPoints()))
        assert np.isfinite(constraint.Eval(configuration)).all()
    np.testing.assert_allclose(measured, expected, atol=1e-10, rtol=0)
    model.set_voxels()
