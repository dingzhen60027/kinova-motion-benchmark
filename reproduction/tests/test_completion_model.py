"""Tests of the revised online obstacle representation and native maps."""
import numpy as np
import csdecomp as csd
from reproduction.common import ROOT, read
from reproduction.paper_completion.model import ForestEnvironment
from reproduction.paper_completion.forest import build_roadmap
from reproduction.paper_completion.data import planner, endpoint_audit


def test_forest_online_geometry_and_independent_checks():
    cfg = read(ROOT / 'reproduction/paper_completion/config.json')
    env = ForestEnvironment(cfg, 0)
    q = np.vstack([env.base.centers, np.random.default_rng(83).uniform(-5, 5, (4096, 2))])
    assert env.minimal.num_collision_pairs == 0
    assert not env.cuda(env.base.centers).any()
    assert np.array_equal(env.cuda(q), env.independent(q))
    assert endpoint_audit(env, np.array([-4., -4.]))['canonical_valid']


def test_forest_real_map_build_load_and_collision_activation(tmp_path):
    cfg = read(ROOT / 'reproduction/paper_completion/config.json')
    result = build_roadmap(cfg, 0, 200, tmp_path)
    assert result['actual_nodes'] == 200
    assert result['collision_map_entries'] > 0
    assert np.isclose(result['native_voxel_half_side'], 0.03)
    env = ForestEnvironment(cfg, 0)
    p = planner(env)
    p.LoadRoadmap(str(tmp_path / 'native.map'))
    p.BuildCollisionSet(env.voxels)
    drm = p.drm
    assert len(drm.id_to_pose_map) == 200
    assert all(i in drm.node_adjacency_map[j]
               for i, neighbors in drm.node_adjacency_map.items() for j in neighbors)
    ok, path = p.Plan(np.array(env.cfg['start']), np.array(env.cfg['goal']), env.voxels, env.voxel_radius)
    assert ok
    assert np.array_equal(path[0], env.cfg['start'])
    assert np.array_equal(path[-1], env.cfg['goal'])
