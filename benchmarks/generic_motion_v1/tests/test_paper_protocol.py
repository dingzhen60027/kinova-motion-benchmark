"""Paper profile unit conversion and offline identity regressions."""
from copy import deepcopy
import numpy as np
import pytest
from benchmarks.generic_motion_v1.roadmap import HERE, load_config, roadmap_identity


def test_paper_voxel_length_matches_native_api():
    import csdecomp as csd
    cfg = load_config(HERE / 'configs/rss2025_eizo.json')
    options = csd.RoadmapOptions()
    options.offline_voxel_resolution = cfg['roadmap']['offline_voxel_resolution']
    assert 2 * options.offline_voxel_resolution == pytest.approx(.06)
    assert options.get_offline_voxel_radius() == pytest.approx(np.sqrt(3) * .03)
    assert cfg['eizo']['configuration_margin'] == .01
    assert cfg['eizo']['delta'] == cfg['eizo']['epsilon'] == .005
    assert cfg['eizo']['mixing_steps'] == 60
    assert cfg['eizo']['max_iterations'] == 30
    # Evaluation overrides never modify the frozen task or numerical validator.
    original = load_config()
    assert original['eizo']['max_iterations'] == 500
    for key in ('velocity_limit_rad_s', 'acceleration_limit_rad_s2',
                'validation_step_rad', 'region_coverage_tolerance_rad'):
        assert cfg[key] == original[key]


def test_map_identity_changes_with_graph_cache_or_model():
    cfg = load_config()['roadmap']
    identity = roadmap_identity(6000, 0, cfg, 'robot', 'geometry')
    reordered = dict(reversed(list(cfg.items())))
    assert roadmap_identity(6000, 0, reordered, 'robot', 'geometry') == identity
    for key, value in [('max_neighbors', 10), ('offline_voxel_resolution', .03)]:
        alternate = deepcopy(cfg)
        alternate[key] = value
        assert roadmap_identity(6000, 0, alternate, 'robot', 'geometry') != identity
    assert roadmap_identity(6000, 0, cfg, 'other', 'geometry') != identity
    assert roadmap_identity(6000, 0, cfg, 'robot', 'other') != identity
