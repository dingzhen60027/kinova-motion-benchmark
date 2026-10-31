"""Graph-pool ordering and restored coverage regressions; CPU only."""
import numpy as np
import pytest
from pydrake.all import HPolyhedron
from benchmarks.generic_motion_v1.runners.paper_graph import (
    selected_pool_indices, complete_pool_cover)


def test_selected_indices_map_candidate_segments_to_unordered_pool():
    pool = [object(), object(), object()]
    candidate = dict(points=np.zeros((3, 7)), segment_regions=[2, 0])
    selected = selected_pool_indices(candidate, pool)
    assert selected.tolist() == [2, 0]
    assert selected[np.asarray([0, 1, 0])].tolist() == [2, 0, 2]
    for wrong in ([0], [True, 0], [3, 0], [-1, 0], [1., 0.]):
        with pytest.raises(ValueError):
            selected_pool_indices(dict(candidate, segment_regions=wrong), pool)


def test_recovery_reinflates_only_uncovered_original_edges_of_both_paths(tmp_path):
    a, b, c = np.zeros(7), np.ones(7), np.ones(7) * 2
    d = b.copy(); d[1] = -1
    regions = [HPolyhedron.MakeBox(a - .1, b + .1)]
    seeds = [np.asarray([a, b])]
    calls = []
    def inflate(u, v):
        calls.append(np.asarray([u, v]))
        return HPolyhedron.MakeBox(np.minimum(u, v) - .01, np.maximum(u, v) + .01)
    paths = [np.asarray([a, b, c]), np.asarray([a, d, c])]
    pool, returned = complete_pool_cover(regions, seeds, paths, inflate, 1e-6, tmp_path, lambda: None)
    assert len(calls) == 3
    assert len(returned) == len(pool) == 4
    assert all(any(np.array_equal(call, edge) for path in paths
                   for edge in np.asarray(list(zip(path[:-1], path[1:])))) for call in calls)
    assert all(any(r.PointInSet(u) and r.PointInSet(v) for r in pool)
               for path in paths for u, v in zip(path[:-1], path[1:]))


def test_canonical_double_endpoints_survive_float32_seed_cover(tmp_path):
    from benchmarks.generic_motion_v1.runners.paper_graph import protected_edge_anchors
    a, b = np.zeros(7), np.ones(7)
    fixed = np.asarray([a.copy(), b.copy()]); fixed[0, 0] -= 3e-7; fixed[1, 0] += 3e-7
    path = np.asarray([a, b]); region = HPolyhedron.MakeBox(a, b)
    assert not region.PointInSet(fixed[0], 0.)
    anchors = protected_edge_anchors(path, [path], fixed)
    assert np.array_equal(anchors[-2:], fixed)
    def no_new_region(*args):
        pytest.fail('The bounded floating-point correction should reuse this region')
    pool, _ = complete_pool_cover([region], [path], [path], no_new_region, 1e-6,
                                  tmp_path, lambda: None, fixed)
    assert pool[0].PointInSet(fixed[0], 0.) and pool[0].PointInSet(fixed[1], 0.)
