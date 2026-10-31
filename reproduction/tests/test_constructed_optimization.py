"""Solve the paper transcriptions against independent analytic/finite oracles."""
import itertools
import numpy as np
from pydrake.all import HPolyhedron, BsplineBasis, BsplineTrajectory, KnotVectorType
from reproduction.common import ROOT, read
from reproduction.paper_constructed.optimization import lscs, lgcs, nto
from reproduction.paper_constructed.model import ForestEnvironment
from reproduction.paper_constructed.validation import validate


def test_perspective_gcs_matches_all_simple_corridors():
    boxes = [([-3, -1], [-.8, 1]), ([-.8, .5], [.8, 1.5]),
             ([.8, -1], [3, 1]), ([-3, -2], [3, -.8])]
    regions = [HPolyhedron.MakeBox(np.array(lo), np.array(hi)) for lo, hi in boxes]
    start, goal = np.array([-2., 0.]), np.array([2., 0.])
    oracle = []
    for count in range(1, len(regions) + 1):
        for seq in itertools.permutations(range(len(regions)), count):
            if not regions[seq[0]].PointInSet(start) or not regions[seq[-1]].PointInSet(goal):
                continue
            if not all(regions[a].IntersectsWith(regions[b]) for a, b in zip(seq[:-1], seq[1:])):
                continue
            result = lscs([regions[i] for i in seq], start, goal)
            if result['solver_success']:
                oracle.append(result['objective'])
    result = lgcs(regions, start, goal)
    assert result['solver_success'] and result['globally_optimal']
    assert abs(result['objective'] - min(oracle)) < 2e-5
    assert abs(result['objective'] - 4.2) < 2e-5
    for segment, i in enumerate(result['segment_regions']):
        assert regions[i].PointInSet(result['points'][segment], 1e-6)
        assert regions[i].PointInSet(result['points'][segment + 1], 1e-6)


def test_conic_planners_seven_dimensional_analytic_optimum():
    region = HPolyhedron.MakeBox(-np.ones(7) * 2, np.ones(7) * 2)
    start, goal = -np.ones(7) * .5, np.ones(7) * .5
    for planner in (lscs, lgcs):
        result = planner([region], start, goal)
        assert result['solver_success']
        assert abs(result['objective'] - np.sqrt(7)) < 1e-5


def test_nto_retains_drm_nodes_and_published_point_count():
    cfg = read(ROOT / 'reproduction/paper_constructed/config.json')
    env = ForestEnvironment(cfg, 1)
    env.start, env.goal = np.array([-4., -4.]), np.array([-4., 4.])
    env.cfg.update(start=env.start, goal=env.goal)
    # An independently free short detour retains its interior node exactly in
    # the warm start, rather than introducing eight arclength-sampled controls.
    path = np.array([env.start, [-4.2, 0.], env.goal])
    assert env.independent(np.vstack([np.linspace(path[0], path[1], 100),
                                      np.linspace(path[1], path[2], 100)])).all()
    for timed in (False, True):
        result = nto(path, env, timed)
        assert result['solver_success'] and validate(result, env)['passed']
        assert np.array_equal(result['warm_start_original_nodes'], path)
        controls = np.vstack([path[0], path, path[-1]]) if timed else path
        assert np.array_equal(result['warm_start_controls'], controls)
        assert result['control_points'] == len(controls)
        nc = cfg['nto_collocation']['forest']['time' if timed else 'distance']
        assert np.array_equal(result['collocation_locations'], np.arange(1, nc + 1) / (nc + 1))
        assert result['collocation_points'] == nc
        if timed:
            basis = BsplineBasis(4, len(controls), KnotVectorType.kClampedUniform,
                                 0, result['warm_start_duration'])
            initial = BsplineTrajectory(basis, [q[:, None] for q in controls])
            assert np.max(np.abs(initial.EvalDerivative(0))) < 1e-12
            assert np.max(np.abs(initial.EvalDerivative(initial.end_time()))) < 1e-12


def test_gcs_reports_disconnected_sets_as_infeasible():
    regions = [HPolyhedron.MakeBox(np.array([-3., -1.]), np.array([-2., 1.])),
               HPolyhedron.MakeBox(np.array([2., -1.]), np.array([3., 1.]))]
    result = lgcs(regions, np.array([-2.5, 0.]), np.array([2.5, 0.]))
    assert not result['solver_success'] and result['solver_status'] == 3
