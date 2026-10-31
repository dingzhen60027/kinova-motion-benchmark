"""Real CPU SCIP MISOCP tests, with independent corridor and algebra oracles."""
import itertools
import json

import numpy as np
from pydrake.all import HPolyhedron
import pytest

from benchmarks.generic_motion_v1.runners import paper_lgcs, paper_lgcs_scip


def box(low, high):
    return HPolyhedron.MakeBox(np.asarray(low, float), np.asarray(high, float))


def test_explicit_dependency_probe_and_provisioned_binary():
    probe = paper_lgcs.dependency_probe(backend='scip')
    assert probe['solver_success'] and abs(probe['objective']-2.) < 1e-7
    assert probe['solver_version'] == [10, 0, 2]
    assert probe['pyscipopt_version'] == '6.3.0'
    files = paper_lgcs_scip.dependency_files()
    assert any('libscip' in path.name for path in files)
    assert any(path.name == 'METADATA' for path in files)
    assert all(path.is_relative_to(paper_lgcs_scip.SCIP_TARGET) for path in files)


def test_single_region_seven_joint_equivalence_and_saved_evidence(tmp_path):
    regions = [box(-np.ones(7), np.ones(7))]
    start, goal = -.5*np.ones(7), .5*np.ones(7)
    scip = paper_lgcs.solve(regions, start, goal, time_limit=5., out=tmp_path, backend='scip')
    gurobi = paper_lgcs.solve(regions, start, goal, time_limit=5.)
    assert scip['solver_success'] and scip['algebraic_validation']['passed']
    assert scip['scip_original_solution_feasible']
    assert abs(scip['solver_objective']-gurobi['solver_objective']) < 1e-7
    np.testing.assert_array_equal(scip['points'][[0, -1]], [start, goal])
    for key in ['graph_edges', 'graph_vertices', 'solver_variables', 'solver_constraints',
                'perspective_membership_rows']:
        assert scip[key] == gurobi[key]
    assert scip['solver_soc_constraints'] == gurobi['solver_quadratic_constraints']
    assert scip['solver_quadratic_constraints'] == 0
    for name in ['model.cip', 'scip.set', 'scip.log', 'solver_solution.npz', 'solution.sol', 'lgcs_solver.json']:
        assert (tmp_path/name).stat().st_size > 0
    assert scip['solver_dual_bound'] <= scip['solver_objective'] + 1e-7
    assert scip['solver_parameters']['limits/time'] <= 5.


def test_graph_solution_matches_all_simple_corridor_paths(tmp_path):
    from reproduction.paper_completion.optimization import lscs
    regions = [box([-3, -1], [-.8, 1]), box([-.8, .5], [.8, 1.5]),
               box([.8, -1], [3, 1]), box([-3, -2], [3, -.8])]
    start, goal = np.array([-2., 0.]), np.array([2., 0.])
    costs = []
    for count in range(1, len(regions)+1):
        for sequence in itertools.permutations(range(len(regions)), count):
            if not regions[sequence[0]].PointInSet(start) or not regions[sequence[-1]].PointInSet(goal):
                continue
            if not all(regions[a].IntersectsWith(regions[b]) for a, b in zip(sequence[:-1], sequence[1:])):
                continue
            oracle = lscs([regions[i] for i in sequence], start, goal)
            if oracle['solver_success']:
                costs.append(oracle['objective'])
    answer = paper_lgcs.solve(regions, start, goal, time_limit=5., out=tmp_path, backend='scip')
    assert answer['solver_success'] and answer['gap'] <= 1e-6
    assert abs(answer['solver_objective']-min(costs)) < 2e-5
    assert abs(answer['solver_objective']-4.2) < 2e-5
    for segment, index in enumerate(answer['segment_regions']):
        for point in answer['points'][segment:segment+2]:
            assert regions[index].PointInSet(point, 1e-7)


def test_above_restricted_quadratic_size_returns_bound_and_candidate(tmp_path):
    answer = paper_lgcs.solve([box(-np.ones(7), np.ones(7)) for _ in range(6)],
        np.zeros(7), np.ones(7)*.5, time_limit=5., out=tmp_path, backend='scip')
    assert answer['solver_variables'] == 300
    assert answer['solver_success'] and answer['algebraic_validation']['passed']
    # A bounded solve may retain a valid incumbent before proving its bound.
    # The analytical straight-line optimum independently checks both sides.
    optimum = np.sqrt(7)*.5
    assert answer['solver_dual_bound'] <= optimum+1e-7
    assert answer['gap'] >= 0
    if answer['solver_status'] == 'gaplimit':
        assert answer['gap'] <= 1e-6
    assert abs(answer['solver_objective']-np.sqrt(7)*.5) < 1e-7
    assert answer['globally_optimal'] == (answer['solver_status'] == 'optimal')


def test_disconnected_reports_infeasible_without_points(tmp_path):
    answer = paper_lgcs.solve([box([-3., -1.], [-2., 1.]), box([2., -1.], [3., 1.])],
                             [-2.5, 0.], [2.5, 0.], time_limit=3., out=tmp_path, backend='scip')
    assert not answer['solver_success'] and answer['solver_status'] == 'infeasible'
    assert answer['points'] is None and answer['solver_objective'] is None


def test_construction_budget_failure_preserves_model(tmp_path):
    with pytest.raises(TimeoutError, match='construction'):
        paper_lgcs.solve([box([-1.], [1.])], [0.], [.5], time_limit=1e-12, out=tmp_path, backend='scip')
    assert (tmp_path/'model.cip').is_file()
    saved = json.loads((tmp_path/'lgcs_failure.json').read_text())
    assert saved['solver_success'] is False and 'TimeoutError' in saved['error']


def test_missing_explicit_target_is_unsupported_not_fallback(monkeypatch, tmp_path):
    monkeypatch.setattr(paper_lgcs_scip, 'SCIP_TARGET', tmp_path/'missing')
    with pytest.raises(paper_lgcs.LGCSUnsupported):
        paper_lgcs.dependency_probe(tmp_path/'out', backend='scip')
    failure = json.loads((tmp_path/'out/lgcs_failure.json').read_text())
    assert failure['status'] == 'unsupported' and failure['solver'] == 'SCIP'


def test_algebra_gate_rejects_fractional_flow_and_norm_violation():
    args = ([(1, 0), (0, 2)], 1, np.array([0.]), np.array([1.]))
    bad, _ = paper_lgcs_scip._residuals(*args, np.array([.5, .5]), np.empty((0, 1)),
                                      np.array([.5]), np.array([[.5]]), [])
    assert not bad['passed'] and bad['flow_integrality'] == .5
    bad, _ = paper_lgcs_scip._residuals(*args, np.ones(2), np.empty((0, 1)),
                                      np.array([.9]), np.array([[1.]]), [])
    assert not bad['passed'] and bad['soc_norm'] > .09


def test_unknown_backend_never_silently_selects_solver():
    with pytest.raises(ValueError, match='Unknown explicit'):
        paper_lgcs.dependency_probe(backend='automatic')
    with pytest.raises(ValueError, match='Unknown explicit'):
        paper_lgcs.solve([box([-1.], [1.])], [0.], [.5], time_limit=1., backend='automatic')
