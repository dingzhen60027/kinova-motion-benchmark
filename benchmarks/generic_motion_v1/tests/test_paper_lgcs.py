"""Real MISOCP oracle checks and alternate-route identity/failure regressions."""
import itertools
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from pydrake.all import HPolyhedron

from benchmarks.generic_motion_v1.runners import paper_lgcs


def box(low, high):
    return HPolyhedron.MakeBox(np.asarray(low, float), np.asarray(high, float))


def test_actual_seven_joint_solve_preserves_query_and_global_dependencies(tmp_path):
    import gurobipy
    from reproduction.paper_completion import optimization
    original_model, original_function = gurobipy.Model, optimization.lgcs
    start, goal = -np.ones(7)*.5, np.ones(7)*.5
    regions = [box(-np.ones(7), np.ones(7))]
    candidate = paper_lgcs.solve(regions, start, goal, time_limit=5., out=tmp_path)
    assert candidate['solver_success'] and candidate['globally_optimal']
    assert candidate['objective'] == 'minimum_distance'
    assert abs(candidate['solver_objective']-np.sqrt(7)) < 1e-7
    np.testing.assert_array_equal(candidate['points'][[0, -1]], [start, goal])
    assert candidate['segment_regions'] == [0]
    assert candidate['reproduction_status'] == 'adapted'
    assert candidate['solver_parameters']['NonConvex'] == 0
    assert 0 < candidate['solver_parameters']['TimeLimit'] <= 5
    assert candidate['solver_parameters']['FeasibilityTol'] == 1e-9
    assert candidate['solver_quality']['MaxVio'] < 1e-8
    assert gurobipy.Model is original_model and optimization.lgcs is original_function
    assert (tmp_path/'model.mps.gz').is_file()
    saved = json.loads((tmp_path/'lgcs_solver.json').read_text())
    assert saved['solver_variables'] == 10 and saved['solver_quadratic_constraints'] == 1


def test_graph_solution_matches_exhaustive_corridor_oracle():
    from reproduction.paper_completion.optimization import lscs
    regions = [box([-3, -1], [-.8, 1]), box([-.8, .5], [.8, 1.5]),
               box([.8, -1], [3, 1]), box([-3, -2], [3, -.8])]
    start, goal = np.array([-2., 0.]), np.array([2., 0.])
    oracle = []
    for count in range(1, len(regions)+1):
        for sequence in itertools.permutations(range(len(regions)), count):
            if not regions[sequence[0]].PointInSet(start) or not regions[sequence[-1]].PointInSet(goal):
                continue
            if not all(regions[a].IntersectsWith(regions[b]) for a, b in zip(sequence[:-1], sequence[1:])):
                continue
            answer = lscs([regions[i] for i in sequence], start, goal)
            if answer['solver_success']:
                oracle.append(answer['objective'])
    answer = paper_lgcs.solve(regions, start, goal, time_limit=5.)
    assert answer['solver_success'] and answer['globally_optimal']
    assert abs(answer['solver_objective']-min(oracle)) < 2e-5
    assert abs(answer['solver_objective']-4.2) < 2e-5
    for segment, index in enumerate(answer['segment_regions']):
        for point in answer['points'][segment:segment+2]:
            assert regions[index].PointInSet(point, 1e-8)


def test_disconnected_graph_reports_infeasible_without_candidate():
    answer = paper_lgcs.solve([box([-3., -1.], [-2., 1.]), box([2., -1.], [3., 1.])],
                             [-2.5, 0.], [2.5, 0.], time_limit=3.)
    assert not answer['solver_success'] and answer['solver_status'] == 3
    assert answer['solver_objective'] is None and 'points' not in answer


def test_installed_restricted_license_failure_keeps_submitted_model(tmp_path):
    regions = [box(-np.ones(7), np.ones(7)) for _ in range(6)]
    try:
        answer = paper_lgcs.solve(regions, np.zeros(7), np.ones(7)*.5, time_limit=5., out=tmp_path)
    except paper_lgcs.LGCSUnsupported as error:
        assert error.dependency_unsupported
        metadata = error.optimization_metadata
        assert metadata['solver_error_number'] == 10010
        assert metadata['solver_variables'] > 200
        assert metadata['solver_success'] is False and metadata['status'] == 'unsupported'
        saved = json.loads((tmp_path/'lgcs_failure.json').read_text())
        assert saved['solver_variables'] == metadata['solver_variables']
    else:
        # The same regression remains usable after an authorized full license
        # is installed, without assuming that a restricted license must exist.
        assert answer['solver_success']
    assert (tmp_path/'model.mps.gz').is_file()


@pytest.mark.parametrize('budget', [0, -1, np.inf, np.nan, True])
def test_invalid_budget_never_enters_solver(budget):
    with pytest.raises(ValueError, match='budget'):
        paper_lgcs.solve([box([-1.], [1.])], [0.], [.5], time_limit=budget)


def _fake_native(monkeypatch, native_success=True):
    import sys
    instances = []
    plans = []

    class DRM:
        def __init__(self):
            self.id_to_node_map = {i: np.r_[float(i), np.zeros(6)] for i in range(4)}
            self.node_adjacency_map = {0: [1, 3], 1: [0, 2], 2: [1, 3], 3: [0, 2]}
            self.id_to_pose_map = {i: np.eye(4) for i in range(4)}
            self.collision_map = {8: [0, 1], 9: [2, 3]}
            instances.append(self)

        def Read(self, path):
            pass

        def Write(self, path):
            Path(path).write_text(json.dumps(sorted(self.id_to_node_map)))

    class Planner:
        def __init__(self, plant, options, quiet):
            self.plant, self.options = plant, options
            plans.append(self)

        def LoadRoadmap(self, path):
            self.path = path

        def BuildCollisionSet(self, voxels):
            self.voxels = voxels

        def Plan(self, start, goal, voxels, radius):
            self.query = (start.copy(), goal.copy(), voxels, radius)
            return native_success, np.array([start, goal]) if native_success else []

    monkeypatch.setitem(sys.modules, 'csdecomp', SimpleNamespace(
        DRM=DRM, DrmPlannerOptions=SimpleNamespace, DrmPlanner=Planner))
    return instances, plans


@pytest.mark.parametrize('validated', [True, False])
def test_alternate_route_prunes_graph_and_keeps_fixed_input(monkeypatch, tmp_path, validated):
    instances, plans = _fake_native(monkeypatch)
    source = tmp_path/'original.map'
    source.write_bytes(b'original frozen roadmap')
    endpoints = np.zeros((2, 7)); endpoints[1, 0] = 3.
    low, high = np.full(7, -1.), np.ones(7)
    low[0], high[0] = .5, 1.5
    regions = [box(low, high)]
    model = SimpleNamespace(csd_plant=object(), voxels=object(), voxel_radius=.04)
    validations = []

    def validate(path, out):
        validations.append((path.copy(), out))
        return dict(passed=validated, status='success_validated_numerical' if validated else 'collision_failed')

    path, metadata = paper_lgcs.second_route(model, source, endpoints, regions,
        {'try_shortcutting': True}, seed=9, out=tmp_path/'attempt', validate_path=validate)
    copied = instances[0]
    assert sorted(copied.id_to_node_map) == [0, 2, 3]
    assert copied.node_adjacency_map == {0: [3], 2: [3], 3: [0, 2]}
    assert sorted(copied.id_to_pose_map) == [0, 2, 3]
    assert copied.collision_map == {8: [0], 9: [2, 3]}
    assert source.read_bytes() == b'original frozen roadmap'
    plan = plans[0]
    assert plan.plant is model.csd_plant and plan.voxels is model.voxels
    assert plan.query[2] is model.voxels and plan.query[3] == .04
    np.testing.assert_array_equal(plan.query[:2], endpoints)
    assert len(validations) == 1
    assert metadata['blocked_nodes'] == 1 and metadata['endpoints_changed'] is False
    assert metadata['status'] == ('found' if validated else 'alternate_path_validation_failed')
    if validated:
        np.testing.assert_array_equal(path, endpoints)
    else:
        assert path is None


def test_no_eligible_region_does_not_call_native_planner(monkeypatch, tmp_path):
    instances, plans = _fake_native(monkeypatch)
    source = tmp_path/'original.map'; source.write_bytes(b'unchanged')
    endpoints = np.zeros((2, 7)); endpoints[1, 0] = .5
    path, metadata = paper_lgcs.second_route(SimpleNamespace(), source, endpoints,
        [box(-np.ones(7), np.ones(7))], {}, seed=0, out=tmp_path/'attempt',
        validate_path=lambda path, out: pytest.fail('No candidate should be validated'))
    assert path is None and metadata['status'] == 'no_eligible_interior_region'
    assert not instances and not plans


def test_real_dependency_probe_executes_and_records(tmp_path):
    result = paper_lgcs.dependency_probe(tmp_path)
    assert result['solver_success'] and result['objective'] == 1.
    assert result['status'] == 'available'
    assert json.loads((tmp_path/'lgcs_dependency_probe.json').read_text())['variables'] == 1
