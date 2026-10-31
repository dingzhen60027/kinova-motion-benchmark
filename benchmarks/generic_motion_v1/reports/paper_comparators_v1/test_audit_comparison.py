"""Mutation tests for protocol identity; synthetic data are never report input."""
from copy import deepcopy
import json

import numpy as np
import pytest

from benchmarks.generic_motion_v1.reports.paper_comparators_v1.audit_comparison import (
    LABELS, NEW_PREFIX, REUSE_PREFIX, BLOCKS, SUCCESS, validate_pairing,
    validate_matrix, compare_sources, compare_frozen_inputs, check_nto_parameters, audit_original_attempt,
    collect_native_diagnostics)


def rows_for_task(case_id='case', split='dev', family='sparse_clutter', scene='scene', seed=0, legal=True):
    return [dict(label=label, case_id=case_id, roadmap_id=f'roadmap{seed}', method_seed=seed,
                 split=split, family=family, cluster_id=scene, roadmap_seed=seed, roadmap_nodes=6000,
                 input_legal=legal, q_start=[0.] * 7, q_goal=[1.] * 7, input_sha256=case_id,
                 robot_sha256='robot', geometry_sha256='geometry', budget_seconds=60.,
                 straight_line_collision=True, native_drm_success=legal, drm_success=legal,
                 path_sha256='path' if legal else None, success=legal,
                 status=SUCCESS if legal else 'invalid_endpoint', cold_e2e_seconds=1.,
                 final_validation_passed=True if legal else None, cost=2. if legal else None)
            for label in LABELS]


def matrix():
    rows = []
    families = ('sparse_clutter', 'dense_clutter', 'bottleneck', 'route_choice')
    for family in families:
        for scene in range(5):
            for endpoint in range(3):
                for seed in range(3):
                    rows += rows_for_task(f'{family}{scene}_{endpoint}', family=family,
                                          scene=f'{family}{scene}', seed=seed)
    for family in (*families, 'open_control'):
        for endpoint in range(2):
            rows += rows_for_task(f'smoke_{family}_{endpoint}', split='smoke', family=family,
                                  scene=f'smoke_{family}', legal=family != 'dense_clutter')
    return rows


def parameters(timed=False):
    path = np.zeros((3, 7))
    profile = dict(name='rss2025_franka_nto_reconstruction_v1', clearance_m=0.,
                   collocation_parameter_Nc=150, influence_distance_m=.01, maximum_iterations=2000,
                   maximum_optimizer_seconds=60., solver_feasibility_tolerance=1e-6,
                   warm_start_resampled=False, minimum_control_points=None,
                   velocity_limit_rad_s=1., acceleration_limit_rad_s2=1.)
    options = dict(max_iter=2000, max_cpu_time=58., tol=1e-6) if timed else {
        'Major iterations limit':2000, 'Major feasibility tolerance':1e-6,
        'Major optimality tolerance':1e-6, 'Time limit':58.}
    return path, dict(profile=profile, solver='IPOPT' if timed else 'SNOPT', spline_order=4 if timed else 2,
                     collocation_parameter_Nc=150, collocation_points=149,
                     collocation_locations=(np.arange(1, 150)/150).tolist(),
                     warm_start_resampled=False, refinement_warm_start=False, legacy_cfg_nto_used=False,
                     warm_start_node_count=3, warm_start_control_count=5 if timed else 3,
                     warm_start_endpoint_copies=2 if timed else 0, parameters=options,
                     reported_iteration_limits=dict(defaults_overridden=False))


def test_complete_synthetic_matrix():
    assert len(validate_matrix(matrix())) == 190


@pytest.mark.parametrize('mutation', ['missing_method', 'duplicate', 'different_endpoint', 'different_path',
                                     'different_frontend', 'different_input', 'illegal_success', 'nonfinite_time'])
def test_pairing_rejects_incomparable_inputs(mutation):
    rows = rows_for_task()
    if mutation == 'missing_method': rows.pop()
    elif mutation == 'duplicate': rows.append(deepcopy(rows[0]))
    elif mutation == 'different_endpoint': rows[0]['q_goal'][0] = 2.
    elif mutation == 'different_path': rows[0]['path_sha256'] = 'changed'
    elif mutation == 'different_frontend': rows[0]['drm_success'] = False
    elif mutation == 'different_input': rows[0]['input_sha256'] = 'changed'
    elif mutation == 'illegal_success': rows[0]['input_legal'] = False
    elif mutation == 'nonfinite_time': rows[0]['cold_e2e_seconds'] = float('nan')
    with pytest.raises(ValueError): validate_pairing(rows)


@pytest.mark.parametrize('mutation', ['missing_case', 'formal_test', 'wrong_budget', 'missing_scene', 'wrong_family'])
def test_matrix_rejects_denominator_drift(mutation):
    rows = matrix()
    if mutation == 'missing_case': rows = rows[7:]
    elif mutation == 'formal_test':
        for row in rows[:7]: row['split'] = 'test'
    elif mutation == 'wrong_budget':
        for row in rows: row['budget_seconds'] = 120.
    elif mutation == 'missing_scene':
        for row in rows: row['cluster_id'] = 'shared'
    elif mutation == 'wrong_family':
        for row in rows[:7]: row['family'] = 'open_control'
    with pytest.raises(ValueError): validate_matrix(rows)


@pytest.mark.parametrize('timed', [False, True])
def test_original_profile_passes(timed):
    path, record = parameters(timed)
    check_nto_parameters(record, 'minimum_time' if timed else 'minimum_distance', path)


@pytest.mark.parametrize('mutation', ['margin', 'refinement', 'total_iterations', 'points', 'location',
                                     'order', 'resampled', 'refined_seed', 'extra_controls', 'budget'])
def test_profile_detects_silent_strengthening(mutation):
    path, record = parameters()
    if mutation == 'margin': record['profile']['clearance_m'] = .001
    elif mutation == 'refinement': record['profile']['maximum_native_solves'] = 4
    elif mutation == 'total_iterations': record['parameters']['Iterations limit'] = 100000
    elif mutation == 'points': record['collocation_points'] = 150
    elif mutation == 'location': record['collocation_locations'][0] = 0.
    elif mutation == 'order': record['spline_order'] = 4
    elif mutation == 'resampled': record['warm_start_resampled'] = True
    elif mutation == 'refined_seed': record['refinement_warm_start'] = True
    elif mutation == 'extra_controls': record['warm_start_control_count'] = 10
    elif mutation == 'budget': record['parameters']['Time limit'] = 61.
    with pytest.raises(ValueError): check_nto_parameters(record, 'minimum_distance', path)


def source_manifests():
    manifests = {}
    for prefix in (NEW_PREFIX, REUSE_PREFIX):
        for block in BLOCKS:
            count = (20 if block.startswith('smoke') else 120) if prefix == NEW_PREFIX else (
                50 if block.startswith('smoke') else 300)
            manifests[prefix+block] = dict(source_hashes={'a':'hash'}, timeout_seconds=60,
                formal_test_run=False, jobs=[dict(evaluation_config={'a':1},
                    method='NTO_PAPER' if prefix == NEW_PREFIX else 'EI_ZO') for _ in range(count)])
    return manifests


def test_identical_runtime_allows_distinct_commit_history():
    values = source_manifests()
    for i, record in enumerate(values.values()): record['source_code_commit'] = str(i)
    assert len(compare_sources(values)) == 64


@pytest.mark.parametrize('field,value', [('source_hashes', {'a':'changed'}), ('timeout_seconds', 61),
                                       ('formal_test_run', True), ('jobs', [])])
def test_source_mismatch_is_rejected(field, value):
    values = source_manifests()
    next(iter(values.values()))[field] = value
    with pytest.raises(ValueError): compare_sources(values)


def test_timeout_before_parameter_save_is_preserved(tmp_path):
    row = dict(attempt=str(tmp_path), success=False, status='timeout')
    assert audit_original_attempt(row)['profile_saved'] is False


def test_success_without_parameter_evidence_fails(tmp_path):
    with pytest.raises(ValueError):
        audit_original_attempt(dict(attempt=str(tmp_path), success=True, status=SUCCESS))


def test_original_group_cannot_hide_refinement_round(tmp_path):
    (tmp_path/'refinement').mkdir()
    with pytest.raises(ValueError):
        audit_original_attempt(dict(attempt=str(tmp_path), success=False, status='timeout'))


def frozen_fixture():
    old = {field: {'fixture':1} for field in ('runtime_source_hashes', 'evaluation_config',
        'config_file_sha256', 'data_files', 'roadmap_files', 'external_dependency_files')}
    old.update(timeout_seconds=60, gpu_workers=1)
    new = dict(deepcopy(old), planned_registered_slots=380, test_planner_calls=0, reused_evidence={})
    index = dict(slots=380, test_planner_calls=0, blocks=[])
    jobs = [dict(case_id='a', roadmap_id='map', case={'q_start':[0.]*7}, method_seed=1,
                 solver_seed=2, solver_seed_used=False, evaluation_config={}, roadmap_dir='map') for _ in range(7)]
    manifests = {'fixture':dict(source_hashes={'fixture':1}, jobs=jobs)}
    return new, old, index, manifests, {}


def test_full_case_seed_and_dependency_freeze_matches():
    compare_frozen_inputs(*frozen_fixture())


@pytest.mark.parametrize('field', ['runtime_source_hashes', 'evaluation_config', 'config_file_sha256',
                                  'data_files', 'roadmap_files', 'external_dependency_files'])
def test_freeze_mismatch_fails(field):
    args = frozen_fixture(); args[0][field] = {'changed':1}
    with pytest.raises(ValueError): compare_frozen_inputs(*args)


@pytest.mark.parametrize('field,value', [('case', {'q_start':[1.]*7}), ('method_seed', 2),
    ('solver_seed', 3), ('solver_seed_used', True), ('evaluation_config', {'changed':1}), ('roadmap_dir','different')])
def test_full_manifest_mismatch_fails(field, value):
    args = frozen_fixture(); args[3]['fixture']['jobs'][0][field] = value
    with pytest.raises(ValueError): compare_frozen_inputs(*args)


def test_native_exception_without_details_is_not_hidden(tmp_path):
    (tmp_path/'nto_solver_result.json').write_text(json.dumps(dict(solver_status='exception',
        solver_success=False, error='native box witness assertion', traceback='fixture')))
    rows = [dict(split='dev', attempt=str(tmp_path), label='NTO-distance', case_id='one',
                 roadmap_seed=0, status='solver_failed')]
    result = collect_native_diagnostics(rows)
    recorded = result['original_nto_dev_native_outcomes']['NTO-distance'][0]
    assert recorded['solver_status'] == 'exception' and recorded['info'] is None and recorded['count'] == 1
    assert result['original_nto_dev_native_exceptions'][0]['terminal_status'] == 'solver_failed'


def test_collision_rejection_keeps_native_success_status(tmp_path):
    (tmp_path/'nto_solver_result.json').write_text(json.dumps(dict(solver_status='kSolutionFound',
        solver_success=True, native_solver_details=dict(info=1))))
    row = dict(split='dev', attempt=str(tmp_path), label='NTO-distance', case_id='one',
               roadmap_seed=0, status='collision_failed', minimum_sampled_clearance_m=-.001)
    result = collect_native_diagnostics([row])
    assert result['original_nto_dev_native_outcomes']['NTO-distance'][0]['solver_success'] is True
    assert result['original_nto_dev_collision_failures'][0]['minimum_sampled_clearance_m'] == -.001
    assert result['original_nto_dev_native_exceptions'] == []


def test_missing_timeout_native_result_is_not_invented(tmp_path):
    result = collect_native_diagnostics([dict(split='dev', attempt=str(tmp_path), status='timeout')])
    assert result['original_nto_dev_native_outcomes'] == {}
