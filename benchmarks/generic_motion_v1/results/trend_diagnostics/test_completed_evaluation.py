"""CPU-only acceptance-checker regression fixtures, never experimental results."""
import copy
import importlib.util
import json
from pathlib import Path
import sys

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import check_completed_evaluation as audit
import check_metric_bounds as bounds


def test_complete_matrix_requires_every_case_seed_method_and_track():
    dataset = {'dev': [{'problem_id': 'p1'}, {'problem_id': 'p2'}]}
    pairs = [('A', 'minimum_distance'), ('A', 'minimum_time')]
    jobs = [dict(case_id=c['problem_id'], replica_seed=s, method=m, objective=o)
            for c in dataset['dev'] for s in (0, 1, 2) for m, o in pairs]
    manifest = {'split': 'dev', 'jobs': jobs}
    assert audit.check_declared_matrix([manifest], dataset, pairs=pairs)['errors'] == []
    missing = audit.check_declared_matrix([dict(manifest, jobs=jobs[:-1])], dataset, pairs=pairs)
    assert missing['missing_jobs'] == 1
    duplicate = audit.check_declared_matrix([dict(manifest, jobs=jobs+[jobs[0]])], dataset, pairs=pairs)
    assert any(e['check'] == 'duplicate_declared_job' for e in duplicate['errors'])
    unexpected = dict(jobs[0], replica_seed=3)
    assert any(e['check'] == 'unexpected_declared_job' for e in audit.check_declared_matrix(
        [dict(manifest, jobs=jobs+[unexpected])], dataset, pairs=pairs)['errors'])


def protocol_fixture():
    case = dict(problem_id='p1', robot_sha256='robot', collision_geometry_sha256='geometry',
                velocity_limits_rad_s=[1.]*7, acceleration_limits_rad_s2=[1.]*7,
                paired_seeds=[dict(drm_seed=0, method_seed=19)])
    job = dict(job_id='j1', case_id='p1', replica_seed=0, method_seed=19, case=case,
               evaluation_config=dict(velocity_limit_rad_s=1., acceleration_limit_rad_s2=1., ompl={'planning_seconds': 60.}))
    manifest = dict(timeout_seconds=60., source_hashes={'benchmarks/generic_motion_v1/runners/validate.py': 'validator'}, jobs=[job])
    return case, job, manifest


def test_protocol_detects_changed_input_motion_limit_seed_validator_and_budget(monkeypatch):
    monkeypatch.setattr(audit, 'resolve_case', lambda job, cache: job['case'])
    case, job, manifest = protocol_fixture()
    dataset = {'dev': [case]}
    good = audit.check_protocol([manifest], dataset)
    assert not good['errors'] and good['ompl_native_planning_caps_seconds'] == [60.]
    for field, value, expected in [('method_seed', 20, 'unpaired_algorithm_seed'),
                                  ('evaluation_config', dict(job['evaluation_config'], velocity_limit_rad_s=2.), 'motion_limit_changed'),
                                  ('case', dict(case, robot_sha256='small_robot'), 'fixed_input_changed')]:
        broken = dict(manifest, jobs=[dict(job, **{field: value})])
        assert any(x['check'] == expected for x in audit.check_protocol([broken], dataset)['errors'])
    other = dict(manifest, source_hashes={'benchmarks/generic_motion_v1/runners/validate.py': 'coarser_validator'})
    assert any(x['check'] == 'mixed_validator_source' for x in audit.check_protocol([manifest, other], dataset)['errors'])
    assert any(x['check'] == 'unexpected_outer_budget' for x in audit.check_protocol([dict(manifest, timeout_seconds=10)], dataset)['errors'])


def test_index_checks_all_fields_and_rejects_duplicate_or_missing_rows(tmp_path):
    run = tmp_path/'run'
    fresh = dict(record_id='id', run=str(run), status='infrastructure_error', raw_status='solver_failed',
                 failure_reason='BoxBoxGradient assertion failed', receipt_integrity=True)
    indexed = {field: audit.csv_value(fresh.get(field)) for field in audit.results.RECORD_FIELDS}
    assert not audit.check_record_index([fresh], [indexed], {run})
    hidden = dict(indexed, status='solver_failure')
    assert any(e.get('field') == 'status' for e in audit.check_record_index([fresh], [hidden], {run}))
    assert audit.check_record_index([fresh], [], {run})
    assert audit.check_record_index([fresh], [indexed, indexed], {run})


def test_summary_checks_include_invalid_inputs_and_every_failure():
    case = dict(split='smoke', family='open_control', validation_status='valid_input')
    chosen = dict(split='smoke', category='open_control', objective='minimum_distance', method='A',
                  input_valid=True, status='timeout')
    planned = [dict(case=case, job={'method': 'A', 'objective': 'minimum_distance'}, row=chosen, actual_attempts=1)]
    row = dict(comparison='g', row_type='status', split='smoke', objective='minimum_distance', method='A',
               planned_jobs='1', actual_attempts='1', legal_jobs='1', finished_legal_jobs='1', pending_jobs='0', invalid_input_jobs='0')
    row.update({s: '1' if s == 'timeout' else '0' for s in audit.STATES if s != 'invalid_input'})
    summary = [dict(row, category=c) for c in ('all', 'open_control')]
    assert not audit.check_summary_counts([chosen], planned, summary, 'g')
    summary[0]['timeout'] = '0'
    assert any(e.get('field') == 'timeout' for e in audit.check_summary_counts([chosen], planned, summary, 'g'))


def test_pending_review_never_collects_or_reports_scores(tmp_path, monkeypatch):
    run = tmp_path/'run'; run.mkdir()
    (run/'run_manifest.json').write_text('{}')
    monkeypatch.setattr(audit.batch, 'verify', lambda *a, **k: dict(integrity_passed=True, complete=False, completed_jobs=1, planned_jobs=2))
    monkeypatch.setattr(audit.results, 'collect', lambda *a, **k: pytest.fail('Incomplete run cannot receive final statistics'))
    report = audit.review([run], [{'id': 'g'}], records_path=tmp_path/'missing.csv', summary_path=tmp_path/'missing.csv')
    assert report['status'] == 'pending' and not report['passed']
    assert 'completed_groups' not in report and 'metric_bounds' not in report
    destination = audit.save_report(report, tmp_path/'acceptance.json')
    assert destination.name == 'acceptance.pending.json'
    assert not (tmp_path/'acceptance.json').exists()


def test_main_pending_returns_nonzero_and_only_pending_file(tmp_path, monkeypatch):
    selection = tmp_path/'selection.json'; selection.write_text('{}')
    monkeypatch.setattr(audit, 'resolve_scope', lambda *a: ([], []))
    monkeypatch.setattr(audit, 'review', lambda *a, **k: dict(status='pending', passed=False, errors=[]))
    output = tmp_path/'result.json'
    assert audit.main(['--runs', 'missing', '--selection', str(selection), '--output', str(output)]) == 2
    assert output.with_name('result.pending.json').exists() and not output.exists()


def test_scope_rejects_unselected_runs_and_conditioned_comparisons():
    selection = {'comparisons': [dict(id='main', mode='end_to_end', runs=['results/a']),
                                 dict(id='conditional', mode='fixed_frontend', runs=['results/a'])]}
    runs, specs = audit.resolve_scope(selection, ['results/a'], ['main'])
    assert specs[0]['id'] == 'main'
    with pytest.raises(ValueError, match='coverage differ'):
        audit.resolve_scope(selection, ['results/b'], ['main'])
    with pytest.raises(ValueError, match='end_to_end'):
        audit.resolve_scope(selection, None, ['conditional'])
    with pytest.raises(ValueError, match='Unknown'):
        audit.resolve_scope(selection, None, ['missing'])


def metric_fixture():
    case = dict(validation_status='valid_input', q_start=[0.]*7, q_goal=[2.]+[0.]*6)
    row = dict(record_id='r', selected_comparisons='["new_test"]', status='success_numerical',
               problem_id='p', objective='minimum_time', path_length_rad='2.0', trajectory_duration_s='3.0')
    config = lambda r: dict(velocity_limit_rad_s=1., acceleration_limit_rad_s2=1.)
    return case, row, config


def test_metric_bounds_include_explicit_new_comparison_and_reject_impossible_values():
    case, row, config = metric_fixture()
    result = bounds.check_rows([row], ['new_test'], {'p': case}, config)
    assert result['passed'] and result['counts'] == {'path_checks': 1, 'time_checks': 1}
    assert not bounds.check_rows([row], ['missing'], {'p': case}, config)['passed']
    for field, value in [('path_length_rad', '1.9'), ('trajectory_duration_s', '2.9'),
                         ('path_length_rad', ''), ('trajectory_duration_s', 'nan')]:
        result = bounds.check_rows([dict(row, **{field: value})], ['new_test'], {'p': case}, config)
        assert not result['passed']
        json.dumps(result, allow_nan=False)
    assert not bounds.check_rows([row], ['new_test'], {'p': dict(case, validation_status='invalid_endpoint')}, config)['passed']


def test_zero_success_group_is_accounted_without_fabricating_metrics():
    case, row, config = metric_fixture()
    result = bounds.check_rows([dict(row, status='timeout')], ['new_test'], {'p': case}, config)
    assert result['passed'] and result['observations']['new_test'] == 1
    assert result['groups']['new_test'] == 0 and result['counts']['path_checks'] == 0


def test_reference_protocol_cannot_silently_coarsen_validation(monkeypatch):
    monkeypatch.setattr(audit, 'resolve_case', lambda job, cache: job['case'])
    case, job, reference = protocol_fixture()
    changed = dict(reference, jobs=[dict(job, evaluation_config=dict(job['evaluation_config'], validation_step_rad=.1))])
    checked = audit.check_protocol([changed], {'dev': [case]}, reference)
    assert any(e['check'] == 'validation_protocol_changed_from_reference' for e in checked['errors'])


def test_summary_metric_and_common_success_pair_are_recomputed():
    selected = [dict(problem_id=p, replica_seed=0, split='smoke', objective='minimum_distance', category='open_control',
                     method=m, status=s, input_valid=True, objective_value=v)
                for p, m, s, v in [('p1', 'A', 'success_numerical', 2.), ('p2', 'A', 'success_numerical', 4.),
                                   ('p1', 'B', 'success_numerical', 1.), ('p2', 'B', 'timeout', None)]]
    metric = dict(comparison='g', row_type='metric', split='smoke', objective='minimum_distance', category='all',
                  method='A', metric='objective_value', scope='validated_success_only_descriptive', count='2', mean='3.0', median='3.0', p90='3.8', p95='3.9')
    paired = dict(metric, row_type='paired', method_b='B', scope='common_success_a_minus_b', count='1', mean='1.0', median='1.0', p90='1.0', p95='1.0')
    assert not audit.check_summary_metrics(selected, [metric, paired], 'g')
    assert any(e['field'] == 'mean' for e in audit.check_summary_metrics(selected, [dict(metric, mean='4.')], 'g'))
    assert any(e['field'] == 'count' for e in audit.check_summary_metrics(selected, [dict(paired, count='2')], 'g'))
    assert any(e['field'] == 'p95' for e in audit.check_summary_metrics(selected, [dict(metric, p95='4.')], 'g'))


@pytest.mark.parametrize('scope,count,mean,median,p90,p95', [
    ('all_finished_legal', '3', '20.7', '2.0', '48.4', '54.2'),
    ('success_numerical_only', '1', '2.0', '2.0', '2.0', '2.0'),
    ('failed_legal_only', '2', '30.05', '30.05', '54.01', '57.005'),
])
def test_latency_strata_are_independently_recomputed(scope, count, mean, median, p90, p95):
    selected = [dict(problem_id=str(index), replica_seed=0, split='smoke', objective='minimum_distance',
                     category='open_control', method='A', status=state, input_valid=valid, cold_e2e_seconds=value)
                for index, (state, valid, value) in enumerate([
                    ('success_numerical', True, 2.), ('timeout', True, 60.),
                    ('infrastructure_error', True, .1), ('solver_failure', True, None),
                    ('invalid_input', False, 999.)])]
    row = dict(comparison='g', row_type='metric', split='smoke', objective='minimum_distance', category='all',
               method='A', metric='cold_e2e_seconds', scope=scope,
               count=count, mean=mean, median=median, p90=p90, p95=p95)
    assert not audit.check_summary_metrics(selected, [row], 'g')
    assert any(e['field'] == 'count' for e in audit.check_summary_metrics(selected, [dict(row, count='5')], 'g'))


def test_fixed_regions_use_exact_reconstructed_eligibility_and_two_jobs(tmp_path, monkeypatch):
    from benchmarks.generic_motion_v1.runners import shared_initialization
    # The source-stage fixture has one eligible and one invalid registration.
    source = tmp_path/'source'; source.mkdir()
    (source/'run_manifest.json').write_text(json.dumps({'jobs': [dict(method='EI_ZO', objective='minimum_distance', replica_seed=0, roadmap_id='map')]}))
    run = tmp_path/'fixed'; run.mkdir()
    coverage = dict(registered_task_replicas=2, eligible_task_replicas=1,
                    coverage=[dict(problem_id='p1', replica_seed=0, eligible=True), dict(problem_id='p2', replica_seed=0, eligible=False)])
    (run/'initialization_coverage.json').write_text(json.dumps(coverage))
    monkeypatch.setattr(shared_initialization, 'prepare_index', lambda *a, **k: ({}, coverage))
    jobs = [dict(case_id='p1', replica_seed=0, method=m, objective='minimum_distance', evaluation_config={}) for m in ('EI_ZO', 'LGCS')]
    manifest = dict(shared_initialization_source=str(source), split='test', jobs=jobs)
    valid = audit.check_conditioned_cohort(run, manifest, {'test': []}, {source})
    assert not valid['errors'] and valid['expected_paired_jobs'] == 2
    wrong = audit.check_conditioned_cohort(run, dict(manifest, jobs=jobs[:1]), {'test': []}, {source})
    assert any(e['check'] == 'conditioned_paired_matrix' for e in wrong['errors'])
    stale = audit.check_conditioned_cohort(run, manifest, {'test': []}, {tmp_path/'new_source'})
    assert any(e['check'] == 'conditioned_source_is_not_new_primary_run' for e in stale['errors'])
    (run/'initialization_coverage.json').write_text(json.dumps(dict(coverage, eligible_task_replicas=0)))
    assert any(e['check'] == 'conditioned_eligibility_changed' for e in audit.check_conditioned_cohort(run, manifest, {'test': []}, {source})['errors'])


def repair_pair_fixture(valid=True):
    import compare_repaired_dev as comparison
    row = dict(problem_id='p1', method='EI_ZO', objective='minimum_distance', replica_seed=0,
               input_valid=valid, status='success_numerical' if valid else 'invalid_input',
               raw_status='success_validated_numerical' if valid else 'invalid_endpoint',
               category='open_control', objective_value=2. if valid else None, objective_unit='rad',
               path_length_rad=2. if valid else None, cold_e2e_seconds=1., frontend_validated=valid,
               result_path=None, validation_path=None, roadmap_id='map')
    row.update({field: 'equal' for field in comparison.EQUAL_FIELDS})
    row.update(budget_seconds=60., planner_seed=15, cluster_id='scene')
    return row


def test_repair_pairing_retains_invalid_registrations_and_rejects_unmatched_inputs():
    import compare_repaired_dev as comparison
    legal = repair_pair_fixture(); invalid = dict(repair_pair_fixture(False), problem_id='bad')
    pairs, failures = comparison.exact_pairs([legal, invalid], [dict(legal, objective_value=1.5), invalid])
    assert len(pairs) == len(failures) == 1
    assert pairs[0]['cost_delta_new_minus_old'] == -.5
    assert failures[0]['old_status'] == failures[0]['new_status'] == 'invalid_input'
    with pytest.raises(ValueError, match='one-to-one'):
        comparison.exact_pairs([legal, invalid], [legal])
    with pytest.raises(ValueError, match='Unmatched'):
        comparison.exact_pairs([legal], [dict(legal, input_sha256='changed')])
    with pytest.raises(ValueError, match='validity changed'):
        comparison.exact_pairs([legal], [dict(legal, input_valid=False)])


def test_repair_within_cohort_uses_primary_ci_and_preserves_direction():
    import compare_repaired_dev as comparison
    rows = [dict(comparison='new', category='all', row_type='paired', metric='objective_value',
                 scope='common_success_a_minus_b', method='EI_ZO', method_b=other, objective=objective,
                 mean=mean, ci_low=lower, ci_high=upper, count='7', scene_clusters='3', unit=unit)
            for other, objective, mean, lower, upper, unit in
            [('RRTCONNECT', 'minimum_distance', '-2', '-3', '-1', 'rad'),
             ('NTO_REFINED', 'minimum_time', '1', '0', '2', 's')]]
    result = comparison.within_new_comparisons(rows, 'new')
    assert result[0]['paired_mean'] == 2 and result[0]['scene_cluster_ci95'] == [1., 3.]
    assert result[1]['paired_mean'] == -1 and result[1]['scene_cluster_ci95'] == [-2., 0.]
    assert all(r['common_success_queries'] == 7 for r in result)
    with pytest.raises(ValueError, match='Missing or duplicate'):
        comparison.within_new_comparisons(rows[:1], 'new')


def test_unprepared_or_pending_repair_comparison_cannot_emit_final_scores(tmp_path, monkeypatch):
    import compare_repaired_dev as comparison
    run = tmp_path/'new_run'; prefix = tmp_path/'output'
    assert comparison.compare(run, tmp_path/'records.csv', 'old', None, prefix) == 2
    assert prefix.with_name(prefix.name+'_pending.json').exists()
    assert not prefix.with_suffix('.json').exists() and not prefix.with_name(prefix.name+'_pairs.csv').exists()
    run.mkdir(); (run/'run_manifest.json').write_text('{}')
    monkeypatch.setattr(comparison.results, 'run_status', lambda *a: dict(planned=14, completed=13, pending=1))
    monkeypatch.setattr(comparison, 'verify', lambda *a: pytest.fail('Pending cohort must stop before final statistics'))
    assert comparison.compare(run, tmp_path/'records.csv', 'old', None, prefix) == 2
    assert not prefix.with_suffix('.json').exists()
