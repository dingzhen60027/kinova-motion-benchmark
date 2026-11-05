"""Reporting tests use synthetic accounting fixtures, never benchmark evidence."""
import json
from pathlib import Path

import numpy as np
import pytest

from benchmarks.generic_motion_v1.runners.summarize import summarize, _cluster_ratio
from benchmarks.generic_motion_v1.runners.replay import replay, _trajectory


@pytest.fixture(autouse=True)
def accounting_fixtures_are_not_sealed_experiments(monkeypatch):
    # These unit fixtures exercise statistics only. Separate lifecycle tests
    # exercise the real integrity verifier and corruption rejection.
    monkeypatch.setattr('benchmarks.generic_motion_v1.runners.summarize._verify_run',
                        lambda *a, **k: dict(integrity_passed=True, fixture_only=True))
    def synthetic_acceptance(run, manifest, job, attempt, cache):
        # These fixtures deliberately lack real robot arrays and receipts. The
        # full saved-output acceptance is covered with real sealed fixtures in
        # test_valid_output and test_run_summary_acceptance instead.
        from benchmarks.generic_motion_v1.runners.results import STATUS
        result = json.loads((attempt / 'result.json').read_text())
        path = attempt / 'validation.json'
        validation = json.loads(path.read_text()) if path.exists() else {}
        metrics = result.get('metrics', {})
        status = STATUS[result['status']]
        return dict(status=status, objective_value=validation.get('cost', metrics.get('cost')),
                    objective_unit=validation.get('cost_unit', metrics.get('cost_unit')))
    monkeypatch.setattr('benchmarks.generic_motion_v1.runners.summarize._accepted_attempt', synthetic_acceptance)


def _write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def _job(root, name, case, method, status=None, cost=None, drm=None):
    job = dict(job_id=name, case_id=case['problem_id'], case=case,
               scene_id=case['scene_id'], family=case['family'], method=method,
               objective='minimum_distance', method_seed=0, roadmap_id='synthetic-map')
    if status:
        out = root / 'jobs' / name / 'attempts/0001'
        _write(out / 'result.json', dict(status=status, frontend_success=drm,
                                        metrics=dict(cost=cost, cost_unit='rad'),
                                        timing=dict(warm_query_seconds=1.)))
        _write(out / 'process.json', dict(cold_e2e_seconds=2., sampled_gpu_peak_mib=100.))
        _write(out / 'receipt.json', dict(status=status, files={}))
    return job


def test_full_denominator_pending_and_pairing(tmp_path):
    cases = [dict(problem_id=f'p{i}', scene_id=f's{i}', family='sparse_clutter',
                  validation_status='valid') for i in range(3)]
    jobs = [_job(tmp_path, 'a0', cases[0], 'EI_ZO', 'success_validated_numerical', 4., True),
            _job(tmp_path, 'a1', cases[1], 'EI_ZO', 'collision_failed', 1., True),
            _job(tmp_path, 'a2', cases[2], 'EI_ZO'),
            _job(tmp_path, 'b0', cases[0], 'NTO', 'success_validated_numerical', 3., True),
            _job(tmp_path, 'b1', cases[1], 'NTO', 'solver_failed', None, True),
            _job(tmp_path, 'b2', cases[2], 'NTO')]
    _write(tmp_path / 'run_manifest.json', dict(jobs=jobs))
    report = summarize(tmp_path)
    group = next(g for g in report['groups'] if g['method'] == 'EI_ZO' and g['family'] == 'all')
    assert report['planned_jobs'] == 6 and report['finished_jobs'] == 4 and not report['complete']
    assert group['unconditional_success']['estimate'] == 1 / 3
    assert group['conditional_on_drm_success']['estimate'] == .5
    assert group['success_cost']['mean'] == 4.
    pair = report['paired_comparisons'][0]
    assert pair['paired_legal_jobs'] == 3
    assert pair['common_success_cost_difference_a_minus_b']['estimate'] == 1.
    assert pair['common_success_cost_difference_a_minus_b']['denominator'] == 1
    assert pair['common_success_cost_difference_a_minus_b']['ci95'] is None


def test_invalid_inputs_and_open_control_share_cluster(tmp_path):
    jobs = []
    for i in range(4):
        case = dict(problem_id=f'p{i}', scene_id=f'open_{i}', family='open_control',
                    validation_status='invalid_endpoint' if i == 3 else 'valid', scene_cluster_id=f'unique_but_same_world_{i}')
        jobs.append(_job(tmp_path, f'a{i}', case, 'EI_ZO',
                         'invalid_endpoint' if i == 3 else 'success_validated_numerical', 2., i != 3))
    _write(tmp_path / 'run_manifest.json', dict(jobs=jobs))
    group = next(g for g in summarize(tmp_path)['groups'] if g['family'] == 'all')
    assert group['planned_jobs'] == 4 and group['invalid_input_jobs'] == 1
    assert group['unconditional_success']['denominator'] == 3
    assert group['unconditional_success']['independent_scene_clusters'] == 1
    assert group['unconditional_success']['ci95'] is None


def test_native_success_and_seed_gate_use_distinct_denominators(tmp_path):
    cases = [dict(problem_id=f'p{i}', scene_id=f's{i}', family='sparse_clutter',
                  validation_status='valid') for i in range(2)]
    jobs = [_job(tmp_path, 'passed', cases[0], 'EI_ZO', 'success_validated_numerical', 2., True),
            _job(tmp_path, 'bad_seed', cases[1], 'EI_ZO', 'frontend_failed', None, False)]
    path = tmp_path / 'jobs/bad_seed/attempts/0001/result.json'
    result = json.loads(path.read_text())
    result.update(frontend_native_success=True, frontend_validation_status='collision_failed')
    _write(path, result)
    _write(tmp_path / 'run_manifest.json', dict(jobs=jobs))
    group = next(g for g in summarize(tmp_path)['groups'] if g['family'] == 'all')
    assert group['conditional_on_drm_success']['estimate'] == 1.
    assert group['conditional_on_native_drm_success']['estimate'] == .5
    assert group['unconditional_success']['estimate'] == .5


def test_cluster_bootstrap_preserves_unequal_cluster_sizes():
    rows = [dict(cluster_id='a', success=True)] * 10 + [dict(cluster_id='b', success=False)]
    result = _cluster_ratio(rows, lambda r: r['success'], lambda r: 1)
    assert result['estimate'] == 10 / 11
    assert result['independent_scene_clusters'] == 2
    assert result['ci95'] == [0., 1.]


def test_replay_preserves_failed_samples_without_gpu(tmp_path, monkeypatch):
    from benchmarks.generic_motion_v1.model import Model, HERE
    def forbidden(*args, **kwargs):
        raise AssertionError('Replay must not call CUDA')
    monkeypatch.setattr(Model, 'cuda', forbidden)
    saved = np.load(HERE / 'reports/model_audit/raw_samples.npz')['random_states'][:3]
    case = dict(problem_id='synthetic_replay_test', scene_id='s0', family='open_control',
                q_start=saved[0].tolist(), q_goal=saved[-1].tolist(), voxels_xyz=[], voxel_radius_m=.04)
    _write(tmp_path / 'case.json', case)
    _write(tmp_path / 'result.json', dict(status='collision_failed', method='EI_ZO', objective='minimum_distance'))
    _write(tmp_path / 'validation.json', dict(time_axis='normalized_path_parameter', check='finite sample test fixture'))
    np.savez_compressed(tmp_path / 'trajectory_samples.npz', q=saved, time=[0., .5, 1.], clearance=[.1, -.01, .2])
    report = replay(tmp_path)
    data = json.loads(Path(report['data_json']).read_text())
    assert report['has_trajectory'] and report['trajectory_status'] == 'collision_failed'
    assert len(data['frames'][0]['moving_boxes']) == 15
    assert len(data['fixed_boxes']) == 5
    assert data['frames'][1]['clearance_m'] == -.01
    assert '<script src=' not in Path(report['html']).read_text()
    assert Path(report['scene_png']).stat().st_size > 10000


def test_no_candidate_does_not_fabricate_path(tmp_path):
    q, t, clearance, validation, source, has_trajectory = _trajectory(
        tmp_path, dict(q_start=[0.] * 7, q_goal=[1.] * 7))
    assert not has_trajectory and source is None and clearance is None
    assert len(q) == 2


def test_generation_and_model_failures_are_invalid_inputs(tmp_path):
    statuses = ['valid_input', 'invalid_scene_geometry', 'model_consistency_failed']
    jobs = []
    for i, status in enumerate(statuses):
        case = dict(problem_id=f'p{i}', scene_id=f's{i}', family='dense_clutter', validation_status=status)
        jobs.append(_job(tmp_path, f'j{i}', case, 'EI_ZO'))
    _write(tmp_path / 'run_manifest.json', dict(jobs=jobs))
    group = next(g for g in summarize(tmp_path)['groups'] if g['family'] == 'all')
    assert group['legal_input_jobs'] == 1 and group['invalid_input_jobs'] == 2


def test_missing_endpoints_have_no_fabricated_trajectory(tmp_path):
    q, time, clearance, validation, source, has_trajectory = _trajectory(
        tmp_path, dict(q_start=None, q_goal=None))
    assert not has_trajectory and source is None
    np.testing.assert_array_equal(q, np.zeros((1, 7)))


def test_legacy_directory_roadmap_label_resolves_actual_metadata(tmp_path):
    case = dict(problem_id='p', scene_id='s', family='open_control', validation_status='valid_input')
    job = _job(tmp_path, 'j', case, 'EI_ZO', 'success_validated_numerical', 2., True)
    roadmap = tmp_path / 'external_roadmap'
    _write(roadmap / 'metadata.json', dict(roadmap_id='actual-canonical-id', offline_seconds=43.,
                                         actual_nodes=5981, timing={'graph_seconds': 12.}))
    job.update(roadmap_id='legacy-directory-name', roadmap_dir=str(roadmap))
    _write(tmp_path / 'run_manifest.json', dict(jobs=[job]))
    result = summarize(tmp_path)
    assert result['unresolved_offline_roadmap_ids'] == []
    assert result['offline_roadmaps'][0]['roadmap_id'] == 'actual-canonical-id'
    assert result['offline_roadmaps'][0]['offline_seconds'] == 43.
    csv = (tmp_path / 'summary/summary.csv').read_text()
    assert 'actual-canonical-id' in csv and 'legacy-directory-name' in csv
    assert b'\r\n' not in (tmp_path / 'summary/summary.csv').read_bytes()


def test_interrupted_attempt_awaiting_retry_is_not_finished(tmp_path):
    case = dict(problem_id='p', scene_id='s', family='sparse_clutter', validation_status='valid_input')
    job = _job(tmp_path, 'j', case, 'EI_ZO', 'infrastructure_error')
    first = tmp_path / 'jobs/j/attempts/0001'
    _write(first / 'receipt.json', dict(status='infrastructure_error', retry_pending=True, files={}))
    _write(tmp_path / 'run_manifest.json', dict(jobs=[job]))
    report = summarize(tmp_path)
    assert not report['complete'] and report['finished_jobs'] == 0
    assert report['status_counts'] == {'retry_pending': 1}
    group = next(g for g in report['groups'] if g['family'] == 'all')
    assert group['pending_jobs'] == 1 and group['unconditional_success']['denominator'] == 1
    assert group['timing_and_memory']['cold_e2e_seconds']['failure']['count'] == 0
    import csv
    row = next(csv.DictReader((tmp_path / 'summary/summary.csv').open()))
    assert row['last_attempt_status'] == 'infrastructure_error'
    assert row['retry_pending'] == 'True' and row['attempt_sealed'] == 'True'
    second = first.parent / '0002'
    _write(second / 'result.json', dict(status='success_validated_numerical', frontend_success=True,
                                      metrics=dict(cost=3., cost_unit='rad')))
    _write(second / 'receipt.json', dict(status='success_validated_numerical', files={}))
    report = summarize(tmp_path)
    assert report['complete'] and report['planned_jobs'] == report['finished_jobs'] == 1
    assert report['status_counts'] == {'success_numerical': 1}


def test_unsealed_terminal_result_is_running_and_excluded_from_paired_time(tmp_path):
    case = dict(problem_id='p', scene_id='s', family='sparse_clutter', validation_status='valid_input')
    jobs = [_job(tmp_path, 'a', case, 'EI_ZO', 'success_validated_numerical', 2., True),
            _job(tmp_path, 'b', case, 'NTO', 'success_validated_numerical', 3., True)]
    (tmp_path / 'jobs/a/attempts/0001/receipt.json').unlink()
    _write(tmp_path / 'run_manifest.json', dict(jobs=jobs))
    report = summarize(tmp_path)
    assert not report['complete'] and report['finished_jobs'] == 1
    assert report['status_counts'] == {'running': 1, 'success_numerical': 1}
    group = next(g for g in report['groups'] if g['method'] == 'EI_ZO' and g['family'] == 'all')
    assert group['success_cost']['count'] == 0 and group['unconditional_success']['numerator'] == 0
    comparison = report['paired_comparisons'][0]
    assert comparison['paired_finished_jobs'] == 0
    assert comparison['observed_cold_time_difference_a_minus_b']['denominator'] == 0
    assert comparison['common_success_cost_difference_a_minus_b']['denominator'] == 0


def test_non_attempt_directory_cannot_replace_numbered_attempt(tmp_path):
    case = dict(problem_id='p', scene_id='s', family='open_control', validation_status='valid_input')
    job = _job(tmp_path, 'j', case, 'EI_ZO', 'frontend_failed')
    (tmp_path / 'jobs/j/attempts/z-not-an-attempt').mkdir()
    _write(tmp_path / 'run_manifest.json', dict(jobs=[job]))
    report = summarize(tmp_path)
    assert report['complete'] and report['status_counts'] == {'no_solution_within_budget': 1}


def test_recorded_metrics_are_stratified_without_filling_missing_or_pending_values(tmp_path):
    """Mirror real result.metrics plus validation.json, including a failed candidate."""
    statuses = ['success_validated_numerical', 'success_validated_numerical',
                'collision_failed', 'frontend_failed', 'success_validated_numerical', 'solver_failed']
    records = [dict(regions=2, facets=8, recovery_attempts=0, normalized_path_length=1., minimum_sampled_clearance_m=.03),
               dict(regions=4, facets=12, recovery_attempts=2, normalized_path_length=3., minimum_sampled_clearance_m=.04),
               dict(regions=3, facets=9, recovery_attempts=1, normalized_path_length=99.),
               {},
               dict(regions=100, facets=100, recovery_attempts=100, normalized_path_length=100., minimum_sampled_clearance_m=100.),
               dict(regions=float('nan'), facets=float('inf'), recovery_attempts=True,
                    normalized_path_length=float('nan'), minimum_sampled_clearance_m=float('nan'))]
    jobs = []
    for i, (status, metrics) in enumerate(zip(statuses, records)):
        case = dict(problem_id=f'p{i}', scene_id=f's{i}', family='sparse_clutter', validation_status='valid_input')
        jobs.append(_job(tmp_path, f'j{i}', case, 'EI_ZO', status))
        attempt = tmp_path / f'jobs/j{i}/attempts/0001'
        result = json.loads((attempt / 'result.json').read_text())
        result['metrics'] = metrics
        _write(attempt / 'result.json', result)
    _write(tmp_path / 'jobs/j0/attempts/0001/validation.json', dict(minimum_sampled_clearance_m=.02))
    _write(tmp_path / 'jobs/j1/attempts/0001/validation.json', dict(normalized_path_length=float('nan')))
    _write(tmp_path / 'jobs/j2/attempts/0001/validation.json', dict(normalized_path_length=2., minimum_sampled_clearance_m=-.01))
    (tmp_path / 'jobs/j4/attempts/0001/receipt.json').unlink()
    _write(tmp_path / 'run_manifest.json', dict(jobs=jobs))
    report = summarize(tmp_path)
    group = next(g for g in report['groups'] if g['family'] == 'all')
    assert group['planned_jobs'] == 6 and group['pending_jobs'] == 1
    stats = group['metrics']
    for name in ['regions', 'facets', 'recovery_attempts', 'normalized_path_length', 'minimum_sampled_clearance_m']:
        assert stats[name]['all_finished']['count'] == 3
        assert stats[name]['success']['count'] == 2
        assert stats[name]['failure']['count'] == 1
    assert stats['regions']['all_finished']['mean'] == 3.
    assert stats['facets']['all_finished']['p50'] == 9.
    assert stats['facets']['success']['mean'] == 10.
    assert stats['recovery_attempts']['success']['min'] == 0.
    assert stats['normalized_path_length']['all_finished']['mean'] == 2.
    assert stats['minimum_sampled_clearance_m']['failure']['min'] == -.01
    assert stats['minimum_sampled_clearance_m']['success']['min'] == .02
    assert stats['minimum_sampled_clearance_m']['success']['max'] == .04
    csv = (tmp_path / 'summary/summary.csv').read_text()
    assert 'normalized_path_length' in csv and 'facets' in csv
    assert 'nan' not in csv.lower() and 'inf' not in csv.lower()
