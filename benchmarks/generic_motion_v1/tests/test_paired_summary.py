"""Small CPU-only summary semantics; no solver, robot or live-run reads."""
import csv

import pytest
from benchmarks.generic_motion_v1.runners import results


def pair(problem, *, states=(True, True), cold=(10., 8.), backend=(3., 2.),
         cluster='scene', seed=0, objective='minimum_distance', cohort='comparison'):
    return [dict(problem_id=problem, replica_seed=seed, planner_seed=100 + seed,
        input_sha256='fixed-' + problem, split='test', category='clutter',
        cluster_id=cluster, method=method, objective=objective, input_valid=True,
        status=results.SUCCESS if ok else 'solver_failure',
        cold_e2e_seconds=seconds, backend_seconds=back,
        objective_value=4. if method == 'EI_ZO' else 5., cohort=cohort)
        for method, ok, seconds, back in zip(('EI_ZO', 'LGCS'), states, cold, backend)]


@pytest.fixture
def calculate(monkeypatch):
    def selected(rows, spec, verified):
        chosen = [r for r in rows if r.get('cohort', 'comparison') == spec['id']]
        planned = [dict(row=r, actual_attempts=1,
            case=dict(split=r['split'], family=r['category'],
                validation_status='valid_input' if r['input_valid'] else 'generation_failed'),
            job=dict(method=r['method'], objective=r['objective'])) for r in chosen]
        return chosen, planned
    monkeypatch.setattr(results, '_selected_rows', selected)
    def run(rows, modes=None):
        specs = [dict(id=id, mode=mode, runs=[]) for id, mode in
                 (modes or {'comparison': 'end_to_end'}).items()]
        return [r for r in results.summarize(rows, {'comparisons': specs}) if r.get('category') == 'all']
    return run


def metric(summary, name, scope, comparison='comparison'):
    return next(r for r in summary if r['row_type'] == 'paired' and r['metric'] == name
                and r['scope'] == scope and r['comparison'] == comparison)


def test_early_failure_cannot_become_common_success_speed_advantage(calculate):
    summary = calculate(pair('early', states=(False, True), cold=(.1, 20.), cluster='a')
                        + pair('both', cold=(10., 8.), cluster='b'))
    all_finished = metric(summary, 'cold_e2e_seconds', 'paired_finished_a_minus_b')
    common = metric(summary, 'cold_e2e_seconds', 'common_success_cold_a_minus_b')
    assert all_finished['count'] == 2 and all_finished['mean'] == pytest.approx(-8.95)
    assert common['count'] == 1 and common['mean'] == 2.
    assert common['scene_clusters'] == 1 and common['ci_low'] is None
    outcomes = {r['metric']: r['outcome_count'] for r in summary if r['row_type'] == 'paired_outcome'}
    assert outcomes == {'both_success': 1, 'a_only_success': 0, 'b_only_success': 1, 'neither_success': 0}


@pytest.mark.parametrize('missing', [None, float('nan'), float('inf')])
def test_missing_common_time_stays_empty_and_does_not_change_outcomes(calculate, tmp_path, missing):
    summary = calculate(pair('p', cold=(missing, 8.)))
    row = metric(summary, 'cold_e2e_seconds', 'common_success_cold_a_minus_b')
    assert row['count'] == row['scene_clusters'] == 0
    assert row['mean'] is row['ci_low'] is row['ci_high'] is None
    both = next(r for r in summary if r['row_type'] == 'paired_outcome' and r['metric'] == 'both_success')
    assert both['outcome_count'] == both['count'] == 1
    path = tmp_path / 'summary.csv'
    results._csv(path, [row], results.SUMMARY_FIELDS)
    saved = next(csv.DictReader(path.open()))
    assert saved['mean'] == saved['ci_low'] == saved['ci_high'] == ''
    assert saved['outcome_count'] == ''  # Not an outcome row; never pretend count=0.


def test_joint_outcomes_and_scene_confidence_intervals_keep_repetitions_together(calculate):
    states = [(True, True), (True, False), (False, True), (False, False)]
    rows = sum((pair(f'p{i}', states=value, cluster=f'scene{i // 2}') for i, value in enumerate(states)), [])
    first = calculate(rows)
    copies = sum((pair(f'p{i}', states=value, cluster=f'scene{i // 2}', seed=seed)
                  for seed in range(3) for i, value in enumerate(states)), [])
    repeated = calculate(copies)
    for outcome in ['both_success', 'a_only_success', 'b_only_success', 'neither_success']:
        one = next(r for r in first if r['row_type'] == 'paired_outcome' and r['metric'] == outcome)
        three = next(r for r in repeated if r['row_type'] == 'paired_outcome' and r['metric'] == outcome)
        assert one['outcome_count'] == 1 and one['count'] == 4
        assert three['outcome_count'] == 3 and three['count'] == 12
        assert one['mean'] == three['mean'] == .25
        assert one['scene_clusters'] == three['scene_clusters'] == 2
        assert (one['ci_low'], one['ci_high']) == (three['ci_low'], three['ci_high']) == (0., .5)
    success = metric(first, 'success', 'paired_success_rate_a_minus_b')
    assert success['scene_clusters'] == 2 and success['count'] == 4


def test_single_scene_has_no_confidence_interval(calculate):
    summary = calculate(pair('a') + pair('b', states=(False, False)))
    pairs = [r for r in summary if r['row_type'] in ('paired', 'paired_outcome')]
    assert pairs and all(r['scene_clusters'] == 1 for r in pairs)
    assert all(r['ci_low'] is None and r['ci_high'] is None for r in pairs)


def test_full_and_fixed_initial_clocks_are_not_pooled(calculate):
    rows = pair('same-task', cold=(15., 5.), cohort='full')
    rows += pair('same-task', cold=(2., 4.), cohort='fixed')
    summary = calculate(rows, {'full': 'end_to_end', 'fixed': 'fixed_initial_regions'})
    full = metric(summary, 'cold_e2e_seconds', 'common_success_cold_a_minus_b', 'full')
    fixed = metric(summary, 'cold_e2e_seconds', 'common_success_cold_a_minus_b', 'fixed')
    assert full['mean'] == 10. and full['count'] == 1 and full['mode'] == 'end_to_end'
    assert fixed['mean'] == -2. and fixed['count'] == 1 and fixed['mode'] == 'fixed_initial_regions'


def test_fixed_frontend_keeps_backend_and_cold_metrics_separate(calculate):
    summary = calculate(pair('p', cold=(12., 8.), backend=(2., 3.)), {'comparison': 'fixed_frontend'})
    all_backend = metric(summary, 'backend_seconds', 'paired_finished_a_minus_b')
    common_backend = metric(summary, 'backend_seconds', 'common_success_backend_a_minus_b')
    cold = metric(summary, 'cold_e2e_seconds', 'common_success_cold_a_minus_b')
    assert all_backend['mean'] == common_backend['mean'] == -1.
    assert cold['mean'] == 4.


def test_invalid_inputs_are_not_counted_as_paired_planning_failures(calculate):
    rows = pair('valid') + pair('invalid', states=(False, False))
    for row in rows[2:]: row['input_valid'] = False
    summary = calculate(rows)
    outcomes = [r for r in summary if r['row_type'] == 'paired_outcome']
    assert all(r['count'] == 1 for r in outcomes)
    assert sum(r['outcome_count'] for r in outcomes) == 1


def test_time_objective_is_separate_from_computation_seconds(calculate):
    summary = calculate(pair('p', objective='minimum_time', cold=(12., 8.)))
    duration = metric(summary, 'objective_value', 'common_success_a_minus_b')
    compute = metric(summary, 'cold_e2e_seconds', 'common_success_cold_a_minus_b')
    assert duration['mean'] == -1. and duration['unit'] == 's'
    assert compute['mean'] == 4. and compute['objective'] == 'minimum_time'


def test_pair_cannot_mix_scene_cluster_identities(calculate):
    rows = pair('p'); rows[1]['cluster_id'] = 'different'
    with pytest.raises(ValueError, match='scene cluster'): calculate(rows)


def roadmap(tmp_path, name='map0', **changes):
    directory = tmp_path / name
    directory.mkdir()
    native = directory / 'native.map'
    native.write_bytes(name.encode())
    metadata = dict(status='ready', roadmap_id=name, native_map_sha256=results.sha(native),
        native_map_bytes=native.stat().st_size, actual_nodes=6000, offline_seconds=45.,
        timing_scope='Recorded builder stages; Python startup external', seed=0,
        robot_sha256='robot', collision_geometry_sha256='geometry', model_sha256='model',
        roadmap_protocol_sha256='protocol', config={'resolution': .03})
    metadata.update(changes)
    results.write(directory / 'metadata.json', metadata)
    (directory / 'sampling.npz').write_bytes(b'other build artifact')
    return directory


def declaration(directory, *, run='run', id='job', method='EI_ZO', pending=False, **job_changes):
    job = dict(job_id=id, method=method, roadmap_dir=str(directory),
               roadmap_id=results.read(directory / 'metadata.json')['roadmap_id'])
    job.update(job_changes)
    return dict(run=run, job=job, actual_attempts=0 if pending else 2,
                row=None if pending else {'roadmap_load_seconds': .2})


def test_offline_build_is_once_across_methods_tasks_retries_and_comparison_modes(tmp_path):
    directory = roadmap(tmp_path)
    first = declaration(directory)
    second = declaration(directory, id='other', method='LGCS')
    summaries = results._offline_roadmap_summary([first, second, first, second])
    assert len(summaries) == 1
    row = summaries[0]
    assert row['offline_roadmap_seconds'] == 45. and row['roadmap_nodes'] == 6000
    assert row['planned_jobs'] == 2 and row['actual_attempts'] == 4
    assert row['roadmap_timed_load_jobs'] == 2
    assert row['roadmap_native_bytes'] == 4 and row['roadmap_bytes'] > 4
    assert row['mode'] == 'offline' and 'cold_e2e_seconds' not in row
    assert 'not every possible use' in row['scope']


def test_offline_distinct_seed_maps_remain_separate(tmp_path):
    zero, one = roadmap(tmp_path), roadmap(tmp_path, 'map1', seed=1, offline_seconds=46.)
    rows = results._offline_roadmap_summary([declaration(zero), declaration(one, id='other')])
    assert [r['roadmap_id'] for r in rows] == ['map0', 'map1']
    assert [r['offline_roadmap_seconds'] for r in rows] == [45., 46.]


@pytest.mark.parametrize('method,used', [('RRTCONNECT', True), ('RRTSTAR', True), ('EI_ZO', False)])
def test_independent_planners_are_not_charged_the_pairing_roadmap(tmp_path, method, used):
    directory = roadmap(tmp_path)
    assert results._offline_roadmap_summary([declaration(directory, method=method, roadmap_used=used)]) == []


def test_ready_map_can_be_declared_before_any_query_load(tmp_path):
    directory = roadmap(tmp_path)
    row, = results._offline_roadmap_summary([declaration(directory, pending=True)])
    assert row['planned_jobs'] == 1 and row['actual_attempts'] == row['roadmap_timed_load_jobs'] == 0
    assert row['offline_roadmap_seconds'] == 45.


def test_missing_offline_facts_remain_blank_not_zero(tmp_path):
    directory = roadmap(tmp_path, actual_nodes=None, native_map_bytes=None, offline_seconds=None, timing_scope=None)
    row, = results._offline_roadmap_summary([declaration(directory)])
    path = tmp_path / 'summary.csv'
    results._csv(path, [row], results.SUMMARY_FIELDS)
    saved = next(csv.DictReader(path.open()))
    assert saved['roadmap_nodes'] == saved['roadmap_native_bytes'] == saved['offline_roadmap_seconds'] == ''
    assert saved['offline_roadmap_timing_scope'] == ''
    assert 'cold_e2e_seconds' not in saved
    assert saved['roadmap_bytes'] != ''


@pytest.mark.parametrize('changes', [dict(config={'resolution': .01}), dict(robot_sha256='other'),
    dict(model_sha256='other'), dict(collision_geometry_sha256='other'), dict(seed=1),
    dict(offline_seconds=9.), dict(actual_nodes=999)])
def test_same_id_conflicting_offline_sources_are_not_averaged_or_first_selected(tmp_path, changes):
    import shutil
    one = roadmap(tmp_path)
    two = tmp_path / 'copy'
    shutil.copytree(one, two)
    metadata = results.read(two / 'metadata.json'); metadata.update(changes)
    results.write(two / 'metadata.json', metadata)
    with pytest.raises(ValueError, match='Conflicting offline roadmap'):
        results._offline_roadmap_summary([declaration(one), declaration(two, id='other')])


def test_same_id_different_native_maps_are_rejected_even_when_each_hash_is_valid(tmp_path):
    one = roadmap(tmp_path)
    two = roadmap(tmp_path, 'map1', roadmap_id='map0')
    with pytest.raises(ValueError, match='Conflicting offline roadmap'):
        results._offline_roadmap_summary([declaration(one), declaration(two, id='other')])


def test_identical_map_copies_are_merged_and_sources_kept(tmp_path):
    import shutil
    one = roadmap(tmp_path)
    two = tmp_path / 'copy'; shutil.copytree(one, two)
    row, = results._offline_roadmap_summary([declaration(one), declaration(two, run='second')])
    assert row['planned_jobs'] == 2 and row['offline_roadmap_seconds'] == 45.
    assert len(row['roadmap_sources']) == 2 and len(row['roadmap_metadata_sha256']) == 1


def test_next_offline_summary_rereads_changed_metadata(tmp_path):
    directory = roadmap(tmp_path)
    planned = [declaration(directory)]
    assert results._offline_roadmap_summary(planned)[0]['offline_roadmap_seconds'] == 45.
    metadata = results.read(directory / 'metadata.json'); metadata['offline_seconds'] = 46.
    results.write(directory / 'metadata.json', metadata)
    assert results._offline_roadmap_summary(planned)[0]['offline_roadmap_seconds'] == 46.


@pytest.mark.parametrize('fault', ['hash', 'bytes', 'not_ready', 'wrong_job_id'])
def test_offline_sources_must_match_available_build_evidence(tmp_path, fault):
    directory = roadmap(tmp_path)
    item = declaration(directory)
    metadata = results.read(directory / 'metadata.json')
    if fault == 'hash': metadata['native_map_sha256'] = 'wrong'
    if fault == 'bytes': metadata['native_map_bytes'] = 999
    if fault == 'not_ready': metadata['status'] = 'building'
    if fault == 'wrong_job_id': item['job']['roadmap_id'] = 'wrong'
    results.write(directory / 'metadata.json', metadata)
    with pytest.raises(ValueError): results._offline_roadmap_summary([item])


def test_summarize_appends_one_offline_record_without_changing_online_metrics(monkeypatch, tmp_path):
    directory = roadmap(tmp_path)
    rows = pair('p')
    planned = []
    for row in rows:
        item = declaration(directory, id=row['method'], method=row['method'])
        item.update(row=row, case={'split':'test','family':'clutter','validation_status':'valid_input'})
        item['job']['objective'] = row['objective']
        planned.append(item)
    monkeypatch.setattr(results, '_selected_rows', lambda *a: (rows, planned))
    summary = results.summarize(rows, {'comparisons': [
        {'id':'full','mode':'end_to_end'}, {'id':'fixed','mode':'fixed_frontend'}]})
    offline = [r for r in summary if r['row_type'] == 'offline_roadmap']
    assert len(offline) == 1 and offline[0]['planned_jobs'] == 2
    assert metric(summary, 'cold_e2e_seconds', 'common_success_cold_a_minus_b', 'full')['mean'] == 2.
    assert metric(summary, 'backend_seconds', 'common_success_backend_a_minus_b', 'fixed')['mean'] == 1.



def test_missing_declared_native_hash_still_binds_actual_content(tmp_path):
    import shutil
    one = roadmap(tmp_path, native_map_sha256=None, native_map_bytes=None)
    two = tmp_path / 'copy'; shutil.copytree(one, two)
    (two / 'native.map').write_bytes(b'bad0')  # same length; same metadata
    with pytest.raises(ValueError, match='Conflicting offline roadmap'):
        results._offline_roadmap_summary([declaration(one), declaration(two, id='other')])
    row, = results._offline_roadmap_summary([declaration(one)])
    assert row['roadmap_native_sha256'] == results.sha(one / 'native.map')
    assert row['roadmap_native_bytes'] is None


def test_ready_metadata_without_native_artifact_cannot_claim_a_built_map(tmp_path):
    directory = roadmap(tmp_path, native_map_sha256=None, native_map_bytes=None)
    (directory / 'native.map').unlink()
    with pytest.raises(ValueError, match='native artifact is missing'):
        results._offline_roadmap_summary([declaration(directory)])
