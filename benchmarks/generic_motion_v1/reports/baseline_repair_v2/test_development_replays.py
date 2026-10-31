"""Small synthetic replay-selection/gate fixtures; no renderer, solver or GPU."""
from collections import Counter
from copy import deepcopy
import json
from pathlib import Path

import pytest

from benchmarks.generic_motion_v1.reports.baseline_repair_v2 import make_development_replays as replay


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def row(case, label, *, blocked=True, success=True, split='dev', seed=0, legal=True):
    method, objective = replay.METHODS[label]
    return dict(case_id=case, label=label, method=method, objective=objective, split=split,
        roadmap_seed=seed, roadmap_id=f'map-{seed}', method_seed=0, job_id=f'{case}-{seed}-{label}',
        input_legal=legal, success=success, status=replay.SUCCESS if success else 'solver_failed',
        straight_line_collision=blocked, family='bottleneck', drm_success=True, seed_cost_validated=True,
        roadmap_nodes=6000, input_sha256=case, q_start=[0.] * 7, q_goal=[1.] * 7,
        robot_sha256='robot', geometry_sha256='geometry')


@pytest.fixture
def complete_study(tmp_path):
    here = tmp_path / 'reports'
    blocks, rows, runs = [], [], []
    for name, count in replay.BLOCKS.items():
        split, seed = name.split('_s')
        seed = int(seed)
        run = tmp_path / 'results' / name
        save(run / 'run_manifest.json', dict(synthetic=True, name=name))
        digest = replay.sha(run / 'run_manifest.json')
        block = dict(name=name, split=split, planned_jobs=count, run=str(run), run_manifest_sha256=digest)
        blocks.append(block)
        runs.append(dict(run=str(run), manifest_sha256=digest, source_commit=replay.SOURCE,
            integrity=dict(complete=True, integrity_passed=True)))
        for i in range(count // 5):
            for label in replay.LABELS:
                r = row(f'{split}-case-{i:03d}', label, split=split, seed=seed, legal=split == 'dev' or i < 8)
                if not r['input_legal']:
                    r.update(status='invalid_endpoint', success=False, drm_success=False, seed_cost_validated=False)
                r.update(run=str(run), attempt=str(run / 'jobs' / r['job_id'] / 'attempts' / '000'))
                rows.append(r)
    index = dict(blocks=blocks, slots=950, test_planner_calls=0)
    completed = dict(complete=True, completed_jobs=950, total_jobs=950, blocks=[
        dict(b, verification=dict(complete=True, integrity_passed=True, completed_jobs=b['planned_jobs'],
            planned_jobs=b['planned_jobs'], errors=[], unsealed_attempts=[])) for b in blocks])
    frozen = dict(source_commit=replay.SOURCE, planned_registered_slots=950, runtime_source_hashes={})
    for name, data in [('run_index.json', index), ('completed_blocks.json', completed), ('execution_freeze.json', frozen)]:
        save(here / name, data)
    index_sha, freeze_sha = replay.sha(here / 'run_index.json'), replay.sha(here / 'execution_freeze.json')
    groups = []
    for split in ('smoke', 'dev'):
        for label in replay.LABELS:
            pool = [r for r in rows if r['split'] == split and r['label'] == label]
            groups.append(dict(split=split, method=label, family='all', registered=len(pool),
                success=sum(r['success'] for r in pool), status_counts=dict(Counter(r['status'] for r in pool))))
    integrity = dict(passed=True, errors=[], complete=True, source_code_commit=replay.SOURCE,
        registered_slots=950, legal_slots=940, invalid_slots=10, groups=groups,
        run_manifest_sha256={b['run']: b['run_manifest_sha256'] for b in blocks},
        evidence=dict(run_index=dict(sha256=index_sha), freeze=dict(sha256=freeze_sha)))
    lgcs = dict(passed=True, errors=[], complete=True, index_sha256=index_sha, freeze_sha256=freeze_sha,
        new_source_commit=replay.SOURCE, expected_lgcs_jobs=190, paired_dev=dict(n=180, legal_n=180))
    nto = dict(passed=True, errors=[], index_sha256=index_sha, planned_new_slots=950, new_nto_slots=380, paired_dev_nto_slots=360)
    for name, data in [('integrity', integrity), ('lgcs', lgcs), ('nto', nto)]:
        save(here / (name + '_audit.json'), data)
    metrics = dict(complete=True, comparable=True, requested_jobs=950, rows=rows, runs=runs)
    return here, metrics


def test_complete_gate_and_whole_matrix(complete_study):
    here, metrics = complete_study
    gate = replay.completion_barrier(here, True)
    replay.check_metrics(metrics, gate)
    assert len(gate['run_hashes']) == 4


@pytest.mark.parametrize('audit', ['integrity', 'lgcs', 'nto'])
@pytest.mark.parametrize('change', [{'passed': False}, {'passed': 1}, {'errors': ['synthetic error']}])
def test_failed_audit_precedes_any_metrics_read(complete_study, monkeypatch, audit, change):
    here, _ = complete_study
    path = here / (audit + '_audit.json')
    data = replay.read(path)
    data.update(change)
    save(path, data)
    original_read = replay.read
    def guarded_read(path):
        assert Path(path).name != 'do-not-read-metrics.json', 'Metrics read before the independent-audit gate'
        return original_read(path)
    monkeypatch.setattr(replay, 'HERE', here)
    monkeypatch.setattr(replay, 'read', guarded_read)
    with pytest.raises(ValueError):
        replay.main(['--planning-finished', '--metrics', str(here / 'do-not-read-metrics.json')])


@pytest.mark.parametrize('audit', ['integrity', 'lgcs', 'nto'])
def test_unbound_audit_rejected(complete_study, audit):
    here, _ = complete_study
    path = here / (audit + '_audit.json')
    data = replay.read(path)
    if audit == 'integrity':
        data['evidence']['run_index']['sha256'] = 'another-study'
    else:
        data['index_sha256'] = 'another-study'
    save(path, data)
    with pytest.raises(ValueError):
        replay.completion_barrier(here, True)


@pytest.mark.parametrize('change', [dict(complete=False), dict(completed_jobs=949)])
def test_incomplete_study_rejected(complete_study, change):
    here, _ = complete_study
    path = here / 'completed_blocks.json'
    data = replay.read(path)
    data.update(change)
    save(path, data)
    with pytest.raises(ValueError):
        replay.completion_barrier(here, True)


def test_explicit_completion_and_no_test_scope(complete_study):
    here, _ = complete_study
    with pytest.raises(ValueError):
        replay.completion_barrier(here, False)
    path = here / 'run_index.json'
    data = replay.read(path)
    data['test_planner_calls'] = 1
    save(path, data)
    with pytest.raises(ValueError):
        replay.completion_barrier(here, True)


@pytest.mark.parametrize('mutation', ['duplicate', 'endpoint', 'status', 'label', 'test', 'subset', 'wrong_run'])
def test_bad_metrics_are_rejected(complete_study, mutation):
    here, metrics = complete_study
    gate = replay.completion_barrier(here, True)
    metrics = deepcopy(metrics)
    if mutation == 'duplicate': metrics['rows'][1] = metrics['rows'][0]
    elif mutation == 'endpoint': metrics['rows'][1]['q_goal'][0] = 2.
    elif mutation == 'status': metrics['rows'][1]['success'] = False
    elif mutation == 'label': metrics['rows'][1]['method'] = 'NTO_PAPER'
    elif mutation == 'test': metrics['rows'][1]['split'] = 'test'
    elif mutation == 'subset': metrics['reporting_problem_ids'] = ['selected-case']
    elif mutation == 'wrong_run': metrics['runs'][0]['manifest_sha256'] = 'different'
    with pytest.raises(ValueError): replay.check_metrics(metrics, gate)


def test_common_success_precedes_per_label_and_prefers_blocked_lexical():
    rows = [row(case, label, blocked=blocked) for case, blocked in
            [('a-unblocked', False), ('c-blocked', True), ('b-blocked', True)] for label in replay.LABELS]
    for r in rows:
        r['cost'] = 0 if r['case_id'] == 'c-blocked' else 1000
        r['cold_e2e_seconds'] = r['cost']
    rows.append(row('d-failure', 'LSCS', success=False))
    choices, selection = replay.select_illustrations(list(reversed(rows)))
    assert selection['shared_case_for_five_methods'] is True
    assert [c['name'] for c in choices] == [*replay.LABELS, 'failure_seed_only']
    assert all(c['row']['case_id'] == 'b-blocked' for c in choices[:5])


def test_fallback_is_per_label_and_seed_is_never_a_final_success():
    rows = [row(f'case-{i}', label) for i, label in enumerate(replay.LABELS)]
    rows += [row('a-failure', 'NTO-refined-distance', success=False), row('0-fast-unblocked', 'LSCS', success=False, blocked=False)]
    rows += [row('000-test', label, split='test') for label in replay.LABELS]
    rows += [row('000-smoke', label, split='smoke') for label in replay.LABELS]
    choices, selection = replay.select_illustrations(rows)
    assert selection['shared_case_for_five_methods'] is False
    assert len(choices) == 6 and not selection['missing']
    failed = choices[-1]
    assert failed['role'] == 'verified_frontend_seed_only'
    assert failed['row']['case_id'] == 'a-failure' and failed['row']['success'] is False
    assert all(c['row']['split'] == 'dev' for c in choices)


def test_missing_views_are_reported_and_duplicates_rejected():
    rows = [row('only', 'LSCS'), row('bad-input', 'LGCS', success=False, legal=False)]
    choices, selection = replay.select_illustrations(rows)
    assert len(choices) == 1
    assert {m['label'] for m in selection['missing']} == set(replay.LABELS[1:]) | {'failure_seed_only'}
    with pytest.raises(ValueError): replay.select_illustrations(rows + [rows[0]])


def test_seed_projection_is_byte_identical_and_original_stays_unchanged(tmp_path):
    attempt = tmp_path / 'source'
    for path in ('input.json', 'result.json', 'process.json', 'receipt.json'):
        save(attempt / path, dict(status='solver_failed'))
    save(attempt / 'frontend_validation/validation.json', dict(passed=True))
    (attempt / 'frontend_validation/trajectory_samples.npz').write_bytes(b'fixture-only-not-loaded-as-an-array')
    files = replay.source_snapshot(attempt)
    r = row('failure', 'NTO-refined-distance', success=False)
    r['attempt'] = str(attempt)
    projected = replay.seed_projection(r, tmp_path / 'report-projection', dict(files=files))
    assert replay.source_snapshot(attempt) == files
    assert projected['source_status'] == 'solver_failed' and projected['role'] == 'verified_frontend_seed_only'
    for name, data in projected['files'].items():
        assert replay.sha(tmp_path / 'report-projection' / name) == data['sha256']
    (attempt / 'unexpected.txt').write_text('mutation')
    assert replay.source_snapshot(attempt) != files
