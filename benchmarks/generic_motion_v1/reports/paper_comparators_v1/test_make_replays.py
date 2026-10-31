"""Synthetic completion, provenance and selector checks; no renderer or GPU."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from benchmarks.generic_motion_v1.reports.paper_comparators_v1 import make_replays as replay


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def row(case, label, *, blocked=True, success=True, split='dev', seed=0, legal=True):
    method, objective = replay.METHODS[label]
    return dict(case_id=case, label=label, method=method, objective=objective, split=split,
        roadmap_seed=seed, roadmap_id=f'map-{seed}', method_seed=0, job_id=f'{case}-{seed}-{label}',
        input_legal=legal, success=success, status=replay.SUCCESS if success else 'solver_failed',
        straight_line_collision=blocked, family='bottleneck', drm_success=True, seed_cost_validated=True,
        roadmap_nodes=6000, input_sha256=case, q_start=[0.] * 7, q_goal=[1.] * 7,
        robot_sha256='robot', geometry_sha256='geometry')


@pytest.fixture
def study(tmp_path):
    here = tmp_path / 'reports'
    blocks, reused, source_runs, rows, runs = [], {}, {}, [], []
    for prefix, labels in [('new', ('NTO-distance', 'NTO-time')),
                          ('old', tuple(l for l in replay.ALL_LABELS if l not in ('NTO-distance', 'NTO-time')))]:
        for name in replay.BLOCKS:
            split, seed = name.split('_s')
            seed = int(seed)
            n = 10 if split == 'smoke' else 60
            count = n * len(labels)
            run = tmp_path / 'results' / (prefix + name)
            save(run / 'run_manifest.json', dict(synthetic=True, name=name, prefix=prefix))
            digest = replay.sha(run / 'run_manifest.json')
            check = dict(complete=True, integrity_passed=True, errors=[], unsealed_attempts=[],
                         completed_jobs=count, planned_jobs=count)
            if prefix == 'new':
                blocks.append(dict(name=name, split=split, planned_jobs=count, run=str(run), run_manifest_sha256=digest))
            else:
                reused[str(run)] = dict(jobs=count, verification=check, manifest_sha256=digest)
            source_runs[str(run)] = dict(manifest_sha256=digest, source_commit=prefix, integrity=check)
            runs.append(dict(run=str(run), manifest_sha256=digest, source_commit=prefix))
            for i in range(n):
                for label in labels:
                    r = row(f'{split}-case-{i:03d}', label, split=split, seed=seed, legal=split == 'dev' or i < 8)
                    if not r['input_legal']:
                        r.update(status='invalid_endpoint', success=False, drm_success=False, seed_cost_validated=False)
                    r.update(run=str(run), attempt=str(run / 'jobs' / r['job_id'] / 'attempts' / '000'))
                    rows.append(r)
    index = dict(slots=380, test_planner_calls=0, blocks=blocks)
    done = dict(complete=True, completed_jobs=380, total_jobs=380, blocks=[
        dict(b, verification=source_runs[b['run']]['integrity']) for b in blocks])
    frozen = dict(planned_registered_slots=380, test_planner_calls=0, reused_evidence=reused, runtime_source_hashes={})
    for name, value in [('run_index.json', index), ('completed_blocks.json', done), ('execution_freeze.json', frozen)]:
        save(here / name, value)
    metrics = dict(complete=True, comparable=True, requested_jobs=1330, rows=rows, runs=runs,
                   expected_method_labels=list(replay.ALL_LABELS))
    metrics_path = here / 'comparison_metrics/metrics.json'
    save(metrics_path, metrics)
    audit = dict(passed=True, errors=[], registered_slots=1330, new_registered_slots=380,
        reused_registered_slots=950, legal_slots=1316, invalid_slots=14, new_legal_slots=376,
        no_test_planner_calls=True, source_runs=source_runs, metrics_sha256=replay.sha(metrics_path),
        freeze_sha256=replay.sha(here / 'execution_freeze.json'), run_index_sha256=replay.sha(here / 'run_index.json'))
    audit_path = here / 'comparison_audit.json'
    save(audit_path, audit)
    return here, metrics_path, audit_path


def test_full_gate(study):
    here, metric, audit = study
    barrier = replay.completion_barrier(here, True, audit)
    replay.check_metrics(replay.read(metric), metric, barrier)
    assert len(barrier['run_hashes']) == 8


@pytest.mark.parametrize('change', [dict(complete=False), dict(completed_jobs=379)])
def test_incomplete_planning_prevents_metric_read(study, monkeypatch, change):
    here, metric, audit = study
    value = replay.read(here / 'completed_blocks.json'); value.update(change)
    save(here / 'completed_blocks.json', value)
    original = replay.read
    def guarded(path):
        assert Path(path) != metric, 'Read metrics before completion'
        return original(path)
    monkeypatch.setattr(replay, 'read', guarded)
    monkeypatch.setattr(replay, 'HERE', here)
    with pytest.raises(ValueError):
        replay.main(['--planning-finished', '--metrics', str(metric), '--audit', str(audit)])


@pytest.mark.parametrize('change', [dict(passed=False), dict(passed=1), dict(errors=['failed']),
    dict(legal_slots=1330), dict(new_registered_slots=379), dict(no_test_planner_calls=False),
    dict(freeze_sha256='changed'), dict(run_index_sha256='changed')])
def test_audit_guard(study, change):
    here, _, audit = study
    value = replay.read(audit); value.update(change); save(audit, value)
    with pytest.raises(ValueError): replay.completion_barrier(here, True, audit)


def test_explicit_gate_and_changed_source(study):
    here, _, audit = study
    with pytest.raises(ValueError): replay.completion_barrier(here, False, audit)
    source = next(iter(replay.read(audit)['source_runs']))
    save(Path(source) / 'run_manifest.json', dict(changed=True))
    with pytest.raises(ValueError): replay.completion_barrier(here, True, audit)


@pytest.mark.parametrize('mutation', ['duplicate', 'endpoint', 'status', 'label', 'test', 'subset', 'wrong_run'])
def test_metric_guards(study, mutation):
    here, metric, audit = study
    gate = replay.completion_barrier(here, True, audit)
    data = deepcopy(replay.read(metric))
    if mutation == 'duplicate': data['rows'][1] = data['rows'][0]
    elif mutation == 'endpoint': data['rows'][1]['q_goal'][0] = 2.
    elif mutation == 'status': data['rows'][1]['success'] = False
    elif mutation == 'label': data['rows'][1]['method'] = 'NTO_REFINED'
    elif mutation == 'test': data['rows'][1]['split'] = 'test'
    elif mutation == 'subset': data['reporting_problem_ids'] = ['chosen']
    elif mutation == 'wrong_run': data['runs'][0]['manifest_sha256'] = 'changed'
    with pytest.raises(ValueError): replay.check_metrics(data, metric, gate)


def test_hash_guard(study):
    here, metric, audit = study
    gate = replay.completion_barrier(here, True, audit)
    value = replay.read(metric); value['new_field'] = 1; save(metric, value)
    with pytest.raises(ValueError): replay.check_metrics(value, metric, gate)


def test_common_lexical_choice_ignores_cost_and_refined():
    rows = [row(case, label, blocked=blocked) for case, blocked in
            [('a-unblocked', False), ('c-blocked', True), ('b-blocked', True)] for label in replay.ALL_LABELS]
    for r in rows:
        r['cost'] = 0 if r['case_id'] == 'c-blocked' else 1000
        r['cold_e2e_seconds'] = r['cost']
    rows.append(row('failed', 'NTO-distance', success=False))
    choices, selection = replay.select_illustrations(list(reversed(rows)))
    assert [c['name'] for c in choices] == [*replay.LABELS, 'failure_seed_only']
    assert all(c['row']['case_id'] == 'b-blocked' for c in choices[:5])
    assert selection['shared_case_for_five_methods'] and selection['common_query_straight_line_collision'] is True
    assert all(c['row']['method'] != 'NTO_REFINED' for c in choices)


def test_unblocked_common_is_disclosed_and_blocked_time_added():
    rows = [row('a-unblocked', label, blocked=False) for label in replay.LABELS]
    rows += [row('z-blocked', 'NTO-time'), row('failure', 'NTO-distance', success=False)]
    choices, selection = replay.select_illustrations(rows)
    assert selection['shared_case_for_five_methods'] and selection['common_query_straight_line_collision'] is False
    assert [c['name'] for c in choices] == [*replay.LABELS, 'NTO-time_blocked', 'failure_seed_only']
    assert choices[5]['row']['case_id'] == 'z-blocked'


def test_missing_original_never_uses_refined_or_test():
    rows = [row('dev', 'NTO-refined-distance'), row('test', 'NTO-distance', split='test'),
            row('smoke', 'NTO-distance', split='smoke'), row('bad', 'NTO-distance', legal=False)]
    choices, selection = replay.select_illustrations(rows)
    assert not choices
    assert {m['label'] for m in selection['missing']} == set(replay.LABELS) | {'failure_seed_only'}


def test_per_label_fallback_and_seed_only():
    rows = [row(f'case-{i}', label) for i, label in enumerate(replay.LABELS)]
    rows += [row('a-failure', 'NTO-distance', success=False),
             row('0-refined-failure', 'NTO-refined-distance', success=False)]
    choices, selection = replay.select_illustrations(rows)
    assert not selection['shared_case_for_five_methods'] and not selection['missing']
    assert choices[-1]['row']['case_id'] == 'a-failure'
    assert choices[-1]['role'] == 'verified_frontend_seed_only'
    with pytest.raises(ValueError): replay.select_illustrations(rows + [rows[0]])


def test_seed_projection_is_byte_identical_and_gallery_explicit(tmp_path):
    attempt = tmp_path / 'source'
    for name in ('input.json', 'result.json', 'process.json', 'receipt.json'):
        save(attempt / name, dict(status='solver_failed'))
    save(attempt / 'frontend_validation/validation.json', dict(passed=True))
    (attempt / 'frontend_validation/trajectory_samples.npz').write_bytes(b'synthetic bytes, no renderer')
    before = replay.source_snapshot(attempt)
    r = row('failed', 'NTO-distance', success=False); r['attempt'] = str(attempt)
    projection = replay.seed_projection(r, tmp_path / 'projection', dict(files=before))
    assert replay.source_snapshot(attempt) == before
    assert projection['role'] == 'verified_frontend_seed_only'
    for name, data in projection['files'].items():
        assert replay.sha(tmp_path / 'projection' / name) == data['sha256']
    record = dict(complete=True, selection=dict(shared_case_for_five_methods=True,
        common_query_straight_line_collision=False, missing=[], unavailable_optional=[]),
        illustrations=[dict(name='failure_seed_only', role='verified_frontend_seed_only', case_id='failed',
            roadmap_seed=0, source_status='solver_failed', straight_line_collision=True, status='created')])
    replay.gallery(record, tmp_path)
    page = (tmp_path / 'index.html').read_text()
    assert 'SEED ONLY' in page and 'straight-line collision label is False' in page
