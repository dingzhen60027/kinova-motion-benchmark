"""Small CPU fixtures only; run after formal planning finishes."""
import copy
import json

import pytest

from benchmarks.generic_motion_v1.reports.baseline_study.make_formal_replays import (
    LABELS, SUCCESS, completion_barrier, label_replay, seed_projection, select_illustrations, sha)


def row(case, label, blocked=True, status=SUCCESS, seed_valid=True):
    return dict(case_id=case, roadmap_seed=0, roadmap_id='map0', method_seed=123,
        job_id=case+'-'+label, label=label, straight_line_collision=blocked,
        family='sparse_clutter', input_legal=True, success=status == SUCCESS,
        status=status, drm_success=seed_valid, seed_cost_validated=seed_valid)


def test_blocked_common_pair_preferred_and_eizo_views_reused():
    rows = [row(case, label, blocked) for case, blocked in [('aaa', False), ('zzz', True)]
            for label in LABELS]
    first, meta = select_illustrations(rows)
    reversed_choices, _ = select_illustrations(list(reversed(rows)))
    assert first == reversed_choices and meta['shared_case_for_five_methods']
    assert {item['row']['case_id'] for item in first} == {'zzz'}
    assert {item['name']: item.get('reuse_of') for item in first}['EI_ZO_distance_blocked'] == 'LSCS'
    assert {item['name']: item.get('reuse_of') for item in first}['EI_ZO_time_blocked'] == 'SCSTO'


def test_no_common_pair_keeps_unblocked_nto_honest():
    rows = [row(label, label, label != 'NTO-distance') for label in LABELS]
    choices, meta = select_illustrations(rows)
    assert not meta['shared_case_for_five_methods']
    nto = next(item for item in choices if item['name'] == 'NTO-distance')
    assert nto['row']['straight_line_collision'] is False
    assert {item['name'] for item in choices}.issuperset(LABELS)


def test_eizo_blocked_views_added_when_shared_case_is_unblocked():
    rows = [row('aaa-common', label, False) for label in LABELS]
    rows += [row('zzz-blocked', label) for label in ('LSCS', 'SCSTO')]
    choices, meta = select_illustrations(rows)
    assert meta['shared_case_for_five_methods']
    assert next(item for item in choices if item['name'] == 'LSCS')['row']['case_id'] == 'aaa-common'
    for name in ('EI_ZO_distance_blocked', 'EI_ZO_time_blocked'):
        item = next(item for item in choices if item['name'] == name)
        assert item['row']['case_id'] == 'zzz-blocked' and 'reuse_of' not in item


def test_failure_needs_verified_seed_and_prefers_declared_category():
    rows = [row(label, label) for label in LABELS]
    rows += [row('a-unsupported', 'LGCS', status='unsupported'),
             row('z-collision', 'NTO-time', status='collision_failed'),
             row('a-no-seed', 'NTO-distance', status='collision_failed', seed_valid=False)]
    choices, _ = select_illustrations(rows)
    failure = next(item for item in choices if item['name'] == 'failure_seed_only')
    assert failure['row']['case_id'] == 'z-collision'
    assert failure['role'] == 'verified_frontend_seed_only'


def test_missing_success_or_blocked_views_not_fabricated():
    rows = [row(label, label, False) for label in LABELS if label != 'NTO-distance']
    choices, meta = select_illustrations(rows)
    assert 'NTO-distance' not in {item['name'] for item in choices}
    assert {item['label'] for item in meta['missing']} == {
        'NTO-distance', 'EI_ZO_distance_blocked', 'EI_ZO_time_blocked', 'failure_seed_only'}


def test_explicit_post_planning_flag_required_before_any_file_read(tmp_path):
    with pytest.raises(ValueError, match='planning-finished'):
        completion_barrier(tmp_path/'nonexistent-index', tmp_path/'nonexistent-done', False)


def test_seed_projection_preserves_original_bytes_and_cannot_overwrite(tmp_path):
    source = tmp_path/'sealed-attempt'
    (source/'frontend_validation').mkdir(parents=True)
    files = {'input.json': b'{"case":"fixture"}',
             'result.json': b'{"status":"solver_failed"}', 'process.json': b'{}',
             'frontend_validation/trajectory_samples.npz': b'fixture-only-no-rendering',
             'frontend_validation/validation.json': b'{"passed":true}'}
    for name, payload in files.items():
        (source/name).write_bytes(payload)
    receipt = {'files': {name: sha(source/name) for name in files}}
    (source/'receipt.json').write_text(json.dumps(receipt))
    before = copy.deepcopy(files)
    target = tmp_path/'report-projection'
    provenance = seed_projection(dict(attempt=str(source), status='solver_failed'), target, receipt)
    assert provenance['role'] == 'verified_frontend_seed_only'
    assert all((source/name).read_bytes() == payload for name, payload in before.items())
    for copied, ref in provenance['files'].items():
        assert sha(target/copied) == ref['sha256']
    with pytest.raises(FileExistsError):
        seed_projection(dict(attempt=str(source), status='solver_failed'), target, receipt)


def test_derived_html_preserves_json_strings_and_labels_seed_only(tmp_path):
    output = tmp_path/'derived-replay'
    output.mkdir()
    data = dict(sample_source='trajectory_samples.npz', fixture='</script><div>literal text</div>')
    (output/'replay.json').write_text(json.dumps(data))
    saved = label_replay(output, dict(attempt='/fixture/original', status='collision_failed',
        label='NTO-distance', run='/fixture/run', straight_line_collision=True),
        'verified_frontend_seed_only', 'fixture-receipt-sha')
    page = (output/'replay.html').read_text()
    embedded = page.split('<script id="data" type="application/json">', 1)[1].split('</script>', 1)[0]
    assert json.loads(embedded)['fixture'] == data['fixture']
    assert r'\u003c/script>' in embedded and '</script>' not in embedded
    assert 'SEED ONLY' in page and saved['display_role'] == 'verified_frontend_seed_only'
    assert saved['original_status'] == 'collision_failed'


def test_success_view_rejects_subdirectory_fallback_samples(tmp_path):
    (tmp_path/'replay.json').write_text(json.dumps(dict(
        sample_source='frontend_validation/trajectory_samples.npz')))
    with pytest.raises(ValueError, match='saved final samples'):
        label_replay(tmp_path, dict(attempt='/fixture/original', status=SUCCESS, label='LSCS',
            run='/fixture/run', straight_line_collision=True), 'validated_final_trajectory', 'fixture-sha')
