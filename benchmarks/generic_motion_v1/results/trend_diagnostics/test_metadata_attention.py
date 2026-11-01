"""Metadata attention rules; tiny JSON only, no trajectory/collision/solver calls."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_completed_evaluation as audit

PROCESS = {'exit_code': 0, 'timed_out': False}


@pytest.mark.parametrize('status', ['success_validated_numerical', 'solver_failed', 'collision_failed', 'frontend_failed'])
def test_normal_success_nonconvergence_and_reported_failures_are_not_exception_alerts(status):
    assert audit.metadata_issues({'status': status}, PROCESS) == []


def test_only_exact_known_drm_failure_is_allowed():
    result = dict(status='frontend_failed', failed_stage='drm_path',
                  exception=repr(RuntimeError(audit.KNOWN_DRM_NO_ROUTE)))
    assert audit.metadata_issues(result, PROCESS) == []
    for changed in (dict(result, exception=repr(RuntimeError(audit.KNOWN_DRM_NO_ROUTE+' CUDA fault'))),
                    dict(result, exception=repr(ValueError(audit.KNOWN_DRM_NO_ROUTE))),
                    dict(result, exception=repr(RuntimeError('Native sidecar ABI/layout disagrees'))),
                    dict(result, status='success_validated_numerical')):
        assert audit.metadata_issues(changed, PROCESS)


def test_ompl_no_exact_solution_requires_correct_class_status_message_and_phase():
    result = dict(status='planner_timeout', failed_stage='sampling_planning', native_planner_status='Approximate solution',
                  exception="OmplNoExactSolution('OMPL did not return an exact solution: Approximate solution')")
    assert audit.metadata_issues(result, PROCESS) == []
    for changed in (dict(result, native_planner_status='Timeout'), dict(result, failed_stage='drm_path'),
                    dict(result, exception="RuntimeError('OMPL did not return an exact solution: Approximate solution')")):
        assert audit.metadata_issues(changed, PROCESS)


def test_known_budget_timeout_is_not_an_unknown_exception_but_cuda_cannot_hide_in_timeout():
    for message in audit.KNOWN_TIMEOUT_MESSAGES | {'Online deadline before validation'}:
        result = dict(status='timeout', exception=repr(TimeoutError(message)))
        assert audit.metadata_issues(result, PROCESS) == []
    for message in ('CUDA launch failure', 'unknown internal timeout', 'Online deadline before invented_phase'):
        issues = audit.metadata_issues(dict(status='timeout', exception=repr(TimeoutError(message))), PROCESS)
        assert issues
    issues = audit.metadata_issues(dict(status='timeout', error_category='CUDA_error'), PROCESS)
    assert any(item['check'] == 'cuda_error_even_if_timeout' for item in issues)


def test_unknown_exception_and_native_crash_are_attention_even_on_timeout():
    assert audit.metadata_issues(dict(status='timeout', exception="KeyError('node42')"), PROCESS)
    assert audit.metadata_issues(dict(status='timeout'), {'exit_code': -11, 'timed_out': True})
    assert audit.metadata_issues(dict(status='timeout'), {'exit_code': -15, 'timed_out': True}) == []


def saved_metadata(tmp_path, *, status='solver_failed', before=None):
    run = tmp_path/'run'
    attempt = run/'jobs/j1/attempts/0001'
    attempt.mkdir(parents=True)
    (run/'run_manifest.json').write_text(json.dumps({'source_code_commit': 'frozen-fixture', 'jobs': [{'job_id': 'j1'}]}))
    documents = {'result.json': {'status': status}, 'process.json': PROCESS}
    if before is not None:
        documents['result_before_external_stop.json'] = before
    for name, value in documents.items():
        (attempt/name).write_text(json.dumps(value))
    files = {name: audit.sha(attempt/name) for name in documents}
    # Deliberately absent: metadata-only checking must never open arrays.
    files['trajectory_samples.npz'] = 'not-opened-by-metadata-audit'
    (attempt/'receipt.json').write_text(json.dumps({'status': status, 'files': files}))
    return run, attempt


def test_metadata_only_cli_reads_no_arrays_or_full_statistics(tmp_path, monkeypatch):
    run, _ = saved_metadata(tmp_path)
    monkeypatch.setattr(audit, 'review', lambda *a, **kw: pytest.fail('Must not invoke full acceptance'))
    output = tmp_path/'metadata.json'
    assert audit.main(['--metadata-only', '--run', str(run), '--output', str(output)]) == 0
    report = json.loads(output.read_text())
    assert report['passed'] and report['sealed_attempts'] == 1
    assert report['final_statistics_produced'] is False


def test_pre_stop_cuda_exception_is_found_without_reclassifying_original_evidence(tmp_path):
    run, attempt = saved_metadata(tmp_path, status='timeout',
        before={'status': 'solver_failed', 'exception': "RuntimeError('CUDA launch failed')"})
    before = {path.name: path.read_bytes() for path in attempt.iterdir()}
    report = audit.review_metadata(run)
    assert not report['passed'] and report['attention_required']
    assert any(item['source'] == 'result_before_external_stop.json' and item['check'] == 'cuda_error_even_if_timeout'
               for item in report['errors'])
    assert before == {path.name: path.read_bytes() for path in attempt.iterdir()}


def test_metadata_corruption_and_unknown_terminal_status_are_rejected(tmp_path):
    run, attempt = saved_metadata(tmp_path)
    (attempt/'result.json').write_text('{malformed')
    assert not audit.review_metadata(run)['passed']
    run2, _ = saved_metadata(tmp_path/'unknown', status='unrecognized')
    assert not audit.review_metadata(run2)['passed']


def test_interrupted_and_unsealed_attempts_are_not_fabricated_planning_exceptions(tmp_path):
    run, attempt = saved_metadata(tmp_path)
    receipt = json.loads((attempt/'receipt.json').read_text())
    receipt['retry_pending'] = True
    (attempt/'receipt.json').write_text(json.dumps(receipt))
    (attempt.parent/'0002').mkdir()
    report = audit.review_metadata(run)
    assert report['passed'] and report['sealed_attempts'] == 0
    assert report['interrupted_attempts'] == report['unsealed_attempts'] == 1


def test_formal_acceptance_stops_before_statistics_on_hidden_exception(tmp_path, monkeypatch):
    run, _ = saved_metadata(tmp_path, status='timeout',
        before={'status': 'frontend_failed', 'exception': "KeyError('unknown roadmap node')"})
    monkeypatch.setattr(audit.batch, 'verify', lambda *a, **kw: {'complete': True, 'integrity_passed': True})
    monkeypatch.setattr(audit.results, 'collect', lambda *a, **kw: pytest.fail('Attention must block final statistics'))
    report = audit.review([run], [{'id': 'new'}], records_path=tmp_path/'absent.csv', summary_path=tmp_path/'absent.csv')
    assert report['status'] == 'failed' and not report['passed']
    assert report['metadata_checks'][0]['attention_required']
    assert any(item['check'] == 'unknown_exception_even_if_timeout' for item in report['errors'])


def handoff_fixture(tmp_path):
    run, attempt = saved_metadata(tmp_path)
    result = dict(status='solver_failed', failed_stage='collision_recovery',
                  exception=repr(ValueError(audit.KNOWN_HANDOFF_FAILURE)))
    (attempt/'result.json').write_text(json.dumps(result))
    manifest = json.loads((run/'run_manifest.json').read_text())
    manifest['source_hashes'] = {audit.HANDOFF_SOURCE: audit.HANDOFF_SOURCE_SHA256}
    (run/'run_manifest.json').write_text(json.dumps(manifest))
    trace = ('Traceback (most recent call last):\n'
        f'  File "/checkout/{audit.HANDOFF_SOURCE}", line 117, in native_region_with_anchors\n'
        f'    raise ValueError("{audit.KNOWN_HANDOFF_FAILURE}")\n'
        f'ValueError: {audit.KNOWN_HANDOFF_FAILURE}\n')
    (attempt/'exception.txt').write_text(trace)
    path = attempt/'recovery/000/native_handoff/0005/handoff.json'
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps(dict(status='rejected', error=result['exception'],
        fixed_anchors_changed=False, native_predicate_tolerance=0., native_refinement_rounds=16,
        native_anchor_containment_after=[True, False], allowed_normalized_correction=1e-6,
        maximum_normalized_correction=1.19e-7, maximum_total_normalized_offset_change_from_input=1.20e-7)))
    seal_fixture(attempt)
    return run, attempt, path


def seal_fixture(attempt):
    receipt = json.loads((attempt/'receipt.json').read_text())
    for path in attempt.rglob('*'):
        if path.is_file() and path.name != 'receipt.json':
            receipt['files'][str(path.relative_to(attempt))] = audit.sha(path)
    (attempt/'receipt.json').write_text(json.dumps(receipt))


def test_exact_bound_handoff_failure_is_visible_but_not_unknown(tmp_path):
    run, attempt, _ = handoff_fixture(tmp_path)
    original = {str(p): p.read_bytes() for p in attempt.rglob('*') if p.is_file()}
    report = audit.review_metadata(run)
    assert report['passed'] and not report['attention_required']
    evidence, = report['reviewed_numerical_failures']
    assert evidence['source_line'] == 117 and evidence['raw_status_unchanged']
    assert evidence['result_normalization_unchanged']
    assert original == {str(p): p.read_bytes() for p in attempt.rglob('*') if p.is_file()}


@pytest.mark.parametrize('fault', ['stage', 'status', 'message', 'source_hash', 'source_path',
    'source_line', 'function', 'trace_unbound', 'handoff_unbound', 'anchors', 'bounds', 'all_contained',
    'rounds', 'no_evidence'])
def test_handoff_exemption_requires_exact_frozen_location_and_bound_evidence(tmp_path, fault):
    run, attempt, path = handoff_fixture(tmp_path)
    if fault in ('stage', 'status', 'message'):
        result = json.loads((attempt/'result.json').read_text())
        key, value = {'stage': ('failed_stage', 'trajectory_optimization'),
            'status': ('status', 'timeout'), 'message': ('exception', repr(ValueError(audit.KNOWN_HANDOFF_FAILURE+' extra')))}[fault]
        result[key] = value
        (attempt/'result.json').write_text(json.dumps(result))
        receipt = json.loads((attempt/'receipt.json').read_text()); receipt['status'] = result['status']
        (attempt/'receipt.json').write_text(json.dumps(receipt))
    elif fault == 'source_hash':
        manifest = json.loads((run/'run_manifest.json').read_text())
        manifest['source_hashes'][audit.HANDOFF_SOURCE] = 'unreviewed-source'
        (run/'run_manifest.json').write_text(json.dumps(manifest))
    elif fault in ('source_path', 'source_line', 'function', 'trace_unbound'):
        old, new = {'source_path': ('/runners/native_handoff.py', '/runners/unknown.py'),
            'source_line': ('line 117', 'line 118'), 'function': ('native_region_with_anchors', 'unknown'),
            'trace_unbound': ('Traceback', 'modified Traceback')}[fault]
        target = attempt/'exception.txt'; target.write_text(target.read_text().replace(old, new))
    elif fault == 'no_evidence':
        path.unlink()
        receipt = json.loads((attempt/'receipt.json').read_text())
        del receipt['files'][str(path.relative_to(attempt))]
        (attempt/'receipt.json').write_text(json.dumps(receipt))
    else:
        record = json.loads(path.read_text())
        key, value = {'anchors': ('fixed_anchors_changed', True),
            'bounds': ('maximum_normalized_correction', 1e-5),
            'all_contained': ('native_anchor_containment_after', [True, True]),
            'rounds': ('native_refinement_rounds', 15), 'handoff_unbound': ('extra', True)}[fault]
        record[key] = value; path.write_text(json.dumps(record))
    if fault not in ('trace_unbound', 'handoff_unbound'):
        seal_fixture(attempt)
    report = audit.review_metadata(run)
    assert not report['passed'] and report['attention_required']
    assert not report['reviewed_numerical_failures']


def typed_handoff_fixture(tmp_path, code='bounded_predicate_rejected'):
    run, attempt, path = handoff_fixture(tmp_path)
    manifest = json.loads((run/'run_manifest.json').read_text())
    manifest['source_hashes'][audit.HANDOFF_SOURCE] = 'a'*64
    manifest['jobs'][0]['evaluation_config'] = {'region_coverage_tolerance_rad': 1e-6}
    (run/'run_manifest.json').write_text(json.dumps(manifest))
    record = json.loads(path.read_text())
    if code == 'bounded_predicate_rejected':
        message = audit.failures.HANDOFF_BOUNDED_MESSAGE
        function = 'bounded_native_offset_search'
        record['bounded_native_offset_search'] = [dict(row=14, initial_offset=-.1,
            maximum_permitted_offset=-.099999, native_predicate_calls=2,
            status='no_passing_offset_within_bound')]
    else:
        message = 'Native handoff correction 1.2e-06 exceeds fixed tolerance 1e-06'
        function = 'check_bound'
        record['maximum_total_normalized_offset_change_from_input'] = 1.2e-6
    exception = 'NativeHandoffError('+repr(message)+')'
    record.update(error=exception, error_code=code)
    path.write_text(json.dumps(record))
    result = json.loads((attempt/'result.json').read_text())
    result.update(exception=exception, error_category=audit.failures.HANDOFF_KIND,
        failure_classification=dict(kind=audit.failures.HANDOFF_KIND,
        code=code, source=audit.HANDOFF_SOURCE, source_sha256='a'*64,
        evidence=str(path.relative_to(attempt))))
    (attempt/'result.json').write_text(json.dumps(result))
    (attempt/'exception.txt').write_text('Traceback (most recent call last):\n'
        f'  File "/checkout/{audit.HANDOFF_SOURCE}", line 100, in {function}\n'
        f'    raise NativeHandoffError("{message}")\n{audit.HANDOFF_TRACE_CLASS}: {message}\n')
    seal_fixture(attempt)
    return run, attempt, path


@pytest.mark.parametrize('code', ['bounded_predicate_rejected', 'correction_exceeds_tolerance'])
def test_typed_handoff_uses_shared_rules_and_independent_bound_trace(tmp_path, code):
    run, _, _ = typed_handoff_fixture(tmp_path, code)
    report = audit.review_metadata(run)
    assert report['passed']
    accepted, = report['reviewed_numerical_failures']
    assert accepted['code'] == code and accepted['receipt_verified']
    assert accepted['trace_sha256'] and accepted['raw_status_unchanged']


@pytest.mark.parametrize('fault', ['trace_unbound', 'trace_function', 'no_trace', 'marker_hash',
    'marker_code', 'changed_bound', 'anchor_changed', 'unknown_class', 'handoff_unbound'])
def test_typed_handoff_unknown_or_unbound_evidence_still_blocks(tmp_path, fault):
    run, attempt, path = typed_handoff_fixture(tmp_path)
    if fault in ('trace_unbound', 'trace_function'):
        trace = attempt/'exception.txt'
        trace.write_text(trace.read_text().replace('bounded_native_offset_search', 'unexpected_function'))
    elif fault == 'no_trace':
        (attempt/'exception.txt').unlink()
    elif fault in ('marker_hash', 'marker_code', 'unknown_class'):
        result = json.loads((attempt/'result.json').read_text())
        if fault == 'unknown_class':
            result['exception'] = result['exception'].replace('NativeHandoffError', 'ValueError')
        else:
            result['failure_classification']['source_sha256' if fault == 'marker_hash' else 'code'] = 'unknown'
        (attempt/'result.json').write_text(json.dumps(result))
    elif fault == 'changed_bound':
        manifest = json.loads((run/'run_manifest.json').read_text())
        manifest['jobs'][0]['evaluation_config']['region_coverage_tolerance_rad'] = 2e-6
        (run/'run_manifest.json').write_text(json.dumps(manifest))
    else:
        record = json.loads(path.read_text())
        record['fixed_anchors_changed' if fault == 'anchor_changed' else 'extra'] = True
        path.write_text(json.dumps(record))
    if fault not in ('trace_unbound', 'handoff_unbound'):
        seal_fixture(attempt)
    report = audit.review_metadata(run)
    assert not report['passed'] and not report['reviewed_numerical_failures']


@pytest.fixture(params=['correction_exceeds_tolerance', 'bounded_predicate_rejected'])
def actual_classified_handoff_failure(tmp_path, monkeypatch, request):
    """Actual native CPU guard, actual baseline catch helper and Python trace."""
    import traceback
    import numpy as np
    import csdecomp as csd
    from pydrake.all import HPolyhedron
    from benchmarks.generic_motion_v1.runners.native_handoff import NativeHandoffError, native_region_with_anchors
    from benchmarks.generic_motion_v1.runners.csdecomp_baseline import _exception_outcome

    def no_gpu(*args, **kwargs):
        pytest.fail('Metadata regression must not invoke CUDA')
    for name in ('CudaEdgeInflator', 'EditRegionsCuda', 'CheckCollisionFreeCuda', 'CheckCollisionFreeVoxelsCuda'):
        monkeypatch.setattr(csd, name, no_gpu)
    run, attempt = saved_metadata(tmp_path)
    out = attempt/'recovery/000/native_handoff/0000'
    if request.param == 'bounded_predicate_rejected':
        source = audit.ROOT/('results/repaired_dev_all_seeds/jobs/'
            'dev-dense_clutter-002-p01-LGCS-minimum_distance-83e6f638/attempts/0001/'
            'recovery/000/native_handoff/0005/original.npz')
        with np.load(source) as data:
            region, anchors = HPolyhedron(data['A'], data['b']), data['anchors']
        tolerance = 1.3e-7
    else:
        region = HPolyhedron.MakeBox(-np.ones(7), np.ones(7))
        anchors = np.zeros((2, 7)); anchors[1, 0] = 1.01
        tolerance = 1e-6
    try:
        native_region_with_anchors(region, anchors, tolerance, out)
    except NativeHandoffError as exc:
        assert exc.code == request.param
        result = dict(exception=repr(exc), failed_stage='collision_recovery')
        trace = traceback.format_exc()
        result.update(_exception_outcome(exc, 'collision_recovery', out=attempt,
            config={'region_coverage_tolerance_rad': tolerance}))
        assert result['status'] == 'solver_failed' and result['error_category'] == audit.failures.HANDOFF_KIND
        assert trace.splitlines()[-1] == audit.HANDOFF_TRACE_CLASS+': '+str(exc)
    else:
        pytest.fail('Actual native numerical rejection was not reproduced')
    (attempt/'exception.txt').write_text(trace)
    (attempt/'result.json').write_text(json.dumps(result))
    manifest = json.loads((run/'run_manifest.json').read_text())
    manifest['source_hashes'] = {audit.HANDOFF_SOURCE: audit.sha(audit.ROOT/audit.HANDOFF_SOURCE)}
    manifest['jobs'][0]['evaluation_config'] = {'region_coverage_tolerance_rad': tolerance}
    (run/'run_manifest.json').write_text(json.dumps(manifest))
    seal_fixture(attempt)
    return run, attempt


def test_real_native_failure_baseline_classification_trace_and_receipt_pass(actual_classified_handoff_failure):
    run, attempt = actual_classified_handoff_failure
    before = {str(path): path.read_bytes() for path in attempt.rglob('*') if path.is_file()}
    report = audit.review_metadata(run)
    assert report['passed'], report['errors']
    evidence, = report['reviewed_numerical_failures']
    assert evidence['kind'] == audit.failures.HANDOFF_KIND and evidence['receipt_verified']
    assert before == {str(path): path.read_bytes() for path in attempt.rglob('*') if path.is_file()}


@pytest.mark.parametrize('fault', ['other_category', 'cuda_category', 'missing_evidence', 'bare_class', 'wrong_module'])
def test_real_native_failure_exemption_is_not_generic(actual_classified_handoff_failure, fault):
    run, attempt = actual_classified_handoff_failure
    result = json.loads((attempt/'result.json').read_text())
    if fault in ('bare_class', 'wrong_module'):
        path = attempt/'exception.txt'
        path.write_text(path.read_text().replace(audit.HANDOFF_TRACE_CLASS,
            'NativeHandoffError' if fault == 'bare_class' else 'unreviewed.module.NativeHandoffError'))
    elif fault == 'missing_evidence':
        result.pop('failure_classification')
    else:
        result['error_category'] = 'CUDA_error' if fault == 'cuda_category' else 'unexpected_backend_exception'
    (attempt/'result.json').write_text(json.dumps(result))
    seal_fixture(attempt)
    report = audit.review_metadata(run)
    assert not report['passed'] and report['attention_required']
    assert any(item['check'] == 'error_category_even_if_timeout' for item in report['errors'])
