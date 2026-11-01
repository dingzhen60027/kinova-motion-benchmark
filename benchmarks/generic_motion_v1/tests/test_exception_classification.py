"""Status-only CPU regressions: no geometry changes, optimizer or GPU work."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import pytest
from benchmarks.generic_motion_v1.runners.csdecomp_baseline import _exception_outcome, _apply_deadline_outcome
from benchmarks.generic_motion_v1.batch import _external_stop_outcome
from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.common import ROOT, read, sha, write
from benchmarks.generic_motion_v1.runners import csdecomp_baseline as runner, ompl_baselines as ompl
from benchmarks.generic_motion_v1.runners.failure_classification import (
    HANDOFF_KIND, HANDOFF_SOURCE, exception_needs_attention, handoff_failure_evidence,
    known_exception_category, unexpected_process_exit, LEGACY_DRM_BUILD,
    LEGACY_DRM_NO_ROUTE, LEGACY_DRM_SOURCE, LEGACY_DRM_SOURCE_SHA256, legacy_drm_failure_evidence)
from benchmarks.generic_motion_v1.runners.native_handoff import NativeHandoffError, native_region_with_anchors
from benchmarks.generic_motion_v1.tests.test_batch import prepared
from benchmarks.generic_motion_v1.tests.test_extended_integration import mock_execution


def test_exact_native_no_route_retains_normal_search_failure():
    error = RuntimeError('[DRMPlanner] No route found through tested roadmap connections.')
    assert _exception_outcome(error, 'drm_path') == {'status': 'frontend_failed'}
    for unknown in (ValueError(str(error)), RuntimeError(str(error)+' unexpected suffix'),
                    KeyError('unknown node'), RuntimeError('Native sidecar ABI/layout disagrees')):
        result = _exception_outcome(unknown, 'drm_path')
        assert result == {'status': 'infrastructure_error', 'error_category': 'unexpected_frontend_exception'}


@pytest.mark.parametrize('error', [RuntimeError('CUDA launch failure'), TimeoutError('CUDA operation failed')])
def test_cuda_cannot_be_disguised_as_timeout(error):
    result = _exception_outcome(error, 'drm_path')
    assert result == {'status': 'infrastructure_error', 'error_category': 'CUDA_error'}
    result['exception'] = repr(error)
    _apply_deadline_outcome(result, True)
    assert result['status'] == 'infrastructure_error' and result['deadline_exceeded'] is True
    _external_stop_outcome(result, timed_out=True, returncode=-15)
    assert result['status'] == 'infrastructure_error' and result['error_category'] == 'CUDA_error'
    assert result['exception'] == repr(error)


@pytest.mark.parametrize('status', ['infrastructure_error', 'unsupported', 'model_consistency_failed'])
def test_existing_failure_survives_both_deadline_layers(status):
    result = {'status': status, 'exception': "RuntimeError('already observed failure')"}
    _apply_deadline_outcome(result, True)
    _external_stop_outcome(result, timed_out=True, returncode=-15)
    assert result['status'] == status and result['deadline_exceeded']


def test_normal_budget_nonconvergence_and_validation_outcomes_are_unchanged():
    assert _exception_outcome(TimeoutError('NTO construction exhausted online budget'), 'trajectory_optimization') == {'status': 'timeout'}
    assert _exception_outcome(ValueError('Numerical sampling cap exceeded'), 'validation') == {'status': 'validation_undetermined'}
    ordinary = {'status': 'solver_failed', 'metrics': {'solver_status': 'Iteration limit'}}
    before = copy.deepcopy(ordinary)
    _apply_deadline_outcome(ordinary, False)
    assert ordinary == before
    _apply_deadline_outcome(ordinary, True)
    assert ordinary['status'] == 'timeout'
    running = {'status': 'running'}
    _external_stop_outcome(running, timed_out=True, returncode=-15)
    assert running['status'] == 'timeout' and running['error'] == 'External job deadline'


def test_ompl_exact_solution_failure_and_native_signal_semantics_are_unchanged():
    assert _exception_outcome(ompl.OmplNoExactSolution('Approximate solution'), 'sampling_planning') == {
        'status': 'planner_timeout', 'native_planner_status': 'Approximate solution'}
    result = {'status': 'running'}
    _external_stop_outcome(result, timed_out=False, returncode=-11)
    assert result['status'] == 'infrastructure_error' and result['signal_number'] == 11


@pytest.mark.parametrize('phase', ['drm_path', 'trajectory_optimization', 'collision_recovery', 'eizo_inflation', 'validation'])
@pytest.mark.parametrize('error', [KeyError('unexpected backend state'), ValueError('unreviewed numerical failure'),
                                  TimeoutError('unknown internal deadline')])
def test_unknown_exceptions_never_turn_into_search_or_solver_or_timeout_failures(phase, error):
    result = dict(exception=repr(error), failed_stage=phase, **_exception_outcome(error, phase))
    assert result['status'] == 'infrastructure_error'
    _apply_deadline_outcome(result, True)
    _external_stop_outcome(result, timed_out=True, returncode=-15)
    assert result['status'] == 'infrastructure_error'


@pytest.mark.parametrize('status', ['Invalid start', 'Invalid goal', 'Unrecognized', 'Approximate solution extra'])
def test_ompl_invalid_or_unrecognized_status_is_not_ordinary_search_failure(status):
    error = ompl.OmplNoExactSolution(status)
    assert _exception_outcome(error, 'sampling_planning')['status'] == 'infrastructure_error'
    saved = dict(status='planner_failed', native_planner_status=status,
                 exception=repr(error), failed_stage='sampling_planning')
    assert known_exception_category(saved) is None and exception_needs_attention(saved)


def test_unknown_error_category_and_crash_survive_external_timeout():
    result = dict(status='timeout', error_category='unreviewed_backend_error')
    assert exception_needs_attention(result)
    _external_stop_outcome(result, timed_out=True, returncode=-15)
    assert result['status'] == 'infrastructure_error'
    crash = dict(status='running')
    _external_stop_outcome(crash, timed_out=True, returncode=-11)
    assert crash['status'] == 'infrastructure_error' and crash['signal_number'] == 11
    assert unexpected_process_exit(dict(exit_code=-11, timed_out=True))
    assert not unexpected_process_exit(dict(exit_code=-15, timed_out=True))


@pytest.mark.parametrize('error', [RuntimeError('CUDA device failure'), TimeoutError('CUDA operation failed'),
                                 KeyError('unknown planner state'), TimeoutError('unknown internal deadline')])
def test_actual_worker_catch_preserves_exception_and_infrastructure_status(mock_execution, tmp_path, monkeypatch, error):
    def fail(*args, **kwargs):
        raise error
    monkeypatch.setattr(ompl, 'solve', fail)
    out = tmp_path/'attempt'
    result = runner.run_case(mock_execution, None, out, method='RRTCONNECT')
    assert result['status'] == 'infrastructure_error'
    assert result['exception'] == repr(error) and result['failed_stage'] == 'sampling_planning'
    assert read(out/'result.json')['status'] == 'infrastructure_error'
    assert type(error).__name__ in (out/'exception.txt').read_text()


@pytest.mark.parametrize('initial', [dict(status='infrastructure_error', error_category='CUDA_error',
                                       exception="RuntimeError('CUDA device failure')"),
                                    dict(status='solver_failed', exception="KeyError('hidden backend failure')",
                                         failed_stage='trajectory_optimization')])
def test_real_external_timeout_keeps_prior_backend_error_and_receipt(tmp_path, monkeypatch, initial):
    run, _, _ = prepared(tmp_path, monkeypatch)
    original = subprocess.Popen
    def worker(argv, **kwargs):
        attempt = argv[argv.index('--attempt')+1]
        script = ('from pathlib import Path\nimport json,time\n'
                  'Path('+repr(attempt)+',"result.json").write_text('+repr(json.dumps(initial))+')\n'
                  'time.sleep(30)\n')
        return original([sys.executable, '-c', script], **kwargs)
    monkeypatch.setattr(batch.subprocess, 'Popen', worker)
    monkeypatch.setattr(batch, '_gpu_memory', lambda pid: None)
    job = read(run/'run_manifest.json')['jobs'][0]
    result = batch.execute(job, run, .3)
    attempt = run/'jobs'/job['job_id']/'attempts/0001'
    assert result['status'] == 'infrastructure_error' and result['deadline_exceeded']
    assert read(attempt/'process.json')['timed_out'] is True
    assert read(attempt/'result_before_external_stop.json') == initial
    assert read(attempt/'receipt.json')['status'] == 'infrastructure_error'
    assert result['exception'] == initial['exception']


def real_handoff_failure(attempt):
    """Invoke the actual CPU native predicate and finite correction guard."""
    import numpy as np
    from pydrake.geometry.optimization import HPolyhedron
    region = HPolyhedron(np.vstack([np.eye(7), -np.eye(7)]), np.ones(14))
    anchors = np.zeros((1, 7)); anchors[0, 0] = 1.01
    directory = attempt/'recovery/000/native_handoff/0000'
    with pytest.raises(NativeHandoffError) as caught:
        native_region_with_anchors(region, anchors, 1e-6, directory)
    error = caught.value
    fields = _exception_outcome(error, 'collision_recovery', out=attempt,
                                config={'region_coverage_tolerance_rad':1e-6})
    return dict(exception=repr(error), failed_stage='collision_recovery', **fields)


def test_real_typed_handoff_rejection_requires_saved_bound_evidence(tmp_path):
    result = real_handoff_failure(tmp_path)
    assert result['status'] == 'solver_failed' and result['error_category'] == HANDOFF_KIND
    source = {HANDOFF_SOURCE:sha(ROOT/HANDOFF_SOURCE)}
    marker = result['failure_classification']
    receipt = {'files': {marker['evidence']:sha(tmp_path/marker['evidence'])}}
    evidence = handoff_failure_evidence(tmp_path, result, expected_tolerance=1e-6,
                                       source_hashes=source, receipt=receipt)
    assert evidence['code'] == 'correction_exceeds_tolerance' and evidence['receipt_verified']
    assert known_exception_category(result, handoff_evidence=evidence) == HANDOFF_KIND
    assert not exception_needs_attention(result, handoff_evidence=evidence)
    assert exception_needs_attention(result)
    for changed in (dict(result, exception=result['exception'].replace('NativeHandoffError','ValueError')),
                    dict(result, failed_stage='validation')):
        assert handoff_failure_evidence(tmp_path, changed, expected_tolerance=1e-6, source_hashes=source, receipt=receipt) is None
    assert handoff_failure_evidence(tmp_path, result, expected_tolerance=1e-5, source_hashes=source, receipt=receipt) is None
    assert handoff_failure_evidence(tmp_path, result, expected_tolerance=1e-6, source_hashes={}, receipt=receipt) is None
    assert handoff_failure_evidence(tmp_path, result, expected_tolerance=1e-6, source_hashes=source, receipt={'files':{}}) is None
    error = NativeHandoffError('Native handoff correction 0.01 exceeds fixed tolerance 1e-06',
                              code='correction_exceeds_tolerance')
    assert _exception_outcome(error, 'collision_recovery', out=tmp_path, config={})['status'] == 'infrastructure_error'


def legacy_drm_fixture(attempt):
    result = dict(status='frontend_failed', failed_stage='drm_path', exception=repr(RuntimeError(LEGACY_DRM_NO_ROUTE)))
    query = dict(LEGACY_DRM_BUILD, exception=result['exception'], exception_type='RuntimeError',
        status='native_exception', query_success=False, cuda_changed=False, endpoint_change=False,
        search_budget_changed=False)
    write(attempt/'query.json', query)
    write(attempt/'build_receipt.json', dict(LEGACY_DRM_BUILD, status='ready'))
    (attempt/'exception.txt').write_text('Traceback (most recent call last):\n'
        f'  File "/old/checkout/{LEGACY_DRM_SOURCE}", line 194, in query\n'
        '    success,path=module.plan(planner,start,goal,np.asfortranarray(voxels.T),float(radius))\n'
        f'RuntimeError: {LEGACY_DRM_NO_ROUTE}\n')
    receipt = {'files':{name:sha(attempt/name) for name in ('query.json','build_receipt.json','exception.txt')}}
    return result, receipt, {LEGACY_DRM_SOURCE:LEGACY_DRM_SOURCE_SHA256}


def test_legacy_drm_message_is_only_accepted_by_bound_reader_evidence(tmp_path):
    result, receipt, sources = legacy_drm_fixture(tmp_path)
    evidence = legacy_drm_failure_evidence(tmp_path, result, source_hashes=sources, receipt=receipt)
    assert evidence and not exception_needs_attention(result, legacy_drm_evidence=evidence)
    assert exception_needs_attention(result)
    assert _exception_outcome(RuntimeError(LEGACY_DRM_NO_ROUTE),'drm_path')['status'] == 'infrastructure_error'


@pytest.mark.parametrize('fault', ['source_hash','trace_line','trace_function','trace_source','message','phase',
                                   'raw_status','binary_hash','native_hash','query_result','unbound_trace','unbound_query'])
def test_legacy_drm_exemption_rejects_any_unbound_or_different_origin(tmp_path, fault):
    result, receipt, sources = legacy_drm_fixture(tmp_path)
    if fault == 'source_hash': sources[LEGACY_DRM_SOURCE]='0'*64
    elif fault.startswith('trace_'):
        path=tmp_path/'exception.txt'; text=path.read_text()
        old,new={'trace_line':('line 194','line 195'), 'trace_function':('in query','in other'),
                 'trace_source':('native_drm_query.py','other.py')}[fault]
        path.write_text(text.replace(old,new)); receipt['files']['exception.txt']=sha(path)
    elif fault in ('message','phase','raw_status'):
        key,value={'message':('exception',repr(ValueError(LEGACY_DRM_NO_ROUTE))),
                   'phase':('failed_stage','validation'),'raw_status':('status','timeout')}[fault]
        result[key]=value
    elif fault in ('binary_hash','native_hash','query_result'):
        path=tmp_path/'query.json';query=read(path)
        key,value={'binary_hash':('binary_sha256','0'*64),'native_hash':('native_library_sha256','0'*64),
                   'query_result':('query_success',True)}[fault]
        query[key]=value;write(path,query);receipt['files']['query.json']=sha(path)
    else:
        receipt['files'].pop('exception.txt' if fault=='unbound_trace' else 'query.json')
    assert legacy_drm_failure_evidence(tmp_path,result,source_hashes=sources,receipt=receipt) is None


def test_real_call_sites_use_status_helpers_and_keep_pre_stop_evidence():
    # AST-level integration verification is added by the deferred staging runner;
    # ordinary post-application pytest exercises the real imported helpers above.
    from pathlib import Path
    import ast
    root = next(path for path in Path(__file__).resolve().parents if (path/'scripts/evaluate.py').is_file())
    baseline = ast.parse((root/'benchmarks/generic_motion_v1/runners/csdecomp_baseline.py').read_text())
    run_case = next(node for node in baseline.body if isinstance(node, ast.FunctionDef) and node.name == 'run_case')
    called = {node.func.id for node in ast.walk(run_case) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
    assert {'_exception_outcome', '_apply_deadline_outcome'} <= called
    batch = ast.parse((root/'benchmarks/generic_motion_v1/batch.py').read_text())
    execute = next(node for node in batch.body if isinstance(node, ast.FunctionDef) and node.name == 'execute')
    assert any(isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == '_external_stop_outcome' for node in ast.walk(execute))
    text = ast.get_source_segment((root/'benchmarks/generic_motion_v1/batch.py').read_text(), execute)
    assert text.index('result_before_external_stop.json') < text.index('_external_stop_outcome(')
