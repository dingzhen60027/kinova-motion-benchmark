"""Independent CPU-only audit of execution/reporting exception classification."""
from pathlib import Path
import copy
import hashlib
import json
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
from benchmarks.generic_motion_v1.runners import failure_classification as f
from benchmarks.generic_motion_v1.runners.csdecomp_baseline import _exception_outcome, _apply_deadline_outcome
from benchmarks.generic_motion_v1.batch import _external_stop_outcome
from benchmarks.generic_motion_v1.runners.results import attempt_row
from benchmarks.generic_motion_v1.tests.test_results import make_run, make_attempt, _reseal
from benchmarks.generic_motion_v1.tests.test_exception_classification import real_handoff_failure, legacy_drm_fixture
from benchmarks.generic_motion_v1.common import read, write, sha

OUT=ROOT/'results/trend_diagnostics/failure_classification_independent'
OUT.mkdir(exist_ok=True)
checks=[]
started=time.monotonic()

with tempfile.TemporaryDirectory(prefix='csdecomp_failure_review_') as temp:
    base=Path(temp)
    # Exact known strings are not substring whitelists; fake exception classes
    # and unknown timeout sources remain infrastructure outcomes at each layer.
    errors=[RuntimeError('CUDA launch failure'),TimeoutError('cUdA deadline'),
            KeyError('internal backend state'),RuntimeError('unknown native error'),
            TimeoutError('unreviewed deadline'),ValueError(f.KNOWN_DRM_NO_ROUTE),
            RuntimeError(f.KNOWN_DRM_NO_ROUTE+' suffix')]
    stages=['drm_path','collision_recovery','trajectory_optimization','validation']
    for i,error in enumerate(errors):
        for phase in stages:
            result=dict(exception=repr(error),failed_stage=phase,**_exception_outcome(error,phase))
            assert result['status']=='infrastructure_error'
            _apply_deadline_outcome(result,True)
            assert result['status']=='infrastructure_error'
            _external_stop_outcome(result,timed_out=True,returncode=-15)
            assert result['status']=='infrastructure_error' and result['exception']==repr(error)
    checks.append(dict(name='unknown_cuda_survive_worker_and_external_deadlines',passed=True,cases=len(errors)*len(stages)))
    for n,raw in enumerate(('solver_failed','timeout','frontend_failed','validation_undetermined')):
        for timed_out in (False,True):
            run,manifest,job=make_run(base,name=f'row_{n}_{timed_out}')
            attempt=make_attempt(run,job,status=raw,valid=False)
            result=read(attempt/'result.json');result.update(exception="RuntimeError('unknown CUDA backend error')",failed_stage='trajectory_optimization')
            write(attempt/'result.json',result)
            process=read(attempt/'process.json');process.update(timed_out=timed_out,exit_code=-15 if timed_out else 0,cold_e2e_seconds=61.)
            write(attempt/'process.json',process);_reseal(attempt)
            row=attempt_row(run,manifest,job,attempt)
            assert row['status']=='infrastructure_error' and row['objective_value'] is None
    checks.append(dict(name='sealed_results_unknown_cuda_never_normalized_to_timeout',passed=True,cases=8))
    for returncode in (-11,-6,-1,1,42):
        result=dict(status='timeout')
        _external_stop_outcome(result,timed_out=True,returncode=returncode)
        assert result['status']=='infrastructure_error'
    for returncode in (-15,-9):
        result=dict(status='running')
        _external_stop_outcome(result,timed_out=True,returncode=returncode)
        assert result['status']=='timeout'
    checks.append(dict(name='deadline_termination_vs_crash_signal_distinguished',passed=True,cases=7))
    # CPU-only native HPolyhedron rejection, not a fabricated successful planner.
    handoff=base/'handoff';handoff.mkdir()
    result=real_handoff_failure(handoff)
    marker=result['failure_classification'];path=handoff/marker['evidence']
    source={f.HANDOFF_SOURCE:sha(ROOT/f.HANDOFF_SOURCE)}
    receipt=dict(files={marker['evidence']:sha(path)})
    positive=f.handoff_failure_evidence(handoff,result,expected_tolerance=1e-6,source_hashes=source,receipt=receipt)
    assert positive and not f.exception_needs_attention(result,handoff_evidence=positive)
    for fault in ('missing_receipt_binding','wrong_receipt_hash','wrong_source_hash','changed_bound','traversal','error_category'):
        r,s,seal=copy.deepcopy(result),copy.deepcopy(source),copy.deepcopy(receipt)
        bound=1e-6
        if fault=='missing_receipt_binding':seal['files'].clear()
        elif fault=='wrong_receipt_hash':seal['files'][marker['evidence']]='0'*64
        elif fault=='wrong_source_hash':s[f.HANDOFF_SOURCE]='0'*64
        elif fault=='changed_bound':bound=2e-6
        elif fault=='traversal':r['failure_classification']['evidence']='../handoff.json'
        else:r['error_category']='CUDA_error'
        evidence=f.handoff_failure_evidence(handoff,r,expected_tolerance=bound,source_hashes=s,receipt=seal)
        assert f.exception_needs_attention(r,handoff_evidence=evidence)
    checks.append(dict(name='actual_numerical_failure_requires_source_bound_receipt_and_bound',passed=True,negative_cases=6,positive=positive))
    # Historical reader-only exception cannot enter the live worker or current
    # source manifest. Exact reviewed historical evidence remains readable.
    legacy=base/'legacy';legacy.mkdir()
    result,receipt,sources=legacy_drm_fixture(legacy)
    evidence=f.legacy_drm_failure_evidence(legacy,result,source_hashes=sources,receipt=receipt)
    assert evidence and not f.exception_needs_attention(result,legacy_drm_evidence=evidence)
    live_hash=sha(ROOT/f.LEGACY_DRM_SOURCE)
    assert live_hash != f.LEGACY_DRM_SOURCE_SHA256
    assert f.legacy_drm_failure_evidence(legacy,result,source_hashes={f.LEGACY_DRM_SOURCE:live_hash},receipt=receipt) is None
    assert _exception_outcome(RuntimeError(f.LEGACY_DRM_NO_ROUTE),'drm_path')['status']=='infrastructure_error'
    _external_stop_outcome(result,timed_out=True,returncode=-15)
    assert result['status']=='infrastructure_error'
    checks.append(dict(name='legacy_no_route_not_enabled_for_current_runtime',passed=True,current_source_sha256=live_hash,reviewed_source_sha256=f.LEGACY_DRM_SOURCE_SHA256))
    for payload in ("__import__('os').system('false')", "RuntimeError('x','y')", "pkg.RuntimeError('x')", "not Python("):
        result=dict(status='timeout',exception=payload,failed_stage='drm_path')
        assert f.known_exception_category(result) is None and f.exception_needs_attention(result)
    checks.append(dict(name='exception_repr_parser_closed_and_nonexecuting',passed=True,cases=4))

files=['benchmarks/generic_motion_v1/runners/failure_classification.py','benchmarks/generic_motion_v1/runners/csdecomp_baseline.py','benchmarks/generic_motion_v1/batch.py','benchmarks/generic_motion_v1/runners/results.py']
summary=dict(passed=True,blocking_findings=[],checks=checks,source_sha256={x:sha(ROOT/x) for x in files},
    seconds=time.monotonic()-started,gpu_calls=0,planner_calls=0,solver_calls=0,
    review='Worker catch uses exact type/message/phase; final deadline cannot replace infrastructure status. External timeout retains pre-stop evidence and rejects unknown errors/crashes. Sealed reader requires source+receipt+bound for typed handoff failures and exact reviewed old source/build/trace for legacy no-route. Current runtime rejects legacy no-route.')
(OUT/'review.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
