"""Fault boundaries in the real worker adapter, with a CPU-only fake backend."""
import sys
import types

import numpy as np
import pytest

from research_platform import runner,storage


@pytest.fixture
def worker_attempt(tmp_path,monkeypatch):
    run=tmp_path/'run'
    job=dict(id='job',kind='method',case_id='case',method='LSCS',dependencies={})
    storage.atomic(run/'manifest.json',dict(jobs=[job],source_hashes={}))
    storage.atomic(run/'resolved.json',{})
    storage.atomic(run/'environment.json',{})
    (run/'source_snapshot').mkdir();storage.freeze_inputs(run)
    out=storage.new_attempt(run,'job')
    storage.atomic(out/'attempt.json',dict(dependency_receipts={}))
    monkeypatch.setattr(runner,'parent_death_signal',lambda:None)
    backend=types.ModuleType('research_platform.backend')
    monkeypatch.setitem(sys.modules,'research_platform.backend',backend)
    return run,out,backend


def test_worker_serializes_actual_numpy_values(worker_attempt):
    run,out,backend=worker_attempt
    backend.execute_job=lambda *args:dict(status='success',solver_success=np.bool_(True),
        validation_passed=np.bool_(True),metrics=dict(cost=np.float64(1.2)),
        diagnostic=np.array([1.,2.]))
    runner.worker(run,'job',out.name)
    result=storage.read(out/'result.json')
    assert result['status']=='success' and result['solver_success'] is True
    assert result['diagnostic']==[1.,2.] and result['metrics']['cost']==1.2


@pytest.mark.parametrize('bad_value',[float('nan'),float('inf'),object()])
def test_invalid_worker_result_becomes_explicit_error_not_partial_success(worker_attempt,bad_value):
    run,out,backend=worker_attempt
    backend.execute_job=lambda *args:dict(status='success',solver_success=True,
        validation_passed=True,diagnostic=bad_value)
    runner.worker(run,'job',out.name)
    result=storage.read(out/'result.json')
    assert result['status']=='error'
    assert result['solver_success'] is False and result['validation_passed'] is False
    assert result['error'] and result['traceback']
    assert not list(out.glob('.write-*'))


def test_worker_exception_preserves_debug_artifact_and_records_failure(worker_attempt):
    run,out,backend=worker_attempt
    def fail(job,resolved,deps,dest):
        (dest/'debug.txt').write_text('evidence before backend failure\n')
        raise RuntimeError('injected numerical failure')
    backend.execute_job=fail
    runner.worker(run,'job',out.name)
    result=storage.read(out/'result.json')
    assert result['status']=='error' and 'injected numerical failure' in result['error']
    assert (out/'debug.txt').read_text()=='evidence before backend failure\n'
