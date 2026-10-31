"""Adversarial lifecycle checks; synthetic processes are not planning results."""
import json
import subprocess
import sys
import time
from pathlib import Path
import pytest
from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.common import read, write, sha
from benchmarks.generic_motion_v1.runners.summarize import summarize
from benchmarks.generic_motion_v1.tests.test_batch import prepared

@pytest.mark.parametrize('timeout', [float('nan'), float('inf'), -1., 0., True])
def test_invalid_timeout_rejected_before_process(tmp_path, timeout):
    with pytest.raises(ValueError, match='positive finite'):
        batch.execute({}, tmp_path, timeout)
    assert not list(tmp_path.iterdir())

@pytest.mark.parametrize('field', ['methods', 'objectives', 'roadmaps'])
def test_duplicate_experiment_axes_rejected(tmp_path, monkeypatch, field):
    run, manifest, roadmap = prepared(tmp_path, monkeypatch)
    kwargs=dict(objectives=['minimum_distance'],methods=['EI_ZO'],roadmaps=[roadmap])
    kwargs[field] *= 2
    with pytest.raises(ValueError, match='duplicates'):
        batch.prepare(manifest,output=tmp_path/'bad',**kwargs)
    assert not (tmp_path/'bad').exists()

def test_missing_identity_rejected(tmp_path, monkeypatch):
    run,_,_=prepared(tmp_path,monkeypatch)
    (run/'identity.json').unlink()
    assert not batch.verify(run)['integrity_passed']

def test_empty_receipt_cannot_finish_job(tmp_path, monkeypatch):
    run,_,_=prepared(tmp_path,monkeypatch)
    job=read(run/'run_manifest.json')['jobs'][0]
    write(run/'jobs'/job['job_id']/'attempts/0001/receipt.json',dict(status='success_validated_numerical',files={}))
    check=batch.verify(run)
    assert not check['integrity_passed'] and not check['complete']

def test_corruption_cannot_be_summarized(tmp_path, monkeypatch):
    run,_,_=prepared(tmp_path,monkeypatch)
    job=read(run/'run_manifest.json')['jobs'][0]
    a=run/'jobs'/job['job_id']/'attempts/0001'
    write(a/'result.json',dict(status='success_validated_numerical',success=True))
    write(a/'receipt.json',dict(status='success_validated_numerical',files={'result.json':'not-the-hash'}))
    with pytest.raises(ValueError,match='unverified evidence'):
        summarize(run)
    assert not (run/'summary').exists()

def test_malformed_receipt_fails_closed(tmp_path, monkeypatch):
    run,_,_=prepared(tmp_path,monkeypatch)
    job=read(run/'run_manifest.json')['jobs'][0]
    a=run/'jobs'/job['job_id']/'attempts/0001';a.mkdir(parents=True)
    (a/'receipt.json').write_text('{broken')
    assert not batch.verify(run)['integrity_passed']

def test_deadline_not_blocked_by_gpu_memory_query(tmp_path, monkeypatch):
    run,_,_=prepared(tmp_path,monkeypatch)
    original=subprocess.Popen
    monkeypatch.setattr(batch.subprocess,'Popen',lambda argv,**kwargs: original(
        [sys.executable,'-c','import time;time.sleep(1.0)'],**kwargs))
    def slow_poll(pid):
        time.sleep(1.5)
        return None
    monkeypatch.setattr(batch,'_gpu_memory',slow_poll)
    result=batch.execute(read(run/'run_manifest.json')['jobs'][0],run,.15)
    assert result['status']=='timeout'
    assert result['cold_e2e_seconds'] < .8
    assert batch.verify(run)['integrity_passed']

def test_source_change_during_worker_is_failure(tmp_path, monkeypatch):
    run,_,_=prepared(tmp_path,monkeypatch)
    original=subprocess.Popen
    monkeypatch.setattr(batch.subprocess,'Popen',lambda argv,**kwargs: original(
        [sys.executable,'-c',"import pathlib; pathlib.Path('unused').name"],**kwargs))
    states=iter([{}, {'changed.py':'new-hash'}])
    monkeypatch.setattr(batch,'code_inventory',lambda:next(states))
    monkeypatch.setattr(batch,'_gpu_memory',lambda pid:None)
    result=batch.execute(read(run/'run_manifest.json')['jobs'][0],run,2.)
    assert result['status']=='infrastructure_error'
    assert result['success'] is False
    assert 'source changed' in result['error']

def test_summary_without_input_validity_is_not_legal():
    from benchmarks.generic_motion_v1.runners.summarize import _legal
    assert not _legal({})
    assert not _legal({'validation_status':{}})
    assert not _legal({'validation_status':{'valid':'yes'}})
    assert _legal({'validation_status':'valid_input'})

def test_replay_rejects_nonfinite_time_and_clearance(tmp_path):
    import numpy as np
    from benchmarks.generic_motion_v1.runners.replay import _trajectory
    for time_values, clear in [([0.,float('nan')],[1.,1.]),([0.,1.],[1.,float('inf')])]:
        np.savez(tmp_path/'trajectory_samples.npz',q=np.zeros((2,7)),time=time_values,clearance=clear)
        with pytest.raises(ValueError,match='malformed'):
            _trajectory(tmp_path,{})

def test_manifest_identifiers_cannot_escape_job_directory(tmp_path):
    from benchmarks.generic_motion_v1.common import cases
    p=tmp_path/'cases.jsonl'
    for identifier in ['../escape', 'a/../../escape', '', None]:
        p.write_text(json.dumps({'problem_id':identifier})+'\n')
        with pytest.raises(ValueError,match='identifiers'):
            cases(p)

def test_nonfinite_json_contract_is_rejected(tmp_path):
    p=tmp_path/'bad.json';p.write_text('{"timeout": NaN}')
    with pytest.raises(ValueError,match='Nonfinite JSON'):
        read(p)

def test_worker_identity_conflict_cannot_be_hidden(tmp_path, monkeypatch):
    run,_,_=prepared(tmp_path,monkeypatch)
    job=read(run/'run_manifest.json')['jobs'][0]
    original=subprocess.Popen
    def child(argv, **kwargs):
        attempt=Path(argv[-1])
        code='import json,pathlib; pathlib.Path('+repr(str(attempt/'result.json'))+').write_text(json.dumps({"status":"frontend_failed","method":"WRONG_METHOD"}))'
        return original([sys.executable,'-c',code],**kwargs)
    monkeypatch.setattr(batch.subprocess,'Popen',child)
    monkeypatch.setattr(batch,'_gpu_memory',lambda pid:None)
    result=batch.execute(job,run,2.)
    assert result['status']=='infrastructure_error'
    assert result['identity_conflicts']['method']['actual']=='WRONG_METHOD'
    assert batch.verify(run)['integrity_passed']

def test_cli_audit_cannot_replace_frozen_acceptance_report():
    from benchmarks.generic_motion_v1.__main__ import _guard_audit_output
    from benchmarks.generic_motion_v1.common import BASE
    with pytest.raises(ValueError,match='frozen evidence'):
        _guard_audit_output(BASE/'reports/acceptance.json')
