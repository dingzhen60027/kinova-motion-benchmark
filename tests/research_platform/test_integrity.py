"""CPU-only fault injection against immutable inputs and recorded attempts.

All evidence is synthetic and isolated in pytest temporary directories. These
checks never create a robot model or invoke numerical planning/optimization.
"""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys

import pytest

from research_platform import storage


@pytest.fixture
def frozen_run(tmp_path):
    run=tmp_path/'experiment'
    job=dict(id='e000_r000_n00200_LSCS',kind='method',case_id='e000_r000_n00200',
             method='LSCS',environment='forest',dependencies={})
    storage.atomic(run/'manifest.json',dict(name='fault_injection',jobs=[job],source_hashes={}))
    storage.atomic(run/'resolved.json',dict(config={'name':'fault_injection'}))
    storage.atomic(run/'environment.json',dict(numerical_environment='fake-no-GPU'))
    path=run/'source_snapshot/runner.py';path.parent.mkdir();path.write_text('# frozen source\n')
    storage.freeze_inputs(run)
    return run,job


def completed(run,job,status='success',**metadata):
    attempt=storage.new_attempt(run,job['id'])
    storage.atomic(attempt/'result.json',dict(status='success',solver_success=True,
        validation_passed=True,input_hash='fixed-input',metrics={'cost':1.0}))
    (attempt/'stderr.log').write_text('retain diagnostic\n')
    storage.seal_attempt(attempt,status,**metadata)
    return attempt


def test_missing_planned_job_never_verifies_as_complete(frozen_run):
    run,job=frozen_run
    answer=storage.verify_run(run)
    assert answer['complete'] is False and answer['verified'] is False
    assert answer['all_valid'] is False
    assert answer['missing_jobs']==[job['id']]
    assert storage.verify_run(run,require_complete=False)['complete'] is False


def test_unsealed_attempt_is_incomplete_even_if_worker_wrote_success(frozen_run):
    run,job=frozen_run
    attempt=storage.new_attempt(run,job['id'])
    storage.atomic(attempt/'result.json',dict(status='success',solver_success=True,validation_passed=True))
    result=storage.read_result(run,job['id'])
    assert result['status']=='interrupted' and not result['validation_passed']
    verification=storage.verify_run(run)
    assert not verification['complete'] and not verification['all_valid']
    assert verification['partial_attempts']


@pytest.mark.parametrize('name', ['manifest.json','resolved.json','environment.json','source_snapshot/runner.py'])
def test_frozen_input_content_changes_are_rejected(frozen_run,name):
    run,_=frozen_run
    path=run/name
    path.write_bytes(path.read_bytes()+b'\n ')
    with pytest.raises(ValueError,match='Frozen experiment inputs changed'):
        storage.verify_inputs(run)


@pytest.mark.parametrize('mutation', ['add','delete'])
def test_snapshot_membership_changes_are_rejected(frozen_run,mutation):
    run,_=frozen_run
    if mutation=='add':(run/'source_snapshot/unrecorded.py').write_text('# new code')
    else:(run/'source_snapshot/runner.py').unlink()
    with pytest.raises(ValueError,match='Frozen experiment inputs changed'):
        storage.verify_inputs(run)


@pytest.mark.parametrize('mutation', ['edit_result','add_file','delete_log'])
def test_case_evidence_tampering_is_rejected(frozen_run,mutation):
    run,job=frozen_run;attempt=completed(run,job)
    if mutation=='edit_result':
        record=storage.read(attempt/'result.json');record['metrics']['cost']=0
        storage.atomic(attempt/'result.json',record)
    elif mutation=='add_file':(attempt/'unrecorded.csv').write_text('new data\n')
    else:(attempt/'stderr.log').unlink()
    with pytest.raises(ValueError,match='Attempt evidence changed'):
        storage.verify_run(run)


def test_timeout_receipt_overrides_optimistic_worker_result(frozen_run):
    run,job=frozen_run;completed(run,job,status='timeout',wall_seconds=120)
    result=storage.read_result(run,job['id'])
    assert result['status']=='timeout'
    assert result['validation_passed'] is False
    assert storage.verify_run(run)['all_valid'] is False


def test_receipt_status_tamper_cannot_turn_timeout_into_success(frozen_run):
    run,job=frozen_run;attempt=completed(run,job,status='timeout')
    receipt=storage.read(attempt/'receipt.json');receipt['status']='success'
    storage.atomic(attempt/'receipt.json',receipt)
    with pytest.raises(ValueError):storage.read_result(run,job['id'])


def test_sealed_attempt_cannot_be_reclassified_in_place(frozen_run):
    run,job=frozen_run;attempt=completed(run,job,status='timeout')
    before=storage.inventory(attempt)
    with pytest.raises(FileExistsError):storage.seal_attempt(attempt,'success')
    assert storage.inventory(attempt)==before


def test_failed_nonmethod_job_cannot_be_all_valid(tmp_path):
    run=tmp_path/'run';job=dict(id='plan',kind='plan',dependencies={})
    storage.atomic(run/'manifest.json',dict(jobs=[job],source_hashes={}))
    storage.atomic(run/'resolved.json',{});storage.atomic(run/'environment.json',{})
    (run/'source_snapshot').mkdir();storage.freeze_inputs(run)
    completed(run,job,status='planning_failed')
    answer=storage.verify_run(run)
    assert answer['complete'] is True
    assert answer['all_valid'] is False


def test_new_attempt_preserves_all_failed_attempt_bytes(frozen_run):
    run,job=frozen_run;first=completed(run,job,status='optimization_failed')
    before=storage.inventory(first)
    second=completed(run,job,status='success')
    assert first!=second and storage.inventory(first)==before
    assert storage.selected_attempt(run,job['id'])==second
    assert storage.read_result(run,job['id'])['status']=='success'
    assert len(storage.attempts(run,job['id']))==2


def test_flock_excludes_other_process_and_releases_after_exit(tmp_path):
    path=tmp_path/'experiment.lock'
    code='''import sys
from research_platform.storage import lock
try:
    with lock(sys.argv[1]): pass
except RuntimeError:
    raise SystemExit(73)
'''
    with storage.lock(path):
        blocked=subprocess.run([sys.executable,'-c',code,str(path)],cwd=storage.ROOT,
                               capture_output=True,text=True,timeout=5)
        assert blocked.returncode==73,blocked.stderr
    available=subprocess.run([sys.executable,'-c',code,str(path)],cwd=storage.ROOT,
                             capture_output=True,text=True,timeout=5)
    assert available.returncode==0,available.stderr


def test_symlink_cannot_be_substituted_for_saved_evidence(frozen_run,tmp_path):
    run,job=frozen_run;attempt=completed(run,job)
    external=tmp_path/'external';external.write_text('retain diagnostic\n')
    (attempt/'stderr.log').unlink();(attempt/'stderr.log').symlink_to(external)
    with pytest.raises(ValueError,match='Symlink'):storage.verify_attempt(attempt)


def dependency_run(tmp_path):
    run=tmp_path/'dependency_run'
    scene=dict(id='scene',kind='scene',dependencies={})
    plan=dict(id='plan',kind='plan',dependencies={'scene':'scene'})
    method=dict(id='method',kind='method',case_id='case',method='LSCS',environment='forest',
                dependencies={'plan':'plan'})
    storage.atomic(run/'manifest.json',dict(name='dependency_fault',jobs=[scene,plan,method],source_hashes={}))
    storage.atomic(run/'resolved.json',{});storage.atomic(run/'environment.json',{})
    (run/'source_snapshot').mkdir();storage.freeze_inputs(run)
    a=completed(run,scene)
    b=completed(run,plan,dependency_receipts={'scene':storage.sha(a/'receipt.json')})
    c=completed(run,method,dependency_receipts={'plan':storage.sha(b/'receipt.json')})
    return run,scene,plan,method,a,b,c


def test_dependency_retry_removes_old_method_from_strict_success_summary(tmp_path):
    from research_platform.reporting import summarize_run
    run,scene,plan,method,a,b,c=dependency_run(tmp_path)
    old=storage.inventory(c)
    assert storage.read_result(run,method['id'])['status']=='success'
    completed(run,plan,dependency_receipts={'scene':storage.sha(a/'receipt.json')})
    stale=storage.read_result(run,method['id'])
    assert stale['status']=='stale_dependency' and not stale['validation_passed']
    assert storage.inventory(c)==old
    summary=summarize_run(run)
    assert summary['methods'][0]['requested']==1
    assert summary['methods'][0]['strict_successes']==0
    assert summary['status_counts']=={'stale_dependency':1}


def test_transitive_dependency_retry_cannot_leave_method_successful(tmp_path):
    run,scene,plan,method,a,b,c=dependency_run(tmp_path)
    completed(run,scene)
    assert storage.read_result(run,plan['id'])['status']=='stale_dependency'
    result=storage.read_result(run,method['id'])
    assert result['status']=='stale_dependency'
    assert not result['validation_passed']


def test_sealed_latest_interruption_keeps_fully_recorded_plan_incomplete(tmp_path):
    run,scene,plan,method,a,b,c=dependency_run(tmp_path)
    interrupted=completed(run,method,status='interrupted',
        dependency_receipts={'plan':storage.sha(b/'receipt.json')})
    assert storage.verify_attempt(interrupted)['status']=='interrupted'
    answer=storage.verify_run(run)
    assert answer['missing_jobs']==[] and answer['partial_attempts']==[]
    assert answer['complete'] is False and answer['verified'] is False
    assert answer['all_valid'] is False
    assert answer['interrupted_jobs']==[method['id']]


def test_successful_retry_supersedes_interruption_without_discarding_history(tmp_path):
    run,scene,plan,method,a,b,c=dependency_run(tmp_path)
    interrupted=completed(run,method,status='interrupted',
        dependency_receipts={'plan':storage.sha(b/'receipt.json')})
    before=storage.inventory(interrupted)
    successful=completed(run,method,
        dependency_receipts={'plan':storage.sha(b/'receipt.json')})
    answer=storage.verify_run(run)
    assert storage.selected_attempt(run,method['id'])==successful
    assert answer['complete'] is True and answer['verified'] is True
    assert answer['all_valid'] is True and answer['interrupted_jobs']==[]
    assert storage.inventory(interrupted)==before
    assert storage.verify_attempt(interrupted)['status']=='interrupted'


def test_dependent_result_does_not_hide_corrupted_dependency_files(tmp_path):
    run,scene,plan,method,a,b,c=dependency_run(tmp_path)
    (a/'stderr.log').write_text('changed dependency evidence')
    with pytest.raises(ValueError,match='Attempt evidence changed'):
        storage.read_result(run,method['id'])


@pytest.fixture
def scheduled_run(tmp_path,monkeypatch):
    from research_platform import runner
    run=tmp_path/'scheduled'
    job=dict(id='method-case-LSCS',kind='method',case_id='case',method='LSCS',environment='forest',dependencies={})
    environment={'numerical_environment':'mock-runtime'}
    storage.atomic(run/'manifest.json',dict(name='scheduler_fault',jobs=[job],cases=[{'id':'case'}],
        source_hashes={},config={'limits':{'job_timeout_seconds':1}},resolved_sha256='mock-resolved'))
    storage.atomic(run/'resolved.json',{});storage.atomic(run/'environment.json',environment)
    (run/'source_snapshot').mkdir();storage.freeze_inputs(run)
    monkeypatch.setattr(runner,'ROOT',tmp_path/'mock-repo')
    monkeypatch.setattr(runner,'sources',lambda resolved:{})
    monkeypatch.setattr(runner,'fingerprint',lambda:environment)
    calls=[]
    def execute(command,out,timeout):
        calls.append((command,out,timeout))
        storage.atomic(out/'result.json',dict(status='success',solver_success=True,
            validation_passed=True,input_hash='fixed-input',metrics={'cost':1.0}))
        (out/'stdout.log').write_text('mock worker; no numerical execution\n')
        return dict(outcome='finished',exit_code=0,wall_seconds=.001)
    monkeypatch.setattr(runner,'execute_process',execute)
    return run,job,runner,calls


def test_default_resume_does_not_retry_completed_numerical_failure(scheduled_run):
    run,job,runner,calls=scheduled_run
    original=completed(run,job,status='optimization_failed')
    before=storage.inventory(original)
    answer=runner.schedule(run)
    assert calls==[]
    assert len(storage.attempts(run,job['id']))==1
    assert storage.inventory(original)==before
    assert answer['statuses'][job['id']]=='optimization_failed'


def test_explicit_retry_creates_new_attempt_and_preserves_old_failure(scheduled_run):
    run,job,runner,calls=scheduled_run
    original=completed(run,job,status='optimization_failed')
    before=storage.inventory(original)
    result=runner.schedule(run,retry_jobs=[job['id']])
    assert len(calls)==1 and len(storage.attempts(run,job['id']))==2
    assert storage.inventory(original)==before
    assert storage.selected_attempt(run,job['id'])!=original
    assert storage.read_result(run,job['id'])['status']=='success'
    assert storage.read(storage.selected_attempt(run,job['id'])/'attempt.json')['retry_of']=='0001'
    assert result['all_valid'] is True


def test_interrupted_resume_seals_old_attempt_and_runs_new_one(scheduled_run):
    run,job,runner,calls=scheduled_run
    original=storage.new_attempt(run,job['id'])
    storage.atomic(original/'result.json',dict(status='success',solver_success=True,validation_passed=True))
    (original/'stderr.log').write_text('last message before external interruption\n')
    before={name:storage.sha(original/name) for name in ['result.json','stderr.log']}
    runner.schedule(run)
    assert len(calls)==1 and len(storage.attempts(run,job['id']))==2
    for name,expected in before.items():assert storage.sha(original/name)==expected
    assert storage.verify_attempt(original)['status']=='interrupted'
    assert storage.read_result(run,job['id'])['status']=='success'


def test_unknown_retry_job_is_rejected_before_execution(scheduled_run):
    run,job,runner,calls=scheduled_run
    with pytest.raises(ValueError,match='Unknown retry jobs'):
        runner.schedule(run,retry_jobs=['unplanned-job'])
    assert not calls and not storage.attempts(run,job['id'])


def test_signal_interruption_receipt_preserves_reason_and_rejects_partial_success(scheduled_run,monkeypatch):
    run,job,runner,calls=scheduled_run
    def interrupted(command,out,timeout):
        storage.atomic(out/'result.json',dict(status='success',solver_success=True,validation_passed=True))
        (out/'stdout.log').write_text('worker output before SIGTERM\n')
        raise KeyboardInterrupt('Received signal SIGTERM')
    monkeypatch.setattr(runner,'execute_process',interrupted)
    with pytest.raises(KeyboardInterrupt,match='SIGTERM'):
        runner.schedule(run)
    attempt=storage.selected_attempt(run,job['id'])
    receipt=storage.verify_attempt(attempt)
    assert receipt['status']=='interrupted' and receipt['reason']=='Received signal SIGTERM'
    assert storage.read(attempt/'result.json')['status']=='success'
    assert storage.read_result(run,job['id'])['validation_passed'] is False
    assert (attempt/'stdout.log').read_text()=='worker output before SIGTERM\n'


def test_single_case_does_not_claim_batch_complete_and_resume_reuses_shared_inputs(tmp_path,monkeypatch):
    from research_platform import runner
    from research_platform.config import validate_config
    config=validate_config(dict(version=1,name='single_then_batch',environment='forest',
        environment_seeds=[0,1],roadmap_seeds=[0],roadmap_sizes=[200],
        methods=['DRM','LSCS'],scene_profile='random'))
    cases,jobs=runner.job_graph(config)
    run=tmp_path/'single_then_batch';environment={'fake-runtime':True}
    storage.atomic(run/'manifest.json',dict(name='single_then_batch',jobs=jobs,cases=cases,
        source_hashes={},config=config,resolved_sha256='fake-resolved'))
    storage.atomic(run/'resolved.json',{});storage.atomic(run/'environment.json',environment)
    (run/'source_snapshot').mkdir();storage.freeze_inputs(run)
    monkeypatch.setattr(runner,'ROOT',tmp_path/'fake-root')
    monkeypatch.setattr(runner,'sources',lambda resolved:{})
    monkeypatch.setattr(runner,'fingerprint',lambda:environment)
    calls=[]
    def execute(command,out,timeout):
        calls.append(command[command.index('--job')+1])
        storage.atomic(out/'result.json',dict(status='success',solver_success=True,validation_passed=True))
        return dict(outcome='finished',exit_code=0,wall_seconds=.001)
    monkeypatch.setattr(runner,'execute_process',execute)
    first=runner.schedule(run,case_id=cases[0]['id'])
    assert first['complete'] is False and first['all_valid'] is False
    assert set(first['missing_jobs'])=={'scene-e001','plan-e001_r000_n00200',
        'method-e001_r000_n00200-DRM','method-e001_r000_n00200-LSCS'}
    assert len(calls)==5 and 'map-r000-n00200' in calls
    completed_ids=set(calls)
    original={job_id:storage.inventory(storage.selected_attempt(run,job_id)) for job_id in completed_ids}
    second=runner.schedule(run)
    assert second['complete'] is True and second['all_valid'] is True
    assert len(calls)==9 and calls.count('map-r000-n00200')==1
    for job_id in completed_ids:
        assert len(storage.attempts(run,job_id))==1
        assert storage.inventory(storage.selected_attempt(run,job_id))==original[job_id]
