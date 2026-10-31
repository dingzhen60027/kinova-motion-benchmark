"""Public status uses exactly the primary table's saved-output rule."""
import json
import shutil
from pathlib import Path
import pytest

from benchmarks.generic_motion_v1 import cli, outcomes
from benchmarks.generic_motion_v1.runners import results
from benchmarks.generic_motion_v1.common import write, read, sha
from benchmarks.generic_motion_v1.tests.test_valid_output import evidence, _seal, _change


def test_status_uses_valid_output_not_solver_convergence(evidence,monkeypatch,capsys):
    run,manifest,job,attempt=evidence
    monkeypatch.setattr(cli,'require_environment',lambda:None)
    cli.evaluate_main(['status','--run',str(run)])
    status=json.loads(capsys.readouterr().out)
    assert status['statuses']=={'success_numerical':1}
    assert status['raw_statuses']=={'solver_failed':1}
    assert status['completed']==1 and status['pending']==0
    assert results.attempt_row(run,manifest,job,attempt)['status']=='success_numerical'


def test_status_keeps_first_terminal_and_reports_later_attempts(evidence):
    run,manifest,job,attempt=evidence
    later=attempt.with_name('0002');shutil.copytree(attempt,later)
    _change(attempt,'validation.json',passed=False)
    status=results.run_status(run)
    assert status['statuses']=={'solver_failure':1}
    assert status['actual_attempts']==2 and status['ignored_later_terminal_attempts']==1
    chain=status['attempt_relations'][0]
    assert chain['selected_attempt']==1
    assert [a['selected'] for a in chain['attempts']]==[True,False]
    assert chain['attempts'][1]['previous_attempt']==str(attempt)
    assert outcomes.assess_saved_output(later)['accepted']


def test_status_distinguishes_unsealed_retry_and_unstarted(evidence):
    run,manifest,job,attempt=evidence
    retry=attempt.with_name('0002');shutil.copytree(attempt,retry)
    receipt=read(retry/'receipt.json');receipt['retry_pending']=True;write(retry/'receipt.json',receipt)
    # Later interrupted work cannot turn an already completed primary attempt pending.
    fresh=dict(job,job_id='retry');manifest['jobs'].append(fresh)
    fresh_dir=run/'jobs/retry/attempts/0001';shutil.copytree(retry,fresh_dir)
    unsealed=dict(job,job_id='unsealed');manifest['jobs'].append(unsealed)
    (run/'jobs/unsealed/attempts/0001').mkdir(parents=True)
    manifest['jobs'].append(dict(job,job_id='not-started'))
    write(run/'run_manifest.json',manifest)
    status=results.run_status(run)
    assert status['completed']==1 and status['pending']==3
    assert status['not_started_jobs']==status['unsealed_jobs']==status['interrupted_jobs_awaiting_resume']==1
    assert status['attempt_lifecycles']=={'terminal':1,'retry_pending':2,'unsealed':1}
    assert status['attempt_relations'][0]['selected_attempt']==1


def test_status_keeps_full_candidate_receipt_checks(evidence,monkeypatch):
    run,manifest,job,attempt=evidence
    counts=[]
    original=sha
    def counted(path):
        if Path(path).name=='candidate.npz':counts.append(str(path))
        return original(path)
    monkeypatch.setattr(results,'sha',counted)
    monkeypatch.setattr(outcomes,'sha',counted)
    assert results.run_status(run)['statuses']=={'success_numerical':1}
    assert counts==[str(attempt/'candidate.npz')]*2


def test_status_handles_malformed_selected_receipt(evidence):
    run,manifest,job,attempt=evidence
    (attempt/'receipt.json').write_text('[]')
    status=results.run_status(run)
    assert status['statuses']=={'infrastructure_error':1}
    assert status['completed']==1 and status['pending']==0
