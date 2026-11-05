"""Per-run summary and public module CLI use the primary acceptance rule."""
import json
from copy import deepcopy
import shutil
import sys

import numpy as np
import pytest

from benchmarks.generic_motion_v1.common import read,write
from benchmarks.generic_motion_v1.runners import summarize as reporting
from benchmarks.generic_motion_v1.tests.test_valid_output import evidence, _change,_seal


def allow_fixture_manifest(monkeypatch):
    # The sealed candidate fixtures test actual receipts/arrays; only the run
    # source/roadmap provenance is synthetic in this isolated accounting test.
    monkeypatch.setattr(reporting, '_verify_run', lambda *a, **k: dict(integrity_passed=True))


def test_per_run_accepts_independently_validated_nonconvergence(evidence, monkeypatch, tmp_path):
    run, manifest, job, attempt = evidence
    allow_fixture_manifest(monkeypatch)
    report = reporting.summarize(run, output=tmp_path/'summary')
    assert report['status_counts'] == {'success_numerical': 1}
    group = next(g for g in report['groups'] if g['family'] == 'all')
    assert group['success_cost']['mean'] == read(attempt/'validation.json')['cost']
    assert report['finished_jobs'] == 1 and report['complete']


def test_per_run_cannot_select_later_better_terminal(evidence, monkeypatch, tmp_path):
    run, manifest, job, attempt = evidence
    allow_fixture_manifest(monkeypatch)
    later=attempt.with_name('0002');shutil.copytree(attempt,later)
    _change(attempt, 'validation.json', passed=False)
    report=reporting.summarize(run, output=tmp_path/'summary')
    assert report['status_counts'] == {'solver_failure': 1}
    assert report['groups'][0]['success_cost']['count'] == 0
    assert reporting._selected_attempt(run,job['job_id']) == attempt


def test_per_run_rechecks_corrupted_saved_candidate(evidence, monkeypatch, tmp_path):
    run, manifest, job, attempt = evidence
    allow_fixture_manifest(monkeypatch)
    with (attempt/'candidate.npz').open('ab') as stream: stream.write(b'corruption')
    report=reporting.summarize(run, output=tmp_path/'summary')
    assert report['status_counts'] == {'infrastructure_error': 1}
    assert report['groups'][0]['success_cost']['count'] == 0


def test_module_summary_output_argument_uses_shared_acceptance(evidence, monkeypatch, tmp_path, capsys):
    run, manifest, job, attempt = evidence
    allow_fixture_manifest(monkeypatch)
    from benchmarks.generic_motion_v1 import __main__ as module_cli
    out=tmp_path/'fresh_cli_output'
    monkeypatch.setattr(sys,'argv',['benchmark','summarize','--run',str(run),'--output',str(out)])
    module_cli.main()
    printed=json.loads(capsys.readouterr().out)
    assert printed['status_counts']=={'success_numerical':1}
    assert read(out/'summary.json')['status_counts']==printed['status_counts']
    assert not (run/'summary').exists()


@pytest.fixture
def sealed_study_evidence(evidence,monkeypatch,tmp_path):
    """Full numeric receipt fixtures; collision masks remain explicit test doubles.

    This substitutes only source/roadmap run verification, never acceptance,
    trajectory reconstruction, endpoint checks, budget checks or receipts.
    """
    from benchmarks.generic_motion_v1.runners import study_metrics as study
    run,manifest,job,attempt=evidence
    case=job['case'];case['problem_sha256']='numeric-accounting-fixture'
    case.update(straight_line_collision=True,endpoint_sampling=dict(distance_bin='short'),
                start_goal_joint_distance=1.,normalized_start_goal_joint_distance=.1)
    roadmap=tmp_path/'fixture-roadmap'
    write(roadmap/'metadata.json',dict(roadmap_id='fixture-map',requested_nodes=6000,actual_nodes=6000,seed=0))
    job.update(roadmap_id='fixture-map',roadmap_dir=str(roadmap),scene_id=case['scene_id'],family=case['family'])
    manifest.update(source_code_commit='fixture-source',source_hashes={'fixture-accounting':'same'})
    write(run/'run_manifest.json',manifest);write(attempt/'request.json',job)
    write(attempt/'input.json',dict(case,_objective='minimum_distance'))
    candidate=read(attempt/'candidate.json');candidate['solver_converged']=False
    write(attempt/'candidate.json',candidate);_seal(attempt)
    monkeypatch.setattr(study,'verify',lambda *a,**k:dict(integrity_passed=True,complete=True,fixture_only=True))
    return run,manifest,job,attempt


@pytest.mark.parametrize('first_valid',[True,False])
def test_study_analyzer_uses_complete_acceptance_and_first_terminal(sealed_study_evidence,tmp_path,first_valid):
    from benchmarks.generic_motion_v1.runners import study_metrics as study
    run,manifest,job,attempt=sealed_study_evidence
    if not first_valid:
        later=attempt.with_name('0002');shutil.copytree(attempt,later)
        _change(attempt,'validation.json',passed=False)
    report=study.analyze([run],tmp_path/'study',expected_labels=['NTO-distance'])
    row=report['rows'][0]
    assert row['attempt']==str(attempt)
    assert row['success'] is first_valid
    assert row['cost']==(1. if first_valid else None)
    group=next(g for g in report['groups'] if g['scope']=='obstacle_main')
    assert group['valid_success']['estimate']==int(first_valid)


@pytest.mark.parametrize('first_valid',[True,False])
def test_extended_analyzer_keeps_valid_nonconvergence_and_first_terminal(
        sealed_study_evidence,tmp_path,first_valid):
    from benchmarks.generic_motion_v1.runners import extended_metrics as extended
    run,manifest,source_job,source=sealed_study_evidence
    jobs=[];methods=('EI_ZO','RRTCONNECT','RRTSTAR','CHOMP','STOMP','TRAJOPT')
    for method in methods:
        job=deepcopy(source_job);job.update(job_id=method,method=method)
        attempt=run/'jobs'/method/'attempts/0001';shutil.copytree(source,attempt)
        result=read(attempt/'result.json')
        result.update(method=method,status='solver_failed' if method=='TRAJOPT' else 'success_validated_numerical',
                      roadmap_used=method not in ('RRTCONNECT','RRTSTAR'))
        for key in ('frontend_native_success','frontend_success','drm_success'):
            result[key]=True if result['roadmap_used'] else None
        if result['roadmap_used']:
            np.save(attempt/'path.npy',np.asarray([job['case']['q_start'],job['case']['q_goal']]))
            write(attempt/'frontend_validation/validation.json',dict(passed=True,cost=1.,minimum_sampled_clearance_m=.01))
        candidate=read(attempt/'candidate.json')
        candidate.update(solver_success=method!='TRAJOPT',solver_converged=method!='TRAJOPT')
        write(attempt/'candidate.json',candidate);write(attempt/'result.json',result)
        write(attempt/'request.json',job);_seal(attempt)
        jobs.append(job)
        if method=='TRAJOPT' and not first_valid:
            later=attempt.with_name('0002');shutil.copytree(attempt,later)
            _change(attempt,'validation.json',passed=False)
    manifest['jobs']=jobs;write(run/'run_manifest.json',manifest)
    report=extended.analyze([run],tmp_path/'extended')
    row=next(r for r in report['rows'] if r['label']=='TrajOpt')
    assert row['attempt'].endswith('/TRAJOPT/attempts/0001')
    assert row['success'] is first_valid and row['solver_converged'] is False
    assert row['cost']==(1. if first_valid else None)
    group=next(g for g in report['groups'] if g['scope']=='all' and g['label']=='TrajOpt')
    assert group['validated_incumbent_nonconverged']==int(first_valid)
    assert group['valid_success']['estimate']==int(first_valid)


def test_study_preserves_unsupported_raw_reason_after_status_normalization(sealed_study_evidence):
    from benchmarks.generic_motion_v1.runners import study_metrics as study
    run,manifest,job,attempt=sealed_study_evidence
    _change(attempt,'result.json',status='unsupported')
    rows,_=study.load_rows([run]);row=rows[0]
    assert row['status']=='infrastructure_error'
    assert row['last_attempt_status']=='unsupported' and row['dependency_unsupported'] is True
    assert row['success'] is False and row['cost'] is None
