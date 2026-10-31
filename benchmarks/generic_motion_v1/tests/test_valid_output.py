"""Output validity is independent of solver convergence, but requires evidence."""
import json
from pathlib import Path

import pytest
from benchmarks.generic_motion_v1.tests.evidence_fixtures import save_numeric_output
from benchmarks.generic_motion_v1.common import BASE, read

from benchmarks.generic_motion_v1.common import write, sha
from benchmarks.generic_motion_v1 import outcomes, dataset
from benchmarks.generic_motion_v1.runners import results


def _seal(attempt):
    result = json.loads((attempt / "result.json").read_text())
    write(attempt / "receipt.json", dict(status=result["status"],
        files={str(p.relative_to(attempt)): sha(p) for p in attempt.rglob("*")
               if p.is_file() and p.name != "receipt.json"}))


def _change(attempt, name, **changes):
    path = attempt / name
    data = json.loads(path.read_text())
    data.update(changes)
    write(path, data)
    _seal(attempt)


@pytest.fixture
def evidence(tmp_path):
    run = tmp_path / "results/run"
    attempt = run / "jobs/task/attempts/0001"
    attempt.mkdir(parents=True)
    case = dict(problem_id="dev-case",scene_id="dev-scene",family="bottleneck",
        split="dev",validation_status="valid_input",q_start=[0.]*7,q_goal=[1.,0.,0.,0.,0.,0.,0.])
    robot=read(BASE/"configs/robot.json")
    case.update({k:robot[k] for k in ("robot_sha256","collision_geometry_sha256")})
    case.update(velocity_limits_rad_s=[1.]*7,acceleration_limits_rad_s2=[1.]*7,endpoint_velocity_rad_s=[0.]*7)
    job = dict(job_id="task",case_id="dev-case",case=case,method="NTO_PAPER",
        objective="minimum_distance",method_seed=1,replica_seed=0,
        evaluation_config=dict(endpoint_tolerance_rad=1e-5,validation_step_rad=.005))
    manifest = dict(jobs=[job],timeout_seconds=60,split="dev")
    write(run / "run_manifest.json",manifest)
    write(attempt / "request.json",job)
    write(attempt / "input.json",dict(case,_objective="minimum_distance"))
    write(attempt / "result.json",dict(status="solver_failed",problem_id="dev-case",scene_id="dev-scene",
        family="bottleneck",method="NTO_PAPER",objective="minimum_distance",method_seed=1,budget_seconds=60))
    write(attempt / "process.json",dict(exit_code=0,timed_out=False,timeout_seconds=60,cold_e2e_seconds=59.9))
    save_numeric_output(attempt,case,job['evaluation_config'],kind='bspline',solver_success=False)
    _seal(attempt)
    return run,manifest,job,attempt


def test_valid_output_not_solver_convergence_and_witness_share_rule(evidence,tmp_path):
    run,manifest,job,attempt=evidence
    row=results.attempt_row(run,manifest,job,attempt)
    assert row["status"]=="success_numerical"
    assert row["raw_status"]=="solver_failed"
    assert row["solver_reported_success"] is False
    assert row["solver_converged"] is None
    assert row["objective_value"]==1.
    assert row["first_valid_seconds"] is None
    assert row["status_basis"]==outcomes.NONCONVERGED_STATUS_BASIS
    witness=dataset.numerical_witness(attempt/"result.json",job["case"],tmp_path)
    assert witness["status_basis"]==row["status_basis"]
    assert witness["raw_status"]=="solver_failed"
    witnesses,_=dataset.collect_witnesses([job["case"]],tmp_path)
    assert list(witnesses)==["dev-case"]


@pytest.mark.parametrize("file,changes",[
    ("process.json",dict(exit_code=1)),
    ("process.json",dict(timed_out=True)),
    ("process.json",dict(cold_e2e_seconds=60.001)),
    ("process.json",dict(timeout_seconds=120)),
    ("process.json",dict(cold_e2e_seconds=None)),
    ("result.json",dict(status="timeout")),
    ("result.json",dict(status="infrastructure_error")),
    ("result.json",dict(exception="CUDA failed")),
    ("result.json",dict(failed_stage="validation")),
    ("result.json",dict(method="different")),
    ("result.json",dict(budget_seconds=None)),
    ("input.json",dict(q_goal=[2.]*7)),
    ("request.json",dict(method_seed=2)),
    ("candidate.json",dict(objective="minimum_time")),
    ("candidate.json",dict(solver_success=True)),
    ("validation.json",dict(passed=False)),
    ("validation.json",dict(collision_free=False)),
    ("validation.json",dict(joint_limits_passed=False)),
    ("validation.json",dict(derivative_limits_passed=False)),
    ("validation.json",dict(independent_collisions=1)),
    ("validation.json",dict(model_disagreements=1)),
    ("validation.json",dict(contact_band_samples=1)),
    ("validation.json",dict(endpoint_error_rad=1e-3)),
    ("validation.json",dict(sample_step_bound_rad=.1)),
    ("validation.json",dict(minimum_sampled_clearance_m=0.)),
    ("validation.json",dict(cost=None)),
    ("validation.json",dict(cost_unit="s")),
])
def test_incomplete_or_contradictory_evidence_never_promoted(evidence,tmp_path,file,changes):
    run,manifest,job,attempt=evidence
    _change(attempt,file,**changes)
    assert not outcomes.assess_saved_output(attempt)["accepted"]
    assert results.attempt_row(run,manifest,job,attempt)["status"]!="success_numerical"
    assert dataset.numerical_witness(attempt/"result.json",job["case"],tmp_path) is None


@pytest.mark.parametrize("artifact",["candidate.json","candidate.npz","validation.json",
    "trajectory_samples.npz","process.json","input.json","request.json"])
def test_missing_candidate_or_bound_evidence_never_promoted(evidence,artifact):
    run,manifest,job,attempt=evidence
    (attempt/artifact).unlink()
    _seal(attempt)
    assert not outcomes.assess_saved_output(attempt)["accepted"]
    assert results.attempt_row(run,manifest,job,attempt)["status"]!="success_numerical"


def test_receipt_corruption_and_no_verify_never_promote(evidence):
    run,manifest,job,attempt=evidence
    assert results.attempt_row(run,manifest,job,attempt,verify_files=False)["status"]=="solver_failure"
    (attempt/"candidate.npz").write_bytes(b"modified after validation")
    assert not outcomes.assess_saved_output(attempt)["accepted"]
    assert results.attempt_row(run,manifest,job,attempt)["status"]=="infrastructure_error"


def test_legal_original_input_is_required(evidence):
    run,manifest,job,attempt=evidence
    job["case"]["validation_status"]="generation_failed"
    write(run/"run_manifest.json",manifest)
    write(attempt/"request.json",job)
    write(attempt/"input.json",job["case"])
    _seal(attempt)
    assert not outcomes.assess_saved_output(attempt)["accepted"]
    assert results.attempt_row(run,manifest,job,attempt)["status"]=="invalid_input"


def test_later_valid_attempt_never_replaces_first_terminal_failure(evidence,tmp_path,monkeypatch):
    run,manifest,job,attempt=evidence
    later=attempt.with_name("0002")
    attempt.rename(later)
    attempt.mkdir()
    write(attempt/"result.json",dict(status="solver_failed"))
    _seal(attempt)
    assert outcomes.assess_saved_output(later)["accepted"]
    assert results.attempt_row(run,manifest,job,later)["status"]=="success_numerical"
    assert dataset.numerical_witness(later/"result.json",job["case"],tmp_path) is not None
    monkeypatch.setattr("benchmarks.generic_motion_v1.batch.verify",lambda *a,**kw:{"integrity_passed":True})
    rows,_=results.collect([run])
    chosen,_=results._selected_rows(rows,{"id":"fixed","runs":[str(run)]})
    assert len(chosen)==1 and chosen[0]["attempt_number"]==1
    assert chosen[0]["status"]!="success_numerical"


@pytest.mark.parametrize("raw",["success_validated_numerical","solver_failed"])
@pytest.mark.parametrize("bad",["no_candidate","late","receipt_mismatch"])
def test_reported_success_and_nonconvergence_have_same_minimum_evidence(evidence,raw,bad):
    run,manifest,job,attempt=evidence
    _change(attempt,"result.json",status=raw)
    _change(attempt,"candidate.json",solver_success=(raw!="solver_failed"))
    if bad=="no_candidate":
        (attempt/"candidate.npz").unlink(); _seal(attempt)
    elif bad=="late":
        _change(attempt,"process.json",cold_e2e_seconds=61.)
    else:
        (attempt/"candidate.npz").write_bytes(b"tampered")
    assert not outcomes.assess_saved_output(attempt)["accepted"]
    row=results.attempt_row(run,manifest,job,attempt)
    assert row["status"]!="success_numerical"
    assert row["failure_reason"]
    assert row["raw_status"]==raw


def test_same_rule_for_time_track_and_all_methods(evidence):
    run,manifest,job,attempt=evidence
    job["method"]="ANY_CURRENT_OR_FUTURE_METHOD"
    job["objective"]="minimum_time"
    write(run/"run_manifest.json",manifest)
    write(attempt/"request.json",job)
    _change(attempt,"input.json",_objective="minimum_time")
    save_numeric_output(attempt,job['case'],job['evaluation_config'],kind='bezier',objective='minimum_time',solver_success=False)
    _change(attempt,"result.json",method=job["method"],objective="minimum_time")
    _seal(attempt)
    row=results.attempt_row(run,manifest,job,attempt)
    assert row["status"]=="success_numerical" and row["trajectory_duration_s"]==4.


import numpy as np

def _replace_archive(attempt, filename, update):
    with np.load(attempt / filename, allow_pickle=False) as data:
        arrays = {key: data[key].copy() for key in data.files}
    update(arrays)
    np.savez_compressed(attempt / filename, **arrays)
    _seal(attempt)


@pytest.mark.parametrize('filename,value', [
    ('candidate.json', []), ('validation.json', []), ('process.json', []),
    ('input.json', []), ('request.json', []), ('receipt.json', []),
    ('result.json', []), ('candidate.json', None), ('validation.json', 3),
])
def test_non_object_json_fails_closed(evidence, filename, value):
    run, manifest, job, attempt = evidence
    write(attempt / filename, value)
    if filename not in {'result.json', 'receipt.json'}:
        _seal(attempt)
    decision = outcomes.assess_saved_output(attempt)
    assert decision['accepted'] is False
    row = results.attempt_row(run, manifest, job, attempt)
    assert row['status'] != 'success_numerical'
    assert dataset.numerical_witness(attempt / 'result.json', job['case'], run.parents[1]) is None


@pytest.mark.parametrize('filename', ['candidate.json', 'validation.json', 'process.json', 'result.json', 'receipt.json'])
def test_unreadable_json_does_not_crash_aggregation(evidence, filename):
    run, manifest, job, attempt = evidence
    (attempt / filename).write_bytes(b'{this is not json')
    decision = outcomes.assess_saved_output(attempt)
    assert not decision['accepted']
    row = results.attempt_row(run, manifest, job, attempt)
    assert row['status'] != 'success_numerical'


@pytest.mark.parametrize('field', ['samples', 'normalized_path_length', 'actual_maximum_sample_joint_step_rad', 'path_length_rad', 'minimum_sampled_clearance_m'])
def test_historical_missing_numeric_summary_is_not_inferred(evidence, field):
    *_, attempt = evidence
    value = json.loads((attempt / 'validation.json').read_text())
    value.pop(field, None)
    write(attempt / 'validation.json', value)
    _seal(attempt)
    assert outcomes.assess_saved_output(attempt)['accepted'] is False


@pytest.mark.parametrize('mutation', ['wrong_q', 'wrong_time', 'fractional_segment', 'integer_collision_mask', 'nonfinite_clearance', 'false_collision_mask'])
def test_saved_sample_arrays_must_match_candidate_and_summary(evidence, mutation):
    *_, attempt = evidence
    def mutate(a):
        if mutation == 'wrong_q': a['q'][len(a['q'])//2, 0] += .1
        elif mutation == 'wrong_time': a['time'] = a['time'][::-1]
        elif mutation == 'fractional_segment': a['segment'] = a['segment'].astype(float)+.1
        elif mutation == 'integer_collision_mask': a['native'] = a['native'].astype(np.int64)
        elif mutation == 'nonfinite_clearance': a['clearance'][0] = np.inf
        elif mutation == 'false_collision_mask': a['native'][0] = False
    _replace_archive(attempt, 'trajectory_samples.npz', mutate)
    assert outcomes.assess_saved_output(attempt)['accepted'] is False


def test_zero_active_spline_duration_rejected(evidence):
    *_, attempt = evidence
    _replace_archive(attempt, 'candidate.npz', lambda a: a.__setitem__('knots', np.zeros_like(a['knots'])))
    assert not outcomes.assess_saved_output(attempt)['accepted']


def test_self_consistent_out_of_joint_bounds_rejected(evidence):
    _, _, job, attempt = evidence
    save_numeric_output(attempt, job['case'], job['evaluation_config'], length=100., solver_success=False)
    _seal(attempt)
    assert not outcomes.assess_saved_output(attempt)['accepted']


def test_candidate_archive_cannot_override_metadata(evidence):
    *_, attempt = evidence
    _replace_archive(attempt, 'candidate.npz', lambda a: a.__setitem__('objective', np.array([0.])))
    assert not outcomes.assess_saved_output(attempt)['accepted']


@pytest.mark.parametrize('key,value', [('cost', True), ('cost', -1.), ('samples', True), ('native_collisions', False)])
def test_scalar_boolean_or_negative_count_or_cost_rejected(evidence, key, value):
    *_, attempt = evidence
    _change(attempt, 'validation.json', **{key:value})
    assert not outcomes.assess_saved_output(attempt)['accepted']


def test_witness_existential_does_not_reselect_attempt(evidence, tmp_path, monkeypatch):
    run, manifest, job, attempt = evidence
    later = attempt.with_name('0002')
    attempt.rename(later)
    attempt.mkdir()
    write(attempt/'result.json', dict(status='solver_failed'))
    _seal(attempt)
    assert outcomes.assess_saved_output(later)['accepted']
    monkeypatch.setattr('benchmarks.generic_motion_v1.batch.verify', lambda *a,**k:dict(integrity_passed=True))
    indexed,_ = results.collect([run])
    picked,_ = results._selected_rows(indexed, dict(id='independent', runs=[str(run)]))
    assert len(picked)==1 and picked[0]['attempt_number']==1
    assert picked[0]['status']!='success_numerical'
    witnesses,_ = dataset.collect_witnesses([job['case']], tmp_path)
    assert witnesses['dev-case']['result'].endswith('0002/result.json')


def test_dataset_scanner_skips_non_object_result(tmp_path):
    path = tmp_path/'results/run/jobs/task/attempts/0001/result.json'
    write(path, [])
    witnesses, stats = dataset.collect_witnesses([], tmp_path)
    assert witnesses == {}
    assert stats.get('unreadable_result_records', 0) == 1


@pytest.mark.parametrize('field,value', [('problem_id', []), ('status', [])])
def test_dataset_scanner_skips_wrongly_typed_identity_fields(evidence, tmp_path, field, value):
    _, _, job, attempt=evidence
    data=json.loads((attempt/'result.json').read_text())
    data[field]=value
    write(attempt/'result.json', data)
    witnesses, stats=dataset.collect_witnesses([job['case']], tmp_path)
    assert witnesses=={}


def test_receipt_file_io_failure_does_not_crash_aggregation(evidence, monkeypatch):
    run, manifest, job, attempt=evidence
    original=results.sha
    def fail_one(path):
        if Path(path).name=='candidate.npz':
            raise PermissionError('diagnostic unreadable artifact')
        return original(path)
    monkeypatch.setattr(results, 'sha', fail_one)
    row=results.attempt_row(run, manifest, job, attempt)
    assert row['status']=='infrastructure_error'


def test_unsupported_candidate_representation_cannot_be_success(evidence):
    *_,attempt=evidence
    _change(attempt,'candidate.json',kind='not_a_trajectory')
    assert outcomes.assess_saved_output(attempt)['accepted'] is False


def test_non_npz_coefficients_cannot_be_success(evidence):
    *_,attempt=evidence
    (attempt/'candidate.npz').write_bytes(b'not an array archive')
    _seal(attempt)
    assert outcomes.assess_saved_output(attempt)['accepted'] is False


def test_nonfinite_saved_candidate_cannot_be_success(evidence):
    *_,attempt=evidence
    np.savez_compressed(attempt/'candidate.npz',controls=np.full((2,7),np.nan),knots=[0.,0.,1.,1.])
    _seal(attempt)
    assert outcomes.assess_saved_output(attempt)['accepted'] is False


def test_reported_actual_sampling_step_must_respect_bound(evidence):
    *_,attempt=evidence
    _change(attempt,'validation.json',actual_maximum_sample_joint_step_rad=.5)
    assert outcomes.assess_saved_output(attempt)['accepted'] is False


def test_saved_collision_samples_cannot_disagree_with_passing_summary(evidence):
    *_,attempt=evidence
    q=np.linspace(np.zeros(7),np.ones(7),1000)
    np.savez_compressed(attempt/'trajectory_samples.npz',q=q,time=np.linspace(0,1,len(q)),
        native=np.zeros(len(q),bool),independent=np.zeros(len(q),bool),clearance=np.full(len(q),-.1))
    _seal(attempt)
    assert outcomes.assess_saved_output(attempt)['accepted'] is False


def test_manifest_cache_is_scoped_to_one_operation(evidence,monkeypatch):
    run,manifest,job,attempt=evidence
    reads=[]
    original=outcomes.read
    def counted(path):
        if Path(path).name=='run_manifest.json': reads.append(str(path))
        return original(path)
    monkeypatch.setattr(outcomes,'read',counted)
    cache={}
    assert outcomes.assess_saved_output(attempt,evidence_cache=cache)['accepted']
    assert outcomes.assess_saved_output(attempt,evidence_cache=cache)['accepted']
    assert len(reads)==1
    manifest['timeout_seconds']=59
    write(run/'run_manifest.json',manifest)
    assert not outcomes.assess_saved_output(attempt)['accepted']
    assert len(reads)==2


@pytest.mark.parametrize('residual',[None,True,1.])
def test_region_residual_summary_cannot_contradict_pass(evidence,residual):
    *_,attempt=evidence
    _change(attempt,'validation.json',region_residual=residual)
    assert not outcomes.assess_saved_output(attempt)['accepted']


@pytest.mark.parametrize('file,changes,expected',[
    ('process.json',{'timed_out':True},'timeout'),
    ('process.json',{'cold_e2e_seconds':61.},'timeout'),
    ('process.json',{'exit_code':2},'infrastructure_error'),
    ('result.json',{'exception':'CUDA failure'},'infrastructure_error'),
])
def test_external_failure_evidence_overrides_solver_flag(evidence,file,changes,expected):
    run,manifest,job,attempt=evidence
    _change(attempt,file,**changes)
    assert results.attempt_row(run,manifest,job,attempt)['status']==expected
