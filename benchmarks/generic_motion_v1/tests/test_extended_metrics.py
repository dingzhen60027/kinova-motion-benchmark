"""Synthetic accounting fixtures only; not benchmark or solver evidence."""
import csv
import shutil
import numpy as np
import pytest
from benchmarks.generic_motion_v1.runners import extended_metrics as extended
from benchmarks.generic_motion_v1.runners import study_metrics as study
from benchmarks.generic_motion_v1.tests.test_study_metrics import make_run, _case, _read, _write

METHODS = (("EI_ZO", "minimum_distance"), ("RRTCONNECT", "minimum_distance"),
           ("RRTSTAR", "minimum_distance"), ("CHOMP", "minimum_distance"),
           ("STOMP", "minimum_distance"), ("TRAJOPT", "minimum_distance"))

@pytest.fixture
def extended_run(make_run, monkeypatch):
    for method, label in zip(METHODS, extended.LABELS):
        monkeypatch.setitem(study.LABELS, method, label)
    def make(name="extended", cases=None, outcomes=None, **kwargs):
        root = make_run(name, cases=cases, methods=METHODS, outcomes=outcomes, **kwargs)
        manifest = _read(root / "run_manifest.json")
        for job in manifest["jobs"]:
            label = study.LABELS[(job["method"], job["objective"])]
            attempt = root / "jobs" / job["job_id"] / "attempts/0001"
            result = _read(attempt / "result.json")
            result["roadmap_used"] = label not in extended.INDEPENDENT
            if label in extended.INDEPENDENT:
                result.update(frontend_native_success=None, frontend_success=None, drm_success=None)
                (attempt / "path.npy").unlink(missing_ok=True)
                if (attempt / "frontend_validation").exists():
                    shutil.rmtree(attempt / "frontend_validation")
            _write(attempt / "result.json", result)
        return root
    return make

def _attempt(run, label, case="p0"):
    return run / "jobs" / f"{case}-{label}" / "attempts/0001"

def test_complete_six_method_pairings_and_no_drm_denominator_for_ompl(extended_run, tmp_path):
    run = extended_run()
    result = extended.analyze([run], tmp_path / "metrics")
    assert result["complete"] and result["comparable"]
    assert result["requested_jobs"] == 6
    all_groups = [g for g in result["groups"] if g["scope"] == "all"]
    assert len(all_groups) == 6
    assert len([p for p in result["paired_comparisons"] if p["scope"] == "all"]) == 15
    for group in all_groups:
        independent = group["label"] in extended.INDEPENDENT
        for key in extended.CONDITIONAL_DRM_FIELDS:
            assert group[key]["applicable"] is (not independent)
            assert group[key]["denominator"] == (None if independent else 1)
    assert len(result["drm_references"]) == 1
    rows = list(csv.DictReader((tmp_path / "metrics/observations.csv").open()))
    assert len(rows) == 6
    assert all("native_objective" in row and "initialization" in row for row in rows)

def test_all_legal_failures_count_invalid_inputs_do_not_dilute_rate(extended_run, tmp_path):
    cases = [_case("ok"), _case("fail"), _case("invalid", valid="invalid_endpoint")]
    outcomes = {("fail", label): dict(status="planner_timeout") for label in extended.LABELS}
    run = extended_run(cases=cases, outcomes=outcomes)
    result = extended.analyze([run], tmp_path / "metrics")
    for group in (g for g in result["groups"] if g["scope"] == "all"):
        assert group["requested_jobs"] == 3
        assert group["legal_jobs"] == 2
        assert group["invalid_jobs"] == 1
        assert group["valid_success"]["estimate"] == .5
        assert group["statistics"]["cold_e2e_seconds"]["all_legal"]["count"] == 2
    for pair in (p for p in result["paired_comparisons"] if p["scope"] == "all"):
        assert pair["paired_legal_jobs"] == 2
        assert pair["common_success_cost_difference"]["denominator"] == 1

def test_open_control_shared_world_not_recounted_as_independent_scenes(extended_run, tmp_path):
    run = extended_run(cases=[_case("control0", "open_control"), _case("control1", "open_control")])
    result = extended.analyze([run], tmp_path / "metrics")
    for group in result["groups"]:
        assert group["legal_scene_clusters"] == 1
        assert group["valid_success"]["ci95"] is None

@pytest.mark.parametrize("field,value", [("roadmap_used", True), ("frontend_success", False),
                                         ("frontend_native_success", False), ("drm_success", False)])
def test_independent_search_cannot_claim_shared_drm(extended_run, tmp_path, field, value):
    run = extended_run()
    path = _attempt(run, "RRTConnect") / "result.json"
    result = _read(path); result[field] = value; _write(path, result)
    with pytest.raises(ValueError, match="Independent OMPL"):
        extended.analyze([run], tmp_path / "metrics")

def test_independent_search_cannot_carry_seed_path(extended_run, tmp_path):
    run = extended_run()
    np.save(_attempt(run, "RRT*") / "path.npy", np.zeros((2, 7)))
    with pytest.raises(ValueError, match="Independent OMPL"):
        extended.analyze([run], tmp_path / "metrics")

def test_shared_frontend_mismatch_saved_as_noncomparable(extended_run, tmp_path):
    run = extended_run()
    path = _attempt(run, "CHOMP") / "result.json"
    result = _read(path); result["frontend_native_success"] = False; _write(path, result)
    with pytest.raises(ValueError, match="Shared DRM mismatch"):
        extended.analyze([run], tmp_path / "metrics")
    report = _read(tmp_path / "metrics/metrics.json")
    assert report["complete"] is False and report["comparable"] is False
    assert report["pairing_differences"]

def test_missing_method_rejected_even_if_other_rows_are_successful(extended_run, tmp_path):
    run = extended_run()
    manifest = _read(run / "run_manifest.json"); manifest["jobs"] = manifest["jobs"][:-1]
    _write(run / "run_manifest.json", manifest)
    with pytest.raises(ValueError, match="all six methods"):
        extended.analyze([run], tmp_path / "metrics")

def test_identical_input_hash_does_not_hide_endpoint_mismatch(extended_run, tmp_path):
    run = extended_run()
    manifest = _read(run / "run_manifest.json")
    manifest["jobs"][1]["case"]["q_goal"][0] += .001
    _write(run / "run_manifest.json", manifest)
    with pytest.raises(ValueError, match="q_goal"):
        extended.analyze([run], tmp_path / "metrics")

@pytest.mark.parametrize("field", ["evaluation_config", "source_hashes", "timeout_seconds"])
def test_different_configs_sources_or_budgets_cannot_be_pooled(extended_run, tmp_path, field):
    first = extended_run("first", cases=[_case("p0")])
    second = extended_run("second", cases=[_case("p1")])
    manifest = _read(second / "run_manifest.json")
    if field == "evaluation_config":
        for job in manifest["jobs"]: job[field] = {"different": True}
    elif field == "source_hashes": manifest[field]["runners/ompl_baselines.py"] = "different"
    else: manifest[field] = 30
    _write(second / "run_manifest.json", manifest)
    with pytest.raises(ValueError, match="cannot be pooled"):
        extended.analyze([first, second], tmp_path / "metrics")

def test_native_objective_from_candidate_remains_separate_from_evaluation(extended_run, tmp_path):
    run = extended_run()
    path = _attempt(run, "CHOMP") / "candidate.json"
    candidate = _read(path); candidate["native_objective"] = "fixture smoothness objective, not path length"
    _write(path, candidate)
    result = extended.analyze([run], tmp_path / "metrics")
    row = next(row for row in result["rows"] if row["label"] == "CHOMP")
    assert row["native_objective"] == candidate["native_objective"]
    assert row["native_objective_source"] == "candidate"
    assert row["evaluation_objective"] != row["native_objective"]

def test_unverified_evidence_refused_before_analysis(extended_run, monkeypatch, tmp_path):
    run = extended_run()
    monkeypatch.setattr(study, "verify", lambda *a, **k: dict(integrity_passed=False, complete=True))
    with pytest.raises(ValueError, match="unverified evidence"):
        extended.analyze([run], tmp_path / "metrics")

@pytest.mark.parametrize('status', ['success_validated_numerical', 'collision_failed'])
def test_returned_independent_sampler_has_complete_timing_without_optimizer_timer(status):
    timing = dict(model_setup_seconds=.1, sampling_planning_seconds=2., validation_seconds=.5)
    selected, complete, excluded, reasons = extended.independent_completed_stage_times(
        timing, dict(status=status), True, True)
    assert selected == timing and complete
    assert excluded == reasons == []

@pytest.mark.parametrize('status,candidate,validation', [
    ('planner_timeout', False, False), ('timeout', True, False),
    ('planner_failed', False, False), ('success_validated_numerical', False, True),
])
def test_incomplete_independent_sampler_keeps_raw_timing_but_excludes_stage_statistics(status, candidate, validation):
    timing = dict(model_setup_seconds=.1, sampling_planning_seconds=2., validation_seconds=0.)
    original = timing.copy()
    selected, complete, excluded, reasons = extended.independent_completed_stage_times(
        timing, dict(status=status), candidate, validation)
    assert timing == original
    assert not complete and reasons
    assert selected == dict(model_setup_seconds=.1)
    assert set(excluded) == {'sampling_planning_seconds', 'validation_seconds'}

def test_independent_completion_contract_is_used_by_analyzer(extended_run, tmp_path):
    run = extended_run()
    for label in extended.INDEPENDENT:
        path = _attempt(run, label) / 'result.json'
        result = _read(path)
        result['timing'] = dict(sampling_planning_seconds=1., validation_seconds=.2)
        _write(path, result)
    report = extended.analyze([run], tmp_path / 'metrics')
    for group in report['groups']:
        if group['label'] in extended.INDEPENDENT:
            assert group['complete_backend_timing_jobs'] == 1
            assert group['stage_seconds']['sampling_planning_seconds']['all_legal']['count'] == 1
            assert 'no trajectory-optimization stage' in group['stage_timing_scope']

@pytest.mark.parametrize('converged,termination,status,success',[
    (True,'CONVERGED','success_validated_numerical',True),
    (False,'INTERNAL_TIME_LIMIT','success_validated_numerical',True),
    (False,'SCO_ITERATION_LIMIT','success_validated_numerical',True),
    (False,'INTERNAL_TIME_LIMIT','collision_failed',False),
    (None,'CONVERGED','success_validated_numerical',True),
])
def test_native_convergence_is_separate_from_common_validation_and_cost(
        extended_run,tmp_path,converged,termination,status,success):
    run=extended_run(outcomes={('p0','TrajOpt'):dict(status=status,cost=3.)})
    path=_attempt(run,'TrajOpt')/'candidate.json'
    candidate=_read(path)
    candidate.update(solver_converged=converged,termination=termination,
                     native_status='native-status',solver_status='solver-status',cost=999.,solver_objective=-123.)
    _write(path,candidate)
    report=extended.analyze([run],tmp_path/'metrics')
    row=next(row for row in report['rows'] if row['label']=='TrajOpt')
    assert row['candidate_native_status']=='native-status'
    assert row['candidate_solver_status']=='solver-status'
    assert row['candidate_termination']==termination
    assert row['reported_termination']==termination and row['termination_source']=='termination'
    assert row['cost']==(3. if success else None)
    group=next(g for g in report['groups'] if g['label']=='TrajOpt' and g['scope']=='all')
    assert group['termination_counts']=={termination:1}
    assert group['converged_successes']==int(success and converged is True)
    assert group['validated_incumbent_nonconverged']==int(success and converged is False)
    assert group['validated_unknown_convergence']==int(success and converged is None)
    assert group['unknown_convergence']==int(converged is None)
    assert group['converged_successes']+group['validated_incumbent_nonconverged']+group['validated_unknown_convergence']==int(success)

def test_missing_convergence_does_not_mean_converged_when_solver_success_is_true(extended_run,tmp_path):
    run=extended_run()
    report=extended.analyze([run],tmp_path/'metrics')
    for group in report['groups']:
        assert group['converged_successes']==0
        assert group['unknown_convergence']==1
        assert group['validated_unknown_convergence']==1

def test_no_candidate_has_separate_termination_and_unknown_count(extended_run,tmp_path):
    run=extended_run(outcomes={('p0','TrajOpt'):dict(status='timeout')})
    (_attempt(run,'TrajOpt')/'candidate.json').unlink()
    report=extended.analyze([run],tmp_path/'metrics')
    group=next(g for g in report['groups'] if g['label']=='TrajOpt' and g['scope']=='all')
    assert group['termination_counts']=={'no_candidate':1}
    assert group['unknown_convergence']==1 and group['unknown_convergence_candidates']==0
    assert group['no_candidate_jobs']==1

@pytest.mark.parametrize('value',[0,1,'false','CONVERGED'])
def test_malformed_explicit_convergence_flag_rejected(extended_run,tmp_path,value):
    run=extended_run()
    path=_attempt(run,'TrajOpt')/'candidate.json'
    candidate=_read(path);candidate['solver_converged']=value;_write(path,candidate)
    with pytest.raises(ValueError,match='explicit boolean'):
        extended.analyze([run],tmp_path/'metrics')
