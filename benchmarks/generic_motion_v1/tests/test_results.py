"""Accounting regression tests: failed attempts cannot become successful rows."""
import csv
import json
from pathlib import Path

import pytest
from benchmarks.generic_motion_v1.tests.evidence_fixtures import save_numeric_output
from benchmarks.generic_motion_v1.common import BASE, read

from benchmarks.generic_motion_v1.common import sha, write
from benchmarks.generic_motion_v1.runners import results
from benchmarks.generic_motion_v1.runners.failure_classification import HANDOFF_SOURCE


def make_run(root, name="run", method="EI_ZO", objective="minimum_distance", category="open_control"):
    run = root / name
    run.mkdir()
    job = dict(job_id="job", case_id="test-open_control-000-p00", method=method,
               objective=objective, method_seed=41, replica_seed=0, roadmap_id="r0",
               evaluation_config={"validation_step_rad": .005, "endpoint_tolerance_rad": 1e-5},
               case=dict(problem_id="test-open_control-000-p00", scene_id="test-open_control-000",
                         split="test", family=category, validation_status="valid_input",
                         problem_sha256="input", robot_sha256="robot", collision_geometry_sha256="geometry", q_start=[0.]*7, q_goal=[0.]*7))
    robot = read(BASE/"configs/robot.json")
    job["case"].update({k:robot[k] for k in ("robot_sha256","collision_geometry_sha256")})
    manifest = dict(jobs=[job], source_code_commit="commit", source_hashes={"runner": "abc"},
                    timeout_seconds=60, split="test")
    write(run / "run_manifest.json", manifest)
    return run, manifest, job


def make_attempt(run, job, n=1, status="success_validated_numerical", valid=True, retry=False, cost=4.):
    out = run / "jobs" / job["job_id"] / "attempts" / f"{n:04d}"
    out.mkdir(parents=True)
    result = dict(status=status, problem_id=job["case"]["problem_id"],
                  scene_id=job["case"]["scene_id"], family=job["case"]["family"],
                  method=job["method"], objective=job["objective"], method_seed=job["method_seed"],
                  budget_seconds=60, timing={"cold_e2e_seconds": 1., "warm_query_seconds": .8,
                  "trajectory_optimization_seconds": .2, "validation_seconds": .3})
    write(out / "result.json", result)
    write(out / "process.json", dict(cold_e2e_seconds=1.5, exit_code=0, timed_out=False, timeout_seconds=60))
    write(out / "request.json", job)
    write(out / "candidate.json", dict(solver_success=True, kind="polygon", objective=job["objective"]))
    write(out / "input.json", job["case"])
    save_numeric_output(out,job['case'],job['evaluation_config'],length=cost,passed=valid)
    files = {p.name: sha(p) for p in out.iterdir() if p.is_file()}
    write(out / "receipt.json", dict(status=status, retry_pending=retry, files=files))
    return out


@pytest.fixture(autouse=True)
def isolate_provenance(monkeypatch):
    # Real provenance verification has extensive separate tests. These fixtures
    # exercise attempt choice/statistics with tiny synthetic receipts.
    monkeypatch.setattr("benchmarks.generic_motion_v1.batch.verify",
                        lambda *a, **kw: {"integrity_passed": True})


@pytest.mark.parametrize("raw,expected", [
    ("invalid_endpoint", "validation_failed"),
    ("planner_timeout", "no_solution_within_budget"),
    ("frontend_failed", "no_solution_within_budget"),
    ("timeout", "timeout"),
    ("unsupported", "infrastructure_error"),
    ("model_consistency_failed", "infrastructure_error"),
    ("collision_failed", "validation_failed"),
    ("solver_failed", "solver_failure"),
])
def test_status_mapping_preserves_reason(tmp_path, raw, expected):
    run, manifest, job = make_run(tmp_path)
    attempt = make_attempt(run, job, status=raw)
    row = results.attempt_row(run, manifest, job, attempt)
    assert row["status"] == expected
    assert row["raw_status"] == raw
    assert row["objective_value"] is None


def _reseal(attempt):
    result = read(attempt/'result.json')
    files = {p.relative_to(attempt).as_posix():sha(p) for p in attempt.rglob('*')
             if p.is_file() and p != attempt/'receipt.json'}
    write(attempt/'receipt.json', dict(status=result['status'], files=files))


@pytest.mark.parametrize('raw', ['solver_failed', 'timeout', 'frontend_failed', 'validation_undetermined'])
@pytest.mark.parametrize('timed_out', [False, True])
def test_unknown_saved_exception_cannot_be_normalized_as_timeout_or_solver_failure(tmp_path, raw, timed_out):
    run, manifest, job = make_run(tmp_path)
    attempt = make_attempt(run, job, status=raw, valid=False)
    result = read(attempt/'result.json')
    result.update(exception="KeyError('unexpected backend state')", failed_stage='trajectory_optimization')
    write(attempt/'result.json', result)
    write(attempt/'process.json', dict(exit_code=-15 if timed_out else 0,
        timed_out=timed_out, cold_e2e_seconds=60.1 if timed_out else 1.5))
    _reseal(attempt)
    row = results.attempt_row(run, manifest, job, attempt)
    assert row['status'] == 'infrastructure_error' and row['raw_status'] == raw
    assert row['objective_value'] is None


@pytest.mark.parametrize('change', ['unknown_category', 'cuda_category', 'crash_during_timeout'])
def test_saved_backend_metadata_takes_precedence_over_timeout(tmp_path, change):
    run, manifest, job = make_run(tmp_path)
    attempt = make_attempt(run, job, status='timeout', valid=False)
    result = read(attempt/'result.json')
    if change.endswith('category'):
        result['error_category'] = 'CUDA_error' if change == 'cuda_category' else 'unreviewed_internal_error'
        write(attempt/'result.json', result)
    else:
        write(attempt/'process.json', dict(exit_code=-11, timed_out=True, cold_e2e_seconds=60.1))
    _reseal(attempt)
    assert results.attempt_row(run, manifest, job, attempt)['status'] == 'infrastructure_error'


def test_typed_numerical_handoff_rejection_stays_solver_failure_with_complete_evidence(tmp_path):
    from benchmarks.generic_motion_v1.common import ROOT
    from benchmarks.generic_motion_v1.tests.test_exception_classification import real_handoff_failure
    run, manifest, job = make_run(tmp_path)
    job['evaluation_config']['region_coverage_tolerance_rad'] = 1e-6
    manifest['source_hashes'][HANDOFF_SOURCE] = sha(ROOT/HANDOFF_SOURCE)
    attempt = make_attempt(run, job, status='solver_failed', valid=False)
    result = read(attempt/'result.json'); result.update(real_handoff_failure(attempt))
    write(attempt/'result.json', result); _reseal(attempt)
    before = {p.relative_to(attempt).as_posix():p.read_bytes() for p in attempt.rglob('*') if p.is_file()}
    row = results.attempt_row(run, manifest, job, attempt)
    assert row['status'] == 'solver_failure' and row['raw_status'] == 'solver_failed'
    assert before == {p.relative_to(attempt).as_posix():p.read_bytes() for p in attempt.rglob('*') if p.is_file()}
    missing_source = dict(manifest, source_hashes={})
    assert results.attempt_row(run, missing_source, job, attempt)['status'] == 'infrastructure_error'
    assert results.attempt_row(run, manifest, job, attempt, verify_files=False)['status'] == 'infrastructure_error'
    result['exception'] = result['exception'].replace('NativeHandoffError','ValueError')
    write(attempt/'result.json', result); _reseal(attempt)
    assert results.attempt_row(run, manifest, job, attempt)['status'] == 'infrastructure_error'


def test_reviewed_legacy_no_route_keeps_historical_search_failure(tmp_path):
    from benchmarks.generic_motion_v1.tests.test_exception_classification import legacy_drm_fixture
    run, manifest, job = make_run(tmp_path)
    attempt = make_attempt(run, job, status='frontend_failed', valid=False)
    fields, _, sources = legacy_drm_fixture(attempt)
    manifest['source_hashes'].update(sources)
    result = read(attempt/'result.json');result.update(fields)
    write(attempt/'result.json',result);_reseal(attempt)
    assert results.attempt_row(run,manifest,job,attempt)['status']=='no_solution_within_budget'
    assert results.attempt_row(run,dict(manifest,source_hashes={}),job,attempt)['status']=='infrastructure_error'


def test_success_requires_standalone_validation_and_receipt(tmp_path):
    run, manifest, job = make_run(tmp_path)
    attempt = make_attempt(run, job, valid=False)
    row = results.attempt_row(run, manifest, job, attempt)
    assert row["status"] == "validation_failed"
    assert row["path_length_rad"] is None
    write(attempt / "validation.json", dict(passed=True, cost=1, cost_unit="rad"))
    row = results.attempt_row(run, manifest, job, attempt)
    assert row["status"] == "infrastructure_error"
    assert row["receipt_integrity"] is False


def test_no_first_solution_or_duration_inferred_from_wall_time(tmp_path):
    run, manifest, job = make_run(tmp_path)
    row = results.attempt_row(run, manifest, job, make_attempt(run, job))
    assert row["cold_e2e_seconds"] == 1.5
    assert row["worker_seconds"] == 1.
    assert row["within_call_query_seconds"] == .8
    assert row["first_valid_seconds"] is None
    assert row["trajectory_duration_s"] is None
    assert row["cluster_id"] == "open_control_shared_base_world"


def test_all_attempts_exported_but_no_best_of(tmp_path):
    run, manifest, job = make_run(tmp_path)
    make_attempt(run, job, 1, status="solver_failed")
    make_attempt(run, job, 2, cost=1.)
    rows, inventory = results.collect([run])
    chosen, planned = results._selected_rows(rows, {"id": "check", "runs": [str(run)]})
    assert len(rows) == 2
    assert inventory[0]["actual_attempts"] == 2
    assert chosen[0]["status"] == "solver_failure"
    assert chosen[0]["attempt_number"] == 1
    assert rows[1]["previous_attempt"] == str(run / "jobs/job/attempts/0001")
    assert planned[0]["actual_attempts"] == 2


def test_interrupted_retry_does_not_replace_terminal_observation(tmp_path):
    run, _, job = make_run(tmp_path)
    make_attempt(run, job, 1, status="infrastructure_error", retry=True)
    make_attempt(run, job, 2, status="timeout")
    make_attempt(run, job, 3, cost=.1)
    rows, _ = results.collect([run])
    chosen, _ = results._selected_rows(rows, {"id": "check", "runs": [str(run)]})
    assert chosen[0]["attempt_number"] == 2
    assert chosen[0]["status"] == "timeout"


def test_pending_jobs_do_not_create_fabricated_attempt_rows(tmp_path):
    run, _, _ = make_run(tmp_path)
    rows, _ = results.collect([run])
    summary = results.summarize(rows, {"comparisons": [{"id": "check", "runs": [str(run)]}]})
    assert rows == []
    status = next(r for r in summary if r["row_type"] == "status")
    assert status["planned_jobs"] == status["pending_jobs"] == 1
    assert status["actual_attempts"] == status["finished_legal_jobs"] == 0
    assert status["success_rate"] is None


def test_duplicate_run_observations_rejected(tmp_path):
    run, _, job = make_run(tmp_path)
    make_attempt(run, job)
    rows, _ = results.collect([run])
    with pytest.raises(ValueError, match="Duplicate observation"):
        results._selected_rows(rows, {"id": "check", "runs": [str(run), str(run)]})


def test_cost_pairs_require_both_to_succeed(tmp_path):
    left, _, jl = make_run(tmp_path, "left", method="EI_ZO")
    right, _, jr = make_run(tmp_path, "right", method="RRTCONNECT")
    make_attempt(left, jl, status="solver_failed")
    make_attempt(right, jr, cost=1.)
    rows, _ = results.collect([left, right])
    summary = results.summarize(rows, {"comparisons": [{"id": "check", "runs": [str(left), str(right)]}]})
    costs = [r for r in summary if r["row_type"] == "paired" and r["metric"] == "objective_value"]
    assert costs and all(r["count"] == 0 and r["mean"] is None for r in costs)
    success = next(r for r in summary if r["row_type"] == "paired" and r["metric"] == "success")
    assert success["count"] == 1 and success["mean"] == -1
    assert success["ci_low"] is None  # shared physical base is a single cluster


def test_protocol_mix_requires_explicit_compatibility(tmp_path):
    left, _, jl = make_run(tmp_path, "left", method="EI_ZO")
    right, manifest, jr = make_run(tmp_path, "right", method="RRTCONNECT")
    manifest["source_hashes"] = {"runner": "different"}
    write(right / "run_manifest.json", manifest)
    make_attempt(left, jl)
    make_attempt(right, jr)
    rows, _ = results.collect([left, right])
    spec = {"id": "check", "runs": [str(left), str(right)]}
    with pytest.raises(ValueError, match="compatibility_note"):
        results._selected_rows(rows, spec)
    spec["compatibility_note"] = "Only formatting changed; implementation and collision protocol were reviewed."
    assert len(results._selected_rows(rows, spec)[0]) == 2


def test_missing_csv_metrics_are_empty_not_zero(tmp_path):
    file = tmp_path / "rows.csv"
    results._csv(file, [{"first_valid_seconds": None, "trajectory_duration_s": None}], results.RECORD_FIELDS)
    row = next(csv.DictReader(file.open()))
    assert row["first_valid_seconds"] == row["trajectory_duration_s"] == ""


def test_adapter_first_valid_is_not_cold_first_valid(tmp_path):
    run, manifest, job = make_run(tmp_path, method="RRTCONNECT")
    attempt = make_attempt(run, job)
    write(attempt / "validated_progress/summary.json", dict(first_valid_seconds=.7,
        validated_observations=1, validation_charged_to_budget=True,
        time_origin="OMPL adapter entry; includes native dependency setup and all in-run validation"))
    receipt = json.loads((attempt / "receipt.json").read_text())
    receipt["files"]["validated_progress/summary.json"] = sha(attempt / "validated_progress/summary.json")
    write(attempt / "receipt.json", receipt)
    row = results.attempt_row(run, manifest, job, attempt)
    assert row["observed_first_valid_adapter_seconds"] == .7
    assert row["first_valid_seconds"] is None


def test_validation_protocol_cannot_be_overridden_by_note(tmp_path):
    left, _, jl = make_run(tmp_path, "left", method="EI_ZO")
    right, manifest, jr = make_run(tmp_path, "right", method="RRTCONNECT")
    jr["evaluation_config"]["validation_step_rad"] = .1
    write(right / "run_manifest.json", manifest)
    make_attempt(left, jl)
    make_attempt(right, jr)
    rows, _ = results.collect([left, right])
    with pytest.raises(ValueError, match="validation or motion-limit"):
        results._selected_rows(rows, {"id": "check", "runs": [str(left), str(right)],
                                      "compatibility_note": "This note cannot relax validation."})


def test_pending_run_is_explicit_not_fabricated(tmp_path):
    summary = results.summarize([], {"comparisons": [{"id": "current_test", "runs": [
        {"path": str(tmp_path / "future"), "allow_pending": True}]}]})
    assert len(summary) == 1
    assert summary[0]["row_type"] == "coverage"
    assert "success_rate" not in summary[0]


def test_result_indexes_are_links_not_copies(tmp_path):
    run, _, job = make_run(tmp_path)
    make_attempt(run, job)
    output = tmp_path / "index"
    results._index_links(output, [run])
    results._index_links(output, [run])  # idempotent for incremental refresh
    for name in ("trajectories", "logs"):
        link = output / name / run.name
        assert link.is_symlink()
        assert link.resolve() == (run / "jobs").resolve()



def _bound_bytes(attempt, name, payload):
    path = attempt / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)
    receipt = json.loads((attempt / "receipt.json").read_text())
    receipt["files"][name] = sha(path)
    write(attempt / "receipt.json", receipt)
    return path


def test_invalid_input_endpoint_remains_invalid_input(tmp_path):
    run, manifest, job = make_run(tmp_path)
    job["case"]["validation_status"] = "generation_failed"
    row = results.attempt_row(run, manifest, job,
                             make_attempt(run, job, status="invalid_endpoint"))
    assert row["input_valid"] is False
    assert row["status"] == "invalid_input"


@pytest.mark.parametrize("method,initial_name", [
    ("EI_ZO", "rounds/000/regions.npz"),
    ("LGCS", "first_corridor/regions.npz"),
])
def test_initial_region_identity_uses_actual_pre_repair_artifact(tmp_path, method, initial_name):
    run, manifest, job = make_run(tmp_path, method=method)
    attempt = make_attempt(run, job)
    initial = _bound_bytes(attempt, initial_name, b"original region arrays")
    final = _bound_bytes(attempt, "regions.npz", b"different corrected region arrays")
    obsolete = _bound_bytes(attempt, "initial_regions.npz", b"wrong legacy filename")
    row = results.attempt_row(run, manifest, job, attempt)
    assert row["initial_regions_sha256"] == sha(initial)
    assert row["initial_regions_sha256"] not in {sha(final), sha(obsolete)}
    assert row["receipt_integrity"] is True


@pytest.mark.parametrize("method", ["EI_ZO", "LGCS"])
def test_final_regions_do_not_stand_in_for_missing_initial_regions(tmp_path, method):
    run, manifest, job = make_run(tmp_path, method=method)
    attempt = make_attempt(run, job)
    _bound_bytes(attempt, "regions.npz", b"only final corrected region arrays")
    assert results.attempt_row(run, manifest, job, attempt)["initial_regions_sha256"] is None


def test_fixed_initial_region_gate_rejects_different_saved_regions(tmp_path):
    left, _, jl = make_run(tmp_path, "left", method="EI_ZO")
    right, _, jr = make_run(tmp_path, "right", method="LGCS")
    la, ra = make_attempt(left, jl), make_attempt(right, jr)
    for attempt in (la, ra):
        _bound_bytes(attempt, "path.npy", b"same fixed seed path")
        _bound_bytes(attempt, "frontend_validation/validation.json", b'{"passed":true}')
    _bound_bytes(la, "rounds/000/regions.npz", b"same original region arrays")
    _bound_bytes(ra, "first_corridor/regions.npz", b"same original region arrays")
    spec = {"id": "initial_regions", "mode": "fixed_frontend",
            "require_shared_initial_regions": True, "runs": [str(left), str(right)]}
    rows, _ = results.collect([left, right])
    assert len(results._selected_rows(rows, spec)[0]) == 2
    _bound_bytes(ra, "first_corridor/regions.npz", b"different original region arrays")
    rows, _ = results.collect([left, right])
    with pytest.raises(ValueError, match="Shared initial regions"):
        results._selected_rows(rows, spec)


def test_figure_scope_does_not_modify_original_rows_or_memberships():
    rows = [
        {"record_id": "old", "selected_comparisons": ["current_old"]},
        {"record_id": "shared", "selected_comparisons": ["current_old", "current_new"]},
        {"record_id": "new", "selected_comparisons": ["current_new"]},
        {"record_id": "unselected", "selected_comparisons": []},
    ]
    summary = [{"comparison": "current_old"}, {"comparison": "current_new"}]
    original = json.loads(json.dumps([rows, summary]))
    scoped_rows, scoped_summary = results._figure_inputs(rows, summary, {"current_new"})
    assert [row["record_id"] for row in scoped_rows] == ["shared", "new"]
    assert all(row["selected_comparisons"] == ["current_new"] for row in scoped_rows)
    assert scoped_summary == [{"comparison": "current_new"}]
    assert [rows, summary] == original
    # Even a downstream change to the copied membership cannot contaminate CSV data.
    scoped_rows[0]["selected_comparisons"].append("renderer_local")
    assert [rows, summary] == original


def test_figure_scope_absent_preserves_existing_behavior():
    rows, summary = [{"selected_comparisons": ["current_old"]}], [{"comparison": "current_old"}]
    scope = results._figure_comparisons({"comparisons": [{"id": "current_old"}]})
    actual_rows, actual_summary = results._figure_inputs(rows, summary, scope)
    assert actual_rows is rows
    assert actual_summary is summary


@pytest.mark.parametrize("scope", [
    [], None, "current_new", [""], [" "], [7], ["current_new", "current_new"], ["typo"],
])
def test_invalid_figure_scope_rejected_before_export(tmp_path, monkeypatch, scope):
    selection = tmp_path / "selection.json"
    write(selection, {"comparisons": [{"id": "current_new", "runs": []}], "figure_comparisons": scope})
    def cannot_discover(*args, **kwargs):
        pytest.fail("Invalid figure scope must fail before discovery or any output writes")
    monkeypatch.setattr(results, "discover", cannot_discover)
    with pytest.raises(ValueError, match="figure_comparisons"):
        results.export(tmp_path / "output", roots=[tmp_path], selection=selection)
    assert not (tmp_path / "output").exists()


def test_export_figure_scope_keeps_full_csv_history(tmp_path, monkeypatch):
    rows = [
        {"record_id": "old", "selected_comparisons": ["current_old"], "status": "timeout"},
        {"record_id": "new", "selected_comparisons": ["current_old", "current_new"],
         "status": "no_solution_within_budget"},
    ]
    summary = [
        {"comparison": "current_old", "row_type": "status", "finished_legal_jobs": 2},
        {"comparison": "current_new", "row_type": "status", "finished_legal_jobs": 1},
    ]
    original = json.loads(json.dumps([rows, summary]))
    selection = tmp_path / "selection.json"
    write(selection, {"comparisons": [{"id": key, "runs": []} for key in ("current_old", "current_new")],
                      "figure_comparisons": ["current_new"]})
    monkeypatch.setattr(results, "discover", lambda _: [])
    monkeypatch.setattr(results, "collect", lambda *a, **kw: (rows, []))
    monkeypatch.setattr(results, "summarize", lambda *a: summary)
    captured = []
    monkeypatch.setattr(results, "_figures", lambda *args: captured.append(args))
    output = tmp_path / "output"
    report = results.export(output, roots=[tmp_path], selection=selection)
    assert report["actual_attempts"] == report["summary_rows"] == 2
    with (output / "records.csv").open() as stream:
        exported = list(csv.DictReader(stream))
    assert [row["record_id"] for row in exported] == ["old", "new"]
    assert json.loads(exported[1]["selected_comparisons"]) == ["current_old", "current_new"]
    with (output / "summary.csv").open() as stream:
        assert [row["comparison"] for row in csv.DictReader(stream)] == ["current_old", "current_new"]
    assert len(captured) == 1
    _, figure_rows, figure_summary = captured[0]
    assert [row["record_id"] for row in figure_rows] == ["new"]
    assert figure_rows[0]["selected_comparisons"] == ["current_new"]
    assert figure_summary == [summary[1]]
    assert [rows, summary] == original
