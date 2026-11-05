"""CPU orchestration checks using tiny sealed planning and execution fixtures."""
import csv
from pathlib import Path

import pytest

from benchmarks.generic_motion_v1.common import read, sha, write
from benchmarks.generic_motion_v1.simulation import cli, execution


@pytest.fixture
def executor(monkeypatch):
    calls = []
    identity = {"simulator.py": "frozen-code"}
    monkeypatch.setattr(cli, "source_hashes", lambda: dict(identity))

    def run(attempt, output, config=None, render=False, disable_control=False):
        attempt, output = Path(attempt), Path(output)
        output.mkdir(parents=True, exist_ok=False)
        original = read(attempt / "result.json")
        calls.append(str(attempt))
        (output / "evidence.txt").write_text("fixed physical test fixture\n")
        result = dict(problem_id=original["problem_id"], scene_id=original["scene_id"], family=original["family"],
                      method=original["method"], original_planning_status=original["status"],
                      status="execution_success_numerical" if original["status"] == "success_validated_numerical" else "no_validated_reference",
                      source_attempt=str(attempt), source_receipt_sha256=sha(attempt / "receipt.json"),
                      simulation_source_hashes=dict(identity),
                      artifacts={"evidence.txt": sha(output / "evidence.txt")})
        write(output / "result.json", result)
        write(output / "receipt.json", dict(result_sha256=sha(output / "result.json")))
        return result

    monkeypatch.setattr(cli, "run", run)
    return calls, identity, run


def source_run(tmp_path, jobs=1, pending=False):
    directory = tmp_path / "planning"
    directory.mkdir()
    manifest = dict(jobs=[])
    for i in range(jobs):
        job = dict(job_id=f"job-{i}", method="NTO_REFINED", objective="minimum_time", problem_id=f"case-{i}",
                   scene_id=f"scene-{i}", family="open_control")
        manifest["jobs"].append(job)
        if not pending:
            source_attempt(directory, job, "0001")
    write(directory / "run_manifest.json", manifest)
    return directory


def source_attempt(directory, job, number, status="success_validated_numerical", retry_pending=False):
    attempt = directory / "jobs" / job["job_id"] / "attempts" / number
    attempt.mkdir(parents=True)
    result = dict(job, status=status)
    write(attempt / "result.json", result)
    write(attempt / "input.json", dict(job, validation_status="valid_input"))
    write(attempt / "request.json", job)
    write(attempt / "process.json", dict(exit_code=0))
    write(attempt / "receipt.json", dict(status=status, retry_pending=retry_pending,
                                        files={p.name: sha(p) for p in attempt.iterdir()}))
    return attempt


def test_single_run_cli_verify_and_changed_artifact_rejected(tmp_path, executor):
    source = source_run(tmp_path)
    attempt = source / "jobs/job-0/attempts/0001"
    output = tmp_path / "execution"
    assert cli.main(["run", "--attempt", str(attempt), "--output", str(output)]) == 0
    assert cli.main(["verify", "--run", str(output)]) == 0
    (output / "evidence.txt").write_text("damaged")
    assert cli.main(["verify", "--run", str(output)]) != 0


def test_batch_resume_verify_and_summarize_are_consistent(tmp_path, executor):
    calls, _, _ = executor
    source = source_run(tmp_path, jobs=2)
    output = tmp_path / "execution"
    assert cli.main(["batch", "--source-run", str(source), "--output", str(output), "--methods", "NTO_REFINED"]) == 0
    assert len(calls) == 2
    assert cli.main(["verify", "--run", str(output)]) == 0
    assert cli.main(["summarize", "--run", str(output)]) == 0
    assert read(output / "summary.json")["completed"] == 2
    with (output / "records.csv").open() as handle:
        records = list(csv.DictReader(handle))
    assert len(records) == 2
    assert {row["problem_id"] for row in records} == {"case-0", "case-1"}
    assert cli.main(["resume", "--run", str(output)]) == 0
    assert len(calls) == 2


@pytest.mark.parametrize("command", ["verify", "summarize", "resume"])
def test_corrupted_batch_artifact_cannot_enter_statistics_or_resume(tmp_path, executor, command):
    calls, _, _ = executor
    source = source_run(tmp_path)
    output = tmp_path / "execution"
    cli.batch(source, output, ["NTO_REFINED"])
    (output / "jobs/job-0/evidence.txt").write_text("damaged")
    assert cli.main([command, "--run", str(output)]) != 0
    assert len(calls) == 1


@pytest.mark.parametrize("kind", ["pending", "unmatched_method"])
def test_invalid_batch_request_retains_failure_evidence(tmp_path, executor, kind):
    calls, _, _ = executor
    source = source_run(tmp_path, pending=kind == "pending")
    output = tmp_path / "execution"
    method = "NTO_REFINED" if kind == "pending" else "NO_SUCH_METHOD"
    assert cli.main(["batch", "--source-run", str(source), "--output", str(output), "--methods", method]) != 0
    assert not calls
    assert (output / "failure.log").is_file()
    assert (output / "failure.log").read_text()


def test_first_terminal_failure_is_not_replaced_by_later_success(tmp_path, executor):
    calls, _, _ = executor
    source = source_run(tmp_path, pending=True)
    job = read(source / "run_manifest.json")["jobs"][0]
    first = source_attempt(source, job, "0001", status="solver_failed")
    source_attempt(source, job, "0002")
    output = tmp_path / "execution"
    cli.batch(source, output, ["NTO_REFINED"])
    assert calls == [str(first)]
    assert read(output / "jobs/job-0/result.json")["status"] == "no_validated_reference"


def test_retry_pending_attempt_is_not_a_terminal_result(tmp_path, executor):
    calls, _, _ = executor
    source = source_run(tmp_path, pending=True)
    job = read(source / "run_manifest.json")["jobs"][0]
    source_attempt(source, job, "0001", status="infrastructure_error", retry_pending=True)
    terminal = source_attempt(source, job, "0002")
    cli.batch(source, tmp_path / "execution", ["NTO_REFINED"])
    assert calls == [str(terminal)]


def test_completed_resume_allows_new_source_without_executing_again(tmp_path, executor):
    calls, identity, _ = executor
    source = source_run(tmp_path)
    output = tmp_path / "execution"
    cli.batch(source, output, ["NTO_REFINED"])
    identity["simulator.py"] = "new-code"
    assert cli.main(["resume", "--run", str(output)]) == 0
    assert len(calls) == 1


def test_partial_resume_refuses_changed_runtime_before_new_calls(tmp_path, executor, monkeypatch):
    calls, identity, run = executor
    source = source_run(tmp_path, jobs=2)
    output = tmp_path / "execution"

    def interrupted(attempt, destination, *args):
        if Path(destination).name == "job-1":
            raise KeyboardInterrupt("deliberate interruption before execution")
        return run(attempt, destination, *args)

    monkeypatch.setattr(cli, "run", interrupted)
    with pytest.raises(KeyboardInterrupt):
        cli.batch(source, output, ["NTO_REFINED"])
    assert len(calls) == 1
    identity["simulator.py"] = "new-code"
    assert cli.main(["resume", "--run", str(output)]) != 0
    assert len(calls) == 1


def test_partial_resume_completes_only_missing_jobs(tmp_path, executor, monkeypatch):
    calls, _, run = executor
    source = source_run(tmp_path, jobs=2)
    output = tmp_path / "execution"

    def interrupted(attempt, destination, *args):
        if Path(destination).name == "job-1":
            raise KeyboardInterrupt("deliberate interruption before execution")
        return run(attempt, destination, *args)

    monkeypatch.setattr(cli, "run", interrupted)
    with pytest.raises(KeyboardInterrupt):
        cli.batch(source, output, ["NTO_REFINED"])
    monkeypatch.setattr(cli, "run", run)
    assert cli.main(["resume", "--run", str(output)]) == 0
    assert len(calls) == 2
    assert len(set(calls)) == 2


def test_changed_source_receipt_rejected_before_resuming_pending_job(tmp_path, executor, monkeypatch):
    calls, _, run = executor
    source = source_run(tmp_path, jobs=2)
    output = tmp_path / "execution"

    def interrupted(attempt, destination, *args):
        if Path(destination).name == "job-1":
            raise KeyboardInterrupt("deliberate interruption before execution")
        return run(attempt, destination, *args)

    monkeypatch.setattr(cli, "run", interrupted)
    with pytest.raises(KeyboardInterrupt):
        cli.batch(source, output, ["NTO_REFINED"])
    receipt = source / "jobs/job-1/attempts/0001/receipt.json"
    receipt.write_text(receipt.read_text() + "\n")
    monkeypatch.setattr(cli, "run", run)
    assert cli.main(["resume", "--run", str(output)]) != 0
    assert len(calls) == 1


def test_changed_result_seal_is_rejected(tmp_path, executor):
    source = source_run(tmp_path)
    output = tmp_path / "execution"
    cli.batch(source, output, ["NTO_REFINED"])
    result = output / "jobs/job-0/result.json"
    changed = read(result)
    changed["status"] = "execution_failed"
    write(result, changed)
    assert cli.main(["verify", "--run", str(output)]) != 0


def test_duplicate_source_jobs_cannot_inflate_denominator(tmp_path, executor):
    calls, _, _ = executor
    source = source_run(tmp_path)
    manifest = read(source / "run_manifest.json")
    manifest["jobs"].append(dict(manifest["jobs"][0]))
    write(source / "run_manifest.json", manifest)
    output = tmp_path / "execution"
    assert cli.main(["batch", "--source-run", str(source), "--output", str(output), "--methods", "NTO_REFINED"]) != 0
    assert not calls
    assert (output / "failure.log").is_file()


def test_retry_pending_only_does_not_become_completed_input(tmp_path, executor):
    calls, _, _ = executor
    source = source_run(tmp_path, pending=True)
    job = read(source / "run_manifest.json")["jobs"][0]
    source_attempt(source, job, "0001", status="infrastructure_error", retry_pending=True)
    output = tmp_path / "execution"
    assert cli.main(["batch", "--source-run", str(source), "--output", str(output), "--methods", "NTO_REFINED"]) != 0
    assert not calls
    assert (output / "failure.log").is_file()


def test_verify_and_summary_report_partial_coverage_without_fabricating_success(tmp_path, executor, monkeypatch):
    calls, _, run = executor
    source = source_run(tmp_path, jobs=2)
    output = tmp_path / "execution"

    def interrupted(attempt, destination, *args):
        if Path(destination).name == "job-1":
            raise KeyboardInterrupt("deliberate interruption before execution")
        return run(attempt, destination, *args)

    monkeypatch.setattr(cli, "run", interrupted)
    with pytest.raises(KeyboardInterrupt):
        cli.batch(source, output, ["NTO_REFINED"])
    verification = cli.verify(output)
    assert verification["integrity_passed"]
    assert not verification["complete"]
    assert verification["completed"] == 1
    summary = cli.summarize(output)
    assert summary["planned"] == 2 and summary["completed"] == 1
    assert summary["statuses"] == {"execution_success_numerical": 1, "pending": 1}
    assert len(calls) == 1


@pytest.mark.parametrize("fault", ["missing_input_binding", "missing_result_binding", "status_mismatch"])
def test_source_attempt_requires_bound_input_result_and_matching_status(tmp_path, fault):
    source = source_run(tmp_path)
    attempt = source / "jobs/job-0/attempts/0001"
    receipt = read(attempt / "receipt.json")
    if fault == "status_mismatch":
        receipt["status"] = "solver_failed"
    else:
        receipt["files"].pop("input.json" if fault == "missing_input_binding" else "result.json")
    write(attempt / "receipt.json", receipt)
    with pytest.raises(ValueError):
        execution.checked_attempt(attempt)


@pytest.mark.parametrize("command", ["render", "view"])
@pytest.mark.parametrize("fault", [None, "damaged_artifact", "missing_receipt"])
def test_visualization_checks_sealed_evidence_before_loading_states(tmp_path, executor, monkeypatch, command, fault):
    from benchmarks.generic_motion_v1.simulation import render
    source = source_run(tmp_path)
    attempt = source / "jobs/job-0/attempts/0001"
    output = tmp_path / "execution"
    assert cli.main(["run", "--attempt", str(attempt), "--output", str(output)]) == 0
    calls = []

    def display(*args, **kwargs):
        calls.append((args, kwargs))
        return dict(status="displayed_fixture")

    monkeypatch.setattr(render, "render_rollout", display)
    monkeypatch.setattr(render, "play_rollout", display)
    if fault == "damaged_artifact":
        (output / "evidence.txt").write_text("damaged")
    elif fault == "missing_receipt":
        (output / "receipt.json").unlink()
    arguments = [command, "--run", str(output), "--collision-debug"]
    if command == "render":
        arguments += ["--output", str(tmp_path / "render")]
    status = cli.main(arguments)
    if fault:
        assert status != 0
        assert not calls, "Damaged or unsealed evidence reached the visualization loader"
    else:
        assert status == 0
        assert len(calls) == 1
        args, kwargs = calls[0]
        assert args[:2] == (output / "model/model.xml", output / "rollout.npz")
        assert kwargs == dict(camera="overview", collision_debug=True)
