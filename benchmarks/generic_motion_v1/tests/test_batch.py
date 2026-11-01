import json
import fcntl
from pathlib import Path
import subprocess
import sys

import pytest
from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.common import read, write


def prepared(tmp_path, monkeypatch, split="smoke"):
    # These fixtures launch only mocked workers; never contend with a real
    # experiment's GPU lock when the CPU regression suite runs concurrently.
    monkeypatch.setattr(batch, "BASE", tmp_path)
    monkeypatch.setattr(batch, "code_inventory", lambda: {})
    manifest = tmp_path / "cases.jsonl"
    case = dict(problem_id="c0", split=split, scene_id="s0", family="sparse_clutter",
                paired_seeds=[dict(drm_seed=0, method_seed=81237, solver_seed=9129)])
    manifest.write_text(json.dumps(case) + "\n")
    roadmap = tmp_path / "roadmap"
    write(roadmap / "metadata.json", dict(seed=0, status="ready"))
    run = batch.prepare(manifest, [roadmap], tmp_path / "run", objectives=["minimum_distance"],
                        methods=["EI_ZO"])
    return run, manifest, roadmap


def test_declared_independent_method_seed_and_denominator(tmp_path, monkeypatch):
    run, _, _ = prepared(tmp_path, monkeypatch)
    manifest = read(run / "run_manifest.json")
    assert manifest["jobs"][0]["method_seed"] == 81237
    assert manifest["jobs"][0]["solver_seed"] == 9129
    check = batch.verify(run)
    assert check["integrity_passed"] and not check["complete"]
    assert check["planned_jobs"] == 1 and check["completed_jobs"] == 0


def test_test_set_cannot_execute(tmp_path, monkeypatch):
    with pytest.raises(ValueError, match="locked"):
        prepared(tmp_path, monkeypatch, split="test")


def test_complete_paper_suite_creates_five_valid_method_objective_pairs(tmp_path, monkeypatch):
    _, manifest, roadmap = prepared(tmp_path, monkeypatch)
    run = batch.prepare(manifest, [roadmap], tmp_path / 'paper_run',
        objectives=['minimum_distance', 'minimum_time'], methods=['EI_ZO', 'NTO_PAPER', 'LGCS'])
    jobs = read(run / 'run_manifest.json')['jobs']
    assert {(j['method'], j['objective']) for j in jobs} == {
        ('EI_ZO', 'minimum_distance'), ('EI_ZO', 'minimum_time'),
        ('NTO_PAPER', 'minimum_distance'), ('NTO_PAPER', 'minimum_time'),
        ('LGCS', 'minimum_distance')}
    assert len(jobs) == 5
    assert len({json.dumps(batch.resolve_case(j), sort_keys=True) for j in jobs}) == 1
    assert len({j['method_seed'] for j in jobs}) == 1
    with pytest.raises(ValueError, match='nonempty'):
        batch.prepare(manifest, [roadmap], tmp_path / 'invalid_paper_run',
            objectives=['minimum_time'], methods=['LGCS'])


def test_resume_rejects_mutated_manifest(tmp_path, monkeypatch):
    run, manifest, _ = prepared(tmp_path, monkeypatch)
    manifest.write_text(manifest.read_text() + "\n")
    with pytest.raises(ValueError, match="manifest changed"):
        batch.resume(run)


def test_resume_rejects_mutated_map(tmp_path, monkeypatch):
    run, _, roadmap = prepared(tmp_path, monkeypatch)
    write(roadmap / "metadata.json", dict(seed=99))
    with pytest.raises(ValueError, match="roadmap changed"):
        batch.resume(run)


def test_real_process_timeout_is_saved_and_resume_never_retries(tmp_path, monkeypatch):
    run, _, _ = prepared(tmp_path, monkeypatch)
    original = subprocess.Popen
    monkeypatch.setattr(batch.subprocess, "Popen", lambda argv, **kwargs:
                        original([sys.executable, "-c", "import time; print('started', flush=True); time.sleep(30)"], **kwargs))
    monkeypatch.setattr(batch, "_gpu_memory", lambda pid: None)
    job = read(run / "run_manifest.json")["jobs"][0]
    result = batch.execute(job, run, .15)
    assert result["status"] == "timeout"
    attempt = run / "jobs" / job["job_id"] / "attempts" / "0001"
    assert read(attempt / "process.json")["timed_out"]
    before = {str(p): p.read_bytes() for p in attempt.iterdir() if p.is_file()}
    monkeypatch.setattr(batch, "execute", lambda *a, **k: pytest.fail("Do not retry completed failure"))
    assert batch.resume(run)["complete"]
    assert before == {str(p): p.read_bytes() for p in attempt.iterdir() if p.is_file()}
    (attempt / "stdout.log").write_text("corrupted")
    assert not batch.verify(run)["integrity_passed"]


def test_unsealed_interruption_is_preserved_and_retried_in_new_attempt(tmp_path, monkeypatch):
    run, _, _ = prepared(tmp_path, monkeypatch)
    job = read(run / "run_manifest.json")["jobs"][0]
    first = run / "jobs" / job["job_id"] / "attempts" / "0001"
    first.mkdir(parents=True)
    (first / "partial.txt").write_text("actual partial output must survive")
    write(first / "result.json", dict(status="running", stage="planning"))
    assert not batch.verify(run)["integrity_passed"]
    original = subprocess.Popen
    monkeypatch.setattr(batch.subprocess, "Popen", lambda argv, **kwargs:
                        original([sys.executable, "-c", "print('second attempt process')"], **kwargs))
    monkeypatch.setattr(batch, "_gpu_memory", lambda pid: None)
    result = batch.resume(run)
    assert result["complete"] and result["integrity_passed"]
    assert (first / "partial.txt").read_text() == "actual partial output must survive"
    assert read(first / "result.json")["error_category"] == "coordinator_interrupted"
    assert read(first / "result_before_interruption.json")["stage"] == "planning"
    assert (first.parent / "0002" / "receipt.json").exists()


def test_run_manifest_tampering_rejected(tmp_path, monkeypatch):
    run, _, _ = prepared(tmp_path, monkeypatch)
    manifest = read(run / "run_manifest.json")
    manifest["jobs"][0]["case_ref"]["record_sha256"] = "tampered"
    write(run / "run_manifest.json", manifest)
    assert not batch.verify(run)["integrity_passed"]


def test_active_coordinator_lock_rejects_resume_without_mutating_attempt(tmp_path, monkeypatch):
    run, _, _ = prepared(tmp_path, monkeypatch)
    job = read(run / "run_manifest.json")["jobs"][0]
    attempt = run / "jobs" / job["job_id"] / "attempts" / "0001"
    write(attempt / "result.json", dict(status="running", stage="planning"))
    (attempt / "partial.txt").write_text("live worker output")
    before = {str(p): p.read_bytes() for p in attempt.iterdir()}
    monkeypatch.setattr(batch, "execute", lambda *a, **k: pytest.fail("Do not launch another worker"))
    with (run / ".resume.lock").open("a+") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(RuntimeError, match="Another coordinator"):
            batch.resume(run, max_jobs=0)
        assert before == {str(p): p.read_bytes() for p in attempt.iterdir()}
    # Once the owner releases the lock, normal interruption recovery can
    # proceed. max_jobs=0 seals evidence without launching a GPU worker.
    check = batch.resume(run, max_jobs=0)
    assert check["integrity_passed"] and not check["complete"]
    assert read(attempt / "receipt.json")["retry_pending"]
    assert (attempt / "partial.txt").read_text() == "live worker output"
    assert (run / ".resume.lock").exists()
