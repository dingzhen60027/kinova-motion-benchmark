"""A later interrupted attempt cannot reopen a completed benchmark query."""
import json

import pytest

from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.common import read, sha, write
from benchmarks.generic_motion_v1.tests.test_batch import prepared


@pytest.fixture
def lifecycle(tmp_path, monkeypatch):
    run, _, _ = prepared(tmp_path, monkeypatch)
    job = read(run / 'run_manifest.json')['jobs'][0]
    monkeypatch.setattr(batch, 'execute', lambda *a, **k: pytest.fail('Completed jobs must never execute again'))
    return run, job


def attempt(run, job, number, *, pending=False, sealed=True):
    directory = run / 'jobs' / job['job_id'] / 'attempts' / f'{number:04d}'
    if not sealed:
        write(directory / 'result.json', dict(status='running', stage='planning'))
        (directory / 'partial.txt').write_bytes(b'unique interrupted evidence\n')
        return directory
    result = dict(status='infrastructure_error' if pending else 'solver_failed', success=False)
    if not pending:
        result.update(batch._result_identity(job))
        write(directory / 'request.json', job)
        write(directory / 'process.json', dict(exit_code=0, timed_out=False))
    write(directory / 'result.json', result)
    files = {str(p.relative_to(directory)): sha(p) for p in directory.rglob('*') if p.is_file()}
    receipt = dict(status=result['status'], files=files)
    if pending:
        receipt['retry_pending'] = True
    write(directory / 'receipt.json', receipt)
    return directory


def contents(directory):
    return {str(p.relative_to(directory)): p.read_bytes() for p in directory.rglob('*') if p.is_file()}


def changed_current_source(run, tmp_path, monkeypatch):
    """Keep a valid historical snapshot while today's source differs."""
    relative = 'lifecycle_fixture_runtime.py'
    snapshot = run / 'source_snapshot' / relative
    snapshot.parent.mkdir(parents=True, exist_ok=True)
    snapshot.write_bytes(b'original runtime\n')
    manifest = read(run / 'run_manifest.json')
    manifest['source_hashes'] = {relative: sha(snapshot)}
    write(run / 'run_manifest.json', manifest)
    write(run / 'identity.json', dict(run_manifest_sha256=sha(run / 'run_manifest.json')))
    monkeypatch.setattr(batch, 'ROOT', tmp_path)
    (tmp_path / relative).write_bytes(b'changed current runtime\n')
    monkeypatch.setattr(batch, 'code_inventory', lambda: {relative: sha(tmp_path / relative)})


def test_terminal_failure_then_retry_pending_stays_complete(lifecycle, tmp_path, monkeypatch):
    run, job = lifecycle
    first = attempt(run, job, 1)
    attempt(run, job, 2, pending=True)
    before = contents(run / 'jobs')
    changed_current_source(run, tmp_path, monkeypatch)
    assert batch._first_terminal_attempt(first.parent.iterdir()) == first
    check = batch.verify(run)
    assert check['integrity_passed'] and check['complete'] and check['completed_jobs'] == 1
    assert batch.resume(run)['complete']
    assert contents(run / 'jobs') == before


def test_terminal_then_unsealed_is_only_preserved_and_sealed(lifecycle, tmp_path, monkeypatch):
    run, job = lifecycle
    first = attempt(run, job, 1)
    later = attempt(run, job, 2, sealed=False)
    original = contents(later)
    first_before = contents(first)
    changed_current_source(run, tmp_path, monkeypatch)
    permissive = batch.verify(run, allow_unsealed=True)
    assert permissive['complete'] and permissive['integrity_passed']
    assert not batch.verify(run)['integrity_passed']
    result = batch.resume(run)
    assert result['complete'] and result['integrity_passed']
    assert not result['unsealed_attempts']
    assert contents(first) == first_before
    assert (later / 'partial.txt').read_bytes() == original['partial.txt']
    assert json.loads((later / 'result_before_interruption.json').read_text()) == json.loads(original['result.json'])
    assert read(later / 'receipt.json')['retry_pending'] is True
    assert batch._first_terminal_attempt(first.parent.iterdir()) == first
    after = contents(run / 'jobs')
    assert batch.resume(run)['complete']
    assert contents(run / 'jobs') == after


def test_pending_terminal_pending_selects_first_terminal(lifecycle):
    run, job = lifecycle
    attempt(run, job, 1, pending=True)
    terminal = attempt(run, job, 2)
    attempt(run, job, 3, pending=True)
    before = contents(run / 'jobs')
    assert batch._first_terminal_attempt(terminal.parent.iterdir()) == terminal
    assert batch.verify(run)['complete']
    assert batch.resume(run)['complete']
    assert contents(run / 'jobs') == before


@pytest.mark.parametrize('payload', [
    'not JSON', '[]', 'null', '42',
    json.dumps(dict(status='success_made_up', files={})),
    json.dumps(dict(status='solver_failed', retry_pending='false', files={})),
    json.dumps(dict(status='solver_failed', retry_pending=0, files={})),
    json.dumps(dict(status='solver_failed', retry_pending=[], files={})),
])
def test_malformed_later_receipt_fails_closed_even_after_terminal(lifecycle, payload):
    run, job = lifecycle
    first = attempt(run, job, 1)
    later = attempt(run, job, 2, pending=True)
    (later / 'receipt.json').write_text(payload)
    before = contents(run / 'jobs')
    with pytest.raises((ValueError, TypeError, KeyError)):
        batch._first_terminal_attempt(first.parent.iterdir())
    check = batch.verify(run, allow_unsealed=True)
    assert not check['integrity_passed'] and not check['complete']
    with pytest.raises(ValueError):
        batch.resume(run)
    assert contents(run / 'jobs') == before


def test_unfinished_job_rejects_changed_source_before_sealing(lifecycle, tmp_path, monkeypatch):
    run, job = lifecycle
    interrupted = attempt(run, job, 1, sealed=False)
    before = contents(interrupted)
    changed_current_source(run, tmp_path, monkeypatch)
    with pytest.raises(ValueError, match='current source changed'):
        batch.resume(run)
    assert contents(interrupted) == before
    assert not (interrupted / 'receipt.json').exists()


def test_original_interruption_still_resumes_in_new_attempt(lifecycle, monkeypatch):
    run, job = lifecycle
    interrupted = attempt(run, job, 1, sealed=False)
    calls = []

    def execute(frozen_job, frozen_run, timeout):
        assert frozen_job == job and frozen_run == run
        calls.append((frozen_job['job_id'], timeout))
        assert read(interrupted / 'receipt.json')['retry_pending'] is True
        attempt(run, job, 2)

    monkeypatch.setattr(batch, 'execute', execute)
    check = batch.resume(run)
    assert check['complete'] and check['integrity_passed']
    assert len(calls) == 1
    assert (interrupted / 'partial.txt').read_bytes() == b'unique interrupted evidence\n'
    assert read(interrupted / 'result_before_interruption.json')['status'] == 'running'
    assert batch._first_terminal_attempt(interrupted.parent.iterdir()).name == '0002'
