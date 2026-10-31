"""Actual Linux signal delivery at the coordinator/worker process boundary."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from research_platform import runner, storage


def alive_group(pgid):
    members = []
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == pgid and fields[0] not in {'Z', 'X'}:
                members.append(int(path.parent.name))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            pass
    return members


@pytest.mark.parametrize('outcome', ['normal', 'timeout', 'metadata_error'])
def test_original_sigterm_handler_is_restored_on_every_exit(tmp_path, monkeypatch, outcome):
    def sentinel(signum, frame):
        raise AssertionError('Original handler should not be called by this test')

    previous = signal.signal(signal.SIGTERM, sentinel)
    command = [sys.executable, '-c', 'import time; time.sleep(30)' if outcome == 'timeout' else 'pass']
    try:
        if outcome == 'metadata_error':
            def fail(*args, **kwargs):
                raise OSError('injected metadata failure')
            monkeypatch.setattr(runner, 'atomic', fail)
            with pytest.raises(OSError, match='injected metadata failure'):
                runner.execute_process(command, tmp_path, .15)
        else:
            result = runner.execute_process(command, tmp_path, .15)
            assert result['outcome'] == ('timeout' if outcome == 'timeout' else 'finished')
        assert signal.getsignal(signal.SIGTERM) is sentinel
    finally:
        signal.signal(signal.SIGTERM, previous)


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux process groups and parent-death signal')
def test_real_sigterm_cleans_entire_group_and_restores_handler(tmp_path):
    out = tmp_path / 'worker'
    out.mkdir()
    # Match the production worker's parent-death behavior. The grandchild must
    # still be stopped even though it ignores TERM and the worker leader exits.
    child = ('import os,signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); '
             'print("grandchild-ready",os.getpid(),flush=True); time.sleep(30)')
    worker = ('import subprocess,sys,time; '
              'from research_platform.runner import parent_death_signal; parent_death_signal(); '
              f'subprocess.Popen([sys.executable,"-c",{child!r}]); time.sleep(30)')
    command = [sys.executable, '-c', worker]
    coordinator = f'''import json,signal
from pathlib import Path
from research_platform.runner import execute_process
def previous_handler(signum,frame): pass
signal.signal(signal.SIGTERM,previous_handler)
try:
    execute_process({command!r},{str(out)!r},30)
except KeyboardInterrupt as exc:
    Path({str(tmp_path / 'interrupted.json')!r}).write_text(json.dumps(dict(
        reason=str(exc),handler_restored=signal.getsignal(signal.SIGTERM) is previous_handler)))
'''
    process = None
    pgid = None
    try:
        with (tmp_path / 'coordinator_stdout.log').open('x') as stdout, \
                (tmp_path / 'coordinator_stderr.log').open('x') as stderr:
            process = subprocess.Popen([sys.executable, '-c', coordinator], cwd=storage.ROOT,
                                       stdout=stdout, stderr=stderr, start_new_session=True)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                if (out / 'stdout.log').exists() and 'grandchild-ready' in (out / 'stdout.log').read_text():
                    break
                assert process.poll() is None, 'Coordinator exited before worker became ready'
                time.sleep(.02)
            assert 'grandchild-ready' in (out / 'stdout.log').read_text()
            pgid = storage.read(out / 'process.json')['pid']
            os.kill(process.pid, signal.SIGTERM)
            assert process.wait(timeout=5) == 0
        record = json.loads((tmp_path / 'interrupted.json').read_text())
        assert record == dict(reason='Received signal SIGTERM', handler_restored=True)
        deadline = time.monotonic() + .5
        while alive_group(pgid) and time.monotonic() < deadline:
            time.sleep(.01)
        assert not alive_group(pgid), 'SIGTERM left an unbounded worker descendant'
        assert (out / 'stdout.log').read_text().startswith('grandchild-ready')
    finally:
        if pgid is None and (out / 'process.json').exists():
            pgid = storage.read(out / 'process.json')['pid']
        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
        if pgid is not None:
            deadline = time.monotonic() + 1
            while alive_group(pgid) and time.monotonic() < deadline:
                time.sleep(.01)
