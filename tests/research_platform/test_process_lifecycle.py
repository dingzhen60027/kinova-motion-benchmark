"""Real CPU subprocess lifecycle tests; only short-lived Python sleep processes."""
import os
from pathlib import Path
import signal
import sys
import time

import pytest

from research_platform import runner, storage


def active_group(pgid):
    found=[]
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields=path.read_text().rsplit(')',1)[1].split()
            if int(fields[2])==pgid and fields[0] not in {'Z','X'}:
                found.append(int(path.parent.name))
        except (FileNotFoundError,ProcessLookupError,PermissionError):
            pass
    return found


def cleanup(pgid):
    try:os.killpg(pgid,signal.SIGKILL)
    except ProcessLookupError:pass
    deadline=time.monotonic()+1
    while active_group(pgid) and time.monotonic()<deadline:time.sleep(.01)


def child_command(ignore_term=False,parent_exit=False):
    child=('import os,signal,time; '
           + ('signal.signal(signal.SIGTERM,signal.SIG_IGN); ' if ignore_term else '')
           + 'print("grandchild-ready",os.getpid(),flush=True); time.sleep(30)')
    parent=('import os,subprocess,sys,time; '
            'print("parent-ready",os.getpid(),flush=True); '
            f'subprocess.Popen([sys.executable,"-c",{child!r}]); '
            + ('time.sleep(.06)' if parent_exit else 'time.sleep(30)'))
    return [sys.executable,'-c',parent]


@pytest.mark.skipif(sys.platform!='linux',reason='Linux process-group lifecycle contract')
def test_real_point15_second_budget_stops_sleeping_process_tree_and_seals_logs(tmp_path):
    out=tmp_path/'0001';out.mkdir()
    execution=runner.execute_process(child_command(),out,.15)
    pgid=storage.read(out/'process.json')['pid']
    try:
        assert execution['outcome']=='timeout'
        assert execution['exit_code']!=0
        assert .15<=execution['wall_seconds']<3
        text=(out/'stdout.log').read_text()
        assert 'parent-ready' in text and 'grandchild-ready' in text
        deadline=time.monotonic()+.5
        while active_group(pgid) and time.monotonic()<deadline:time.sleep(.01)
        assert not active_group(pgid)
        receipt=storage.seal_attempt(out,'timeout',**execution)
        assert receipt['status']=='timeout'
        assert storage.verify_attempt(out)==receipt
        assert receipt['files']['stdout.log']==storage.sha(out/'stdout.log')
    finally:cleanup(pgid)


@pytest.mark.skipif(sys.platform!='linux',reason='Linux process-group lifecycle contract')
def test_sigterm_ignoring_grandchild_cannot_survive_timeout(tmp_path):
    out=tmp_path/'0001';out.mkdir()
    execution=runner.execute_process(child_command(ignore_term=True),out,.15)
    pgid=storage.read(out/'process.json')['pid']
    try:
        assert execution['outcome']=='timeout'
        assert 'grandchild-ready' in (out/'stdout.log').read_text()
        deadline=time.monotonic()+.5
        while active_group(pgid) and time.monotonic()<deadline:time.sleep(.01)
        assert not active_group(pgid),'The process-group leader exited but its SIGTERM-ignoring child survived'
    finally:cleanup(pgid)


@pytest.mark.skipif(sys.platform!='linux',reason='Linux process-group lifecycle contract')
def test_normal_parent_exit_cannot_leave_descendant_running_after_job_return(tmp_path):
    out=tmp_path/'0001';out.mkdir()
    execution=runner.execute_process(child_command(parent_exit=True),out,.15)
    pgid=storage.read(out/'process.json')['pid']
    try:
        deadline=time.monotonic()+.5
        while active_group(pgid) and time.monotonic()<deadline:time.sleep(.01)
        assert not active_group(pgid),'A completed process leader left an unbounded descendant'
    finally:cleanup(pgid)


@pytest.mark.skipif(sys.platform!='linux',reason='Linux process-group lifecycle contract')
def test_process_metadata_write_failure_still_terminates_launched_group(tmp_path,monkeypatch):
    out=tmp_path/'0001';out.mkdir()
    launched=[];real_popen=runner.subprocess.Popen
    def track(*args,**kwargs):
        process=real_popen(*args,**kwargs);launched.append(process);return process
    def fail_metadata(*args,**kwargs):
        raise OSError('injected metadata write failure')
    monkeypatch.setattr(runner.subprocess,'Popen',track)
    monkeypatch.setattr(runner,'atomic',fail_metadata)
    try:
        with pytest.raises(OSError,match='injected metadata'):
            runner.execute_process(child_command(),out,.15)
        assert len(launched)==1
        deadline=time.monotonic()+.5
        while active_group(launched[0].pid) and time.monotonic()<deadline:time.sleep(.01)
        assert not active_group(launched[0].pid),'A failure writing process metadata leaked the started worker'
    finally:
        for process in launched:
            cleanup(process.pid)
            process.wait(timeout=2)
