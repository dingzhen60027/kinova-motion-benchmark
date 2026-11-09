"""Formal coordinator regressions; GPU/solver execution is covered by smoke-check."""
import csv
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import run_baselines as formal
from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.common import read, write
from benchmarks.generic_motion_v1.roadmap import load_config
from benchmarks.generic_motion_v1.runners.results import _figure_comparisons


def minimal_experiment(tmp_path, monkeypatch):
    base=tmp_path/'inputs'
    for split in formal.SPLITS:
        data=[dict(problem_id=split+'-'+state,scene_id=split+'-scene',family='bottleneck',split=split,
                   validation_status=state,paired_seeds=[dict(drm_seed=s,method_seed=100+s,solver_seed=200+s)
                                                         for s in formal.SEEDS])
              for state in ('valid_input','generation_failed')]
        manifest=base/'manifests'/f'{split}.jsonl';manifest.parent.mkdir(parents=True,exist_ok=True)
        manifest.write_text(''.join(json.dumps(c)+'\n' for c in data))
    for seed in formal.SEEDS:
        write(base/f'roadmaps/rss2025_n6000_s{seed:03d}/metadata.json',dict(seed=seed,status='ready'))
    monkeypatch.setattr(formal,'BASE',base)
    monkeypatch.setattr(formal,'code_inventory',lambda:{})
    monkeypatch.setattr(batch,'code_inventory',lambda:{})
    monkeypatch.setattr(formal,'relative',lambda p:str(Path(p).resolve()))
    output=tmp_path/'experiment';output.mkdir()
    write(output/'preflight/environment.json',{})
    monkeypatch.setattr(formal,'subprocess',SimpleNamespace(run=lambda *a,**k:None,STDOUT=-2))
    return output,formal.ROOT/'configs/evaluation.json'


def test_real_batch_matrix_has_all_pairs_seeds_and_only_legal_inputs(tmp_path,monkeypatch):
    output,config=minimal_experiment(tmp_path,monkeypatch)
    exp=formal.prepare(output,config)
    rows=list(csv.DictReader((output/'matrix.csv').open()))
    assert exp['planned_legal_jobs']==3*3*14==len(rows)
    assert exp['reused_jobs']==0 and exp['new_jobs']==126
    assert len({(r['problem_id'],r['replica_seed'],r['method'],r['objective']) for r in rows})==126
    assert {r['problem_id'] for r in rows}=={s+'-valid_input' for s in formal.SPLITS}
    assert sum(r['objective']=='minimum_time' for r in rows)==36
    assert len({Path(s['name']).name for s in exp['stages']})==9  # Export link names must not collide.
    assert _figure_comparisons(read(output/'selection.json'))=={'baselines_test'}
    for stage in exp['stages']:
        result=batch.verify(output/stage['name'])
        assert result['integrity_passed'] and not result['complete']
        manifest=read(output/stage['name']/'run_manifest.json')
        assert manifest['registered_inputs']==2 and manifest['excluded_generation_failures']==1


def test_interrupted_preparation_reuses_unexecuted_manifests(tmp_path,monkeypatch):
    output,config=minimal_experiment(tmp_path,monkeypatch)
    original=batch.prepare
    calls=[]
    def interrupted(*a,**kw):
        calls.append(a[2])
        if len(calls)==3:raise RuntimeError('interrupted preparation')
        return original(*a,**kw)
    monkeypatch.setattr(batch,'prepare',interrupted)
    with pytest.raises(RuntimeError,match='interrupted preparation'):formal.prepare(output,config)
    first=output/'smoke-seed-0/run_manifest.json';before=first.read_bytes()
    monkeypatch.setattr(batch,'prepare',original)
    exp=formal.prepare(output,config)
    assert exp['planned_legal_jobs']==126 and first.read_bytes()==before


def test_no_historical_reuse_without_hardware_thread_evidence(tmp_path,monkeypatch):
    cfg=load_config(formal.ROOT/'configs/evaluation.json')
    write(tmp_path/'run_manifest.json',dict(source_hashes={},timeout_seconds=60,source_code_commit='historical',
          jobs=[dict(evaluation_config=cfg)]))
    monkeypatch.setattr(batch,'verify',lambda *a,**k:pytest.fail('Missing provenance must reject before importing results'))
    audit,reuse=formal.reusable_time_run(tmp_path,cfg,{}, {})
    assert not reuse
    assert 'missing_sealed_runtime_hardware_and_thread_attestation' in audit['reasons']


@pytest.mark.parametrize('state',['timeout','solver_failure','validation_failed','no_solution_within_budget','success_numerical'])
def test_ordinary_planning_outcomes_do_not_stop_pipeline(tmp_path,monkeypatch,state):
    monkeypatch.setattr(batch,'_first_terminal_attempt',lambda _:tmp_path)
    monkeypatch.setattr(formal,'attempt_row',lambda *a,**k:dict(status=state))
    assert formal.finished(tmp_path,{},dict(job_id='job'),{})['status']==state


def test_infrastructure_failure_stops_before_next_job(tmp_path,monkeypatch):
    monkeypatch.setattr(batch,'_first_terminal_attempt',lambda _:tmp_path)
    monkeypatch.setattr(formal,'attempt_row',lambda *a,**k:dict(status='infrastructure_error',failure_reason='CUDA illegal access'))
    with pytest.raises(RuntimeError,match='CUDA illegal access'):
        formal.finished(tmp_path,{},dict(job_id='job'),{})


def test_saved_success_is_fully_verified_and_corruption_stops(tmp_path):
    from benchmarks.generic_motion_v1.tests.test_results import make_run, make_attempt
    run,manifest,job=make_run(tmp_path)
    attempt=make_attempt(run,job)
    assert formal.finished(run,manifest,job,{})['status']=='success_numerical'
    with (attempt/'candidate.npz').open('ab') as stream:stream.write(b'corrupt')
    with pytest.raises(RuntimeError,match='Systemic failure'):
        formal.finished(run,manifest,job,{})


def test_worker_command_log_is_inside_attempt(tmp_path):
    from benchmarks.generic_motion_v1.__main__ import _command_ledger
    attempt=tmp_path/'attempt'
    assert _command_ledger(['module','_worker','--attempt',str(attempt)])==attempt/'commands.jsonl'
    assert _command_ledger(['module','methods'])==formal.BASE/'reports/cli_commands.jsonl'


def test_gpu_guard_allows_desktop_but_rejects_another_planner(monkeypatch):
    monkeypatch.setattr(formal,'subprocess',SimpleNamespace(run=lambda *a,**k:SimpleNamespace(
        stdout='1754, /usr/libexec/gnome-remote-desktop-daemon, 165 MiB\n')))
    assert len(formal.gpu_idle())==1
    monkeypatch.setattr(formal,'subprocess',SimpleNamespace(run=lambda *a,**k:SimpleNamespace(
        stdout='123, python, 900 MiB\n')))
    with pytest.raises(RuntimeError,match='GPU compute workload'):formal.gpu_idle()
    monkeypatch.setattr(formal,'subprocess',SimpleNamespace(run=lambda *a,**k:SimpleNamespace(
        stdout=f'{os.getpid()}, python, 188 MiB\n')))
    assert formal.gpu_idle(allow_current_process=True)
    with pytest.raises(RuntimeError,match='GPU compute workload'):formal.gpu_idle()


def test_lock_is_exclusive_and_released_without_deleting_inode(tmp_path):
    path=tmp_path/'experiment.lock'
    with formal.locked(path):
        with pytest.raises(RuntimeError,match='Another coordinator'):
            with formal.locked(path):pass
    with formal.locked(path):assert path.exists()


def test_dependency_change_between_queries_stops(tmp_path,monkeypatch):
    output,config=minimal_experiment(tmp_path,monkeypatch)
    native=tmp_path/'solver.so';native.write_bytes(b'original')
    write(output/'preflight/environment.json',dict(threads={},runtime_stats={str(native):[native.stat().st_size,native.stat().st_mtime_ns]}))
    formal.prepare(output,config)
    formal.verify_context_fast(output)
    native.write_bytes(b'changed native dependency')
    with pytest.raises(ValueError,match='Runtime dependency changed'):formal.verify_context_fast(output)


def test_start_requires_actual_smoke_evidence(tmp_path,monkeypatch):
    monkeypatch.setattr(formal,'prepare',lambda *a:None)
    monkeypatch.setattr(formal,'verify_context',lambda *a:None)
    monkeypatch.setattr(formal.shutil,'which',lambda _: '/usr/bin/tmux')
    with pytest.raises(RuntimeError,match='Run smoke-check'):
        formal.start(tmp_path,Path('config'),'kinova-baselines')
