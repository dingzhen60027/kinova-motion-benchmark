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


def test_repair_reuse_proof_rejects_other_runtime_or_function_changes():
    old={formal.LGCS_SOURCE:'old','validator.py':'same'}
    new=dict(old);new[formal.LGCS_SOURCE]='new'
    before='LIMIT = 1\ndef second_route():\n    return False\ndef solve():\n    return LIMIT\n'
    after=before.replace('return False','return True')
    assert formal.lgcs_change_scope(old,new,before,after)['affected_methods']==['LGCS']
    with pytest.raises(ValueError,match='only the LGCS'):
        formal.lgcs_change_scope(old,dict(new,**{'validator.py':'changed'}),before,after)
    with pytest.raises(ValueError,match='outside LGCS'):
        formal.lgcs_change_scope(old,new,before,after.replace('LIMIT = 1','LIMIT = 2'))


def test_repair_keeps_original_evidence_reruns_all_lgcs_and_resumes_pending(tmp_path,monkeypatch):
    output,config=minimal_experiment(tmp_path,monkeypatch)
    env=dict(python='test',gpu='test',cpu='test',threads={},packages={},runtime_files={},runtime_stats={})
    write(output/'preflight/environment.json',env)
    original=formal.prepare(output,config)
    frozen={p:p.read_bytes() for p in output.rglob('*.json')}
    for index,stage in enumerate(original['stages']):
        directory=output/stage['name'];m=read(directory/'run_manifest.json')
        for method in ('LGCS','RRTCONNECT'):
            job=next(j for j in m['jobs'] if j['method']==method)
            attempt=directory/'jobs'/job['job_id']/'attempts/0001'
            # Ordinary failures are retained too; the LGCS cohort is replaced
            # in full, including successful and failed observations.
            state='timeout' if method=='RRTCONNECT' else 'success_validated_numerical'
            r=dict(status=state)
            if index==0 and method=='LGCS':
                from benchmarks.generic_motion_v1.runners.failure_classification import KNOWN_DRM_NO_ROUTE
                r=dict(status='infrastructure_error',failed_stage='alternate_drm_path',exception=repr(RuntimeError(KNOWN_DRM_NO_ROUTE)))
            write(attempt/'result.json',r);write(attempt/'receipt.json',dict(status=r['status'],files={}))
    old_attempts={p:p.read_bytes() for p in output.glob('*-seed-*/jobs/*/attempts/*/*.json')}
    monkeypatch.setattr(formal,'gpu_idle',lambda:[])
    monkeypatch.setattr(formal,'environment',lambda:env)
    monkeypatch.setattr(formal,'lgcs_change_scope',lambda *a:dict(policy='LGCS-only test proof'))
    monkeypatch.setattr(formal.subprocess,'check_output',lambda args,**kw:
        'old module' if kw.get('text') else (formal.ROOT/args[-1].split(':',1)[1]).read_bytes(),raising=False)
    monkeypatch.setattr(formal,'attempt_row',lambda _r,_m,_j,a,**kw:dict(status=read(a/'result.json')['status']))
    # This orchestration fixture uses small synthetic receipts. Real receipt
    # corruption rejection has its own integration regression above.
    monkeypatch.setattr(batch,'verify',lambda *a,**kw:dict(integrity_passed=True,complete=True,errors=[]))
    repaired=formal.repair_lgcs(output,config)
    assert repaired['reused_jobs']==9 and repaired['new_jobs']==117
    assert len(repaired['repair']['superseded_lgcs_attempts'])==9
    assert len(repaired['stages'])==6
    jobs=[];kept=[]
    for stage in repaired['stages']:
        manifest=read(output/stage['name']/'run_manifest.json')
        jobs.extend(manifest['jobs'])
        if stage['read_only']:
            kept.extend(manifest['jobs'])
            assert all((output/stage['name']/'jobs'/j['job_id']).is_symlink() for j in manifest['jobs'])
        else:
            assert not any((output/stage['name']/'jobs').glob('*'))
    assert len(jobs)==len({j['job_id'] for j in jobs})==126
    assert {j['method'] for j in kept}=={'RRTCONNECT'}
    assert all(p.read_bytes()==b for p,b in {**frozen,**old_attempts}.items())
    assert formal.repair_lgcs(output,config)['stages']==repaired['stages']
    # A tampered repair matrix must stop, never silently fall back to originals.
    (output/'repair_matrix.csv').write_text('corrupt')
    with pytest.raises(ValueError,match='Frozen experiment input changed'):
        formal.verify_context(output)


def test_retained_stage_is_never_executed(tmp_path,monkeypatch):
    monkeypatch.setattr(batch,'resume',lambda *a,**kw:pytest.fail('Read-only cohort must not resume'))
    monkeypatch.setattr(batch,'verify',lambda *a,**kw:dict(integrity_passed=True,complete=True))
    assert formal.execute_stage(tmp_path,dict(name='retained-smoke',read_only=True))['complete']
    monkeypatch.setattr(batch,'verify',lambda *a,**kw:dict(integrity_passed=True,complete=False))
    with pytest.raises(ValueError,match='Retained cohort'):formal.execute_stage(tmp_path,dict(name='retained-smoke',read_only=True))


def test_retained_view_uses_original_worker_manifest_without_weakening_validator(tmp_path,monkeypatch):
    from benchmarks.generic_motion_v1.tests.test_results import make_run,make_attempt
    from benchmarks.generic_motion_v1.runners import results
    run,manifest,job=make_run(tmp_path)
    attempt=make_attempt(run,job)
    view=tmp_path/'retained-smoke';view.mkdir()
    copied=dict(manifest,selection='explicit_lgcs_repair_retained',
                derived_from_manifests={formal.relative(run/'run_manifest.json'):formal.sha(run/'run_manifest.json')})
    write(view/'run_manifest.json',copied)
    (view/'jobs').mkdir();(view/'jobs'/job['job_id']).symlink_to(run/'jobs'/job['job_id'],target_is_directory=True)
    assert formal.finished(view,copied,job,{})['status']=='success_numerical'
    # The unmodified validator must still reject the selection-view manifest.
    assert results.attempt_row(view,copied,job,attempt)['status']=='infrastructure_error'
    # Exercise the actual summary selector and CSV writer using original
    # receipts; the expensive source audit is covered by batch regressions.
    monkeypatch.setattr(batch,'verify',lambda *a,**kw:dict(integrity_passed=True))
    write(tmp_path/'repair.json',{})
    write(tmp_path/'repair_selection.json',dict(comparisons=[dict(id='test',mode='end_to_end',runs=[str(view)])]))
    exp=dict(stages=[dict(name=view.name,split='smoke',read_only=True)],selection_path='repair_selection.json',
             repair=dict(superseded_lgcs_attempts=[]),statistics='paired numerical test')
    result=formal.summarize_repair(tmp_path,exp)
    assert result['actual_attempts']==1
    row=next(csv.DictReader((tmp_path/'analysis/records.csv').open()))
    assert row['status']=='success_numerical' and row['selected_comparisons']=='["test"]'
    with (attempt/'candidate.npz').open('ab') as stream:stream.write(b'corrupt')
    with pytest.raises(RuntimeError,match='Systemic failure'):formal.finished(view,copied,job,{})
    copied['derived_from_manifests']={}
    with pytest.raises(ValueError,match='source manifest binding'):formal.finished(view,copied,job,{})
