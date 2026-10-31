"""CPU tests for paired source-region execution; no Model/CUDA query runs."""
from pathlib import Path
import json
import shutil
from collections import Counter
import numpy as np
import pytest
from benchmarks.generic_motion_v1.common import read,write,sha,BASE
from benchmarks.generic_motion_v1.roadmap import load_config
from benchmarks.generic_motion_v1.runners import shared_initialization as shared
from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.tests.evidence_fixtures import save_numeric_output


def seal(attempt):
    write(attempt/'receipt.json',dict(status=read(attempt/'result.json')['status'],files={str(p.relative_to(attempt)):sha(p) for p in attempt.rglob('*') if p.is_file() and p.name!='receipt.json'}))


@pytest.fixture
def source(tmp_path):
    cfg=load_config();robot=read(BASE/'configs/robot.json')
    case=dict(problem_id='dev-source',scene_id='dev-scene',family='bottleneck',split='dev',validation_status='valid_input',
        q_start=[0.]*7,q_goal=[1.,0.,0.,0.,0.,0.,0.],voxels_xyz=[],voxel_radius_m=.04,
        robot_sha256=robot['robot_sha256'],collision_geometry_sha256=robot['collision_geometry_sha256'],
        velocity_limits_rad_s=[1.]*7,acceleration_limits_rad_s2=[1.]*7,endpoint_velocity_rad_s=[0.]*7,
        paired_seeds=[dict(drm_seed=0,method_seed=3,solver_seed=4)])
    run=tmp_path/'source';attempt=run/'jobs/source/attempts/0001';attempt.mkdir(parents=True)
    job=dict(job_id='source',case_id=case['problem_id'],scene_id=case['scene_id'],family=case['family'],method='EI_ZO',objective='minimum_distance',method_seed=3,replica_seed=0,roadmap_id='map0',evaluation_config=cfg,case=case)
    manifest=dict(jobs=[job],timeout_seconds=60,split='dev')
    write(run/'run_manifest.json',manifest);write(attempt/'request.json',job);write(attempt/'input.json',dict(case,_objective='minimum_distance'));write(attempt/'evaluation.json',cfg)
    write(attempt/'result.json',dict(status='solver_failed',roadmap_sha256=__import__('hashlib').sha256(b'map').hexdigest(),timing={'eizo_inflation_seconds':.2}))
    write(attempt/'process.json',dict(cold_e2e_seconds=59.,timed_out=False,exit_code=0))
    q=np.array([case['q_start'],case['q_goal']]);np.save(attempt/'path.npy',q)
    frontend=attempt/'frontend_validation';frontend.mkdir()
    save_numeric_output(frontend,case,cfg,kind='polygon',solver_success=True)
    region=attempt/'rounds/000';region.mkdir(parents=True)
    np.savez_compressed(region/'regions.npz',A_0000=np.vstack([np.eye(7),-np.eye(7)]),b_0000=np.full(14,3.))
    write(region/'corridor.json',dict(protocol='consecutive_active_corridor_v1',complete=True,regions=[dict(original_seed_edge_index=0,covered_edge_indices=[0])]))
    write(attempt/'eizo_parameters.json',dict(cfg['eizo'],seed=3))
    seal(attempt)
    return run,manifest,job,attempt,case,cfg


def binding(source):
    run,m,j,a,c,cfg=source
    value,reason=shared.bind_source(run,m,j,a,c,cfg)
    assert reason is None
    return value


@pytest.mark.parametrize('raw',['solver_failed','timeout','success_validated_numerical'])
def test_source_final_optimizer_outcome_never_selects_eligible(source,raw):
    run,m,j,a,c,cfg=source
    original=read(a/'result.json');original['status']=raw;write(a/'result.json',original);seal(a)
    b=binding(source)
    data=shared.check_binding(b,c,cfg,3,'map0')
    assert np.array_equal(data['path'],np.load(a/'path.npy'))


@pytest.mark.parametrize('field,value',[('case_id','wrong'),('method_seed',4),('robot_sha256','wrong'),('evaluation_config_sha256','wrong'),('roadmap_id','wrong'),('replica_seed',8),('roadmap_native_sha256','a'*64),('source_attempt_cold_seconds',.1),('source_initialization_recorded_stage_seconds',0.),('source_initialization_recorded_stages',{'eizo_inflation_seconds':0.})])
def test_wrong_bound_identity_rejected_even_with_recomputed_digest(source,field,value):
    *_,c,cfg=source;b=binding(source);b[field]=value
    b['initialization_digest']=shared.digest({k:v for k,v in b.items() if k!='initialization_digest'})
    with pytest.raises(ValueError):shared.check_binding(b,c,cfg,3,'map0')


@pytest.mark.parametrize('file',['path.npy','rounds/000/regions.npz','rounds/000/corridor.json','receipt.json','input.json','evaluation.json'])
def test_postprepare_tampering_rejected(source,file):
    run,m,j,a,c,cfg=source;b=binding(source)
    (a/file).write_bytes(b'tampered')
    with pytest.raises((ValueError,TypeError)):shared.check_binding(b,c,cfg,3,'map0')


def test_missing_region_is_explicit_unavailable_not_optimizer_failure(source):
    run,m,j,a,c,cfg=source
    (a/'rounds/000/regions.npz').unlink();seal(a)
    b,why=shared.bind_source(run,m,j,a,c,cfg)
    assert b is None and why=='source_initial_regions_unavailable'


def test_source_missing_initial_region_does_not_substitute_final_region(source):
    run,m,j,a,c,cfg=source
    shutil.move(a/'rounds/000/regions.npz',a/'regions.npz');seal(a)
    assert shared.bind_source(run,m,j,a,c,cfg)[1]=='source_initial_regions_unavailable'


def test_source_failed_frontend_is_not_eligible(source):
    run,m,j,a,c,cfg=source
    v=read(a/'frontend_validation/validation.json');v['passed']=False;write(a/'frontend_validation/validation.json',v);seal(a)
    assert shared.bind_source(run,m,j,a,c,cfg)[1]=='source_frontend_not_validated'


def test_source_samples_must_reproduce_path(source):
    run,m,j,a,c,cfg=source
    q=np.load(a/'path.npy');q[1,0]=.8;np.save(a/'path.npy',q);seal(a)
    with pytest.raises(ValueError,match='reproduce'):binding(source)


def test_loader_preserves_coefficients_and_path_bytes(source,tmp_path):
    run,m,j,a,c,cfg=source;b=binding(source)
    out=tmp_path/'output';out.mkdir()
    q,regions,mapping=shared.load_bound(b,c,cfg,3,'map0',out)
    assert (out/'path.npy').read_bytes()==(a/'path.npy').read_bytes()
    assert (out/'shared_initialization/regions.npz').read_bytes()==(a/'rounds/000/regions.npz').read_bytes()
    assert np.array_equal(regions[0].A(),np.vstack([np.eye(7),-np.eye(7)]))
    assert mapping==[dict(original_seed_edge_index=0,covered_edge_indices=[0])]


def test_prepare_cohort_ignores_source_final_success_and_keeps_all_coverage(source,monkeypatch):
    run,m,j,a,c,cfg=source
    monkeypatch.setattr(batch,'verify',lambda *a,**k:dict(integrity_passed=True,complete=True))
    invalid=dict(c,problem_id='invalid',validation_status='generation_failed')
    bindings,coverage=shared.prepare_index(run,[c,invalid],[(0,'map0')],cfg)
    assert len(bindings)==1 and coverage['planned_paired_jobs']==2
    assert [(r['eligible'],r.get('reason')) for r in coverage['coverage']]==[(True,None),(False,'invalid_input')]


def test_incomplete_source_plan_rejected(source,monkeypatch):
    run,m,j,a,c,cfg=source
    monkeypatch.setattr(batch,'verify',lambda *a,**k:dict(integrity_passed=True,complete=False))
    with pytest.raises(ValueError,match='complete'):shared.prepare_index(run,[c],[(0,'map0')],cfg)


def test_prepare_requires_complete_paired_backend_cohort(tmp_path):
    with pytest.raises(ValueError,match='both EI_ZO/LGCS'):
        batch.prepare(tmp_path/'missing',[],tmp_path/'out',objectives=['minimum_distance'],methods=['LGCS'],mode=shared.MODE,initial_source_run=tmp_path/'source')


def test_fixed_mode_load_failure_remains_in_predeclared_denominator(tmp_path,monkeypatch):
    from benchmarks.generic_motion_v1.tests.test_results import make_run,make_attempt
    from benchmarks.generic_motion_v1.runners import results
    run,manifest,job=make_run(tmp_path)
    job.update(execution_mode=shared.MODE,initialization_digest='fixed',shared_initialization=dict(initialization_digest='fixed',files={}))
    other=dict(job,job_id='lgcs',method='LGCS')
    manifest['jobs']=[job,other];write(run/'run_manifest.json',manifest)
    for j in manifest['jobs']:
        attempt=make_attempt(run,j,status='infrastructure_error')
        record=read(attempt/'result.json');record.update(execution_mode=shared.MODE,initialization_digest='fixed',failed_stage='shared_initialization_load',error='simulated source loading failure',frontend_success=False)
        write(attempt/'result.json',record);seal(attempt)
    monkeypatch.setattr(batch,'verify',lambda *a,**k:dict(integrity_passed=True))
    rows,_=results.collect([run])
    spec=dict(id='fixed',mode=shared.MODE,runs=[str(run)])
    selected,planned=results._selected_rows(rows,spec)
    assert len(selected)==len(planned)==2
    assert all(r['status']=='infrastructure_error' for r in selected)
    summaries=results.summarize(rows,dict(comparisons=[spec]))
    counts=[r for r in summaries if r['row_type']=='status' and r['category']=='all']
    assert len(counts)==2
    assert all(r['legal_jobs']==r['finished_legal_jobs']==r['infrastructure_error']==1 for r in counts)


def test_fixed_mode_cannot_drop_one_backend_from_comparison(tmp_path,monkeypatch):
    from benchmarks.generic_motion_v1.tests.test_results import make_run,make_attempt
    from benchmarks.generic_motion_v1.runners import results
    run,m,j=make_run(tmp_path);j.update(execution_mode=shared.MODE,initialization_digest='fixed',shared_initialization={});write(run/'run_manifest.json',m)
    make_attempt(run,j,status='timeout')
    monkeypatch.setattr(batch,'verify',lambda *a,**k:dict(integrity_passed=True))
    rows,_=results.collect([run])
    with pytest.raises(ValueError,match='both predeclared'):
        results._selected_rows(rows,dict(id='fixed',mode=shared.MODE,runs=[str(run)]))


@pytest.mark.parametrize('scenario',['success','repair','load_failure','load_timeout','wrong_roadmap'])
def test_run_case_fixed_lscs_skips_first_generation_but_uses_common_validation(source,tmp_path,monkeypatch,scenario):
    import sys,types
    from benchmarks.generic_motion_v1.runners import csdecomp_baseline as runner
    from benchmarks.generic_motion_v1.tests.test_baseline_validation import Model as NumericModel
    import benchmarks.generic_motion_v1.model as model_module
    from pydrake.all import HPolyhedron
    run,m,j,a,c,cfg=source;b=binding(source)
    class Model(NumericModel):
        def __init__(self,*args):
            self.robot_sha256=c['robot_sha256'];self.collision_geometry_sha256=c['collision_geometry_sha256']
            self.voxels_xyz=np.empty((0,3));self.voxels=None;self.voxel_radius=.04
            self.mplant=None;self.robot_geometry_ids=[]
    class Domain:
        def MakeBox(self,*args):pass
    class Inflator:
        def __init__(self,*args):pass
        def inflateEdge(self,*args):pytest.fail('First inflation must not execute in shared mode')
    fake=types.SimpleNamespace(HPolyhedron=Domain,EizoOptions=lambda:types.SimpleNamespace(),CudaEdgeInflator=Inflator)
    class NoGPUMemoryMonitor:
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def report(self):return {'test_double':True}
    monkeypatch.setattr(runner,'MemoryMonitor',NoGPUMemoryMonitor)
    edits=[]
    if scenario=='repair':
        original_validate=runner.validate
        def one_collision(*args,**kwargs):
            v=original_validate(*args,**kwargs)
            if str(kwargs['out']).endswith('/rounds/000'):
                v.update(passed=False,status='collision_failed',collision_points=np.array([[.5,0,0,0,0,0,0]]),collision_regions=np.array([0]))
            return v
        monkeypatch.setattr(runner,'validate',one_collision)
        monkeypatch.setattr(runner,'native_region_with_anchors',lambda region,*a:region)
        fake.EditRegionsOptions=lambda:types.SimpleNamespace()
        def editor(*args):
            edits.append(args);return args[4],(np.zeros((7,1)),np.zeros((7,1)))
        fake.EditRegionsCuda=editor
    elif scenario in ('load_failure','load_timeout'):
        def fail_load(*args):raise (TimeoutError('controlled load deadline') if scenario=='load_timeout' else ValueError('controlled source loading failure'))
        monkeypatch.setattr(shared,'load_bound',fail_load)
    elif scenario=='wrong_roadmap':
        b['roadmap_native_sha256']='0'*64
        b['initialization_digest']=shared.digest({k:v for k,v in b.items() if k!='initialization_digest'})
    monkeypatch.setitem(sys.modules,'csdecomp',fake);monkeypatch.setattr(model_module,'Model',Model)
    monkeypatch.setattr(runner,'dependency_audit',lambda *a,**k:None)
    monkeypatch.setattr(runner,'plan_query',lambda *a,**k:pytest.fail('DRM query must not rerun'))
    monkeypatch.setattr(runner,'build_active_corridor',lambda *a,**k:pytest.fail('First corridor must not regenerate'))
    # Keep actual author LSCS CPU solve and the common numeric validator with a
    # CPU-only geometry oracle; no native collision or GPU function is present.
    road=tmp_path/'road';road.mkdir();(road/'native.map').write_bytes(b'map')
    write(road/'metadata.json',dict(status='ready',native_map_sha256=sha(road/'native.map'),
        robot_sha256=c['robot_sha256'],collision_geometry_sha256=c['collision_geometry_sha256'],
        config=cfg['roadmap'],roadmap_id='map0',offline_seconds=1.))
    out=tmp_path/'worker'
    result=runner.run_case(c,road,out,config=cfg,seed=3,method='EI_ZO',shared_initialization=b)
    assert result['execution_mode']==shared.MODE and result['budget_seconds']==60
    if scenario in ('load_failure','load_timeout','wrong_roadmap'):
        assert result['status']==('timeout' if scenario=='load_timeout' else 'infrastructure_error'),result
        assert not result['frontend_success'] and not (out/'candidate.npz').exists()
        return
    assert result['status']=='success_validated_numerical',result
    if scenario=='repair':
        assert len(edits)==1 and result['metrics']['recovery_attempts']==1
        assert (out/'coverage/recovery_000/native_edit_inputs.npz').is_file()
    assert 'shared_initialization_load_seconds' in result['timing']
    assert 'drm_path_seconds' not in result['timing']
    assert (out/'shared_initialization/regions.npz').read_bytes()==(a/'rounds/000/regions.npz').read_bytes()
    with np.load(out/'rounds/000/regions.npz') as new,np.load(a/'rounds/000/regions.npz') as old:
        assert all(np.array_equal(new[k],old[k]) for k in old.files)


@pytest.mark.parametrize('repair',[False,True])
def test_lgcs_initial_hook_keeps_alternate_route_and_common_validation(source,tmp_path,monkeypatch,repair):
    import sys,types,time
    from benchmarks.generic_motion_v1.runners import paper_graph, paper_lgcs, csdecomp_baseline as runner
    from benchmarks.generic_motion_v1.tests.test_baseline_validation import Model
    from pydrake.all import HPolyhedron
    run,m,j,a,c,cfg=source;b=binding(source);out=tmp_path/'graph';out.mkdir()
    q,regions,mapping=shared.load_bound(b,c,cfg,3,'map0',out)
    calls=[]
    fake=types.SimpleNamespace();monkeypatch.setitem(sys.modules,'csdecomp',fake)
    if repair:
        original_validate=paper_graph.validate
        def one_collision(*args,**kwargs):
            v=original_validate(*args,**kwargs)
            if str(kwargs['out']).endswith('/rounds/000'):
                v.update(passed=False,status='collision_failed',collision_points=np.array([[.5,0,0,0,0,0,0]]),collision_regions=np.array([0]))
            return v
        monkeypatch.setattr(paper_graph,'validate',one_collision)
        monkeypatch.setattr(paper_graph,'native_region_with_anchors',lambda region,*a:region)
        fake.EditRegionsOptions=lambda:types.SimpleNamespace()
        def editor(*args):calls.append('native_repair');return args[4],(np.zeros((7,1)),np.zeros((7,1)))
        fake.EditRegionsCuda=editor
    monkeypatch.setattr(runner,'build_active_corridor',lambda *a,**k:pytest.fail('Do not regenerate shared first corridor'))
    def alternate(*args,**kwargs):calls.append('alternate');return None,dict(status='no_alternative')
    def solve(pool,start,goal,**kwargs):
        calls.append('solve');assert kwargs['time_limit']<=60
        return dict(kind='polygon',points=np.array([start,goal]),objective='minimum_distance',solver_success=True,segment_regions=[0])
    monkeypatch.setattr(paper_lgcs,'second_route',alternate);monkeypatch.setattr(paper_lgcs,'solve',solve)
    result=dict(timing={},metrics={})
    model=Model();model.mplant=None;model.robot_geometry_ids=[];model.voxels=None;model.voxel_radius=.04
    candidate,validation,pool=paper_graph.run_graph(model,c,cfg,out,q,np.array([c['q_start'],c['q_goal']]),tmp_path,3,
        lambda *a:pytest.fail('Covered initial path does not need extra inflation'),lambda phase:calls.append(phase),result,time.perf_counter()+60,initial_corridor=(regions,mapping))
    assert validation['passed'] and 'alternate' in calls and 'solve' in calls and 'validation' in calls
    if repair:
        assert calls.count('native_repair')==1 and calls.count('solve')==2
        assert (out/'recovery/000/native_edit_inputs.npz').is_file()
    with np.load(out/'first_corridor/regions.npz') as new,np.load(a/'rounds/000/regions.npz') as old:
        assert all(np.array_equal(new[k],old[k]) for k in old.files)


def test_source_cost_scope_keeps_stage_sum_and_full_original_attempt_separate(source):
    b=binding(source)
    assert b['source_initialization_recorded_stage_seconds']==.2
    assert b['source_attempt_cold_seconds']==59
    assert 'not a separate exact frontend wall clock' in b['source_initialization_timing_scope']


def test_source_explicit_eizo_config_mismatch_is_not_eligible(source):
    run,m,j,a,c,cfg=source
    params=read(a/'eizo_parameters.json');key=next(iter(cfg['eizo']));params[key]='wrong'
    write(a/'eizo_parameters.json',params);seal(a)
    with pytest.raises(ValueError,match='parameters'):binding(source)


def test_check_bindings_caches_only_inside_one_operation_and_rechecks_later(source,monkeypatch):
    run,m,j,a,c,cfg=source;b=binding(source);original=shared.read;reads=[]
    def counted(p):
        if Path(p)==run/'run_manifest.json':reads.append(str(p))
        return original(p)
    monkeypatch.setattr(shared,'read',counted)
    entries=[(b,c,cfg,3,'map0')]*3
    shared.check_bindings(entries)
    assert len(reads)==1
    (a/'path.npy').write_bytes(b'mutated after the operation')
    with pytest.raises(ValueError):shared.check_bindings(entries)
    assert len(reads)==2


def test_real_prepare_emits_two_compact_jobs_and_all_eligibility_coverage(source,tmp_path,monkeypatch):
    run,m,j,a,c,cfg=source
    # Source integrity is exercised separately. This test isolates the prepare
    # contract and proves there is no optimizer-success selector or case copy.
    monkeypatch.setattr(batch,'verify',lambda *a,**k:dict(integrity_passed=True,complete=True))
    monkeypatch.setattr(batch,'code_inventory',lambda:{})
    invalid=dict(c,problem_id='invalid',validation_status='generation_failed')
    manifest=tmp_path/'cases.jsonl';manifest.write_text('\n'.join(json.dumps(x) for x in [c,invalid])+'\n')
    road=tmp_path/'roadmap';write(road/'metadata.json',dict(seed=0,status='ready',roadmap_id='map0'))
    output=batch.prepare(manifest,[road],tmp_path/'fixed_run',methods=['EI_ZO','LGCS'],objectives=['minimum_distance'],config=cfg,mode=shared.MODE,initial_source_run=run)
    plan=read(output/'run_manifest.json');coverage=read(output/'initialization_coverage.json')
    assert plan['execution_mode']==shared.MODE and len(plan['jobs'])==2
    assert coverage['registered_task_replicas']==2 and coverage['eligible_task_replicas']==1
    assert {j['method'] for j in plan['jobs']}=={'EI_ZO','LGCS'}
    assert all('case' not in job and 'case_ref' in job for job in plan['jobs'])
    assert len({job['initialization_digest'] for job in plan['jobs']})==1
    assert all(batch.resolve_case(job)==c for job in plan['jobs'])


@pytest.mark.parametrize('mode',['end_to_end','fixed_frontend'])
def test_conditioned_jobs_cannot_be_mixed_into_fullflow_modes(tmp_path,monkeypatch,mode):
    from benchmarks.generic_motion_v1.tests.test_results import make_run,make_attempt
    from benchmarks.generic_motion_v1.runners import results
    run,m,j=make_run(tmp_path);j.update(execution_mode=shared.MODE,initialization_digest='fixed',shared_initialization={});write(run/'run_manifest.json',m)
    make_attempt(run,j,status='timeout');monkeypatch.setattr(batch,'verify',lambda *a,**k:dict(integrity_passed=True))
    rows,_=results.collect([run])
    with pytest.raises(ValueError,match='cannot be mixed'):
        results._selected_rows(rows,dict(id='bad',mode=mode,runs=[str(run)]))


def test_cli_forwards_explicit_mode_source_and_full_cohort(tmp_path,monkeypatch,capsys):
    from benchmarks.generic_motion_v1 import cli
    monkeypatch.setattr(cli,'require_environment',lambda:None)
    seen=[]
    monkeypatch.setattr(batch,'prepare',lambda *a,**kw:(seen.append((a,kw)) or tmp_path/'output'))
    cli.evaluate_main(['batch','--split','test','--methods','EI_ZO','LGCS','--track','path','--seeds','0','--mode',shared.MODE,'--initial-source-run',str(tmp_path/'source'),'--output',str(tmp_path/'output'),'--prepare-only'])
    assert len(seen)==1
    args,kw=seen[0]
    assert kw['mode']==shared.MODE and kw['initial_source_run']==tmp_path/'source'
    assert kw['methods']==['EI_ZO','LGCS'] and kw['objectives']==['minimum_distance']
    assert kw['problem'] is None and kw['timeout']==60 and kw['allow_test'] is True
    assert json.loads(capsys.readouterr().out)['prepared'] is True


def test_first_terminal_source_is_never_replaced_by_later_better_initialization(source,monkeypatch):
    run,m,j,a,c,cfg=source
    later=a.with_name('0002');shutil.copytree(a,later)
    (a/'rounds/000/regions.npz').unlink();seal(a)
    monkeypatch.setattr(batch,'verify',lambda *a,**k:dict(integrity_passed=True,complete=True))
    bindings,coverage=shared.prepare_index(run,[c],[(0,'map0')],cfg)
    assert bindings=={} and coverage['coverage'][0]['reason']=='source_initial_regions_unavailable'
    b,why=shared.bind_source(run,m,j,later,c,cfg)
    assert why is None
    with pytest.raises(ValueError,match='first terminal'):
        shared.check_binding(b,c,cfg,3,'map0')


def test_conditioned_job_uses_existing_real_process_deadline(source,tmp_path,monkeypatch):
    import subprocess,sys
    run,m,j,a,c,cfg=source;b=binding(source)
    job=dict(j,execution_mode=shared.MODE,initialization_digest=b['initialization_digest'],shared_initialization=b)
    newrun=tmp_path/'newrun';write(newrun/'run_manifest.json',dict(jobs=[job],source_hashes={}))
    monkeypatch.setattr(batch,'code_inventory',lambda:{})
    monkeypatch.setattr(batch,'_gpu_memory',lambda pid:None)
    original=subprocess.Popen
    monkeypatch.setattr(batch.subprocess,'Popen',lambda argv,**kw:original([sys.executable,'-c','import time; time.sleep(30)'],**kw))
    result=batch.execute(job,newrun,.15)
    attempt=newrun/'jobs/source/attempts/0001'
    assert result['status']=='timeout' and result['execution_mode']==shared.MODE
    assert result['initialization_digest']==b['initialization_digest']
    assert read(attempt/'process.json')['timed_out'] is True
    assert read(attempt/'receipt.json')['status']=='timeout'


@pytest.mark.parametrize('unsealed',['result.json','process.json'])
def test_source_cannot_add_unsealed_result_or_process_to_new_binding(source,unsealed):
    run,m,j,a,c,cfg=source
    receipt=read(a/'receipt.json');del receipt['files'][unsealed];write(a/'receipt.json',receipt)
    assert (a/unsealed).is_file()
    with pytest.raises(ValueError,match='sealed metadata/process'):
        binding(source)


@pytest.mark.parametrize('mutation',['drop_mode','job_digest_wrong','objective_time','coverage_digest_wrong','coverage_count_wrong'])
def test_planverify_checks_declaration_consistency(source,tmp_path,monkeypatch,mutation):
    run,m,j,a,c,cfg=source
    real_verify=batch.verify
    monkeypatch.setattr(batch,'verify',lambda *a,**k:dict(integrity_passed=True,complete=True))
    monkeypatch.setattr(batch,'code_inventory',lambda:{})
    manifest=tmp_path/'cases.jsonl';manifest.write_text(json.dumps(c)+'\n')
    road=tmp_path/'road';write(road/'metadata.json',dict(seed=0,status='ready',roadmap_id='map0'))
    out=batch.prepare(manifest,[road],tmp_path/'planned',methods=['EI_ZO','LGCS'],objectives=['minimum_distance'],config=cfg,mode=shared.MODE,initial_source_run=run)
    plan=read(out/'run_manifest.json');coverage=read(out/'initialization_coverage.json')
    if mutation=='drop_mode':
        for item in plan['jobs']:item.pop('execution_mode')
    elif mutation=='job_digest_wrong':
        for item in plan['jobs']:item['initialization_digest']='unrelated'
    elif mutation=='objective_time':
        for item in plan['jobs']:item['objective']='minimum_time'
    elif mutation=='coverage_digest_wrong':coverage['coverage'][0]['initialization_digest']='other'
    elif mutation=='coverage_count_wrong':coverage['eligible_task_replicas']=99;coverage['planned_paired_jobs']=198
    write(out/'initialization_coverage.json',coverage)
    plan['initialization_coverage_sha256']=sha(out/'initialization_coverage.json')
    write(out/'run_manifest.json',plan);write(out/'identity.json',dict(run_manifest_sha256=sha(out/'run_manifest.json')))
    result=real_verify(out)
    assert result['integrity_passed'] is False, (mutation,result)


def test_finite_coefficients_with_nonfinite_anchor_residual_rejected(source):
    run,m,j,a,c,cfg=source
    q=np.load(a/'path.npy');q[1,:2]=2.
    c['q_goal']=q[1].tolist();j['case']=c
    write(a/'request.json',j);write(a/'input.json',dict(c,_objective='minimum_distance'))
    write(run/'run_manifest.json',m);np.save(a/'path.npy',q)
    save_numeric_output(a/'frontend_validation',c,cfg,kind='polygon',solver_success=True)
    A=np.array([[1e308,-1e308,0.,0.,0.,0.,0.]]);b=np.array([1e308])
    np.savez_compressed(a/'rounds/000/regions.npz',A_0000=A,b_0000=b);seal(a)
    with np.errstate(over='ignore',invalid='ignore'):
        assert not np.isfinite(A@q.T-b[:,None]).all()
        with pytest.raises((ValueError,TypeError)):binding(source)

@pytest.mark.parametrize('tail',[
    ['--track','time','--methods','EI_ZO','LGCS','--initial-source-run','/tmp/missing-fixed-source'],
    ['--track','path','--methods','EI_ZO','--initial-source-run','/tmp/missing-fixed-source'],
    ['--track','path','--methods','EI_ZO','LGCS'],
    ['--track','path','--methods','EI_ZO','LGCS','--problem','test-specific','--initial-source-run','/tmp/missing-fixed-source'],
])
def test_public_cli_rejects_invalid_fixed_modes_without_planning(tmp_path,monkeypatch,tail):
    from benchmarks.generic_motion_v1 import cli
    monkeypatch.setattr(cli,'require_environment',lambda:None)
    monkeypatch.setattr(batch,'resume',lambda *a,**k:pytest.fail('No execution permitted in malformed mode'))
    with pytest.raises(ValueError):
        cli.evaluate_main(['batch','--mode',shared.MODE,'--split','test','--output',str(tmp_path/'newrun'),'--prepare-only']+tail)
    assert not (tmp_path/'newrun').exists()
