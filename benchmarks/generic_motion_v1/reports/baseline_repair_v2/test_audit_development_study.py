"""Synthetic CPU fixtures only; never read partial study outcomes."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest

from benchmarks.generic_motion_v1.reports.baseline_repair_v2 import audit_development_study as a


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def fixture_matrix():
    cfg = dict(drm=dict(max_number_planning_attempts=20,max_iterations_steering_to_node=10,
        max_nodes_to_expand=10000,online_edge_step_size=.1),roadmap={'config':'fixed'},
        endpoint_tolerance_rad=1e-5,validation_step_rad=.005,validation_minimum_samples=400,
        validation_maximum_samples=100000,feasibility_tolerance=1e-5)
    frozen=dict(source_commit=a.SOURCE,runtime_source_hashes={'module.py':'a'*64},declaration={'methods':list(a.METHODS)},
        planned_registered_slots=950,test_planner_calls=0,timeout_seconds=60.,gpu_workers=1,blocks=[],
        evaluation_config=cfg,data_files={},roadmap_files={})
    manifests={};datasets={};roads={};index={'slots':950,'test_planner_calls':0,'blocks':[]}
    for name,(split,seed,count,legal) in a.BLOCKS.items():
        b=dict(name=name,split=split,manifest='/fixture/'+split+'.jsonl',roadmap='/fixture/map'+str(seed),
               run='/fixture/'+name,registered_inputs=count,legal_inputs=legal,planned_jobs=count*5)
        frozen['blocks'].append(b);index['blocks'].append(dict(b,run_manifest_sha256='f'*64))
        frozen['data_files'][b['manifest']]='b'*64
        mapfile=b['roadmap']+'/metadata.json';frozen['roadmap_files'][mapfile]='c'*64
        roadmap=dict(seed=seed,status='ready',roadmap_id='map'+str(seed),robot_sha256='r',collision_geometry_sha256='g',
                     requested_nodes=6000,actual_nodes=6000)
        roads[name]=roadmap
        data=[dict(problem_id=f'{split}-{i}',split=split,scene_id=f'{split}-scene{i//2}',family='open_control',
            validation_status='valid_input' if i<legal else 'invalid_endpoint',q_start=[0.]*7 if i<legal else None,
            q_goal=[1.]*7 if i<legal else None,voxel_radius_m=.04,robot_sha256='r',collision_geometry_sha256='g',
            paired_seeds=[dict(drm_seed=s,method_seed=100+i+s,solver_seed=200+i+s) for s in range(3)]) for i in range(count)]
        datasets[name]=data;jobs=[]
        for case in data:
            p=case['paired_seeds'][seed]
            for method,objective in a.METHODS:
                jobs.append(dict(job_id=case['problem_id']+method+objective,case_id=case['problem_id'],case=deepcopy(case),
                    scene_id=case['scene_id'],family=case['family'],method=method,objective=objective,
                    roadmap_id=roadmap['roadmap_id'],roadmap_dir=b['roadmap'],method_seed=p['method_seed'],
                    solver_seed=p['solver_seed'],solver_seed_used=False,evaluation_config=deepcopy(cfg)))
        manifests[name]=dict(source_code_commit=a.SOURCE,source_hashes=deepcopy(frozen['runtime_source_hashes']),
            split=split,formal_test_run=False,study_protocol=None,selection='all',manifest_path=b['manifest'],
            manifest_sha256='b'*64,timeout_seconds=60,roadmap_files={mapfile:'c'*64},jobs=jobs)
    completed=dict(complete=True,completed_jobs=950,total_jobs=950,blocks=[dict(b,verification=dict(
        complete=True,integrity_passed=True,completed_jobs=b['planned_jobs'])) for b in index['blocks']])
    return frozen,index,completed,manifests,datasets,roads


def test_exact_950_matrix_and_940_legal():
    frozen,index,completed,manifests,data,roads=fixture_matrix()
    blocks=a.preflight(frozen,index,completed)
    records=[a.audit_matrix(frozen,b,manifests[name],data[name],roads[name]) for name,b in blocks.items()]
    assert sum(r['registered'] for r in records)==950
    assert sum(r['legal'] for r in records)==940
    assert sum(r['invalid'] for r in records)==10


@pytest.mark.parametrize('mutation',[
    lambda f,i,c:i['blocks'].pop(),
    lambda f,i,c:i['blocks'].append(deepcopy(i['blocks'][0])),
    lambda f,i,c:c.update(complete=False),
    lambda f,i,c:c.update(completed_jobs=949),
    lambda f,i,c:f.update(source_commit='0'*40),
    lambda f,i,c:f.update(test_planner_calls=1),
    lambda f,i,c:f.update(timeout_seconds=61),
    lambda f,i,c:f.update(gpu_workers=2),
    lambda f,i,c:c['blocks'][0]['verification'].update(complete=False),
])
def test_preflight_refuses_incomplete_or_changed_plan(mutation):
    f,i,c,*_=fixture_matrix();mutation(f,i,c)
    with pytest.raises(ValueError):a.preflight(f,i,c)


@pytest.mark.parametrize('mutation',[
    lambda m,d,r:m['jobs'].pop(),
    lambda m,d,r:m['jobs'].append(deepcopy(m['jobs'][0])),
    lambda m,d,r:m['jobs'][0]['case']['q_goal'].__setitem__(0,2),
    lambda m,d,r:m['jobs'][0].update(method_seed=999),
    lambda m,d,r:m['jobs'][0].update(solver_seed=999),
    lambda m,d,r:m['jobs'][0]['evaluation_config'].update(validation_step_rad=.1),
    lambda m,d,r:m['jobs'][0].update(roadmap_id='other'),
    lambda m,d,r:m['jobs'][0]['case'].update(robot_sha256='changed'),
    lambda m,d,r:m.update(formal_test_run=True),
    lambda m,d,r:m.update(selection='first-per-family'),
    lambda m,d,r:m['source_hashes'].update({'module.py':'changed'}),
    lambda m,d,r:r.update(seed=1),
    lambda m,d,r:r.update(collision_geometry_sha256='changed'),
    lambda m,d,r:r.update(actual_nodes=5999),
])
def test_matrix_rejects_missing_seed_model_endpoint_or_config(mutation):
    f,i,c,ms,ds,rs=fixture_matrix();m,d,r=ms['smoke_s000'],ds['smoke_s000'],rs['smoke_s000']
    mutation(m,d,r)
    with pytest.raises(ValueError):a.audit_matrix(f,i['blocks'][0],m,d,r)


def outcome(tmp_path, *, name='new', method='EI_ZO', path=True, status=a.SUCCESS):
    f,i,c,ms,ds,rs=fixture_matrix();j=deepcopy(ms['smoke_s000']['jobs'][0]);j['method']=method
    out=tmp_path/name;out.mkdir();(out/'stderr.log').write_text('');(out/'stdout.log').write_text('')
    result=dict(status=status,frontend_native_success=path,frontend_success=path)
    return dict(job=j,attempt=out,path=np.asarray([[0.]*7,[1.]*7]) if path else None,result=result,
        legal=True,process={'exit_code':0},query=None)


def test_bitwise_paths_match_while_optimizer_status_can_change(tmp_path):
    old=outcome(tmp_path,name='old',status='solver_failed');new=outcome(tmp_path)
    assert a.compare_frontend(old,new)['passed']
    new['path'][1,0]+=1e-12
    assert not a.compare_frontend(old,new)['passed']


def test_missing_returned_path_fails(tmp_path):
    old=outcome(tmp_path,name='old');new=outcome(tmp_path,path=False)
    assert not a.compare_frontend(old,new)['passed']


@pytest.mark.parametrize('error_exit',['closest','frontier'])
def test_only_reviewed_abort_to_failure_is_allowed(tmp_path,error_exit):
    old=outcome(tmp_path,name='old',path=False,status='infrastructure_error');new=outcome(tmp_path,path=False,status='frontend_failed')
    old['process']['exit_code']=-6
    if error_exit=='closest':(old['attempt']/'stderr.log').write_text('assertion closest_node >= 0 failed')
    else:(old['attempt']/'stdout.log').write_text('Failed to find path in roadmap after verification step.')
    new['query']=dict(status='native_exception',query_success=False)
    result=a.compare_frontend(old,new)
    assert result['passed'] and result['kind']=='reviewed_abort_to_query_failure'
    new['query']['status']='returned';new['path']=np.empty((0,))
    assert a.compare_frontend(old,new)['passed']
    new['path']=np.ones((2,7))
    assert not a.compare_frontend(old,new)['passed']
    new['path']=None
    new['result']['status']=a.SUCCESS
    assert not a.compare_frontend(old,new)['passed']


def test_unknown_abort_not_silently_treated_as_repaired(tmp_path):
    old=outcome(tmp_path,name='old',path=False,status='infrastructure_error');new=outcome(tmp_path,path=False,status='frontend_failed')
    old['process']['exit_code']=-6;new['query']=dict(status='native_exception',query_success=False)
    assert not a.compare_frontend(old,new)['passed']


def test_comparison_refuses_different_query_budget(tmp_path):
    old=outcome(tmp_path,name='old');new=outcome(tmp_path)
    new['job']['evaluation_config']['drm']['max_nodes_to_expand']=10
    with pytest.raises(ValueError,match='config'):a.compare_frontend(old,new)


def test_nto_refined_pairs_with_paper_without_merging_labels(tmp_path):
    old=outcome(tmp_path,name='old',method='NTO_PAPER');new=outcome(tmp_path,method='NTO_REFINED')
    assert a.compare_frontend(old,new)['passed']
    assert a.aggregate([new])[0]['method']=='NTO-refined-distance'


def query_fixture(tmp_path):
    row=outcome(tmp_path);j=row['job']
    build={k:'a'*64 for k in ('binary_sha256','native_library_sha256','patched_source_sha256','upstream_source_sha256')}
    q=dict(build,status='returned',query_success=True,endpoint_change=False,cuda_changed=False,search_budget_changed=False,
        q_start=j['case']['q_start'],q_goal=j['case']['q_goal'],online_voxel_radius_m=.04,
        layout_before=dict(num_joints=7,loaded=True,initialized=True,**{
            k:float(np.float32(v)) for k,v in j['evaluation_config']['drm'].items()}))
    row['query']=q;save(row['attempt']/'build_receipt.json',build)
    return row,build


def test_query_invariants_pass(tmp_path):
    row,build=query_fixture(tmp_path);assert a.check_query(row,build)=='returned'


def test_query_checks_loaded_roadmap_identity(tmp_path):
    row,build=query_fixture(tmp_path);row['query']['layout_before']['nodes']=6000
    row['result']['roadmap_sha256']='expected'
    assert a.check_query(row,build,dict(actual_nodes=6000,native_map_sha256='expected'))=='returned'
    with pytest.raises(ValueError,match='another graph'):
        a.check_query(row,build,dict(actual_nodes=6000,native_map_sha256='other'))


@pytest.mark.parametrize('mutation',[
    lambda r:r['query'].update(binary_sha256='changed'),
    lambda r:r['query'].update(endpoint_change=True),
    lambda r:r['query'].update(q_goal=[2.]*7),
    lambda r:r['query'].update(online_voxel_radius_m=.039),
    lambda r:r['query']['layout_before'].update(max_number_planning_attempts=10),
    lambda r:r['query'].update(query_success=False),
    lambda r:r.update(path=None),
    lambda r:r['query'].update(status='running'),
])
def test_query_changed_invariants_are_rejected(tmp_path,mutation):
    row,build=query_fixture(tmp_path);mutation(row)
    with pytest.raises(ValueError):a.check_query(row,build)


def test_interrupted_query_saved_as_timeout_not_success(tmp_path):
    row,build=query_fixture(tmp_path);row['query']['status']='running';row['path']=None
    row['result'].update(status='timeout',frontend_native_success=False,frontend_success=False)
    assert a.check_query(row,build)=='running'


def test_invalid_denominator_and_controls_separate(tmp_path):
    good=outcome(tmp_path,name='good');invalid=outcome(tmp_path,name='invalid',path=False,status='invalid_endpoint')
    failed=outcome(tmp_path,name='failure',path=False,status='frontend_failed')
    invalid['legal']=False;failed['job']['family']='bottleneck'
    groups=a.aggregate([good,invalid,failed]);allrows=next(g for g in groups if g['family']=='all')
    assert (allrows['registered'],allrows['legal'],allrows['invalid'],allrows['success'],allrows['legal_failures'])==(3,2,1,1,1)
    assert next(g for g in groups if g['family']=='open_control')['registered']==2


def test_failed_early_audit_saved_and_never_overwritten(tmp_path):
    f,i,c,*_=fixture_matrix();c['complete']=False
    paths=[tmp_path/n for n in ('freeze.json','index.json','completed.json')]
    for p,d in zip(paths,(f,i,c)):save(p,d)
    out=tmp_path/'integrity_audit.json'
    result=a.audit(out,freeze_path=paths[0],index_path=paths[1],completed_path=paths[2])
    assert result['passed'] is False and result['complete'] is False and result['errors']
    assert out.exists() and out.with_suffix('.md').exists()
    with pytest.raises(ValueError,match='Preserve'):a.audit(out)


def test_file_inventory_detects_tampering(tmp_path):
    p=tmp_path/'source.py';p.write_text('original');digest=a.sha(p)
    assert a.check_files({str(p):digest})['passed']
    p.write_text('changed')
    with pytest.raises(ValueError,match='Changed'):a.check_files({str(p):digest})
