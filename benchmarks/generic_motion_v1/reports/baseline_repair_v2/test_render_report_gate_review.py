"""Read-only report-gate review with synthetic metadata; no actual metrics read."""
from copy import deepcopy
from collections import Counter
import json
from pathlib import Path
import numpy as np
import pytest

from benchmarks.generic_motion_v1.reports.baseline_repair_v2 import render_report as renderer


class ReachedHistoricalRead(Exception):
    pass


def install_inputs(monkeypatch, *, audit_name=None, audit_value=None, metrics_changes=None, completed_changes=None):
    values={
        'metrics.json':dict(complete=True,comparable=True,requested_jobs=950),
        'execution_freeze.json':dict(source_commit='synthetic-unrelated-source'),
        'completed_blocks.json':dict(complete=True,completed_jobs=950),
        **{n+'_audit.json':dict(passed=True,errors=[]) for n in ('integrity','lgcs','nto')},
    }
    if audit_name:values[audit_name+'_audit.json']=audit_value
    values['metrics.json'].update(metrics_changes or {})
    values['completed_blocks.json'].update(completed_changes or {})
    def read(path):
        if 'dev_metrics_stage_qualified' in str(path):raise ReachedHistoricalRead()
        value=values[path.name]
        if value is None:raise FileNotFoundError(path)
        return deepcopy(value)
    monkeypatch.setattr(renderer,'read',read)
    monkeypatch.setattr(renderer,'write',lambda *args:pytest.fail('No artifact may be written during gate-only fixture'))


@pytest.mark.parametrize('name',['integrity','lgcs','nto'])
@pytest.mark.parametrize('value',[dict(passed=False),dict(),dict(passed=1),None])
def test_missing_or_nonpassing_one_of_three_audits_rejects(monkeypatch,name,value):
    install_inputs(monkeypatch,audit_name=name,audit_value=value)
    with pytest.raises((ValueError,FileNotFoundError)):
        renderer.main()


@pytest.mark.parametrize('metrics,completed',[
    ({'complete':False},{}),({'comparable':False},{}),({'requested_jobs':949},{}),
    ({},{'complete':False}),({},{'completed_jobs':949}),
])
def test_incomplete_or_incomparable_declared_run_rejects(monkeypatch,metrics,completed):
    install_inputs(monkeypatch,metrics_changes=metrics,completed_changes=completed)
    with pytest.raises(ValueError):renderer.main()


@pytest.mark.parametrize('name',['integrity','lgcs','nto'])
def test_inconsistent_passed_and_errors_rejects(monkeypatch,name):
    install_inputs(monkeypatch,audit_name=name,audit_value=dict(passed=True,complete=False,
        errors=['synthetic audit failure'],index_sha256='wrong-source-index'))
    with pytest.raises(ValueError):renderer.main()


def test_three_unbound_passed_booleans_refused():
    with pytest.raises((ValueError,KeyError)):
        renderer.validate_evidence({}, {}, {}, {}, {n:{'passed':True} for n in ('integrity','lgcs','nto')}, {},'x','y')


def set_fixture_pairs(metrics):
    """Construct point estimates and placeholder intervals, never bootstrap."""
    result=[]
    for objective,a,b in renderer.CURRENT_PAIRS:
        a,b=sorted((a,b))
        maps={label:{renderer.row_key(r)[:3]:r for r in metrics['rows'] if r['split']=='dev' and r['label']==label}
              for label in (a,b)}
        pairs=[(maps[a][key],maps[b][key]) for key in sorted(maps[a])]
        joint=[(x,y) for x,y in pairs if x['success'] and y['success']]
        record=dict(split='dev',roadmap_nodes=6000,scope='obstacle_main',objective=objective,
                    method_a=a,method_b=b,paired_legal_jobs=len(pairs))
        for field,pool,func in [
            ('valid_success_difference',pairs,lambda x,y:int(x['success'])-int(y['success'])),
            ('common_success_cost_difference',joint,lambda x,y:x['cost']-y['cost']),
            ('observed_cold_latency_difference',pairs,lambda x,y:x['cold_e2e_seconds']-y['cold_e2e_seconds'])]:
            n=sum(func(x,y) for x,y in pool);d=len(pool);estimate=n/d if d else None
            clusters=len({x['cluster_id'] for x,y in pool})
            ci=[estimate-.1,estimate+.2] if clusters>=2 else None
            if field=='valid_success_difference' and ci:ci=[max(-1,ci[0]),min(1,ci[1])]
            record[field]=dict(numerator=n,denominator=d,estimate=estimate,ci95=ci,
                independent_scene_clusters=clusters,bootstrap_seed=45827,bootstrap_replicates=2000)
        result.append(record)
    metrics['paired_comparisons']=result


def bound_fixture():
    labels=[x for x,_ in renderer.METHODS];old_labels=dict(renderer.METHODS)
    families=['sparse_clutter','dense_clutter','bottleneck','route_choice','open_control']
    rows=[];blocks=[]
    for split,seed in [('smoke',0),('dev',0),('dev',1),('dev',2)]:
        blocks.append(dict(name=f'{split}_s{seed:03d}',run='/fixture/'+split+str(seed),run_manifest_sha256=f'{seed+1:064x}'))
        for i in range(10 if split=='smoke' else 60):
            fam=families[i//(2 if split=='smoke' else 15)]
            legal=split=='dev' or i<8
            for label in labels:
                rows.append(dict(case_id=f'{split}-{i}',roadmap_id='map'+str(seed),method_seed=100+i+seed,
                    label=label,split=split,family=fam,cluster_id=f'{split}-{fam}-{i%15//3}' if split=='dev' else 'smoke-'+fam,
                    objective='minimum_time' if label in ('SCSTO','NTO-refined-time') else 'minimum_distance',
                    status=renderer.SUCCESS if legal else 'invalid_endpoint',success=legal,input_legal=legal,drm_success=legal,
                    q_start=[0]*7 if legal else None,q_goal=[1]*7 if legal else None,
                    robot_sha256='robot',geometry_sha256='geometry',input_sha256=f'{split}-{i}-hash',
                    budget_seconds=60,cost=2. if legal else None,cold_e2e_seconds=3.,roadmap_nodes=6000,
                    warm_query_seconds=2. if legal and i%5 else None,
                    frontend_component_seconds=.6 if legal and i%3 else None,
                    timing={k:.1 for k in renderer.FRONTEND_TIMERS} if legal and i%3 else {},
                    straight_line_collision=bool(i%2)))
    oldrows=[dict(r,label=old_labels[r['label']]) for r in rows if r['split']=='dev']
    def ratio(pool):
        n=sum(r['success'] for r in pool);d=len(pool)
        return dict(numerator=n,denominator=d,estimate=n/d if d else None,ci95=[n/d,n/d] if d else None)
    def groups(data):
        out=[]
        for split,label in sorted({(r['split'],r['label']) for r in data}):
            p=[r for r in data if r['split']==split and r['label']==label]
            scopes=[('obstacle_main',[r for r in p if r['family']!='open_control']),
                    ('open_control',[r for r in p if r['family']=='open_control'])]
            scopes += [('family:'+f,[r for r in p if r['family']==f]) for f in families[:4]]
            scopes += [('straight_line:'+str(flag),[r for r in p if r['family']!='open_control' and r['straight_line_collision'] is flag]) for flag in (True,False)]
            for scope,subset in scopes:
                if not subset:continue
                legal=[r for r in subset if r['input_legal']]
                stats={}
                for field in ('cold_e2e_seconds','warm_query_seconds'):
                    values=[r[field] for r in legal if r[field] is not None]
                    stats[field]={'all_legal':dict(count=len(values),mean=float(np.mean(values)) if values else None,
                        p50=float(np.quantile(values,.5)) if values else None,p95=float(np.quantile(values,.95)) if values else None)}
                out.append(dict(split=split,label=label,scope=scope,roadmap_nodes=6000,legal_jobs=len(legal),
                    legal_status_counts=dict(Counter(r['status'] for r in legal)),valid_success=ratio(legal),
                    conditional_validated_drm=ratio([r for r in legal if r['drm_success']]),
                    statistics=stats,stage_seconds={}))
        return out
    metrics=dict(complete=True,comparable=True,requested_jobs=950,rows=rows,groups=groups(rows),runs=[dict(
        run=b['run'],manifest_sha256=b['run_manifest_sha256'],source_commit=renderer.SOURCE,
        integrity=dict(complete=True,integrity_passed=True)) for b in blocks])
    set_fixture_pairs(metrics)
    old=dict(complete=True,comparable=True,requested_jobs=900,rows=oldrows,groups=groups(oldrows),runs=[dict(
        run='/fixture/old',manifest_sha256=renderer.OLD_MANIFEST_SHA,source_commit=renderer.OLD_SOURCE)])
    counts=[]
    for split,label in sorted({(r['split'],r['label']) for r in rows}):
        p=[r for r in rows if r['split']==split and r['label']==label]
        counts.append(dict(split=split,method=label,family='all',registered=len(p),success=sum(r['success'] for r in p),
                           status_counts=dict(Counter(r['status'] for r in p))))
    audits=dict(integrity=dict(passed=True,errors=[],complete=True,source_code_commit=renderer.SOURCE,
        registered_slots=950,legal_slots=940,invalid_slots=10,groups=counts,
        run_manifest_sha256={b['run']:b['run_manifest_sha256'] for b in blocks},
        evidence=dict(run_index={'sha256':'indexhash'},freeze={'sha256':'freezehash'},completed={'sha256':'fixturehash'})),
        lgcs=dict(passed=True,errors=[],complete=True,index_sha256='indexhash',freeze_sha256='freezehash',
            new_source_commit=renderer.SOURCE,old_source_commit=renderer.OLD_SOURCE,expected_lgcs_jobs=190,
            paired_dev=dict(n=180,legal_n=180)),
        nto=dict(passed=True,errors=[],index_sha256='indexhash',planned_new_slots=950,new_nto_slots=380,paired_dev_nto_slots=360))
    frozen=dict(source_commit=renderer.SOURCE,environment=dict(python='synthetic Python',gpu='synthetic GPU',
        packages={name:'synthetic-'+name for name in ('drake','csdecomp','scsplanning','clarabel')}))
    completed=dict(complete=True,completed_jobs=950,total_jobs=950)
    index=dict(slots=950,test_planner_calls=0,blocks=blocks)
    return metrics,frozen,completed,index,audits,old,'indexhash','freezehash'


def test_positive_bound_950_fixture_passes():
    assert len(renderer.validate_evidence(*bound_fixture()))==4


@pytest.mark.parametrize('change',[
    lambda x:x[4]['integrity'].update(complete=False),
    lambda x:x[4]['integrity'].update(legal_slots=939),
    lambda x:x[4]['integrity'].update(errors=['bad']),
    lambda x:x[4]['integrity']['evidence']['run_index'].update(sha256='different'),
    lambda x:x[4]['integrity']['evidence']['freeze'].update(sha256='different'),
    lambda x:x[4]['integrity']['run_manifest_sha256'].update({'/fixture/dev2':'different'}),
    lambda x:x[4]['lgcs'].update(expected_lgcs_jobs=189),
    lambda x:x[4]['lgcs'].update(index_sha256='different'),
    lambda x:x[4]['lgcs'].update(freeze_sha256='different'),
    lambda x:x[4]['lgcs'].update(new_source_commit='different'),
    lambda x:x[4]['nto'].update(new_nto_slots=379),
    lambda x:x[4]['nto'].update(index_sha256='different'),
    lambda x:x[0]['runs'][0].update(manifest_sha256='different'),
    lambda x:x[0]['runs'][0].update(source_commit='different'),
    lambda x:x[5].update(comparable=False),
    lambda x:x[5]['runs'][0].update(manifest_sha256='different'),
    lambda x:x[0]['rows'].__setitem__(0,deepcopy(x[0]['rows'][1])),
    lambda x:x[0]['rows'].pop(),
    lambda x:x[0]['groups'].append(deepcopy(x[0]['groups'][0])),
    lambda x:x[0]['groups'][0].update(legal_jobs=999),
    lambda x:x[0]['groups'][0]['valid_success'].update(numerator=999),
    lambda x:x[4]['integrity']['groups'][0].update(success=999),
    lambda x:x[0]['rows'][0].update(status='new_unreported_failure'),
    lambda x:x[0]['rows'][0].update(cost=float('nan')),
    lambda x:x[0]['groups'][0]['statistics']['cold_e2e_seconds']['all_legal'].update(p95=9.),
    lambda x:x[0]['groups'][0]['valid_success'].update(ci95=[-.1,1.1]),
    lambda x:x[0]['groups'][0]['statistics']['warm_query_seconds']['all_legal'].update(count=999),
    lambda x:x[0]['groups'][0]['statistics']['warm_query_seconds']['all_legal'].update(mean=999),
    lambda x:x[0]['groups'][0]['statistics']['cold_e2e_seconds']['all_legal'].update(mean=999),
])
def test_binding_and_denominator_tampering_rejected(change):
    fixture=bound_fixture();change(fixture)
    with pytest.raises((ValueError,KeyError)):renderer.validate_evidence(*fixture)


@pytest.mark.parametrize('existing',['report','comparison','receipt'])
def test_existing_any_output_refused_before_read_or_write(tmp_path,monkeypatch,existing):
    base=tmp_path/'benchmark';out=base/'reports/baseline_repair_v2';out.mkdir(parents=True)
    monkeypatch.setattr(renderer,'BASE',base);monkeypatch.setattr(renderer,'OUT',out)
    paths={'report':base/'reports/benchmark_v1_reliable_baselines.md',
           'comparison':out/'historical_comparison.json','receipt':out/'report_receipt.json'}
    paths[existing].write_text('preserved-evidence')
    monkeypatch.setattr(renderer,'read',lambda *args:pytest.fail('Existing output must reject before reads'))
    with pytest.raises(FileExistsError):renderer.main()
    assert paths[existing].read_text()=='preserved-evidence'
    assert sum(p.exists() for p in paths.values())==1


def test_full_synthetic_render_creates_bound_outputs(tmp_path,monkeypatch):
    fixture=bound_fixture();metrics,frozen,completed,index,audits,old,_,_=fixture
    base=tmp_path/'benchmark';out=base/'reports/baseline_repair_v2';out.mkdir(parents=True)
    monkeypatch.setattr(renderer,'BASE',base);monkeypatch.setattr(renderer,'OUT',out)
    values={'metrics.json':metrics,'execution_freeze.json':frozen,'completed_blocks.json':completed,
        'run_index.json':index,**{n+'_audit.json':a for n,a in audits.items()}}
    def read(path):
        if 'dev_metrics_stage_qualified' in str(path):return old
        if path.name=='metadata.json':return dict(roadmap_id=path.parent.name,offline_seconds=2.)
        return values[path.name]
    def sha(path):
        if 'dev_metrics_stage_qualified' in str(path):return renderer.OLD_METRICS_SHA
        return {'run_index.json':'indexhash','execution_freeze.json':'freezehash'}.get(Path(path).name,'fixturehash')
    def no_bootstrap(rows,numerator,denominator):
        n=sum(numerator(r) for r in rows);d=sum(denominator(r) for r in rows)
        return dict(numerator=n,denominator=d,estimate=n/d if d else None,ci95=None)
    monkeypatch.setattr(renderer,'read',read);monkeypatch.setattr(renderer,'sha',sha)
    monkeypatch.setattr(renderer,'_cluster_ratio',no_bootstrap)
    renderer.main()
    text=(base/'reports/benchmark_v1_reliable_baselines.md').read_text()
    assert '候选终检累计' in text and '运动限位失败' in text and '本轮固定 Kinova Gen3' in text
    assert '同一机器跨时段的版本配对观测' in text and '不能单独归因为求解器改动' in text
    assert 'DRM 验收后成功' in text and '20 个场景' in text
    assert 'synthetic Python' in text and 'synthetic GPU' in text and 'synthetic-drake' in text
    assert '主指标定义' in text and '当前方法的同输入配对比较' in text and '成功率差/pp' in text
    assert 'boxbox_backend_investigation.md' in text and 'drake/pull/25144' in text
    assert '冻结的 Drake 1.51.1' in text and '未应用该补丁' in text
    assert 'cold mean/s' in text and 'warm mean/s' in text and '公共DRM及seed检查/s' in text
    assert '不是独立热启动重跑' in text and '不通过 cold−warm' in text
    receipt=json.loads((out/'report_receipt.json').read_text())
    assert receipt['legal_calls']==940 and len(receipt['run_manifest_sha256'])==4
    comparison=json.loads((out/'historical_comparison.json').read_text())
    assert comparison['comparisons'][0]['all_legal_cold_latency_difference']['estimate']==0
    assert len(comparison['current_method_comparisons'])==4
    timing=comparison['timing_statistics'][0]
    assert timing['cold_all_legal']['count']==180
    assert timing['warm_available_legal']['count']==144
    assert timing['frontend_component_available_legal']['count']==120
    assert timing['frontend_component_available_legal']['mean']==pytest.approx(.6)


def test_current_pair_orientation_and_ci_conversion():
    metrics=bound_fixture()[0]
    for r in metrics['rows']:
        if r['label']=='LSCS' and r['success']:r['cost']=5.;r['cold_e2e_seconds']=7.
    set_fixture_pairs(metrics)
    result=renderer.current_method_comparisons(metrics)
    pair=result[0]
    assert (pair['method_a'],pair['method_b'])==('LSCS','LGCS')
    assert pair['common_success_cost_difference']['estimate']==3.
    assert pair['common_success_cost_difference']['ci95']==pytest.approx([2.8,3.1])
    assert pair['observed_cold_latency_difference']['estimate']==4.
    assert pair['cost_unit']=='rad' and result[-1]['cost_unit']=='s'


def test_cost_uses_joint_success_and_latency_keeps_failures():
    metrics=bound_fixture()[0]
    first=next(r for r in metrics['rows'] if r['split']=='dev' and r['label']=='LGCS')
    first.update(success=False,status='timeout',cost=None,cold_e2e_seconds=60.)
    set_fixture_pairs(metrics)
    pair=renderer.current_method_comparisons(metrics)[0]
    assert pair['common_success_jobs']==179
    assert pair['valid_success_difference']['estimate']==pytest.approx(1/180)
    assert pair['observed_cold_latency_difference']['denominator']==180
    assert pair['observed_cold_latency_difference']['estimate']==pytest.approx(-57/180)
    assert pair['common_success_cost_difference']['denominator']==179


def test_zero_common_success_has_no_invented_cost_or_interval():
    metrics=bound_fixture()[0]
    for r in metrics['rows']:
        if r['label']=='LGCS':r.update(success=False,status='solver_failed',cost=None)
    set_fixture_pairs(metrics)
    pair=renderer.current_method_comparisons(metrics)[0]
    assert pair['common_success_jobs']==0
    assert pair['common_success_cost_difference']['estimate'] is None
    assert pair['common_success_cost_difference']['ci95'] is None
    assert renderer.delta_cell(pair['common_success_cost_difference'])=='—'


@pytest.mark.parametrize('mutation',[
    lambda m:m['paired_comparisons'].pop(),
    lambda m:m['paired_comparisons'].append(deepcopy(m['paired_comparisons'][0])),
    lambda m:m['paired_comparisons'][0].update(objective='minimum_time'),
    lambda m:m['paired_comparisons'][0].update(roadmap_nodes=3000),
    lambda m:m['paired_comparisons'][0].update(paired_legal_jobs=179),
    lambda m:m['paired_comparisons'][0]['valid_success_difference'].update(numerator=1),
    lambda m:m['paired_comparisons'][0]['common_success_cost_difference'].update(estimate=1),
    lambda m:m['paired_comparisons'][0]['common_success_cost_difference'].update(denominator=179),
    lambda m:m['paired_comparisons'][0]['observed_cold_latency_difference'].update(estimate=1),
    lambda m:m['paired_comparisons'][0]['valid_success_difference'].update(independent_scene_clusters=180),
    lambda m:m['paired_comparisons'][0]['valid_success_difference'].update(bootstrap_seed=1),
    lambda m:m['paired_comparisons'][0]['valid_success_difference'].update(ci95=[-2,2]),
    lambda m:m['paired_comparisons'][0]['common_success_cost_difference'].update(ci95=[1,0]),
    lambda m:next(r for r in m['rows'] if r['split']=='dev' and r['label']=='LGCS').update(method_seed=99999),
    lambda m:next(r for r in m['rows'] if r['split']=='dev' and r['label']=='LGCS').update(cluster_id='changed'),
    lambda m:next(r for r in m['rows'] if r['split']=='dev' and r['label']=='LGCS').update(q_start=[99]*7),
    lambda m:next(r for r in m['rows'] if r['split']=='dev' and r['label']=='LGCS').update(input_legal=False),
])
def test_current_pair_tampering_rejected(mutation):
    metrics=bound_fixture()[0];mutation(metrics)
    with pytest.raises((ValueError,KeyError)):renderer.current_method_comparisons(metrics)


def test_missing_warm_excluded_and_real_zero_kept():
    stats=renderer.observed_time_statistics([None,0.,2.,4.])
    assert stats==dict(count=3,mean=2.,p50=2.,p95=pytest.approx(3.8))
    assert renderer.observed_time_statistics([None,None])==dict(count=0,mean=None,p50=None,p95=None)


@pytest.mark.parametrize('value',[-1.,float('nan'),float('inf'),True,'2'])
def test_malformed_timing_not_silently_zero_filled(value):
    with pytest.raises(ValueError):renderer.observed_time_statistics([value])


def test_frontend_time_conditional_available_and_invalid_excluded():
    row=dict(input_legal=True,frontend_component_seconds=6.,timing={k:1. for k in renderer.FRONTEND_TIMERS})
    invalid=dict(row,input_legal=False,frontend_component_seconds=900.)
    missing=dict(input_legal=True,frontend_component_seconds=None,timing={})
    stats=renderer.frontend_time_statistics([row,invalid,missing])
    assert stats['count']==1 and stats['mean']==6.
    row['frontend_component_seconds']=5.
    with pytest.raises(ValueError,match='timer sum'):renderer.frontend_time_statistics([row])


def test_frontend_time_missing_component_rejected():
    row=dict(input_legal=True,frontend_component_seconds=6.,timing={k:1. for k in renderer.FRONTEND_TIMERS[:-1]})
    with pytest.raises(ValueError,match='complete saved timers'):renderer.frontend_time_statistics([row])
