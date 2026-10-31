"""Render the completed engineering study from sealed, independently audited rows."""
from collections import Counter
from pathlib import Path
import json

import numpy as np

from benchmarks.generic_motion_v1.common import BASE, read, write, sha
from benchmarks.generic_motion_v1.runners.summarize import _cluster_ratio

OUT=Path(__file__).resolve().parent
METHODS=[('LSCS','LSCS'),('LGCS','LGCS'),('NTO-refined-distance','NTO-distance'),
         ('SCSTO','SCSTO'),('NTO-refined-time','NTO-time')]
SOURCE='e6b7e372e091ce5b01417073e0880e7c3068910c'
OLD_SOURCE='72984027030175b7bca78e2dcf592b38f0c1864b'
OLD_MANIFEST_SHA='44452e26ea4e0eacee15e4df12e9cf183b48c561c6fd1ddf73ab86631a75919c'
OLD_METRICS_SHA='bebd650a691be6282cc2ab420ef9e8e35e9da46cfc04070f2585ac5022d7f9a3'
SUCCESS='success_validated_numerical'
FAILURES=('frontend_failed','solver_failed','collision_failed','validation_undetermined','timeout',
          'unsupported','infrastructure_error','invalid_endpoint','invalid_scene_geometry',
          'model_consistency_failed','limit_failed')
CURRENT_PAIRS=(('minimum_distance','LSCS','LGCS'),
               ('minimum_distance','LSCS','NTO-refined-distance'),
               ('minimum_distance','LGCS','NTO-refined-distance'),
               ('minimum_time','SCSTO','NTO-refined-time'))
FRONTEND_TIMERS=('model_setup_seconds','gpu_initialization_warmup_seconds','roadmap_load_seconds',
                 'online_collision_map_seconds','drm_path_seconds','drm_path_validation_seconds')


def require(condition,message):
    if not condition:raise ValueError(message)


def unique(rows,key,description):
    result={}
    for row in rows:
        k=key(row);require(k not in result,'Duplicate '+description+': '+str(k));result[k]=row
    return result


def row_key(row):
    return row['case_id'],row['roadmap_id'],row['method_seed'],row['label']


def observed_time_statistics(values):
    available=[]
    for value in values:
        if value is None:continue
        require(isinstance(value,(int,float)) and not isinstance(value,bool) and np.isfinite(value) and value>=0,
                'Malformed observed duration')
        available.append(value)
    if not available:return dict(count=0,mean=None,p50=None,p95=None)
    values=np.asarray(available,float)
    return dict(count=len(values),mean=float(values.mean()),p50=float(np.quantile(values,.5)),p95=float(np.quantile(values,.95)))


def verify_time_statistics(saved,expected,field):
    require(saved['count']==expected['count'],'Timing available count differs: '+field)
    for key in ('mean','p50','p95'):
        require(saved[key] is None if expected[key] is None else
            saved[key] is not None and np.isclose(saved[key],expected[key],rtol=1e-12,atol=1e-12),
            'Timing statistic differs from legal observations: '+field+'/'+key)


def frontend_time_statistics(rows):
    values=[]
    for row in rows:
        if not row['input_legal']:continue
        value=row.get('frontend_component_seconds')
        if value is None:continue
        timing=row['timing']
        require(all(key in timing for key in FRONTEND_TIMERS),'Frontend component lacks complete saved timers')
        parts=observed_time_statistics([timing[key] for key in FRONTEND_TIMERS])
        require(parts['count']==len(FRONTEND_TIMERS) and np.isclose(value,sum(timing[key] for key in FRONTEND_TIMERS),
                rtol=1e-12,atol=1e-12),'Frontend component differs from recorded timer sum')
        values.append(value)
    return observed_time_statistics(values)


def current_method_comparisons(metrics):
    """Check paired deltas from rows; retain existing scene-bootstrap intervals."""
    selected=[p for p in metrics['paired_comparisons'] if
        p['split']=='dev' and p['roadmap_nodes']==6000 and p['scope']=='obstacle_main']
    pair_index=unique(selected,lambda p:(p['objective'],frozenset((p['method_a'],p['method_b']))),'cross-method pair')
    require(set(pair_index)=={(o,frozenset((a,b))) for o,a,b in CURRENT_PAIRS},'Expected exactly four current method pairs')
    rows=[r for r in metrics['rows'] if r['split']=='dev' and r['roadmap_nodes']==6000 and r['family']!='open_control']
    result=[]
    for objective,a,b in CURRENT_PAIRS:
        source=pair_index[(objective,frozenset((a,b)))]
        left_label,right_label=source['method_a'],source['method_b']
        maps={label:unique([r for r in rows if r['label']==label],lambda r:row_key(r)[:3],'paired method input')
              for label in (left_label,right_label)}
        left,right=maps[left_label],maps[right_label]
        require(set(left)==set(right) and len(left)==source['paired_legal_jobs']==180,'Cross-method pairing/denominator differs')
        pairs=[]
        for key in sorted(left):
            x,y=left[key],right[key]
            require(x['input_legal'] is True and y['input_legal'] is True and x['objective']==y['objective']==objective,
                    'Cross-method objective/legality differs')
            for field in ('q_start','q_goal','input_sha256','robot_sha256','geometry_sha256','budget_seconds',
                          'cluster_id','family','straight_line_collision'):
                require(x[field]==y[field],'Cross-method fixed input differs: '+field)
            pairs.append((x,y))
        joint=[(x,y) for x,y in pairs if x['success'] and y['success']]
        sign=1 if left_label==a else -1
        item=dict(objective=objective,method_a=a,method_b=b,paired_legal_jobs=180,common_success_jobs=len(joint),
                  cost_unit='rad' if objective=='minimum_distance' else 's')
        specifications=(('valid_success_difference',pairs,lambda x,y:int(x['success'])-int(y['success'])),
            ('common_success_cost_difference',joint,lambda x,y:x['cost']-y['cost']),
            ('observed_cold_latency_difference',pairs,lambda x,y:x['cold_e2e_seconds']-y['cold_e2e_seconds']))
        for field,pool,delta in specifications:
            value=source[field];n=sum(delta(x,y) for x,y in pool);d=len(pool)
            clusters=len({x['cluster_id'] for x,y in pool})
            require(value['denominator']==d and np.isclose(value['numerator'],n,rtol=1e-10,atol=1e-10)
                and (np.isclose(value['estimate'],n/d,rtol=1e-10,atol=1e-10) if d else value['estimate'] is None),
                'Cross-method delta point estimate/denominator differs: '+field)
            require(value['independent_scene_clusters']==clusters and value['bootstrap_seed']==45827 and
                value['bootstrap_replicates']==2000,'Cross-method cluster/bootstrap metadata differs')
            ci=value['ci95']
            require((ci is None if clusters<2 else ci is not None) and
                (ci is None or (len(ci)==2 and np.isfinite(ci).all() and ci[0]<=ci[1])),
                'Malformed cross-method cluster interval')
            if field=='valid_success_difference' and ci is not None:
                require(-1<=ci[0]<=ci[1]<=1,'Success difference interval outside probability bounds')
            item[field]=dict(value,numerator=sign*value['numerator'],
                estimate=None if value['estimate'] is None else sign*value['estimate'],
                ci95=None if ci is None else (ci if sign==1 else [-ci[1],-ci[0]]))
        result.append(item)
    return result


def delta_cell(value,scale=1.,digits=3):
    if value['estimate'] is None:return '—'
    point=number(scale*value['estimate'],digits)
    ci=value['ci95']
    return point+' ['+(', '.join(number(scale*x,digits) for x in ci) if ci is not None else '无CI')+']'


def validate_evidence(metrics,frozen,completed,index,audits,old,index_sha,freeze_sha,completed_sha=None):
    """Bind small audit summaries and metric identities; do not rerun bootstrap."""
    require(all(a.get('passed') is True and a.get('errors')==[] for a in audits.values())
        and set(audits)=={'integrity','lgcs','nto'},'Three passing, error-free audits required')
    require(metrics.get('complete') is True and metrics.get('comparable') is True and
        metrics['requested_jobs']==950 and completed.get('complete') is True and
        completed['completed_jobs']==completed['total_jobs']==950,'All 950 sealed slots required')
    require(frozen['source_commit']==SOURCE and index['slots']==950 and index['test_planner_calls']==0,'Wrong study source/scope')
    blocks=unique(index['blocks'],lambda b:b['name'],'block')
    require(set(blocks)=={'smoke_s000','dev_s000','dev_s001','dev_s002'},'Wrong four blocks')
    run_hashes={str(Path(b['run']).resolve()):b['run_manifest_sha256'] for b in blocks.values()}
    require(len(run_hashes)==4,'Repeated run')
    runs=unique(metrics['runs'],lambda r:str(Path(r['run']).resolve()),'metric run')
    require(set(runs)==set(run_hashes),'Metrics are from different runs')
    for run,info in runs.items():
        require(info['manifest_sha256']==run_hashes[run] and info['source_commit']==SOURCE and
            info['integrity'].get('complete') is True and info['integrity'].get('integrity_passed') is True,'Metric run not sealed')
    integ=audits['integrity'];lgcs=audits['lgcs'];nto=audits['nto']
    require(integ.get('complete') is True and integ['source_code_commit']==SOURCE and
        (integ['registered_slots'],integ['legal_slots'],integ['invalid_slots'])==(950,940,10) and
        integ['run_manifest_sha256']==run_hashes,'Integrity coverage/source mismatch')
    require(integ['evidence']['run_index']['sha256']==index_sha and integ['evidence']['freeze']['sha256']==freeze_sha,
            'Integrity evidence belongs to a different study')
    if completed_sha is not None:
        require(integ['evidence']['completed']['sha256']==completed_sha,'Completion ledger changed after integrity audit')
    require(lgcs.get('complete') is True and lgcs['index_sha256']==index_sha and lgcs['freeze_sha256']==freeze_sha and
        lgcs['new_source_commit']==SOURCE and lgcs['old_source_commit']==OLD_SOURCE and lgcs['expected_lgcs_jobs']==190 and
        lgcs['paired_dev']['n']==lgcs['paired_dev']['legal_n']==180,'LGCS audit binding/coverage differs')
    require(nto['index_sha256']==index_sha and
        (nto['planned_new_slots'],nto['new_nto_slots'],nto['paired_dev_nto_slots'])==(950,380,360),'NTO audit binding/coverage differs')
    require(old.get('complete') is True and old.get('comparable') is True and old['requested_jobs']==900 and
        len(old['runs'])==1 and old['runs'][0]['manifest_sha256']==OLD_MANIFEST_SHA and
        old['runs'][0]['source_commit']==OLD_SOURCE,'Wrong historical metrics reference')
    for dataset,expected,labels in ((metrics,950,{x for x,_ in METHODS}),(old,900,{y for _,y in METHODS})):
        rows=dataset['rows'];require(len(rows)==expected,'Row count differs from declaration')
        unique(rows,row_key,'observation')
        require({r['label'] for r in rows}==labels,'Unexpected method labels')
        require(all(r['status'] in (SUCCESS,*FAILURES) and r['success']==(r['status']==SUCCESS) for r in rows),'Unknown or inconsistent status')
        require(all(np.isfinite(r['cold_e2e_seconds']) and r['cold_e2e_seconds']>=0 and
            (not r['success'] or (r['cost'] is not None and np.isfinite(r['cost']) and r['cost']>=0)) for r in rows),
            'Invalid observed cost/latency')
        require(sum(r['split']=='dev' for r in rows)==900 and
            sum(bool(r['input_legal']) for r in rows)==(940 if expected==950 else 900),'Wrong split/legal denominator')
        for label in labels:
            dev=[r for r in rows if r['split']=='dev' and r['label']==label]
            require(len(dev)==180 and all(r['input_legal'] and r['family']!='open_control' for r in dev)
                and len({r['cluster_id'] for r in dev})==20,'Wrong dev denominator/clustering')
        gs=unique(dataset['groups'],lambda g:(g['split'],g['roadmap_nodes'],g['scope'],g['label']),'metric group')
        for group in gs.values():
            scope=group['scope']
            pool=[r for r in rows if r['split']==group['split'] and r['label']==group['label'] and r['roadmap_nodes']==group['roadmap_nodes']]
            if scope=='obstacle_main':pool=[r for r in pool if r['family']!='open_control']
            elif scope=='open_control':pool=[r for r in pool if r['family']=='open_control']
            elif scope.startswith('family:'):pool=[r for r in pool if r['family']==scope.split(':',1)[1]]
            elif scope.startswith('straight_line:'):pool=[r for r in pool if r['family']!='open_control' and str(r['straight_line_collision'])==scope.split(':',1)[1]]
            else:continue
            legal=[r for r in pool if r['input_legal']];seed=[r for r in legal if r['drm_success'] is True]
            require(group['legal_jobs']==len(legal) and group['legal_status_counts']==dict(Counter(r['status'] for r in legal)),'Group status count mismatch')
            for field in ('cold_e2e_seconds','warm_query_seconds'):
                verify_time_statistics(group['statistics'][field]['all_legal'],
                    observed_time_statistics([r[field] for r in legal]),field)
            for name,selected in [('valid_success',legal),('conditional_validated_drm',seed)]:
                n=sum(r['success'] for r in selected);d=len(selected);ratio=group[name]
                require(ratio['numerator']==n and ratio['denominator']==d and
                    (ratio['estimate']==n/d if d else ratio['estimate'] is None),'Group ratio differs from observations')
                ci=ratio['ci95']
                require(ci is None or (len(ci)==2 and np.isfinite(ci).all() and 0<=ci[0]<=ci[1]<=1),'Malformed success interval')
    counts=unique(integ['groups'],lambda g:(g['split'],g['method'],g['family']),'integrity group')
    for split in ('smoke','dev'):
        for label,_ in METHODS:
            pool=[r for r in metrics['rows'] if r['split']==split and r['label']==label]
            group=counts[(split,label,'all')]
            require(group['registered']==len(pool) and group['status_counts']==dict(Counter(r['status'] for r in pool))
                and group['success']==sum(r['success'] for r in pool),'Independent counts differ from metrics')
    return run_hashes


def number(value, digits=3):
    return '—' if value is None else f'{value:.{digits}f}'


def interval(ratio):
    ci=ratio['ci95']
    return '—' if ci is None else f'{ci[0]*100:.1f}–{ci[1]*100:.1f}%'


def table(headers, rows):
    return '\n'.join(['| '+' | '.join(headers)+' |',
                      '| '+' | '.join(['---']*len(headers))+' |',
                      *['| '+' | '.join(map(str,row))+' |' for row in rows]])


def main():
    target=BASE/'reports/benchmark_v1_reliable_baselines.md'
    comparison_file=OUT/'historical_comparison.json'
    receipt_file=OUT/'report_receipt.json'
    if any(p.exists() for p in (target,comparison_file,receipt_file)):
        raise FileExistsError('Keep previous report/comparison/receipt; no existing evidence was modified')
    metrics=read(OUT/'metrics/metrics.json')
    frozen=read(OUT/'execution_freeze.json')
    completed=read(OUT/'completed_blocks.json')
    audits={name:read(OUT/f'{name}_audit.json') for name in ('integrity','lgcs','nto')}
    if (metrics.get('complete') is not True or metrics.get('comparable') is not True or
            metrics['requested_jobs']!=950 or completed.get('complete') is not True or
            completed['completed_jobs']!=950 or any(a.get('passed') is not True or a.get('errors')!=[] for a in audits.values())):
        raise ValueError('Completion requires all 950 sealed slots and three passing independent audits')
    oldpath=BASE/'reports/baseline_study/dev_metrics_stage_qualified/metrics.json'
    old=read(oldpath)
    require(sha(oldpath)==OLD_METRICS_SHA,'Archived historical metrics changed')
    index=read(OUT/'run_index.json')
    run_hashes=validate_evidence(metrics,frozen,completed,index,audits,old,sha(OUT/'run_index.json'),sha(OUT/'execution_freeze.json'),sha(OUT/'completed_blocks.json'))
    across=current_method_comparisons(metrics)
    across_table=[[c['method_a']+' − '+c['method_b'],c['paired_legal_jobs'],
        delta_cell(c['valid_success_difference'],100.,2),c['common_success_jobs'],
        delta_cell(c['common_success_cost_difference']),c['cost_unit'],delta_cell(c['observed_cold_latency_difference'])]
        for c in across]
    current={g['label']:g for g in metrics['groups'] if g['split']=='dev' and g['scope']=='obstacle_main'}
    previous={g['label']:g for g in old['groups'] if g['split']=='dev' and g['scope']=='obstacle_main'}
    oldrows={(r['case_id'],r['roadmap_id'],r['method_seed'],r['label']):r for r in old['rows']}
    comparisons=[]
    summary=[]
    failures=[]
    for label, oldlabel in METHODS:
        g=current[label]; before=previous[oldlabel]
        success=g['valid_success']; cond=g['conditional_validated_drm']
        times=g['statistics']['cold_e2e_seconds']['all_legal']
        summary.append([label,f'{int(before["valid_success"]["numerator"])}/{before["legal_jobs"]}',
            f'{int(success["numerator"])}/{g["legal_jobs"]}',f'{success["estimate"]*100:.1f}%',interval(success),
            f'{int(cond["numerator"])}/{int(cond["denominator"])}',number(times['p50']),number(times['p95'])])
        counts=g['legal_status_counts']
        failures.append([label,*[counts.get(s,0) for s in FAILURES]])
        require(sum(counts.get(s,0) for s in FAILURES)+int(success['numerator'])==g['legal_jobs'],'Failure categories do not sum to legal denominator')
        paired=[]
        for after in metrics['rows']:
            if after['split']!='dev' or after['label']!=label:
                continue
            key=(after['case_id'],after['roadmap_id'],after['method_seed'],oldlabel)
            before=oldrows[key]
            for field in ('q_start','q_goal','robot_sha256','geometry_sha256','input_sha256','objective','budget_seconds',
                          'input_legal','cluster_id','family','straight_line_collision'):
                if after[field]!=before[field]:
                    raise ValueError(f'Comparison input changed: {key}/{field}')
            paired.append(dict(a=before,b=after,cluster_id=after['cluster_id']))
        if len(paired)!=180 or {row_key(p['a']) for p in paired}!={row_key(r) for r in old['rows'] if r['label']==oldlabel}:
            raise ValueError('Missing historical/new task pairing')
        joint=[p for p in paired if p['a']['success'] and p['b']['success']]
        comparisons.append(dict(label=label,historical_label=oldlabel,paired_jobs=180,
            success_transitions=dict(Counter(f'{p["a"]["status"]} -> {p["b"]["status"]}' for p in paired)),
            common_success_jobs=len(joint),
            old_common_success_mean_cost=float(np.mean([p['a']['cost'] for p in joint])) if joint else None,
            new_common_success_mean_cost=float(np.mean([p['b']['cost'] for p in joint])) if joint else None,
            cost_unit='rad' if paired[0]['b']['objective']=='minimum_distance' else 's',
            paired_cost_difference=_cluster_ratio(joint,lambda p:p['b']['cost']-p['a']['cost'],lambda p:1),
            paired_success_difference=_cluster_ratio(paired,lambda p:int(p['b']['success'])-int(p['a']['success']),lambda p:1),
            common_success_cold_latency_difference=_cluster_ratio(joint,lambda p:p['b']['cold_e2e_seconds']-p['a']['cold_e2e_seconds'],lambda p:1),
            all_legal_cold_latency_difference=_cluster_ratio(paired,lambda p:p['b']['cold_e2e_seconds']-p['a']['cold_e2e_seconds'],lambda p:1),
            source_scope='Separate engineering versions, matched inputs; no pooling into one method score'))
    comparison=dict(historical_metrics_sha256=sha(oldpath),new_metrics_sha256=sha(OUT/'metrics/metrics.json'),
        comparisons=comparisons,current_method_comparisons=across)
    family=[]
    for name in ('sparse_clutter','dense_clutter','bottleneck','route_choice'):
        row=[name]
        for label,_ in METHODS:
            g=next(g for g in metrics['groups'] if g['split']=='dev' and g['scope']=='family:'+name and g['label']==label)
            row.append(f'{int(g["valid_success"]["numerator"])}/{g["legal_jobs"]}')
        family.append(row)
    blocked=[]
    for label,oldlabel in METHODS:
        for flag in (True,False):
            after=next(g for g in metrics['groups'] if g['split']=='dev' and g['label']==label and g['scope']=='straight_line:'+str(flag))
            before=next(g for g in old['groups'] if g['split']=='dev' and g['label']==oldlabel and g['scope']=='straight_line:'+str(flag))
            blocked.append([label,'直连碰撞' if flag else '直连未发现碰撞',after['legal_jobs'],
                int(before['valid_success']['numerator']),int(after['valid_success']['numerator'])])
    costs=[[c['label'],c['common_success_jobs'],number(c['old_common_success_mean_cost']),
            number(c['new_common_success_mean_cost']),c['cost_unit'],
            number(c['paired_cost_difference']['estimate'])] for c in comparisons]
    stages=[];latencies=[];timing_statistics=[]
    for label,_ in METHODS:
        g=current[label]
        pool=[r for r in metrics['rows'] if r['split']=='dev' and r['label']==label and
              r['roadmap_nodes']==6000 and r['family']!='open_control' and r['input_legal']]
        cold=g['statistics']['cold_e2e_seconds']['all_legal']
        warm=g['statistics']['warm_query_seconds']['all_legal']
        front=frontend_time_statistics(pool)
        latencies.append([label,cold['count'],number(cold['mean']),number(cold['p50']),number(cold['p95']),
                          warm['count'],number(warm['mean']),number(warm['p50']),number(warm['p95'])])
        timing_statistics.append(dict(label=label,cold_all_legal=cold,warm_available_legal=warm,
                                      frontend_component_available_legal=front))
        row=[label,f'{number(front["mean"])} (n={front["count"]})']
        for key in ('eizo_inflation_seconds','alternate_drm_path_seconds','trajectory_optimization_seconds','collision_recovery_seconds','validation_seconds'):
            stat=g['stage_seconds'].get(key,{}).get('all_legal')
            row.append('—' if not stat else f'{number(stat["mean"])} (n={stat["count"]})')
        stages.append(row)
    comparison['timing_statistics']=timing_statistics
    offline=[]
    for seed in range(3):
        m=read(BASE/f'roadmaps/rss2025_n6000_s{seed:03d}/metadata.json')
        offline.append([seed,m['roadmap_id'],number(m['offline_seconds'])])
    allrows=metrics['rows']; legal=sum(r['input_legal'] for r in allrows)
    statuses=dict(Counter(r['status'] for r in allrows))
    environment=frozen['environment'];packages=environment['packages']
    env_rows=[['Python',environment['python'].splitlines()[0].replace('|','/')],
        *[[name,packages[name]] for name in ('drake','csdecomp','scsplanning','clarabel')],
        ['GPU / driver / memory',environment['gpu'].replace('\n','; ').replace('|','/')],
        ['SCIP','独立后端；具体版本与构建见 LGCS 独立审计及冻结依赖清单']]
    chunks=[
        '# 论文方法 baseline：修复与完整开发测试\n',
        f'代码版本 `{frozen["source_commit"]}`。实际完成 **950 个登记组合**：10 道 smoke × 1 张路网 × 5 个组合，以及 60 道 dev × 3 张路网 × 5 个组合。其中 {legal} 次合法输入调用、{950-legal} 条输入无效记录。开发集有 20 个场景，重复端点和路网种子按场景聚类统计。冻结的正式 test 与既有正式结果保留原样。\n',
        '## 方法与实际修复\n',
        '- LSCS/SCSTO：调用原生 CSDecomp DRM、EI-ZO 和作者 SCSPlanning；仍执行区域覆盖、碰撞修复及统一终检。\n'
        '- 公共 DRM：项目内独立 CPU 模块仅把两个耗尽搜索的进程终止出口改为正常失败返回，继续链接原安装的 CUDA 函数。\n'
        '- LGCS：保留两路 DRM → EI-ZO → 图上混合整数最短路径问题，使用显式 SCIP 后端。保存整数流、模型、原始/对偶界与 gap；近零线段使用原始范数锥表达，解决平方表达的数值容差放大。\n'
        '- NTO_REFINED：保留论文的 SNOPT 距离目标、IPOPT 时间目标、样条阶数与 DRM 路径初始化规则。统一使用 1 mm 优化安全距离，从 149 个内部配点起步，最多 4 轮、600 配点；SNOPT 总迭代上限明确为 100000。全部轮次、几何检查和最终验证共用外部 60 秒预算。\n',
        '方法依据：[论文 VII-A 与 Appendix D](https://arxiv.org/html/2504.10783v1#S7.SS1)。本报告的机器人轨道为固定 Kinova Gen3；原始 NTO_PAPER、Gurobi 配置及其结果仍独立保存，版本间没有合并择优。\n',
        '本轮固定 Kinova Gen3、静态障碍和关节空间端点，测量论文方法在这套通用任务上的表现。\n',
        '### 实际冻结环境\n',table(['组件','实际记录'],env_rows)+'\n',
        '环境表直接读取运行前 execution_freeze.environment；继续使用既有 Conda，未替换原 CSDecomp wheel。二进制与依赖的逐文件 SHA 由完整性审计绑定。\n',
        '## 主指标定义\n',
        '成功率以全部合法固定查询为分母：在统一 60 秒外部预算下，只有通过相同模型、端点、碰撞及运动约束数值验收的输出才算成功；前端失败、求解失败、超时和未决均保留，无效输入另计。距离目标的代价是关节空间 PWL 各段欧氏长度之和（rad）；时间目标的代价是轨迹总时长（s）。冷启动时延是独立进程从启动到成功或失败结束的实际墙钟时间，包含初始化、规划和终检，离线建图另列。\n',
        '## 当前方法的同输入配对比较\n',
        table(['A − B','合法配对n','成功率差/pp [95% CI]','共同成功n','共同成功代价差 [95% CI]','代价单位','全合法冷时延差/s [95% CI]'],across_table)+'\n',
        '四行均来自本版 dev/6000/obstacle_main 的同题、同路网与同随机种子配对。所有差值方向为 A−B：成功率差为正表示 A 成功率更高；代价差和冷时延差为负表示 A 更低。距离与时间目标分别比较，不跨目标比较代价。代价仅条件于双方共同成功；冷时延差使用全部 180 个合法配对，包含失败与超时。95% 区间按场景聚类，保留同场景内端点及路网重复；渲染器独立核对配对身份、分母和点估计，区间沿用已绑定 metrics，不重复 bootstrap。\n',
        '## 60 道开发题的配对结果\n',
        '每种方法的分母均为 180 次合法查询。95% 区间采用 20 个场景的 cluster bootstrap；冷启动分位数包含合法输入上的失败与超时。\n',
        table(['方法','旧版成功','本版成功','本版成功率','95% 区间','本版 DRM 验收后成功','冷启动 p50/s','冷启动 p95/s'],summary)+'\n',
        '### 失败分类\n',
        table(['方法','前端失败','求解失败','碰撞失败','数值未决','超时','依赖不支持','基础设施错误','端点无效','场景几何无效','模型一致性失败','运动限位失败'],failures)+'\n',
        '### 各障碍族\n',table(['场景族',*[m[0] for m in METHODS]],family)+'\n',
        '### 直线连接标签\n',
        table(['方法','输入标签','查询数','旧版成功','本版成功'],blocked)+'\n',
        '标签只描述固定端点直线段的数值检查，不表示配置空间拓扑或全局可行性已经证明。\n',
        '### 共同成功题的代价\n',
        table(['方法','共同成功数','旧版平均代价','本版平均代价','单位','配对差：本版−旧版'],costs)+'\n',
        '只在新旧共同成功的相同题、相同路网和种子上比较代价；距离与时间分别处理。完整配对置信区间、成功/失败转移、全合法及共同成功冷启动时延差见 [historical_comparison.json](baseline_repair_v2/historical_comparison.json)。条件成功题时延不代表全体任务时延。新旧冷时延是同一机器跨时段的版本配对观测，机器负载也可能不同，不能单独归因为求解器改动。\n',
        '## 阶段耗时与离线开销\n',
        table(['方法','cold n','cold mean/s','cold p50/s','cold p95/s','warm n','warm mean/s','warm p50/s','warm p95/s'],latencies)+'\n',
        'cold 使用全部合法查询，包含失败与超时；warm 仅使用合法查询中实际保存的 warm_query_seconds，n 是该字段可用的记录数。warm 是同一次执行完成初始化后的计时，不是独立热启动重跑；缺失记录不填零，也不通过 cold−warm 估算初始化开销，两列可能使用不同样本集合。\n',
        table(['方法','公共DRM及seed检查/s','EI-ZO累计/s','第二路DRM/s','优化及内部细化累计/s','区域碰撞修复累计/s','候选终检累计/s'],stages)+'\n',
        '“公共DRM及seed检查”按已保存 frontend_component_seconds 的实际定义，累加模型设置、GPU预热、路网加载、在线碰撞表、DRM查询和seed数值检查六项 timer；仅纳入六项均完整可用的合法记录，显示条件均值与 n。该值包含上述初始化工作，但不含解释器启动、依赖探测和后续优化，不能称为完整冷启动或单独原生DRM查询时间。\n',
        '其余后端列为具有完整后端计时的尝试中，各阶段累计时长的均值，n 为尝试数。整个后端未完成时，即使早期阶段已返回，其后端计时也不进入均值；原始值保留，不填零。这是条件统计，不能相加当作全体线上均值。cold_e2e 包含解释器、模型/路网加载、GPU 初始化、规划与终检。\n',
        table(['离线路网种子','路网ID','实际离线构建秒数'],offline)+'\n',
        '本轮复用这三张路网；离线构建时间单列，不混入或从线上耗时中隐去。\n',
        '## 验收与可复现证据\n',
        '- 完整运行表：[run_index.json](baseline_repair_v2/run_index.json)。\n'
        '- 实际命令与退出状态：[commands.jsonl](baseline_repair_v2/commands.jsonl)。\n'
        '- 输入、环境、编译模块和源代码冻结：[execution_freeze.json](baseline_repair_v2/execution_freeze.json)。\n'
        '- 全局独立验收：[integrity_audit.json](baseline_repair_v2/integrity_audit.json)。\n'
        '- LGCS 独立验收：[lgcs_audit.json](baseline_repair_v2/lgcs_audit.json)。\n'
        '- NTO 独立验收：[nto_audit.json](baseline_repair_v2/nto_audit.json)。\n'
        '- 全部指标、逐条观测与阶段统计：[metrics.json](baseline_repair_v2/metrics/metrics.json)、[observations.csv](baseline_repair_v2/metrics/observations.csv)。\n'
        '- 回归测试：[479 项测试日志](baseline_repair_v2/full_tests_v1.log)；[11 项执行协议测试](baseline_repair_v2/development_study_fixture_tests.log)。\n',
        '```bash\n'
        'bash local/python.sh -m pytest benchmarks/generic_motion_v1/tests -q\n'
        '# 恢复已经冻结的四个运行块；已封存的成功和失败都不会重跑。\n'
        'bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_repair_v2.run_development_study\n'
        'bash local/python.sh -m benchmarks.generic_motion_v1 verify --run benchmarks/generic_motion_v1/results/rss2025_reliable_v2_dev_s000\n'
        '```\n',
        '## 具体未决项\n',
        'Drake 的 BoxBoxGradient 在保存的开发构型上仍能由纯 double 距离接口触发，详见[本地后端调查](baseline_repair_v2/boxbox_backend_investigation.md)。对症候选补丁 [Drake PR #25144](https://github.com/RobotLocomotion/drake/pull/25144) 在调查时仍 open、未合并；本批保持冻结的 Drake 1.51.1，未应用该补丁，也未声称本机已实测修好。相关失败如实保留，外部超时与原生 solver 限时分开记录；只有通过统一数值终检的输出才记成功。\n',
        '碰撞终检使用冻结的 0.005 rad 采样准则及相同碰撞模型，并保留全部采样数组；其结论是有限数值验证。时间轨迹另检查完整导数控制点界和静止端点。下一步算法比较应固定这套输入、指标和预算，单独声明方法及版本，不能按测试成绩挑题或改变碰撞模型。\n',
    ]
    # All checks/rendering precede writes. Exclusive creation also prevents a
    # concurrent invocation from replacing existing evidence after preflight.
    with comparison_file.open('x') as stream:json.dump(comparison,stream,indent=2,allow_nan=False)
    with target.open('x') as stream:stream.write('\n'.join(chunks))
    receipt=dict(report=str(target),report_sha256=sha(target),
        script_sha256=sha(__file__),metrics_sha256=sha(OUT/'metrics/metrics.json'),
        comparison_sha256=sha(comparison_file),audit_hashes={n:sha(OUT/f'{n}_audit.json') for n in audits},
        run_manifest_sha256=run_hashes,index_sha256=sha(OUT/'run_index.json'),freeze_sha256=sha(OUT/'execution_freeze.json'),
        historical_metrics_sha256=sha(oldpath),ci_verification_scope='Frozen statistics implementation/input binding; CI values are not independently recomputed by the renderer',
        current_method_comparison_pairs=len(across),current_pair_point_estimates_rechecked=True,
        statuses=statuses,registered_slots=950,legal_calls=legal,invalid_slots=950-legal)
    with receipt_file.open('x') as stream:json.dump(receipt,stream,indent=2,allow_nan=False)
    print(target)


if __name__=='__main__':
    main()
