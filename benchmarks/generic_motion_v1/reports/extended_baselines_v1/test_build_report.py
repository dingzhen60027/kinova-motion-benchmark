"""Synthetic renderer fixtures only: no benchmark scores are created here."""
from collections import Counter
from copy import deepcopy
import itertools
import json
from pathlib import Path
import pytest
from benchmarks.generic_motion_v1.reports.extended_baselines_v1 import build_report as builder


def stat(values):
    return dict(count=len(values),mean=sum(values)/len(values) if values else None,
                p50=values[0] if values else None,p95=values[0] if values else None)


def group(rows,split,label,scope):
    legal=[row for row in rows if row['input_legal']]
    successes=[row for row in legal if row['success']]
    stats={}
    for field in ['cold_e2e_seconds','warm_query_seconds','cost']:
        stats[field]={pool:stat([row[field] for row in selected if row[field] is not None])
                      for pool,selected in [('all_legal',legal),('validated_success',successes)]}
    rate=len(successes)/len(legal) if legal else None
    return dict(split=split,label=label,scope=scope,requested_jobs=len(rows),legal_jobs=len(legal),
        status_counts=dict(Counter(row['status'] for row in rows)),legal_status_counts=dict(Counter(row['status'] for row in legal)),
        valid_success=dict(numerator=len(successes),denominator=len(legal),estimate=rate,ci95=[rate,rate] if rate is not None else None),
        converged_successes=0,validated_incumbent_nonconverged=0,validated_unknown_convergence=len(successes),
        unknown_convergence=len(legal),termination_counts={'unknown':len(legal)},statistics=stats,
        complete_backend_timing_jobs=len(successes),stage_seconds={})


@pytest.fixture
def metrics():
    rows=[]
    for label in builder.LABELS:
        for split,count,repeats in [('dev',60,3),('smoke',10,1)]:
            for index in range(count):
                family=builder.FAMILIES[index%4] if split=='dev' or index<8 else 'open_control'
                legal=split=='dev' or index not in [2,3]
                for seed in range(repeats):
                    rows.append(dict(case_id=f'{split}-{index}',label=label,split=split,input_legal=legal,
                        success=legal,status=builder.SUCCESS if legal else 'invalid_endpoint',family=family,
                        straight_line_collision=index%2==0,cold_e2e_seconds=1.,warm_query_seconds=.5,
                        cost=2. if legal else None,robot_sha256='fixture-robot',geometry_sha256='fixture-geometry'))
    groups=[]
    for split in ['dev','smoke']:
        for label in builder.LABELS:
            pool=[row for row in rows if row['split']==split and row['label']==label]
            strata=[('all',pool),('obstacle_main',[r for r in pool if r['family']!='open_control']),
                    ('open_control',[r for r in pool if r['family']=='open_control'])]
            strata += [('family:'+f,[r for r in pool if r['family']==f]) for f in builder.FAMILIES]
            strata += [('straight_line:'+str(value),[r for r in pool if r['straight_line_collision'] is value]) for value in [True,False]]
            groups += [group(selected,split,label,scope) for scope,selected in strata if selected]
    pairs=[]
    for a,b in itertools.combinations(builder.LABELS,2):
        delta=dict(numerator=0.,denominator=180,estimate=0.,ci95=[0.,0.])
        pairs.append(dict(split='dev',scope='all',method_a=a,method_b=b,paired_legal_jobs=180,
            common_success_cost_difference=deepcopy(delta),valid_success_difference=deepcopy(delta),
            observed_cold_latency_difference=deepcopy(delta)))
    return dict(complete=True,comparable=True,requested_jobs=1140,rows=rows,groups=groups,
                pairing_differences=[],paired_comparisons=pairs,expected_method_labels=list(builder.LABELS))


def auxiliary():
    cfg=json.loads((builder.BASE/'configs/extended_baselines.json').read_text())
    frozen=dict(source_commit='fixture-source-not-an-experiment',environment=dict(gpu='fixture-gpu',executable='fixture-python',
        python='3.10.0 fixture',packages={'numpy':'fixture'}),evaluation_config=cfg,timeout_seconds=60.)
    provenance=dict(ompl=dict(dependency={'source_commit':'fixture-ompl'},wheel={'sha256':'fixture-wheel'}),
        chomp_stomp={'STOMP_source_commit':'fixture-stomp'},
        trajopt=dict(author_commit='fixture-trajopt',build_receipt={'transport_only_patch_sha256':'fixture-patch'}))
    maps=[dict(seed=i,requested_nodes=6000,actual_nodes=5999,offline_seconds=60.) for i in range(3)]
    audit=dict(passed=True,check_count=123,errors=[])
    protected={'/fixture/manifests/test.jsonl':'fixture-protected-test-sha','/fixture/manifests/test.SHA256SUMS':'fixture-checksum-sha'}
    commands=[dict(event='finish',block='fixture',action='prepare',returncode=0,argv=['python','-m','fixture','--output','a b'])]
    return frozen,audit,protected,maps,commands,provenance


def test_complete_matrix_counts_are_verified(metrics):
    assert len(builder.validate_counts(metrics))==1140


@pytest.mark.parametrize('mutation',[
    lambda m:m.update(complete=False),lambda m:m.update(comparable=False),
    lambda m:m.update(requested_jobs=1139),lambda m:m['rows'].pop(),
    lambda m:m['rows'][0].update(input_legal=False),lambda m:m['rows'][0].update(split='test'),
    lambda m:m['groups'][0]['status_counts'].update(forged=1),
    lambda m:m['groups'][0].update(converged_successes=1),
    lambda m:m['groups'][0]['statistics']['cold_e2e_seconds']['all_legal'].update(mean=2.),
])
def test_partial_mixed_or_forged_metrics_refused(metrics,mutation):
    mutation(metrics)
    with pytest.raises(ValueError):builder.validate_counts(metrics)


def test_report_keeps_sources_objectives_denominators_and_comparison_direction(metrics):
    text=builder.render(metrics,*auxiliary())
    assert '1,140' in text and '1,128' in text and '12 个无效槽位' in text
    assert '180/180' in text and '8/8' in text
    assert 'C-space' in text and '未收敛但验证成功' in text
    assert '每个罚系数阶段最多 50 次 SQP' in text
    assert '不截断为 60 秒' in text
    assert '未产生候选的失败不补造轨迹' in text
    assert 'LSCS − RRTConnect' in text
    assert 'fixture-source-not-an-experiment' in text
    assert 'fixture-protected-test-sha' in text
    assert '没有新增规划调用' in text
    assert 'schema' not in text
    assert '实际构建记录' in text
    assert "'a b'" in text
    assert '交互回放图库' not in text


def test_replays_are_linked_only_after_the_gallery_exists(metrics):
    assert '交互回放图库' in builder.render(metrics,*auxiliary(),replay_available=True)


def test_renderer_requires_all_fifteen_paired_comparisons(metrics):
    metrics['paired_comparisons'].pop()
    with pytest.raises(ValueError,match='15 method pairs'):
        builder.render(metrics,*auxiliary())


def test_refuses_existing_output_before_reading_evidence(tmp_path):
    output=tmp_path/'existing.md';output.write_text('keep')
    with pytest.raises(ValueError,match='overwrite'):
        builder.build(tmp_path,output)
    assert output.read_text()=='keep'


def test_refuses_existing_receipt_without_creating_report(tmp_path):
    (tmp_path/'report_receipt.json').write_text('{}')
    output=tmp_path/'new.md'
    with pytest.raises(ValueError,match='receipt'):
        builder.build(tmp_path,output)
    assert not output.exists()


def test_refuses_missing_incomplete_evidence_without_creating_report(tmp_path):
    output=tmp_path/'new.md'
    with pytest.raises(FileNotFoundError):builder.build(tmp_path,output)
    assert not output.exists()


def test_cell_escape_and_missing_values():
    assert builder.escape('a|b\nc')==r'a\|b c'
    assert builder.number(None)=='—'
    assert builder.ratio({'estimate':None})=='—'
    assert builder.difference({'estimate':None})=='—'
    assert builder.difference({'estimate':.1,'ci95':[-.2,.2]},100)=='10.000 [-20.000, 20.000]'

@pytest.mark.parametrize('change,match',[
    ({'passed':False},'has not passed'),
    ({'errors':['actual audit error']},'has not passed'),
    ({'metrics_sha256':'wrong'},'these metrics'),
    ({'execution_freeze_sha256':'wrong'},'execution freeze'),
    ({'run_index_sha256':'wrong'},'run index'),
])
def test_audit_must_bind_current_completed_metrics_and_freeze(metrics,tmp_path,change,match):
    metrics_path=tmp_path/'metrics.json';metrics_path.write_text(json.dumps(metrics))
    freeze=tmp_path/'execution_freeze.json';freeze.write_text('{}')
    index=tmp_path/'run_index.json';index.write_text('{}')
    audit=dict(passed=True,errors=[],counts=dict(registered_slots=1140,legal_slots=1128,invalid_slots=12,
        receipts=1140,validated_successes=1128),metrics_sha256=builder.sha(metrics_path),
        execution_freeze_sha256=builder.sha(freeze),run_index_sha256=builder.sha(index))
    audit.update(change)
    audit_path=tmp_path/'audit.json';audit_path.write_text(json.dumps(audit))
    with pytest.raises(ValueError,match=match):builder.validate_inputs(tmp_path,metrics_path,audit_path)


def test_documented_replay_example_selects_only_sealed_success(tmp_path,monkeypatch):
    import subprocess
    import sys
    documentation=(builder.BASE/'EXTENDED_BASELINES.md').read_text()
    code=documentation.split("<<'PY'\n",1)[1].split('\nPY',1)[0]
    for name,status,sealed in [('a','success_validated_numerical',False),('b','timeout',True),
                               ('c','solver_failed',True),('d','success_validated_numerical',True)]:
        path=tmp_path/'run/jobs'/name/'attempts/0001';path.mkdir(parents=True)
        (path/'validation.json').write_text('{"passed":true}')
        (path/'candidate.json').write_text('{}')
        (path/'result.json').write_text(json.dumps({'status':status}))
        if sealed:(path/'receipt.json').write_text('{}')
    captured=[]
    monkeypatch.setattr(sys,'argv',['fixture',str(tmp_path/'run'),str(tmp_path/'replay')])
    monkeypatch.setattr(subprocess,'run',lambda args,**kwargs:captured.append((args,kwargs)))
    exec(compile(code,'EXTENDED_BASELINES.md replay example','exec'),{})
    assert len(captured)==1
    assert captured[0][0][captured[0][0].index('--attempt')+1]==str(tmp_path/'run/jobs/d/attempts/0001')
    assert captured[0][1]['check'] is True


def test_documentation_contains_paired_analysis_and_precise_timing_scope():
    documentation=(builder.BASE/'EXTENDED_BASELINES.md').read_text()
    assert 'runners.extended_metrics' in documentation
    assert '--runs "$EXTENDED_ROOT/dev_all" --output "$EXTENDED_ROOT/dev_all_metrics"' in documentation
    assert '20 independent obstacle scenes' in documentation
    assert '1,128 legal-input calls' in documentation
    assert 'not\n  clipped to 60 seconds' in documentation
    assert 'not a\n  second independently measured warm-start run' in documentation
