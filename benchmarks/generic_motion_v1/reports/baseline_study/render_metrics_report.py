"""Render verified baseline metrics and fixed-budget observed-latency figures.

Reads sealed dev evidence only. No planner, solver, or GPU is invoked.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
BENCHMARK = HERE.parents[1]
LABELS = {'minimum_distance': ('LSCS', 'LGCS', 'NTO-distance'),
          'minimum_time': ('SCSTO', 'NTO-time')}
COLORS = {'LSCS': '#2166ac', 'LGCS': '#762a83', 'NTO-distance': '#d6604d',
          'SCSTO': '#2166ac', 'NTO-time': '#d6604d'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and np.isfinite(value)


def number(value, digits=3):
    return f'{value:.{digits}f}' if finite(value) else '—'


def interval(item, scale=1., digits=3):
    if item.get('estimate') is None:
        return '—'
    estimate = number(item['estimate']*scale, digits)
    bounds = item.get('ci95')
    return estimate + (f' [{number(bounds[0]*scale, digits)}, {number(bounds[1]*scale, digits)}]' if bounds else ' [CI 不可估]')


def rate(item, with_ci=True):
    count = f"{int(item['numerator'])}/{int(item['denominator'])}"
    value = interval(item, 100., 2) if with_ci else number(None if item['estimate'] is None else item['estimate']*100, 2)
    return f'{count}; {value}%'


def table(headers, rows):
    return ['| '+' | '.join(headers)+' |', '| '+' | '.join('---' for _ in headers)+' |'] + [
        '| '+' | '.join(str(cell).replace('\n', ' ').replace('|', '\\|') for cell in row)+' |' for row in rows] + ['']


def statistic(group, field, scope='all_legal'):
    return group['statistics'][field][scope]


def mean_n(item, scale=1.):
    return f"{number(None if item['mean'] is None else item['mean']*scale)} (n={item['count']})"


def raw_stat(values):
    values = np.asarray([value for value in values if finite(value)], float)
    return dict(count=len(values), mean=float(values.mean()) if len(values) else None,
                p50=float(np.median(values)) if len(values) else None,
                p90=float(np.quantile(values, .9)) if len(values) else None,
                p95=float(np.quantile(values, .95)) if len(values) else None,
                min=float(values.min()) if len(values) else None,
                max=float(values.max()) if len(values) else None)


def load_verified(metrics_path):
    """Fail closed on stale schemas, mismatched CSV, unsealed runs or pairing."""
    from ...batch import verify

    metrics_path = Path(metrics_path)
    csv_path = metrics_path.with_name('observations.csv')
    data = read(metrics_path)
    require(data.get('complete') is True and data.get('schema_version') == 1,
            'Metrics are incomplete or use an unsupported schema')
    require(set(data.get('expected_method_labels', [])) == set(sum(LABELS.values(), ())),
            'Metrics must be regenerated with all five declared paper methods')
    require(not data.get('pairing_differences'), 'Shared frontend differs between methods')
    rows = data['rows']
    require(len(rows) == data['requested_jobs'] and bool(rows), 'Observation count mismatch')
    require(all(row['split'] == 'dev' for row in rows), 'This report accepts dev evidence only')
    require({row['roadmap_nodes'] for row in rows} == {6000},
            'This report is the completed 6000-node dev study; do not pool unrun scaling conditions')
    require({row['budget_seconds'] for row in rows} == {60}, 'This observed-latency report requires the declared 60 s budget')
    require(all('completed_stage_seconds' in row and 'backend_timing_complete' in row for row in rows),
            'Old metrics lack completed backend timing eligibility; regenerate metrics')
    require(all(row['attempt_sealed'] and not row['retry_pending'] for row in rows), 'Unsealed observations')
    require(len({(row['run'], row['job_id']) for row in rows}) == len(rows), 'Duplicate observations')
    case_ids = {row['case_id'] for row in rows}
    map_ids = {row['roadmap_id'] for row in rows}
    expected_pairs = {(case_id, map_id, label) for case_id in case_ids
                      for map_id in map_ids for label in sum(LABELS.values(), ())}
    actual_pairs = [(row['case_id'], row['roadmap_id'], row['label']) for row in rows]
    require(set(actual_pairs) == expected_pairs and len(actual_pairs) == len(expected_pairs),
            'Missing or duplicate case/roadmap/method comparisons')
    require(len(case_ids) == 60 and len(map_ids) == 3 and
            len({row['scene_id'] for row in rows}) == 20,
            'Completed dev study requires the declared 60 cases, 20 scenes and three maps')
    require(all(row['success'] == (row['status'] == 'success_validated_numerical') for row in rows),
            'Success flags disagree with final status')
    for row in rows:
        if row['success']:
            require(row['input_legal'] and row['final_validation_passed'] is True
                    and row['solver_reported_success'] is True and row['drm_success'] is True
                    and row['native_drm_success'] is True, 'Success lacks solver/seed/validation gates')
    with csv_path.open(newline='') as stream:
        csv_rows = list(csv.DictReader(stream))
    require(len(csv_rows) == len(rows), 'CSV and JSON observation counts differ')
    for csv_row, row in zip(csv_rows, rows):
        for key, value in csv_row.items():
            expected = '' if row[key] is None else str(row[key])
            require(value == expected, f'CSV and JSON differ on {row["job_id"]}/{key}')
    for group in data['groups']:
        require('conditional_resolved_validated_drm' in group,
                'Old metrics lack the validated-seed support-condition statistic; regenerate metrics')
    for pair in data['paired_comparisons']:
        require('common_success_cold_latency_difference' in pair,
                'Old metrics lack common-success latency intervals; regenerate metrics')
    for record in data['runs']:
        require(record['integrity'].get('complete') and record['integrity'].get('integrity_passed'),
                'Input run was not verified')
        run = Path(record['run'])
        require(sha(run/'run_manifest.json') == record['manifest_sha256'], 'Run manifest hash changed')
        checked = verify(run)
        require(checked['complete'] and checked['integrity_passed'], f'Sealed run failed fresh verification: {run}')
        manifest = read(run/'run_manifest.json')
        require(manifest['formal_test_run'] is False, 'Formal test evidence must not enter this dev report')
        frozen_manifest = Path(manifest['manifest_path'])
        require(sha(frozen_manifest) == manifest['manifest_sha256'], 'Dev input manifest changed')
        declared_cases = {json.loads(line)['problem_id'] for line in frozen_manifest.read_text().splitlines() if line}
        require(case_ids == declared_cases, 'Dev input cases were omitted or added')
        observed = {row['job_id'] for row in rows if row['run'] == str(run)}
        require(observed == {job['job_id'] for job in manifest['jobs']}, 'Metrics omit or add sealed jobs')
    mains = {group['label']: group for group in data['groups'] if group['scope'] == 'obstacle_main'}
    require(set(mains) == set(sum(LABELS.values(), ())), 'Missing main objective groups')
    for label, group in mains.items():
        selected = [row for row in rows if row['label'] == label and row['family'] != 'open_control']
        legal = [row for row in selected if row['input_legal']]
        require(len(legal) == group['legal_jobs'] and len(selected) == group['requested_jobs'], 'Group denominator mismatch')
        require(dict(Counter(row['status'] for row in legal)) == group['legal_status_counts'], 'Group statuses mismatch')
        require(sum(row['success'] for row in legal) == group['valid_success']['numerator'], 'Group success mismatch')
        require(sum(row['backend_timing_complete'] for row in legal) == group['complete_backend_timing_jobs'],
                'Completed backend timing count differs from observations')
        for stage, scopes in group['stage_seconds'].items():
            actual = raw_stat([row['completed_stage_seconds'].get(stage) for row in legal])
            saved = scopes['all_legal']
            require(actual['count'] == saved['count'], f'{label}/{stage} completed timing count mismatch')
            require((actual['mean'] is None and saved['mean'] is None) or
                    (actual['mean'] is not None and saved['mean'] is not None and
                     np.isclose(actual['mean'], saved['mean'], rtol=1e-12, atol=1e-12)),
                    f'{label}/{stage} completed timing mean mismatch')
        for field in ('cold_e2e_seconds', 'warm_query_seconds', 'cost', 'sampled_gpu_peak_mib'):
            for scope, sample in [('all_legal', legal), ('validated_success', [row for row in legal if row['success']])]:
                actual = raw_stat([row[field] for row in sample])
                saved = group['statistics'][field][scope]
                require(actual['count'] == saved['count'], f'{label}/{field} observed n mismatch')
                for key in ('mean', 'p50', 'p90', 'p95'):
                    require((actual[key] is None and saved[key] is None) or
                            (actual[key] is not None and saved[key] is not None and np.isclose(actual[key], saved[key], rtol=1e-12, atol=1e-12)),
                            f'{label}/{field}/{key} differs from actual observations')
    return data, rows, mains, {str(metrics_path): sha(metrics_path), str(csv_path): sha(csv_path)}


def load_context(directory):
    audit_path = directory/'test_full_reaudit.json'
    readiness_path = directory/'test_input_readiness.json'
    audit, readiness = read(audit_path), read(readiness_path)
    require(audit.get('passed') is True and not audit['errors'], 'Test input reaudit did not pass')
    require(readiness.get('passed') is True and not readiness['errors'], 'Test input readiness did not pass')
    require(readiness['formal_test_planning_executed'] is False and readiness['planner_calls'] == 0,
            'Test planning state changed; report scope must be reviewed')
    require(audit['manifest_sha256'] == sha(BENCHMARK/'manifests/test.jsonl'), 'Frozen test manifest changed')
    require((audit['problem_count'], audit['valid_input_count'], audit['invalid_input_count']) == (420, 395, 25),
            'Test accounting differs from the audited v1 scope')
    builds_path = directory/'scaling_map_builds.json'
    ledger = read(builds_path)
    require(ledger.get('test_queries_executed') == 0, 'Scaling ledger contains unreviewed test planning')
    builds = ledger['builds']
    require(len(builds) == 9, 'Expected the nine completed additional offline builds')
    require({(item['metadata']['requested_nodes'], item['metadata']['seed']) for item in builds}
            == {(nodes, seed) for nodes in (3000, 9000, 12000) for seed in range(3)},
            'Incomplete or duplicate scaling roadmap matrix')
    for item in builds:
        metadata = item['metadata']
        require(item['returncode'] == 0 and metadata['status'] == 'ready', 'Offline build failed or is incomplete')
        require(item.get('test_queries_executed') == 0, 'Offline build record includes test planning')
        command = item['command']; path = Path(command[command.index('--output')+1])
        require(read(path/'metadata.json') == metadata, 'Offline build metadata differs from saved record')
        require(sha(path/'native.map') == metadata['native_map_sha256'], 'Offline roadmap hash mismatch')
    return audit, readiness, builds, {str(path): sha(path) for path in (audit_path, readiness_path, builds_path)}


def figures(rows, mains, directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import PercentFormatter

    directory.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False, 'axes.titleweight': 'semibold', 'savefig.facecolor': 'white'})
    paths = []
    for objective, labels in LABELS.items():
        fig, axes = plt.subplots(1, 3, figsize=(15.2, 4.7), gridspec_kw={'width_ratios': [1.35, 1.35, 1.]})
        maximum = max(row['cold_e2e_seconds'] for row in rows if row['objective'] == objective and row['input_legal'])
        for index, label in enumerate(labels):
            sample = [row for row in rows if row['label'] == label and row['input_legal'] and row['family'] != 'open_control']
            require(all(finite(row['cold_e2e_seconds']) for row in sample), 'Missing legal output latency cannot be silently excluded from figure')
            total = len(sample)
            success = sorted(row['cold_e2e_seconds'] for row in sample if row['success'])
            all_times = sorted(row['cold_e2e_seconds'] for row in sample)
            # Failures retain probability mass at +infinity in the validated
            # output ECDF, so the visible plateau is the actual success rate.
            axes[0].step([.45]+success+[maximum*1.02], [0.]+(np.arange(1, len(success)+1)/total).tolist()+[len(success)/total],
                         where='post', color=COLORS[label], linewidth=2., label=f'{label} ({len(success)}/{total})')
            axes[1].step([.45]+all_times+[maximum*1.02], [0.]+(np.arange(1,total+1)/total).tolist()+[1.],
                         where='post', color=COLORS[label], linewidth=2., label=label)
            estimate = mains[label]['valid_success']
            require(estimate['ci95'] is not None, 'Success-rate figure requires a supported confidence interval')
            value, (low, high) = estimate['estimate'], estimate['ci95']
            axes[2].errorbar(value, index, xerr=[[value-low],[high-value]], fmt='o', color=COLORS[label],
                             capsize=4, markersize=7, linewidth=1.6)
            axes[2].text(min(high+.025, .98), index, f'{100*value:.1f}%', va='center', ha='left', fontsize=9)
        for ax in axes[:2]:
            ax.set_xscale('log'); ax.set_xlim(.45, max(60.,maximum)*1.03); ax.set_ylim(0.,1.03)
            ax.set_xticks([.5,1,2,5,10,20,60], labels=['0.5','1','2','5','10','20','60'])
            ax.yaxis.set_major_formatter(PercentFormatter(1.)); ax.grid(True, alpha=.18)
            ax.set_xlabel('Observed cold wall time (s)')
        axes[0].set_title('A  Validated-output latency ECDF', loc='left')
        axes[0].set_ylabel('Fraction of all legal queries')
        axes[0].legend(loc='upper left', frameon=False, fontsize=8)
        axes[1].set_title('B  Termination latency, including failures', loc='left')
        axes[2].set_title('C  Numerical success with 95% CI', loc='left')
        axes[2].set_yticks(range(len(labels)), labels=labels); axes[2].invert_yaxis()
        axes[2].set_xlim(0,1.05); axes[2].xaxis.set_major_formatter(PercentFormatter(1.)); axes[2].grid(axis='x',alpha=.18)
        axes[2].set_xlabel('Success / all legal queries')
        title = 'Minimum distance' if objective == 'minimum_distance' else 'Minimum time'
        fig.suptitle(f'{title} · fixed dev queries · 6000-node roadmaps', x=.055, ha='left', fontsize=14, fontweight='semibold')
        fig.text(.055, .035, 'One observed 60 s budget per query; curves are not shorter-budget reruns or first-feasible-iterate times.\n'
                 'Failures remain in the denominator. Intervals resample complete scene clusters (20 scenes, 2000 replicates).', fontsize=9, color='#454545')
        fig.subplots_adjust(left=.055, right=.97, top=.82, bottom=.24, wspace=.35)
        path = directory/f'{objective}_observed_metrics.png'
        fig.savefig(path, dpi=180); plt.close(fig); paths.append(path)
    return paths


def render(data, rows, mains, audit, readiness, builds, images, hashes, output):
    case_count = len({row['case_id'] for row in rows})
    scene_count = len({row['scene_id'] for row in rows})
    map_count = len({row['roadmap_id'] for row in rows})
    lgcs = mains['LGCS']
    lines = ['# Kinova 通用运动规划：论文 baseline 指标', '',
             f"本报告读取已封存、重新核验的 **{data['requested_jobs']} 次 dev 运行**：{case_count} 个固定问题、{scene_count} 个障碍场景、{map_count} 个请求 6000 节点 DRM、五种论文方法。计算源码 commit 为 `{data['runs'][0]['source_commit']}`。距离和时间目标分开比较。结果是 Kinova 适配，不是作者 FR3/实机实验的精确复跑。", '',
             '数值成功要求固定输入合法、原生 DRM 成功、seed 独立检查通过、优化器返回成功且最终轨迹通过共同验证。失败、超时、许可证阻塞和进程异常均保留。有限碰撞采样不是连续安全证明。', '',
             '## 有效成功率与共同前端', '',
             '所有主成功率以全部合法查询为分母。方括号为场景整组 bootstrap 95% 区间：2000 次、PCG64 seed 45827；同场景端点和路网重复保持成组。条件率用于定位失败阶段，不能替代主分母。', '']
    lines += ['这些区间反映当前有限场景的经验重采样不确定性。分层只有少量独立场景时估计较弱；全成功或全失败会产生退化的 [100%, 100%] 或 [0%, 0%] bootstrap 区间，不能证明总体成功概率恰为 1 或 0。', '']
    for objective, labels in LABELS.items():
        lines += [f"### {'最短距离' if objective == 'minimum_distance' else '最短时间'}", '']
        lines += table(['方法','有效成功 / 合法查询；95% CI','成功 / 原生 DRM 成功','成功 / 严格 seed 准入','许可/依赖限制','基础设施异常'],
            [[label,rate(mains[label]['valid_success']),rate(mains[label]['conditional_native_drm']),
              rate(mains[label]['conditional_validated_drm']),mains[label]['legal_status_counts'].get('unsupported',0),
              mains[label]['legal_status_counts'].get('infrastructure_error',0)] for label in labels])
    lines += [f"LGCS 的许可证限制单独解释：{int(lgcs['valid_success']['numerator'])}/{int(lgcs['valid_success']['denominator'])} 是当前部署的有效输出率。{int(lgcs['conditional_validated_drm']['denominator'])} 条准入 seed 中有 {lgcs['legal_status_counts'].get('unsupported',0)} 个 MISOCP 被受限许可证拒绝。实际通过验证的输出数为 {int(lgcs['valid_success']['numerator'])}。这不证明 LGCS 优化算法比 LSCS 更不可靠。未进入优化器的查询，后端规模支持性仍未知。", '',
              '以下辅助口径只去除**观察到**依赖或基础设施失败，不表示其他案例获得一般性求解支持保证，也不消除规模选择偏差。', '']
    lines += table(['方法','无观察到依赖/基础设施故障','在该子集上的成功','在该子集且 seed 准入后的成功'],
        [[label,rate(mains[label]['dependency_or_infrastructure_resolved_rate']),rate(mains[label]['conditional_resolved_success']),
          rate(mains[label]['conditional_resolved_validated_drm'])] for label in sum(LABELS.values(),())])
    categories = ['frontend_failed','solver_failed','collision_failed','limit_failed','validation_undetermined','timeout','unsupported','infrastructure_error']
    lines += ['### 全部合法失败的分类', '']
    lines += table(['方法']+categories, [[label]+[mains[label]['legal_status_counts'].get(key,0) for key in categories] for label in sum(LABELS.values(),())])
    lines += ['`solver_failed` 包含保存的后端数值异常，不能自动归为算法无解；完整诊断见 [NTO 后端错误](full_baseline/nto_backend_failures.md)。原生 DRM 失败或断言也不是连续配置空间不可行的证明。', '',
              '## 实际延迟与输出累积分布', '',
              '冷耗时是外层进程墙钟，包含启动、模型/地图载入、依赖探测、GPU 初始化、规划、验证及输出。下表包含所有合法失败，缺失值绝不填零。warm 是同次执行去除初始化的阶段，不是独立热启动实验。超时清理可使观察墙钟略超 60 s。', '']
    for objective, labels in LABELS.items():
        lines += [f"### {'距离' if objective == 'minimum_distance' else '时间'}目标延迟", '']
        lines += table(['方法','合法冷耗时 n','冷 p50 / p90 / p95 (s)','冷最大 (s)','warm p50 (s; observed n)'],
            [[label,statistic(mains[label],'cold_e2e_seconds')['count'],
              ' / '.join(number(statistic(mains[label],'cold_e2e_seconds')[key]) for key in ('p50','p90','p95')),
              number(statistic(mains[label],'cold_e2e_seconds')['max']),
              f"{number(statistic(mains[label],'warm_query_seconds')['p50'])}; n={statistic(mains[label],'warm_query_seconds')['count']}"] for label in labels])
        matching = [path for path in images if path.name.startswith(objective)]
        if matching:
            relative = matching[0].relative_to(output.parent)
            lines += [f'![{objective} observed metrics]({relative.as_posix()})', '']
    lines += ['图 A 的分母始终是全部合法查询；未产生有效输出的任务保留在无穷延迟处，曲线平台即主成功率。图 B 显示所有合法任务的终止耗时，包括快速失败。图 A/B 均来自一次固定 60 s 预算的实际记录，**不是**短预算重跑，也不是首次可行迭代时间；快速失败不能解释成更快产出有效轨迹。', '',
              '## 成本与成对比较', '',
              '下表仅描述各方法自己的成功样本，不能用不同成功子集的均值排名。距离成本单位 rad，时间成本为物理轨迹持续时间 s。成本/端点参考使用忽略障碍的弦长或独立关节静止到静止运动松弛；不是经验最优值或紧的最优性差距。', '']
    for objective, labels in LABELS.items():
        unit = 'rad' if objective == 'minimum_distance' else 's'
        lines += table([f"{'距离' if objective == 'minimum_distance' else '时间'}方法",'成功成本 observed n',f'成本均值 / p50 ({unit})','成本/端点参考均值 (n)','相对同题 seed 长度改善均值 (n)'],
            [[label,statistic(mains[label],'cost','validated_success')['count'],
              ' / '.join(number(statistic(mains[label],'cost','validated_success')[key]) for key in ('mean','p50')),
              mean_n(statistic(mains[label],'cost_over_endpoint_reference','validated_success')),
              mean_n(statistic(mains[label],'distance_improvement_over_seed_fraction','validated_success'),100.)+'%' if objective=='minimum_distance' else '不适用'] for label in labels])
        pairs = [pair for pair in data['paired_comparisons'] if pair['scope']=='obstacle_main' and pair['objective']==objective]
        lines += table(['同目标 A − B','全部合法配对 n','主成功率差 pp [CI]','共同成功 n',f'共同成功成本差 ({unit}) [CI]','共同成功冷耗时差 s [CI]','全部合法冷耗时差 s [CI]'],
            [[pair['method_a']+' − '+pair['method_b'],pair['paired_legal_jobs'],interval(pair['valid_success_difference'],100.,2),
              int(pair['common_success_cost_difference']['denominator']),interval(pair['common_success_cost_difference']),
              interval(pair['common_success_cold_latency_difference']),interval(pair['observed_cold_latency_difference'])] for pair in pairs])
    lines += ['差值方向严格为 A−B。成本和相应共同成功耗时均条件于同题双方成功；总体耗时差保留合法失败。许可/基础设施条件筛选只作为辅助分析，不能消除由模型规模和失败造成的选择效应。', '',
              '## 场景族、直连状态和距离分层', '',
              '分层在方法运行前固定，表内给有效成功/该层合法查询及 95% CI。直连状态来自数值采样；bottleneck 与 route_choice 描述 workspace 几何，不声称已知 C-space 同伦。', '']
    scopes = [('family:sparse_clutter','障碍族：sparse_clutter'),('family:dense_clutter','障碍族：dense_clutter'),
              ('family:bottleneck','障碍族：bottleneck'),('family:route_choice','障碍族：route_choice'),
              ('straight_line:True','直连：采样检测到碰撞'),('straight_line:False','直连：采样未检测到碰撞'),
              ('distance_bin:short','规范化端点距离：short'),('distance_bin:medium','规范化端点距离：medium'),('distance_bin:long','规范化端点距离：long')]
    index = {(group['scope'],group['label']):group for group in data['groups']}
    for objective, labels in LABELS.items():
        lines += [f"### {'距离' if objective == 'minimum_distance' else '时间'}目标分层", '']
        lines += table(['固定分层','独立场景数']+list(labels),
            [[name,index[(scope,labels[0])]['legal_scene_clusters']]+[rate(index[(scope,label)]['valid_success']) for label in labels] for scope,name in scopes])
    lines += ['本次 dev 没有 open_control，不能由零行推断已评测无新增障碍对照；smoke 中的 open_control 保持在此前独立报告。', '',
              '## 区域、恢复、显存和 clearance', '',
              '所有单元标明实际可观察 n；区域/面/恢复取保存结果中可获得的全部合法记录，显存为进程分配的采样峰值，clearance 只描述最终成功轨迹。缺失不当作零，NTO 的零区域表示该方法不使用凸区域池。', '']
    lines += table(['方法','区域均值 (n)','面数均值 (n)','恢复均值 / 最大 (n)','显存 p50 / p95 MiB (n)','成功最小 clearance / p50 mm (n)'],
        [[label,mean_n(statistic(mains[label],'region_count')),mean_n(statistic(mains[label],'facets')),
          f"{number(statistic(mains[label],'repair_count')['mean'])} / {number(statistic(mains[label],'repair_count')['max'],0)} (n={statistic(mains[label],'repair_count')['count']})",
          f"{number(statistic(mains[label],'sampled_gpu_peak_mib')['p50'],0)} / {number(statistic(mains[label],'sampled_gpu_peak_mib')['p95'],0)} (n={statistic(mains[label],'sampled_gpu_peak_mib')['count']})",
          f"{number(None if statistic(mains[label],'minimum_sampled_clearance_m','validated_success')['min'] is None else 1000*statistic(mains[label],'minimum_sampled_clearance_m','validated_success')['min'])} / {number(None if statistic(mains[label],'minimum_sampled_clearance_m','validated_success')['p50'] is None else 1000*statistic(mains[label],'minimum_sampled_clearance_m','validated_success')['p50'])} (n={statistic(mains[label],'minimum_sampled_clearance_m','validated_success')['count']})"] for label in sum(LABELS.values(),())])
    stage_fields = [('eizo_inflation_seconds','EI-ZO 构造'),('alternate_drm_path_seconds','第二路径'),('trajectory_optimization_seconds','轨迹优化'),('collision_recovery_seconds','恢复'),('validation_seconds','最终轨迹数值验证')]
    lines += ['下面的后端阶段表仅纳入完整返回候选与最终验证、且无中断异常的后端调用；正常返回的 solver_failed 仍可纳入。未完整返回的后端所有阶段计时均排除，即便其中的膨胀或早期工作已经完成；原始计时与初始化零值仍保留在逐次记录的 timing 中。不存在的阶段记为“—”，完成后端中的真实零恢复保留。表中均值是条件阶段描述，不是全体查询延迟；所有合法失败仍计入前述冷耗时。恢复轮验证累计在对应阶段中，不应直接当作原论文纯求解器时间。', '']
    lines += table(['方法','完整计时后端 / 合法查询']+[name+' s (n)' for _,name in stage_fields],
        [[label,f"{mains[label]['complete_backend_timing_jobs']}/{mains[label]['legal_jobs']}"]+[mean_n(mains[label]['stage_seconds'][key]['all_legal']) if key in mains[label]['stage_seconds'] else '—' for key,_ in stage_fields] for label in sum(LABELS.values(),())])
    references = data['drm_references']; legal_ref = [row for row in references if row['input_legal']]
    valid_ref = [row for row in legal_ref if row['drm_success']]
    ref_length = raw_stat([row['seed_cost_rad'] for row in valid_ref]); ref_time = raw_stat([row['frontend_component_seconds'] for row in valid_ref])
    lines += ['## DRM-PWL 距离前端参考', '',
              f"按预先指定的 LSCS 调用抽取 {len(legal_ref)} 个合法查询，严格 seed 准入 {len(valid_ref)} 个。抽取不依赖后续优化器成功，不从五次调用中挑选最快记录。DRM-PWL 是原生几何折线参考，不是优化器输出，也没有可用于 minimum-time 比较的物理时间参数化。", '']
    lines += table(['原始记录指标','observed n','均值','p50','p90','p95'],
        [['已验收 seed 折线长度 (rad)',ref_length['count']]+[number(ref_length[key]) for key in ('mean','p50','p90','p95')],
         ['原调用前端组件计时之和 (s)',ref_time['count']]+[number(ref_time[key]) for key in ('mean','p50','p90','p95')]])
    lines += ['组件之和包括模型配置、GPU 预热、地图载入、碰撞缓存、DRM Plan 和 seed 数值验收；不包括解释器启动、依赖探测以及未归入这些阶段的开销，也没有轨迹优化。它不是独立 DRM 冷/热耗时，不与完整方法冷耗时计算速度比。', '',
              '## 已完成的额外离线路网', '',
              '下列九份是实际完成的离线构建；按请求节点数和 seed 列出真实保留节点，不能把请求规模直接当作实际节点。其规划分数尚未执行，本文不填写这些规模的成功率或在线延迟。离线时间与在线查询分开，原 6000 节点产物被复用而非重复计作新实验。', '']
    lines += table(['请求节点','seed','实际节点','有向边','离线构建 s','外层进程 s','地图 MiB'],
        [[item['metadata']['requested_nodes'],item['metadata']['seed'],item['metadata']['actual_nodes'],item['metadata']['directed_edges'],
          number(item['metadata']['offline_seconds']),number(item['process_wall_seconds']),number(item['metadata']['native_map_bytes']/2**20)] for item in builds])
    lines += ['## Test 输入验收与正式评测边界', '',
              f"420 条冻结 test 的实际输入复核已通过：**{audit['valid_input_count']} 条合法输入、{audit['invalid_input_count']} 条生成无效输入**，{audit['scene_count']} 个场景 ID；复核重放了 {audit['endpoint_proposals_replayed']} 个端点采样提案。通过审计表示真实状态和冻结证据一致，不表示 420 条都合法，也不证明全局路径可行。", '',
              '**用户已选择保留 v1：395 个合法问题进入正式算法评测分母，25 个输入生成失败独立保留在 420 条登记记录中。** 本报告生成时正式 test 规划尚未启动；需要先完成执行门控与代码冻结。原输入不替换，本报告只有 dev 在线成绩及真实离线构建记录，不将后续正式矩阵写成已完成。', '',
              '## 来源和复现命令', '',
              '证据：[指标 JSON](baseline_study/dev_metrics/metrics.json)、[逐次 CSV](baseline_study/dev_metrics/observations.csv)、[test 实际复核](baseline_study/test_full_reaudit.json)、[test 数据口径审查](baseline_study/test_input_readiness.md)、[离线构建台账](baseline_study/scaling_map_builds.json)、[五方法运行报告](benchmark_v1_full_baseline.md)。', '',
              '```bash',
              'bash local/python.sh -m benchmarks.generic_motion_v1.runners.study_metrics \\',
              '  --runs benchmarks/generic_motion_v1/results/rss2025_full_dev_v1 \\',
              '  --output benchmarks/generic_motion_v1/reports/baseline_study/dev_metrics',
              'bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_study.render_metrics_report',
              '```', '',
              '渲染前要求所有 run 完整且重新通过 SHA/receipt 核验，CSV 逐字段等于 JSON observations，五方法完整配对，主统计与实际 observations 一致。脚本不运行规划或碰撞检查。输入和渲染源码 SHA-256：', '']
    lines += table(['文件','SHA-256'], [[str(Path(path).relative_to(BENCHMARK)) if BENCHMARK in Path(path).parents else str(path),value] for path,value in hashes.items()])
    return '\n'.join(lines)+'\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metrics', type=Path, default=HERE/'dev_metrics/metrics.json')
    parser.add_argument('--output', type=Path, default=HERE.parent/'benchmark_v1_baseline_metrics.md')
    parser.add_argument('--no-figures', action='store_true')
    args = parser.parse_args(argv)
    data, rows, mains, hashes = load_verified(args.metrics.resolve())
    audit, readiness, builds, context_hashes = load_context(HERE)
    hashes.update(context_hashes); hashes[str(Path(__file__).resolve())] = sha(__file__)
    output = args.output.resolve(); output.parent.mkdir(parents=True, exist_ok=True)
    images = [] if args.no_figures else figures(rows, mains, output.parent/'baseline_study/figures')
    content = render(data, rows, mains, audit, readiness, builds, images, hashes, output)
    require(all(sha(path) == expected for path,expected in hashes.items()), 'Inputs changed during rendering; refusing stale report')
    output.write_text(content)
    print(json.dumps(dict(report=str(output), observations=len(rows), verified_runs=len(data['runs']),
                          figures=[str(path) for path in images], test_planning_executed=False),indent=2))


if __name__ == '__main__':
    main()
