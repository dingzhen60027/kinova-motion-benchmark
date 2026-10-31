"""Render completed, independently audited formal baseline evidence.

This consumer never runs a planner, solver, collision checker, receipt audit or
bootstrap. It must be called only after the study and independent audits finish.
Small evidence-file hashes bind the completed audits to the metrics being shown.
"""
from __future__ import annotations

import argparse
from collections import Counter
from itertools import combinations
import json
from pathlib import Path

import numpy as np

from .render_metrics_report import (
    BENCHMARK, COLORS, HERE, LABELS, finite, interval, mean_n, number, rate,
    raw_stat, read, require, sha, statistic, table,
)

METHODS = sum(LABELS.values(), ())
FAMILIES = ('sparse_clutter', 'dense_clutter', 'bottleneck', 'route_choice')
PAIR_LABELS = {('EI_ZO', 'minimum_distance'): 'LSCS',
               ('LGCS', 'minimum_distance'): 'LGCS',
               ('NTO_PAPER', 'minimum_distance'): 'NTO-distance',
               ('EI_ZO', 'minimum_time'): 'SCSTO',
               ('NTO_PAPER', 'minimum_time'): 'NTO-time'}
FAILURES = ('frontend_failed', 'solver_failed', 'collision_failed', 'limit_failed',
            'validation_undetermined', 'timeout', 'unsupported', 'infrastructure_error')
BACKEND_STAGES = (('eizo_inflation_seconds', 'EI-ZO'),
                  ('alternate_drm_path_seconds', '第二路径'),
                  ('trajectory_optimization_seconds', '轨迹优化'),
                  ('collision_recovery_seconds', '恢复'),
                  ('validation_seconds', '数值验证'))


def checked_file(reference, base, fingerprints):
    path = Path(reference['path'])
    path = path if path.is_absolute() else base / path
    path = path.resolve()
    actual = sha(path)
    require(actual == reference['sha256'], f'Evidence hash mismatch: {path}')
    fingerprints[str(path)] = actual
    return read(path)


def row_key(row):
    return row['case_id'], row['roadmap_nodes'], row['roadmap_seed'], row['label']


def protocol_rows(protocol, index, phases, selected_ids=None):
    """Derive the exact matrix from the frozen protocol, never from successes."""
    blocks = {item['block']: item for item in index['blocks']}
    require(set(blocks) == set(protocol['blocks']), 'Run index and protocol blocks differ')
    require(len(blocks) == len(index['blocks']), 'Duplicate run-index block')
    expected = {}
    for name, block in protocol['blocks'].items():
        if block['phase'] not in phases:
            continue
        require(len(block['requested_node_budgets']) == len(block['roadmap_seeds']) == 1,
                'Report expects one fixed size and seed per declared block')
        labels = [PAIR_LABELS[tuple(pair)] for pair in block['method_pairs']]
        require(set(labels) == set(METHODS), 'Unexpected method/objective protocol')
        nodes, seed = block['requested_node_budgets'][0], block['roadmap_seeds'][0]
        require(block['problem_count'] == len(set(block['problem_ids'])), 'Duplicate protocol case')
        for case in block['problem_ids']:
            if selected_ids is not None and case not in selected_ids:
                continue
            for label in labels:
                key = case, nodes, seed, label
                require(key not in expected, f'Duplicate registered observation: {key}')
                expected[key] = blocks[name]['run']
    return expected


def require_matrix(data, expected, expected_runs, source_commit):
    require(data.get('complete') is True and data.get('comparable') is True,
            'Incomplete or incomparable metrics are diagnostic evidence, not formal scores')
    require(data.get('schema_version') == 1, 'Unsupported metrics schema')
    require(not data.get('pairing_differences'), 'Different shared frontend paths or outcomes')
    require(set(data['expected_method_labels']) == set(METHODS), 'Missing declared methods')
    rows = data['rows']
    require(len(rows) == data['requested_jobs'] == len(expected), 'Registered matrix count mismatch')
    observed = {row_key(row): row for row in rows}
    require(len(observed) == len(rows), 'Duplicate observation, including reused 6000-node data')
    require(set(observed) == set(expected), 'Missing or extra prespecified case/size/seed/method')
    fixed_inputs, seeds = {}, {}
    for key, row in observed.items():
        require(row['run'] == expected[key], 'Observation attributed to the wrong registered block')
        require(row['split'] == 'test' and row['budget_seconds'] == 60, 'Wrong split or budget')
        require(row['attempt_sealed'] is True and row['retry_pending'] is False, 'Unsealed attempt')
        require('completed_stage_seconds' in row and 'backend_timing_complete' in row,
                'Completed-stage timing eligibility is missing')
        require(row['success'] == (row['status'] == 'success_validated_numerical'), 'Success/status mismatch')
        if row['input_legal']:
            require(finite(row['cold_e2e_seconds']) and row['cold_e2e_seconds'] > 0,
                    'A legal failure or success has missing output latency')
        if row['success']:
            require(row['input_legal'] and row['native_drm_success'] is True and row['drm_success'] is True
                    and row['solver_reported_success'] is True and row['final_validation_passed'] is True,
                    'Numerical success lacks input, frontend, solver or validator gates')
            require(finite(row['cost']) and row['cost'] >= 0, 'Success lacks a finite objective cost')
        signature = json.dumps({key: row[key] for key in ('input_sha256', 'input_legal', 'robot_sha256',
            'geometry_sha256', 'q_start', 'q_goal', 'family', 'distance_bin', 'straight_line_collision',
            'cluster_id')}, sort_keys=True)
        previous = fixed_inputs.setdefault(row['case_id'], signature)
        require(previous == signature, 'Fixed input differs across methods, sizes or seeds')
        task = key[:3]
        require(seeds.setdefault(task, row['method_seed']) == row['method_seed'], 'Method seeds differ within a pair')
    records = {item['run']: item for item in data['runs']}
    require(len(records) == len(data['runs']) and set(records) == set(expected_runs),
            'Metrics do not reference exactly the audited input blocks')
    for run, record in records.items():
        require(record['source_commit'] == source_commit, 'Frozen runtime commit changed')
        require(record['manifest_sha256'] == expected_runs[run], 'Run manifest binding mismatch')
        require(record['integrity'].get('complete') is True and record['integrity'].get('integrity_passed') is True,
                'Metrics were derived from an unverified or incomplete run')
    return observed


def sample_for_scope(rows, nodes, scope, label):
    sample = [row for row in rows if row['roadmap_nodes'] == nodes and row['label'] == label]
    if scope == 'obstacle_main':
        return [row for row in sample if row['family'] != 'open_control']
    if scope == 'open_control':
        return [row for row in sample if row['family'] == 'open_control']
    kind, value = scope.split(':', 1)
    field = {'family': 'family', 'straight_line': 'straight_line_collision', 'distance_bin': 'distance_bin'}[kind]
    value = {'True': True, 'False': False}.get(value, value)
    return [row for row in sample if row['family'] != 'open_control' and row[field] == value]


def check_group_counts(data):
    """Cheap arithmetic checks only; statistical intervals come from the audit."""
    groups = {}
    for group in data['groups']:
        key = group['roadmap_nodes'], group['scope'], group['label']
        require(key not in groups, 'Duplicate metric group')
        groups[key] = group
        sample = sample_for_scope(data['rows'], *key)
        legal = [row for row in sample if row['input_legal']]
        require(group['requested_jobs'] == len(sample) and group['legal_jobs'] == len(legal), 'Group denominator mismatch')
        require(group['invalid_jobs'] == len(sample)-len(legal), 'Invalid-input accounting mismatch')
        require(group['legal_status_counts'] == dict(Counter(row['status'] for row in legal)), 'Failure taxonomy mismatch')
        require(group['valid_success']['numerator'] == sum(row['success'] for row in legal)
                and group['valid_success']['denominator'] == len(legal), 'Main success numerator or denominator mismatch')
        require(group['legal_scene_clusters'] == len({row['cluster_id'] for row in legal}), 'Physical scene cluster count mismatch')
        if group['scope'] == 'open_control':
            require(group['valid_success']['ci95'] is None, 'A single control world cannot supply scene-generalization CI')
        for field, scopes in group['statistics'].items():
            for scope, subset in [('all_legal', legal), ('validated_success', [row for row in legal if row['success']])]:
                actual, saved = raw_stat([row[field] for row in subset]), scopes[scope]
                require(actual['count'] == saved['count'], f'{key}/{field} observed n mismatch')
                for statistic_name in ('mean', 'p50', 'p90', 'p95'):
                    a, b = actual[statistic_name], saved[statistic_name]
                    require((a is None and b is None) or (a is not None and b is not None and np.isclose(a, b, atol=1e-12, rtol=1e-12)),
                            f'{key}/{field}/{statistic_name} mismatch')
        require(group['complete_backend_timing_jobs'] == sum(row['backend_timing_complete'] for row in legal),
                'Complete backend timing count mismatch')
        for field, scopes in group['stage_seconds'].items():
            actual, saved = raw_stat([row['completed_stage_seconds'].get(field) for row in legal]), scopes['all_legal']
            require(actual['count'] == saved['count'], 'Completed-stage observed n mismatch')
            require((actual['mean'] is None and saved['mean'] is None) or
                    (actual['mean'] is not None and saved['mean'] is not None and np.isclose(actual['mean'], saved['mean'])),
                    'Completed-stage mean mismatch')
    for nodes in {row['roadmap_nodes'] for row in data['rows']}:
        scopes = {scope for size, scope, _ in groups if size == nodes}
        expected_scopes = {'obstacle_main', 'straight_line:True', 'straight_line:False'}
        expected_scopes.update('family:'+family for family in FAMILIES)
        expected_scopes.update('distance_bin:'+value for value in ('short', 'medium', 'long'))
        if any(row['roadmap_nodes'] == nodes and row['family'] == 'open_control' for row in data['rows']):
            expected_scopes.add('open_control')
        require(scopes == expected_scopes, 'Missing or extra prespecified reporting strata')
        for scope in scopes:
            for label in METHODS:
                require((nodes, scope, label) in groups, 'A report stratum is missing a method')
            for objective, labels in LABELS.items():
                pairs = [item for item in data['paired_comparisons'] if item['roadmap_nodes'] == nodes
                         and item['scope'] == scope and item['objective'] == objective]
                require({frozenset((item['method_a'], item['method_b'])) for item in pairs}
                        == {frozenset(pair) for pair in combinations(labels, 2)}, 'Missing same-objective paired comparison')
                for pair in pairs:
                    require('common_success_cold_latency_difference' in pair, 'Paired common-success time CI missing')
    return groups


def load_evidence(main_path, scaling_path, bundle_path, protocol_path, index_path):
    fingerprints = {}
    bundle, protocol, index = read(bundle_path), read(protocol_path), read(index_path)
    require(bundle.get('schema_version') == 1 and bundle.get('kind') == 'formal_baseline_audit_bundle', 'Unsupported audit bundle')
    require(bundle.get('complete') is True and bundle.get('passed') is True and not bundle.get('errors'),
            'The formal audit bundle is incomplete or failed')
    require(protocol['state'] == 'approved', 'Protocol is not approved')
    require(sha(protocol_path) == bundle['protocol_sha256'] == index['protocol_sha256'], 'Protocol hash mismatch')
    require(sha(index_path) == bundle['run_index_sha256'], 'Run index hash mismatch')
    require(bundle['source_code_commit'] == protocol['source_code_commit'], 'Audit used a different runtime')
    runs = {item['run']: item['run_manifest_sha256'] for item in index['blocks']}
    require(len(runs) == len(index['blocks']) == len(protocol['blocks']), 'Run-index cardinality mismatch')
    require(bundle['run_manifest_sha256'] == runs, 'Audit does not cover the frozen full matrix')
    source = protocol['source_code_commit']
    input_audit = checked_file(bundle['evidence']['input'], bundle_path.parent, fingerprints)
    require(input_audit.get('passed') is True and not input_audit.get('errors'), 'Input audit failed')
    require(bundle['evidence']['input']['sha256'] == protocol['input_audit']['sha256'], 'Input audit differs from frozen protocol')
    require(input_audit['manifest_sha256'] == protocol['manifest']['sha256'], 'Input audit manifest mismatch')
    accounting = protocol['dataset_accounting']
    require((input_audit['problem_count'], input_audit['valid_input_count'], input_audit['invalid_input_count'])
            == (accounting['registered'], accounting['legal'], accounting['input_generation_failures']), 'Input audit accounting mismatch')
    for kind, flag in [('integrity', 'integrity_passed'), ('independent', 'independent_passed')]:
        audit = checked_file(bundle['evidence'][kind], bundle_path.parent, fingerprints)
        require(audit.get('complete') is True and audit.get('passed') is True and audit.get(flag) is True
                and not audit.get('errors'), f'{kind} audit did not complete and pass')
        require(audit['protocol_sha256'] == bundle['protocol_sha256'] and audit['run_manifest_sha256'] == runs,
                f'{kind} audit does not bind every registered run')
        require(audit['covered_job_slots'] == protocol['execution']['total_distinct_job_slots'], 'Audit omitted registered failures')
        if kind == 'independent':
            require(audit.get('all_outcomes_reviewed') is True and set(audit['reviewed_methods']) == set(METHODS),
                    'Independent audit did not cover all methods and outcome classes')
    datasets = []
    for name, path in [('main', main_path), ('scaling', scaling_path)]:
        require(sha(path) == bundle['metrics_sha256'][name], f'{name} metrics are not bound to the passed audit bundle')
        fingerprints[str(path)] = sha(path)
        datasets.append(read(path))
    main, scaling = datasets
    size_ids = checked_file(protocol['size_selection']['problem_ids_file'], protocol_path.parent, fingerprints)
    require(len(set(size_ids)) == len(size_ids) == protocol['size_selection']['registered'], 'Fixed size-effect IDs differ')
    require(main.get('reporting_problem_ids') is None, 'Main metrics silently selected a reporting subset')
    require(set(scaling.get('reporting_problem_ids') or []) == set(size_ids), 'Scaling did not use the fixed 80-ID subset')
    main_runs = {item['run']: item['run_manifest_sha256'] for item in index['blocks'] if item['phase'] == 'main'}
    main_rows = require_matrix(main, protocol_rows(protocol, index, {'main'}), main_runs, source)
    scale_rows = require_matrix(scaling, protocol_rows(protocol, index, {'main', 'size_effect'}, set(size_ids)), runs, source)
    shared = set(main_rows) & set(scale_rows)
    require(shared and all(key[1] == 6000 for key in shared), 'Scaling shared rows are not the main 6000-node reference')
    for key in shared:
        require(main_rows[key] == scale_rows[key], '6000-node scaling data were rerun, substituted or changed')
    distinct = dict(main_rows)
    distinct.update(scale_rows)
    require(len(distinct) == protocol['execution']['total_distinct_job_slots'], 'Distinct matrix accounting mismatch')
    require(sum(row['input_legal'] for row in distinct.values()) == protocol['execution']['total_legal_planner_jobs'],
            'Legal planner-job accounting mismatch')
    require({row['roadmap_nodes'] for row in scaling['rows']} == {3000, 6000, 9000, 12000}, 'Missing size-effect node budget')
    require({row['roadmap_seed'] for row in scaling['rows']} == {0, 1, 2}, 'Missing roadmap seed')
    maps = {}
    for block in protocol['blocks'].values():
        for path, digest in block['roadmap_files'].items():
            if Path(path).name == 'metadata.json' and path not in maps:
                maps[path] = checked_file({'path': path, 'sha256': digest}, protocol_path.parent, fingerprints)
    for row in distinct.values():
        meta = maps[row['roadmap_metadata_path']]
        require((row['roadmap_nodes'], row['roadmap_seed'], row['actual_roadmap_nodes']) ==
                (meta['requested_nodes'], meta['seed'], meta['actual_nodes']), 'Requested/retained roadmap nodes mismatch')
    for path in (bundle_path, protocol_path, index_path, Path(__file__).resolve()):
        fingerprints[str(path)] = sha(path)
    return dict(main=main, scaling=scaling, main_groups=check_group_counts(main),
                scaling_groups=check_group_counts(scaling), protocol=protocol, index=index,
                input_audit=input_audit, maps=list(maps.values()), reused_rows=len(shared),
                distinct_jobs=len(distinct), fingerprints=fingerprints)


def percentile_text(stat):
    return ' / '.join(number(stat[key]) for key in ('p50', 'p90', 'p95'))


def objective_section(data, groups, nodes, scope, objective):
    labels, rows = LABELS[objective], []
    unit = 'rad' if objective == 'minimum_distance' else 's'
    rows += table(['方法', '有效成功/合法查询 [95% CI]', '成功/原生 DRM', '成功/严格 seed', '依赖限制', '进程异常'],
        [[label, rate(groups[nodes, scope, label]['valid_success']), rate(groups[nodes, scope, label]['conditional_native_drm']),
          rate(groups[nodes, scope, label]['conditional_validated_drm']),
          groups[nodes, scope, label]['legal_status_counts'].get('unsupported', 0),
          groups[nodes, scope, label]['legal_status_counts'].get('infrastructure_error', 0)] for label in labels])
    rows += table(['方法'] + list(FAILURES), [[label] + [groups[nodes, scope, label]['legal_status_counts'].get(key, 0)
                  for key in FAILURES] for label in labels])
    rows += table(['方法', '冷耗时 n', '冷 p50/p90/p95 (s)', 'warm p50 (s; n)', f'成功成本均值/p50 ({unit}; n)'],
        [[label, statistic(groups[nodes, scope, label], 'cold_e2e_seconds')['count'],
          percentile_text(statistic(groups[nodes, scope, label], 'cold_e2e_seconds')),
          f"{number(statistic(groups[nodes, scope, label], 'warm_query_seconds')['p50'])}; n={statistic(groups[nodes, scope, label], 'warm_query_seconds')['count']}",
          f"{number(statistic(groups[nodes, scope, label], 'cost', 'validated_success')['mean'])} / {number(statistic(groups[nodes, scope, label], 'cost', 'validated_success')['p50'])}; n={statistic(groups[nodes, scope, label], 'cost', 'validated_success')['count']}"] for label in labels])
    pairs = [pair for pair in data['paired_comparisons'] if pair['roadmap_nodes'] == nodes and pair['scope'] == scope and pair['objective'] == objective]
    rows += table(['同目标 A − B', '合法配对 n', '成功率差 pp [CI]', '共同成功 n', f'共同成功成本差 {unit} [CI]', '共同成功冷耗时差 s [CI]', '所有合法冷耗时差 s [CI]'],
        [[pair['method_a']+' − '+pair['method_b'], pair['paired_legal_jobs'], interval(pair['valid_success_difference'], 100, 2),
          int(pair['common_success_cost_difference']['denominator']), interval(pair['common_success_cost_difference']),
          interval(pair['common_success_cold_latency_difference']), interval(pair['observed_cold_latency_difference'])] for pair in pairs])
    return rows


def draw_figures(evidence, directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import PercentFormatter
    directory.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False})
    paths = []
    for objective, labels in LABELS.items():
        title = 'Minimum distance' if objective == 'minimum_distance' else 'Minimum time'
        for scope in ('obstacle_main', 'open_control'):
            fig, axes = plt.subplots(1, 3, figsize=(15.2, 4.8), gridspec_kw={'width_ratios': [1.3, 1.3, 1]})
            maximum = 60.
            for index, label in enumerate(labels):
                sample = [row for row in sample_for_scope(evidence['main']['rows'], 6000, scope, label) if row['input_legal']]
                times = sorted(row['cold_e2e_seconds'] for row in sample)
                success = sorted(row['cold_e2e_seconds'] for row in sample if row['success'])
                maximum = max(maximum, max(times))
                for ax, values in zip(axes[:2], (success, times)):
                    x = [.1] + values + [max(60., max(times))*1.03]
                    y = [0.] + (np.arange(1, len(values)+1)/len(sample)).tolist() + [len(values)/len(sample)]
                    ax.step(x, y, where='post', linewidth=1.8, color=COLORS[label], label=f'{label} ({len(success)}/{len(sample)})')
                estimate = evidence['main_groups'][6000, scope, label]['valid_success']
                value = estimate['estimate']
                bounds = estimate['ci95']
                axes[2].plot(value, index, 'o', color=COLORS[label])
                if bounds is not None:
                    axes[2].hlines(index, bounds[0], bounds[1], color=COLORS[label])
                    axes[2].vlines(bounds, index-.025, index+.025, color=COLORS[label])
                axes[2].annotate(f'{100*value:.1f}%', (value, index), xytext=(7, 8), textcoords='offset points', fontsize=9)
            for ax in axes[:2]:
                ax.set_xscale('log'); ax.set_xlim(.1, maximum*1.05); ax.set_ylim(0, 1.03)
                ax.yaxis.set_major_formatter(PercentFormatter(1)); ax.grid(alpha=.2)
                ax.set_xlabel('Observed cold wall time (s)')
            axes[0].set_title('A  Validated-output latency ECDF', loc='left')
            axes[0].legend(loc='best', frameon=False, fontsize=8)
            axes[0].set_ylabel('Fraction of all legal queries')
            axes[1].set_title('B  Termination latency, including failures', loc='left')
            axes[2].set_title('C  Numerical success', loc='left')
            axes[2].set_yticks(range(len(labels)), labels=labels); axes[2].invert_yaxis()
            axes[2].set_xlim(0, 1.08); axes[2].xaxis.set_major_formatter(PercentFormatter(1)); axes[2].grid(axis='x', alpha=.2)
            axes[2].set_xlabel('Success / all legal queries')
            fig.suptitle(f'{title} · {scope} · 6000-node main study', x=.055, ha='left', weight='semibold', fontsize=14)
            text = '95% scene-cluster bootstrap intervals; endpoint and roadmap repetitions remain in their scene clusters.' if scope == 'obstacle_main' else 'One physical control world: descriptive repeated queries; no scene-generalization confidence interval.'
            fig.text(.055, .035, 'One actual 60 s run per query; not shorter-budget reruns or first-feasible-iterate time. Failures stay in the denominator.\n'+text, fontsize=9)
            fig.subplots_adjust(left=.055, right=.97, top=.82, bottom=.24, wspace=.37)
            path = directory/f'{objective}_{scope}.png'; fig.savefig(path, dpi=180); plt.close(fig); paths.append(path)
        fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.7))
        sizes = [3000, 6000, 9000, 12000]
        for label in labels:
            groups = [evidence['scaling_groups'][size, 'obstacle_main', label] for size in sizes]
            rates = [group['valid_success']['estimate'] for group in groups]
            axes[0].plot(sizes, rates, 'o-', label=label, color=COLORS[label])
            for size, group in zip(sizes, groups):
                bounds = group['valid_success']['ci95']
                require(bounds is not None, 'Missing prespecified scene-cluster scaling interval')
                axes[0].vlines(size, bounds[0], bounds[1], color=COLORS[label])
                axes[0].hlines(bounds, size-70, size+70, color=COLORS[label])
            axes[1].plot(sizes, [statistic(group, 'cold_e2e_seconds')['p50'] for group in groups], 'o-', color=COLORS[label])
            axes[2].plot(sizes, [statistic(group, 'cost', 'validated_success')['mean'] for group in groups], 'o-', color=COLORS[label])
        for ax in axes:
            ax.set_xticks(sizes, ['3000', '6000', '9000', '12000']); ax.set_xlabel('Requested roadmap nodes'); ax.grid(alpha=.2)
        axes[0].set_ylim(0, 1.05); axes[0].yaxis.set_major_formatter(PercentFormatter(1)); axes[0].legend(frameon=False, fontsize=8)
        axes[0].set_title('A  Success / all legal queries, 95% CI', loc='left')
        axes[1].set_title('B  All-legal cold latency median (s)', loc='left')
        axes[2].set_title('C  Successful cost mean (descriptive)', loc='left')
        axes[2].set_ylabel('Joint path length (rad)' if objective == 'minimum_distance' else 'Trajectory duration (s)')
        fig.suptitle(f'{title} · prespecified fixed-ID roadmap-size study', x=.06, ha='left', fontsize=14, weight='semibold')
        fig.text(.06, .025, 'Same fixed scene/case IDs and seeds at all four sizes. The 6000-node observations are reused from the main study.\n'
                 'Cost curves have different successful subsets and do not establish a quality ranking. Invalid inputs remain in accounting.', fontsize=9)
        fig.subplots_adjust(left=.06, right=.98, top=.8, bottom=.25, wspace=.28)
        path = directory/f'{objective}_scaling.png'; fig.savefig(path, dpi=180); plt.close(fig); paths.append(path)
    return paths


def render(evidence, images, output, commands):
    main, scale = evidence['main'], evidence['scaling']
    groups, size_groups, protocol = evidence['main_groups'], evidence['scaling_groups'], evidence['protocol']
    accounting, size_selection = protocol['dataset_accounting'], protocol['size_selection']
    lines = ['# Kinova 通用规划：正式论文方法 baseline', '',
        f"本报告仅消费完整运行和已通过的输入、完整性及独立审计。冻结运行源码为 `{protocol['source_code_commit']}`。主评测 {main['requested_jobs']} 个登记槽位，规模分析 {scale['requested_jobs']} 条记录，其中 {evidence['reused_rows']} 条复用主评测；去重后为 {evidence['distinct_jobs']} 个登记槽位，其中 {protocol['execution']['total_legal_planner_jobs']} 个合法规划调用、{protocol['execution']['input_invalid_job_slots']} 个无效输入槽位。不能把两表直接相加。", '',
        '这是固定 Kinova 模型和公开环境上的论文方法适配；LGCS/NTO 含已公开标注的数学转写，不宣称重现作者私有 FR3/实机数据。有限采样验证不是连续无碰撞证明，DRM 失败也不证明全局不可行。', '',
        '## 输入与统计单位', '',
        f"冻结 test 保留 {accounting['registered']} 条记录：{accounting['legal']} 条合法输入进入算法分母，{accounting['input_generation_failures']} 条端点生成失败单独保留。障碍任务 {accounting['legal_obstacle_tasks']} 条来自 {accounting['legal_random_obstacle_worlds']} 个合法物理场景；控制任务 {accounting['legal_control_tasks']} 条仅来自 {accounting['physical_control_worlds']} 个无新增障碍物理世界。路网 seed 重复不是新的随机场景。", '',
        '成功必须满足输入、原生前端、独立 seed、优化器及最终共同数值验证门控。主率包含所有合法失败、超时、许可限制及基础设施错误；条件成功率只定位阶段，不能取代主率。置信区间来自预先指定的场景整组 bootstrap（2000 次、PCG64 seed 45827）。全成/全败可产生退化区间，不能证明总体概率为 1/0；单一控制世界不报告场景泛化 CI。', '']
    for scope, title in [('obstacle_main', '障碍主套件'), ('open_control', '无新增障碍对照')]:
        lines += [f'## {title}：6000 节点', '']
        for objective in LABELS:
            lines += [f"### {'最短距离' if objective == 'minimum_distance' else '最短时间'}", '']
            lines += objective_section(main, groups, 6000, scope, objective)
            image_paths = [path for path in images if path.name == f'{objective}_{scope}.png']
            if image_paths:
                lines += [f"![{objective} {scope}]({image_paths[0].relative_to(output.parent).as_posix()})", '']
    lines += ['冷耗时包含所有合法失败与进程启动、载入、规划、验证及输出；warm 是同次调用扣除初始化的阶段记录，不是独立热启动实验。有效输出 ECDF 将无有效输出的失败保留在无穷延迟处；终止 ECDF 则保留快速失败的实际终止时间。它们来自固定 60 s 预算记录，不是短预算重跑或首次可行迭代时间。', '',
              '成功成本均值仅描述各自成功子集，不能直接排名。成对差严格为 A−B，成本及共同成功耗时只在同题双方成功的子集比较；所有合法耗时差仍包含失败。许可失败反映当前部署支持范围，不足以断言算法质量较差。', '',
              '## 支持限制与后端阶段计时', '']
    for scope, title in [('obstacle_main', '障碍主套件'), ('open_control', '无新增障碍对照')]:
        lines += [f'### {title}', '']
        lines += table(['方法', '无观察到依赖/进程故障', '该子集成功', '该子集且 seed 准入的成功'],
            [[label, rate(groups[6000, scope, label]['dependency_or_infrastructure_resolved_rate']),
              rate(groups[6000, scope, label]['conditional_resolved_success']),
              rate(groups[6000, scope, label]['conditional_resolved_validated_drm'])] for label in METHODS])
        lines += table(['方法', '完整计时后端/合法查询']+[name+' s (n)' for _, name in BACKEND_STAGES],
            [[label, f"{groups[6000, scope, label]['complete_backend_timing_jobs']}/{groups[6000, scope, label]['legal_jobs']}"]+
             [mean_n(groups[6000, scope, label]['stage_seconds'][key]['all_legal']) if key in groups[6000, scope, label]['stage_seconds'] else '—'
              for key, _ in BACKEND_STAGES] for label in METHODS])
        lines += table(['方法', '区域均值 (n)', '面均值 (n)', '恢复均值 (n)', 'GPU 峰值 p95 MiB (n)', '成功 clearance 最小 mm (n)'],
            [[label, mean_n(statistic(groups[6000, scope, label], 'region_count')),
              mean_n(statistic(groups[6000, scope, label], 'facets')), mean_n(statistic(groups[6000, scope, label], 'repair_count')),
              f"{number(statistic(groups[6000, scope, label], 'sampled_gpu_peak_mib')['p95'])} (n={statistic(groups[6000, scope, label], 'sampled_gpu_peak_mib')['count']})",
              f"{number(None if statistic(groups[6000, scope, label], 'minimum_sampled_clearance_m', 'validated_success')['min'] is None else 1000*statistic(groups[6000, scope, label], 'minimum_sampled_clearance_m', 'validated_success')['min'])} (n={statistic(groups[6000, scope, label], 'minimum_sampled_clearance_m', 'validated_success')['count']})"] for label in METHODS])
    lines += ['后端阶段表仅纳入完整返回候选及最终验证、无中断异常的后端。正常返回的 solver_failed 仍可纳入；未完整返回的后端全部阶段排除，即便早期工作已完成。原始 timing 保留，不把初始化零值当作完成计时；完成后端的真实零恢复保留。条件阶段均值不能替代全体延迟，显存是采样峰值，clearance 是有限样本最小值。支持条件子集也存在规模选择效应。', '',
              '## 预先固定的障碍分层', '']
    scopes = [('family:'+family, family) for family in FAMILIES]
    scopes += [('straight_line:True', '直连采样碰撞'), ('straight_line:False', '直连采样无碰撞')]
    scopes += [('distance_bin:'+value, value) for value in ('short', 'medium', 'long')]
    for objective, labels in LABELS.items():
        lines += [f"### {'距离' if objective == 'minimum_distance' else '时间'}目标", '']
        lines += table(['固定分层', '合法物理场景数']+list(labels),
            [[title, groups[6000, scope, labels[0]]['legal_scene_clusters']]+
             [rate(groups[6000, scope, label]['valid_success']) for label in labels] for scope, title in scopes])
    lines += ['bottleneck/route_choice 是 workspace 几何类别，不代表已知 C-space 同伦。直连无碰撞指有限数值采样；分层场景数和样本数越少，其区间解释越受限制。', '',
              '## 路网规模：同一组固定 ID', '',
              f"按冻结规则取 {size_selection['registered']} 个 ID，其中 {size_selection['legal']} 合法、{size_selection['invalid']} 输入无效。四档节点请求预算使用相同 ID 与三个 seed，6000 档直接复用主评测记录；该档没有新规划调用。无效 ID 不补抽。", '']
    for objective, labels in LABELS.items():
        lines += [f"### {'距离' if objective == 'minimum_distance' else '时间'}目标规模结果", '']
        lines += table(['请求节点', '方法', '有效成功/合法 [CI]', '未支持', '合法冷 p50/p90/p95 s (n)', '成功成本均值 (n)'],
            [[size, label, rate(size_groups[size, 'obstacle_main', label]['valid_success']),
              size_groups[size, 'obstacle_main', label]['legal_status_counts'].get('unsupported', 0),
              percentile_text(statistic(size_groups[size, 'obstacle_main', label], 'cold_e2e_seconds'))+
              f" (n={statistic(size_groups[size, 'obstacle_main', label], 'cold_e2e_seconds')['count']})",
              mean_n(statistic(size_groups[size, 'obstacle_main', label], 'cost', 'validated_success'))]
             for size in (3000, 6000, 9000, 12000) for label in labels])
        paths = [path for path in images if path.name == f'{objective}_scaling.png']
        if paths:
            lines += [f"![{objective} scaling]({paths[0].relative_to(output.parent).as_posix()})", '']
    lines += ['规模曲线的成功成本仍来自不同成功子集，只作描述，不充当跨规模成对质量结论。地图表分别列出请求数与真实保留节点数；孤立点剔除不会另建规模组。离线构建与在线查询分别计时，复用已有地图不代表离线成本为零。', '']
    lines += table(['请求节点', 'seed', '实际节点', '有向边', '保存的离线构建 s'],
        [[item['requested_nodes'], item['seed'], item['actual_nodes'], item['directed_edges'], number(item['offline_seconds'])]
         for item in sorted(evidence['maps'], key=lambda item: (item['requested_nodes'], item['seed']))])
    lines += ['离线数值是原始构建记录，未重新当作控制硬件竞争的串行建图实验。', '', '## DRM-PWL 前端参考', '',
              '仅使用预先指定 LSCS 调用的严格准入 seed，不挑选五方法里最快或最优 seed。折线长度是距离前端参考，不是优化器输出，也没有 minimum-time 的物理时间参数化。组件时间之和不是独立冷启动时间。', '']
    for scope, title in [('obstacle_main', '障碍主套件'), ('open_control', '无新增障碍对照')]:
        sample = [row for row in main['drm_references'] if row['input_legal'] and row['drm_success']
                  and (row['family'] == 'open_control') == (scope == 'open_control')]
        lines += table([title, 'observed n', '均值', 'p50', 'p90', 'p95'],
            [[name, values['count']]+[number(values[key]) for key in ('mean', 'p50', 'p90', 'p95')]
             for name, values in [('seed 长度 rad', raw_stat([row['seed_cost_rad'] for row in sample])),
                                  ('前端组件之和 s', raw_stat([row['frontend_component_seconds'] for row in sample]))]])
    lines += ['前端组件含模型配置、GPU 预热、地图载入、碰撞缓存、DRM Plan 和 seed 验收；不含解释器、依赖探测及未归入组件的开销，不能据此计算相对完整算法的速度比。', '', '## 复现与证据绑定', '',
              '以下命令只在完整运行和独立审计完成后执行。渲染器不重新运行规划、碰撞检查、receipt 审计或 bootstrap；缺失/失败审计、矩阵缺项、重复计入 6000 档或指标未绑定均会拒绝输出。', '', '```bash']
    lines += commands + ['```', '']
    lines += table(['证据文件', 'SHA-256'], [[path, digest] for path, digest in evidence['fingerprints'].items()])
    return '\n'.join(lines)+'\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--main-metrics', type=Path, required=True)
    parser.add_argument('--scaling-metrics', type=Path, required=True)
    parser.add_argument('--audit-bundle', type=Path, required=True)
    parser.add_argument('--protocol', type=Path, default=HERE/'formal_protocol_v1.json')
    parser.add_argument('--run-index', type=Path, default=HERE/'formal_run_index.json')
    parser.add_argument('--output', type=Path, default=HERE.parent/'benchmark_v1_formal_baselines.md')
    parser.add_argument('--no-figures', action='store_true')
    args = parser.parse_args(argv)
    paths = [path.resolve() for path in (args.main_metrics, args.scaling_metrics, args.audit_bundle, args.protocol, args.run_index)]
    evidence = load_evidence(*paths)
    output = args.output.resolve()
    require(output.name != 'benchmark_v1_baseline_metrics.md', 'Refusing to overwrite the dev report')
    images = [] if args.no_figures else draw_figures(evidence, output.parent/'baseline_study/formal_figures')
    import shlex
    commands = ['bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_study.render_formal_report \\',
                '  --main-metrics '+shlex.quote(str(paths[0]))+' \\',
                '  --scaling-metrics '+shlex.quote(str(paths[1]))+' \\',
                '  --audit-bundle '+shlex.quote(str(paths[2]))]
    content = render(evidence, images, output, commands)
    require(all(sha(path) == digest for path, digest in evidence['fingerprints'].items()),
            'Evidence changed during rendering; refusing an inconsistent report')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(content)
    print(json.dumps({'report': str(output), 'distinct_job_slots': evidence['distinct_jobs'],
                      'reused_6000_observations': evidence['reused_rows'],
                      'figures': [str(path) for path in images]}, indent=2))


if __name__ == '__main__':
    main()
