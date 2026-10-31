"""Build a Chinese comparator report exclusively from complete audited evidence."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from benchmarks.generic_motion_v1.reports.paper_comparators_v1.audit_comparison import (
    PAPER, REFINED, LABELS, SUCCESS, read, sha, require, key, validate_matrix)

DISPLAY = {'NTO-distance': 'NTO-distance / SNOPT', 'NTO-time': 'NTO-time / IPOPT',
           'NTO-refined-distance': 'NTO-refined-distance / SNOPT',
           'NTO-refined-time': 'NTO-refined-time / IPOPT'}
FAMILIES = ('sparse_clutter', 'dense_clutter', 'bottleneck', 'route_choice')
PAIRS = (('minimum_distance', 'LSCS', 'LGCS'),
         ('minimum_distance', 'LSCS', 'NTO-distance'),
         ('minimum_distance', 'LGCS', 'NTO-distance'),
         ('minimum_time', 'SCSTO', 'NTO-time'))


def number(value, digits=3):
    if value is None:
        return '—'
    value = 0.0 if abs(value) < .5 * 10 ** (-digits) else value
    return f'{value:.{digits}f}'


def table(headers, rows):
    return '\n'.join(['| ' + ' | '.join(headers) + ' |',
                      '| ' + ' | '.join(['---'] * len(headers)) + ' |',
                      *['| ' + ' | '.join(map(str, row)) + ' |' for row in rows]])


def ratio(value):
    if value['estimate'] is None:
        return '—'
    ci = value['ci95']
    suffix = '；单一场景不估计 CI' if ci is None else f" [{100*ci[0]:.1f}, {100*ci[1]:.1f}]"
    return f"{100*value['estimate']:.1f}%" + suffix


def delta(value, scale=1.):
    if value['estimate'] is None:
        return '—'
    ci = value['ci95']
    suffix = '；无 CI' if ci is None else ' [' + ', '.join(number(x * scale) for x in ci) + ']'
    return number(value['estimate'] * scale) + suffix


def scope_rows(rows, group):
    pool = [r for r in rows if r['split'] == group['split'] and
            r['roadmap_nodes'] == group['roadmap_nodes'] and r['label'] == group['label']]
    scope = group['scope']
    if scope == 'obstacle_main':
        return [r for r in pool if r['family'] != 'open_control']
    if scope == 'open_control':
        return [r for r in pool if r['family'] == 'open_control']
    if scope.startswith('family:'):
        return [r for r in pool if r['family'] == scope.split(':', 1)[1]]
    if scope.startswith('straight_line:'):
        return [r for r in pool if r['family'] != 'open_control' and
                str(r['straight_line_collision']) == scope.split(':', 1)[1]]
    if scope.startswith('distance_bin:'):
        return [r for r in pool if r['family'] != 'open_control' and
                r['distance_bin'] == scope.split(':', 1)[1]]
    raise ValueError('Unknown report stratum: ' + scope)


def check_ratio(saved, rows, values):
    require(saved['denominator'] == len(values), 'Ratio denominator differs from actual observations')
    require(np.isclose(saved['numerator'], sum(values), atol=1e-10, rtol=1e-10), 'Ratio numerator differs')
    require((saved['estimate'] is None if not values else
             np.isclose(saved['estimate'], np.mean(values), atol=1e-10, rtol=1e-10)), 'Ratio estimate differs')
    clusters = len({r['cluster_id'] for r in rows})
    require(saved['independent_scene_clusters'] == clusters, 'Scene cluster count differs')
    require(saved['bootstrap_seed'] == 45827 and saved['bootstrap_replicates'] == 2000,
            'Bootstrap protocol differs')
    ci = saved['ci95']
    require(ci is None if clusters < 2 else ci is not None and len(ci) == 2 and
            np.isfinite(ci).all() and ci[0] <= ci[1], 'Malformed scene cluster confidence interval')


def check_statistics(saved, values):
    values = [v for v in values if v is not None]
    require(saved['count'] == len(values), 'Statistic available observation count differs')
    expected = {'mean': np.mean(values), 'p50': np.quantile(values, .5), 'p90': np.quantile(values, .9),
                'p95': np.quantile(values, .95)} if values else dict.fromkeys(('mean', 'p50', 'p90', 'p95'))
    for field, value in expected.items():
        require(saved[field] is None if value is None else
                saved[field] is not None and np.isclose(saved[field], value, rtol=1e-12, atol=1e-12),
                'Statistic differs from actual observations: ' + field)


def validate_metrics(metrics, audit, metrics_sha):
    require(audit.get('passed') is True and audit.get('errors') == [], 'A passing comparison audit is required')
    require(audit['metrics_sha256'] == metrics_sha, 'Audit binds a different metrics file')
    require((audit['registered_slots'], audit['new_registered_slots'], audit['reused_registered_slots'],
             audit['legal_slots'], audit['invalid_slots'], audit['new_legal_slots']) ==
            (1330, 380, 950, 1316, 14, 376), 'Audit denominator differs')
    require(metrics['complete'] is True and metrics['comparable'] is True and metrics['requested_jobs'] == 1330,
            'Incomplete comparison metrics')
    require(set(metrics['expected_method_labels']) == set(LABELS), 'Wrong method set')
    validate_matrix(metrics['rows'])
    groups = {}
    for group in metrics['groups']:
        identity = group['split'], group['scope'], group['label']
        require(identity not in groups, 'Duplicate metric group')
        groups[identity] = group
        rows = scope_rows(metrics['rows'], group)
        legal = [r for r in rows if r['input_legal']]
        success = [r for r in legal if r['success']]
        seed = [r for r in legal if r['drm_success'] is True]
        require(group['requested_jobs'] == len(rows) and group['legal_jobs'] == len(legal) and
                group['invalid_jobs'] == len(rows) - len(legal), 'Group denominator mismatch')
        require(group['status_counts'] == dict(Counter(r['status'] for r in rows)) and
                group['legal_status_counts'] == dict(Counter(r['status'] for r in legal)), 'Group status count mismatch')
        check_ratio(group['valid_success'], legal, [int(r['success']) for r in legal])
        check_ratio(group['conditional_validated_drm'], seed, [int(r['success']) for r in seed])
        for field in ('cold_e2e_seconds', 'warm_query_seconds'):
            check_statistics(group['statistics'][field]['all_legal'], [r[field] for r in legal])
        check_statistics(group['statistics']['cost']['validated_success'], [r['cost'] for r in success])
        for stage in ('eizo_inflation_seconds', 'trajectory_optimization_seconds', 'validation_seconds'):
            if stage in group['stage_seconds']:
                check_statistics(group['stage_seconds'][stage]['all_legal'],
                                 [r['completed_stage_seconds'].get(stage) for r in legal])
    for label in LABELS:
        for split, scope in (('dev', 'obstacle_main'), ('smoke', 'obstacle_main'), ('smoke', 'open_control')):
            require((split, scope, label) in groups, 'Missing mandatory report group')
        require(('dev', 'open_control', label) not in groups, 'Dev control was manufactured')
        for family in FAMILIES:
            require(('dev', 'family:' + family, label) in groups, 'Missing family report group')
        for blocked in (True, False):
            require(('dev', 'straight_line:' + str(blocked), label) in groups,
                    'Missing straight-line collision stratum')
    return groups


def paired(metrics, specification):
    objective, left, right = specification
    matches = [p for p in metrics['paired_comparisons'] if p['split'] == 'dev' and
               p['roadmap_nodes'] == 6000 and p['scope'] == 'obstacle_main' and p['objective'] == objective and
               {p['method_a'], p['method_b']} == {left, right}]
    require(len(matches) == 1, 'Missing or duplicate requested paired comparison')
    pair = matches[0]
    labels = (pair['method_a'], pair['method_b'])
    maps = [{key(r): r for r in metrics['rows'] if r['split'] == 'dev' and r['family'] != 'open_control'
             and r['label'] == label and r['input_legal']} for label in labels]
    require(set(maps[0]) == set(maps[1]) and len(maps[0]) == pair['paired_legal_jobs'] == 180,
            'Paired legal task denominator differs')
    matched = [(maps[0][k], maps[1][k]) for k in sorted(maps[0])]
    joint = [(a, b) for a, b in matched if a['success'] and b['success']]
    specs = [('valid_success_difference', matched, lambda a, b: int(a['success']) - int(b['success'])),
             ('common_success_cost_difference', joint, lambda a, b: a['cost'] - b['cost']),
             ('observed_cold_latency_difference', matched, lambda a, b: a['cold_e2e_seconds'] - b['cold_e2e_seconds'])]
    output = dict(method_a=left, method_b=right, paired_legal_jobs=len(matched), common_success_jobs=len(joint))
    sign = 1 if labels[0] == left else -1
    for field, pool, function in specs:
        source = pair[field]
        check_ratio(source, [a for a, _ in pool], [function(a, b) for a, b in pool])
        output[field] = dict(source, estimate=None if source['estimate'] is None else sign * source['estimate'],
                            ci95=None if source['ci95'] is None else source['ci95'] if sign == 1 else
                            [-source['ci95'][1], -source['ci95'][0]])
    return output


def result_rows(groups, labels, split='dev', scope='obstacle_main'):
    rows = []
    for label in labels:
        group = groups[split, scope, label]
        success = group['legal_status_counts'].get(SUCCESS, 0)
        cold = group['statistics']['cold_e2e_seconds']['all_legal']
        warm = group['statistics']['warm_query_seconds']['all_legal']
        cost = group['statistics']['cost']['validated_success']
        unit = 'rad' if group['objective'] == 'minimum_distance' else 's'
        rows.append([DISPLAY.get(label, label), f"{success}/{group['legal_jobs']}", ratio(group['valid_success']),
                     f"{number(cold['mean'])} / {number(cold['p50'])} / {number(cold['p90'])} / {number(cold['p95'])}",
                     f"{number(warm['mean'])} (n={warm['count']})",
                     f"{number(cost['mean'])} {unit} (n={cost['count']})"])
    return rows


def failure_rows(groups, labels):
    columns = ('frontend_failed', 'solver_failed', 'collision_failed', 'timeout',
               'validation_undetermined', 'unsupported', 'infrastructure_error')
    rows = []
    for label in labels:
        counts = groups['dev', 'obstacle_main', label]['legal_status_counts']
        other = {k: v for k, v in counts.items() if k not in (SUCCESS, *columns)}
        rows.append([DISPLAY.get(label, label), *[counts.get(c, 0) for c in columns],
                     '；'.join(f'{k}={v}' for k, v in sorted(other.items())) or '0'])
    return rows


def straight_line_rows(groups):
    """Preserve each stratum's legal denominator; do not infer C-space feasibility."""
    output = []
    for label in LABELS:
        cells = []
        counts = []
        for blocked in (True, False):
            group = groups['dev', 'straight_line:' + str(blocked), label]
            success = group['legal_status_counts'].get(SUCCESS, 0)
            require(0 <= success <= group['legal_jobs'], 'Invalid straight-line success denominator')
            cells.append(f"{success}/{group['legal_jobs']}")
            counts.append((success, group['legal_jobs']))
        main = groups['dev', 'obstacle_main', label]
        require(sum(n for _, n in counts) == main['legal_jobs'] and
                sum(n for n, _ in counts) == main['legal_status_counts'].get(SUCCESS, 0),
                'Straight-line strata do not partition the dev main comparison')
        output.append([DISPLAY.get(label, label), *cells])
    return output


def render(metrics, audit, metrics_path, audit_path, output, source_audit=None, replay_index=None):
    metrics_path, audit_path, output = map(Path, (metrics_path, audit_path, output))
    groups = validate_metrics(metrics, audit, sha(metrics_path))
    root = output.parent
    def link(path, text):
        import os
        return f'[{text}]({os.path.relpath(Path(path).resolve(), root.resolve())})'
    main = table(['方法', '合法成功/调用', '成功率及场景 95% CI',
                  '冷启动秒数：均值 / P50 / P90 / P95', '热查询均值秒数', '成功轨迹代价均值'], result_rows(groups, PAPER))
    failures = table(['方法', '前端失败', '求解失败', '碰撞失败', '超时', '验证未决', '依赖不支持',
                      '基础设施错误', '其他'], failure_rows(groups, PAPER))
    pairs = [paired(metrics, p) for p in PAIRS]
    pair_table = table(['A − B', '合法配对 / 共同成功', '成功率差 pp [95% CI]',
                        '共同成功代价差 [95% CI]', '全部合法冷延迟差秒 [95% CI]'],
        [[p['method_a'] + ' − ' + p['method_b'], f"{p['paired_legal_jobs']} / {p['common_success_jobs']}",
          delta(p['valid_success_difference'], 100), delta(p['common_success_cost_difference']),
          delta(p['observed_cold_latency_difference'])] for p in pairs])
    family_table = table(['工作空间障碍族', *PAPER], [[family, *[
        f"{groups['dev', 'family:'+family, label]['legal_status_counts'].get(SUCCESS, 0)}/"
        f"{groups['dev', 'family:'+family, label]['legal_jobs']}" for label in PAPER]] for family in FAMILIES])
    line_table = table(['方法', '直连发生碰撞：合法成功 / 合法调用',
                        '直连未检出碰撞：合法成功 / 合法调用'], straight_line_rows(groups))
    controls = table(['方法', 'smoke 障碍合法成功 / 合法调用', 'smoke 无新增障碍成功 / 合法调用'],
        [[label, *[f"{groups['smoke', scope, label]['legal_status_counts'].get(SUCCESS, 0)}/"
                    f"{groups['smoke', scope, label]['legal_jobs']}" for scope in ('obstacle_main', 'open_control')]]
         for label in PAPER])
    refined = table(['工程扩展', '合法成功/调用', '成功率及场景 95% CI',
                     '冷启动秒数：均值 / P50 / P90 / P95', '热查询均值秒数', '成功轨迹代价均值'], result_rows(groups, REFINED))
    stage_rows = []
    for label in PAPER:
        stage_cells = []
        stages = groups['dev', 'obstacle_main', label]['stage_seconds']
        for stage in ('eizo_inflation_seconds', 'trajectory_optimization_seconds', 'validation_seconds'):
            saved = stages.get(stage, {}).get('all_legal')
            stage_cells.append('—' if saved is None else f"{number(saved['mean'])} (n={saved['count']})")
        stage_rows.append([label, *stage_cells])
    stage_table = table(['方法', 'EI-ZO 膨胀均值秒数', '轨迹优化累计均值秒数', '数值验证累计均值秒数'], stage_rows)
    metadata_paths = {r['roadmap_seed']: Path(r['roadmap_metadata_path']) for r in metrics['rows']}
    offline = []
    for seed, path in sorted(metadata_paths.items()):
        saved = read(path)
        require(saved['seed'] == seed and saved['requested_nodes'] == 6000, 'Offline roadmap identity mismatch')
        offline.append([seed, saved['actual_nodes'], number(saved['offline_seconds']), link(path, '构建证据')])
    sources = table(['来源', '原始注册槽位', '源提交', 'run manifest SHA256'], [
        [Path(path).name, entry['planned_jobs'], entry['source_commit'], entry['manifest_sha256']]
        for path, entry in sorted(audit['source_runs'].items())])
    profiles = audit['original_nto_saved_profiles']
    freeze = read(metrics_path.parent.parent / 'execution_freeze.json')
    require(sha(metrics_path.parent.parent / 'execution_freeze.json') == audit['freeze_sha256'],
            'Execution freeze changed after comparison audit')
    environment = freeze['environment']
    packages = environment['packages']
    native_exceptions = audit['original_nto_dev_native_exceptions']
    native_failure_rows = []
    for label in ('NTO-distance', 'NTO-time'):
        for record in audit['original_nto_dev_native_outcomes'].get(label, []):
            if record['terminal_status'] == SUCCESS:
                continue
            native_failure_rows.append([label, record['terminal_status'], record['count'],
                record['solver_status'], record['info'] if record['info'] is not None else
                record['native_status'] if record['native_status'] is not None else '—'])
    output_lines = [
        '# 论文比较方法：Kinova 通用规划基准实际测试\n',
        '论文 VII-A 的完整规划比较组已经在同一套固定 Kinova 题目上对齐：'
        '**最短距离为 LSCS、LGCS、NTO-SNOPT，最短时间为 SCSTO、NTO-IPOPT**。'
        '本次新增运行原始 NTO 的两个目标，使用相同代码快照下已封存的 LSCS、LGCS、SCSTO 结果进行配对比较。\n',
        '**工作量口径：380 个新注册槽位（376 次合法输入调用、4 条非法输入记录）＋950 个复用槽位'
        '＝1330 条记录。后者包括单独呈现的 refined 工程扩展。**'
        '这不表示本次新执行了 1330 次规划。全部记录为 smoke/dev；本轮没有调用正式 test。\n',
        '## 固定实验口径\n',
        '开发集是 60 个预先冻结问题、20 个独立障碍场景，四族各 15 题；'
        '每题使用 6000 节点路网的 0、1、2 三个种子，故每种方法 180 个合法调用。'
        'smoke 另有 10 条固定记录，其中 8 题合法、2 条生成失败。'
        '同一任务的模型、障碍球、端点、预算、DRM 路径 SHA 与前端结果逐项相同。'
        '所有方法共用 60 秒外部冷启动墙钟预算，保留失败和超时。\n',
        '环境保持现有 Ubuntu 22.04 / Conda；GPU 记录为 '
        f"`{environment['gpu']}`，Drake {packages['drake']}、CSDecomp {packages['csdecomp']}、"
        f"SCSPlanning {packages['scsplanning']}。"
        'LSCS 调用作者公开的凸优化实现（本机 Clarabel，论文 MOSEK），LGCS 使用相同透视图模型（本机 SCIP，论文 Gurobi），'
        'SCSTO 调用作者公开的交替凸优化实现；'
        'NTO 距离目标使用 SNOPT 二阶 B 样条，时间目标使用 IPOPT 四阶 B 样条。'
        'NTO 原始组固定零距离下界、149 个内部 k/150 碰撞绑定和单次求解；'
        '没有借用 refined 的 1 mm 裕度、增点或追加求解。'
        f'已保存参数的 {profiles} 次原始 NTO 构造均通过逐项审计；其余记录在求解参数保存前退出，仍保留在分母。\n',
        '## 原论文五种方法：开发集主结果\n', main + '\n',
        '冷启动统计覆盖全部合法调用，包含前端失败、求解失败与超时；热查询统计覆盖有实际计时的合法调用。'
        '轨迹代价只针对经过数值验证的成功输出：距离为关节空间 PWL 弧长（rad），时间为满足固定速度/加速度约束的轨迹时长（s）。'
        '不能把两个不同目标的代价直接比较，也不能仅按各自成功集合的均值排名。\n',
        '成功率 CI 按场景聚类 bootstrap（PCG64、2000 次、seed=45827），'
        '同一场景的多个端点与三个路网种子一起重采样；180 次调用不是 180 个独立场景。'
        '冷延迟的 P50/P90/P95 描述已观测调用分布，未把重复路网种子当成独立环境来估计不确定性。\n',
        failures + '\n',
        table(['原始 NTO', '最终分类', '次数', '原生求解返回状态', 'SNOPT info / IPOPT status'],
              native_failure_rows) + '\n' if native_failure_rows else '',
        '上表只列本轮已写回的原始求解器失败/未通过验证记录；未写回原生状态的超时仍计入前表。'
        '`collision_failed` 指输出被独立碰撞验证拒绝，不等同于求解器报告失败；'
        '原生求解器的不可行状态也不能证明整个运动规划任务无解。'
        f'原始组本轮有 {len(native_exceptions)} 条原生求解器异常返回，按实际最终分类保留，'
        '详细 error 与 traceback 位于对应 `nto_solver_result.json`，不会套用 refined 组的异常统计。'
        f"另有 {audit['original_snopt_native_log_limits_checked']} 份原生 SNOPT 日志明确打印总迭代 10000、"
        '主迭代 2000、次迭代 500，与原始组默认参数一致。\n',
        '## 同一任务的配对比较\n', pair_table + '\n',
        '所有差值方向为 A − B。成功率和冷延迟差以全部 180 个合法配对为分母；'
        '代价差仅限双方共同成功，距离单位 rad，时间单位 s。共同成功是条件子集，不能外推为全任务最优性结论。\n',
        line_table + '\n',
        '直连标签在任何方法运行前固定。上表把原始五方法与单独命名的 refined 扩展按同一标签分层，'
        '便于判断成功轨迹代价究竟覆盖了哪些任务。若某方法主要在直连未检出碰撞的子集成功，'
        '其较低的成功条件代价不能解释为在需要避障的全部任务上更优。'
        '直连发生碰撞只否定该条直连路径；未证明任务无解，也没有为任一方法重新选择端点。\n',
        '## 障碍族与独立无障碍对照\n', family_table + '\n', controls + '\n',
        '开发集没有 open_control，因此不伪造开发集无障碍统计。smoke 的两条 open_control '
        '共享同一个基础世界，单独展示，不加入障碍主套件，也不报告基于多个独立场景的 CI。'
        'bottleneck 和 route_choice 是工作空间几何类别；未证明 C-space 同伦类别或随机题可行性。\n',
        '## 工程增强 NTO：单独结果\n', refined + '\n',
        'refined 使用预先固定的 1 mm 裕度、最多四次原生求解与最多 600 个碰撞绑定，'
        '仍在同一 60 秒总预算内。这两行来自上一轮封存结果；主表中的原始 NTO 没有被它们替换。\n',
        '## 离线成本与证据\n',
        stage_table + '\n',
        '阶段均值只计入整个后端正常返回候选和最终验证、且完整写回计时的记录；正常返回的求解失败保留。'
        '超时、异常或中断的后端累计值不进入阶段均值，也不补零；每格明确可用 n。'
        '同一调用若发生恢复重试，优化和验证计时累加各轮耗时；n 是符合统计条件的调用数。'
        '因此这些有条件的阶段均值不能相加后冒充全任务冷启动耗时。原始未完成计时仍完整保存。\n',
        table(['路网种子', '实际节点数', '原始离线构建秒数', '记录'], offline) + '\n',
        '路网离线成本单列，未摊入在线冷/热延迟，也没有从在线冷启动中删除模型建立、路网加载、GPU 初始化或验证。'
        '冷启动来自外部进程实际墙钟记录；超时不是零耗时。复用结果的测量发生在上一轮，'
        '新 NTO 结果在本轮测量；源代码、配置和机器一致，仍可能存在运行时系统负载变化。\n',
        sources + '\n',
        f"运行代码 SHA 清单指纹：`{audit['source_hash_fingerprint']}`。不同 Git 提交的新增内容仅为报告/证据时，"
        '通过逐文件运行源码与配置散列相等判断可比较性。\n',
        '- ' + link(metrics_path, '全部逐题指标与分组结果') + '；' + link(metrics_path.with_name('observations.csv'), '逐调用 CSV') + '\n'
        '- ' + link(audit_path, '本轮证据与原始 NTO 参数审计') + '\n'
        '- ' + link(metrics_path.parent.parent / 'commands.jsonl', '实际命令、开始/结束时间与退出状态') + '\n'
        '- ' + link(metrics_path.parent.parent / 'execution_freeze.json', '执行前配置、环境和散列冻结') + '\n',
    ]
    if source_audit:
        require(Path(source_audit).is_file(), 'Requested paper source audit is missing')
        output_lines.append('- ' + link(source_audit, '论文对比算法及来源核查') + '\n')
    if replay_index:
        require(Path(replay_index).is_file(), 'Requested replay index is missing')
        output_lines.append('- ' + link(replay_index, '原始轨迹交互回放') + '\n')
    output_lines += [
        '\n只读复核与重新生成报告（输出写入新的临时目录，不覆盖已封存证据）：\n',
        '```bash\ncomparison_check_dir=$(mktemp -d /tmp/csdecomp-comparison-check.XXXXXX)\n'
        'bash local/python.sh -m benchmarks.generic_motion_v1.reports.paper_comparators_v1.audit_comparison \\\n'
        '  --metrics benchmarks/generic_motion_v1/reports/paper_comparators_v1/comparison_metrics/metrics.json \\\n'
        '  --output "$comparison_check_dir/comparison_audit.json"\n'
        'bash local/python.sh -m benchmarks.generic_motion_v1.reports.paper_comparators_v1.build_report \\\n'
        '  --metrics benchmarks/generic_motion_v1/reports/paper_comparators_v1/comparison_metrics/metrics.json \\\n'
        '  --audit "$comparison_check_dir/comparison_audit.json" \\\n'
        '  --output "$comparison_check_dir/benchmark_v1_paper_comparators.md"\n```\n',
        '重新测量完整五方法 smoke（写入新的结果目录，不覆盖历史结果）：\n',
        '```bash\ncomparison_run_dir=$(mktemp -d /tmp/csdecomp-comparison-run.XXXXXX)\n'
        'bash local/python.sh -m benchmarks.generic_motion_v1 batch \\\n'
        '  --manifest benchmarks/generic_motion_v1/manifests/smoke.jsonl \\\n'
        '  --roadmaps benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s000 \\\n'
        '  --config benchmarks/generic_motion_v1/configs/rss2025_reliable.json \\\n'
        '  --methods EI_ZO LGCS NTO_PAPER --objectives minimum_distance minimum_time \\\n'
        '  --timeout 60 --output "$comparison_run_dir/results"\n```\n',
        '## 结果适用范围\n',
        '算法目标、原生求解器和论文中的比较关系已经落实到固定的 Kinova benchmark。'
        '模型是冻结的七自由度 Kinova，而论文仿真机器人为 Franka/FR3；本报告的数值直接回答 Kinova 主套件上的比较结果。'
        '论文没有公开 NTO 完整实现，149 个内部约束点依据归档作者 TeX，初始化端点复制与求解器限额均由本地固定协议明确记录。'
        '最终成功要求独立有限采样验证通过；有限采样不构成连续无碰撞数学证明。'
        '前端失败任务的全局可行性仍未知，数值求解失败也不能作为无解证明。\n',
        '论文与代码来源：[RSS 2025 原文](https://www.roboticsproceedings.org/rss21/p045.html)、'
        '[CSDecomp 作者仓库](https://github.com/wernerpe/csdecomp)、'
        '[SCSPlanning 作者实现](https://github.com/TobiaMarcucci/scsplanning)。\n',
    ]
    return '\n'.join(output_lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metrics', type=Path, required=True)
    parser.add_argument('--audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source-audit', type=Path)
    parser.add_argument('--replay-index', type=Path)
    parser.add_argument('--receipt', type=Path,
                        help='Default: next to the audit, report_receipt.json')
    args = parser.parse_args()
    text = render(read(args.metrics), read(args.audit), args.metrics, args.audit, args.output,
                  args.source_audit, args.replay_index)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text)
    receipt_path = args.receipt or args.audit.parent / 'report_receipt.json'
    inputs = {str(path.resolve()): sha(path) for path in
              [args.metrics, args.audit, *([args.source_audit] if args.source_audit else []),
               *([args.replay_index] if args.replay_index else [])]}
    source_files = [Path(__file__).resolve(), Path(__file__).with_name('audit_comparison.py').resolve()]
    receipt = dict(schema_version=1, report_path=str(args.output.resolve()), report_sha256=sha(args.output),
                   metrics_path=str(args.metrics.resolve()), metrics_sha256=sha(args.metrics),
                   audit_path=str(args.audit.resolve()), audit_sha256=sha(args.audit), inputs=inputs,
                   source_files={str(path): sha(path) for path in source_files})
    receipt_path.parent.mkdir(parents=True, exist_ok=True)
    receipt_path.write_text(json.dumps(receipt, indent=2, ensure_ascii=False) + '\n')
    print(f'Wrote audited comparator report: {args.output}')


if __name__ == '__main__':
    main()
