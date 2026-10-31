"""Read-only comparison of sealed development studies; never invokes planners."""
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import statistics
import subprocess

HERE = Path(__file__).resolve().parent
REPORTS = HERE.parent
ROOT = REPORTS.parents[2]


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def mean(values):
    values = list(values)
    return statistics.mean(values) if values else None


def table(headers, rows):
    return '\n'.join(['| ' + ' | '.join(headers) + ' |',
                      '| ' + ' | '.join(['---'] * len(headers)) + ' |'] +
                     ['| ' + ' | '.join(str(x) for x in row) + ' |' for row in rows])


def key(row):
    return row['case_id'], row['roadmap_id'], row['method_seed']


def main(output):
    output = Path(output)
    if any((output / name).exists() for name in ('comparison.json', 'comparison.md', 'receipt.json')):
        raise FileExistsError('Choose a fresh output directory; preserve prior analysis')
    inputs = {
        'extended_metrics': REPORTS / 'extended_baselines_v1/metrics/metrics.json',
        'paper_metrics': REPORTS / 'paper_comparators_v1/comparison_metrics/metrics.json',
        'frontend_review': HERE / 'rrt_frontend_review.json',
        'compatibility_review': HERE / 'paper_compatibility_review.json',
    }
    sources = {name: dict(path=str(path), sha256=sha(path)) for name, path in inputs.items()}
    ext, old = read(inputs['extended_metrics']), read(inputs['paper_metrics'])
    front, compatible = read(inputs['frontend_review']), read(inputs['compatibility_review'])
    assert all(m['complete'] and m['comparable'] for m in (ext, old))
    assert compatible['environment_equal'] and compatible['common_evaluation_config_equal']
    selections = [(ext, label, '本轮扩展比较') for label in ('RRTConnect', 'RRT*', 'LSCS', 'CHOMP', 'STOMP', 'TrajOpt')]
    selections += [(old, label, '前轮论文比较') for label in ('LGCS', 'NTO-distance', 'NTO-refined-distance',
                                                            'SCSTO', 'NTO-time', 'NTO-refined-time')]
    methods, pools = [], {}
    for source, label, period in selections:
        rows = [r for r in source['rows'] if r['split'] == 'dev' and r['label'] == label]
        assert len(rows) == 180 and all(r['input_legal'] and r['attempt_sealed'] for r in rows)
        assert len({r['case_id'] for r in rows}) == 60 and len({r['cluster_id'] for r in rows}) == 20
        successes = [r for r in rows if r['success']]
        seed = [r for r in rows if r['drm_success'] is True]
        method = dict(label=label, source_period=period, objective=rows[0]['objective'],
                      attempts=180, successes=len(successes), status_counts=dict(Counter(r['status'] for r in rows)),
                      cold_all_mean_s=mean(r['cold_e2e_seconds'] for r in rows),
                      validated_cost_mean=mean(r['cost'] for r in successes),
                      valid_seed_attempts=len(seed) if label not in ('RRTConnect', 'RRT*') else None,
                      conditional_seed_success=sum(r['success'] for r in seed) if seed else None)
        methods.append(method); pools[label] = {key(r): r for r in rows}
    reference = pools['RRTConnect']
    paired = []
    for method in methods:
        label = method['label']
        if label == 'RRTConnect' or method['objective'] != 'minimum_distance':
            continue
        other = pools[label]
        assert set(reference) == set(other)
        for k, row in reference.items():
            assert all(row[f] == other[k][f] for f in ('input_sha256', 'robot_sha256', 'geometry_sha256',
                       'q_start', 'q_goal', 'cluster_id', 'budget_seconds'))
        joint = [k for k in sorted(reference) if reference[k]['success'] and other[k]['success']]
        paired.append(dict(other=label, source_period=method['source_period'], common_success=len(joint),
            rrt_length_mean_rad=mean(reference[k]['cost'] for k in joint),
            other_length_mean_rad=mean(other[k]['cost'] for k in joint),
            rrt_minus_other_mean_rad=mean(reference[k]['cost'] - other[k]['cost'] for k in joint),
            mean_per_task_length_ratio=mean(reference[k]['cost'] / other[k]['cost'] for k in joint),
            rrt_cold_mean_s=mean(reference[k]['cold_e2e_seconds'] for k in joint),
            other_cold_mean_s=mean(other[k]['cold_e2e_seconds'] for k in joint)))
    frozen_pair = next(p for p in ext['paired_comparisons'] if p['split'] == 'dev' and p['scope'] == 'all'
                       and p['method_a'] == 'LSCS' and p['method_b'] == 'RRTConnect')
    summary = dict(created_utc=datetime.now(timezone.utc).isoformat(), analysis_only=True, new_planner_calls=0,
        sources=sources, methods=methods, paired_quality=paired, frozen_lscs_minus_rrt_pair=frozen_pair,
        case_count=60, scene_count=20, paired_repetitions_per_method=180,
        scope='Only dev. Historical methods are explicitly marked; this is not one new same-runtime experiment.',
        uncertainty='Use existing frozen paired CI for fresh LSCS/RRT. Costs condition on joint success; all-call latency includes failures.',
        backend_scope='RRTConnect/RRT* provide geometric paths, not dynamically constrained minimum-time trajectories.')
    distance = [m for m in methods if m['objective'] == 'minimum_distance']
    times = [m for m in methods if m['objective'] == 'minimum_time']
    shared = next(p for p in paired if p['other'] == 'LSCS')
    simplified = front['simplification']['common_success']
    f = frozen_pair['common_success_cost_difference']
    ci = [-f['ci95'][1], -f['ci95'][0]]
    lines = [
        '# RRTConnect 与论文方法的同题比较', '',
        '**RRTConnect 的高成功率在当前冻结 Kinova 开发集上成立；与 LSCS 相比多出的 36 次成功全部发生在 DRM 前端失败的题目上。** '
        '在二者共同成功的 110 次调用中，RRTConnect 简化后的路径均长接近 LSCS，端到端冷延迟更低。'
        '这些是已有真实记录的重新分析，本次新增规划调用为 0，未运行正式 test。', '',
        '## 口径与跨轮可比性', '',
        '60 道固定题、20 个独立障碍场景、每题三次重复，每方法 180 次合法调用。模型、障碍、端点、'
        '公共数值验证、共有配置及外部 60 秒上限相同；97 个共享数据文件、15 个地图文件、1780 个共享依赖文件的冻结 SHA 相同。'
        '扩展比较与前轮论文比较的运行源码不同，因此主表明确标注来源，跨轮数字属于历史描述性比较。'
        '时延的直接结论优先采用本轮重新测量的 LSCS 对照。', '',
        '新旧 LSCS 的 180 次状态与 DRM 路径 SHA 相同，110 次成功中有 109 次代价完全相同；另一次相差 '
        '0.006007 rad，恢复后的区域面数也不同，保留为未隔离的跨运行数值变化。不能宣称两轮逐位完全一致。', '',
        '## 几何路径主表', '',
        table(['方法', '来源', '成功/180', '成功率', '全调用冷延迟均值 s', '成功输出长度均值 rad'],
              [[m['label'], m['source_period'], f"{m['successes']}/180", f"{m['successes']/1.8:.1f}%",
                f"{m['cold_all_mean_s']:.3f}", f"{m['validated_cost_mean']:.3f}"] for m in distance]), '',
        '全调用冷延迟包括失败；成功输出长度的样本集合各不相同，不能直接用最后一列作质量排名。'
        '冷延迟不含离线 DRM 构建，离线成本另计；RRTConnect/RRT* 不使用离线地图。'
        'NTO-refined 是单独登记的工程增强，不替换原始 NTO。CHOMP/STOMP/TrajOpt 的原生目标含平滑与碰撞项，公共质量统一用已验证的路径弧长。', '',
        '## 成功率差来自哪一层', '',
        table(['LSCS 与 RRTConnect 的配对结果', '次数'], [['共同成功', front['success_intersection']],
              ['仅 RRTConnect 成功', front['rrt_only']], ['仅 LSCS 成功', front['lscs_only']], ['均未成功', front['neither']]]), '',
        'LSCS 的 70 次失败都在 EI-ZO 之前：50 次原生报告当前 roadmap 未找到无碰撞路径，19 次尝试耗尽返回 False，'
        '1 次原生路径被独立验证拒绝；RRTConnect 分别成功 28、7、1 次。19 次 False 中仅 4 次日志明确含端点 steering 失败，'
        '不能把全部失败统称端点连接失败。已有合法 DRM 种子的 110 次中，EI-ZO→LSCS 全部通过终检。', '',
        table(['障碍族', 'LSCS', 'RRTConnect', '额外成功'], [[family, f"{v['lscs_success']}/{v['n']}",
               f"{v['rrt_success']}/{v['n']}", v['rrt_only']] for family, v in front['family'].items()]), '',
        '记录支持“当前有限 DRM 前端限制了覆盖，独立双向树绕过了其中部分失败”。尚未做节点数、端点连接或预算消融，'
        '不能进一步断言只改其中某个参数就会解决全部失败。roadmap 中无路径也不等于原问题无解。', '',
        '## 共同成功题：质量和速度', '',
        table(['与 RRTConnect 配对的方法', '共同成功数', 'RRT 长度 rad', '对方长度 rad', '平均逐题 RRT/对方长度比',
               'RRT 冷延迟 s', '对方冷延迟 s'], [[p['other'], p['common_success'], f"{p['rrt_length_mean_rad']:.3f}",
               f"{p['other_length_mean_rad']:.3f}", f"{p['mean_per_task_length_ratio']:.3f}",
               f"{p['rrt_cold_mean_s']:.3f}", f"{p['other_cold_mean_s']:.3f}"] for p in paired]), '',
        '每行条件子集不同。逐题长度比的均值与两个均长的比值不是同一个统计量。跨轮时延仅作描述，'
        '不当作随机交错、同源码实验的因果效应。', '',
        f"本轮共同 110 次，RRT−LSCS 平均路径长度差 {shared['rrt_minus_other_mean_rad']:.4f} rad，"
        f"沿用封存的场景聚类 95% CI [{ci[0]:.4f}, {ci[1]:.4f}]，没有清晰的长度优势。"
        f"冷启动均时为 {shared['rrt_cold_mean_s']:.4f} s 对 {shared['other_cold_mean_s']:.4f} s。"
        '全 180 次时 LSCS 均时更低，是另一种统计口径，不能解释为同题成功时更快；其中许多前端失败约 0.8–1.0 s 就结束。', '',
        'RRTConnect 没有在长度上全面胜过论文方法：同样 110 次上 LGCS 均长约 6.226 rad，RRT 约 6.309 rad；'
        '与 refined NTO 的共同 87 次上，RRT 的逐题长度比平均约 1.111。原始 NTO-distance 的 35 次成功全部来自'
        '直连未检出碰撞的子集，在 141 次直连受阻调用中成功数为 0，低成功条件均长具有强选择效应。', '',
        '## 为什么当前 RRTConnect 也有不错的路径质量', '',
        '这里实际执行的是官方 OMPL 双向 RRTConnect＋同一个 CSDecomp CUDA 碰撞回调＋官方顶点简化。'
        '它在线从固定起终点扩展两棵树，无须先在既有 DRM 中找到路径；当前结果也包括简化效果。'
        '[官方算法说明](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTConnect.html)。', '',
        f"共同 110 次，原始 RRT 均长 {simplified['raw_cost']['mean']:.3f} → 简化后 {simplified['simplified_cost']['mean']:.3f} rad；"
        f"DRM 初始路径 {front['drm_seed_cost_common_success']['mean']:.3f} → LSCS {shared['other_length_mean_rad']:.3f} rad。"
        '因此不应把简化后的结果描述成未经处理的原始 RRT 树路径。', '',
        '同一成功子集内，RRT 原生搜索约 0.050 s、简化约 0.064 s；论文管线的 DRM 约 0.056 s、EI-ZO 约 0.240 s、'
        'LSCS 优化约 0.0017 s。另外，论文管线还对 DRM 种子做了约 0.716 s 的终检；最终终检约 0.884 s，'
        'RRT 最终终检约 0.630 s。冷延迟差同时包含搜索、走廊构造和验证工作，不能全部归因于优化器算得慢。', '',
        'OMPL 内部搜索上限为 10 s，简化上限 0.5 s；DRM 在固定尝试耗尽时终止。共同外部上限 60 s 并不代表各方法'
        '都实际搜索 60 s。RRT* 在同一内部限额下还做代价优化与重连；当前 120/180 的结果只描述这个有限预算配置。', '',
        '公平性检查未发现放宽碰撞模型：起终点和模型/几何 SHA 相同，OMPL 调用同一个 Model.cuda；规划边采样 0.01 rad，'
        '公共终检 0.005 rad 且使用 CUDA 与独立 Drake。RRT 的第 147 个候选落入 ±2 µm 接触带，被判未决，没有计成功。'
        '有限采样验证不等于连续无碰撞证明。', '',
        '## 时间轨单独比较', '',
        table(['方法', '成功/180', '全调用冷延迟 s', '成功轨迹时长均值 s'], [[m['label'],f"{m['successes']}/180",
               f"{m['cold_all_mean_s']:.3f}",f"{m['validated_cost_mean']:.3f}"] for m in times]), '',
        'SCSTO 与原始 NTO-time 共同成功 88 次时，轨迹时长均值为 5.513 s 与 6.816 s；与 refined-time 共同成功 '
        '100 次时为 5.616 s 与 6.598 s。这里评价满足固定速度、加速度和静止端点约束的连续轨迹。'
        'RRTConnect 当前只输出几何路径，不能据此宣布它在最短时间目标上胜过 SCSTO。', '',
        '## 与论文结论的关系', '',
        '原论文 Table I 将 DRM 成功率单列，后端统计以 DRM 已找到路径为条件；作者实际比较 NTO、LSCS、LGCS、SCSTO，'
        '没有在 Table I 实验比较 RRTConnect。原文的优势陈述针对 NTO，不能解释成已胜过所有采样规划器。'
        '[RSS 正式论文第 10 页](https://www.roboticsproceedings.org/rss21/p045.pdf)。', '',
        '当前应保留 RRTConnect＋原生简化作为强端到端基线。研究 EI-ZO/凸优化后端时，应同时报告公共初始路径下的条件表现，'
        '把前端覆盖与后端质量分开；后续如要归因具体改动，需要预先固定题目与预算的前端替换/预算消融。此报告不启动这些新实验。', '',
        '## 可复核证据', '',
        '- [逐题前端诊断](rrt_frontend_review.json)',
        '- [跨轮模型、配置、依赖与原始数据兼容性](paper_compatibility_review.json)',
        '- [本报告计算值](comparison.json)',
        '- [扩展基线完整报告](../benchmark_v1_extended_baselines.md)',
        '- [论文方法完整报告](../benchmark_v1_paper_comparators.md)', '',
        '重新分析可运行：bash local/python.sh -m benchmarks.generic_motion_v1.reports.rrtconnect_comparison_v1.analyze '
        '--output /tmp/csdecomp-rrt-comparison-new。仅读取已封存数据；输出目录必须尚无同名分析产物。', '',
    ]
    output.mkdir(parents=True, exist_ok=True)
    (output/'comparison.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False)+'\n')
    (output/'comparison.md').write_text('\n'.join(lines))
    assert all(sha(inputs[name]) == record['sha256'] for name, record in sources.items())
    receipt = dict(complete=True, created_utc=datetime.now(timezone.utc).isoformat(), new_planner_calls=0,
                   source_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                   builder_sha256=sha(__file__),sources=sources,
                   outputs={name:sha(output/name) for name in ('comparison.json','comparison.md')})
    (output/'receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
    print(json.dumps(dict(complete=True, methods=len(methods), distance_pairs=len(paired), output=str(output))))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE)
    main(parser.parse_args().output)
