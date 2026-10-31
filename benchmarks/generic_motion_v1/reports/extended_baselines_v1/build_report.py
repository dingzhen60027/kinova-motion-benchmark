"""Render the completed six-method study only after independent evidence audit.

This script never calls a planner, GPU, solver, or experiment coordinator. It
refuses partial metrics, an unaudited matrix, stale source bindings, changed test
manifests, or a nonempty output path. All displayed numbers come from the sealed
study; renderer tests use explicitly synthetic fixtures outside the report path.
"""
from collections import Counter
from datetime import datetime, timezone
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shlex

OUT = Path(__file__).resolve().parent
BASE = OUT.parents[1]
ROOT = BASE.parents[1]
LABELS = ('LSCS', 'RRTConnect', 'RRT*', 'CHOMP', 'STOMP', 'TrajOpt')
SUCCESS = 'success_validated_numerical'
FAMILIES = ('sparse_clutter', 'dense_clutter', 'bottleneck', 'route_choice')
DEFAULT_REPORT = BASE/'reports/benchmark_v1_extended_baselines.md'


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(value, digits=3):
    return '—' if value is None else f'{value:.{digits}f}'


def escape(value):
    return str(value).replace('|', r'\|').replace('\n', ' ')


def table(headers, rows):
    return '\n'.join(['| '+' | '.join(map(escape, headers))+' |',
                       '| '+' | '.join('---' for _ in headers)+' |'] +
                      ['| '+' | '.join(map(escape, row))+' |' for row in rows])+'\n'


def ratio(value):
    if value.get('estimate') is None:
        return '—'
    result=f"{100*value['estimate']:.1f}%"
    if value.get('ci95') is not None:
        result += f" [{100*value['ci95'][0]:.1f}, {100*value['ci95'][1]:.1f}]"
    return result


def difference(value, scale=1.):
    if value.get('estimate') is None:
        return '—'
    result=number(scale*value['estimate'])
    if value.get('ci95') is not None:
        result += f" [{number(scale*value['ci95'][0])}, {number(scale*value['ci95'][1])}]"
    return result


def statistic(group, field, pool='all_legal'):
    value=group['statistics'][field][pool]
    return f"{number(value['mean'])} (n={value['count']})"


def group(metrics, split, label, scope='all'):
    rows=[row for row in metrics['groups'] if row['split']==split and row['label']==label and row['scope']==scope]
    require(len(rows)==1, f'Expected one {split}/{label}/{scope} group')
    return rows[0]


def outcome_fraction(value):
    return f"{int(value['valid_success']['numerator'])}/{int(value['valid_success']['denominator'])}"


def validate_counts(metrics):
    require(metrics.get('complete') is True and metrics.get('comparable') is True,
            'Only completed, comparable metrics may be published')
    rows=metrics['rows']
    require(metrics['requested_jobs']==1140 and len(rows)==1140, 'Expected the complete 1140-slot matrix')
    require(sum(row['input_legal'] for row in rows)==1128, 'Expected 1128 legal slots and 12 invalid records')
    require({row['split'] for row in rows}=={'smoke','dev'}, 'Formal test observations cannot enter this study')
    require(set(metrics['expected_method_labels'])==set(LABELS), 'Unexpected method labels')
    require(not metrics.get('pairing_differences'), 'Unresolved shared-frontend mismatch')
    for split,registered,legal in [('smoke',10,8),('dev',180,180)]:
        for label in LABELS:
            selected=[row for row in rows if row['split']==split and row['label']==label]
            value=group(metrics,split,label)
            require(len(selected)==registered and sum(row['input_legal'] for row in selected)==legal,
                    f'Wrong matrix denominator for {split}/{label}')
            require(value['requested_jobs']==registered and value['legal_jobs']==legal,
                    f'Aggregate denominator disagrees for {split}/{label}')
            require(value['status_counts']==dict(Counter(row['status'] for row in selected)),
                    f'Status histogram disagrees for {split}/{label}')
            successes=sum(row['success'] for row in selected if row['input_legal'])
            require(value['valid_success']['numerator']==successes and value['valid_success']['denominator']==legal,
                    f'Success aggregate disagrees for {split}/{label}')
            require(sum(value[key] for key in ('converged_successes','validated_incumbent_nonconverged',
                    'validated_unknown_convergence'))==successes, 'Convergence partition does not match validated successes')
            require(sum(value['termination_counts'].values())==legal, 'Termination histogram omits legal observations')
            # Check the principal mean columns against observations, including
            # failures in latency and excluding them only from output quality.
            for field,pool in [('cold_e2e_seconds','all_legal'),('warm_query_seconds','all_legal'),
                               ('cost','validated_success')]:
                available=[row[field] for row in selected if row['input_legal'] and
                           (pool!='validated_success' or row['success']) and row[field] is not None]
                saved=value['statistics'][field][pool]
                require(saved['count']==len(available), 'Available-value denominator changed')
                expected=sum(available)/len(available) if available else None
                require(saved['mean'] is None if expected is None else
                        saved['mean'] is not None and math.isclose(saved['mean'],expected,rel_tol=1e-10,abs_tol=1e-10),
                        f'Mean differs from raw observations: {split}/{label}/{field}')
    for label in LABELS:
        require(len({row['case_id'] for row in rows if row['split']=='dev' and row['label']==label})==60,
                'Repeated seed observations must bind exactly 60 dev tasks')
        require(len({row['case_id'] for row in rows if row['split']=='smoke' and row['label']==label})==10,
                'Smoke input registrations differ')
    return rows


def validate_inputs(directory, metrics_path, audit_path):
    directory=Path(directory)
    frozen_path=directory/'execution_freeze.json'
    metrics=read(metrics_path); frozen=read(frozen_path); audit=read(audit_path)
    rows=validate_counts(metrics)
    require(audit.get('passed') is True and not audit.get('errors'), 'Independent final audit has not passed')
    require(audit['counts']['registered_slots']==1140 and audit['counts']['legal_slots']==1128
            and audit['counts']['invalid_slots']==12 and audit['counts']['receipts']==1140,
            'Independent audit counts disagree with the declared matrix')
    require(audit['counts']['validated_successes']==sum(row['success'] for row in rows),
            'Independent audit success count disagrees with metrics')
    require(audit.get('metrics_sha256')==sha(metrics_path), 'Final audit does not bind these metrics')
    require(audit.get('execution_freeze_sha256')==sha(frozen_path), 'Final audit does not bind this execution freeze')
    require(audit.get('run_index_sha256')==sha(directory/'run_index.json'), 'Final audit does not bind this run index')
    require(frozen['planned_registered_slots']==1140 and frozen['test_planner_calls']==0,
            'Freeze has a different task matrix')
    require(frozen['timeout_seconds']==60. and frozen['gpu_workers']==1, 'Unexpected execution budget/concurrency')
    require(not frozen.get('reused_evidence'), 'This report requires fresh six-method observations, not pooled historical results')
    completed_path=directory/'completed_blocks.json'; completed=read(completed_path)
    require(completed['complete'] is True and completed['completed_jobs']==1140 and len(completed['blocks'])==4,
            'All four run blocks must be sealed before reporting')
    require(len(metrics['runs'])==4, 'Expected four audited run manifests')
    frozen_runs={str(Path(block['run']).resolve()):block for block in frozen['blocks']}
    for item in metrics['runs']:
        path=Path(item['run'])
        require(str(path.resolve()) in frozen_runs, 'Unexpected run outside the declared matrix')
        require(item['manifest_sha256']==sha(path/'run_manifest.json'), 'Run manifest changed after metrics')
        manifest=read(path/'run_manifest.json')
        require(len(manifest['jobs'])==frozen_runs[str(path.resolve())]['planned_jobs'], 'Run registration count changed')
        require(manifest['source_hashes']==frozen['runtime_source_hashes'], 'Run uses different runtime sources')
        require(manifest['timeout_seconds']==60., 'Run has a different external budget')
        require(all(job['evaluation_config']==frozen['evaluation_config'] for job in manifest['jobs']),
                'Run configuration differs from the declared profile')
    protected={path:digest for path,digest in frozen['data_files'].items()
               if path.endswith('/manifests/test.jsonl') or path.endswith('/manifests/test.SHA256SUMS')}
    require(len(protected)==2, 'Missing frozen formal-test protection hashes')
    require(all(Path(path).is_file() and sha(path)==digest for path,digest in protected.items()),
            'Protected formal-test files changed')
    maps=[]
    for path in sorted({block['roadmap'] for block in frozen['blocks']}):
        meta_path=Path(path)/'metadata.json'
        require(frozen['roadmap_files'].get(str(meta_path.resolve()))==sha(meta_path), 'Offline map metadata changed')
        maps.append(dict(path=str(meta_path),sha256=sha(meta_path),**read(meta_path)))
    command_path=directory/'commands.jsonl'
    commands=[json.loads(line) for line in command_path.read_text().splitlines() if line.strip()]
    finished=[row for row in commands if row.get('event')=='finish']
    require(any(row.get('action')=='metrics' and row.get('returncode')==0 for row in finished),
            'Missing successful actual metrics command')
    return metrics,frozen,audit,completed,protected,maps,commands


def render(metrics, frozen, audit, protected, maps, commands, provenance, replay_available=False):
    rows=metrics['rows']; dev=[group(metrics,'dev',label) for label in LABELS]
    total_success=sum(row['success'] for row in rows)
    source=frozen['source_commit']; env=frozen['environment']; cfg=frozen['evaluation_config']
    lines=['# 扩展运动规划基线：完整接入与开发集实测', '',
        '**RRTConnect、RRT*、CHOMP、STOMP、TrajOpt 已接入统一 CLI，并与本轮重新运行的 LSCS 完成配对测试。** '
        f'本轮封存 1,140 个槽位：1,128 次合法输入调用、2 道已冻结生成失败输入 × 6 方法形成的 12 个无效槽位，数值验证成功共 {total_success} 条。'
        '全部为 smoke/dev；历史结果未混入这次运行，正式 test 没有新增规划调用。', '',
        '## 固定输入、方法和评价口径', '',
        '开发集是 60 道固定题，使用种子 0、1、2 的三次配对重复，每方法 180 次合法调用；'
        'smoke 每方法登记 10 题，其中 8 题合法。三次重复不是三个新场景。所有方法保持相同的七关节 Kinova 模型、'
        '碰撞过滤对、障碍体素球、端点、独立终检和 60 秒外部预算。', '',
        table(['方法','执行实现','初始化与原生目标'],[
            ['LSCS','CSDecomp DRM → EI-ZO → 作者 SCSPlanning / Clarabel','固定 DRM 路径；凸走廊内 PWL 弧长'],
            ['RRTConnect','官方 OMPL 2.0.1 原生实现','独立双向树；首先求可行路径，再原生顶点简化'],
            ['RRT*','官方 OMPL 2.0.1 原生实现','独立采样与重连；固定时间内优化路径长度'],
            ['CHOMP','固定端点加速度度量的协变下降','相同 DRM 路径；平滑代价＋规范模型距离势的 C-space 弧长积分'],
            ['STOMP','官方 ros-industrial/stomp 未修改的 C++ 核心','相同 DRM 路径；原生控制代价＋规范模型距离势任务回调'],
            ['TrajOpt','作者 BasicTrustRegionSQP ＋ BPMPD 2.21','相同 DRM 路径；离散平滑目标及有符号距离碰撞约束']]),
        '公共质量指标统一为通过独立验证的七关节 PWL 欧氏弧长，单位 rad。CHOMP/STOMP/TrajOpt 的原生平滑与障碍目标'
        '另外保存，不冒称与 PWL 弧长完全相同。CHOMP 使用原论文的协变更新，障碍泛函明确适配为规范几何的 '
        'C-space 距离势积分；没有更换机械臂碰撞模型。五个扩展方法属于几何路径轨道，未虚构连续最短时间优化器。', '',
        '## 开发集主结果', '',
        table(['方法','验证成功/合法调用','成功率及场景 95% CI','冷启动均值 / P50 / P95，s','热查询均值，s','成功路径长度均值，rad'],[
            [g['label'],outcome_fraction(g),ratio(g['valid_success']),
             ' / '.join(number(g['statistics']['cold_e2e_seconds']['all_legal'][key]) for key in ('mean','p50','p95')),
             statistic(g,'warm_query_seconds'),statistic(g,'cost','validated_success')] for g in dev]),
        '冷启动均值和分位数包含全部合法调用的成功、前端失败、求解失败和超时。路径长度只统计经过公共终检的输出；'
        '不同方法成功集合不同，不能据此按该均值直接排名。CI 使用场景聚类 bootstrap：PCG64，2,000 次，seed=45827；'
        '同场景的多个端点及重复种子一起重采样。', '',
        '## 失败分类与求解收敛', '']
    statuses=sorted({status for g in dev for status in g['legal_status_counts'] if status!=SUCCESS})
    lines += [table(['方法',*statuses],[[g['label'],*[g['legal_status_counts'].get(status,0) for status in statuses]] for g in dev]),
        '原生近似路径不计成功；planner_timeout 记录预算内没有原生精确解，timeout 是外部或共享期限中断。'
        '这些状态以及求解器不可行返回都不能证明整道运动规划题无解。失败时已产生的轨迹、搜索树、求解日志和异常信息均保留；未产生候选的失败不补造轨迹。','',
        table(['方法','收敛且验证成功','未收敛但验证成功','验证成功、收敛未知','合法调用中收敛未知','原生终止统计'],[
            [g['label'],g['converged_successes'],g['validated_incumbent_nonconverged'],g['validated_unknown_convergence'],
             g['unknown_convergence'],'; '.join(f'{key}: {value}' for key,value in sorted(g['termination_counts'].items()))] for g in dev]),
        '收敛只认候选中明确的 solver_converged 布尔值；缺失标记保持未知，包含未生成候选的前端失败。'
        'TrajOpt 在内部时间或迭代上限处仅可返回已经被 SQP 接受且确实改变的迭代点，仍须通过公共终检；'
        '这些输出与原生 CONVERGED 分开统计，初始化路径不能伪装成超时后的优化结果。','',
        '## 同一题目的配对比较','']
    pairs=[p for p in metrics['paired_comparisons'] if p['split']=='dev' and p['scope']=='all']
    require(len(pairs)==15,'All 15 method pairs must be present')
    lines += [table(['A − B','合法配对 / 共同成功','成功率差 pp [95% CI]','共同成功长度差 rad [95% CI]','全部合法冷延迟差 s [95% CI]'],[
        [p['method_a']+' − '+p['method_b'],f"{p['paired_legal_jobs']} / {int(p['common_success_cost_difference']['denominator'])}",
         difference(p['valid_success_difference'],100),difference(p['common_success_cost_difference']),
         difference(p['observed_cold_latency_difference'])] for p in pairs]),
        '差值方向严格为 A − B。长度差只基于双方共同成功，负值表示 A 的已验证路径更短；'
        '成功率差和冷延迟差保留全部合法配对，冷延迟包含失败。条件质量结果不能外推为全体任务上的最优性结论。','',
        '## 障碍族、直连标签与 smoke 对照','',
        table(['开发集障碍族',*LABELS],[[family,*[outcome_fraction(group(metrics,'dev',label,'family:'+family)) for label in LABELS]] for family in FAMILIES]),
        table(['开发集固定直连标签',*LABELS],[[title,*[outcome_fraction(group(metrics,'dev',label,'straight_line:'+str(value))) for label in LABELS]]
            for value,title in [(True,'直连检出碰撞'),(False,'直连未检出碰撞')]]),
        table(['smoke 分层',*LABELS],[[title,*[outcome_fraction(group(metrics,'smoke',label,scope)) for label in LABELS]]
            for scope,title in [('all','全部合法输入'),('obstacle_main','新增障碍'),('open_control','无新增障碍对照')]]),
        '直连标签在算法运行前冻结，不能用方法成败改题。开发集没有 open_control；smoke 的两题无新增障碍对照'
        '共享同一基础世界，单列且不伪造多场景 CI。bottleneck/route_choice 是工作空间几何描述，不代表已知 C-space 同伦类别。','',
        '## 预算、阶段耗时与离线费用','',
        f"外部冷启动看门狗的终止触发期限为 {frozen['timeout_seconds']:.0f} 秒，包含解释器启动、模型与依赖加载、GPU 初始化、规划、"
        '优化和最终验证。到期触发进程终止后，最多等待 2 秒清理，再强制结束；实际冷延迟包括清理耗时，'
        '不截断为 60 秒。热查询是同一次冷调用中从规划查询阶段起记录的时间；不是第二次独立热启动试验。'
        '未完成后端的部分计时仅保留在原始记录和冷延迟，不冒充完整阶段均值。','',
        f"RRT* 内部规划上限 {cfg['ompl']['planning_seconds']:.0f} 秒，RRTConnect 在找到首条精确解时可提前退出；"
        f"两者使用最多 {cfg['ompl']['simplification_max_steps']} 次顶点简化、{cfg['ompl']['simplification_seconds']:.1f} 秒简化预算。"
        f"CHOMP/STOMP 最多 {cfg['extended_baselines']['chomp']['maximum_iterations']}/"
        f"{cfg['extended_baselines']['stomp']['maximum_iterations']} 次迭代；TrajOpt 每个罚系数阶段最多 "
        f"{cfg['extended_baselines']['trajopt']['maximum_sqp_iterations']} 次 SQP、内部 "
        f"{cfg['extended_baselines']['trajopt']['maximum_optimizer_seconds']:.0f} 秒。扩展方法预留 5 秒用于公共验证；"
        '所有阶段仍受同一外部期限约束。','']
    stagekeys=['drm_path_seconds','eizo_inflation_seconds','sampling_planning_seconds','trajectory_optimization_seconds','validation_seconds']
    stage_rows=[]
    for g in dev:
        values=[]
        for key in stagekeys:
            saved=g['stage_seconds'].get(key,{}).get('all_legal',{})
            values.append(f"{number(saved.get('mean'))} (n={saved.get('count',0)})")
        stage_rows.append([g['label'],g['complete_backend_timing_jobs'],*values])
    lines += [table(['方法','完整后端计时数','DRM 查询 s','EI-ZO s','独立采样 s','轨迹优化 s','终检 s'],stage_rows),
        'DRM 条件成功率和共用种子路径 SHA 检查只适用于 LSCS/CHOMP/STOMP/TrajOpt。OMPL 从固定端点独立搜索；'
        '其路网标签只是重复种子的配对标识，不加载路网，也不承担离线路网构建费用。','',
        table(['路网种子','请求 / 实际节点','已封存构建时间 s','构建时间 / 60，s/查询','构建时间 / 1000，s/查询'],[
            [m['seed'],f"{m['requested_nodes']} / {m['actual_nodes']}",number(m['offline_seconds']),number(m['offline_seconds']/60),number(m['offline_seconds']/1000)] for m in maps]),
        '上表读取既有封存路网的实际构建记录，本轮没有重新构图。Q 次请求的费用模型是“在线冷延迟＋构建时间/Q”；'
        '摊销列是明确的算术情景，不是额外测量，也没有从在线时间中减去缓存或构图开销。','',
        '## 环境与源码证据','',
        f"执行源码提交：`{source}`；本轮所有运行绑定同一 runtime_source_hashes 与完整配置。",
        f"GPU：`{env['gpu']}`；解释器：`{env['executable']}`；Python `{env['python'].split()[0]}`。",
        table(['规范模型绑定','SHA256'],[['robot',rows[0]['robot_sha256']],['collision geometry / filters',rows[0]['geometry_sha256']]]),
        table(['依赖','实际版本'],[[name,version] for name,version in sorted(env['packages'].items())]),
        table(['扩展依赖','固定来源 / SHA'],[
            ['OMPL 2.0.1',provenance['ompl']['dependency']['source_commit']],
            ['OMPL 官方 wheel SHA256',provenance['ompl']['wheel']['sha256']],
            ['STOMP 官方核心 commit',provenance['chomp_stomp']['STOMP_source_commit']],
            ['TrajOpt 作者 commit',provenance['trajopt']['author_commit']],
            ['TrajOpt 传输补丁 SHA256',provenance['trajopt']['build_receipt']['transport_only_patch_sha256']]]),
        '现有 Conda 保持原状。OMPL、编译桥接头文件和 TrajOpt/STOMP 构建产物位于独立依赖目录；'
        '执行冻结同时校验已安装库、原生二进制和绑定源码。STOMP 核心未修改。TrajOpt 保持作者 SQP 优化逻辑，'
        '修复旧 BPMPD 管道传输对部分读写、EOF/关闭及阻塞中断的处理；修改范围和差异补丁单独保存。', '',
        '方法来源：[OMPL 安装与原生库](https://ompl.kavrakilab.org/installation.html)、'
        '[CHOMP 原论文](https://publications.ri.cmu.edu/chomp-covariant-hamiltonian-optimization-for-motion-planning)、'
        '[STOMP 官方 Task 接口](https://github.com/ros-industrial/stomp/blob/b1a87c80f7338caae25a5c689b876da15492aa75/include/stomp/task.h)、'
        '[TrajOpt 作者源码](https://github.com/joschu/trajopt/tree/9c634c060f7e2d61c4a888b0a5700d668a5d16f8)。','',
        '## 验收、限制与复现命令','',
        f"独立最终审计通过；原始文件、receipt、输入绑定及数组代价核查详见 [final_audit.json](extended_baselines_v1/final_audit.json)。"
        f"审计检查数：{audit.get('check_count',audit.get('counts',{}).get('checks','见审计文件'))}。",
        '公共验收要求端点、关节限位、原碰撞对、CUDA 与独立 Drake 几何检查一致通过；0.005 rad 有限采样通过'
        '不等于连续无碰撞数学证明。随机任务的可行性保持 unknown；场景或端点没有因任何算法成功与否被替换。',
        '当前主结论限于冻结 Kinova 开发分布和本机硬件、给定预算与初始化。独立采样与 DRM 热启动优化具有不同前端，'
        '因此主表比较端到端系统，条件 DRM 表只解释共享前端子集。后续算法应保持冻结评价协议，另注册新研究比较。','',
        '正式 test 文件保持以下冻结校验值；本轮未重新评测或调参：','',
        table(['受保护文件','SHA256'],[[Path(path).name,digest] for path,digest in sorted(protected.items())]),
        '完整配置、命令与数组证据：'
        '[配置](../configs/extended_baselines.json)、[实际命令账本](extended_baselines_v1/commands.jsonl)、'
        '[逐调用 CSV](extended_baselines_v1/metrics/observations.csv)、[指标 JSON](extended_baselines_v1/metrics/metrics.json)、'
        '[冻结记录](extended_baselines_v1/execution_freeze.json)。']
    if replay_available:
        lines += ['', '[交互回放图库](extended_baselines_v1/replays/index.html)。']
    lines += ['', '以下命令直接摘自实际执行账本；按记录顺序列出完成事件，返回码保留。重跑应更换输出目录，已有槽位使用 resume，不能覆盖封存证据。','']
    for command in commands:
        if command.get('event')!='finish':
            continue
        lines += [f"`{command.get('block')}/{command.get('action')}`：退出码 {command.get('returncode')}。",'',
                  '```bash',shlex.join(command['argv']),'```','']
    lines += ['辅助复现入口已接入同一 CLI：`methods`、`run`、`batch`、`resume`、`verify`、`summarize`、`replay`。'
              '独立 OMPL 可省略 `--roadmaps` 并使用 `--seed-indices`；共享前端方法必须提供路网。',
              '审计、回放与报告生成的实际调用另存于[后处理命令账本](extended_baselines_v1/postprocess_commands.jsonl)。', '']
    return '\n'.join(lines)


def build(directory=OUT, output=DEFAULT_REPORT, audit_path=None):
    directory=Path(directory).resolve(); output=Path(output).resolve()
    require(not output.exists(), 'Refusing to overwrite an existing report; choose a new explicit output')
    receipt_path=directory/'report_receipt.json'
    require(not receipt_path.exists(), 'Refusing to overwrite an existing report receipt')
    metrics_path=directory/'metrics/metrics.json'; audit_path=Path(audit_path) if audit_path else directory/'final_audit.json'
    metrics,frozen,audit,completed,protected,maps,commands=validate_inputs(directory,metrics_path,audit_path)
    provenance=dict(ompl=read(directory/'ompl_provenance.json'),chomp_stomp=read(directory/'chomp_stomp_audit.json'),
                    trajopt=read(directory/'trajopt_dependency_audit.json'))
    text=render(metrics,frozen,audit,protected,maps,commands,provenance,(directory/'replays/index.html').is_file())
    output.parent.mkdir(parents=True,exist_ok=True); output.write_text(text)
    receipt=dict(generated_utc=datetime.now(timezone.utc).isoformat(),report=str(output),report_sha256=sha(output),
        builder_sha256=sha(__file__),metrics_sha256=sha(metrics_path),execution_freeze_sha256=sha(directory/'execution_freeze.json'),
        final_audit_sha256=sha(audit_path),commands_sha256=sha(directory/'commands.jsonl'),
        completed_blocks_sha256=sha(directory/'completed_blocks.json'),registered_slots=1140,legal_slots=1128,
        test_planner_calls=0,source_commit=frozen['source_commit'],
        output_scope='Derived only from complete independently audited smoke/dev evidence; historical runtime results excluded')
    receipt_path.write_text(json.dumps(receipt,indent=2)+'\n')
    return receipt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,default=OUT)
    parser.add_argument('--output',type=Path,default=DEFAULT_REPORT)
    parser.add_argument('--audit',type=Path)
    args=parser.parse_args()
    print(json.dumps(build(args.directory,args.output,args.audit),indent=2))


if __name__=='__main__':
    main()
