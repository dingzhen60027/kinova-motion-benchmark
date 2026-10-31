"""Build the evidence report from saved outcomes, without running algorithms."""
import argparse
import datetime
import hashlib
from pathlib import Path
import numpy as np
from reproduction.common import ROOT, read, atomic
from reproduction.paper_completion.analyze import write_csv


def number(value, digits=3):
    return 'NA' if value is None else f'{value:.{digits}f}'


def build_throughput_index(out):
    """Normalize driver metadata without treating diagnostic status as a benchmark."""
    ledger = read(out/'verification_commands.json')
    index = {}
    for family in ['forest', 'fr3', 'public_iiwa']:
        records = [r for r in ledger if r['label'] == 'throughput_' + family]
        if not records:
            return
        record = records[-1]
        if record.get('exit_code') != 0 or record['status'] != 'completed':
            return
        run = Path(record['run_dir'])
        manifest = read(run/'manifest.json')
        expected_status = 'completed_diagnostic' if family == 'public_iiwa' else 'completed'
        if manifest['status'] != expected_status:
            raise ValueError(f"Unexpected throughput state: {family}: {manifest['status']}")
        summary = read(run/'summary.json')
        normalized = dict(summary)
        if family == 'public_iiwa':
            normalized.update(batches=summary['measured_batches'],
                sample_sd=summary['sample_sd_million_configurations_per_second'],
                verified_points=summary['independently_verified_points'])
        index[family] = dict(run_dir=str(run), source_status=manifest['status'],
            summary=normalized, command_label=record['label'],
            source_summary_sha256=hashlib.sha256((run/'summary.json').read_bytes()).hexdigest(),
            source_manifest_sha256=hashlib.sha256((run/'manifest.json').read_bytes()).hexdigest())
    atomic(out/'throughput_index.json', index)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='reproduction/reports/paper_completion')
    args = parser.parse_args()
    out = Path(args.output)
    build_throughput_index(out)
    summaries = {key: read(out/key/'summary.json') for key in ['forest','fr3']}
    reference = read(ROOT/'reproduction/reports/paper_protocol/paper_reference.json')
    comparison, lines = [], []
    for family, summary in summaries.items():
        for method in summary['methods']:
            name = method['method']; paper = reference[family][name]
            comparison.append(dict(environment=family, method=name,
                requested=method['requested'], native_drm_successes=method['native_drm_successes'],
                actual_solver_success_rate_given_drm=method['solver_success_rate_given_native_drm'],
                actual_native_cuda_cfr_given_drm=method['native_cuda_cfr_given_drm'],
                actual_solver_success_and_cuda_free_rate_given_drm=method['solver_success_and_native_cuda_free_rate_given_drm'],
                actual_strict_validated_rate_given_drm=method['validated_rate_given_native_drm'],
                paper_solver_success_rate_given_drm=paper['solver_success_rate'],
                paper_cfr_given_drm=paper['collision_free_rate'],
                actual_cost_mean_solver_success=method['cost_mean_solver_success'],
                actual_cost_mean_independent_validated=method['cost_mean_validated'],
                paper_cost_mean=paper['cost'],
                actual_total_ms_mean_attempted=method['total_ms_mean_all_attempts'],
                actual_total_ms_sd_attempted=method['total_ms_sd_all_attempts'],
                paper_total_ms_mean=paper['total_ms'],
                comparable_original_model=False if family=='fr3' else None,
                directly_comparable_timing=False))
    write_csv(out/'paper_vs_actual.csv', comparison)
    audit = {family: read(out/'audit'/family/'audit.json') for family in summaries}
    lines += ['# 按论文补全复现：结果与未完成项', '',
      '**结论：公开方法和参数的实现进一步补全，但完整原论文复现仍未完成。** '
      '特别是现有 FR3 33 球拟合存在已实测证实的过包络伪障碍，不能代表作者原始模型。'
      '本轮所有运行均按冻结配置执行；FR3 数值作为明确标注的重建模型诊断，不作为论文结论的复现证据。', '',
      '本报告对应 `paper_completion_v4`。旧 v1–v3 源码、输入、轨迹、报告和失败记录保持不变。'
      '新运行在同一 Conda 前缀、同一 `main` 工作树上进行，无其他研究项目改动。', '',
      '## 从论文和原始源文件恢复的内容', '',
      '- 论文 V-B 的在线顺序：原生碰撞地图、碰撞集、从 pose map 筛选近目标无碰撞配置、真实位姿 IK、原生 DRM。目标位姿固定，不因规划失败重新抽题。Forest 现在也真实构建离线体素碰撞地图，第二路线同样调用原生 DRM。',
      '- 论文 VI-C：第二路线无路时停止扩展，保留第一路线。VI-B：按反例修改区域并核验整条原始路径覆盖。float32 半空间转换采用有误差界的向外舍入，原生包含检查通过后才继续；碰撞阈值未放宽。',
      '- EI-ZO 正式表 III/IV/V 参数优先；原 arXiv TeX 注释补出 `max_iterations=30`。正文可支持 `tau=0.5`。源码注释中旧体素值与正式表格冲突，使用正式的 0.06 m。论文 IV-C 明确说明设置最大迭代次数可能使概率保证失效；本轮不能声称所有返回区域都取得 δ/ε 概率证书，最终使用保存的数值碰撞检查判定。',
      '- NTO 保留 DRM 节点，取消至少八控制点的弧长重采样。距离使用二阶样条，时间使用四阶样条；首尾各重复一次用于静止端点，是公开声明的推断。原 TeX 注释给出内部 k/Nc 配点，共 Nc−1 个绑定；它与正式正文的 Nc 个点存在计数歧义，尚不能认定作者实际运行代码已恢复。',
      '- LGCS 实现引用文献 Eq. 5.5 的透视流模型在分裂图上的严格代数投影，取消 big-M。`NonConvex=0` 保持凸模型；Gurobi 13 需 `Presolve=0` 避免实际测试中出现的 SOC 识别失败，这也是性能可比性差异。LSCS 仍用 Clarabel 替代未许可的 MOSEK。PDF 式 (19d) 印为每对相邻路径节点相等，结合不同的起终点会使问题不可行；实现采用正文所述连续 PWL SOCP 的解释，此项已列为待作者确认的排版歧义。',
      '- 新模型使用论文前 2025-04-11 官方关节范围；2026 版本扩大了六个关节范围。运动学、八个手臂网格及手部网格经逐字节核对未变。球中心、半径和过滤仍是公开几何重建，绝非作者原始资产。', '',
      '逐项来源、数值和优先级见 [参数追踪](parameter_trace.csv)、[源文件审计](source_audit.md)、'
      '[优化转录审计](optimization_audit.md) 与 [流水线审计](pipeline_audit.md)。', '',
      '## 实际完成的运行', '',
      '| 环境 | 预定题目 | 原生 DRM 成功 | 独立有效种子 | 运行状态 |',
      '|---|---:|---:|---:|---|']
    for family, s in summaries.items():
        lines.append(f"| {family.upper()} | {s['requested']} | {s['native_drm_successes']} | {s['strict_seed_valid']} | {s['manifest_status']} |")
    lines += ['', '400 题来自 10 个环境种子 × 10 个路线图种子 × 4 个规模。'
              '“运行完成”只表示全部预定题目已处理，包含规划失败、许可证失败和求解失败，不等于论文复现完成。', '',
      '| 环境 | 方法 | 实际尝试 | 求解成功 | CUDA 检查通过 | 求解成功且 CUDA 通过 | 独立严格通过 / 预定题目 | TO+CS 均值 ± SD (ms) |',
      '|---|---|---:|---:|---:|---:|---:|---:|']
    for family, s in summaries.items():
        for r in s['methods']:
            lines.append(f"| {family.upper()} | {r['method']} | {r['attempted']} | {r['solver_successes']} | {r['native_cuda_free_trajectories']} | {r['native_cuda_validated_successes']} | {r['validated_successes']} / {r['requested']} | {number(r['total_ms_mean_all_attempts'])} ± {number(r['total_ms_sd_all_attempts'])} |")
    lines += ['', '论文表 I 的 TO SR/CFR 以原生 DRM 成功为条件；上表保留全 400 分母。'
              '对应条件率、求解成功样本代价、独立验证样本代价和论文参考数值见 [paper_vs_actual.csv](paper_vs_actual.csv)。'
              '原 TeX 表格注释明确提到 NTO 求解失败仍可能得到无碰撞轨迹，因此 CUDA CFR 不与求解成功作合取；两者合取另列。没有保存候选或检查的题目保留在原生 DRM 分母中，不算通过。'
              'CUDA 检查是本机发布版本的实际谓词，不代表已经恢复论文二进制。独立验证还检查端点、关节域、速度/加速度和几何；不将微小穿透改判成功。', '',
              '计时为保存的构建和优化时间，包括失败尝试；最终候选独立验证、离线建图、IK、预热和外围预解/回放写入不在正常 TO+CS 计时内。'
              '备用地图序列化/加载、备用路径核验和恢复覆盖记录的写入实际发生在构建计时内，不能声称全部 IO 已被扣除。'
              '异常的墙钟回退单独标记；超时/进程中断且已开始的方法计入尝试数，但没有完整计时的尝试不进入时间均值。'
              '方法尝试数不同，不用边际均值比值声称加速。成对代价要求双方均独立通过，成对时间要求双方有已记录的计时。'
              'SD 使用 ddof=1；统计按环境聚类，400 题不视为独立重复。', '',
      '## 反例恢复触发率', '',
      '| 环境 | 方法 | 触发次数 | 实际尝试 | 给定尝试的触发率 | 给定原生 DRM 成功的触发率 |',
      '|---|---|---:|---:|---:|---:|']
    recovery_rows = []
    for family, s in summaries.items():
        for r in s['methods']:
            if r['method'].startswith('NTO'):
                continue
            recovery_rows.append(dict(environment=family, method=r['method'],
                triggered=r['recovery_triggered'], attempted=r['attempted'],
                native_drm_successes=r['native_drm_successes'],
                given_attempt=r['recovery_rate_given_attempt'],
                given_native_drm=r['recovery_rate_given_native_drm'],
                given_valid_seed=r['recovery_rate_given_valid_seed'],
                given_solver_success=r['recovery_rate_given_solver_success'],
                completed_edit_iterations=r['completed_recovery_iterations']))
            lines.append(f"| {family.upper()} | {r['method']} | {r['recovery_triggered']} | {r['attempted']} | {number(r['recovery_rate_given_attempt'])} | {number(r['recovery_rate_given_native_drm'])} |")
    write_csv(out/'recovery_rates.csv', recovery_rows)
    lines += ['', '论文 VII-A5 报告 Forest 距离/时间恢复率 5.6%/0.46%，Franka 为 40.22%/21.36%。'
              '正文未明确距离问题中 LSCS/LGCS 的池化和精确分母，因此这里按方法报告真实计数及不同条件率，不宣称数值具有相同分母。'
              '触发是保存的候选产生碰撞并需要修复，包含随后修复异常的尝试；完成的修改迭代数另记在 recovery_rates.csv。', '',
      '## 失败和限制', '']
    for family, s in summaries.items():
        lines.append(f"**{family.upper()}**：原始题目状态 `{s['case_status_counts']}`。")
        lines.append('')
        for r in s['methods']:
            lines.append(f"- {r['method']}：`{r['status_counts']}`。")
        lines.append('')
    lines += [
      '本地流程增加了严格种子核验门槛：原生 0.1 步长 DRM 返回的路径若未通过独立检查，LSCS、LGCS、SCSTO 记录为 invalid_seed_path 并跳过膨胀和求解；NTO 仍使用该路径作为初值。'
      '这是当前重建的输入核验选择，不是已恢复的作者接受规则。所有这类原生 DRM 成功题目仍保留在条件率分母中，因此方法条件率不能直接视为论文精确复现。', '',
      'Forest 的 e03_r03_n00400 / SCSTO 异常来自作者 `fixed_velocities.solve` 中的 '
      '`assert result.is_success()`，实际求解器未成功；原 traceback 和求解前区域均已保存。', '',
      '两个预选旧适配失败已单独真实回归通过，旧 e08 边界目标的 float32 穿透仍按失败保留。'
      '旧 r01_n03000 地图在加入在线障碍之前就有 1512/1488 两个分量，在线 IK 不能保证改变这一事实。'
      '有限路线图断连不证明连续配置空间断连。', '',
      '**球模型问题已有独立几何证据。** 常见 home 下，拟合 link5–hand 球对穿透 −35.1577 mm，'
      '而同一运动学下官方两体碰撞网格的凸包距离 +37.0481 mm。凸包包含原网格，其正间距证明这一几何对的球碰撞是伪障碍。'
      '固定其他关节的 31 点 q7 扫描中，22 点发生球碰撞，但官方两体凸包全部分离。'
      '这是特定几何对的诊断，不是整机连续安全证明。没有通过增加碰撞过滤来隐藏这些失败。', '',
      '实际 FR3 为 33 个球、32 个活动球，179 个机器人内部候选对加 32 个桌面候选对，静态共 211 对；动态障碍有 640 对，总共 851 对。'
      '论文报告 1187 对，并将其中 527 对称为自碰撞；其余 660 对对应 33 球×20 障碍。桌面计数与过滤细节仍不完整。原球几何/过滤缺失，单凭总数无法恢复，不能为了凑数添加无意义检查。', '',
      '原公开硬件部件组合也做了实际探针：作者 iiwa 8 盒加 WSG 3 盒可组装成 7DOF 的 11 盒模型，'
      '两个场景各 2000 个随机配置的 CUDA/Drake 核对一致；安装坐标和桌子是声明的重建选择。'
      '这不等于论文原 105 对模型，更不是三台 D415、真实 KUKA 和 15 条实际执行记录。'
      '先前安装坐标错误导致全部碰撞的尝试也保留。见 [硬件部件探针](hardware_probe_summary.json)。', '',
      '还缺作者原始 FR3 球模型、过滤、随机任务输入及完整 NTO 转录；MOSEK 未许可，Gurobi 的含二次项免费模型受 200 变量限制；'
      '当前 csdecomp 0.1.2 二进制晚于论文，CPU/GPU/CUDA 版本也不同。不能声称复现了 17.1 倍加速、27.9% 可靠性提升或 823.4 ms 实机延迟。', '',
      '## 附录吞吐量与历史核心', '']
    throughput_path = out/'throughput_index.json'
    if throughput_path.exists():
        throughput = read(throughput_path)
        lines += ['每次批量为 2,000,000 个配置，预热独立于五次同步计时。Forest/FR3 各有十个环境；公开 iiwa 只作一组已声明模型诊断。', '',
                  '| 模型 | 实际批次 | Mconfig/s 均值 ± SD | 独立核对点 | 不一致点 |',
                  '|---|---:|---:|---:|---:|']
        for family, entry in throughput.items():
            s = entry['summary']
            lines.append(f"| {family} | {s['batches']} | {number(s['mean_million_configurations_per_second'])} ± {number(s['sample_sd'])} | {s['verified_points']} | {s['model_disagreements']} |")
        lines += ['', '吞吐计时包含公开同步包装器的分配与主机/设备传输；随机采样和文件写入不计。'
                  '实际数组、碰撞标志、计时和源码身份可从 [吞吐原始索引](throughput_index.json) 定位。'
                  '模型对数、设备和软件均与原论文不同，尤其公开 iiwa 组装不是原 105 对模型，不计算论文速度比。', '']
    else:
        lines += ['修订模型的吞吐实验尚无完整索引；此项保留未完成状态。', '']
    core_path = out/'paperdate_core_result.json'
    if core_path.exists():
        core = read(core_path)
        if core.get('compiled') is True:
            lines += [f"论文前公开核心已生成编译产物；执行记录状态为 `{core.get('status', 'unknown')}`。"
                      '是否通过各项功能探针须分别查看 [历史核心执行结果](paperdate_core_result.json)，不能由编译成功推断。', '']
            if core.get('status') == 'passed':
                collision, drm, eizo = (core['checks'][name] for name in ['collision', 'drm', 'eizo'])
                lines += [f"本次三项探针均实际通过：CUDA 与解析几何核对 {collision['samples']} 点、"
                          f"与 Drake 核对 {collision['drake_samples']} 点，差异均为 0；"
                          f"原 DRM 构建 {drm['actual_nodes']} 节点地图并求得 {drm['path_nodes']} 节点路径；"
                          f"EI-ZO 返回 {eizo['faces']} 面集合并包含种子，"
                          f"{eizo['independent_validation_samples']} 个独立样本无碰撞。"
                          '原 179 文件和已安装 wheel 的前后校验一致。'
                          '这些是构造场景的小型功能验证，不是原 800 题，也不构成连续空间安全证书。', '']
        else:
            lines += [f"论文前公开核心尚未确认编译成功；执行记录状态为 `{core.get('status', 'unknown')}`。"
                      '依赖配置成功不等于生成库，也不代表已通过 GPU/规划功能探针。'
                      '实际准备或失败记录见 [历史核心执行结果](paperdate_core_result.json)。', '']
        lines += ['构建说明见 [paperdate_build_readiness.md](paperdate_build_readiness.md)。'
                  '隔离构建未替换主实验的 csdecomp 0.1.2。原核心缺少新版 GPU recovery 接口，'
                  '即使核心构建和探针通过，也不等于本轮 800 题已改用原始二进制。', '']
    else:
        lines += ['论文前核心的隔离构建尚未取得最终执行记录；见 [构建准备](paperdate_build_readiness.md)。', '']
    enumeration_path = out/'lgcs_enumeration_results.json'
    if enumeration_path.exists():
        enumeration = read(enumeration_path)
        cases = read(Path(enumeration['run_dir'])/'results.json')
        results = [r.get('result', {}) for r in cases]
        corridors = sum(r.get('corridors_attempted', 0) for r in results)
        optimal = sum(r.get('numerical_global_optimum_established') is True for r in results)
        valid = sum(r.get('independent_validation_passed') is True for r in results)
        optimal_valid = sum(r.get('numerical_global_optimum_established') is True and
                            r.get('independent_validation_passed') is True for r in results)
        timed_out = sum(r.get('timed_out') is True for r in cases)
        unresolved = [r['case_id'] for r in results if r.get('unresolved_corridors', 0)]
        lines += ['## 许可证阻塞的独立补充诊断', '',
                  f'对主实验全部 {len(cases)} 道 `blocked_license` 题，使用保存的原区域图枚举简单路线，'
                  f'实际完成 {corridors} 次走廊 SOCP；{optimal} 题在保存区域图上建立数值最优性，'
                  f'{valid} 个最终候选通过独立几何验证，{len(cases)-valid} 个未通过，{timed_out} 题超时。'
                  f'同时满足数值最优且独立验证通过的为 {optimal_valid} 题。'
                  f'仍有未决走廊的题目为 `{unresolved}`，不声明其最优性。', '',
                  '这是避免专有许可证阻塞的补充数学诊断，并非论文的 Gurobi 实现。'
                  '该诊断使用独立 Drake 几何检查，未调用 CUDA，也没有执行新的反例恢复；保存区域中的最优路径仍可能碰撞。'
                  '主表保留原许可证失败；枚举耗时不纳入 Gurobi 速度比较。'
                  '详见 [实际枚举结果](lgcs_enumeration_results.json) 和 [诊断说明](lgcs_enumeration_diagnostic.md)。', '']
    lines += ['## 验证、命令与原始文件', '']
    for family,a in audit.items():
        lines.append(f"- {family.upper()}：独立重查 {a['independently_rechecked_fixed_points']} 个预定回放点；审计问题 `{a['issues']}`。"
                     f"原始完整轨迹标志中存在 {a['full_saved_model_disagreements']} 个 CUDA/独立几何判定不一致点，"
                     f"其最大绝对几何间距为 {a['maximum_absolute_independent_clearance_on_disagreements']:.3g} m；"
                     '这些边界差异按原严格判据保留，未调整阈值来增加成功数。')
    lines += ['', '所有实际命令、退出码与日志在本目录 `*_commands.json` 和 `verification_commands.json`；'
              '环境清单沿用 [environment.json](../../environment.json)、[Conda 锁](../../conda-explicit.txt) 和 [Python 锁](../../requirements.lock.txt)。'
              '最终全套 pytest 实际通过 26 项（1.76 秒，61 条第三方弃用警告），其中包含 8 项吞吐记录器故障注入测试；'
              '主实验前的 18 项检查记录也保留。另有 8 项独立 CPU 分析审查通过，见 [分析审计](analysis_audit.md)。'
              '记录器的单独验证见 [记录器验证](throughput_recording_verification.json)。'
              '其中合成元数据仅测试统计记账，不进入主实验；首次失败日志也保留。', '',
      '```bash',
      'bash local/python.sh -m pytest -q reproduction/tests',
      'bash local/python.sh -m reproduction.run_completion',
      'bash local/python.sh -m reproduction.paper_completion.run resume --run-dir <RUN_DIR>',
      'bash local/python.sh -m reproduction.paper_completion.analyze --run-dir <FOREST_RUN> <FR3_RUN> --output reproduction/reports/paper_completion --audit --collision-depths',
      'bash local/python.sh -m reproduction.audit_paper_completion --run-dir <FOREST_RUN> <FR3_RUN> --output reproduction/reports/paper_completion/audit --resume',
      '```', '',
      '每个 run 保存源码快照、配置、完整地图、在线 IK 全部尝试、原生种子、每次求解前的区域/种子、优化系数、回放和失败。'
      '终态失败与成功都由续跑跳过；源码或依赖漂移会拒绝续跑。', '',
      '## 达到完整复现还需要什么', '',
      '1. 作者的 FR3 33 球中心/半径、安装变换、碰撞过滤和桌面模型；这决定是否在求解同一个几何问题。',
      '2. 原始题目生成/种子、NTO 样条和求解器选项、验证阈值及计时脚本，消除当前可证实的歧义。',
      '3. 相应求解器许可、作者完整运行代码和匹配的软件版本；本轮已完成历史公开核心的小型验证，完整流水线仍不等价。硬件时延需要匹配硬件才能直接比较。',
      '4. KUKA 与三台 D415 的标定、原始感知和执行记录，以及可访问的真实设备，才能重做表 II 的实机实验。', '',
      '以上缺项不会用合成数据或“近似成功率”补齐。资产请求草稿已保存在 [author_asset_request.md](author_asset_request.md)，尚未发送。', '']
    (out/'reproduction_report.md').write_text('\n'.join(lines))
    atomic(out/'run_index.json', {family:dict(run_dir=s['run_dir'], implementation_hash=s['implementation_hash'], cases=s['requested']) for family,s in summaries.items()})
    atomic(out/'acceptance.json', dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        full_original_paper_reproduced=False, reconstructed_runs_complete=all(s['manifest_status']=='completed' and s['terminal_cases']==s['requested'] for s in summaries.values()),
        original_fr3_model_recovered=False, known_false_obstacle_pair=True,
        hardware_experiment_reproduced=False, native_source_version_matches_paper=False,
        paperdate_core_probe_passed=(read(core_path).get('status') == 'passed') if core_path.exists() else False,
        source_and_artifact_issues={f: s['source_integrity_issues']+s['artifact_issues'] for f,s in summaries.items()},
        independent_audit_issues={f:a['issues'] for f,a in audit.items()},
        actual_method_attempts=sum(r['attempted'] for s in summaries.values() for r in s['methods']),
        independently_validated_results=sum(r['validated_successes'] for s in summaries.values() for r in s['methods'])))
    print(out/'reproduction_report.md')

if __name__=='__main__':
    main()
