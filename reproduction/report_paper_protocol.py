"""Build the current report from audited measurements; no experiment calls."""

import argparse
import json
from pathlib import Path
from reproduction.common import ROOT, atomic, read
from reproduction.paper_protocol.analyze import write_csv


def fmt(value, digits=3):
    return "—" if value is None else f"{value:.{digits}f}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--report-root", type=Path, default=ROOT / "reproduction/reports/paper_protocol"
    )
    args = parser.parse_args()
    out = args.report_root
    index = read(out / "run_index.json")
    summaries = {
        family: read(out / family / "summary.json") for family in ["forest", "fr3"]
    }
    audits = {
        family: read(out / "audit" / family / "audit.json") for family in summaries
    }
    for family, summary in summaries.items():
        if (
            summary["manifest_status"] != "completed"
            or summary["source_integrity_issues"]
            or summary["artifact_issues"]
            or audits[family]["issues"]
        ):
            raise ValueError(
                "Refuse a final report from incomplete or failed audits: " + family
            )
    reference = read(out / "paper_reference.json")
    measurements = []
    table = [
        "| 环境 | 方法 | 实际尝试 | 求解器成功 | 验证通过 / 400 | TO+CS，均值 ± SD (ms) | 通过样本代价，均值 ± SD |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for family, summary in summaries.items():
        for r in summary["methods"]:
            p = reference[family][r["method"]]
            measurements.append(
                dict(
                    environment=family,
                    method=r["method"],
                    requested=r["requested"],
                    attempted=r["attempted"],
                    solver_successes=r["solver_successes"],
                    validated_successes=r["validated_successes"],
                    paper_solver_rate=p["solver_success_rate"],
                    paper_collision_free_rate=p["collision_free_rate"],
                    actual_solver_rate_requested=r["solver_successes"] / r["requested"],
                    actual_solver_rate_given_native_drm=(
                        r["solver_successes"] / summary["native_drm_successes"]
                        if summary["native_drm_successes"]
                        else None
                    ),
                    actual_validated_rate_given_native_drm=r[
                        "validated_rate_given_native_drm"
                    ],
                    paper_denominator="conditioned on native DRM success",
                    actual_validated_rate_requested=r["validated_rate_requested"],
                    paper_cost=p["cost"],
                    actual_cost_mean_validated=r["cost_mean_validated"],
                    paper_total_ms=p["total_ms"],
                    actual_total_ms_mean_all_attempts=r["total_ms_mean_all_attempts"],
                    caveat="Different original inputs/model/hardware, solver substitution and stricter validation; descriptive reference only",
                )
            )
            table.append(
                f"| {family.upper()} | {r['method']} | {r['attempted']} | {r['solver_successes']} | {r['validated_successes']} | {fmt(r['total_ms_mean_all_attempts'])} ± {fmt(r['total_ms_sd_all_attempts'])} | {fmt(r['cost_mean_validated'])} ± {fmt(r['cost_sd_validated'])} |"
            )
    write_csv(out / "paper_vs_actual.csv", measurements)
    ledger = read(ROOT / "reproduction/paper_protocol_v3_commands.json")
    runs = {family: read(Path(index[family]) / "manifest.json") for family in summaries}
    throughput = read(Path(index["fr3_throughput"]) / "summary.json")
    old_fr3 = read(
        ROOT / "reproduction/runs/20261025T080255.482345Z_paper_fr3/manifest.json"
    )
    old_cases = list(
        (ROOT / "reproduction/runs/20261025T080255.482345Z_paper_fr3/cases").glob(
            "*/case.json"
        )
    )
    old_completed = sum(read(p)["status"] == "completed" for p in old_cases)
    source_download = read(out / "public_source_download.json")
    fit = read(ROOT / "reproduction/paper_protocol/assets/fr3_33.json")
    audit_text = []
    for family, a in audits.items():
        resume = read(out / "audit" / family / "resume_check.json")
        audit_text.append(
            f"- {family.upper()}：{a['case_files']:,} 个案例文件；重查 {a['independently_rechecked_fixed_points']:,} 个预定采样点；独立重查、检查点和保存的密集碰撞标志一致。续跑退出码 {resume['exit_code']}，变化的案例文件 {len(resume['changed_case_files'])} 个。"
        )
    for family, a in audits.items():
        audit_text.append(
            f"- {family.upper()}：全部保存回放中有 {a['full_saved_model_disagreements']:,} 条 CUDA/独立几何判定不一致的采样记录（允许跨案例重复）；这些点的独立净距最大绝对值为 {a['maximum_absolute_independent_clearance_on_disagreements']:.3e} m。主结果要求两者均通过，未将差异点改判为成功；随机远离边界的测试不能代替这个边界审计。"
        )
    statuses = []
    for family, summary in summaries.items():
        statuses.append(f"\n### {family.upper()}\n")
        statuses.append(
            f"原生 DRM 成功 {summary['native_drm_successes']}/400，独立种子路径验证通过 {summary['strict_seed_valid']}/400。\n"
        )
        for r in summary["methods"]:
            states = "；".join(f"{k}={v}" for k, v in r["status_counts"].items())
            statuses.append(
                f"- {r['method']}：{states}；有效种子条件下通过 {r['validated_on_valid_seeds']}/{r['strict_seed_valid']}；触发几何恢复 {r['recovery_triggered']} 例。"
            )
    through = f"{throughput['mean_million_configurations_per_second']:.3f} ± {throughput['sample_sd']:.3f}"
    content = (
        f"""# 论文参数重建与实际复现实验报告

日期：2026-10-25。协议：`paper_parameters_v3`。代码与提交位于现有 `main`；原生 C++/CUDA 和此前的 Forest v1 代码、报告、原始运行均保留。

## 完成范围与结论边界

完成 Forest 与 FR3 各 **400 个预定案例**，每例配置五种方法。结果来自实际原生 DRM、CUDA EI-ZO、真实凸/混合整数/非线性求解器及保存轨迹的独立验证。失败、不满足独立验证和未执行的方法均保留。完整分母为每个环境 400，不能将有效种子或求解器成功样本改作总分母。

本次复现采用论文公开的样本规模、路线图参数、EI-ZO 参数、NTO 求解器和碰撞约束数量。作者原始 FR3 球心/半径、随机题单、完整 NTO 转录以及 KUKA 记录未在已检索公开来源中找到。因此这是**已公开参数下的可执行重建**，尚未精确复现原文全部数值或 17.1× 总结性加速，也没有完成 KUKA 实机实验。

## 配置、来源和真实模型

论文版本为 [arXiv:2504.10783v1](https://arxiv.org/abs/2504.10783v1)，下载件 SHA-256 为 `579c9917425fe01b583f067d0a1c9702917ccd810584dbac562fcdf9fead51d0`。具体核对见 [参数清单](parameter_audit.csv)，原始数值对照见 [paper_vs_actual.csv](paper_vs_actual.csv)。对照表中的论文数值不是本机实验结果。原文代价标记为 cost (S only)，本报告只统计独立验证通过的代价，未确认两种筛选口径等价。原文 Table I 的代价、计时和方法成功率均以原生 DRM 成功为条件（Forest DRM SR=1.00，FR3=0.961）；本报告主要成功率分母为全部 400 个请求，CSV 另给出以本机原生 DRM 成功为条件的比率，不能混淆两个分母。

| 参数 | Forest | FR3 |
|---|---|---|
| 环境 × 路线图种子 × 规模 | 10 × 10 × 4 | 10 × 10 × 4 |
| 路线图节点 | 200/400/800/1600 | 3000/6000/9000/12000 |
| k；dTS/dCS；体素边长；在线步长 | 10；10/10；0.06 m；0.1 | 相同 |
| EI-ZO δ/ε；粒子；混合步数；每轮面数 | 0.05/0.01；1000；30；10 | 0.005/0.005；10000；60；10 |
| EI-ZO configuration margin | 0.01 | 0.01 |
| NTO distance | SNOPT，Nc=20 | SNOPT，Nc=150 |
| NTO time | SNOPT，Nc=100 | IPOPT，Nc=150 |

FR3 来源是 [官方 franka_description](https://github.com/frankarobotics/franka_description/tree/7aeeddc449edf8d62b594f9e36a81da53e7796f9)，固定版本 `7aeeddc449edf8d62b594f9e36a81da53e7796f9`。已从公开地址重新下载 {source_download['files']} 个文件并逐项匹配记录的 SHA-256。真实七自由度运动学、关节范围、手爪与 SRDF 过滤均从公开模型建立；33 个球是确定性新拟合，不能冒充作者原始模型。8 个臂段的网格顶点/面中心探针未覆盖数为 {sum(x['uncovered_probes'] for x in fit['mesh_probe_audit'])}；这不是连续体积覆盖证明。粗球近似可能排除真实机械臂的可行姿态，预检发现的一个常见 Panda 式 home 姿态碰撞也原样保留，未通过增加过滤来消除。

本模型为 **33 个机器人球、32 个活动球、211 个静态候选碰撞对 + 640 个机器人—障碍球对 = 851 对**，与原文 1187 对不同。桥接工具默认重建过滤的问题已用 Drake 的候选对列表显式同步；10,000 个随机配置的独立 Drake/CUDA 比对通过。30 个随机关节配置的官方矩阵链/FK 比对误差阈值为 1e-12。

10 个 FR3 场景固定 20 个障碍球，左右侧端点、障碍半径/分布和随机种子均明确记录为未公开输入的重建假设。目标位姿由独立左侧配置的 FK 产生，然后从其他初值实际求解带碰撞约束的位姿 IK，保留所有尝试。40 个原生 `.map` 路线图在 v2 生成，v3 经模型/生成源码、输入配置、依赖版本和复制后文件哈希核对后复用，参见原始运行的 `cache_provenance.json`；未复用任何方法结果。Forest 保留既有静态圆障碍 plant，因此没有实际构建体素碰撞地图；其初始种子使用原生 DRM，LGCS 第二条路径在同一原生图上用 NetworkX 做阻塞区域后的精确图搜索及线段—圆碰撞检查。这个搜索后端与原生 FR3 第二次规划不同，相关时间计入构建，属于额外可比性限制，不能称为作者原始实现的逐行复刻。

LSCS 使用 Clarabel 替代未获许可的 MOSEK。LGCS 使用真实 Gurobi 整数 SOCP；许可证限制和超时保留，不能解释为几何方法失败或默认为最优解。SCSTO 调用作者固定版本 `d3f885aa2b74a10418de96e61ec2df0bf6ef16b8` 的双凸实现。距离 NTO 是二阶 B 样条，时间 NTO 是四阶 B 样条，两者使用真实 Drake `MinimumDistanceLowerBoundConstraint`；距离下界 0、影响偏移 0.01、均匀 Nc、时间最少 8 个控制点、单位速度/加速度、Bezier 阶数 6 均在配置中公开。论文未完整规定这些转录细节，不声称已经恢复作者 NTO 基线。

## 实际结果

{chr(10).join(table)}

距离方法代价单位为 Forest 米、FR3 弧度；时间方法为秒。代价列仅汇总通过验证的样本，样本集合不同，**不能据此作公平的跨方法代价优劣判断**。各方法实际尝试数不同，不能把边际计时均值直接相除作为公平加速比。时间配对仅使用双方都实际尝试的案例；代价配对要求双方均通过验证。配对记录见 [Forest paired.csv](forest/paired.csv) 和 [FR3 paired.csv](fr3/paired.csv)。所有预定案例仍保留在状态统计和验证成功率中。

TO+CS 包括每次实际求解的几何构建、优化以及恢复；失败尝试仍计入均值。通常排除离线路线图、独立验证、预热和文件写入。异常捕获时只有整次方法墙钟时间可用，`methods.csv` 中 `timing_scope=exception_wall_fallback` 单独标记：Forest {sum(r['exception_wall_timings'] for r in summaries['forest']['methods'])} 次、FR3 {sum(r['exception_wall_timings'] for r in summaries['fr3']['methods'])} 次；这些值不是精确拆分的 TO+CS。样本 SD 使用 ddof=1；汇总还提供按有实际观测的环境（最多 10 个）聚类的 2000 次 bootstrap 区间；各方法实际环境数见 `timing_environment_clusters`，不将 400 个相关案例假定为独立试验。方法顺序固定，计时是本实现/本硬件条件下的观测值。

{chr(10).join(statuses)}

本轮 FR3 的 44 条无效种子中，40 条来自同一端点表示问题：e08 原始 double 目标净距约 9.714e-17 m，原生 float32 目标变为 -9.571e-9 m，关节表示误差约 2.139e-7 rad；另 4 条为路径内部碰撞或原生边界判定差异。主结果未通过放宽阈值或替换目标来修正这些失败。另有两条适配失败：e00_r09_n12000 的 LGCS 第二次 DRM 调用抛出无路径异常，e09_r07_n06000 的作者 SCSTO 初始化抛出 `line start is not contained in region 0`。这些属于当前实现/适配限制，未将其解释为算法固有可靠性。

原生库用异常表示“路线图中不存在无碰撞路径”时，原始 `case.json` 的 error 状态保持不变，汇总另以 `semantic_status=drm_failed_native_exception` 标明规划失败，避免当作未知程序故障。逐例规划失败见 `audit/fr3/case_failures.csv`。

每例失败的求解器状态、碰撞/导数/端点检查和异常原因见 [audit/](audit/) 下的 `failures.csv`；碰撞见证点的穿透深度分布见两个环境下的 `collision_depth_summary.json`。穿透统计使用已保存的负距离见证点，既不更改原判定，也不表示连续碰撞认证。Gurobi 的状态、变量数和 gap 逐例保存在 `lgcs_solver_states.csv`。

## 验证、续跑与吞吐量

曲线按连续导数控制点上界选取采样密度，最大请求关节步长 0.005，曲线至少 2001 点；检查独立几何碰撞、端点、关节域、连续导数控制点速度/加速度上界和 Bezier C0/C1。Forest 折线与二阶样条另外检查精确线段—圆距离。三维曲线没有连续碰撞证书。

{chr(10).join(audit_text)}

Forest 距离 NTO 另有 {audits['forest']['exact_forest_NTO_segment_checks']} 条完整分段精确检查。全部成功标记、密集标志、原始优化系数和检查点一致；源码快照与运行依赖哈希核验通过。运行结束后的测试输出和 `pip check` 见命令清单，不将先前测试失败隐藏。

FR3 吞吐量为 **{through} 百万配置/秒**（50 次测量的样本 SD）：10 个实际场景，每场景同一批 200 万配置重复 5 次，单独预热。计时含公开同步包装器的分配和主机/设备传输，不含采样/写文件；对 {throughput['verified_points']:,} 个预定配置做独立 Drake 核对，差异 {throughput['model_disagreements']}。重复测量不是 50 个独立场景。碰撞对数量和硬件均与论文不同，不能将此值直接换算为论文加速比。

本机 Ubuntu 22.04.5 / i7-13700K / Quadro RTX 8000 / CUDA 12.8；论文为 i9-10850K / RTX 3090 / CUDA 12.6。现有 Conda 前缀 `.conda/envs/csdecomp` 被复用，未为本协议安装新依赖。环境、锁定版本和轮子校验见 [environment.json](../../environment.json)、[conda-explicit.txt](../../conda-explicit.txt)、[requirements.lock.txt](../../requirements.lock.txt)。SNOPT、IPOPT、Clarabel 已实际求解；MOSEK 未启用。新建空环境恢复尚未另行测试。

## 历史版本、原始记录和执行命令

v1 Forest 及辅助 Kinova/吞吐量/消融保留在原 [报告](../reproduction_report.md)。v2 Forest 实际完成 400 例；v2 FR3 实际完成 {old_completed} 例后人工中止，另有未完成状态保留。v2 最少 4 个时间样条控制点在静止端点下仅能沿直线移动；修正为至少 8 个控制点后冻结 v3，重新运行两组各 400 例。v2 结果只用于转录诊断，未混入 v3，也未据论文目标数值筛选配置。v2 快照和中止原因保存在对应 manifest，诊断汇总在 [diagnostics_v2/](diagnostics_v2/)。

| 运行 | 原始目录 | 退出码 / 状态 |
|---|---|---|
"""
        + "\n".join(
            f"| {r['label']} | `{Path(r['run_dir']).relative_to(ROOT)}` | {r['exit_code']} |"
            for r in ledger
        )
        + f"""
| FR3 throughput | `{Path(index['fr3_throughput']).relative_to(ROOT)}` | completed |

实际完整命令、时间和日志见 [v3 命令清单](../../paper_protocol_v3_commands.json)、[v2 命令清单](../../paper_protocol_commands.json) 与 [验证命令清单](../../paper_protocol_verification_commands.json)。主运行入口：

```bash
bash reproduction/setup.sh
bash local/python.sh -m reproduction.paper_protocol.run run --environment forest
bash local/python.sh -m reproduction.paper_protocol.run run --environment fr3
bash local/python.sh -m reproduction.paper_protocol.run resume --run-dir <RUN_DIR>
bash local/python.sh -m reproduction.paper_protocol.analyze --run-dir <FOREST_RUN_DIR> <FR3_RUN_DIR> --output reproduction/reports/paper_protocol --audit --collision-depths
bash local/python.sh -m reproduction.audit_paper_protocol --run-dir <FOREST_RUN_DIR> <FR3_RUN_DIR> --output reproduction/reports/paper_protocol/audit --resume
```

`manifest.json`、`source_snapshot/`、输入场景、原生地图、每轮凸区域、候选系数、采样回放、日志与失败均在 `runs/`。原始运行未加入普通 Git 提交；压缩 SHA-256 清单随报告保存。绘图只读取保存结果，使用所有预定案例；[对比图](../../figures/paper_protocol/paper_results.pdf)、[完整状态图](../../figures/paper_protocol/paper_case_statuses.pdf)、[FR3 回放](../../figures/paper_protocol/fr3_scene_replay.pdf) 不重跑规划器。回放按字典序选择第一个案例，不按成功结果挑图。

## 限制与下一阶段重点

1. 优先获得原始 33 球模型/过滤、题单及 NTO 转录，才能开展与原文相同输入下的数值复现。当前模型的 851/1187 对差异和未公开分布是主要可比性限制。原生 DRM 成功率的明显差距也说明不能将重建结果当作已复现原文结果。原始端点与 float32 路径端点的净距核对保存在 `audit/*/seed_endpoint_audit.csv`。
2. 在固定预选题单上单独分析 NTO 的最少控制点、初始化、影响偏移及碰撞裕量；保留所有参数和失败，不用单一成功配置替换主结果。当前的求解器成功不等于无碰撞轨迹。
3. 完整 MOSEK/Gurobi 许可可减少替代求解器和受限模型尺寸的影响；目前的许可证失败仍计入分母。需要再核对方法整体延迟的组成，不能用 TO+CS 当作端到端机器人响应时间。
4. 真正 KUKA + 三台 D415 的传感、控制与执行记录未具备，本次未生成实机成功率、823.4 ms 延迟或其替代数据。更高保真的碰撞体与连续验证也仍需独立工作。
"""
    )
    atomic(
        out / "acceptance.json",
        dict(
            protocol="paper_parameters_v3",
            planned_cases={
                family: summary["requested"] for family, summary in summaries.items()
            },
            all_primary_drivers_completed=all(
                summary["manifest_status"] == "completed"
                for summary in summaries.values()
            ),
            method_slots=sum(
                summary["requested"] * len(summary["methods"])
                for summary in summaries.values()
            ),
            actual_method_attempts=sum(
                r["attempted"]
                for summary in summaries.values()
                for r in summary["methods"]
            ),
            validated_method_results=sum(
                r["validated_successes"]
                for summary in summaries.values()
                for r in summary["methods"]
            ),
            source_and_artifact_audits_passed=all(
                not summary["source_integrity_issues"]
                and not summary["artifact_issues"]
                and not audits[family]["issues"]
                for family, summary in summaries.items()
            ),
            independently_rechecked_points=sum(
                a["independently_rechecked_fixed_points"] for a in audits.values()
            ),
            whole_paper_exactly_reproduced=False,
            unresolved=[
                "original FR3 collision model and random inputs",
                "original full NTO transcription",
                "MOSEK license and unrestricted Gurobi",
                "KUKA/D415 hardware execution",
                "continuous collision certification",
            ],
            report="reproduction_report.md",
        ),
    )
    (out / "reproduction_report.md").write_text(content)
    print("Saved", out / "reproduction_report.md")


if __name__ == "__main__":
    main()
