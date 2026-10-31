# 加强基线与固定条件验收

2026-10-27 · 版本 `v1_20261027` · 开发验收，非正式论文评测

本轮已实现并实际运行 `NTO_distance_strong` 和 `NTO_time_strong`，接入配置、单题/批量执行、冻结输入、续跑、验证、汇总和回放。旧 NTO 与阶段一代码、报告、原始结果保留。6 个固定开发场景中，距离 NTO 的严格通过数从 **2/6 到 5/6**，时间 NTO 从 **3/6 到 6/6**。仍有一个加强距离基线求解失败；其初值可行，不能将失败解释为题目无解。

这里的“加强”是修正基线数值表达和验证中的已知薄弱处，并建立可复用的实验条件，不构成新算法贡献，也不表示复现了作者未公开的 benchmark。

## 实现与共同条件

完整契约见 [Baseline conditions](../../../docs/baseline_conditions.md)。

| 项目 | 本次固定条件 |
|---|---|
| 同题输入 | 相同模型、障碍、碰撞对、起终点和共享 DRM 初始路径；FR3 IK 只求一次 |
| 距离比较 | DRM、LSCS、LGCS、旧/强 NTO_distance；Forest 单位 m，FR3 为关节空间欧氏长度 rad |
| 时间比较 | SCSTO、旧/强 NTO_time；目标为持续时间 s，分量速度/加速度上限均为 1，起终点静止 |
| 共同外层预算 | 每个方法进程 120 s，包含启动、模型构造、求解、终检；清理宽限另记 |
| 强 NTO 内层预算 | 累计 30 s，覆盖构造、求解、验证及所有细化；不按轮重置 |
| 强 NTO 配置 | 时间样条最少 16、最多 64 控制点；初始 150、最多 1000 碰撞位置；每轮最多补 64；最多 5 次细化 |
| 余量与容差 | 优化内部目标 1 mm；求解可行性容差 1e-5；共同终检不豁免负净距 |
| 规划与验证分辨率 | lazy edge 为 0.01，独立验证为 0.002；Forest 另做全参数段数值最小净距检查 |
| 求解器 | Forest 距离/时间及 FR3 距离使用 SNOPT，FR3 时间使用 IPOPT；不按题更换 |

30 s 不是全部旧方法统一执行的累计上限：旧方法保留其单次求解选项，LSCS/SCSTO 依靠外层 watchdog。因此共同强制比较边界是 120 s。共享在线规划/IK 的耗时单列，解释端到端耗时需要一致地计入。原生求解调用可能超过剩余内层时间才返回；这种结果记为预算耗尽，不能以成功结束，外层进程边界负责强制终止。

加强距离基线对 Forest 的每条线段加入完整的线段—圆净距约束；FR3 采用 Drake 碰撞约束并依据验证见证补点。加强时间基线从旧 NTO 的同一条端点静止曲线开始，通过保形插结点增加自由度，再按验证结果补约束、重求解。每轮保存初值、结点、求解参数、状态、候选、终检与停止原因。新旧版本的有限维优化问题明确不同；物理任务、目标和最终通过标准保持相同。

Forest 的所有方法统一增加分段多项式驻点净距检查，覆盖折线、B-spline 和 Bezier，数值未决直接拒绝。FR3 继续使用 native CUDA 与独立 Drake 几何检查。内部 1 mm 是强基线的数值缓冲，不是其他方法额外承担的任务约束；不兼容的固定端点直接报告失败，不移动端点。FR3 未计算全曲线最小净距时明确保持未知。

## 实际运行及环境

在现有 `main`、起始 HEAD `fc4265b11fd67baddac8201595851d6d6edd6b47` 上实施，没有新建分支。运行时工作树含本轮改动，所以每次运行另外冻结全部计算源码及 SHA-256，不能仅凭起始 HEAD 复现。

当前 origin 是作者的 `wernerpe/csdecomp`，GitHub 核查仅有 `READ` 权限，见 [remote_access.json](remote_access.json)。本轮保存在本地 main；未向作者仓库推送或强推。

复用项目 Conda `.conda/envs/csdecomp`：Ubuntu 22.04、Python 3.10.22、Drake 1.51.1、NumPy 2.2.6、SciPy 1.15.3、CSDecomp 0.1.2、Quadro RTX 8000、驱动 570.211.01。线程设置为 OMP=4、OpenBLAS=1、MKL=1。完整包版本、GPU 和数值二进制摘要见 [环境清单](v1_20261027/exports/fr3_environment.json)，实际 CUDA/求解器探针见 [doctor.json](v1_20261027/doctor.json)。

实际执行入口：

```bash
bash local/python.sh research_platform/reports/strengthened_baselines/acceptance_driver.py \
  --label v1_20261027

bash local/python.sh research_platform/reports/strengthened_baselines/bundle_evidence.py \
  --label v1_20261027 --pytest-artifacts /tmp/pytest-of-eric/pytest-210
```

逐条命令、开始时间、退出状态、实测耗时见 [commands.json](v1_20261027/commands.json)，对应 stdout/stderr 日志在同目录。重新运行应使用新 label；打包脚本的 pytest 参数应指向该次测试的实际临时目录，不能复用别次测试的原始输出。

`pytest tests/research_platform -q` 实际 **208 passed**；doctor、三组配置核查、run、verify、replay、resume 均成功。三组 resume 分别核对 219、757、389 个 job 文件，前后 SHA-256 完全不变；历史三组平台验收的完整性验证也通过。见 [验收检查](v1_20261027/checks.json) 和 [输入/源码核对](v1_20261027/exports/input_source_checks.json)。命令成功只表示实验执行完整，不能把其中的数值失败算作方法成功。

## 固定开发集结果

预先固定 Forest seeds 10–13 / roadmap seed 0 / 200 节点，FR3 seeds 10–11 / roadmap seed 0 / 3000 节点，共 6 题、48 个方法 job。之前 0–9 已用于诊断；本组也是开发验收，不是未见测试集。没有按结果替换场景、筛除失败或择优重试。

| 方法 | Forest（4 题） | FR3（2 题） | 合计严格通过 |
|---|---:|---:|---:|
| DRM | 4/4 | 2/2 | 6/6 |
| LSCS | 4/4 | 2/2 | 6/6 |
| LGCS | 4/4 | 2/2 | 6/6 |
| SCSTO | 4/4 | 2/2 | 6/6 |
| NTO_distance | 2/4 | 0/2 | 2/6 |
| NTO_distance_strong | 4/4 | 1/2 | 5/6 |
| NTO_time | 2/4 | 1/2 | 3/6 |
| NTO_time_strong | 4/4 | 2/2 | 6/6 |

严格通过同时要求方法完成、求解器成功和共同终检通过。开发集共 40/48 成功；失败候选的成本仍在原始记录中，但不进入成功成本统计。完整数据：[Forest 汇总](v1_20261027/exports/forest/summary.html)、[FR3 汇总](v1_20261027/exports/fr3/summary.html)、[全部逐题 CSV](v1_20261027/exports/all_jobs.csv)。

时间方法的同题持续时间如下，破折号表示该方法未严格通过，不能以其无效轨迹参与成本比较：

| 场景 | SCSTO / s | 旧 NTO_time / s | 强 NTO_time / s |
|---|---:|---:|---:|
| Forest 10 | 11.658045 | 23.316090 | 8.862677 |
| Forest 11 | 9.339212 | — | 8.864062 |
| Forest 12 | 11.861078 | — | 9.864791 |
| Forest 13 | 11.658045 | 23.316090 | 8.862677 |
| FR3 10 | 6.128354 | 7.091483 | 5.795255 |
| FR3 11 | 3.428525 | — | 2.857919 |

这是轨迹持续时间，不是计算耗时。计算耗时包含失败，并保存在逐题 CSV 中；不能用不同成功子集的均值宣称总体速度或质量优势。Forest 10/13 的障碍不同，但共享直线 seed 相同、部分持续时间近似相同；这组小样本不支持独立同分布或统计显著性推断。

## 单列的 Figure 6 回归

已知 Figure 6 场景单列 8 个 job，6 个严格通过：DRM、LSCS、LGCS、SCSTO 及两个强 NTO。旧距离 NTO 为 `validation_failed`，旧时间 NTO 为 `solver_failed`。强距离长度 **11.127189 m**，强时间持续时间 **10.497446 s**；SCSTO 为 **11.504054 s**。该已知题不并入上述 6 题开发集。

参见 [回归汇总](v1_20261027/exports/regression/summary.html) 和 [轨迹回放](v1_20261027/exports/regression/replay/e000_r000_n00200/replay.html)。三个 run 总计 56 个方法 job、46 成功、10 失败，全部保留。

## 机制对照与失败诊断

以下数值来自本次 208 项测试中的真实 SNOPT/IPOPT 求解，原始文件另存于 [solver_controls](v1_20261027/exports/solver_controls/provenance.json)，不取模拟状态测试的输出冒充实验。

* 无障碍、距离 1、速度/加速度上限均为 1、起终点静止：4 控制点时间为 **3.000000 s**，16 控制点为 **2.005229 s**；物理下界为 **2 s**。初始曲线保形误差 **1.11e-16**。IPOPT 独立求解同样得到 **2.005229 s**，并通过导数包络检查。它说明原有限维表达存在保守性，不能据此声称普遍全局最优。
* 刻意只有一个初始碰撞约束点的窄圆诊断：首次最小净距 **−0.020000 m**，验证拒绝；两次细化后依次 **0.000979330 m** 和 **0.000999663 m**，最终通过。该诊断的初始采样数与开发集的 150 明确不同，目的是触发并核查补点机制。
* Forest 完整线段约束实际得到 **0.000999923 m** 净距；固定两节点穿障碍的不可行对照保留 `solver_failed`，未退回 seed 冒充成功。

唯一失败的加强基线是 **FR3 seed 11 / NTO_distance_strong**，6 轮后仍为 `solver_failed`，累计内层 **21.941950 s**，最后为迭代上限。初始完整离散程序的最大正约束违反为 0，5001 点最小采样净距为 **22.4648 mm**；六轮求解器初值与保存控制点一致，抽查碰撞梯度与中心差分最大差约 **2.22e-8**。SNOPT 首轮离开可行初值后产生约 **92.6 mm** 穿透，后续对失败候选保形插结点并继续热启动，未能恢复。

只读诊断未发现初值遗漏或明显梯度接线错误，也没有通过对照求解证明是哪一种恢复策略导致失败。旧 NTO 在同题虽返回求解成功，密集独立检查存在 **0.06776 mm** 点间穿透，因此也不能算成功。详细实际命令、逐轮数据和限制见 [FR3 失败诊断](fr3_failure_diagnosis/report.md)。全部失败索引见 [failures.json](v1_20261027/exports/failures.json)。

## 可移植证据与后续边界

[raw_evidence.tar.gz](v1_20261027/raw_evidence.tar.gz) 保存三组原始 run、冻结源码、模型/场景/roadmap、轨迹、每轮求解及验证记录、失败和回放，以及实际机制测试输出，共 **1683 个文件**。打包后逐个读取并核对 SHA-256；归档摘要为 `25111ef55fac3acbb0a7562e12f6e9c5bdb8f0999ff44e9e0894db5669555b53`，见 [归档收据](v1_20261027/archive_receipt.json) 与 [逐文件清单](v1_20261027/archive_inventory.json)。另有 [独立最终审计](independent_final_audit.md)。

恢复到一个新目录，避免覆盖现有 run：

```bash
mkdir -p /tmp/csdecomp_baseline_evidence
tar -xzf research_platform/reports/strengthened_baselines/v1_20261027/raw_evidence.tar.gz \
  -C /tmp/csdecomp_baseline_evidence
bash local/research.sh verify \
  --run /tmp/csdecomp_baseline_evidence/runs/v1_20261027_fr3
```

本次已实际恢复到新的临时目录，并对三组恢复后的 run 分别执行 `verify`，全部通过；命令和退出码见 [恢复验证记录](v1_20261027/archive_restore_verification.json)。

当前基线可用于受控的后续研究，但仍有明确边界：Forest 全段检查是 float64 数值方法，不是精确或区间证书；FR3 仍为有限采样、重建的球/盒模型，不是实体机器人连续安全保证；加强距离 NTO 尚缺统一的可行暖启动恢复策略；这套同 seed 比较未覆盖多初值、全局规划器或所有先进优化器。下一版本应优先检验一般性的可行恢复规则、FR3 更可靠的全段验证，以及预先分离开发与测试任务，保持相同预算和完整失败分母。现有结果不足以支持新的谱算法或论文优势结论。
