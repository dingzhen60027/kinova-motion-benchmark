# 论文方法 baseline 的可用性与 LGCS 许可限制

审查日期：2026-10-28。当前仓库 HEAD 为 `ee1b5bed395a5c471f60dee760502b10d74f2844`；被审查实验的冻结计算源码为 `72984027030175b7bca78e2dcf592b38f0c1864b`。本次只读取既有实验，并在既有 Conda 环境执行小型 CPU 求解器探针；没有调用 GPU、修改依赖、改变输入或重跑规划实验。实际探针输出保存在 [readiness_probe.log](readiness_probe.log)。

## 主比较保留五种方法，按目标分表

| 论文方法标签 | 当前真实执行路径 | 适合作为后续 baseline 的范围 |
| --- | --- | --- |
| LSCS | 原生 DRM → 原生 EI-ZO → 作者 SCSPlanning polygonal SOCP，Clarabel | 可用。作者代码复用，MOSEK 改为 Clarabel 的求解器适配必须注明。 |
| LGCS | 相同原生前端；论文 VI-C 第二路径尝试；本地转录的 perspective GCS MISOCP，Gurobi | 有条件可用。不是作者发布的 LGCS 优化代码；本机规模受限许可证覆盖不完整。 |
| NTO-distance | 相同原生 DRM 种子 → Drake 二阶 B-spline / PWL，SNOPT | 可用的论文配置重建；原始 NTO 转录未公开，配点约定和本地求解设置必须注明。 |
| SCSTO | 相同原生 DRM → 原生 EI-ZO → 作者 SCSPlanning biconvex，Clarabel | 可用。作者算法代码复用，当前 Kinova 几何、软件版本、恢复与预算协议仍是适配。 |
| NTO-time | 相同原生 DRM 种子 → Drake 四阶 B-spline，IPOPT | 可用的论文配置重建；保留 DRM 节点、端点重复控制点及单位速度/加速度限制。 |

距离表只比较 LSCS、LGCS、NTO-distance；时间表只比较 SCSTO、NTO-time。不把 rad 与轨迹秒数混排成一个排名。旧加强版 `NTO` 如需加入，应作为额外方法，不能取代本轮 `NTO_PAPER` 后沿用论文标签。以上是**论文方法在固定 Kinova benchmark 上的适配基线**；它们不补齐作者 FR3 球模型、原题单或 KUKA 实机记录，也不等于原论文全部实验的精确复跑。

当前完整 dev 是 60 个固定问题 × 3 个 6000 节点路网 × 5 种方法，共 900 次。五方法的输入、前端结局和成功种子数组均匹配。20 个不同障碍场景才是场景级统计单位；同场景问题及重复路网不能当作独立环境。详见 [完整方法报告](../benchmark_v1_full_baseline.md) 和 [完成凭证](../full_baseline/completion.json)。

## LGCS 的 16 次失败是许可证阻塞

| 分层统计 | 当前实际数值 | 正确解释 |
| --- | ---: | --- |
| 合法预定查询 | 180 | 总体分母固定，不删失败题。 |
| 原生 DRM 返回成功 | 111 | 未通过数值 seed 验证的 1 条不能进入优化器。 |
| 严格 seed 准入 | 110 / 180 = 61.11% | 所有五方法共享此门槛。其余为 68 次原生规划失败、1 次 seed 拒绝和 1 次原生进程断言。 |
| 准入后商业后端实际支持 | 94 / 110 = 85.45% | 16 个构建后的 MISOCP 被许可证拒绝。前端失败的 70 题没有实际构建 LGCS，因此其后端支持性未知。 |
| 准入后许可证阻塞 | 16 / 110 = 14.55% | 保存的异常均为 Gurobi `10010`，不是无解证明或优化算法失败。 |
| 支持子集上的数值验收成功 | 94 / 94 | 仅适用于实际支持的模型规模，不能外推到另外 16 个模型。 |
| 本机交付已验证路径 | 94 / 180 = 52.22% | 是当前安装、许可、前端共同决定的端到端交付率，不是 LGCS 算法可靠性上限。 |

这 16 个案例分布为 bottleneck 10、route_choice 4、dense_clutter 1、sparse_clutter 1；不是随机缺失。保存模型有 148–316 个变量、1868–3551 个线性约束、5–10 个区域。既有变量超过二次模型许可上限的案例，也有变量少于 200 但线性约束超过 2000 的案例。官方说明给出的受限许可上限包括 2000 个线性约束，以及含二次项时 200 个变量；与本机异常一致。[Gurobi 官方许可说明](https://support.gurobi.com/hc/en-us/articles/360051597492-How-do-I-resolve-a-Model-too-large-for-size-limited-Gurobi-license-error)

严格审计核对了全部 180 份记录，包括提交的 MPS、失败模型规模和所有成功门控。94 个成功均保存 `solver_status=2` 和 `globally_optimal=true`；该最优性仅针对已生成的有限区域图。33 次第二路径被实际接受，20 次恢复中有 9 项最终成功。详见 [独立 LGCS 审计](../full_baseline/independent_lgcs_dev_v1/audit.json)。

主表应该并列给出“交付率”“前端准入”“许可证阻塞”“实际优化/验收失败”，而不是只写一列 success rate。现有 LSCS−LGCS 的 8.89 个百分点差值正好等于 16/180，不能据此宣称 LSCS 优化算法更可靠。许可失败的快速返回也不能充当 LGCS 的优化速度优势。

配对分析应按 `(case_id, roadmap_id)` 固定配对。关于当前环境部署，可以保留全部 180 题的交付率和耗时，并明确其系统含义。关于算法质量，应另外报告实际共同支持的子集、子集覆盖及各族分布；同一支持子集上所有方法采用相同分母，不能各自删除失败。成本只能在双方均返回且通过验收的案例配对比较，明确这是成功条件下的描述。场景整组 bootstrap 保留场景内关联，不能消除许可证带来的选择偏差。不得给另外 16 题填零成本、推测解或填入 LSCS 结果。

## DRM seed 可以作为额外距离参考

可以从这批原始记录建立 `DRM-PWL` 前端参考。已有 110 个不同 `(case_id, roadmap_id)` 的 seed 实际通过统一的 CSD 与独立 Drake 数值碰撞检查；五方法共 550 份通过的 seed 轨迹逐数组一致，但只对应 110 个查询。原始 `path.npy`、`frontend_validation/validation.json`、采样和阶段计时均已保存。本次只读重算折线总长与原验证成本相符，均值为 **7.219790162827398 rad**。

使用时固定抽取每个查询的 LSCS 前端记录，抽取条件是 `frontend_success`，不依赖最终优化器成功。未准入的 70 个查询继续保留原状态。种子是否可行来自它自己的真实独立数值验证，而非因为某个优化器后来成功。代价直接计算 `sum(norm(q[i+1]-q[i]))`；不能把它标为 LSCS/LGCS 优化结果，也不能称作全局最短路。

时间必须与参考的工作范围匹配。原 LSCS 记录中，`online_collision_map_seconds + drm_path_seconds + drm_path_validation_seconds` 的已准入子集 p50 为 **0.7294736949261278 s**。这是原运行的**前端组件计时之和**，含数值 seed 验证，未含启动、模型载入、地图载入及未单独计时的输出/阶段开销；不是重新测量的独立 DRM 冷启动或热启动耗时。不能拿完整 LSCS/LGCS 耗时归给 DRM，也不能直接与完整算法冷耗时作公平速度比。若需要正式独立 DRM 延迟，应另设版本明确的独立 runner，真实运行并保留外层墙钟；本次没有实施。

DRM 输出是几何折线，没有满足单位速度/加速度和起终零速度条件的物理时间参数化。因此它只进入距离参考表，不能直接进入 minimum-time 表。有限碰撞采样通过也不是数学连续安全证明。

## 已安装的开源求解路径

本次在原 Conda Python 中实际读取：CVXPY **1.7.5**；`installed_solvers()` 返回 `CLARABEL, GUROBI, MOSEK, OSQP, SCIPY, SCS`。列表代表接口存在，不能替代许可和实际求解验证。`pyscipopt`、`ecos`、`highspy`、`ortools`、`cvxopt`、`cylp` 均未安装；PATH 中未找到 `scip`、`highs`、`cbc`。Drake 中的 IPOPT 与命令行 `ipopt` 是否存在是两回事。

| 实际 CPU 探针 | 结果 |
| --- | --- |
| 连续 SOCP：最小化 t，`norm(x) <= t, x >= 1`，Clarabel | `optimal`，1.4142135626672818 |
| 同一连续 SOCP，SCS | `optimal`，1.414213562461647 |
| 加入真实 binary 变量的 SOCP，Clarabel | 明确拒绝：不是 MIP-capable |
| 同一 mixed-integer SOCP，SCS | 明确拒绝：不是 MIP-capable |
| 同一 mixed-integer SOCP，SCIPY | 明确拒绝：不支持 SOC 锥 |

因此当前环境没有已经安装并验证可直接承接 LGCS 的开源 MISOCP 后端。Clarabel/SCS 可解连续锥问题，不能通过删除整数约束就冒充原 LGCS；SciPy/HiGHS 的 MILP 能力也不等于混合整数二阶锥能力。以上判断由本机实际错误和官方能力表共同支持。[CVXPY 官方求解器能力表](https://www.cvxpy.org/tutorial/solvers/index.html)

可研究的官方开源路线是 **SCIP + PySCIPOpt**。SCIP 当前为 Apache-2.0，其官方文档支持 mixed-integer nonlinear/quadratically constrained 模型，CVXPY 列有 SCIP 的 SOCP 与 MIP 能力；PySCIPOpt 官方提供 Conda 安装方式。当前没有安装或验证这条路线，本报告不把它记为可用结果。[SCIP 许可证](https://github.com/scipopt/scip/blob/master/LICENSE)、[SCIP 支持问题](https://www.scipopt.org/doc/html/WHATPROBLEMS.php)、[PySCIPOpt 官方安装说明](https://pyscipopt.readthedocs.io/en/latest/install.html)

若以后实现，应另命名 `LGCS-SCIP-adapted`，保留二元流、perspective membership、欧氏长度锥、相同区域池、相同端点和预算，并验证提交模型、可行解、数值残差、界、gap、超时和恢复。首先在现有 Gurobi 可解模型上进行同模型交叉核对，再处理原来的 16 个受限模型；不能只验证小探针就声称完整替代成功。商业 Gurobi 原记录应保留独立标签，不能静默把两种求解器结果混成一次实验。小图穷举路径再用 Clarabel 求解可以做独立正确性 oracle，但截断枚举不等于完成原 LGCS 全图优化，不能据此补写成功。

## 当前结论与下一步的边界

五方法已经具备可审计的 Kinova 开发 baseline，LGCS 的商业规模支持不完整，必须在方法比较中显式隔离。最直接的新增参考是从现有真实证据派生的 `DRM-PWL` 路径质量；独立 DRM 计时或开源 LGCS 后端都需要新实现/新运行。本次只完成可用性检查，不修改正式输入，不解锁 420 个 test，不宣称已完成新的算法或正式评测。

实际探针使用 `bash local/python.sh`，没有更换 Python 管理方式。日志中的代码读取 `importlib.util.find_spec`、`shutil.which` 与 `cvxpy.installed_solvers()`，分别调用 `Problem.solve(solver=...)` 测试连续及混合整数 SOCP；种子检查直接读取封存 JSON/NumPy 文件。没有通过 solver-success 过滤或修改任何任务。
