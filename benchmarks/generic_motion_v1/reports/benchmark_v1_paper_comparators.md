# 论文比较方法：Kinova 通用规划基准实际测试

论文 VII-A 的完整规划比较组已经在同一套固定 Kinova 题目上对齐：**最短距离为 LSCS、LGCS、NTO-SNOPT，最短时间为 SCSTO、NTO-IPOPT**。本次新增运行原始 NTO 的两个目标，使用相同代码快照下已封存的 LSCS、LGCS、SCSTO 结果进行配对比较。

**工作量口径：380 个新注册槽位（376 次合法输入调用、4 条非法输入记录）＋950 个复用槽位＝1330 条记录。后者包括单独呈现的 refined 工程扩展。**这不表示本次新执行了 1330 次规划。全部记录为 smoke/dev；本轮没有调用正式 test。

## 固定实验口径

开发集是 60 个预先冻结问题、20 个独立障碍场景，四族各 15 题；每题使用 6000 节点路网的 0、1、2 三个种子，故每种方法 180 个合法调用。smoke 另有 10 条固定记录，其中 8 题合法、2 条生成失败。同一任务的模型、障碍球、端点、预算、DRM 路径 SHA 与前端结果逐项相同。所有方法共用 60 秒外部冷启动墙钟预算，保留失败和超时。

环境保持现有 Ubuntu 22.04 / Conda；GPU 记录为 `Quadro RTX 8000, 570.211.01, 49152 MiB`，Drake 1.51.1、CSDecomp 0.1.2、SCSPlanning 0.1.0。LSCS 调用作者公开的凸优化实现（本机 Clarabel，论文 MOSEK），LGCS 使用相同透视图模型（本机 SCIP，论文 Gurobi），SCSTO 调用作者公开的交替凸优化实现；NTO 距离目标使用 SNOPT 二阶 B 样条，时间目标使用 IPOPT 四阶 B 样条。NTO 原始组固定零距离下界、149 个内部 k/150 碰撞绑定和单次求解；没有借用 refined 的 1 mm 裕度、增点或追加求解。已保存参数的 226 次原始 NTO 构造均通过逐项审计；其余记录在求解参数保存前退出，仍保留在分母。

## 原论文五种方法：开发集主结果

| 方法 | 合法成功/调用 | 成功率及场景 95% CI | 冷启动秒数：均值 / P50 / P90 / P95 | 热查询均值秒数 | 成功轨迹代价均值 |
| --- | --- | --- | --- | --- | --- |
| LSCS | 110/180 | 61.1% [41.7, 79.5] | 1.941 / 1.817 / 3.276 / 4.320 | 1.213 (n=180) | 6.373 rad (n=110) |
| LGCS | 110/180 | 61.1% [41.7, 79.5] | 4.335 / 1.842 / 10.174 / 18.263 | 3.631 (n=180) | 6.226 rad (n=110) |
| NTO-distance / SNOPT | 35/180 | 19.4% [8.9, 31.7] | 5.278 / 1.892 / 13.582 / 20.425 | 4.570 (n=180) | 4.675 rad (n=35) |
| SCSTO | 110/180 | 61.1% [41.7, 79.5] | 2.053 / 2.117 / 3.620 / 3.871 | 1.326 (n=180) | 5.799 s (n=110) |
| NTO-time / IPOPT | 88/180 | 48.9% [32.2, 66.1] | 4.594 / 2.368 / 7.701 / 14.162 | 2.943 (n=177) | 6.816 s (n=88) |

冷启动统计覆盖全部合法调用，包含前端失败、求解失败与超时；热查询统计覆盖有实际计时的合法调用。轨迹代价只针对经过数值验证的成功输出：距离为关节空间 PWL 弧长（rad），时间为满足固定速度/加速度约束的轨迹时长（s）。不能把两个不同目标的代价直接比较，也不能仅按各自成功集合的均值排名。

成功率 CI 按场景聚类 bootstrap（PCG64、2000 次、seed=45827），同一场景的多个端点与三个路网种子一起重采样；180 次调用不是 180 个独立场景。冷延迟的 P50/P90/P95 描述已观测调用分布，未把重复路网种子当成独立环境来估计不确定性。

| 方法 | 前端失败 | 求解失败 | 碰撞失败 | 超时 | 验证未决 | 依赖不支持 | 基础设施错误 | 其他 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| LSCS | 70 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| LGCS | 70 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| NTO-distance / SNOPT | 70 | 30 | 45 | 0 | 0 | 0 | 0 | 0 |
| SCSTO | 70 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| NTO-time / IPOPT | 70 | 1 | 18 | 3 | 0 | 0 | 0 | 0 |

| 原始 NTO | 最终分类 | 次数 | 原生求解返回状态 | SNOPT info / IPOPT status |
| --- | --- | --- | --- | --- |
| NTO-distance | collision_failed | 35 | SolutionResult.kSolutionFound | 1 |
| NTO-distance | solver_failed | 7 | SolutionResult.kInfeasibleConstraints | 13 |
| NTO-distance | collision_failed | 10 | SolutionResult.kSolutionFound | 3 |
| NTO-distance | solver_failed | 10 | SolutionResult.kIterationLimit | 31 |
| NTO-distance | solver_failed | 6 | SolutionResult.kIterationLimit | 32 |
| NTO-distance | solver_failed | 5 | SolutionResult.kSolverSpecificError | 41 |
| NTO-distance | solver_failed | 1 | SolutionResult.kSolverSpecificError | 43 |
| NTO-distance | solver_failed | 1 | exception | — |
| NTO-time | collision_failed | 18 | SolutionResult.kSolutionFound | 0 |
| NTO-time | solver_failed | 1 | SolutionResult.kIterationLimit | 1 |

上表只列本轮已写回的原始求解器失败/未通过验证记录；未写回原生状态的超时仍计入前表。`collision_failed` 指输出被独立碰撞验证拒绝，不等同于求解器报告失败；原生求解器的不可行状态也不能证明整个运动规划任务无解。原始组本轮有 1 条原生求解器异常返回，按实际最终分类保留，详细 error 与 traceback 位于对应 `nto_solver_result.json`，不会套用 refined 组的异常统计。另有 113 份原生 SNOPT 日志明确打印总迭代 10000、主迭代 2000、次迭代 500，与原始组默认参数一致。

## 同一任务的配对比较

| A − B | 合法配对 / 共同成功 | 成功率差 pp [95% CI] | 共同成功代价差 [95% CI] | 全部合法冷延迟差秒 [95% CI] |
| --- | --- | --- | --- | --- |
| LSCS − LGCS | 180 / 110 | 0.000 [0.000, 0.000] | 0.147 [0.064, 0.254] | -2.394 [-4.276, -0.937] |
| LSCS − NTO-distance | 180 / 35 | 41.667 [26.667, 57.778] | 0.010 [0.000, 0.039] | -3.337 [-5.076, -1.799] |
| LGCS − NTO-distance | 180 / 35 | 41.667 [26.667, 57.778] | 0.010 [0.000, 0.039] | -0.943 [-2.695, 0.686] |
| SCSTO − NTO-time | 180 / 88 | 12.222 [5.000, 20.000] | -1.303 [-2.244, -0.410] | -2.541 [-4.096, -1.153] |

所有差值方向为 A − B。成功率和冷延迟差以全部 180 个合法配对为分母；代价差仅限双方共同成功，距离单位 rad，时间单位 s。共同成功是条件子集，不能外推为全任务最优性结论。

| 方法 | 直连发生碰撞：合法成功 / 合法调用 | 直连未检出碰撞：合法成功 / 合法调用 |
| --- | --- | --- |
| LSCS | 75/141 | 35/39 |
| LGCS | 75/141 | 35/39 |
| NTO-distance / SNOPT | 0/141 | 35/39 |
| SCSTO | 75/141 | 35/39 |
| NTO-time / IPOPT | 53/141 | 35/39 |
| NTO-refined-distance / SNOPT | 52/141 | 35/39 |
| NTO-refined-time / IPOPT | 65/141 | 35/39 |

直连标签在任何方法运行前固定。上表把原始五方法与单独命名的 refined 扩展按同一标签分层，便于判断成功轨迹代价究竟覆盖了哪些任务。若某方法主要在直连未检出碰撞的子集成功，其较低的成功条件代价不能解释为在需要避障的全部任务上更优。直连发生碰撞只否定该条直连路径；未证明任务无解，也没有为任一方法重新选择端点。

## 障碍族与独立无障碍对照

| 工作空间障碍族 | LSCS | LGCS | NTO-distance | SCSTO | NTO-time |
| --- | --- | --- | --- | --- | --- |
| sparse_clutter | 18/45 | 18/45 | 0/45 | 18/45 | 11/45 |
| dense_clutter | 8/45 | 8/45 | 2/45 | 8/45 | 7/45 |
| bottleneck | 45/45 | 45/45 | 12/45 | 45/45 | 42/45 |
| route_choice | 39/45 | 39/45 | 21/45 | 39/45 | 28/45 |

| 方法 | smoke 障碍合法成功 / 合法调用 | smoke 无新增障碍成功 / 合法调用 |
| --- | --- | --- |
| LSCS | 2/6 | 1/2 |
| LGCS | 2/6 | 1/2 |
| NTO-distance | 0/6 | 1/2 |
| SCSTO | 2/6 | 1/2 |
| NTO-time | 1/6 | 1/2 |

开发集没有 open_control，因此不伪造开发集无障碍统计。smoke 的两条 open_control 共享同一个基础世界，单独展示，不加入障碍主套件，也不报告基于多个独立场景的 CI。bottleneck 和 route_choice 是工作空间几何类别；未证明 C-space 同伦类别或随机题可行性。

## 工程增强 NTO：单独结果

| 工程扩展 | 合法成功/调用 | 成功率及场景 95% CI | 冷启动秒数：均值 / P50 / P90 / P95 | 热查询均值秒数 | 成功轨迹代价均值 |
| --- | --- | --- | --- | --- | --- |
| NTO-refined-distance / SNOPT | 87/180 | 48.3% [32.2, 65.6] | 9.341 / 2.269 / 40.058 / 60.164 | 5.322 (n=169) | 5.406 rad (n=87) |
| NTO-refined-time / IPOPT | 100/180 | 55.6% [37.8, 72.8] | 7.105 / 3.244 / 15.052 / 46.586 | 3.928 (n=172) | 6.598 s (n=100) |

refined 使用预先固定的 1 mm 裕度、最多四次原生求解与最多 600 个碰撞绑定，仍在同一 60 秒总预算内。这两行来自上一轮封存结果；主表中的原始 NTO 没有被它们替换。

## 离线成本与证据

| 方法 | EI-ZO 膨胀均值秒数 | 轨迹优化累计均值秒数 | 数值验证累计均值秒数 |
| --- | --- | --- | --- |
| LSCS | 0.239 (n=110) | 0.002 (n=110) | 0.885 (n=110) |
| LGCS | 0.308 (n=110) | 3.554 (n=110) | 0.812 (n=110) |
| NTO-distance | — | 5.532 (n=109) | 1.098 (n=109) |
| SCSTO | 0.242 (n=110) | 0.028 (n=110) | 1.050 (n=110) |
| NTO-time | — | 2.907 (n=107) | 1.132 (n=107) |

阶段均值只计入整个后端正常返回候选和最终验证、且完整写回计时的记录；正常返回的求解失败保留。超时、异常或中断的后端累计值不进入阶段均值，也不补零；每格明确可用 n。同一调用若发生恢复重试，优化和验证计时累加各轮耗时；n 是符合统计条件的调用数。因此这些有条件的阶段均值不能相加后冒充全任务冷启动耗时。原始未完成计时仍完整保存。

| 路网种子 | 实际节点数 | 原始离线构建秒数 | 记录 |
| --- | --- | --- | --- |
| 0 | 6000 | 45.878 | [构建证据](../roadmaps/rss2025_n6000_s000/metadata.json) |
| 1 | 6000 | 45.838 | [构建证据](../roadmaps/rss2025_n6000_s001/metadata.json) |
| 2 | 6000 | 46.369 | [构建证据](../roadmaps/rss2025_n6000_s002/metadata.json) |

路网离线成本单列，未摊入在线冷/热延迟，也没有从在线冷启动中删除模型建立、路网加载、GPU 初始化或验证。冷启动来自外部进程实际墙钟记录；超时不是零耗时。复用结果的测量发生在上一轮，新 NTO 结果在本轮测量；源代码、配置和机器一致，仍可能存在运行时系统负载变化。

| 来源 | 原始注册槽位 | 源提交 | run manifest SHA256 |
| --- | --- | --- | --- |
| rss2025_paper_comparators_v1_dev_s000 | 120 | b73f2ddbcbcbb485a883d08b6b42641ddf1e6d81 | 1dcafad86f9b7c103e56f199ba93089937fb1ecb8fbeab138df54f8d7df05a70 |
| rss2025_paper_comparators_v1_dev_s001 | 120 | b73f2ddbcbcbb485a883d08b6b42641ddf1e6d81 | aa7a0a1486ae00781dae6d492a13b174a98ea63878e48315f218f7c31acf7a7a |
| rss2025_paper_comparators_v1_dev_s002 | 120 | b73f2ddbcbcbb485a883d08b6b42641ddf1e6d81 | bc6ab9bd3fcecb2ba441bd9000269f814b60b180593adc70c8c7f7eaf0fc7694 |
| rss2025_paper_comparators_v1_smoke_s000 | 20 | b73f2ddbcbcbb485a883d08b6b42641ddf1e6d81 | f97d9d62d254f4d6e6e4b83379794c8510e75fba9ea2862c68fc042f631e13bb |
| rss2025_reliable_v2_dev_s000 | 300 | e6b7e372e091ce5b01417073e0880e7c3068910c | e8fd4211c317283cc4d3f62efadbe4b6f6caf96cabe91512fa73fd570cd97ecc |
| rss2025_reliable_v2_dev_s001 | 300 | e6b7e372e091ce5b01417073e0880e7c3068910c | ad8c55549ca0ab743f2c175f33ffdac86d142387991083d2f5a6e109acd078e8 |
| rss2025_reliable_v2_dev_s002 | 300 | e6b7e372e091ce5b01417073e0880e7c3068910c | c45f896615a683eb85cc5a0f71abd227cbc38d95977cf150734ee5c66920fa22 |
| rss2025_reliable_v2_smoke_s000 | 50 | e6b7e372e091ce5b01417073e0880e7c3068910c | 7cf5f8466d2629ae8e454886f5ac4f0503832f8111a2fadab0dfc920be6dc815 |

运行代码 SHA 清单指纹：`aabb52706c6042bcb48ab603ead9e1dce0444a916785c3e3726dac7c99a64504`。不同 Git 提交的新增内容仅为报告/证据时，通过逐文件运行源码与配置散列相等判断可比较性。

- [全部逐题指标与分组结果](paper_comparators_v1/comparison_metrics/metrics.json)；[逐调用 CSV](paper_comparators_v1/comparison_metrics/observations.csv)
- [本轮证据与原始 NTO 参数审计](paper_comparators_v1/comparison_audit.json)
- [实际命令、开始/结束时间与退出状态](paper_comparators_v1/commands.jsonl)
- [执行前配置、环境和散列冻结](paper_comparators_v1/execution_freeze.json)

- [论文对比算法及来源核查](paper_comparators_v1/paper_coverage_audit.md)

- [原始轨迹交互回放](paper_comparators_v1/replays/index.html)


只读复核与重新生成报告（输出写入新的临时目录，不覆盖已封存证据）：

```bash
comparison_check_dir=$(mktemp -d /tmp/csdecomp-comparison-check.XXXXXX)
bash local/python.sh -m benchmarks.generic_motion_v1.reports.paper_comparators_v1.audit_comparison \
  --metrics benchmarks/generic_motion_v1/reports/paper_comparators_v1/comparison_metrics/metrics.json \
  --output "$comparison_check_dir/comparison_audit.json"
bash local/python.sh -m benchmarks.generic_motion_v1.reports.paper_comparators_v1.build_report \
  --metrics benchmarks/generic_motion_v1/reports/paper_comparators_v1/comparison_metrics/metrics.json \
  --audit "$comparison_check_dir/comparison_audit.json" \
  --output "$comparison_check_dir/benchmark_v1_paper_comparators.md"
```

重新测量完整五方法 smoke（写入新的结果目录，不覆盖历史结果）：

```bash
comparison_run_dir=$(mktemp -d /tmp/csdecomp-comparison-run.XXXXXX)
bash local/python.sh -m benchmarks.generic_motion_v1 batch \
  --manifest benchmarks/generic_motion_v1/manifests/smoke.jsonl \
  --roadmaps benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s000 \
  --config benchmarks/generic_motion_v1/configs/rss2025_reliable.json \
  --methods EI_ZO LGCS NTO_PAPER --objectives minimum_distance minimum_time \
  --timeout 60 --output "$comparison_run_dir/results"
```

## 结果适用范围

算法目标、原生求解器和论文中的比较关系已经落实到固定的 Kinova benchmark。模型是冻结的七自由度 Kinova，而论文仿真机器人为 Franka/FR3；本报告的数值直接回答 Kinova 主套件上的比较结果。论文没有公开 NTO 完整实现，149 个内部约束点依据归档作者 TeX，初始化端点复制与求解器限额均由本地固定协议明确记录。最终成功要求独立有限采样验证通过；有限采样不构成连续无碰撞数学证明。前端失败任务的全局可行性仍未知，数值求解失败也不能作为无解证明。

论文与代码来源：[RSS 2025 原文](https://www.roboticsproceedings.org/rss21/p045.html)、[CSDecomp 作者仓库](https://github.com/wernerpe/csdecomp)、[SCSPlanning 作者实现](https://github.com/TobiaMarcucci/scsplanning)。
