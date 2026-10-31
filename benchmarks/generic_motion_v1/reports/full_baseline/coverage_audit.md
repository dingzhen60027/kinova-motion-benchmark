# 论文完整基线覆盖审计

2026-10-28；只读审计起点 `612c812e031ce687d2356be133c833bb1d555192`。本审计没有启动规划器、GPU 或优化器，没有修改历史结果。下列结果区分已保存的历史实验与当前 Kinova 固定题单；二者不能合并成同一张算法成绩表。

## 已有证据的实际完整性

本次重新读取两个历史 run 的全部 800 份 `case.json`，按原始状态重计，并逐文件核对已有 `raw_inventory.json.gz` 的 SHA-256、大小与 18 份计算源码快照。未发现缺失或不匹配。

| 历史轨道 | 原始 run | 已核对文件 | 已核对字节 | 完整案例 |
|---|---|---:|---:|---:|
| Forest | `reproduction/runs/20261025T092109.664874Z_paper_completion_forest` | 22409 | 161436285 | 400/400 |
| FR3 重建诊断 | `reproduction/runs/20261025T092858.440656Z_paper_completion_fr3` | 15486 | 1370183848 | 400/400 |

每个轨道为 10 个环境种子 × 10 个 DRM 种子 × 4 个路网规模；Forest 为 200/400/800/1600，FR3 为 3000/6000/9000/12000。两个 run 的计算实现摘要均为 `40c9f3cdf57783cd7ae5d856d4b4e10332dd37091aa45a7c38718cb860d01611`。文件完整性验证仅说明保存证据未变，不能替代新几何验证。

从原始案例重计的严格数值成功数如下；每栏分母为全部 400 个预定案例，原生前端失败未删除。

| 方法 | Forest | FR3 重建诊断 |
|---|---:|---:|
| LSCS | 388 | 240 |
| LGCS | 385 | 208 |
| SCSTO | 387 | 240 |
| NTO_distance | 120 | 74 |
| NTO_time | 231 | 170 |

Forest 原生 DRM 成功 400 次，但 12 条 seed 未通过严格核验；FR3 原生 DRM 成功 241 次，其中 1 条 seed 未通过，另有 157 次 DRM 失败和 2 次 IK 失败。FR3 LGCS 保留 32 次许可证阻塞。上述原始状态与 [历史主报告](../../../../reproduction/reports/paper_completion/reproduction_report.md) 一致。

历史规则对 LSCS/LGCS/SCSTO 拒绝未通过核验的 seed，但 NTO 仍接收原始 DRM seed。因此这些历史成绩不能冒充当前所有方法共用严格前端门槛的 Kinova 对照。历史 NTO 也不是后来加强版本。

另核对了 [加强基线历史归档](../../../../research_platform/reports/strengthened_baselines/v1_20261027/archive_receipt.json) 对应的 `raw_evidence.tar.gz` 整体 SHA，实际仍为 `25111ef55fac3acbb0a7562e12f6e9c5bdb8f0999ff44e9e0894db5669555b53`。该归档含 1683 个文件、56 个方法任务（46 成功、10 失败）：6 个开发案例及单列 Figure 6 回归，不能包装为新的大样本基线成绩。本次没有逐一解包重验这些成员；此前保存的独立审计和恢复验证另见历史报告。

## 哪些内容能复用

1. **Forest 历史 400 题**可以作为独立 `forest2d_reference` 重建轨道复用。原始随机种子没有公开，所以这是符合公开分布的重建，不是作者原题逐题复跑。保留旧求解器、严格碰撞失败、计时定义和源码身份。
2. **LGCS 转录与 NTO 实现**已经存在于 `reproduction/paper_completion/optimization.py`，平台还有相应加固。应审查并接入当前 Kinova 模型、统一输入/预算/验证器；不能直接把旧 FR3 结果当作 Kinova 算法覆盖。
3. **历史吞吐量**已有 Forest/FR3 各 10 个环境 × 5 次同步计时 × 200 万配置，公开 iiwa 另有 5 次。它们只表征各自模型和本机，不支撑原论文硬件速度比。
4. **论文日期公开核心**已隔离编译并完成小型碰撞/DRM/EI-ZO 探针；缺少新版 recovery API，现有 800 个主案例仍使用安装版 CSDecomp 0.1.2。不能用小型探针把主实验重新标为论文二进制。
5. **加强 NTO**是明确不同的有限维/细化算法，仅能作为额外工程对照。原论文 NTO 轨道必须单独保留，避免用加强版本取代论文实现却沿用其标签。

## 不能抹去的缺口

**历史 FR3 球模型不适合作为精确论文基线。** 已保存的独立几何诊断在同一运动学的 home 构型检测到 link5–hand 拟合球穿透 −35.1577 mm，而官方两体碰撞网格凸包相距 +37.0481 mm；31 点 q7 扫描的 22 点有同类球伪碰撞。当前重建总对数 851，论文为 1187，原 33 球中心、半径及过滤缺失。不能凭修改过滤或凑对数消除这一差异。该轨道保留为失败原因与历史重建诊断，不作为作者环境准确复现的证据。

论文硬件实验需要 KUKA LBR iiwa 7 R800、三台 D415、标定、原始感知和 15 条物理执行记录。公开 11 盒模型组装与模拟探针不是这些记录；缺失项不能用仿真填充。MOSEK 未许可、Gurobi 商业规模许可缺失、本机硬件与作者不同、作者完整 NTO 转录未公开，也都不能通过统计处理修补。

## 当前 Kinova 需要补齐的覆盖

审计起点的 [Kinova 报告](../benchmark_v1_paper_algorithm.md) 完成 20 次 smoke 与 360 次 dev：LSCS/SCSTO 各 110/180 dev 数值成功；两者只是论文的单路径方法。当前题单尚缺全套开发对照中的 LGCS、论文参数 NTO_distance、论文参数 NTO_time。旧通用 NTO 使用 IPOPT 距离与最少 16 控制点时间表达，不能自动称为论文的 SNOPT 距离/原始 DRM 节点初始化。

建议按以下次序补齐，而不更换模型、端点或失败案例：

1. 将五个方法作为明确命名的策略接入同一原生 DRM 前端、相同输入 SHA、相同 60 秒端到端预算和独立验证器。区分求解失败、验证失败、许可证阻塞与前端失败。LGCS 的第二路径搜索额外计时；原路径保留规则和区域覆盖必须验收。
2. 先完成所有方法 smoke，再执行全部固定 60 个 dev × 3 个路网种子的配对对照。若新增代码改变已运行 LSCS/SCSTO 的执行路径或数值协议，就重新运行并保存新版本，不与旧结果无说明拼接。
3. 报告全部预定问题为分母、给定原生 DRM 成功与严格 seed 准入的不同条件率；成本只在共同有效案例比较。区分冷启动、同次执行去初始化的 warm 段、独立离线时间和轨迹持续时间。
4. 若需移植论文 10×10×4 规模研究，应单列 Kinova `paper_reference` 轨道，不混入既有 420 test。当前 60×3 单规模开发验收不是这一规模研究。

按任务规范，420 个固定 test 仍是未运行的正式测试集。当前“完整论文基线”可以落实为方法与开发验证覆盖，同时复用并明确标记历史 Forest 参考网格；它不能使缺失的作者 FR3 资产或真实硬件实验成为已复现。

## 审计材料

- [论文实验与参数覆盖台账](../../../../reproduction/reports/paper_completion/experiment_coverage.json)：23 类证据，其中 144 项参数来源分类为已发表 65、作者草稿注释 4、本地重建选择 58、缺失 17。
- [历史运行索引](../../../../reproduction/reports/paper_completion/run_index.json) 与各轨道 `raw_inventory.json.gz`：本次实际全量核验对象。
- [历史分母与分析审计](../../../../reproduction/reports/paper_completion/analysis_audit.md)：CUDA 无碰撞率与求解成功率不混用。
- [加强基线报告](../../../../research_platform/reports/strengthened_baselines/report.md) 和 [独立审计](../../../../research_platform/reports/strengthened_baselines/independent_final_audit.md)：6 个开发案例与 Figure 6 分开保存。
- [任务规范](../../task_spec/BENCHMARK_SPEC.md)：Kinova adaptation 与论文参考轨道区分、模型固定、测试锁定和共同验证要求。

## 新五方法接线的独立 CPU 审查

冻结前另行只读审查了 `paper_graph.py`、其 LGCS 适配接口、主 runner、batch 和源码清单。审查没有调用 GPU。实际执行：

```bash
bash local/python.sh -m pytest -q \
  benchmarks/generic_motion_v1/tests/test_paper_graph.py \
  benchmarks/generic_motion_v1/tests/test_batch.py \
  benchmarks/generic_motion_v1/tests/test_paper_corridor.py
```

修复前相关测试 18 项通过；修复后新回归在内 20 项通过（0.62 s）。本审查定位了一个真实的覆盖遗漏：LGCS 初始区域包含规范化前固定 double 端点，但恢复仅保护 native float32 DRM 边，可能让固定端点被新的切面排除。负责人已加入 `protected_edge_anchors`，把固定端点传入 native 交接、编辑后覆盖与两条原路径补覆盖，且新增回归。审查读取了该修复。

LGCS 输出每段的 `segment_regions` 明确指向无序区域池；验证器先接收该有序子序列，恢复再把采样段索引映射回池索引。两条已接受 DRM 路径的全部原始边均被重新核对，缺覆盖时只重新膨胀原始边。源码清单已包括三个新适配模块和实际调用的历史 LGCS 源文件。batch 的 `_unique` 本来就拒绝空任务，因此只选择不适用的 LGCS 时间目标不会产生成功的空批次。

另指出两个失败证据解释事项：恢复后新一轮若没有候选，上一轮验证不能被误当成新候选的验证；Gurobi 存在 incumbent 不等同于证明全局最优，必须保存并解释原生状态与 `globally_optimal`。这两项不应通过删除失败轨迹来处理。这里的 CPU 检查与源码审查不是实际 GPU LGCS 恢复覆盖率证据；实际运行必须单独计数。
