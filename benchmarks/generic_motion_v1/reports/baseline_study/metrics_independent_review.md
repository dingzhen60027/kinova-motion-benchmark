# 五方法统计实现独立审查

2026-10-28。审查 `runners/study_metrics.py`、`runners/summarize.py` 与
`BASELINE_PROTOCOL.md`，只使用 CPU 读取既有封存开发证据、重算统计及构造明确标记的
内存测试。没有运行规划器，没有改动输入、模型、历史结果或原始计时。本文与
`metrics_independent_review.json`、`.py`、`.log` 为本次新增审查材料。

结论：开发集的成功数、合法输入分母、物理场景聚类、共同成功成本和共同成功延迟
与协议一致。发现并修复了三个会使统计输出产生误导或放松配对门禁的问题。
全部实际开发数据中没有发现
跨方法输入或公共 DRM 路径不一致，因此这些防护问题不改变现有成功数。

## 实际证据和修复

**P1：未执行的后端初始化零值混入阶段均值，已修复。** 原实现直接统计
`row['timing']`。16 次 LGCS 许可证拒绝均发生于 `trajectory_optimization`，没有
返回 candidate 或最终 validation；优化、验证和恢复字段却保留初始化的 0。
原口径把这些值当成测量，优化和验证均值被压低。

| LGCS 阶段 | 原始累计字段直接聚合 n / 均值 s | 完整返回口径 n / 均值 s |
| --- | ---: | ---: |
| 优化 | 110 / 0.16885508486408402 | 94 / 0.19759637590477916 |
| 最终验证 | 110 / 0.6381251940165054 | 94 / 0.7467422483171872 |
| 碰撞恢复 | 110 / 0.00442703426392241 | 94 / 0.005180572010973033 |

左列由原始 receipt 中不变的计时字段重新计算，不冒充一个新实验，也不依赖后来
可能重新导出的派生报告。右列来自新增
`dev_metrics_stage_qualified/metrics.json`。900 条观测的 raw timing 已逐条与
其 `result.json` 对照，全部相同。

新增 `completed_stage_seconds` 专供阶段统计，原始 `timing` 完整保留。前端计时
在对应调用返回后赋值，因此保留已有值；后端 inflation、alternate path、optimization、
validation、recovery 为多轮累计字段，只有正常返回 candidate 与最终 validation、
同时存在优化和验证计时、没有 failed_stage/exception/不完整终态时才纳入。后端中断
时连同此前已完成的后端子调用一起排除，属于保守的“完整后端返回”选择范围。
正常返回的 solver_failed/collision_failed 仍可能计入，不能只选成功样本；完整后端
没有恢复时的真实 0 则保留。每组明确报告完整和不完整后端次数、逐字段 n 和选择范围。
这些条件阶段均值不能替代包含全部合法失败的冷端到端延迟。

**P1：公共前端不一致时异常退出，但已写出的报告仍 complete=true，已修复。**
原代码先写指标文件再检查公共 DRM 路径/准入差异并 raise，消费者若只读 JSON 会
把诊断表误当可比较结果。现仍保留诊断和异常退出，但写入
`complete=false`、`comparable=false` 及原因；正常报告明确 `comparable=true`。
路径 SHA 不同和前端返回状态不同两种内存反例均有回归测试。实际 900 条开发记录
没有公共前端差异。

**P2：合法性过滤前未完整验证跨方法输入身份，已修复。** 原 `pair_metrics` 先过滤
`input_legal` 再决定有哪些方法参加配对。CPU 内存反例取一组真实开发记录的副本，
只把 LGCS 标记成另一个无效输入；它完全消失于合法配对集，报告仍可写出
complete/comparable=true，LGCS legal_jobs=0，其他四方法各为 1。
这是统计边界的隔离反例，显式替换了 `load_rows`，没有伪造或更改任何实验 receipt；
不代表真实 batch verifier 已接受这样的篡改。现已在合法性和报告分层筛选前按
固定题、地图和方法种子比较 input_legal、input SHA、模型、几何、物理场景、预算、
障碍族、距离档和直连属性，使未来合并不同数据版本的完整合法 receipt 也不能
绕过配对门禁。修复前实测反例保存在 `.pre_pair_guard.json`；当前审查 JSON 保存
修复后拒绝结果及修复前证据 SHA。九种属性变异和全部无效输入但版本不同均有
CPU 回归测试，拒绝时不写可比较指标文件。

## 独立复算结果

| 组合 | 合法开发观测 | 严格成功 | 物理场景 | 完整后端计时 n |
| --- | ---: | ---: | ---: | ---: |
| LSCS | 180 | 110 | 20 | 110 |
| LGCS | 180 | 94 | 20 | 94 |
| NTO-distance | 180 | 35 | 20 | 109 |
| SCSTO | 180 | 110 | 20 | 110 |
| NTO-time | 180 | 88 | 20 | 107 |

900 条观测是 60 个题 × 3 个共享地图 × 5 个组合，不是 900 个独立环境。开发集
20 个物理场景各有 3 对端点；bootstrap 按场景整体重采，保留其所有端点和地图观测。
open_control 在 `_row` 中统一使用 `open_control_shared_base_world`；两个或更多
open ID 的测试确认仍只有一个物理聚类，CI 为 null，且不进入 obstacle_main。

四个目标内方法对的共同成功数独立复算为 94、35、35、88；成本差和冷延迟差都使用
完全同一个共同成功集，并与导出值逐项一致。主要成功分母包含全部合法前端失败、
优化失败、unsupported、timeout 和 infrastructure_error；invalid 输入不进入算法
成功率或合法查询耗时。resolved_pair 从双方同时排除依赖/基础设施失败，作为明确
标记的次要选择子集，不改变主分母。没有把求解器返回值直接等同于严格数值成功。

输入/模型/源码/配置/预算混合与重复地图观测已有防护：重复
`(case_id, roadmap_id, method_seed, method, objective)` 拒绝；全部记录的模型及几何
SHA、evaluation config、timeout 和完整 runtime source_hashes 必须一致，地图规模按
requested_nodes 分组，实际保留节点数另存。提交号不同但源码清单完全相同允许合并。
五方法组合缺失与配对缺失已有失败测试，正式矩阵是否齐全仍须另对冻结 study manifest
核验，不能从单独一个完整子运行推断整个预注册矩阵已经执行完毕。

## 可以接受但必须说明的统计限制

- scene bootstrap 条件于本次共享的 3 份地图，不估计从无限随机地图总体重新抽取
  的变异。共享地图不能作为额外独立场景。
- 2000 次 PCG64 seed=45827 的 percentile CI 是描述性区间。若所有样本场景都为
  0% 或 100%，实际 CPU 反例得到 `[0,0]` 或 `[1,1]`；这是经验 bootstrap 在边界的
  退化，**不证明真实总体成功率确定等于 0 或 1**。五场景的族内 dev CI 尤其有限。
- 共同成功、原生 DRM 条件、严格 seed 条件和 resolved 条件区间只描述对应被选中
  的贡献场景及任务，不能外推为全部请求的总体质量；多个比较没有多重性校正。
- source/model/config pooling guard 没有证明不同机器、GPU、安装库二进制和许可证
  能力相同。实际 dev 是同一封存运行；未来跨环境合并还应显式比对环境/依赖指纹，
  不能仅凭 Python 源码 SHA 宣称性能环境一致。
- CSV 当前按 job 选择最终封存 attempt，不是把历史 retry_pending 尝试全部独立计数。
  本次 dev 没有因此重复观测；协议中的“每个 sealed attempt 一行”需要理解或修订为
  “每个最终封存 job 观测一行，原始 attempts 另外保留”，避免续跑历史被误当独立实验。

## 实际命令

在仓库根目录执行，使用现有 Conda wrapper，未调用 GPU：

```bash
bash local/python.sh -m pytest benchmarks/generic_motion_v1/tests/test_study_metrics.py -q
bash local/python.sh -m benchmarks.generic_motion_v1.runners.study_metrics --runs benchmarks/generic_motion_v1/results/rss2025_full_dev_v1 --output benchmarks/generic_motion_v1/reports/baseline_study/dev_metrics_stage_qualified > benchmarks/generic_motion_v1/reports/baseline_study/dev_metrics_stage_qualified.log 2>&1
bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_study.metrics_independent_review > benchmarks/generic_motion_v1/reports/baseline_study/metrics_independent_review.log 2>&1
```

全部修复后定向测试 **65 passed**。独立脚本实际复算 900 条观测、
四个共同成功方法对、所有原始计时字段，并保存隔离反例。当前文件哈希和实际输出
SHA 以同名 JSON 为准。未提交 git commit；由主任务统一审查和提交。
