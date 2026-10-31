# 论文 baseline 交付与结果解读

本轮完成了固定 Kinova Gen3 benchmark 上的五组论文方法 baseline：DRM → EI-ZO → LSCS、LGCS、SCSTO，以及以相同 DRM 路径初始化的 SNOPT 距离优化和 IPOPT 时间优化。代码版本为 `e6b7e372e091ce5b01417073e0880e7c3068910c`；NTO 的固定安全距离和有界配点细化设置在运行前声明。原始配置、源码快照和全部历史结果继续保存。

**实际完成 950 条登记实验记录，其中 940 次合法输入运行、10 条无效输入记录。** 组成是 10 道 smoke × 1 张路网 × 5 个组合，以及全部 60 道 dev × 3 张路网 × 5 个组合。四个运行块全部封存，协调程序正常退出。完整指标、置信区间、逐题配对、环境和命令见[主报告](benchmark_v1_reliable_baselines.md)。

## 实际改善

每项 dev 结果的分母都是相同的 180 次合法查询，来自 20 个场景；端点和路网重复未当成独立场景。

| 目标 | 方法 | 原版本成功 | 本版本成功 |
| --- | --- | ---: | ---: |
| 距离 | LSCS | 110/180 | 110/180 |
| 距离 | LGCS | 94/180 | 110/180 |
| 距离 | NTO / SNOPT | 35/180 | 87/180 |
| 时间 | SCSTO | 110/180 | 110/180 |
| 时间 | NTO / IPOPT | 88/180 | 100/180 |

LGCS 使用显式 SCIP 后端求解相同的混合整数图优化问题，原来的 16 个商业许可规模失败全部恢复为数值验证成功。共同成功题的路径代价与原版本基本一致，平均冷启动时间由 2.18 s 增至 4.33 s。最终 dev 解中 48 个以 `optimal`、62 个以 `gaplimit` 结束，后者保留实际 gap，不标记为已证明全局最优。见[LGCS 解读](baseline_repair_v2/lgcs_audit_execution_summary.md)。

NTO 距离在直连碰撞任务上从 **0/141 提升到 52/141**；时间从 **53/141 提升到 65/141**。距离目标保留了全部 35 个原成功查询；时间目标新增 14 个成功，但原成功中有 2 个在新设置下超时，净增加 12 个。没有逐题择优合并两个版本。

新版本内部的比较也保留了质量与可靠性的取舍：在共同成功的相同查询上，LGCS 相对 LSCS 平均短 0.147 rad；NTO 距离相对 LSCS 平均短 0.424 rad，但有效输出率更低。SCSTO 在与 NTO 时间共同成功的 100 个查询上平均时长少 0.982 s。上述代价结论只适用于各自共同成功子集；完整场景聚类区间见主报告。

## 当前仍存在的真实失败

五组方法均有 **110 个通过独立 seed 验收的公共 DRM 输出和 70 个前端失败**。共同前端返回路径逐元素核对一致；历史上会触发进程 abort 的五次方法调用现在正常保存为前端失败。前端失败不等于问题已被证明不可行。

在前端通过后，LSCS、LGCS、SCSTO 均为 110/110 数值验证成功。NTO 距离另有 11 次求解失败、11 次超时和 1 次碰撞失败；NTO 时间有 8 次超时和 2 次碰撞失败。三条碰撞失败均已用尽预设四轮细化上限，实际最小间隙约为 −0.85、−0.95、−1.41 mm，保留失败，未降低验收阈值。

**主报告末段的 BoxBoxGradient 调查指向历史保存构型，不是本轮新失败的归因。** 本轮 NTO 原生异常记录和 BoxBoxGradient 日志命中均为零；11 次距离求解失败的最后原生 SNOPT 状态为 `info=13` 七次、`info=41` 四次。旧构型的底层问题仍有独立复现，当前环境未应用尚未合并的上游补丁。具体任务和原生状态见[NTO 结果解释](baseline_repair_v2/nto_result_interpretation.md)。

## 验收与回放

- 主回归测试 **479 项通过**，执行协议测试 **11 项通过**；实际日志链接在主报告中。
- [全局独立审计](baseline_repair_v2/integrity_audit.json)、[LGCS 审计](baseline_repair_v2/lgcs_audit.json)、[NTO 审计](baseline_repair_v2/nto_audit.json)均通过，错误为零。
- NTO 审计独立核对 1,310 份新旧 receipt、42,704 个封存文件，重建 206 条保存候选，最大采样差 `2.66e-15 rad`。重建数量包含失败候选，不是成功数量。
- [交互回放图库](baseline_repair_v2/development_replays_v1/index.html)包含同一条直连碰撞 dev 任务上的五方法成功轨迹，以及一条明确标注 **SEED ONLY** 的失败前端种子。回放来源与原始文件 SHA 全部核对，未重新规划。
- [浏览器验收](baseline_repair_v2/browser_checks_v1/inspection.json)实际通过 6 个视图的播放、首末帧拖动、关节状态匹配、视角切换、画布与图片加载检查，保留截图。检查使用独立 Chromium 会话，在全部计时实验结束后运行。

第一次 NTO 审计把“首轮求解被 watchdog 中断、顶层初值副本尚未生成”的五个超时误判为缺失证据。首次审计输出、脚本和日志已保存；随后只修正审计器并增加中断状态测试，原始运行与最终失败状态均未修改。所有验收器的失败和修正也保留为证据。

## 核验与使用

在现有 Conda 环境、仓库根目录核验已完成结果：

```bash
bash local/python.sh -m benchmarks.generic_motion_v1 verify \
  --run benchmarks/generic_motion_v1/results/rss2025_reliable_v2_dev_s000
bash local/python.sh -m benchmarks.generic_motion_v1 verify \
  --run benchmarks/generic_motion_v1/results/rss2025_reliable_v2_dev_s001
bash local/python.sh -m benchmarks.generic_motion_v1 verify \
  --run benchmarks/generic_motion_v1/results/rss2025_reliable_v2_dev_s002
```

新的算法比较使用新的输出目录、同一份 manifest、路网、60 秒总预算和数值 validator；配置入口为 [`rss2025_reliable.json`](../configs/rss2025_reliable.json)，实际批量运行命令在 [commands.jsonl](baseline_repair_v2/commands.jsonl)，单题与批量示例见[使用说明](../README.md)。

本轮没有重新运行正式 test。既有 395 道合法 test、25 条生成失败以及原版本的正式实验结果保持原样；当前开发结果与原正式结果分开保存和报告。距离和时间目标分别比较，成本不跨目标排序。有限碰撞采样通过表示约定分辨率下的数值验收，连续轨迹的速度、加速度和端点约束另外核查。
