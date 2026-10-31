# 公开机械臂 benchmark 的构造与适用边界

本次只读检索于 2026-10-30 完成。检索原论文正文、作者仓库及固定 commit 的源代码，并实际解析两个公开数据文件；没有安装外部环境、运行规划或改动项目冻结数据。逐项来源和未披露事项见 [sources.json](sources.json)，公开数据实读统计见 [robometrics_dataset_inspection_v2.json](robometrics_dataset_inspection_v2.json)。

## MotionBenchMaker：程序化场景变化，但公开集经过规划器筛选

原论文发布 5 种机器人 × 8 类名义环境的 40 个数据集，每组 100 个不同问题。场景通过 SE(3) 中的高斯/均匀扰动或场景 URDF 关节变化生成。对象相对末端位姿经碰撞感知 IK 变成关节端点，也支持直接关节端点。论文 Sec. IV 明确使用调参后的采样规划器、60 s 限额，丢弃未求解的问题；这不是未经可行性筛选的随机关节分布。[原论文 Sec. III–V、Fig. 6–8](https://www.kavrakilab.org/publications/chamzas2022-motion-bench-maker.pdf)

固定源代码进一步确认：`generate.cpp` 使用 60 s、2 次 planning attempts，仅在成功分支保存请求/几何/轨迹并递增题号；默认规划器是 RRTConnect。`benchmark.cpp` 载入同一请求与场景给各算法，显式关闭自动简化与插值。生成时的筛选、评测时的简化口径必须分别记录。[生成器](https://github.com/KavrakiLab/motion_bench_maker/blob/f50542777f4d1ae013b0de430bafbf51fdbdc40e/scripts/generate.cpp#L95-L174)；[评测器](https://github.com/KavrakiLab/motion_bench_maker/blob/f50542777f4d1ae013b0de430bafbf51fdbdc40e/scripts/benchmark.cpp#L57-L95)

论文的一个比较实验是 100 个场景问题、每题 20 次、60 s；优化实验每题 5 次、180 s。其重点是小样本可能改变算法排名，不是这些数值构成通用标准。没有在已查段落/入口找到可直接复用的场景级 dev/test 隔离和完整种子清单。[原论文 Sec. V-A](https://www.kavrakilab.org/publications/chamzas2022-motion-bench-maker.pdf)

**对本项目的启示**：借鉴场景族、显式几何/端点/生成配置与固定请求，不照搬抓取偏置或规划器成功筛选。当前 `feasibility_unknown` 保留规则符合本项目自身问题定义，但其成功率不能与筛选后的 MBM 成功率直接横比。

## VAMP：快速 RRTConnect 是强基线，几何模型与后处理不可省略

ICRA 2024 论文沿用 MBM：Panda 7 DoF、Fetch 8 DoF、Baxter 14 DoF；Panda/Fetch 采用 7 类公开环境、各 100 题。实验每题 5 次，OMPL 允许 5 min，VAMP 允许 100 万迭代；两者并非相同墙钟预算。用共同 Halton 序列、相同运动验证分辨率，并分别报告原始/简化长度及规划/简化耗时。机器人采用球近似，障碍采用几何原语；检查是离散运动验证。[原论文 Sec. III–IV、Table I](https://www.kavrakilab.org/publications/thomason2024vamp.pdf)

当前官方仓库与论文版本不能混同。资源说明指出当前 RRTConnect 默认包含 dynamic-domain 等变化，重现论文需显式调整；还警告 VAMP Panda 夹指间距 6.5 cm、Robometrics 2.5 cm，足以令部分输入失效。官方数据转换保留起点和多个 IK 目标，并标记有效性；评测脚本跳过无效输入。[固定版本资源说明](https://github.com/KavrakiLab/vamp/blob/27cb9b66ebd8d101ce88d330f3b028398afc1cfa/resources/README.md)；[转换器](https://github.com/KavrakiLab/vamp/blob/27cb9b66ebd8d101ce88d330f3b028398afc1cfa/resources/robometrics_to_pkl_json.py)

**对本项目的启示**：RRTConnect＋可靠碰撞检查＋简化应当作为正式强基线。不同机器人球模型、夹爪开度或碰撞过滤对会改变题目本身。公开集接入前必须做模型与端点审计；不得用改球半径或换端点来补救无效输入。

## Robometrics / MπNets：数据数量不等于独立场景数，目标也不同

本次读取固定版本的 YAML，得到以下实际统计（仅数据结构/精确几何去重，不是碰撞验收）：

| 文件 | 题数 | 不同障碍记录数 | 目标形式 |
|---|---:|---:|---|
| `mb_set.yaml` | 800 | 700 | `goal_pose`＋1–20 个 `goal_ik` |
| `mpinets_set.yaml` | 1800 | 1800 | `goal_pose`＋1–20 个 `goal_ik` |

MBM 的额外 100 条来自 `box_panda_flipped` 与 `box_panda` 共享障碍记录；不能称为 800 个独立布局。上述“不同”也不自动证明统计独立。两份数据坐标系均为 `panda_link0`。[固定数据目录](https://github.com/fishbotics/robometrics/tree/81e3d1d605de84100d8ab880b43096aba221a48b/robometrics/content/dataset)；[实际解析结果](robometrics_dataset_inspection_v2.json)

MπNets 原论文测试是三个分别由 Global、Hybrid、两者都能解的 1800 题套件，每套三类环境各 600 题，并声明题目环境/起点/目标未出现在训练中。Hybrid 数据还可按专家最终位置改写目标。其成功指标为末端位置误差 <1 cm、姿态误差 <15° 且无物理违规；并不是固定 `q_goal` 的最短关节路径任务。[原论文 Sec. 4–5](https://proceedings.mlr.press/v205/fishman23a/fishman23a.pdf)

固定 Robometrics 代码当前计算 `success` 时只有位置误差 <1 cm 和无物理违规，尽管同时记录姿态误差。这与上述论文口径不同。其碰撞指标遍历给定轨迹采样点，本身不建立连续无碰撞证明。[Evaluator](https://github.com/fishbotics/robometrics/blob/81e3d1d605de84100d8ab880b43096aba221a48b/robometrics/evaluator.py#L70-L89)；[metrics](https://github.com/fishbotics/robometrics/blob/81e3d1d605de84100d8ab880b43096aba221a48b/robometrics/metrics.py)

**对本项目的启示**：可作为将来的独立外部几何场景轨道，但必须单列模型、端点/目标定义、筛选历史及重复布局。不能将多目标 IK 问题直接拼入当前固定七关节起终点主集，也不应引入视觉学习或操作语义。

## OMPL / Planner Arena：评测协议比单一总分重要

OMPL 的规范是固定完整问题、显式时间/内存/重复数、保存算法配置、硬件、版本、seed、每次结果与优化过程。2015 年论文区分“找到首个解即停止”和“继续优化到预算结束”，并强调不存在跨任务的单一最佳指标。[原论文 Sec. II–III](https://moll.ai/publications/moll2015benchmarking-motion-planning-algorithms.pdf)

官方文档区分 approximate 与真正到达目标，分别记录规划和简化，并提供比规划更细的 `correct solution strict` 验证。文档中的 5 s / 50 次只是示例；不是所有机械臂必须采用的标准。[OMPL benchmark 文档](https://ompl.kavrakilab.org/core/benchmark.html)

Planner Arena 还明确提醒：缺失质量指标被忽略，极低成功率的方法可能只由极少量简单成功题决定均值。对本项目应同时保留全部合法任务成功率、失败/超时分布、共同成功题的配对质量及计算时间、场景级不确定性，以及优化方法的 cost-versus-time。当前共享前端算法另需条件成功率，不能把前端失败当后端优化失败。[Planner Arena 帮助](https://planner-arena.kavrakilab.rice.edu/)

## 本次检查边界

这些资料支持“主集保留固定模型、端点、未按方法成功筛选，同时新增独立外部公开轨道”的方向；它们不能证明本项目所有实现无 bug。模型一致性、几何保守性、实际分布、数值阈值和 manifest 完整性仍须在本地独立检查。对于本次未查明的 seed/版本关联，均在结构化来源中标为未核实，没有补造参数。

复核公开 YAML 的实际命令：

```bash
bash local/python.sh benchmarks/generic_motion_v1/reports/benchmark_design_audit_v2/literature_mbm/inspect_public_datasets.py --output /tmp/robometrics_inspection.json
```

首次匿名检索中的第三方聚合页和 Scribd 未用于结论。网络获取的 404 和超时保存在 [search_log.json](search_log.json) 与 [download_log.json](evidence/download_log.json)。归档的官方源码保留对应许可证，SHA256 见下载记录；没有归档大数据文件或整篇论文副本。
