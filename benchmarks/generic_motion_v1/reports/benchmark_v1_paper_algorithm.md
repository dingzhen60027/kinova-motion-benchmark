# RSS 2025 EI-ZO 在固定 Kinova benchmark 上的算法复现

2026-10-28。已完成实际运行：**20 次 smoke + 360 次 dev**。本轮实现并运行的是原生 `DRM → EI-ZO → LSCS / SCSTO → 碰撞验证与恢复` 单路径流程。它是基于现有固定 Kinova 输入的算法适配复现，不是 FR3 原始环境或论文全部实验的精确复现。

算法运行源码固定在 `e1abf004b4347dff75b15a8868e14b732d3bf539`；参数初始提交为 `0c3028e`。所有运行在现有 `benchmark/generic-motion-v1` 分支上完成，没有修改上游 CUDA/C++、第三方依赖或其他研究项目，没有推送或改动 main。

## 证据入口

- [完成状态与 manifest 校验](paper_algorithm/completion.json)、[267 项测试输出](paper_algorithm/tests_final.txt)
- [论文原文、算法与参数逐项对照](paper_algorithm/source_audit.md)
- [smoke 完整结果](paper_algorithm/rss2025_smoke_v1_analysis.json)、[dev 完整结果](paper_algorithm/rss2025_dev_v1_analysis.json)
- [smoke 独立数组与导数审查](paper_algorithm/independent_smoke_audit.md)
- [dev 全部 220 个成功候选的独立审查](paper_algorithm/independent_dev_audit.md)
- [原生断言失败审查](paper_algorithm/native_failure_audit.md)
- [LSCS 交互回放](paper_algorithm/replays/LSCS/replay.html)、[SCSTO 交互回放](paper_algorithm/replays/SCSTO/replay.html)
- [实际 CLI 命令](paper_algorithm/commands.sh)、[含失败命令的完整台账](paper_algorithm/actual_cli_commands.json)

回放读取保存的原始轨迹、原始障碍和完整机器人碰撞盒，不重新规划。展示案例为 `smoke-bottleneck-000-p00`，其固定端点直线连接有碰撞；这只是工程示例，不作为性能抽样。

## 环境、模型和协议

复用 `.conda/envs/csdecomp`，Ubuntu 22.04.5、Python 3.10.22、Drake 1.51.1、CSDecomp 0.1.2、SCSPlanning 0.1.0、pybezier 0.1.4；本机为 i7-13700K、Quadro RTX 8000、驱动 570.211.01。未重建或覆盖环境。详见 [实际依赖与求解器探测](paper_algorithm/environment.json)、[Conda 清单](paper_algorithm/conda-packages.json)、[pip 清单](paper_algorithm/pip-freeze.txt)、[CPU 清单](paper_algorithm/cpu.json)。

固定机器人仍为完整 Kinova Gen3：7 个活动关节、20 个碰撞盒（15 个运动、5 个固定）、137 个检测对、53 个排除对。固定桌面、夹爪和设备、世界坐标、0.04 m 在线障碍球、所有起终点和运动限幅均保持不变。机器人 SHA 为 `a25f447c8caf4aff6dc76196c3d57cc940f00e802367f48034f06a9973927c75`，几何协议 SHA 为 `896ab9f55c5d8081b843358bd85cde2daae1f432564593f86218bf61733ce4fa`。

使用[独立论文参数配置](../configs/rss2025_eizo.json)，最终 SHA 为 `8c58d9430b82ca92c80b68e0aa11a5886d876f8d838cb9d3a9cb74ce450b9eba`：

| 参数 | 本次值 | 来源 |
|---|---:|---|
| EI-ZO δ / ε | 0.005 / 0.005 | 正式论文 Table IV |
| 更新粒子数 / 每轮最多新增面 | 10000 / 10 | Table IV |
| mixing steps / configuration margin | 60 / 0.01 | Table IV |
| τ / 二分次数 | 0.5 / 9 | 论文典型阈值 / 原生实现设置 |
| 最大膨胀迭代数 | 30 | 作者 arXiv 源文件注释，**非已发表表格数值** |
| DRM 最大任务空间 / 构型距离、邻居数 | 10 / 10 / 10 | Table V 的数值；关节仍使用 rad |
| 离线体素边长 / API 参数 | 0.06 m / 0.03 | API 参数是半边长 |
| 在线 lazy edge 检测步长 | 0.1 rad | Appendix D |
| 最终与种子路径检查步长 | ≤ 0.005 rad | 固定 benchmark 独立验收协议 |

迭代上限在任何在线规划前通过[预运行修订](paper_algorithm/pre_run_amendment.json)明确冻结。它也避免源代码审计识别的极高面数边界风险；没有声称该静态风险曾在本轮触发。论文表格没有给出该上限，不能把 30 宣称为表 IV 的实验参数。

三份新路网使用原来的 PCG64 种子 0/1/2 和 canonical 无新增障碍模型，**实际均为 6000 节点**。原路网与旧结果保留。路网身份现在包含构造配置与模型散列，运行时核对配置，避免把旧图误标为论文配置。

本轮先固定运行设计，再运行完整题单；没有按某方法成功筛题、换端点或换障碍。见[运行前声明](paper_algorithm/preregistered_protocol.json)。smoke/dev/test 的 manifest SHA 全部保持一致。[原冻结文件复核](paper_algorithm/frozen_inputs_check.json)通过：513 个当前文件匹配，另外 5 个早先加固的 Python 源文件保有精确原版本归档。**420 个 test 输入继续冻结，没有运行正式 test。**

## 实际结果

smoke 有 10 个固定问题，两个目标共 20 次：6 次数值验证成功、10 次前端失败、4 次无效端点。无效端点来自原有两个生成失败问题，原样保留，没有重新采样。

dev 为 60 个合法问题、20 个场景、3 份路网、两个目标，共 360 次。每个目标以全部 180 次合法查询为分母：

| 流程 | 验证成功 | 前端失败 | 原生断言退出 | 无条件成功率 |
|---|---:|---:|---:|---:|
| DRM → EI-ZO → LSCS | 110/180 | 69 | 1 | 61.11% |
| DRM → EI-ZO → SCSTO | 110/180 | 69 | 1 | 61.11% |

本轮没有 timeout、solver_failed 或被误计为成功的碰撞候选。两种目标各有 **75 个成功查询的原始端点直线连接有碰撞**，因此成功不只来自直连控制题。以场景为聚类单位、2000 次 bootstrap，两个目标的无条件成功率 95% 区间均为约 **41.67%–79.46%**；三个路网种子不是独立环境。

| 障碍族 | LSCS 成功/全部 | SCSTO 成功/全部 |
|---|---:|---:|
| sparse_clutter | 18/45 | 18/45 |
| dense_clutter | 8/45 | 8/45 |
| bottleneck | 45/45 | 45/45 |
| route_choice | 39/45 | 39/45 |

族名描述工作空间几何，不证明 C-space 窄通道、同伦类别或全局可行性。密集障碍族的完整成功率较低，是当前基线的真实限制。

每个目标原生 DRM 报告成功 111 次，其中 1 次种子路径未通过细查；故“给定原生 DRM 成功”为 **110/111**，“给定数值复核后的合法种子路径”为 **110/110**。后者不能替代上表 110/180 的无条件结果。汇总保留这两个不同分母。

138 次前端失败包含 136 次原生搜索失败和 2 次种子路径细查失败。2 次断言退出是 `dev-sparse_clutter-000-p01` 在第三份路网上的两个目标：原生起点连接候选耗尽后触发 `closest_node >= 0`，发生在 EI-ZO 之前。固定端点本身有效；这不证明问题无解。子进程隔离并保存 SIGABRT、stdout/stderr、phase、输入和文件散列，没有重试后替换失败成绩。

## 耗时与数值质量

外部 60 秒预算包含进程启动、模型与路网载入、规划、恢复和独立验证。下表冷启动分位数包含每个目标全部 180 次成功及失败查询；一次性离线路网时间独立列出。

| 目标 | cold p50 | cold p90 | cold p95 | 最大 cold |
|---|---:|---:|---:|---:|
| minimum_distance | 1.766 s | 3.270 s | 4.320 s | 8.428 s |
| minimum_time | 2.042 s | 3.571 s | 3.827 s | 7.926 s |

三份离线建图实际耗时分别为 **45.878 / 45.838 / 46.369 s**，不计入上述查询延迟。成功查询中，距离/时间目标的平均 EI-ZO 阶段分别为 0.235/0.230 s，优化阶段为 0.0017/0.0286 s，种子路径复核约 0.716/0.715 s，最终验证及恢复中的验证累计约 0.886/1.051 s。完整分阶段、失败分层、GPU 采样显存及离线数据见两个 analysis JSON 和原始 summary。

第二、第三份离线构造命令的运行窗口重叠约 6.43 s，因此这些离线数值是实际观测墙钟时间，不作为严格资源隔离的离线性能比较。380 个在线规划 worker 则由批处理器串行执行。

`warm_query_seconds` 是同一次冷执行去除初始化后的已初始化阶段计时，**不是另外执行的一次热启动实验**。两个目标各有 179 个可记录 warm 值，原生断言退出没有被填成零。不能将含独立验证的冷延迟直接与论文优化组件时间比较。

成功 LSCS 路径长度均值为 6.373 rad，成功 SCSTO 轨迹时长均值为 5.799 s；它们是不同目标、不同单位，不相互排序，也不是全局最优性的证明。dev 成功距离/时间轨迹共检查 140602/219711 个最终样本，两个碰撞 checker 在这些样本上没有分歧，最小已采样几何净距分别为 0.439/0.696 mm。**有限采样通过不构成数学连续无碰撞证书。**

独立事后审查核对了 dev 的 360 份 receipt、11822 个封存文件和全部 220 个成功候选；110 条时间轨迹全部经 SciPy BPoly 独立位置/导数复核。最大端点误差约 1.25e-8 rad，最大采样关节步长 0.004999961 rad。此审查重构数组和检查保存证据，没有重新调用规划器或碰撞计算；实际双碰撞检查来自各次已封存的在线运行。

## 实现修复与真实恢复

1. 紧凑、有序的活跃区域序列保留原始 DRM 种子边、覆盖边和共享锚点；不因同一区域覆盖连续多边而重复创建优化段。仅复用最后一个活跃区域是保序适配，不是论文“任意已有区域”的全部简化策略。
2. 编辑切面后重新检查原始路径覆盖。共享点被切除时，只按受影响片段的**原始 DRM 边**重膨胀，重新构建索引和连接，不引入未经检查的新弦。
3. 修复 Drake float64 → native float32 的区域交接：在不移动端点的前提下实施 ≤1e-6 rad 的有界偏置舍入，并用真实 native 谓词复核。两条旧失败数据在 CPU 回归中先重现、后修复；本轮 dev 的 175 次 native 区域交接检查都通过，无需额外偏置修正。
4. 原生 DRM 返回路径先经过同一双 checker 几何检查，再进入 EI-ZO。smoke 的 `open_control-p00` 两目标在相同采样点检测到约 0.239 mm 穿透，已拒绝，未生成优化结果。
5. 保存 native `EditRegionsCuda` 的真实返回数据：projection 和优化后的碰撞点。原接口文档把它们描述成待重建线段，调用方没有沿用这个错误解释。

dev 的 **19 次成功查询**实际经过碰撞恢复，总计 43 轮恢复；smoke 另有 1 次成功经过 3 轮。展示的 bottleneck 距离案例，双 checker 碰撞计数依次为 **86 → 49 → 38 → 0**，最终长度 7.202974 rad。

本轮所有实际成功序列都未触发连续区域省略，实际恢复也未触发片段重膨胀；这两个分支的当前证据是使用真实作者优化器和受控几何的 CPU 回归，不能声称已被本轮 GPU 题单覆盖。最终测试共 **267 passed**，弃用警告保留在日志。

## 复现边界与后续使用

- 核心使用现有原生 CSDecomp 与作者 SCSPlanning。安装的作者优化器文件匹配官方 `d3f885aa2b74a10418de96e61ec2df0bf6ef16b8`。LSCS 使用 Clarabel，区别于论文 MOSEK；SCSTO 使用作者交替凸优化与 Clarabel。本机实际小问题探测显示 Clarabel/SNOPT/IPOPT 可用，MOSEK 未启用，Drake Gurobi 未编译。
- 未复现 LGCS 的两路径混合整数分支，也没有重新运行 NTO 或声称复现论文的相对速度、可靠性倍数。未运行原论文四种路网规模的完整网格或 KUKA 实机实验。
- Kinova 碰撞盒、固定关节终点与通用障碍分布不同于 FR3 的 33 球模型和原始问题构造；保留现有 benchmark 是本轮前提。
- 第六阶 Bézier、0.01 优化相对阈值、最多 128 个恢复 witness、初次求解加 20 次恢复、细采样双 checker 都明确属于本地工程协议，不伪称全部来自论文表格。
- 原生 Python API 不暴露 EI-ZO 统计检验接受/迭代截断状态；固定 seed 也不保证 CUDA 逐位一致。只报告保存的真实轨迹及数值验证，不声称每个区域已观察到概率接受证据。
- 原生碰撞缓存按在线球心所在 cell 查表，本身不保守覆盖任意半径球；因此保留 lazy 检查、种子路径准入和最终独立验证。原生采样异常路径的静态源码风险仍未排除，不宣称已修复整个第三方库。

后续可把此显式配置作为固定的单路径 EI-ZO 基线。当前主要损失在前端连接，优先处理原生候选耗尽与密集环境连接失败；任何后续方法都继续使用相同输入与独立验收。正式 test 仍需用户另行明确授权。

## 复用命令

从仓库根目录执行；输出目录必须是新的，已有运行可用 `resume`。

```bash
bash local/python.sh -m benchmarks.generic_motion_v1 run \
  --manifest benchmarks/generic_motion_v1/manifests/smoke.jsonl \
  --roadmaps benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s000 \
  --config benchmarks/generic_motion_v1/configs/rss2025_eizo.json \
  --problem smoke-bottleneck-000-p00 --methods EI_ZO \
  --output /tmp/kinova_rss2025_repeated

bash local/python.sh -m benchmarks.generic_motion_v1 verify \
  --run benchmarks/generic_motion_v1/results/rss2025_dev_v1
bash local/python.sh -m benchmarks.generic_motion_v1.reports.paper_algorithm.collect_results
```

上述 `/tmp` 命令是复用示例，实际执行命令见 `paper_algorithm/commands.sh`。CLI 台账也保留了一次手工写错回放 job ID 的非规划失败；随后按真实 run manifest 的 ID 成功生成两个回放。没有把该操作错误混入 380 次规划统计。
