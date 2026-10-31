# RSS 2025 五方法基线：固定 Kinova benchmark 的实际运行报告

2026-10-28。已完成 **50 次 smoke + 900 次 dev**，覆盖论文的 LSCS、LGCS、SCSTO、NTO-distance 和 NTO-time 五个方法/目标组合。dev 共 **437 次严格数值成功、463 次失败或不支持**，全部预定任务和原始失败均保留。这里完成的是固定 Kinova 环境上的五方法实现与开发验证；作者 FR3 原题、四路网规模全网格及物理硬件实验未在此轨道复现。

本轮计算源码冻结于 `72984027030175b7bca78e2dcf592b38f0c1864b`，五种方法均重新运行。旧的 380 次 Kinova 结果、800 个历史 Forest/FR3 重建案例和加强 NTO 结果原样保留，不与本轮拼接计算成功率。

## 方法、环境与共同条件

| 方法 | 实际调用 | 相对论文的边界 |
| --- | --- | --- |
| LSCS | 原生 DRM → EI-ZO → 作者 SCSPlanning polygonal | Clarabel 替代未许可的 MOSEK |
| LGCS | 原生 DRM → 最多两路径 EI-ZO → Gurobi 透视 GCS MISOCP | 公开公式的本地转录；非作者未公开 LGCS 源码；许可失败保留 |
| SCSTO | 原生 DRM → EI-ZO → 作者 SCSPlanning biconvex | 使用作者 Clarabel 实现；本地明确六次 Bézier 与数值验收 |
| NTO-distance | 原生 DRM → Drake KTO 二阶 B-spline → SNOPT | 保留全部原始 DRM 节点，不做控制点增密 |
| NTO-time | 原生 DRM → Drake KTO 四阶 B-spline → IPOPT | 首尾各复制一次以给定静止初值；这是明确重建选择 |

复用现有 Conda `.conda/envs/csdecomp`：Ubuntu 22.04.5、Python 3.10.22，csdecomp 0.1.2、drake 1.51.1、scsplanning 0.1.0、pybezier 0.1.4、clarabel 0.11.1、gurobipy 13.0.3。GPU 为 `Quadro RTX 8000, 570.211.01, 49152 MiB`；i7-13700K CPU。没有安装/覆盖依赖，没有修改上游 CUDA。GPU worker 串行；CPU 未做独占资源预留，结果是本机实测而非作者硬件速度复现。完整清单见 [environment.json](full_baseline/environment.json)，Conda/Python 锁与 CPU 信息见 [上一轮环境清单](paper_algorithm/environment.json)、[Conda 清单](paper_algorithm/conda-packages.json)、[pip 清单](paper_algorithm/pip-freeze.txt)。

共同模型为完整 Kinova Gen3，7 个活动关节、20 个碰撞盒（15 运动、5 固定）、137 个检测对、53 个排除对；桌面、夹爪及固定设备不变，新增障碍用统一 0.04 m 半径球保守覆盖。机器人 SHA 为 `a25f447c8caf4aff6dc76196c3d57cc940f00e802367f48034f06a9973927c75`，几何协议 SHA 为 `896ab9f55c5d8081b843358bd85cde2daae1f432564593f86218bf61733ce4fa`。固定关节起终点直接来自 manifest，无 IK 重选。距离目标为关节 PWL 欧氏长度 rad；时间目标为连续曲线时长 s，各轴速度/加速度上限 1 rad/s 与 1 rad/s²，端点速度为零。

使用 [rss2025_eizo.json](../configs/rss2025_eizo.json)：δ=ε=0.005、10000 个更新粒子、60 mixing steps、每轮最多 10 个面、configuration margin=0.01、τ=0.5、9 次二分、最多 30 轮。30 来自作者 arXiv TeX 注释，不是正式表格数值。路网 k=10、dCS=dTS=10，离线体素边长 0.06 m，在线 lazy 检查步长 0.1 rad。种子路径与最终候选均接受相同的细采样双碰撞检查，最大采样关节步长 0.005 rad；2 µm 接触带保持未决。

NTO 使用论文 Franka 的 SNOPT/IPOPT 与 Nc=150；149 个内部 k/Nc 绑定来自未发表 TeX 注释，与正文“150 个点”存在计数歧义。原 NTO 完整代码未公开，因此名称明确为 `NTO_PAPER` 重建。旧 `NTO` 的最少 16 控制点表达不代替该轨道。迭代上限、求解容差、时间初值和时长边界均保存于每次 `nto_parameters.json`；它们不是已恢复的作者全部设置。LGCS 保留严格 1e-9 Gurobi 容差和Presolve=0 的本地数值选择，没有改用连续松弛或 LSCS 填补许可失败。

## 题单与配对完整性

smoke 为 10 个固定问题 × 5 方法组合；其中原有 2 个生成失败问题在所有方法下仍是无效输入。dev 为 60 个合法固定问题（4 障碍族 × 5 布局 × 3 对端点），各用 3 份 6000 节点路网，每方法 180 次。20 个场景是统计聚类单位，端点与路网重复不算独立环境。所有问题在算法运行前冻结，没有按某方法成功筛题。族名只描述工作空间几何，不证明 C-space 同伦或全局可行性。

本次 180 组 dev 配对的固定输入、已保存初始 DRM 路径散列及前端结果在五种方法间一致；路径散列不匹配和前端结果不匹配均为 0。原生返回路径先过统一严格 seed 准入门槛，然后才进入任一优化器。完整的配对、状态、失败阶段与文件散列见 [smoke analysis](full_baseline/rss2025_full_smoke_v1_analysis.json)、[dev analysis](full_baseline/rss2025_full_dev_v1_analysis.json) 和 [运行前协议](full_baseline/preregistered_protocol.json)。

## 实际成功与失败

| 方法 | smoke 成功/预定 | dev 成功/180 | dev 成功率 % [95% CI] | 前端失败 | 求解失败 | 碰撞失败 | 超时 | 依赖/许可不支持 | 基础设施/原生退出 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| LSCS | 3/10 | 110/180 | 61.11 [41.67, 79.46] | 69 | 0 | 0 | 0 | 0 | 1 |
| LGCS | 1/10 | 94/180 | 52.22 [35.00, 68.89] | 69 | 0 | 0 | 0 | 16 | 1 |
| NTO-distance | 1/10 | 35/180 | 19.44 [8.89, 31.67] | 69 | 30 | 45 | 0 | 0 | 1 |
| SCSTO | 3/10 | 110/180 | 61.11 [41.67, 79.46] | 69 | 0 | 0 | 0 | 0 | 1 |
| NTO-time | 2/10 | 88/180 | 48.89 [32.22, 66.11] | 69 | 1 | 18 | 3 | 0 | 1 |

上表 dev 各方法分母均为全部 180 次合法查询。smoke 的描述计数包含无效题，不能把 /10 与其合法输入成功率混用。smoke 总状态为 `{"collision_failed": 1, "frontend_failed": 25, "invalid_endpoint": 10, "solver_failed": 2, "success_validated_numerical": 10, "unsupported": 2}`。dev 没有在表中出现的失败类别实际计数为零，不表示已证明算法永不产生该错误。

| 方法 | 原生 DRM 成功 | 严格 seed 准入 | 成功/原生 DRM | 成功/严格 seed | 成功且直线受阻 |
| --- | --- | --- | --- | --- | --- |
| LSCS | 111 | 110 | 110/111 | 110/110 | 75 |
| LGCS | 111 | 110 | 94/111 | 94/110 | 59 |
| NTO-distance | 111 | 110 | 35/111 | 35/110 | 0 |
| SCSTO | 111 | 110 | 110/111 | 110/110 | 75 |
| NTO-time | 111 | 110 | 88/111 | 88/110 | 53 |

这两个条件分母必须与无条件结果并列。原生前端失败不证明连续配置空间无解；求解器报告成功也不自动通过几何/动力学验收。未通过的候选、原生信号退出、预算耗尽和许可失败均保留原始记录。

| 障碍族 | LSCS | LGCS | NTO-distance | SCSTO | NTO-time |
| --- | --- | --- | --- | --- | --- |
| sparse_clutter | 18/45 | 17/45 | 0/45 | 18/45 | 11/45 |
| dense_clutter | 8/45 | 7/45 | 2/45 | 8/45 | 7/45 |
| bottleneck | 45/45 | 35/45 | 12/45 | 45/45 | 42/45 |
| route_choice | 39/45 | 35/45 | 21/45 | 39/45 | 28/45 |

## 成对比较与时间

区间直接读取冻结汇总：scene-level cluster bootstrap，2000 次、seed=45827，场景整组重采样。下表均为 A−B，成功率单位为百分点；成本只比较双方均严格成功的同题，时间目标的成本是轨迹持续时间，距离目标的成本是 rad。冷耗时差则包含双方已完成的失败/超时，不能用成功子集均值替代端到端比较。

| 同目标 A−B | 成功率差 pp [95% CI] | 共同成功 n | 成本差 [95% CI] | 冷耗时差 s [95% CI] |
| --- | --- | --- | --- | --- |
| LSCS − LGCS | 8.89 [3.33, 16.11] | 94 | 0.082 [0.027, 0.153] rad | -0.215 [-0.384, -0.063] |
| LSCS − NTO-distance | 41.67 [26.67, 57.78] | 35 | 0.010 [-0.000, 0.039] rad | -3.296 [-4.999, -1.789] |
| LGCS − NTO-distance | 32.78 [20.00, 45.56] | 35 | 0.010 [0.000, 0.039] rad | -3.081 [-4.694, -1.643] |
| SCSTO − NTO-time | 12.22 [5.00, 20.00] | 88 | -1.303 [-2.244, -0.410] s | -2.538 [-4.089, -1.149] |

成本比较条件于双方成功，存在选择效应；20 个开发场景与本机资源不能支持原论文倍数复现或普遍性能排名。LGCS 的依赖/许可阻塞计入部署条件下的失败分母，但不等于其优化算法本身的可靠性上限。

| 方法 | 冷 p50 / p90 / p95 (s) | 冷最大 (s) | warm p50 (s; n) | TO+CS 均值 (s; n) | 成功成本均值 |
| --- | --- | --- | --- | --- | --- |
| LSCS | 1.767 / 3.368 / 4.374 | 8.427 | 1.075; 179 | 0.249; 110 | 6.373 rad |
| LGCS | 1.792 / 4.573 / 5.324 | 10.231 | 1.111; 179 | 0.712; 94 | 6.038 rad |
| NTO-distance | 1.867 / 13.495 / 20.515 | 54.144 | 1.165; 179 | 5.509; 109 | 4.675 rad |
| SCSTO | 2.092 / 3.619 / 3.872 | 7.926 | 1.386; 179 | 0.271; 110 | 5.799 s |
| NTO-time | 2.343 / 7.690 / 14.207 | 60.214 | 1.612; 176 | 2.904; 107 | 6.816 s |

共同外层预算为每次 60 s，包含进程启动、模型与地图载入、依赖探测、GPU 初始化、规划、恢复、数值验证和输出；超时清理宽限会使观测墙钟略超过 60 s。warm 是同一次冷执行去除初始化的已初始化阶段，不是独立重复的热启动试验。TO+CS 仅为已完成的区域构造、替代路径搜索、优化及恢复计时；未完成阶段不能填零。独立最终验证在外，但 LGCS 替代路径验收、恢复中的覆盖与部分文件 IO 在相应阶段内，不能直接对照论文纯组件时间或据此宣称 17.1 倍加速。

| 离线路网 | 实际节点 | 构建观测墙钟 s | 原生地图字节 |
| --- | --- | --- | --- |
| kinova7-full-n6000-s0-e258c5f9914c | 6000 | 45.878 | 5838597 |
| kinova7-full-n6000-s1-e258c5f9914c | 6000 | 45.838 | 5832009 |
| kinova7-full-n6000-s2-e258c5f9914c | 6000 | 46.369 | 5832481 |

三份离线路网复用上一轮冻结产物，未计入任何在线查询。其第二与第三份构建窗口曾重叠约 6.43 s，故这里保留实际离线观测，不把它当作严格资源隔离的建图性能比较。

## 独立复核与实际恢复

| 方法 | 成功候选检查点 | 这些点 CSD/Drake 分歧 | 成功且触发恢复 |
| --- | --- | --- | --- |
| LSCS | 140602 | 0 | 15 |
| LGCS | 113831 | 0 | 9 |
| NTO-distance | 35932 | 0 | 0 |
| SCSTO | 219711 | 0 | 4 |
| NTO-time | 192663 | 0 | 0 |

smoke/dev 各完成 LSCS/SCSTO、NTO、LGCS 三类独立审查，共六份审计；报告生成前要求全部通过、对应同一封存 manifest。它们重复核对同一批运行证据，不能相加视为新增实验。

运行前 301 项测试通过。运行后再次核对 38 个已安装依赖文件、原冻结清单 518 项与上游 CUDA 源码；全部通过。[最终核对记录](full_baseline/final_checks.json)保留原始源码 commit 与逐文件结果。

LSCS/SCSTO 的独立 CPU 审查分别核对 smoke/dev 的 50/900 份 receipts 与 1098/27280 个封存文件，覆盖 20/360 个 EI-ZO 任务及 9/263 轮已返回且完成验证的候选。其中包含碰撞恢复前的失败候选，不将这些轮次当作独立题目。NumPy 重算折线长度、区域归属与归一化关节路径长度；SciPy BPoly 从控制点和各段时长独立重构曲线，复核固定端点、位置/速度连续性、速度/加速度控制点界与保存采样。最大位置差分别为 3.11e-15/3.55e-15 rad。成功状态同时要求优化器返回、数值验收通过及最终候选/区域/采样证据一致；中断或后续求解失败不会因保存过早先候选而被升级为成功。

NTO 的独立 CPU 审查核对 smoke/dev 的 50/900 份 receipts，并用 SciPy 重构 6/216 个有限候选；最大位置差分别为 2.22e-15/2.66e-15 rad。失败求解器即使留下数值可行候选，也仍是失败。

另检查全部 1540 个已保存碰撞采样文件（包括根目录、各轮及种子路径的重复副本）；其中 7 个文件存在 CSD/Drake 边界判定差异，所有差异都在原先固定的 2 µm 接触带内，没有更改阈值。详见 [接触边界审计](full_baseline/contact_disagreement_audit.json)。这些重复保存点不作为独立实验样本。

对 13 个失败候选的[只读诊断](full_baseline/nto_failure_diagnosis.md)核对了原始 DRM 初值、控制点、时间缩放、固定端点和求解器状态，未发现明确接线错误。其中一个 IPOPT 结果的约束残差为零，但全部穿透点落在相邻配点之间；这说明求解器成功不等于最终轨迹无碰撞。SNOPT 的状态码和原始残差保留，日志未提供实际迭代数时不作推断；不可行约束状态不证明整个规划任务不可行。

最终全量审计另外定位到 `dev-route_choice-002-p00-NTO_PAPER-minimum_distance-48b608ce` 的 Drake `BoxBoxGradient()` 导数断言。它发生在 SNOPT 的盒体碰撞约束求值中，原始状态仍保存为 `solver_failed`，但应解释为后端数值异常，不能归因于 NTO 算法本身，也不能外推为论文球模型的行为。另有 SNOPT info 41/43 的求解器特定错误；完整状态分类见 [NTO 后端失败核对](full_baseline/nto_backend_failures.md)。

LGCS dev 审查核对 180 次尝试，实际接受第二路径 33 次，保存的恢复轮数 20，许可限制失败 16 次。池中区域索引、选中折线路径、原始双路径覆盖和编辑见证均保存并独立复核。`globally_optimal` 仅指已保存有限区域图中的优化结论，不是整个自由配置空间的全局最优。

六份审计证据：[smoke LSCS/SCSTO](full_baseline/smoke_scs_audit.json)、[dev LSCS/SCSTO](full_baseline/dev_scs_audit.json)、[smoke NTO](full_baseline/smoke_nto_audit.json)、[dev NTO](full_baseline/dev_nto_audit.json)、[smoke LGCS](full_baseline/independent_lgcs_smoke_v1_strict/audit.json)、[dev LGCS](full_baseline/independent_lgcs_dev_v1/audit.json)。这些 CPU 审查读取实际封存数组与碰撞标志，不重新运行 GPU 碰撞；实际双 checker 数据来自规划后的在线验收。有限碰撞采样不是数学连续安全证明。恢复后失败的新一轮可能没有候选，此时根目录 validation 明确表示最后一个已验证候选，失败成本不进入成功对照。

此前只读接线审查发现并修复了 LGCS 恢复时仅保护 native float32 路径、可能丢失固定 double 端点的问题；当前实现把固定端点贯穿 native 交接和双路径覆盖。详细源码与参数边界见 [覆盖审计](full_baseline/coverage_audit.md)、[原文参数审计](paper_algorithm/source_audit.md)。CUDA API 不暴露每次膨胀的统计接受/截断状态，30 轮上限不能冒充每个区域的 δ/ε 证书；固定随机种子也不保证 CUDA 逐位确定性。

固定任务 `dev-sparse_clutter-000-p01` 在第三份 roadmap 上，于五方法中重复触发同一个 `drm_planner.cpp:249 / closest_node >= 0` 原生断言，发生在任何优化器之前。端点审计仍通过；这些进程退出以基础设施失败保留，不能归为优化器失败或任务不可行。见[五次原始错误及端点证据](full_baseline/native_frontend_failures.json)。未修改上游 CUDA 来隐藏该失败。

## 论文原实验的边界与下一阶段

本轮于 2026-10-28T03:04:27.873551+00:00 重新核查上游完整文件树，HEAD `7661a97bb7c4931be73d2bc4f8c45c3094ff476e`（提交时间 2026-03-20T02:28:29Z），FR3/Franka 文件匹配仍为空。见 [实际 API 记录](full_baseline/current_upstream_assets.json)。该文件树检查本身不是整个网络不存在资产的证明，但当前仍没有作者 33 球模型、过滤和原始题单可用。

历史 [Forest 400 题](../../../reproduction/reports/paper_completion/reproduction_report.md) 已覆盖 10 环境 × 10 路网种子 × 4 规模、五种方法，现已重新核对全部保存文件，可作为独立 `forest2d_reference` 重建轨道引用；原随机题单未公开，不能称逐题精确复跑。历史 FR3 400 题保留为重建诊断：拟合球 link5–hand 曾出现 −35.1577 mm 穿透，而官方碰撞网格凸包相距 +37.0481 mm，且总检查对数 851 不等于论文 1187；这些伪障碍使其不适合冒充原论文几何基线。KUKA iiwa、三台 D415、标定、感知与 15 条物理执行记录也未恢复，本轮没有生成替代的硬件成绩。

后续可直接使用本次固定 Kinova 五方法协议开展方法研究；优先关注共同 DRM 前端连接失败、NTO 的离散碰撞约束与终检差距，以及 LGCS 的许可限制。若修改前端、优化表达或求解器，创建新的声明版本并保留同题/同预算对照。当前只完成单规模 6000 节点 × 3 种子的开发协议，没有把它包装为原论文 10×10×4 规模研究。**420 个 test 仍只生成并冻结，本轮没有规划、调参或完整评分。**

## 保存轨迹回放

[回放索引与实际命令](full_baseline/replay_index.json)优先选取五方法都成功且直连被阻塞的同一题，仅用于展示，不改变完整统计。实际选中 `dev-bottleneck-000-p00`，其直连碰撞标记为 `False`；若为 false，展示的是直连可行问题，并非困难绕障碍案例。[LSCS](full_baseline/replays/LSCS/replay.html)、[LGCS](full_baseline/replays/LGCS/replay.html)、[SCSTO](full_baseline/replays/SCSTO/replay.html)、[NTO-distance](full_baseline/replays/NTO_distance/replay.html)、[NTO-time](full_baseline/replays/NTO_time/replay.html)均读取保存数组。[NTO 碰撞失败](full_baseline/replays/NTO_collision_failure/replay.html)保留失败候选；[LGCS 许可失败](full_baseline/replays/LGCS_license_failure/replay.html)仅展示已有种子路径，不将其称为 LGCS 优化结果。

## 可复现命令与证据入口

从仓库根目录执行。以下两个 batch 命令对应本次协议；已存在的输出目录不能覆盖，重新运行请改为新目录，续跑使用 resume。完成分析与报告会拒绝不完整或未通过审计的结果。

```bash
bash local/python.sh -m benchmarks.generic_motion_v1 batch \
  --manifest benchmarks/generic_motion_v1/manifests/smoke.jsonl \
  --roadmaps benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s000 \
  --config benchmarks/generic_motion_v1/configs/rss2025_eizo.json \
  --methods EI_ZO LGCS NTO_PAPER --objectives minimum_distance minimum_time \
  --timeout 60 --output benchmarks/generic_motion_v1/results/rss2025_full_smoke_v1

bash local/python.sh -m benchmarks.generic_motion_v1 batch \
  --manifest benchmarks/generic_motion_v1/manifests/dev.jsonl \
  --roadmaps benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s000 \
    benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s001 \
    benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s002 \
  --config benchmarks/generic_motion_v1/configs/rss2025_eizo.json \
  --methods EI_ZO LGCS NTO_PAPER --objectives minimum_distance minimum_time \
  --timeout 60 --output benchmarks/generic_motion_v1/results/rss2025_full_dev_v1

bash local/python.sh -m benchmarks.generic_motion_v1 resume \
  --run benchmarks/generic_motion_v1/results/rss2025_full_dev_v1
bash local/python.sh -m benchmarks.generic_motion_v1 verify \
  --run benchmarks/generic_motion_v1/results/rss2025_full_dev_v1
bash local/python.sh -m benchmarks.generic_motion_v1.reports.full_baseline.collect_results
bash local/python.sh -m benchmarks.generic_motion_v1.reports.full_baseline.independent_scs_audit \
  --run benchmarks/generic_motion_v1/results/rss2025_full_dev_v1 \
  --output benchmarks/generic_motion_v1/reports/full_baseline/dev_scs_audit.json
bash local/python.sh -m benchmarks.generic_motion_v1.reports.full_baseline.independent_nto_audit \
  --run benchmarks/generic_motion_v1/results/rss2025_full_dev_v1 \
  --output benchmarks/generic_motion_v1/reports/full_baseline/dev_nto_audit.json
bash local/python.sh -m benchmarks.generic_motion_v1.reports.full_baseline.independent_lgcs_audit \
  --run benchmarks/generic_motion_v1/results/rss2025_full_dev_v1 \
  --output benchmarks/generic_motion_v1/reports/full_baseline/independent_lgcs_dev_v1
bash local/python.sh -m benchmarks.generic_motion_v1.reports.full_baseline.contact_audit
bash local/python.sh -m benchmarks.generic_motion_v1.reports.full_baseline.make_replays
bash local/python.sh -m benchmarks.generic_motion_v1.reports.full_baseline.finalize_evidence
bash local/python.sh -m benchmarks.generic_motion_v1.reports.full_baseline.render_report
```

smoke 的独立审计使用同样入口，将 dev run/输出替换为 smoke。回放使用 `python -m benchmarks.generic_motion_v1 replay --attempt <封存的 attempts/0001 路径> --output <新目录>`，读取保存轨迹，不重新规划。CLI 参数仍以本项目 `bash local/python.sh` 启动。

[smoke 原始 run](../results/rss2025_full_smoke_v1/run_manifest.json)、[dev 原始 run](../results/rss2025_full_dev_v1/run_manifest.json)、[完整汇总及配对区间](../results/rss2025_full_dev_v1/summary/summary.json)、[完成与输入 SHA 核对](full_baseline/completion.json)、[smoke 日志](full_baseline/smoke_batch.log)、[dev 日志](full_baseline/dev_batch.log)、[冻结前测试](full_baseline/tests_preflight_final.txt)。每个 attempt 保存 request、process、receipt、阶段日志、输入、原始数组、多面体、优化控制点和数值验证；源码快照及路网散列在 run_manifest 中。实际执行的失败命令或审计失败记录仍保留，不以本报告替代原始证据。

本报告由 `reports/full_baseline/render_report.py` 从完成的分析、运行汇总和六份独立审计生成，不调用规划器，不生成实验状态。
