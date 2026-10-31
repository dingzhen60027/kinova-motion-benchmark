# 扩展运动规划基线：完整接入与开发集实测

**RRTConnect、RRT*、CHOMP、STOMP、TrajOpt 已接入统一 CLI，并与本轮重新运行的 LSCS 完成配对测试。** 本轮封存 1,140 个槽位：1,128 次合法输入调用、2 道已冻结生成失败输入 × 6 方法形成的 12 个无效槽位，数值验证成功共 717 条。全部为 smoke/dev；历史结果未混入这次运行，正式 test 没有新增规划调用。

## 固定输入、方法和评价口径

开发集是 60 道固定题，使用种子 0、1、2 的三次配对重复，每方法 180 次合法调用；smoke 每方法登记 10 题，其中 8 题合法。三次重复不是三个新场景。所有方法保持相同的七关节 Kinova 模型、碰撞过滤对、障碍体素球、端点、独立终检和 60 秒外部预算。

| 方法 | 执行实现 | 初始化与原生目标 |
| --- | --- | --- |
| LSCS | CSDecomp DRM → EI-ZO → 作者 SCSPlanning / Clarabel | 固定 DRM 路径；凸走廊内 PWL 弧长 |
| RRTConnect | 官方 OMPL 2.0.1 原生实现 | 独立双向树；首先求可行路径，再原生顶点简化 |
| RRT* | 官方 OMPL 2.0.1 原生实现 | 独立采样与重连；固定时间内优化路径长度 |
| CHOMP | 固定端点加速度度量的协变下降 | 相同 DRM 路径；平滑代价＋规范模型距离势的 C-space 弧长积分 |
| STOMP | 官方 ros-industrial/stomp 未修改的 C++ 核心 | 相同 DRM 路径；原生控制代价＋规范模型距离势任务回调 |
| TrajOpt | 作者 BasicTrustRegionSQP ＋ BPMPD 2.21 | 相同 DRM 路径；离散平滑目标及有符号距离碰撞约束 |

公共质量指标统一为通过独立验证的七关节 PWL 欧氏弧长，单位 rad。CHOMP/STOMP/TrajOpt 的原生平滑与障碍目标另外保存，不冒称与 PWL 弧长完全相同。CHOMP 使用原论文的协变更新，障碍泛函明确适配为规范几何的 C-space 距离势积分；没有更换机械臂碰撞模型。五个扩展方法属于几何路径轨道，未虚构连续最短时间优化器。

## 开发集主结果

| 方法 | 验证成功/合法调用 | 成功率及场景 95% CI | 冷启动均值 / P50 / P95，s | 热查询均值，s | 成功路径长度均值，rad |
| --- | --- | --- | --- | --- | --- |
| LSCS | 110/180 | 61.1% [41.7, 79.5] | 1.947 / 1.842 / 4.325 | 1.213 (n=180) | 6.373 (n=110) |
| RRTConnect | 146/180 | 81.1% [66.7, 93.3] | 3.509 / 1.616 / 10.686 | 2.806 (n=180) | 7.355 (n=146) |
| RRT* | 120/180 | 66.7% [50.6, 81.7] | 11.194 / 11.232 / 11.799 | 10.510 (n=180) | 6.676 (n=120) |
| CHOMP | 109/180 | 60.6% [41.7, 78.9] | 3.356 / 2.393 / 8.586 | 2.660 (n=180) | 6.558 (n=109) |
| STOMP | 110/180 | 61.1% [41.7, 79.5] | 3.721 / 4.697 / 6.780 | 3.010 (n=180) | 7.267 (n=110) |
| TrajOpt | 97/180 | 53.9% [37.2, 70.0] | 3.173 / 2.317 / 8.897 | 2.452 (n=180) | 5.853 (n=97) |

冷启动均值和分位数包含全部合法调用的成功、前端失败、求解失败和超时。路径长度只统计经过公共终检的输出；不同方法成功集合不同，不能据此按该均值直接排名。CI 使用场景聚类 bootstrap：PCG64，2,000 次，seed=45827；同场景的多个端点及重复种子一起重采样。

## 失败分类与求解收敛

| 方法 | collision_failed | frontend_failed | planner_timeout | solver_failed | validation_undetermined |
| --- | --- | --- | --- | --- | --- |
| LSCS | 0 | 70 | 0 | 0 | 0 |
| RRTConnect | 0 | 0 | 33 | 0 | 1 |
| RRT* | 0 | 0 | 60 | 0 | 0 |
| CHOMP | 0 | 70 | 0 | 1 | 0 |
| STOMP | 0 | 70 | 0 | 0 | 0 |
| TrajOpt | 13 | 70 | 0 | 0 | 0 |

原生近似路径不计成功；planner_timeout 记录预算内没有原生精确解，timeout 是外部或共享期限中断。这些状态以及求解器不可行返回都不能证明整道运动规划题无解。失败时已产生的轨迹、搜索树、求解日志和异常信息均保留；未产生候选的失败不补造轨迹。

| 方法 | 收敛且验证成功 | 未收敛但验证成功 | 验证成功、收敛未知 | 合法调用中收敛未知 | 原生终止统计 |
| --- | --- | --- | --- | --- | --- |
| LSCS | 0 | 0 | 110 | 180 | author_scsplanning_get_points_returned: 110; no_candidate: 70 |
| RRTConnect | 0 | 0 | 146 | 180 | Exact solution: 147; no_candidate: 33 |
| RRT* | 0 | 0 | 120 | 180 | Exact solution: 120; no_candidate: 60 |
| CHOMP | 0 | 0 | 109 | 180 | no_candidate: 70; unknown: 110 |
| STOMP | 0 | 0 | 110 | 180 | no_candidate: 70; unknown: 110 |
| TrajOpt | 91 | 6 | 0 | 70 | CONVERGED: 103; PENALTY_ITERATION_LIMIT: 6; SCO_ITERATION_LIMIT: 1; no_candidate: 70 |

收敛只认候选中明确的 solver_converged 布尔值；缺失标记保持未知，包含未生成候选的前端失败。TrajOpt 在内部时间或迭代上限处仅可返回已经被 SQP 接受且确实改变的迭代点，仍须通过公共终检；这些输出与原生 CONVERGED 分开统计，初始化路径不能伪装成超时后的优化结果。

## 同一题目的配对比较

| A − B | 合法配对 / 共同成功 | 成功率差 pp [95% CI] | 共同成功长度差 rad [95% CI] | 全部合法冷延迟差 s [95% CI] |
| --- | --- | --- | --- | --- |
| CHOMP − LSCS | 180 / 109 | -0.556 [-1.667, 0.000] | 0.226 [0.035, 0.460] | 1.409 [0.844, 2.000] |
| CHOMP − RRT* | 180 / 101 | -6.111 [-16.111, 2.222] | 0.165 [-0.098, 0.485] | -7.838 [-8.639, -7.026] |
| CHOMP − RRTConnect | 180 / 109 | -20.556 [-35.556, -7.778] | 0.253 [-0.010, 0.534] | -0.153 [-2.131, 1.692] |
| CHOMP − STOMP | 180 / 109 | -0.556 [-1.667, 0.000] | -0.652 [-0.802, -0.518] | -0.364 [-0.758, -0.020] |
| CHOMP − TrajOpt | 180 / 97 | 6.667 [2.222, 11.667] | 0.448 [0.187, 0.804] | 0.183 [-0.243, 0.574] |
| LSCS − RRT* | 180 / 102 | -5.556 [-15.556, 2.778] | -0.012 [-0.201, 0.167] | -9.247 [-9.566, -8.949] |
| LSCS − RRTConnect | 180 / 110 | -20.000 [-35.000, -7.222] | 0.064 [-0.240, 0.351] | -1.562 [-3.058, -0.180] |
| LSCS − STOMP | 180 / 110 | 0.000 [0.000, 0.000] | -0.894 [-1.188, -0.629] | -1.773 [-2.355, -1.186] |
| LSCS − TrajOpt | 180 / 97 | 7.222 [2.778, 12.222] | 0.256 [0.072, 0.458] | -1.226 [-1.858, -0.695] |
| RRT* − RRTConnect | 180 / 120 | -14.444 [-24.444, -6.111] | 0.033 [-0.259, 0.269] | 7.685 [6.321, 8.925] |
| RRT* − STOMP | 180 / 102 | 5.556 [-2.778, 15.556] | -0.858 [-1.241, -0.584] | 7.473 [6.655, 8.323] |
| RRT* − TrajOpt | 180 / 92 | 12.778 [5.000, 21.667] | 0.382 [0.066, 0.692] | 8.021 [7.147, 8.766] |
| RRTConnect − STOMP | 180 / 110 | 20.000 [7.222, 35.000] | -0.957 [-1.250, -0.696] | -0.211 [-2.122, 1.810] |
| RRTConnect − TrajOpt | 180 / 97 | 27.222 [15.542, 41.111] | 0.237 [-0.098, 0.567] | 0.336 [-1.554, 2.239] |
| STOMP − TrajOpt | 180 / 97 | 7.222 [2.778, 12.222] | 1.047 [0.705, 1.474] | 0.547 [0.077, 1.046] |

差值方向严格为 A − B。长度差只基于双方共同成功，负值表示 A 的已验证路径更短；成功率差和冷延迟差保留全部合法配对，冷延迟包含失败。条件质量结果不能外推为全体任务上的最优性结论。

## 障碍族、直连标签与 smoke 对照

| 开发集障碍族 | LSCS | RRTConnect | RRT* | CHOMP | STOMP | TrajOpt |
| --- | --- | --- | --- | --- | --- | --- |
| sparse_clutter | 18/45 | 30/45 | 23/45 | 17/45 | 18/45 | 15/45 |
| dense_clutter | 8/45 | 32/45 | 17/45 | 8/45 | 8/45 | 8/45 |
| bottleneck | 45/45 | 45/45 | 42/45 | 45/45 | 45/45 | 39/45 |
| route_choice | 39/45 | 39/45 | 38/45 | 39/45 | 39/45 | 35/45 |

| 开发集固定直连标签 | LSCS | RRTConnect | RRT* | CHOMP | STOMP | TrajOpt |
| --- | --- | --- | --- | --- | --- | --- |
| 直连检出碰撞 | 75/141 | 107/141 | 83/141 | 74/141 | 75/141 | 62/141 |
| 直连未检出碰撞 | 35/39 | 39/39 | 37/39 | 35/39 | 35/39 | 35/39 |

| smoke 分层 | LSCS | RRTConnect | RRT* | CHOMP | STOMP | TrajOpt |
| --- | --- | --- | --- | --- | --- | --- |
| 全部合法输入 | 3/8 | 7/8 | 6/8 | 3/8 | 3/8 | 3/8 |
| 新增障碍 | 2/6 | 5/6 | 4/6 | 2/6 | 2/6 | 2/6 |
| 无新增障碍对照 | 1/2 | 2/2 | 2/2 | 1/2 | 1/2 | 1/2 |

直连标签在算法运行前冻结，不能用方法成败改题。开发集没有 open_control；smoke 的两题无新增障碍对照共享同一基础世界，单列且不伪造多场景 CI。bottleneck/route_choice 是工作空间几何描述，不代表已知 C-space 同伦类别。

## 预算、阶段耗时与离线费用

外部冷启动看门狗的终止触发期限为 60 秒，包含解释器启动、模型与依赖加载、GPU 初始化、规划、优化和最终验证。到期触发进程终止后，最多等待 2 秒清理，再强制结束；实际冷延迟包括清理耗时，不截断为 60 秒。热查询是同一次冷调用中从规划查询阶段起记录的时间；不是第二次独立热启动试验。未完成后端的部分计时仅保留在原始记录和冷延迟，不冒充完整阶段均值。

RRT* 内部规划上限 10 秒，RRTConnect 在找到首条精确解时可提前退出；两者使用最多 100 次顶点简化、0.5 秒简化预算。CHOMP/STOMP 最多 40/40 次迭代；TrajOpt 每个罚系数阶段最多 50 次 SQP、内部 45 秒。扩展方法预留 5 秒用于公共验证；所有阶段仍受同一外部期限约束。

| 方法 | 完整后端计时数 | DRM 查询 s | EI-ZO s | 独立采样 s | 轨迹优化 s | 终检 s |
| --- | --- | --- | --- | --- | --- | --- |
| LSCS | 110 | 0.076 (n=130) | 0.240 (n=110) | — (n=0) | 0.002 (n=110) | 0.884 (n=110) |
| RRTConnect | 147 | — (n=0) | — (n=0) | 0.413 (n=147) | — (n=0) | 0.775 (n=147) |
| RRT* | 120 | — (n=0) | — (n=0) | 10.087 (n=120) | — (n=0) | 0.671 (n=120) |
| CHOMP | 110 | 0.069 (n=130) | — (n=0) | — (n=0) | 2.816 (n=110) | 0.700 (n=110) |
| STOMP | 110 | 0.074 (n=130) | — (n=0) | — (n=0) | 3.316 (n=110) | 0.768 (n=110) |
| TrajOpt | 110 | 0.073 (n=130) | — (n=0) | — (n=0) | 2.531 (n=110) | 0.639 (n=110) |

DRM 条件成功率和共用种子路径 SHA 检查只适用于 LSCS/CHOMP/STOMP/TrajOpt。OMPL 从固定端点独立搜索；其路网标签只是重复种子的配对标识，不加载路网，也不承担离线路网构建费用。

| 路网种子 | 请求 / 实际节点 | 已封存构建时间 s | 构建时间 / 60，s/查询 | 构建时间 / 1000，s/查询 |
| --- | --- | --- | --- | --- |
| 0 | 6000 / 6000 | 45.878 | 0.765 | 0.046 |
| 1 | 6000 / 6000 | 45.838 | 0.764 | 0.046 |
| 2 | 6000 / 6000 | 46.369 | 0.773 | 0.046 |

上表读取既有封存路网的实际构建记录，本轮没有重新构图。Q 次请求的费用模型是“在线冷延迟＋构建时间/Q”；摊销列是明确的算术情景，不是额外测量，也没有从在线时间中减去缓存或构图开销。

## 环境与源码证据

执行源码提交：`3414dd08f165e0cee0856e86007b53d9b6e97720`；本轮所有运行绑定同一 runtime_source_hashes 与完整配置。
GPU：`Quadro RTX 8000, 570.211.01, 49152 MiB`；解释器：`/home/eric/wjg/wd/csdecomp/.conda/envs/csdecomp/bin/python`；Python `3.10.22`。
| 规范模型绑定 | SHA256 |
| --- | --- |
| robot | a25f447c8caf4aff6dc76196c3d57cc940f00e802367f48034f06a9973927c75 |
| collision geometry / filters | 896ab9f55c5d8081b843358bd85cde2daae1f432564593f86218bf61733ce4fa |

| 依赖 | 实际版本 |
| --- | --- |
| clarabel | 0.11.1 |
| csdecomp | 0.1.2 |
| drake | 1.51.1 |
| gurobipy | 13.0.3 |
| numpy | 2.2.6 |
| pybezier | 0.1.4 |
| scipy | 1.15.3 |
| scsplanning | 0.1.0 |

| 扩展依赖 | 固定来源 / SHA |
| --- | --- |
| OMPL 2.0.1 | c509861210a63ec962bbec72c52823abc16b102e |
| OMPL 官方 wheel SHA256 | 346665d130ef5c2bbd80686aed3fbbd9a4d1097d0f0fd92924aafbdc0497742d |
| STOMP 官方核心 commit | b1a87c80f7338caae25a5c689b876da15492aa75 |
| TrajOpt 作者 commit | 9c634c060f7e2d61c4a888b0a5700d668a5d16f8 |
| TrajOpt 传输补丁 SHA256 | b7d4f47b3f34ea9ab23de17f9e35d67b681320689cb71f5fa7ad68a5c6b35c6a |

现有 Conda 保持原状。OMPL、编译桥接头文件和 TrajOpt/STOMP 构建产物位于独立依赖目录；执行冻结同时校验已安装库、原生二进制和绑定源码。STOMP 核心未修改。TrajOpt 保持作者 SQP 优化逻辑，修复旧 BPMPD 管道传输对部分读写、EOF/关闭及阻塞中断的处理；修改范围和差异补丁单独保存。

方法来源：[OMPL 安装与原生库](https://ompl.kavrakilab.org/installation.html)、[CHOMP 原论文](https://publications.ri.cmu.edu/chomp-covariant-hamiltonian-optimization-for-motion-planning)、[STOMP 官方 Task 接口](https://github.com/ros-industrial/stomp/blob/b1a87c80f7338caae25a5c689b876da15492aa75/include/stomp/task.h)、[TrajOpt 作者源码](https://github.com/joschu/trajopt/tree/9c634c060f7e2d61c4a888b0a5700d668a5d16f8)。

## 验收、限制与复现命令

独立最终审计通过；原始文件、receipt、输入绑定及数组代价核查详见 [final_audit.json](extended_baselines_v1/final_audit.json)。审计检查数：222397。
公共验收要求端点、关节限位、原碰撞对、CUDA 与独立 Drake 几何检查一致通过；0.005 rad 有限采样通过不等于连续无碰撞数学证明。随机任务的可行性保持 unknown；场景或端点没有因任何算法成功与否被替换。
当前主结论限于冻结 Kinova 开发分布和本机硬件、给定预算与初始化。独立采样与 DRM 热启动优化具有不同前端，因此主表比较端到端系统，条件 DRM 表只解释共享前端子集。后续算法应保持冻结评价协议，另注册新研究比较。

正式 test 文件保持以下冻结校验值；本轮未重新评测或调参：

| 受保护文件 | SHA256 |
| --- | --- |
| test.SHA256SUMS | 7eff846c9ce1dcc2b268a2acdde98a93ee7cd3d1658f13d35b80c74eb6b581bd |
| test.jsonl | 9fcbe5ccae28d8681586b1ac6c9986f51f7cc7b5cdc19352c101910e3b6ed03b |

完整配置、命令与数组证据：[配置](../configs/extended_baselines.json)、[实际命令账本](extended_baselines_v1/commands.jsonl)、[逐调用 CSV](extended_baselines_v1/metrics/observations.csv)、[指标 JSON](extended_baselines_v1/metrics/metrics.json)、[冻结记录](extended_baselines_v1/execution_freeze.json)。

[交互回放图库](extended_baselines_v1/replays/index.html)。

以下命令直接摘自实际执行账本；按记录顺序列出完成事件，返回码保留。重跑应更换输出目录，已有槽位使用 resume，不能覆盖封存证据。

`smoke_s000/prepare`：退出码 0。

```bash
/home/eric/wjg/wd/csdecomp/.conda/envs/csdecomp/bin/python -m benchmarks.generic_motion_v1 batch --manifest /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/manifests/smoke.jsonl --roadmaps /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s000 --output /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/extended_baselines_v1_smoke_s000 --methods EI_ZO RRTCONNECT RRTSTAR CHOMP STOMP TRAJOPT --objectives minimum_distance --config /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/configs/extended_baselines.json --timeout 60 --prepare-only
```

`dev_s000/prepare`：退出码 0。

```bash
/home/eric/wjg/wd/csdecomp/.conda/envs/csdecomp/bin/python -m benchmarks.generic_motion_v1 batch --manifest /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/manifests/dev.jsonl --roadmaps /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s000 --output /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/extended_baselines_v1_dev_s000 --methods EI_ZO RRTCONNECT RRTSTAR CHOMP STOMP TRAJOPT --objectives minimum_distance --config /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/configs/extended_baselines.json --timeout 60 --prepare-only
```

`dev_s001/prepare`：退出码 0。

```bash
/home/eric/wjg/wd/csdecomp/.conda/envs/csdecomp/bin/python -m benchmarks.generic_motion_v1 batch --manifest /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/manifests/dev.jsonl --roadmaps /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s001 --output /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/extended_baselines_v1_dev_s001 --methods EI_ZO RRTCONNECT RRTSTAR CHOMP STOMP TRAJOPT --objectives minimum_distance --config /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/configs/extended_baselines.json --timeout 60 --prepare-only
```

`dev_s002/prepare`：退出码 0。

```bash
/home/eric/wjg/wd/csdecomp/.conda/envs/csdecomp/bin/python -m benchmarks.generic_motion_v1 batch --manifest /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/manifests/dev.jsonl --roadmaps /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s002 --output /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/extended_baselines_v1_dev_s002 --methods EI_ZO RRTCONNECT RRTSTAR CHOMP STOMP TRAJOPT --objectives minimum_distance --config /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/configs/extended_baselines.json --timeout 60 --prepare-only
```

`smoke_s000/resume`：退出码 0。

```bash
/home/eric/wjg/wd/csdecomp/.conda/envs/csdecomp/bin/python -m benchmarks.generic_motion_v1 resume --run /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/extended_baselines_v1_smoke_s000
```

`smoke_s000/resume`：退出码 0。

```bash
/home/eric/wjg/wd/csdecomp/.conda/envs/csdecomp/bin/python -m benchmarks.generic_motion_v1 resume --run /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/extended_baselines_v1_smoke_s000
```

`dev_s000/resume`：退出码 0。

```bash
/home/eric/wjg/wd/csdecomp/.conda/envs/csdecomp/bin/python -m benchmarks.generic_motion_v1 resume --run /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/extended_baselines_v1_dev_s000
```

`dev_s001/resume`：退出码 0。

```bash
/home/eric/wjg/wd/csdecomp/.conda/envs/csdecomp/bin/python -m benchmarks.generic_motion_v1 resume --run /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/extended_baselines_v1_dev_s001
```

`dev_s002/resume`：退出码 0。

```bash
/home/eric/wjg/wd/csdecomp/.conda/envs/csdecomp/bin/python -m benchmarks.generic_motion_v1 resume --run /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/extended_baselines_v1_dev_s002
```

`all/metrics`：退出码 0。

```bash
/home/eric/wjg/wd/csdecomp/.conda/envs/csdecomp/bin/python -m benchmarks.generic_motion_v1.runners.extended_metrics --runs /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/extended_baselines_v1_smoke_s000 /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/extended_baselines_v1_dev_s000 /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/extended_baselines_v1_dev_s001 /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/extended_baselines_v1_dev_s002 --output /home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/reports/extended_baselines_v1/metrics
```

辅助复现入口已接入同一 CLI：`methods`、`run`、`batch`、`resume`、`verify`、`summarize`、`replay`。独立 OMPL 可省略 `--roadmaps` 并使用 `--seed-indices`；共享前端方法必须提供路网。
审计、回放与报告生成的实际调用另存于[后处理命令账本](extended_baselines_v1/postprocess_commands.jsonl)。
