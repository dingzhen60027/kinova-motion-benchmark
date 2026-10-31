# 固定开发题上的 EI-ZO / NTO 配对审计

`results/dev_paired_v2` 的 48 个计划任务全部完成并封存：24 个 `success_validated_numerical`，24 个 `frontend_failed`。两个方法在两种目标下各成功 6/12。此结果用于检查平台配对、求解与验证是否工作；它只覆盖四个预先固定的 short 题、四个环境布局，不能作为 60 题开发集或一般机械臂规划的总体方法排名。

本报告仅从已封存 JSON/NPZ 及汇总提取结果。审计实际重新计算了端点误差和轨迹导数控制点界，没有构造 `Model`、执行碰撞查询、调用 GPU 或重新规划。

## 1. 固定设计与完整性

- 运行源码 commit：`5dc135f67f0ec2817fdd7da93260f8e2349a38b9`。
- 输入选择：manifest 顺序的 `first-per-family`，四个障碍族各第一题；不含 `open_control`。
- 运行规模：4 个固定题 × 3 张 roadmap（seeds 0/1/2）× 2 目标 × 2 方法 = 48 jobs。
- 每个 job 的外层冷启动 wall budget 为 60 s。模型、固定端点、障碍球、实际 roadmap、method seed 和数值终检按同题配对；没有更换某一方法的任务。
- `batch.verify` 实际返回 `integrity_passed=true`、`complete=true`、`completed_jobs=48`、`errors=[]`、无未封存 attempt。
- `reports/resume_and_pairing_acceptance.json` 记录真实续跑验收：开始的两个 attempt 逐字节保留；全部完成后再次 resume 没有改变原 attempt 文件。24 个方法对均有一致的 path 文件存在状态；其中 12 个双方确有 path 的方法对，数组全部相同。另 12 对均在前端失败，没有伪造路径。

三个 seed 是同一场景上的重复方法运行，不是三个独立环境。以下四题全部来自 normalized endpoint distance 的 short 分层 `[0.1, 0.3)`：

| 固定题 | 关节欧氏距离 / rad | 归一化端点距离 | 直连数值检查 | 两方法在两目标下的结果 |
|---|---:|---:|---|---|
| dev-sparse_clutter-000-p00 | 4.283300456 | 0.298601976 | 碰撞 | 12 次前端失败 |
| dev-dense_clutter-000-p00 | 3.567684891 | 0.273403547 | 碰撞 | 12 次前端失败 |
| dev-bottleneck-000-p00 | 2.929760864 | 0.185023498 | 未发现碰撞 | 12 次数值验证成功 |
| dev-route_choice-000-p00 | 3.036416336 | 0.197748521 | 未发现碰撞 | 12 次数值验证成功 |

场景名称只描述 workspace 障碍安排，不表示这两个成功题必须经过 C-space 瓶颈或选择不同同伦路线。实际上，成功题全部可数值直连，所有返回的 native DRM path 都是两个端点。因此本批成功子集没有验证复杂绕障能力。端点分层的归一化定义是每关节范围归一后的 RMS；`validation.normalized_path_length` 使用范围归一后的 L2 弧长，两者相差维数因子，不能直接作无单位绕路比。

## 2. 成功和失败分母

| 方法 | 目标 | 合法任务 | 成功 | 前端失败 | 无条件成功率 | DRM 成功后的条件率 |
|---|---|---:|---:|---:|---:|---:|
| EI_ZO | minimum_distance | 12 | 6 | 6 | 50% | 6/6 |
| NTO | minimum_distance | 12 | 6 | 6 | 50% | 6/6 |
| EI_ZO | minimum_time | 12 | 6 | 6 | 50% | 6/6 |
| NTO | minimum_time | 12 | 6 | 6 | 50% | 6/6 |

失败全部保留为 `frontend_failed`，阶段为 `drm_path`，日志为 `[DRMPlanner] No collision-free path exists in roadmap.`。这个错误仅说明当前离散 roadmap 前端未返回连接，不能证明连续规划问题无解。没有超时、CUDA 错误或局部求解失败被删掉；本次固定子集实际没有产生这些状态。

scene-cluster bootstrap 使用 PCG64 seed 45827、2000 次重复，把同一布局的所有 seed 和配对方法一起重采样。各方法/目标的无条件成功率 95% 区间为 `[0, 1]`（四个布局）；条件成功率区间 `[1, 1]` 只来自两个成功布局。每个目标下的配对成功率差 EI_ZO−NTO 为 0，区间 `[0, 0]`，因为本次四题上逐对成败完全相同；这不是方法等价性证明。

## 3. 配对质量与计时

质量比较只使用双方成功的同题、同 roadmap、同 seed、同目标的 6 对；失败没有填补成零代价，也没有跨不同成功子集比较均值。每个目标的质量估计实际上只有两个独立布局。

| 目标 | EI_ZO 共同成功均值 | NTO 共同成功均值 | EI_ZO−NTO 平均差 | scene-bootstrap 95% 区间 |
|---|---:|---:|---:|---:|
| minimum_distance，rad | 2.983088600 | 2.983088600 | 3.7007434e-16 | [2.9605947e-16, 4.4408921e-16] |
| minimum_time，s | 4.081117482 | 3.495128953 | 0.585988529 | [0.584239929, 0.587737130] |

路径长度差处于浮点舍入量级，实际两者都等于直线端点距离，不能解释成可辨认的路径质量差。时间目标上，这两个成功题的 NTO 轨迹实际较短。EI-ZO 使用作者 LSCS/SCSTO 加开源 Clarabel，NTO 使用适配的 Drake KTO/IPOPT；其轨迹表示和局部优化结构不同，这一小子集的结果不构成全局最优性或总体优势结论。只有两个布局时，bootstrap 区间也不能表达未观察到的复杂场景变化。

外层冷启动包含进程启动、import、CUDA context、模型和 roadmap 加载、在线规划、验证与输出。下表时间使用全部 12 个已结束合法任务，包含快速前端失败；因此不能将较短均值直接解释为成功规划更快。

| 方法 | 目标 | 冷启动均值 / s | p50 / s | p90 / s | p95 / s | 已初始化在线段均值 / s |
|---|---|---:|---:|---:|---:|---:|
| EI_ZO | minimum_distance | 1.042142 | 1.001136 | 1.311474 | 1.315333 | 0.218892 |
| NTO | minimum_distance | 1.099914 | 1.089557 | 1.334804 | 1.340072 | 0.245192 |
| EI_ZO | minimum_time | 1.167494 | 1.094573 | 1.534927 | 1.537352 | 0.328904 |
| NTO | minimum_time | 1.399355 | 1.324600 | 2.143900 | 2.172157 | 0.550756 |

对应的配对冷启动差（EI_ZO−NTO）在 minimum_distance 为 −0.057772 s，95% 区间 `[−0.188179, 0.012029]`；minimum_time 为 −0.231861 s，区间 `[−0.458630, −0.005092]`。每项含 12 对、四个布局。单看共同成功，EI-ZO/NTO 的冷启动均值分别为 1.198333/1.318442 s（距离）和 1.454101/1.912732 s（时间）。完整成功/失败 p50/p90/p95 保存在 `summary/summary.json`，没有把失败延迟隐藏。

在线段是同一次真实冷启动中准备完成后的部分，不是第二次求解或常驻服务吞吐。离线三张 DRM 的构建时间独立保存在 roadmap metadata，不计入以上 query 延迟。任务采用固定执行顺序，没有额外随机交叉执行来估计缓存、系统负载或顺序效应；这些 wall time 是本机观察值。成功任务的 sampled GPU peak 为 EI-ZO 4420 MiB、NTO 188 MiB；约 0.2 s 轮询可能漏掉瞬时峰值，不能当作精确 allocator 高水位。

## 4. 成功轨迹的最差数值检查

下表每行取该方法/目标的全部六次成功，既报告真实最大值，也保留固定容差。minimum_distance 只有几何路径，速度/加速度不适用，不填零冒充物理轨迹。

| 方法 | 目标 | 最大端点误差 / rad | 最大速度控制界 / rad·s⁻¹ | 最大加速度控制界 / rad·s⁻² | 最大端点速度 / rad·s⁻¹ |
|---|---|---:|---:|---:|---:|
| EI_ZO | minimum_distance | 9.37136e-16 | 不适用 | 不适用 | 不适用 |
| NTO | minimum_distance | 0 | 不适用 | 不适用 | 不适用 |
| EI_ZO | minimum_time | 5.02399e-15 | 0.999999999909775 | 0.999999999683007 | 6.86778e-12 |
| NTO | minimum_time | 0 | 1.000000000650346 | 1.000000005902936 | 0 |

固定限值为速度 1 rad/s、加速度 1 rad/s²，可行性容差 `1e-5`，端点位置容差 `1e-5 rad`。NTO 导数界有约 `6.50e-10 rad/s` 和 `5.90e-9 rad/s²` 的实际超量，处于固定数值容差内；这里没有将它们写成严格不超过 1。两种时间轨迹报告的连续性残差均为 0。Bezier 和 B-spline 的导数控制点凸包界覆盖相应曲线段，不是仅在离散时间点量速度。

成功轨迹共保存 18,565 个数值检查样本；native collision、independent collision、checker disagreement、contact-band sample 均为 0。四个方法/目标组的最小 sampled clearance 都约为 0.019025026584333 m。距离路径的最大相邻样本关节步长为 0.004999591918 rad；EI-ZO/NTO 时间轨迹分别为 0.004349762880 / 0.004989531776 rad，均在固定 0.005 rad 上限内。碰撞是有限采样验证，不构成连续无碰撞数学证书。

本轮 CPU 从 `candidate.npz` 的原始控制点、knots、durations 独立重算导数界，从 `trajectory_samples.npz` 重算端点误差。与已封存验证报告的最大差分别为 `2.220446049250313e-16` 和 `0`；这确认了保存数组与派生检查的一致性，没有重新执行任何碰撞 checker。

## 5. 回放选择与复核命令

固定 seed-0 的十个回放现已全部实际生成到 `results/dev_paired_v2/replays/`：两组成功题 × 两目标 × 两方法，共八条真实轨迹；另有 sparse/dense 各一个 EI-ZO minimum-distance 前端失败记录，显示失败状态和原始端点。选择与代价最优或计时最快无关。另外在 `results/smoke_v3_fixed/replays/` 保存按冻结 manifest 顺序选择的 minimum-distance 和 minimum-time 各一条成功回放，分别读取 1263 和 2147 个实际验证样本。

生成在全部规划计时结束后进行，只使用 CPU Model 正向运动学，没有重新规划或查询 CUDA 碰撞。每份包含 `replay.html`、`replay.json`、`scene.png`、`joints.png` 和实际命令日志。HTML 没有外部脚本或网络资源；完整轨迹样本保留，交互显示至多 180 个确定性抽样帧，每帧关节角、时间和净距均逐值核对原数组。两个失败回放各只有原始起终点，不绘制假轨迹。12 份文件的生成命令、退出码、SHA256、数组核对及生成前后 receipt 完整性记录保存在 `reports/replay_acceptance.json`。

本代理实际使用 `view_image` 检查了五张 PNG，涵盖 EI-ZO 真实场景、NTO 时间曲线、dense 前端失败场景、smoke 距离曲线及 smoke 时间轨迹场景。完整机器人移动/固定碰撞盒、障碍和世界坐标可见；距离横轴标为 normalized path parameter，时间轨迹标为 seconds。12 份 HTML 的 JavaScript 实际通过 Node 语法检查；交互浏览器操作由主协调者另外验收，不把静态图或语法检查当成浏览器操作已经通过。

实际批量生成命令：

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.reports.generate_acceptance_replays
```

实际 CPU 审计命令（从仓库根目录运行）：

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.reports.paired_baseline_analysis
```

它重新核对 48 个任务的证据完整性、从原数组复算上述数值并输出 `reports/paired_baseline_audit.json`。其中记录 run manifest、summary、resume acceptance 和审计脚本的 SHA256。原始统计可用：

```bash
bash local/python.sh -m benchmarks.generic_motion_v1 verify --run benchmarks/generic_motion_v1/results/dev_paired_v2
bash local/python.sh -m benchmarks.generic_motion_v1 summarize --run benchmarks/generic_motion_v1/results/dev_paired_v2
```

全量 60 题 EI-ZO 开发运行应单独报告；不能把其 360 个运行与本文件的 NTO 24 个运行当作相同分母比较。正式 test manifest 的冻结和后续授权评测也不属于本报告的配对开发诊断。
