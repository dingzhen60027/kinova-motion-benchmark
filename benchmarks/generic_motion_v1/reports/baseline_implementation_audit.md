# 基线实现审计

本报告审计实现提交 `a98f44db533c64f58cf0965d2aebbf610426a170`，只说明已核查的调用、约束、适配与证据，不代替尚在执行的开发实验汇总。采用现有 Conda 环境，未重装依赖、改写上游 CUDA 或修改历史结果。本套件是 **RSS-inspired Kinova-7D adaptation**，不是论文 FR3 实验的精确复现。

源码、安装模块 SHA-256、发行版本和来源见 [baseline_implementation_sources.json](baseline_implementation_sources.json)。本地 `origin/main` 引用为 `7661a97bb7c4931be73d2bc4f8c45c3094ff476e`；该引用本身不证明已安装 wheel 由此提交构建。实际运行的 CSDecomp 扩展、Drake 动态库等二进制散列另存于 [environment.json](environment.json)。

## 原生调用路线

| 阶段 | 实际调用与实现位置 | 约定 |
|---|---|---|
| 离线建图 | [`RoadmapBuilder`](../roadmap.py#L106)：`add_nodes_manual → build_roadmap → build_pose_map → build_collision_map → write` | 无新增场景障碍；独立 PCG64 种子；保存全部随机提案及接收节点；此时间单列 |
| 在线公共前端 | [`DrmPlanner`](../runners/csdecomp_baseline.py#L312)：`LoadRoadmap → BuildCollisionSet → Plan` | 同一 frozen roadmap、模型和 manifest 端点；启用原生 shortcut；记录实际返回路径 |
| 区域计算 | [`CudaEdgeInflator.inflateEdge`](../runners/csdecomp_baseline.py#L344) | 原生 EI-ZO；显式传入 method seed；已有区域可复用；按路径边保持有序区域序列 |
| 最短距离 | [`scsplanning.polygonal.get_points`](../runners/csdecomp_baseline.py#L369) | 直接调用作者实现，优化有序凸集走廊中的 PWL 欧氏长度 |
| 最短时间 | [`scsplanning.biconvex`](../runners/csdecomp_baseline.py#L376) | 直接调用作者交替优化，六次 Bezier，容差 0.01，七轴速度/加速度 1/1，端点静止 |
| 碰撞恢复 | [`EditRegionsCuda`](../runners/csdecomp_baseline.py#L394) | 使用独立终检发现的碰撞见证；最多每轮 128 个见证、20 次恢复；固定种子边不变 |
| 最终验证 | [`validate`](../runners/validate.py#L107) | 两个碰撞 checker、端点、关节限位、导数包络与连接连续性共同决定接受 |

包装层没有另写 LSCS/SCSTO 优化问题。`examples/planning_utils.py` 的恢复包装曾把边终点也写为 `e[0]`；本套件直接传真实的 `path[:-1]` 和 `path[1:]` 给现有 `EditRegionsCuda`。旧示例文件没有被修改。

EI-ZO 参数来自冻结 [evaluation.json](../configs/evaluation.json)：10,000 particles、9 次二分、configuration margin 0.1、delta/epsilon 0.01、tau 0.5、mixing 100、每轮最多 10 个超平面。求解的随机性来自显式 `EizoOptions.seed`；确定性的优化初始化不冒称另一个有效随机种子。不同 roadmap 使用不同保存的 PCG64 节点。

## 同一机器人与碰撞模型

离线构造器和所有在线方法都实例化 [`Model`](../model.py#L42)，冻结全模型 `kinova_sens_on_table.yml`：七个活动关节、20 个碰撞盒、15 个运动机器人几何体、137 个基础检查对。完整检查对、排除对、原语尺寸、变换与来源散列保存在 [G0 审计](model_audit/audit.json)。运行前同时比较 roadmap、manifest 和当前模型的两个身份：

- `robot_sha256 = a25f447c8caf4aff6dc76196c3d57cc940f00e802367f48034f06a9973927c75`
- `collision_geometry_sha256 = 896ab9f55c5d8081b843358bd85cde2daae1f432564593f86218bf61733ce4fa`

上游离线/在线示例确实引用不同 URDF 文件，但审计没有发现其原语尺寸、关节限位或检查对的物理差异。reduced 版折叠固定链，20 links 对比 full 的 29 links；64 个构型中的世界变换/TCP 最大差异约 `3.58e-7`，处于 float32 精度范围。不能仅凭文件名不同宣称碰撞模型不同。新套件仍统一采用 full 构造，拒绝导入身份不匹配的旧地图。

G0 实际检查了基础模型随机 512 点、边上 528 点，以及相同两组构型加在线球后的 512/528 点，另有 66 个接触边界探针；全部分类差异为零。这是 2,146 次构型/环境检查，不是 2,146 个独立随机场景，也不是连续碰撞证明。

NTO 通过 [`nto_plant`](../model.py#L159) 把完全相同的在线 0.04 m 球登记到 Drake SceneGraph。新增碰撞对严格为 `15 × 球数`，逐个几何名称核对身份，不只比较数量。已有真实 CPU 测试显示两球对应 167 对，八个构型的最小距离与独立解析 checker 相差小于 `1e-10 m`。这一步计入 NTO 在线构造时间。

## Clarabel 来源与适配边界

实际安装的 `scsplanning` 发行版本为 **0.1.0**，由本地 vendor 源码安装；vendor HEAD 是作者提交 `d3f885aa2b74a10418de96e61ec2df0bf6ef16b8`。本次核查确认 vendor 工作树干净，安装的 `polygonal.py`、`base_program.py`、`biconvex.py`、`fixed_points.py`、`fixed_velocities.py` 与 vendor 文件逐字节相同。该作者提交本身使用 `ClarabelSolver`，不是本次在本地把作者库改成 Clarabel。

“Clarabel 适配”是相对论文 LSCS 所报告的 MOSEK 配置而言：这里调用 **Drake 1.51.1 的 `ClarabelSolver`**，不能宣称复现原论文 MOSEK 的性能。环境里独立 Python `clarabel` 包为 0.11.1，但它不是这里直接调用的入口，不能用此版本号代替 Drake 所嵌入的后端版本；后者未从当前 Python 接口单独确认。

`pybezier` 的发行元数据版本为 **0.1.4**，但模块内部陈旧的 `__version__` 为 0.1.1，两者均如实记录。每个合法任务的 [`dependency_audit`](../runners/csdecomp_baseline.py#L117) 会用实际 Clarabel 解一个小 LP，保存 available/enabled、求解结果和模块散列。官方 LSCS 与 SCSTO 的真实 CPU 求解另有回归测试；并非只检查包能否 import。NTO 使用现有 Drake `IpoptSolver`，无商业许可证替代逻辑。

## float32 区域交接修复

原生 DRM 返回 float32 路径，manifest 端点保持原始 float64 数值。在两条真实失败记录中，原生种子边全部处于对应区域内，相邻区域也有共同路径顶点，但原始固定起点分别超出区域约 `3.80197e-8` 与 `3.40016e-8`，使严格固定端点的凸优化被判不可行。这是包装数值错误，不应解释为算法自然失败。

[`cover_region_anchors`](../runners/csdecomp_baseline.py#L64) 保持所有法向量 A、机器人/障碍物和端点不变，仅计算逐面非负补偿：

`delta_i = max(0, max_anchor(A_i q - b_i))`，`b'_i = b_i + delta_i`。

固定阈值要求每一面 `delta_i / ||A_i||₂ ≤ 1e-6 rad`；零法向量且不可行的约束拒绝接受。超过阈值立即失败，不重新抽端点、不放大碰撞容差。对应 native path 两端总是 anchors，首/尾区域还包含不变的 manifest 起/终点。新膨胀、复用、编辑恢复、重膨胀都走同一个 helper。每次保存 `coverage/.../before.npz`、`after.npz` 和 `coverage.json`，包括 A、b、anchors、每面补偿和最大归一化补偿。区域略微扩张后仍必须通过原有独立碰撞终检。

[实际诊断](native_solver_failure_diagnosis.md)及[机器证据](native_solver_failure_diagnosis.json)保留原始失败和散列。两例在只修复区域偏移、保持 manifest 端点不变后，实际官方 LSCS/SCSTO CPU 求解成功；[live helper 对照](hardening_proposal/live_helper_diagnostic.json)也确认原 sealed artifacts 未变化。它们是故障诊断，不替换完整 GPU 重跑的实验分数。修复前开发结果必须作为旧实现故障记录单列。

## 公共 NTO 的目标、约束和初始化

[`_nto`](../runners/csdecomp_baseline.py#L138) 是明确声明的 Drake KTO/Ipopt 转录，不冒充作者未公开的 NTO 实验代码。

| 项目 | 最短距离 | 最短时间 |
|---|---|---|
| 曲线 | order-two B-spline，即 PWL | cubic B-spline |
| 目标 | `AddPathLengthCost`；此阶数下为折线欧氏弧长 | `AddDurationCost` |
| 初始化 | 原生 DRM 路径节点保持原有顺序 | 同一 DRM 路径，加首尾重复控制点；保形插结至至少 16 个控制点 |
| 端点 | manifest 原始七维起终点的等式约束 | 相同位置等式，另加零端点速度 |
| 运动约束 | 同一关节限位；参数时长固定 1，不解释为物理运动时间 | 同一关节限位、逐轴 1 rad/s 与 1 rad/s²；T 在 [0.05,100] s |
| 碰撞约束 | 150 个均匀内部参数位置的原生 Drake 最小距离约束 | 同左 |

NTO collision clearance 为 `1e-5 m`、influence distance 为 0.01 m；Ipopt 最多 500 次迭代、tol 为 `1e-6`。该优化余量与终检的两微米接触不确定带分别报告，机器人碰撞球半径没有改变。优化器成功与数值验证通过必须同时成立，失败但有限的迭代结果不算成功。

这是一个固定转录的公共 NTO 基线，未加入此前其他研究平台中的多轮自适应 NTO，也不证明已经穷尽最强 NTO 设置。所有输入的运动约束在 [`_assert_motion_limits`](../runners/csdecomp_baseline.py#L208) 中与 manifest 精确核对，不能靠配置覆盖偷偷更换任务。EI-ZO 与 NTO 各自调用同一确定性 DRM 前端；“相同初始化”的最终实证应来自对应任务、roadmap 的 `path.npy` 数值比较，不只依赖相同随机种子。

## 预算、验证和不能声称的比较

[`batch.execute`](../batch.py#L126) 为所有方法提供同一个 **60 秒外层 wall-clock deadline**，包含新进程、imports、模型、GPU context、地图加载、在线求解、恢复、验证及输出。超时时发出终止信号，最多再等两秒后 kill；真实进程耗时保留，不把回收开销裁成恰好 60 秒。NTO 内部 CPU 限时取 `min(60, 当前剩余在线预算)`，原生 CUDA/作者优化器不能在 Python 内安全抢占，依靠外层进程监管。超时不会因已有候选看似可行而被升级为成功。

`offline_seconds` 是单列的 DRM 建图费用。`warm_query_seconds` 是同一次真实执行中模型、GPU 和地图准备完成之后的在线阶段；它不是额外测量的第二次求解，也不是常驻服务吞吐量。原生缓存、模型加载和 GPU 初始化没有从 cold E2E 中抹除。

统一终检的关节采样间距目标为 0.005 rad，曲线使用导数控制包络确定采样密度、保留分段边界和 knot；最多 100,000 点，超限记为未决而不放粗步长。速度/加速度的控制包络覆盖完整曲线段，但碰撞仍是有限采样。最小距离是 **sampled minimum clearance**。接触带 `|distance| ≤ 2e-6 m` 不记为稳定成功；所有方法使用相同标准。

以下限制保留在结果解释中：

- Python `DrmPlanner.Plan` 没有暴露 shortcut 前内部原始路径。保存的 `path.npy` 是配置 shortcut 后实际返回路径，`path_provenance.json` 明确记为 `not_exposed`，没有复制它伪装成两种路径。
- 原生接口没有可用的完整碰撞查询计数器。本次只能声明共同时间预算，**不能声称已经完成等碰撞查询预算比较**；同样，20 次恢复上限不是相同新增查询量。
- 当前区域数是有序走廊中区域条目的数量，可能包含复用的同一几何区域；不能直接当成不同 EI-ZO 膨胀调用次数或唯一新区域数量。
- GPU 显存是约 0.2 秒间隔 `nvidia-smi` 进程采样峰值，不是精确 allocator high-water mark。
- NTO 的有限 collocation、EI-ZO 的随机区域估计、最终有限采样都不提供全局可行性证明；失败不证明原问题无解。
- 若 NTO 只执行预先固定的开发子集，其方法比较必须使用对应 EI-ZO 配对子集，不得把子集成功率与 EI-ZO 全开发集成功率直接排名。

## 实际测试证据与命令

[完整测试日志](all_tests_after_hardening.log)记录 **88 passed，87 warnings，1.68 s**。其中，[基线输入与覆盖修复专项](hardening_proposal/applied_cpu_tests_v3.log)为 **50 passed**，包含真实官方优化求解、模型外的薄碰撞测试、PWL 精确代价、曲线导数/静止端点、输入形状和限幅一致性、行缩放不变的补偿阈值，以及过大补偿拒绝。日志中的警告不应写成“无警告通过”。此前开发失败、测试构造失败和修复过程日志也保留。

下面是已经实际执行过的 CPU 命令，均在仓库根目录使用现有 Conda Python：

```bash
bash local/python.sh -m pytest -q benchmarks/generic_motion_v1/tests
bash local/python.sh -m pytest -q benchmarks/generic_motion_v1/tests/test_baseline_input_contract.py benchmarks/generic_motion_v1/tests/test_baseline_region_coverage.py benchmarks/generic_motion_v1/tests/test_baseline_validation.py
bash local/python.sh -m benchmarks.generic_motion_v1.reports.hardening_proposal.diagnose_native_solver_failure
```

实际模型/GPU 交叉审计、每个规划 worker 的命令、原始日志与返回码分别由 `model_audit/commands.json`、run manifest、`process.json` 和 `receipt.json` 保存。本报告编写时没有启动新的 GPU 运行；最终开发成功/失败统计以修复后的冻结运行及总审计报告为准。
