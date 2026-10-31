# NTO baseline 的实现、失败与正式评测准备度复查

审计日期：2026-10-28。被审计运行是 `results/rss2025_full_dev_v1`，规划源码固定在 `72984027030175b7bca78e2dcf592b38f0c1864b`。本轮只读已封存任务；另外在现有 Conda 中运行独立 CPU 数值小程序诊断 SNOPT 的日志和迭代选项。没有运行机械臂规划、碰撞查询、CUDA 或正式 test，也没有修改 runtime、配置、历史结果或第三方依赖。

当前结论是：**现有 NTO_PAPER 可以作为已审计、明确声明差异的论文配置重建结果保存，但不能凭距离目标的 0/75 阻塞任务成功就认定 NTO 普遍无效，也不能将它单独称作已经充分加强的 NTO 基线。** 没有发现初值未传入、控制点转置、端点被换、时间缩放或目标写错的证据。已经实际定位到一个可修复的 SNOPT 日志选项问题；另有一个 Drake 几何导数 backend 异常，以及未公开的作者参数、149/150 配点歧义和有限配点的实际漏碰撞，必须分别处理和表述。

## 1. 实际分母和失败分解

60 个固定 dev 题来自 20 个场景，每题使用 3 张 6000 节点 DRM；每个目标的 180 次查询不是 180 个独立环境。47 个题的原始直连有数值碰撞，13 个题未发现直连碰撞，对应下表的 141 和 39 次查询。这里的 blocked 仅指固定起终点直连检查的标签，不能解释为已经证明的 C-space 拓扑类别。

| 目标 / 直连标签 | 查询数 | 前端失败 | 前端基础设施错误 | 通过共同 DRM 检查 | 最终成功 | 碰撞失败 | 求解失败 | 超时 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 距离 / blocked | 141 | 65 | 1 | 75 | 0 | 45 | 30 | 0 |
| 距离 / 未发现直连碰撞 | 39 | 4 | 0 | 35 | 35 | 0 | 0 | 0 |
| 时间 / blocked | 141 | 65 | 1 | 75 | 53 | 18 | 1 | 3 |
| 时间 / 未发现直连碰撞 | 39 | 4 | 0 | 35 | 35 | 0 | 0 | 0 |

距离成功 35/180，DRM 准入后 35/110；时间成功 88/180，准入后 88/110。距离的 30 个 solver_failed 和时间的 1 个合计 31，不能把 31 全写成距离失败。两目标共享相同固定输入与已接受 DRM 路径；时间目标在同一批 75 个 blocked 准入查询上成功 53 次，因此不存在“所有阻塞题都是整体无解”这一解释。两次 infrastructure_error 都发生在 DRM 阶段的 native process abort，不能归入 NTO 优化器失败。

本轮 [artifact audit](nto_readiness_artifact_audit.json) 重新核对了 360 个 NTO 记录和 1339 个实际读取文件的 receipt 哈希。已有 [独立 NTO 审计](../full_baseline/dev_nto_audit.json) 核对全部 900 receipts、27280 个封存文件及共享路径，并用 SciPy 独立重建 216 条有限候选曲线；最大样本差为 `2.6645352591003757e-15 rad`，没有重建一致性错误。它复用了已有碰撞数组，并不构成新碰撞查询或连续安全证明。

## 2. 论文规定、当前实现和未恢复的细节

论文 VII-A 使用 DRM 初值、Drake KinematicTrajectoryOptimization 与逐点碰撞约束；距离求 PWL 长度，时间使用四阶 B-spline、静止端点及速度/加速度界。Appendix D 给出的 Franka 配置为距离 SNOPT、时间 IPOPT、两者 Nc=150，并说明作者分别调整了不同环境和目标的 NTO 参数。[正式论文](https://www.roboticsproceedings.org/rss21/p045.pdf)；本地官方 PDF 的既有核查见 [source audit](../paper_algorithm/source_audit.md)。

| 项目 | 当前实际实现 | 复现判断 |
|---|---|---|
| 距离优化器与表示 | SNOPT；order=2、degree=1；原 DRM 节点就是控制点 | 与公开的 PWL 目标和求解器选择一致 |
| 时间优化器与表示 | IPOPT；order=4、degree=3；保留 DRM 节点并各复制一次首尾控制点 | 阶数/求解器一致；端点复制是声明的重建选择 |
| 初值 | KTO(initial)；距离不重采样；时间未插入额外 knots | 已核对构造器确实设置 initial guess |
| 碰撞配点 | `k/150, k=1,...,149`，实际 149 个约束 | 来自作者 TeX 的注释段；正文说 Nc 个点，精确作者约定未恢复 |
| 距离目标 | 相邻控制点欧氏范数之和；duration 固定 1 | 对一次 B-spline 恰为 PWL 长度，不是误用平滑样条的控制多边形近似 |
| 几何/过滤 | 固定 Kinova 模型；137 个基础 pair，另有 15×voxel_count 个在线 pair；同一碰撞过滤 | 方法间一致；不是原 FR3 的 33 球模型 |
| clearance / influence | 0 m / 0.01 m | 本地重建参数；不能冒称作者未发布的准确数值 |
| 可行性/最优性容差 | SNOPT major feasibility/optimality 均 1e-6；IPOPT tol=1e-6 | 本地声明值；相同数字不表示不同求解器采用相同误差度量 |
| 迭代与时间 | SNOPT major=2000；IPOPT max_iter=2000；共同外层 60 s | 本地协议，论文未发布相同限额；另见下一节实际 SNOPT 默认累计限额 |
| 时间边界与限幅 | duration∈[0.05,100]；初值 length(DRM)+2；v/a 均 ±1 | 本地明确选择；±1 出现在论文硬件描述，不能当成恢复出的 FR3 仿真数值 |

当前 KTO 源码的 trajectory 构造器调用 SetInitialGuess；重建时将归一化 knots 乘 duration，和所保存的实现相同；AddPathLengthCost 逐项求相邻控制点距离。这些行为已对照 [Drake 1.51.1 官方源码](https://raw.githubusercontent.com/RobotLocomotion/drake/v1.51.1/planning/trajectory_optimization/kinematic_trajectory_optimization.cc)。输入模型与几何差异、缺少原作者 NTO 可执行源码等边界仍然存在，不会因这些适配器检查通过而消失。

149 与 150 必须在正式版本中写成“参数 Nc=150，实际149个内部配点”，不能只显示“150点”。作者注释稿不是发布方法的更高优先级规范。若另建严格150个内部点的版本，应冻结具体位置与新版本名，作为声明的转录差异检验保存；不能覆盖历史149点结果。目前没有实测证据说明只增加一个点能解决 0/75，不能凭猜测作这种归因。

## 3. 已经实际查清的 SNOPT 日志与迭代行为

本机运行环境为 Drake 1.51.1，实际 SNOPT 日志版本头是 **7.4-1.2 (Feb 2015)**。原距离任务的 snopt.log 只有 121 字节头，尽管已设置 Major print level=1。本轮对同一现有 solver 实际运行了 17 个 CPU 小程序，完整源快照、原始数组、参数、stdout/stderr 和日志保存在 [snopt_option_probes_v3](snopt_option_probes_v3/summary.json)。这些是数值软件诊断，不是额外 benchmark 分数。

实测发现：使用 solver-specific 字符串 `Print file` 时只有文件头；改用 `SolverOptions.SetOption(CommonSolverOption.kPrintFileName, path)`，同时移除 solver-specific `Print file`，就获得完整参数列表和迭代历史。提高 Major/Minor print level、设置 Print level 或 common console 都没有解决原文件日志问题。仅设 `Summary file=6` 可将迭代摘要输出到已捕获的 stdout。

同版本源码支持这一原因：SNOPT adapter 先将 common print filename 转写为 Print file，再 Pop 指定的 direct 选项；SpecificOptions 对已 Pop direct 选项仍可继续应用残留的 common 空值。使用唯一的 common 文件选项避开这条路径。[SNOPT adapter](https://raw.githubusercontent.com/RobotLocomotion/drake/v1.51.1/solvers/snopt_solver.cc)，[SpecificOptions](https://raw.githubusercontent.com/RobotLocomotion/drake/v1.51.1/solvers/specific_options.cc)。两份官方源快照也保存在本目录；这不是修改 solver 的数值算法。

**日志修复的配对数值证据：**

| CPU 问题 | 旧 direct 文件入口 + stdout 摘要 | 新 common 文件入口 | 原始数组比较 |
|---|---|---|---|
| 两变量 Rosenbrock | info=1，61 total / 34 major | info=1，61 total / 34 major | x、F、Fmul、xmul 全部逐元素完全相等；objective 相等 |
| 164 行问题：同一目标 +149 非线性约束 +14 固定坐标线性等式 | info=1，198 total /184 major | info=1，198 total /184 major | 上述四类数组完全相等；objective 相等 |

原始不带 stdout 摘要的 paper options 和 common 文件版本也得到完全相同数组和 objective；原日志没有迭代数，因此该一对不能凭空补出旧迭代记录。详见 [paired_logging_equivalence.json](snopt_option_probes_v3/paired_logging_equivalence.json)。日志写入会增加 I/O，接近外层时间边界时不能保证端到端耗时或最终状态逐任务不变；旧分数继续保留。

完整日志实际显示：**Major iterations limit=2000、累计 Iteration limit=10000、Minor iterations limit=500**。164 行诊断问题也得到相同值，匹配实际距离 NTO 的 `solver_F` 行数。额外实测把累计限制设为1时，得到 info31、1 total/0 major；把 major 设为1、累计限制设为1000000时，得到 info32、4 total/1 major。官方文档也区分累计 minor、单次 QP minor 和 SQP major 三种限制。[SNOPT 选项说明](https://ccom.ucsd.edu/~optimizers/docs/snopt/options.html)

因此，现有 dev 的10个 info31 是累计迭代限制退出，6个 info32 是 major 限制退出，不能统称“2000 major 用尽”。前者实际 solver wall time 为2.64–45.08 s，后者为28.44–51.48 s，都不等于外层60秒超时。根据本机默认值、164行结构及原选项未覆写累计限额，可以归因到隐藏的默认累计限额；但原始每次确切 major/minor 计数已不可恢复，不能伪造历史10000次迭代日志。

本轮建议**只修日志入口，并把已经实测的有效默认累计10000、minor500和显式major2000写入元数据**。维持原参数冻结；不能因看到弱结果就静默增加预算。若另做 adapted 预算敏感性，需显式分别指定 `Major iterations limit` 和 `Iterations limit`；例如2000与1000000是已在 CPU probe 验证可接受的选项值，仍非作者公布参数、也没有证明能提高机械臂成功率。该变化必须单列，不合并进原 paper-profile 结果。

具体日志修复建议如下，仅为已验证补丁写法，本报告未修改 runtime：

```python
from pydrake.solvers import CommonSolverOption, SolverOptions
solver_options = SolverOptions()
solver_options.SetOption(CommonSolverOption.kPrintFileName, str(snopt_log_path))
# 原来的数值选项保持不变；不要再向 prog 设置 solver-specific "Print file"。
result = solver.Solve(prog, None, solver_options)
```

本机 `MathematicalProgram.SetSolverOption(CommonSolverOption, path)` **不支持**这个两参数重载，实际 TypeError 已保存在 [API probe](common_option_api_probe.log)；必须使用上面的 SolverOptions 入口，不能把未经测试的一行替换当作修复。

## 4. 碰撞失败不等于日志缺失或“控制点太少导致无解”

110 次准入查询的原始 DRM 节点分布是：2点32次、3点40次、4点20次、5点11次、6点4次、8点3次。距离使用这2–8个控制点；时间使用4–10个控制点。距离初值精确表示原始 DRM PWL，而该 PWL 已通过共同的有限分辨率检查。因此“少节点所以没有任何可行曲线”不是由当前证据支持的解释；初值本身就是一个通过同一数值检查的候选。它不是连续无碰撞的数学证明。较少控制点会限制优化自由度，不能自动判定为实现错误；增加节点则改变基线参数化，需要另列适配版本。

距离的45个 collision_failed 都有 Drake success 返回：35个 SNOPT info1，10个 info3；最终都被几何验证拒绝，没有把 info3 或碰撞候选计作成功。45条曲线的最小采样净距为 `-0.0140122226` 至 `-9.53855584e-6 m`，中位数 `-0.000304086081 m`。全部都超出2微米接触歧义带，不能靠放宽舍入容差“修好”。与此同时，保存的149个 SNOPT 碰撞约束最大超量最高仅 `7.06386505e-8`，低于配置1e-6；这个聚合约束数值不是以米计的净距。

已保存的具体反例 `dev-sparse_clutter-003-p00-NTO_PAPER-minimum_time-cd9f390e`：IPOPT成功，100次迭代，约束违背记录为0；8个碰撞样本全在 `[54/150,55/150]` 内，最小净距 `-0.000256373230 m`。它直接显示有限配点之间可以碰撞，详情见 [原始失败诊断](../full_baseline/nto_failure_diagnosis.md)。这解释了部分失败机制，不证明所有失败有同一原因，更不证明作者原FR3实验也会得到相同成功率。

31个 solver_failed 的原始分布为：SNOPT13×7、31×10、32×6、41×5、43×1、backend exception×1；IPOPT迭代限制×1。4条失败求解器候选虽然通过数值几何验证，仍按预先规定保持 solver_failed；不能事后改成成功。全量候选只有7个 checker 判定分歧，均在2微米接触带内且属于失败曲线，不会把这里45条距离碰撞失败改成成功。[全量 backend 证据](../full_baseline/nto_backend_failures.json)

## 5. 必须单独处理的 backend 异常

`dev-route_choice-002-p00-NTO_PAPER-minimum_distance-48b608ce` 的输入路径通过共同检查，但SNOPT调用6.399 s后在Drake `BoxBoxGradient` 的 touching-witness 一致性断言失败，没有返回优化候选。错误涉及两点世界坐标之差小于内部 epsilon 的条件，而不是任务不可行证明。[保存的原始异常和 traceback](../full_baseline/nto_backend_failures.md)，[Drake对应源码](https://raw.githubusercontent.com/RobotLocomotion/drake/v1.51.1/geometry/proximity/distance_to_shape_touching.cc)

这属于几何导数 backend 的执行失败，必须在方法汇总中另列注释；保留原 run status=solver_failed。正式比较前应捕获该失败的优化迭代q和具体geometry pair，建立CPU最小复现并判断官方数值修复是否适用。原记录没有保存失败的迭代点，当前证据不足以安全给出修复代码。不得通过改变机器人碰撞形状、过滤对、球半径、端点或移除该题来回避错误。若暂不能修复，必须明确称为已知backend不支持/失败，而不能纳入“NTO算法找不到路径”的因果结论。

## 6. 是否增加已有 NTO 适配作为额外基线

**值得加入同一固定 dev 的配对诊断，但目前没有实证足以称它已经更强。** 现有 [csdecomp_baseline.py:_nto](../../runners/csdecomp_baseline.py) 与 NTO_PAPER 的差别包括距离也用IPOPT、150个内部点位于k/151、clearance=1e-5 m、max_iter=500，以及仅时间目标通过保形插结点补至至少16个控制点。代码中的minimum16 **不作用于距离目标**；不能宣称它已经给弱距离基线增加了自由度。

其旧实测 [dev_paired_v2审计](../paired_baseline_audit.md) 只有四个预先固定题、三张图、两目标，各方法/目标成功6/12。成功全部来自两个可数值直连场景；另外两个blocked场景全部前端失败。该历史证据验证了接口、求解和重放，却没有验证其blocked绕障能力。它与本轮180次查询的分母和DRM profile不同，不能直接用旧50%与新35/180比较。

若把它加入后续研究标准基线，应使用同一冻结60题、同图、同端点、相同初始DRM path、相同60秒和最终数值validator，并独立命名 adapted_NTO；先补齐它在solver失败时的原始状态、variables/controls/knots保存，达到NTO_PAPER相同的失败可审计性。保留两种结果，不能每题择优拼接。开发运行之前冻结这组既有参数和统计口径；不要从test挑题、调参或删除不利结果。增加这个现有适配不需要发明新算法，但能检验结论是否过分依赖一个未充分恢复的solver/transcription选择。

## 7. 正式评测前的处理边界

必须完成的工程/表述工作：修复并验收SNOPT日志；保存实际有效迭代限额、版本、退出码和日志；149/150语义在方法名和表中写清；backend断言获得最小复现或明确作为未解决实现限制；所有方法使用同一成功/失败定义、输入和终检。60秒是端到端预算，现有求解器实际剩余预算距离为57.886–59.232 s、时间为57.877–59.231 s，不能写成每个优化器独享60秒。冷启动、准备后query段、纯solver调用和离线建图应分别报告，不能与论文不同时间边界的数字直接比速度。

应如实保留的结果包括：有限配点漏碰撞、非凸局部求解器不能保证收敛、已声明迭代限额退出、DRM前端失败、固定Kinova替代FR3、未公开的原参数以及有限采样验证的局限。它们不应通过改变端点、放松终检、返回初始路径冒充优化成功、增加未声明控制点或静默换solver来消除。现阶段还没有证据要求修改NTO的目标、导数约束或重建公式。

## 8. 本轮实际命令和证据

以下命令均已在仓库根目录执行，沿用 `local/python.sh` 指向的现有 Conda 环境：

```bash
bash local/python.sh benchmarks/generic_motion_v1/reports/baseline_study/nto_readiness_artifact_audit.py --run benchmarks/generic_motion_v1/results/rss2025_full_dev_v1 --output benchmarks/generic_motion_v1/reports/baseline_study/nto_readiness_artifact_audit.json > benchmarks/generic_motion_v1/reports/baseline_study/nto_readiness_artifact_audit.log 2>&1
bash local/python.sh benchmarks/generic_motion_v1/reports/baseline_study/snopt_option_probe.py --output benchmarks/generic_motion_v1/reports/baseline_study/snopt_option_probes_v3 > benchmarks/generic_motion_v1/reports/baseline_study/snopt_option_probe_v3.log 2>&1
```

第一条退出0，读取的1339个封存文件哈希一致。第二条17个独立CPU子进程全部正常退出，预先故意设置低迭代上限的失败保留原状态；输出目录内包含所执行脚本的完整快照。配对数组核对用NumPy直接比较raw.npz的x/F/Fmul/xmul，并从原生文本提取total/major计数，保存为paired_logging_equivalence.json。早期选项探针v1/v2和不支持的API调用日志也保留，未掩盖失败探索。

原始论文源码使用本地已归档资料；本轮还只读访问了官方Drake 1.51.1源码与SNOPT文档。无法恢复的作者参数明确标注未恢复。未使用任何正式test优化结果，也未声称这些CPU诊断已修复机械臂任务或提高其成功率。
