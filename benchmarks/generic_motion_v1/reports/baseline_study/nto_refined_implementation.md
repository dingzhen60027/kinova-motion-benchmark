# NTO_REFINED：固定开发政策、实现与 CPU 验收

本轮新增独立 `NTO_REFINED`，保留原 `NTO_PAPER` 的参数及封存结果。没有读取正式 test 结果来选择参数或任务，没有调用 GPU。`NTO_REFINED` 是已声明的数值加强适配，不是找回作者未公开的完整实验代码。

## 实际实现

入口为 [paper_nto_refined.py](../../runners/paper_nto_refined.py) 的 `paper_nto_refined(path, model, case, cfg, out, deadline)`。原 [paper_nto.py](../../runners/paper_nto.py) 只增加默认关闭的内部 profile、配点和上一轮候选参数；默认仍保持原目标、阶数、149 点、零裕度和原 SNOPT 数值选项。

单一政策在运行前保存为 [predeclared_plan.json](nto_refinement_cpu_v1/predeclared_plan.json)：距离 SNOPT/order 2，时间 IPOPT/order 4；原 DRM 控制点与端点复制规则不变；优化最小间隙固定 1 mm；初始仍为 `k/150, k=1..149`。每轮使用原模型 CPU 距离和现有 0.005 rad 数值采样，对间隙小于 0.5 mm 的每个当前配点区间加入最小间隙采样位置。最多四次原生求解，总配点最多600；不插入控制点，不逐题换参数。SNOPT显式 total=100000、major=2000、minor=500；IPOPT max_iter=2000；公差不放松。

所有构造、求解和 CPU 配点检查共享调用者剩余 deadline，不给下一轮重置60秒。每轮 `candidate.json/npz`、solver原始状态/变量/日志、配点、初值、CPU检查数组均保存在 `refinement/round_NN`。最后原生 `solver_success` 原样返回，失败候选不能成为成功；没有返回初始路径冒充优化完成。独立公共 validator 在函数返回后照常执行。内部检查计入 `trajectory_optimization_seconds`，不能被报成纯 solver 时间。

当前 `nto_refinement.json` 区分拟议配点与实际为下一轮添加的配点；第四轮停止时不会把尚未使用的提议当成已求解约束。CPU首轮开发版本的源码和SHA已保存于 [source_snapshot](nto_refinement_cpu_v1/source_snapshot/paper_nto_refined.py)，该元数据澄清没有改变政策或求解运算。

## 依据与实质检查

作者归档 TeX 的距离转录明确写有最小可接受距离 ε，但没有给出数值；因此1mm是公开披露的本地政策，不是论文精确参数。公开正文与归档源码均使用离散碰撞约束、PWL距离目标和DRM初值。[本地作者源](../../../../reproduction/reports/paper_completion/source_evidence/arxiv_text/sections/appendix.tex)

对三条固定 dev 原始路径实际重建 KTO：原生 `AddPathLengthCost` 与逐段欧氏长度之差均为0；149个初始碰撞约束最大值为0.0804、0.0876、0.0903（上界1），最小配点间隙分别11.96、14.96、19.03mm。说明这三条初值既传入了优化，也不是初始碰撞或转置错误。[初始转录证据](nto_refinement_cpu_v1/initial_transcription_diagnostic.json)

初始路径距离均大于影响距离，其约束梯度为0是局部未激活的预期行为。进一步在真正优化后的三个活动配点比较原生 AutoDiff 与中心差分：梯度范数101–110；`h=1e−7`附近最大绝对差约`1.01e−4`至`1.31e−4`，相对差`1.08e−6`至`1.28e−6`。没有发现配置/控制点链式梯度实现错误的证据；这不是所有状态上的梯度正确性证明。[活动约束证据](nto_refinement_cpu_v1/active_constraint_gradient.json)

Drake官方实现确认路径成本累加相邻控制点范数、碰撞约束在参数位置包装配置并传播AutoDiff：[KTO v1.51.1](https://raw.githubusercontent.com/RobotLocomotion/drake/v1.51.1/planning/trajectory_optimization/kinematic_trajectory_optimization.cc)、[距离下界约束 v1.51.1](https://raw.githubusercontent.com/RobotLocomotion/drake/v1.51.1/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.cc)。有限配点和零间隙原配置的漏碰撞已在旧dev被独立证实，因此本轮实现显式裕度与配点细化，未降低最终验证标准。

## 三条固定 dev 真实诊断

选择在执行前声明：既有迭代失败、碰撞失败及后端异常的固定dev任务。它们不是随机性能样本，不能计算为新baseline的整体成功率。均复用原保存DRM路径，未重跑前端和公共GPU终检。

| 固定dev / roadmap seed | 原状态 | 加强版实际CPU观察 |
|---|---|---|
| sparse_clutter-002-p02 / 0 | solver_failed | SNOPT成功；1轮；密集CPU最小间隙0.878mm；2.61s诊断 |
| dense_clutter-002-p00 / 2 | collision_failed | 4轮仍SNOPT不可行；149→226→346→497点；最后负间隙−67.6mm；保留solver_failed候选 |
| route_choice-002-p00 / 0 | BoxBoxGradient异常 | 相同政策下未再次触发该异常，但第三轮耗尽预算；不能称库已修复 |

详见 [逐题结果](nto_refinement_cpu_v1/results.json)。只因第一题改善，不能推断政策对整个dev有效；统一CLI完整dev比较仍需独立汇总全部失败。

首次直接CPU调用没有外部watchdog，SNOPT内部时间选项没有硬停止：第三题72.72s后抛出timeout，原日志保留。随后对同一固定题运行 [独立外部watchdog重验](nto_refinement_cpu_watchdog_v1/watchdog_result.json)：预留既有frontend耗时1.144s，worker预算58.856s，实际58.866s被终止；3轮部分输出完整保留，没有返回成功候选。正式批量必须使用已有CLI的外部worker deadline，不能只信SNOPT Time limit/IPOPT CPU limit。

## BoxBoxGradient 仍是已定位、未修复的后端限制

对既有保存dev构型和唯一 `robotiq::left_inner_finger_collision` / `table::Box` 几何对，重新使用纯double官方接口，未调用AutoDiff或求解器：

| 官方接口 | 实际返回 |
|---|---|
| ComputeSignedDistancePairClosestPoints | BoxBoxGradient断言，进程exit1，无距离返回 |
| ComputeSignedDistancePairwiseClosestPoints | 同断言，进程exit1 |
| ComputePointPairPenetration | 正常返回0对，无该对的有符号间隙或导数 |

[隔离子进程比较及日志](nto_refinement_cpu_v1/boxbox_official_apis/comparison.json)。因此仅用数值差分替换梯度无法修好中心值查询，penetration接口也不是等价距离替代。Drake [v1.51.1](https://raw.githubusercontent.com/RobotLocomotion/drake/v1.51.1/geometry/proximity/distance_to_shape_touching.cc) 与本轮读取的 [master](https://raw.githubusercontent.com/RobotLocomotion/drake/master/geometry/proximity/distance_to_shape_touching.cc) 都仍保留对应1e−14 witness一致性断言；没有证据支持直接升级即可修复。本轮未改第三方库、几何或过滤对，不能宣称这一backend bug已解决。

第一条直接脚本启动因Python模块路径失败的日志也保留；后续使用`-m`真实复现成功。初次把多个官方查询放进一个进程时被断言提前终止，随后改为各自独立子进程，不将未运行API写成已验证。

## 测试、命令与运行边界

[最终CPU单测](nto_refinement_tests_final.log)：22项通过，包括原17项、两个原生求解器、真实失败候选保存、过期预算、配点区间和600点上限。早期一个仅匹配日志空白的测试失败已改为正则；没有改变求解参数来通过测试。

```bash
bash local/python.sh -m pytest benchmarks/generic_motion_v1/tests/test_paper_nto.py benchmarks/generic_motion_v1/tests/test_paper_nto_refined.py -q
bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_study.nto_refinement_cpu_probe --output benchmarks/generic_motion_v1/reports/baseline_study/nto_refinement_cpu_v1
bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_study.nto_refinement_watchdog_probe --output benchmarks/generic_motion_v1/reports/baseline_study/nto_refinement_cpu_watchdog_v1
bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_study.nto_boxbox_api_probe --output benchmarks/generic_motion_v1/reports/baseline_study/nto_refinement_cpu_v1/boxbox_official_apis
```

CPU probe已执行的版本包含直接脚本启动失误，其源码快照保留；当前helper已改为正确模块启动。重跑必须使用新输出目录，不覆盖本次证据。完整批量由主任务统一串行调度GPU，保留60秒外部超时、失败分母、共同初值、同一模型和最终独立验证。本报告没有把内部CPU检查通过标为正式benchmark成功，也没有把有限采样宣称为连续安全证明。
