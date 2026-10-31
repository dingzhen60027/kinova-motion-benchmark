# NTO 数学转录与时间代价差异诊断

本核查只读既有论文、代码及冻结结果；没有改写主实现、主案例或既有报告。新增 CPU 探针已实际运行。对 Forest 距离法的完整碰撞采样诊断由同目录其他记录负责，本文件主要给出时间法的已确认机制与源码一致性结论。

## 核查对象与一手证据

- 论文正式 PDF：`reproduction/paper/2504.10783v1.pdf`，SHA-256 `579c9917425fe01b583f067d0a1c9702917ccd810584dbac562fcdf9fead51d0`。§VII-A 明确使用 Drake KinematicTrajectoryOptimization；距离法用 DRM PWL 初值，时间法以 DRM 节点初始化四阶 B-spline。附录 D 给出 Forest 距离/时间 Nc=20/100、FR3 两类 Nc=150 及求解器选择。
- 作者上传 TeX：[experiments.tex](../paper_completion/source_evidence/arxiv_text/sections/experiments.tex)、[appendix.tex](../paper_completion/source_evidence/arxiv_text/sections/appendix.tex)。注释草稿写的是 `k/Nc, k=1,...,Nc-1`，而正式 PDF 只说 Nc 个点；当前 `k/(Nc+1)` 是已声明的构造选择，不能认定作者就是这个位置集合。时间法详细转录在注释草稿仍是 TODO。
- 主实现：[optimization.py](../../paper_constructed/optimization.py)、[model.py](../../paper_constructed/model.py)、[forest.py](../../paper_constructed/forest.py)、[冻结配置](../../paper_constructed/config.json)。
- Drake 官方 [KTO 文档](https://drake.mit.edu/doxygen_cxx/classdrake_1_1planning_1_1trajectory__optimization_1_1_kinematic_trajectory_optimization.html) 和 [v1.51.1 源码](https://github.com/RobotLocomotion/drake/blob/v1.51.1/planning/trajectory_optimization/kinematic_trajectory_optimization.cc)。对应版本源码已保存于 [nto_source_evidence](nto_source_evidence/source.json)，SHA-256 `8228a28922640f7d04d2d51b01f6be682afe258b5513f1e84cd0f37f11900370`。

## 排除的 API / 单位错误

1. `KinematicTrajectoryOptimization(initial)` 确实把传入轨迹作为初值；不是只读取控制点数量却漏设初值。
2. 四阶 B-spline 是三次多项式，与论文“4th order”一致。距离法二阶 spline 是 PWL；该情况下 AddPathLengthCost 的控制多边形长度正好等于路径长度。
3. KTO 用归一化路径 r(s) 和单独时长 T。AddVelocityBounds / AddAccelerationBounds 内部分别按 T 和 T² 缩放，现有代码没有漏除或多除 T。
4. 端点 `AddPathVelocityConstraint(0,0,s)` 是归一化路径导数，但在正时长下与物理端点速度为零等价。当前 T 下界 0.05，未发现这里的单位错误。
5. `MinimumDistanceLowerBoundConstraint` 确实被包装到各配置采样点；不过“采样点满足”不等于“整条路径无碰撞”。位置、点数、影响距离、求解精度均应单独诊断，不能用求解成功覆盖碰撞验证。

## 已确认：少量控制点与速度包络抬高时间代价

当前时间法把 DRM 节点作为控制点，并在首尾各复制一次，因此 m 个控制点在固定端点位置、端点速度后只有 `(m-4) × DOF` 个自由几何变量。直接 DRM 路径会产生 m=4 的唯一直线三次轨迹。

令单轴位移 D、归一化参数 s=t/T。四控制点休止三次曲线为

`q(s) = D(3s²−2s³)`。

其真实峰值速度为 `1.5D/T`，速度控制点包络为 `3D/T`，相差一倍。两种加速度界都是 `6D/T²`。所以 D=8、速度/加速度界均为 1 时：

- 无障碍任意休止轨迹的物理时间下界为 9 秒（梯形速度曲线）。
- 该固定三次曲线按真实连续导数只需 12 秒。
- KTO 的速度控制点界却要求至少 24 秒。

这不是 SNOPT/IPOPT 未找到更好的局部解：在这个固定转录内 24 秒就是最优。对均匀 clamped cubic 的 m 个控制点，还可由速度控制点差分求和得到必要下界 `T ≥ D / (1−2/[3(m−3)])`；增加控制点会改变有限维问题本身。

**加速度不是这一现象的来源。** 当前三次 spline 使用简单内部结点，二阶导为连续分段线性函数，分量最大绝对值在分段端点达到；其加速度控制点界在这里与连续分量界等价。速度的二次导数曲线则只有包络充分性，控制点不必被曲线取到。

## 实际 CPU 对照

命令（仓库根目录，现有 Conda 启动器）：

```bash
bash local/python.sh reproduction/reports/alignment_diagnosis/time_transcription_probe.py > reproduction/reports/alignment_diagnosis/time_transcription_probe.log 2>&1
```

固定 D=8、v=a=1、无障碍、休止端点、同阶均匀结点；改变控制点数。每项各用 SNOPT 和 IPOPT 求解，并用独立 SciPy B-spline 差分矩阵建立固定 T 线性可行性问题，经 40 次二分验证。全部 12 个求解成功；与 LP 数值可行性界的最大差为 `2.68e−8` 秒。LP 可行性容差为 `1e−9`，这是一项独立数值核验，不是严格算术证书。

| 控制点数 | SNOPT 最优时长（秒） | IPOPT 最优时长（秒） |
|---:|---:|---:|
| 4 | 24.000000 | 24.000000 |
| 5 | 12.000000 | 12.000000 |
| 6 | 10.285714 | 10.285714 |
| 8 | 9.403390 | 9.403390 |
| 12 | 9.358855 | 9.358855 |
| 20 | 9.090488 | 9.090488 |

原始控制点、结点、求解状态、解析导数极值、LP 二分区间及输入哈希全部在 [time_transcription_probe.json](time_transcription_probe.json)；命令输出在 [日志](time_transcription_probe.log)。这组实验没有碰撞约束，不能直接当作 Forest/FR3 新成功率。

## 冻结 793 条时间候选的解析导数复核

读取原 Forest 400、FR3 393 条时间候选，没有重新规划、优化、调用 GPU 或几何查询。对每个多项式区间枚举导数驻点和区间端点，计算连续分量导数极值，没有以有限时间网格代替极值。

| 环境 / 子集 | 数量 | 原平均时长（秒） | 保持相同几何曲线、按真实导数最短缩放（秒） |
|---|---:|---:|---:|
| Forest 四控制点严格有效候选 | 120 | 24.000000 | 12.000000 |
| Forest 全部严格有效候选 | 243 | 17.972381 | 11.948118 |
| FR3 四控制点严格有效候选 | 188 | 10.888022 | 5.499282 |
| FR3 全部严格有效候选 | 344 | 8.339562 | 5.394553 |

Forest 的 120 条四控制点候选真实速度峰值均仅 0.5，而控制点界已到 1。所有 793 条候选中，加速度包络与解析极值的最大绝对差不超过 `4.45e−16`。因此少控制点导致的速度约束保守性已经由主实验冻结数据和独立无障碍对照共同验证。

这里的“缩放”只是诊断性计算，没有替换主表数据，也没有声称进行了新的轨迹安全验证或恢复论文的实验结果。曲线几何保持不变，但它不再满足原来的更强速度控制点界，不能冒充同一 KTO 优化问题的结果。

## 仍然未决及建议的受控实验

1. 先查 DRM 路径段数差异的来源。当前 Forest/FR3 均有大量直接路径，而论文平均 3.45/4.49 段；更少控制点会同时影响 NTO 可表示路径、碰撞采样间隔和时间代价。该机制将任务分布差异传导到基线，并非跨机器速度差可以解释。
2. 在相同冻结 DRM 输入上分别诊断碰撞采样位置、点数和正裕量，每次只改变一项；先报告原约束残差、点间穿透及严格成功率。不要调整阈值把穿透判为有效。
3. 控制点数量可做单独敏感性实验，优先采用保留几何曲线的结点插入，再重新优化。它会扩大可行表示空间，必须标为构造变体；不能宣称论文未公开的控制数就是 8、12 或 20。
4. 真实连续速度约束或诊断性精确重定时能显示包络损失，但改变了原 KTO 转录。若采用，应另列方法，保留原 KTO 结果作为对照。
5. 正式 PDF 没有公开仿真速度/加速度数值、时间 spline 的端点处理、初始时长、完整结点构造及 NTO tuning。仅凭最终成本推回这些参数无法唯一确定；不得以贴近论文表格为选择参数的依据。

首轮诊断脚本把严格成功状态误读为 `success`（实际为 `success_numerical`），导致分组为空和 NaN。该次脚本、输出和日志保存在 [time_probe_attempt01](time_probe_attempt01/README.md)；修正只影响诊断汇总，首轮 12 次数值求解本身均成功。当前最终 JSON 禁止非有限数值，严格数量与主报告 243/344 相符。
