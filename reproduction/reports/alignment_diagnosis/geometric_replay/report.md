# 五个几何优化失败的固定输入诊断与修正

日期：2026-10-26。范围：旧 v5 的全部 **4 个 LGCS 区域残差失败**和 **1 个 Forest SCSTO assertion**。直接读取最后失败求解对应的冻结 `pre_solve` 半空间和端点，没有重新规划、生成区域、移动障碍或放宽验收。FR3 使用 `pre_solve_04`，其余使用 `pre_solve_00`。

## 实际结论

- 4/4 LGCS baseline 精确重现历史区域残差。只将 `IntFeasTol` 从默认 1e−5 收紧到 1e−9，4/4 都通过旧的 1e−5 区域验收。只收紧三个连续容差也 4/4 通过；四项同时收紧也 4/4 通过。
- SCSTO 的 assertion 可精确重现。原因链指向作者初始化中的近共线 knot 删除：把一个 **6.3997e−5** 的三角不等式残差视为共线，导致过渡点移动 **14.764866 mm**，跨出指定区域。单纯把 Clarabel 容差收紧至 1e−10 仍失败；保留所有初始 knot 后通过。
- 新 opt-in adapter 的实际最终验证是 **5/5 通过**，同时通过原 CUDA、独立碰撞、端点、关节界、区域和导数检查。模块对象前后相同；最终运行结束时核对 57 项输入/源码身份通过。

这不是新的 400 题结果，也不能将旧失败记录改为成功。**LGCS 的 54 次受限 Gurobi 许可证超规模失败仍未解决**；这些模型没有在本诊断中伪装成成功。

## LGCS：近整数流与恢复点的误差

原参数保留 `Threads=1, MIPGap=1e−6, TimeLimit=60, NonConvex=0, Presolve=0`。所有对照改变的只是以下指定容差，验收仍为原 1e−5：

- 连续三项：`FeasibilityTol=OptimalityTol=BarQCPConvTol=1e−9`，整数容差保持默认。
- 仅整数：`IntFeasTol=1e−9`，其他容差保持默认。
- 全部四项：上述四项均为 1e−9。

| 案例 | 原残差 | 仅连续三项收紧 | 仅 IntFeasTol 收紧 | 四项全部收紧 |
|---|---:|---:|---:|---:|
| forest / e00_r09_n00200 | 1.18999839e-05 | 3.89645841e-07 | -4.68011718e-08 | 3.99680289e-15 |
| forest / e02_r04_n01600 | 1.32514836e-05 | -5.03528552e-09 | 4.05733654e-07 | -5.03528552e-09 |
| forest / e02_r07_n01600 | 1.83735922e-05 | 1.11022302e-16 | -7.39773132e-10 | 1.11022302e-16 |
| fr3 / e01_r04_n06000 | 2.85835948e-05 | 2.40421905e-10 | 6.02933703e-10 | 2.40421905e-10 |

三种修正对照的 12/12 个求解均通过完整原验证，baseline 4/4 均复现旧失败。未改变区域矩阵、函数族、目标、图、恢复轮数或许可策略。

原四例 `IntVio` 分别是 3.36885e−6、5.43595e−6、7.76193e−6、6.20289e−6；都落在 Gurobi 默认 1e−5 整数容差内。v5 采用齐次约束 `A z ≤ b φ`，提取路线时把 `φ>0.5` 的流视为选中，并直接使用 z 作为点。近整数 φ 与 1 的偏差可被 b 的尺度放大为原空间区域残差。FR3 原解的 `ConstrVio` 只有 **1.3413e−9**，但原空间区域残差为 **2.8584e−5**，清楚表明“solver 线性约束满足”不自动等于“提取点在原区域内”。

仅收紧整数容差即可修正全部四例，支持近整数流是这里的重要机制；三项连续容差也改变求解器数值路径并提高整数精度。这不是证明其余所有模型都只有同一原因，也不证明数学全局最优精度高于保存的 MIP gap。原始及各变体 `.mps.gz`、完整变量值、流值、求解参数和质量指标都保存，便于独立检查。

## SCSTO：几何初始化问题，不是只调整数值验收

历史失败：Forest `e03_r03_n00400`，4 个固定凸区域，作者 SCSTO degree=6。实际仪器化调用顺序：

1. Polygonal SOCP 成功，目标 10.7335912123，最大约束残差 2.16e−9。
2. 作者近共线检测对三个内部 knot 的三角不等式残差为 `[2.7837e−10, 0.0292108420, 6.3996931e−5]`；默认阈值 1e−4 只保留 `[0,2,4]`。
3. 原第 3 个 knot 为 `[1.67562149,1.32592168]`。沿合并直线求 crossing time 后，实际过渡点是 `[1.66705440,1.33794691]`，位移 0.0147648661 m；相应区域半空间残差 0.0147584046。
4. 第一次 FixedPoints 求解器返回 `Solved`，约束残差 1.89e−10；该子问题跳过固定过渡点的区域约束，因此继承了已经越界的初始化 knot。
5. 第一次 FixedVelocities 返回 `PrimalInfeasible`，触发作者 assertion。不可把这个失败结果的数值向量当成有效轨迹或可接受解。

原作者相关代码：[polygonal.get_vertex_indices/get_crossing_time](https://github.com/TobiaMarcucci/scsplanning/blob/d3f885aa2b74a10418de96e61ec2df0bf6ef16b8/scsplanning/polygonal.py#L95)，[FixedPoints 跳过端点的区域约束](https://github.com/TobiaMarcucci/scsplanning/blob/d3f885aa2b74a10418de96e61ec2df0bf6ef16b8/scsplanning/fixed_points.py#L37)。这不是指责所有近共线简化都会失败；本例的问题是长度差阈值不能保证投影点仍在指定区域内。

| 固定输入变体 | 实际结果 | duration | 最大区域残差 |
|---|---|---:|---:|
| 原作者初始化/默认 Clarabel | assertion；FixedVelocities PrimalInfeasible | — | 初始曲线 0.0147584 |
| 仅 Clarabel tol_feas/tol_gap_abs/tol_gap_rel=1e−10、max_iter=300 | 同阶段 assertion | — | 没有修复几何初始化 |
| 仅近共线阈值 1e−6，保留 `[0,2,3,4]` | 原验证通过 | 10.00140850 s | −2.41e−12 |
| 仅近共线阈值 1e−10，保留所有 knot | 原验证通过 | 9.93928417 s | 3.02e−10 |
| 新 adapter：明确保留全部 knot，不使用近共线阈值 | 原验证通过 | 9.93928417 s | 3.02e−10 |

两次阈值实验用于分辨原因，不是生产参数搜索。最终 adapter 明确保留所有 knot；它修正的是 **14.8 mm 的几何初始化变化**，不是仅收紧 solver 容差，更不是放宽碰撞/区域判据。随后仍调用作者原有的两个凸子问题、交替次序及终止条件；最终函数族仍是一段 Bezier 对应一个输入凸区域。

## 可复用实现

`reproduction/alignment_diagnosis/geometric_solvers.py` 提供：

```python
from reproduction.alignment_diagnosis.geometric_solvers import (
    lgcs_strict, scsto_preserve_knots,
)
candidate = lgcs_strict(regions, start, goal)
candidate = scsto_preserve_knots(regions, start, goal, cfg)
```

两者都返回包含正确 `method` 的待验证 candidate，可直接传入原 validation，不能绕过独立检查。SCSTO 在调用作者 biconvex 第一个凸子问题之前检查初始曲线区域残差；若非有限或超过 `cfg["feasibility_tolerance"]`，抛出包含 `initialization` 数组/指标的明确异常，replay 会保存 `failed_initialization.json`，不改变区域。LGCS 保持 v5 原函数代码对象，只通过函数局部的 Gurobi factory 设置四项容差。SCSTO 保持作者 biconvex 原代码对象，只提供函数局部的 knot-preserving polygonal initializer。没有修改 `sys.modules`、模块全局、vendor 文件、v5 源码或环境；实际检查 `gp.Model`、作者 biconvex、get_vertex_indices、v5 lgcs 对象前后相同。小段初始化适配保留作者 MIT 版权与出处。

若初始区域段长度恰为零或非有限值，adapter 显式报错；未为这类未测输入静默删除区域。该限制不影响此次 5 个固定失败案例，不能据此宣称覆盖所有新任务。

## 命令、文件与失败保存

最终可直接复验命令，复用现有 Conda launcher（输出目录需不存在）：

```bash
bash local/python.sh -m reproduction.alignment_diagnosis.geometric_replay \
  --output reproduction/reports/alignment_diagnosis/geometric_replay/new_adapter_run \
  --use-adapters

bash local/python.sh -m reproduction.alignment_diagnosis.geometric_replay \
  --output reproduction/reports/alignment_diagnosis/geometric_replay/new_lgcs_run \
  --methods LGCS --lgcs-variants baseline tight_continuous tight_integer tight_all
```

实际已运行目录：

- `attempt_02/`：正确最终输入的 baseline + 连续三项对照，包含 SCSTO assertion 全调用轨迹。
- `lgcs_integer_all/`：仅整数与四项联合对照。
- `scsto_tight_01/`：紧 Clarabel 的失败对照。
- `scsto_vertex_baseline/`、`scsto_vertex_1e6/`、`scsto_vertex_1e10/`：初始化因果诊断。
- **`adapter_verification_03/`：最终 adapter 5/5 原验证通过，57 项输入/源码 hash 在结束时复核通过。**
- `attempt_01/`：第一次选择了 FR3 的 recovery=0 输入而非最终 recovery=4；两条轨迹仍碰撞。这是错误阶段的诊断，不能用于解释最终区域残差；其输入、结果、源码均保留。Forest 对照仍复现，但报告采用后续正确阶段的结果。
- `adapter_verification/`：首次局部 Gurobi proxy 漏代理 `quicksum`，4 个 LGCS 均显式报错；SCSTO 通过。错误源码和 traceback 保留，修复后才运行上述最终验证。

各阶段保留实际执行源码 snapshot、input_hashes、求解器日志/模型/变量值、逐次 SCSTO constraint arrays、初始曲线、最终轨迹和独立 validation。失败诊断 source hash 可能与当前修正后文件不同，已由同目录的 source snapshot 保存；历史 v5 输入没有改变。所有记录是诊断计算，记录的 elapsed 不用于论文速度比较。

## 最小正常行为控制与最终源码核验

新代码另实际执行固定单区域 2D 和 7D 内点控制，端点距区域边界均为 0.02。2D 使用真实 Forest 几何在场景左下角，7D 使用实际 FR3+桌子（无在线球障碍）；原 CUDA 和独立几何验证均执行，没有用恒真碰撞函数替代。

| 维数 / 方法 | 实测 objective | 解析/导数核验 | 原 validation |
|---|---:|---|---|
| 2D LGCS | 1.131370849906 | 解析直线长 1.131370849898 | 通过 |
| 2D SCSTO | 2.000000004 s | 速度控制点峰值 0.799999999，加速度 0.999999998 | 通过 |
| 7D LGCS | 0.026457513110645 | 解析直线长 0.026457513110646 | 通过 |
| 7D SCSTO | 0.223606799 s | 速度控制点峰值 0.089442718，加速度 0.999999989 | 通过 |

共4/4通过；SCSTO时间同时不低于对应单位速度/加速度bang-bang物理下界。每次调用后原模块对象均未改变。数据/轨迹/源码保存在 `normal_controls/`，命令为：

```bash
bash local/python.sh -m reproduction.alignment_diagnosis.geometric_replay \
  --output reproduction/reports/alignment_diagnosis/geometric_replay/new_normal_controls \
  --normal-controls
```

添加初始化 guard 和 method 元数据后，最终五个原失败输入再运行于 `adapter_verification_03/`，仍5/5通过，所有57项身份在结束时复核通过。最终 Python 编译检查、`git diff --check` 均通过。这些是9项固定输入控制/回归，不是新增论文规模benchmark。
