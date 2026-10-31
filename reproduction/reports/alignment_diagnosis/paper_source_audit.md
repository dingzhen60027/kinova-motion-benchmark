# 论文实验源与版本语义审计

日期：2026-10-26。审计基准：本地 `main`，HEAD `9b1d9719ac0ce3ed421ec4975c58a7b040f1ed99`。本报告只新增证据、逐行 diff 和审计结论；未改写旧实验、未切换环境、未申请许可证、未联系作者、未运行新批量评测。

## 结论

本轮没有找到作者原始 Forest/FR3/NTO 实验 runner、33 球模型或随机题单。已将“公开范围里没有找到”具体化为可复核的 refs、目录树及 **66 个去重 Python/Notebook 源文件的内容检索**，而非只查看主分支 README。

新的重要结果是：

1. 论文日期公开 CSDecomp 依赖文件确实指定 **Drake 1.38.0**；当前是 1.51.1。直接比较相关源文件后，**没有发现碰撞位置约束、距离惩罚、初猜、重建或速度/加速度约束数学定义的改变**。新增梯度稀疏模式可能影响运行速度及求解路径，但不能据此解释采样间漏碰撞。
2. 当前 SCSTO 作者实现虽晚于论文，但相对论文日期提交的算法代码改变仅为 pybezier API 兼容改名及注释。`biconvex` 的交替优化、终止条件没有变化。不能把当前时间代价反转直接归咎于“使用了不同的 SCSTO 算法”。
3. **NTO 安全间距、影响距离、控制点数/扩充规则、初始时间和 knot 参数化仍未披露**，而这些正是当前最值得做固定输入机制实验的因素。当前 `bound=0` 不等于已经恢复作者原设置。
4. 极少控制点的三次样条配合 Drake 导数控制点约束，有可解析说明的保守性。这个机制与当前路线更短、段数更少同时存在，应优先检查，而不是通过改变成功判据来追求论文百分比。

## 1. 证据优先级与公开搜索范围

采用优先级：正式论文正文/附录表格 → 公开作者实现（明确固定 commit）→ 原 TeX 的未发表注释 → 显式构造假设。注释不能覆盖正式表格，也不能当成找回原始运行代码。

主来源：

- [论文 v1](https://arxiv.org/abs/2504.10783v1)，[PDF](https://arxiv.org/pdf/2504.10783v1)，[作者原 TeX](https://arxiv.org/src/2504.10783v1)。本地 PDF SHA-256 `579c9917425fe01b583f067d0a1c9702917ccd810584dbac562fcdf9fead51d0`；TeX 归档 SHA-256 `18c4140e37eb05abe636dd4999612618f565c120ce57fbe966e39545b68f1986`。
- [论文日期 CSDecomp](https://github.com/wernerpe/csdecomp/tree/ee94f8145bb32ef6c08b1e575218335b9daf533f)，2025-03-12；[实际 wheel 对应 v0.1.2](https://github.com/wernerpe/csdecomp/tree/0c26beafd99aa1eb5da39e79f9b9f6df1b404eb5)，2026-03-19。
- [SCSTO 论文日期代码](https://github.com/TobiaMarcucci/scsplanning/tree/882d07ced8925a92723ff0e809a9a3298d1dd032)，[实际代码](https://github.com/TobiaMarcucci/scsplanning/tree/d3f885aa2b74a10418de96e61ec2df0bf6ef16b8)。`882d07c` 的提交时间是美国西岸 2025-04-15 21:11；此提交改包名，前一版 `24c809c` 为 2024-11-20。它是公开的论文时期实现，不是作者私有实验环境的证明。

本轮实际下载并保存：

| 范围 | 实际核对 | 结果与限制 |
|---|---|---|
| CSDecomp | 10 个公开 branch、3 个 tag、全部 9 个 issue/PR 元数据、release 列表 | release 列表为空；13 个完整递归树均未截断；当前 main 为 `7661a97bb7c4931be73d2bc4f8c45c3094ff476e` |
| scsplanning | 3 个 branch、tag/release 列表、全部 6 个 issue/PR 元数据 | 3 个树均未截断；未发现本论文的 NTO 实验配置 |
| 上述所有 refs 的 Python/Notebook | 去重 66 个 Git blob，66/66 实际下载成功；内容检索 `KinematicTrajectoryOptimization`, `MinimumDistanceLowerBoundConstraint`, `NTO`, `forest`, `fr3`, `franka`, `benchmark` | 仅一个 Kinova Notebook 的一般说明匹配；未发现 Forest/FR3/NTO runner。不能排除其他语言、私有文件或未发布对象 |
| 作者公开仓库目录 | wernerpe 42 个、TobiaMarcucci 17 个公开仓库元数据 | 不把目录名检索等同于每个仓库全部历史的内容审计 |
| 相关仓库补查 | `pybmtp`, `cspace_utils`, `iris_benchmarks`, `robot_description_utils`, `drake-franka-setup`, `convex-obstacles-vcc` 的完整默认分支树 | 未找到本论文的 33 球 FR3 或 NTO runner。`pybmtp`、`drake-franka-setup` 晚于论文；不能自动当作原资产 |

具体 URL、SHA、路径、字节数、命中行和树截断标志见 `source_evidence/fetch_manifest.json`、`ref_tree_manifest.json`、`public_python_search.json`、`related_repos_tree_manifest.json`。第一次通用网页搜索只返回非作者索引/转贴，没有作为技术结论证据；随后改用直接 GitHub primary-source API 与固定 commit 原文。

先前审计还核对过 8 个 PR head 的递归树和原 TeX 全归档，见 `../paper_completion/source_audit.md`。本轮未把此前的检查冒充新下载。

## 2. NTO：哪些明确，哪些仍然缺失

下列行号相对于本地 `../paper_completion/source_evidence/arxiv_text/sections/` 下作者原 TeX。

| 项目 | 证据位置 | 分类 | 对当前实现的含义 |
|---|---|---|---|
| Drake KTO、真正几何距离约束 | `experiments.tex:47` | 正式发表 | 当前确实调用 KTO 和 MinimumDistanceLowerBoundConstraint；不是自制代价替代碰撞约束 |
| Forest solver 与 Nc | `appendix.tex:69` | 正式发表 | 距离 SNOPT/Nc=20；时间 SNOPT/Nc=100 |
| FR3 solver 与 Nc | `appendix.tex:69` | 正式发表 | 距离 SNOPT、时间 IPOPT；两者 Nc=150 |
| 时间样条 order=4、DRM 节点初始化控制点 | `experiments.tex:49` | 正式发表 | order=4 指 cubic；当前 order 符合。如何处理不足 4 个节点和端点静止约束未写 |
| 距离曲线为 PWL，以 DRM 路线 warm start | `experiments.tex:21-29,49` | 正式发表 | 二阶 B-spline 可表述 PWL；不确定原始控制点数是否严格等于 DRM 节点数 |
| 碰撞位置 k/Nc, k=1,…,Nc−1 | `appendix.tex:175` | 未发表草稿 | 显式公式给 Nc−1 个内点，和正式正文 Nc 个点有计数歧义。v4 保留此解释；v5 采用 Nc 个内点是另一个声明的构造 |
| 距离样条 order=2、T=1、相邻控制点距离和 | `appendix.tex:179-190` | 未发表草稿 | 支持当前转录。草稿索引不完全一致，未给 M 的选择/补点规则 |
| 最小允许间距 ε | `appendix.tex:190` | 草稿定义，数值未披露 | 当前 0.0 m 是构造假设；EI-ZO configuration margin 不等于 NTO 几何 clearance |
| 影响距离、penalty function | 正文和该草稿均无数值 | 未披露 | 当前 influence offset=0.01 m、Drake 默认 penalty 是构造选择 |
| knot vector、端点复制、initial duration、duration bounds | 无完整说明；`appendix.tex:193` 时间转录仍是 TODO | 未披露 | 当前 uniform clamped、两端各复制一次、path length+2 秒、[0.05,100] 秒均不应表述为原作者设置 |
| 求解容差、迭代/时间预算、缩放与 solver minor options | 无完整说明 | 未披露 | 当前 1e−6、2000、60 秒和案例 240 秒是复现工程限制 |
| simulation velocity/acceleration bounds、SCSTO degree | 硬件 bounds 在 `experiments.tex:85`，simulation 未具体给出 | 未披露 | 当前单位 bounds 及 degree=6 需要继续标注为构造；不能把硬件配置自动当作 simulation 原配置 |

论文说明作者分别调优了 NTO（`experiments.tex:63`），因此在预声明范围内对未披露项做固定输入的敏感性实验是合理的复现工作。必须区分“机制诊断/重新调优”与“恢复原始参数”；不能根据论文百分比挑选最接近的配置并称其为原配置。

公开 `scsplanning/nonconvex.py` **不是**本论文的 NTO baseline：它优化经过已知凸区域的复合 Bezier 曲线，在 [72–81 行](https://github.com/TobiaMarcucci/scsplanning/blob/d3f885aa2b74a10418de96e61ec2df0bf6ef16b8/scsplanning/nonconvex.py#L72)约束控制点属于各凸集；本论文 baseline 则是在完整机器人自由空间对 B-spline 施加离散几何距离约束。不能直接替换再沿用 NTO 名称。

## 3. Drake 1.38.0 → 1.51.1：直接语义比较

论文日期 CSDecomp 的 [examples/pyproject.toml:10](https://github.com/wernerpe/csdecomp/blob/ee94f8145bb32ef6c08b1e575218335b9daf533f/examples/pyproject.toml#L10) 依赖 `^1.38.0`，[锁定 requirements:13](https://github.com/wernerpe/csdecomp/blob/ee94f8145bb32ef6c08b1e575218335b9daf533f/csdecomp/third_party/python/requirements.txt#L13) 为 `drake==1.38.0`。这证明公开仓库当时的依赖，并不能证明私有 benchmark 的确切二进制版本。

实际通过固定 tag 原文下载、逐行 diff：

| Drake 文件 | 比较结果 | 影响判定 |
|---|---|---|
| `planning/trajectory_optimization/kinematic_trajectory_optimization.h` | 字节文本相同 | 本次使用 API 合同没有变化 |
| 同名 `.cc` | 新增 WrappedVelocityConstraint、DerivativeConstraint、EffortConstraint 的梯度稀疏模式；修正注释 | 可能改变求解速度/稀疏 Jacobian 路径；数学表达式未改 |
| `multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.cc/.h` | 仅显式析构函数 | 距离 bound、influence offset、callback 和 Eval 转发未改 |
| `multibody/inverse_kinematics/distance_constraint_utilities.cc/.h` | 文本相同 | 本次审计的 MultibodyPlant 距离/梯度计算适配逻辑未改 |
| `solvers/minimum_value_constraint.cc` | 文本相同 | 默认平滑惩罚/聚合逻辑未改 |

关键源位置：[1.38 KTO AddPathPositionConstraint](https://github.com/RobotLocomotion/drake/blob/v1.38.0/planning/trajectory_optimization/kinematic_trajectory_optimization.cc#L380)、[1.38 初猜与重建](https://github.com/RobotLocomotion/drake/blob/v1.38.0/planning/trajectory_optimization/kinematic_trajectory_optimization.cc#L342)、[1.51 速度界](https://github.com/RobotLocomotion/drake/blob/v1.51.1/planning/trajectory_optimization/kinematic_trajectory_optimization.cc#L660)、[1.51 加速度界](https://github.com/RobotLocomotion/drake/blob/v1.51.1/planning/trajectory_optimization/kinematic_trajectory_optimization.cc#L702)。源文件与完整 diff 均保存在 `source_evidence/drake_*`。

这排除了这些特定 API 在两 tag 之间改变上述数学语义的假说，**不是**整个 Drake/几何引擎/SNOPT/IPOPT 的数值等价证明；本轮没有执行旧二进制 A/B，也没有声称二者优化结果必然相同。

## 4. 控制点数量与时间代价：可检验的解析机制

Drake 对 B-spline 导数控制点施加 bounds，利用凸包性质保证整个区间内满足速度/加速度限制。它是充分条件，未必是曲线真实极值的必要条件。源码从 1.38 到 1.51 保持该性质。

对两节点直线加两端复制的 cubic 曲线，控制点为 `[s,s,g,g]`，记 `D=max_j|g_j−s_j|`，单位对称速度/加速度 bounds：

- 导数控制点是 `[0,3(g−s)/T,0]`，当前 KTO 表述强制 `T≥3D`；
- 曲线的真实峰值速度只有 `1.5D/T`；
- 加速度端点峰值为 `6D/T²`，因此还要求 `T≥sqrt(6D)`。

因此在这种特定四控制点情况下，即使无障碍、solver 全局解决了现有转录，也会因速度控制点界而受到比曲线真实速度条件更强的限制。该结论来自固定样条的解析式，不是本轮新实验数字。

这提供了明确诊断方向：对**同一保存输入和同一初始曲线**做 knot insertion，增加控制点后重新优化，同时检查活跃速度/加速度约束和密集真实导数。精确插结不会改变初始曲线，可减少“换了初猜”这一混杂因素。它仍然属于有声明的敏感性实验；作者是否也补点、如何补点仍未知。

当前 FR3/Forest DRM 段数与论文不同，会改变控制点数与函数族；不能只比较最终平均 duration 就推断算法实现错误或优于原论文。该机制也不能解释所有 NTO 距离漏碰撞，后者必须单独检查约束采样点与间隔。

## 5. SCSTO：原作者代码的实际变化

对上述固定 commits 执行 `git diff`，结果保存为 `source_evidence/scsplanning_paperdate_to_installed.diff`。

- [biconvex.py](https://github.com/TobiaMarcucci/scsplanning/blob/d3f885aa2b74a10418de96e61ec2df0bf6ef16b8/scsplanning/biconvex.py#L9) 的交替次序、`tol=1e−2` 默认、相对改进终止条件未变。
- `FixedPoints` / `FixedVelocities` 中，`transition_points()`、`durations()` 改为属性访问；其他差异为注释。
- `polygonal.py` 中 `domain_split` 改为 `split_domain`，适配 pybezier。该问题有作者仓库的 [issue #5](https://github.com/TobiaMarcucci/scsplanning/issues/5) 与 [PR #6](https://github.com/TobiaMarcucci/scsplanning/pull/6) 记录。
- requirements 从未锁定的 `pydrake/pybezier` 改为 `drake/scipy/pybezier>=0.1.4`。没有恢复作者实验机器的全套锁文件。
- scsplanning 当前 hello_world [degree=5](https://github.com/TobiaMarcucci/scsplanning/blob/d3f885aa2b74a10418de96e61ec2df0bf6ef16b8/examples/hello_world.py#L22)，较晚 CSDecomp Kinova 示例 [degree=6](https://github.com/wernerpe/csdecomp/blob/0c26beafd99aa1eb5da39e79f9b9f6df1b404eb5/examples/kinova_mintime_planning.py#L139)。二者都是示例配置，不能从任一项证明本论文 simulation 的 degree。

因此当前真正未对齐的 SCSTO 条件集中在输入凸区域、degree、simulation bounds、数值依赖及恢复流程，而非已发现的作者算法实质改版。旧 author assertion failure 仍须保留并检查具体输入，不应用“相同源代码”保证所有输入成功。

## 6. CSDecomp 版本与实验条件仍有实质差异

历史 core 与 wheel 的 DRM / roadmap diff 保存为 `source_evidence/csdecomp_paperdate_to_wheel_drm.diff`。新实现增加加载路线图时按当前几何重新验证节点、区分永久几何失效节点与环境碰撞节点；roadmap builder 和碰撞接口也有差异。GPU 区域编辑是在 [2025-06-29 PR #8](https://github.com/wernerpe/csdecomp/pull/8) 公开的，晚于论文日期 main。

此前已经独立构建并小规模验证论文日期 core；该成功不能把现代 wheel 上的 800 例自动变为旧版结果。为研究 NTO 机制应先冻结保存的路线，避免同时重建 DRM、换模型和改 optimizer，导致原因无法分辨。

模型与任务缺口仍未被此次源搜索补齐：作者 33 球资产、原始位姿/随机 seeds、具体 obstacle box、原碰撞过滤和实机观测未恢复。论文写 FR3 1187/527 pairs、KUKA 11 boxes/105 self pairs，但仅凭总计数不能反推唯一物理模型或过滤。应保持当前真实候选对记录，不增加无物理依据的 pair 来凑数。

## 7. 建议的验证顺序与判定标准

1. 固定原 v5 全部输入，重新评价 NTO 各实际约束点的几何距离及 constraint scalar、solver feasibility，同时评价区间。把“约束点失效”“离散点之间失效”“边界数值差异”分开。
2. 在预先声明、有限的敏感性集合中检查 Nc 网格相位、clearance/influence offset、knot insertion 和初始 duration。保留每次失败、时间和输入 hash；将声明的构造调整与正式论文配置并列。
3. 优先恢复最低限度方法行为，再用原始任务集做完整回归。改变任务难度/机器人球模型的实验另列，不和 optimizer 修正混成一个对照。
4. 仍需非受限 Gurobi 才能直接完成超限 LGCS 的作者求解器路线；枚举校验可以验证数学最优解，却不能冒充 Gurobi 时间。实机数据必须来自真实设备或作者原日志，仿真不能补齐该证据。

## 实际执行记录与证据完整性

本轮执行了：本地源码/TeX 逐行读取；两组固定 commit 的 `git diff`；使用 Python `urllib.request` 对 GitHub API 和 raw 固定引用做只读下载；66 个去重源文件内容检索；Drake 7 组文件的文本 diff。未修改任何被比较的源文件。

每次下载的 URL、文件路径与 SHA 保存于本目录的各 manifest。所有 evidence 文件另有统一 `source_evidence_inventory.json`（相对路径、字节数、SHA-256）；网络原文不是执行指令。未发现原 runner 的结论严格限定于上述公开范围。此次报告不包含新增 paper benchmark 分数，不将源代码审计说成数值验证。
