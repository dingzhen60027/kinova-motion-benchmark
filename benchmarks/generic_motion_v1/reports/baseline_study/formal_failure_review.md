# 正式主实验失败记录复查

本复查只解释 [主实验汇总](formal_main_metrics/metrics.json) 中四个障碍族的 **6000 个方法查询槽位**；不含 open_control 和 scaling，也不替代完整实验统计。规划源码版本为 `c8adbd8cef44db25b41559e8fbf2b38e120b624a`。只读检查 361 个相关 attempt，对读取的 1495 个文件逐项核对 seal receipt SHA；没有发现汇总状态与原始状态不一致或已读文件校验错误。全量明细、路径、校验值与边界说明保存在 [JSON 证据](formal_failure_review.json)。没有重跑求解器或 GPU，没有修改原始状态、指标或数值设置。

## 25 个验证未决：全部来自固定接触带

| 方法 | validation_undetermined | 每条曲线最小采样间隙范围 | 采样点数范围 | 接触带内 checker 分歧 |
|---|---:|---:|---:|---:|
| NTO-distance | 19 | −1.936 至 −0.066 µm | 739–1837 | 3 |
| NTO-time | 6 | −1.941 至 −0.163 µm | 1324–2183 | 1 |

25 次求解器均返回成功；全部通过端点、关节和导数界检查，实际相邻采样步长均不超过 0.005 rad，均没有达到 100000 点上限。直接重读保存的轨迹采样数组可确认：所有碰撞标记和 checker 分歧都位于预先固定的 `|clearance| ≤ 2e−6 m` 接触带内，带外碰撞点为零。距离/时间分别有 117/38 个接触带采样点。

因此这些未决是数值接触容差造成的边界判断，**不是采样上限耗尽或后端异常**。不得因穿透量很小、求解器成功而把它们改计为成功；有限采样本身也不是连续无碰撞证明。示例：`test-sparse_clutter-010-p02`、roadmap seed 0，距离最小间隙 −1.848 µm、1531 点；`test-sparse_clutter-006-p04`、seed 0，时间最小间隙 −1.941 µm、1977 点。证据：[距离示例 validation.json](../../results/rss2025_formal_v1_main_s000_sparse_clutter/jobs/test-sparse_clutter-010-p02-NTO_PAPER-minimum_distance-c849e95f/attempts/0001/validation.json)；[时间示例 validation.json](../../results/rss2025_formal_v1_main_s000_sparse_clutter/jobs/test-sparse_clutter-006-p04-NTO_PAPER-minimum_time-c9188d74/attempts/0001/validation.json)。

## 115 个基础设施错误：同一原生 DRM 断言

每个 label 有 23 个 `infrastructure_error`，恰好是 **23 个相同 case/roadmap 配对 × 五个方法 label**，涉及 11 个不同 case、3 个场景。115 次均停在 `drm_path`，进程 signal 6 / exit −6，stderr 全部包含同一断言：

```text
csdecomp/src/planning/drm_planner.cpp:249
DrmPlanner::GetClosestNonCollidingConfigurationInRoadMap(...)
Assertion `closest_node >= 0' failed.
```

源码含义是原生前端在该搜索中没有得到可用候选节点并触发断言。记录不能证明原连续规划问题无解，也不能归因于后续某个优化方法；115 次不是 115 个独立随机场景。示例 `test-dense_clutter-003-p00`、roadmap seed 0 的五个 label 均失败，见 [DRM 原始 stderr](../../results/rss2025_formal_v1_main_s000_dense_clutter/jobs/test-dense_clutter-003-p00-EI_ZO-minimum_distance-93c35df5/attempts/0001/stderr.log)。保持 `infrastructure_error` 原分类。

## 唯一 SCSTO 求解失败：Clarabel NumericalError

`test-route_choice-017-p00`、roadmap seed 1、job `test-route_choice-017-p00-EI_ZO-minimum_time-9339be1b`：前端已接受，失败阶段为 `trajectory_optimization`。日志明确记录 `INFO:drake:Clarabel returns NumericalError`，随后 `scsplanning/biconvex.py → fixed_points.py:90` 的 `assert result.is_success()` 抛出 `AssertionError`，没有返回候选轨迹。见 [SCSTO 原始 stderr](../../results/rss2025_formal_v1_main_s001_route_choice/jobs/test-route_choice-017-p00-EI_ZO-minimum_time-9339be1b/attempts/0001/stderr.log) 和 [SCSTO 调用栈](../../results/rss2025_formal_v1_main_s001_route_choice/jobs/test-route_choice-017-p00-EI_ZO-minimum_time-9339be1b/attempts/0001/exception.txt)。

这是实际的数值求解失败记录，不能解释为碰撞验证失败或已证明不可行。当前日志没有足够的结构化残差等信息判断数值错误的具体条件；保持 `solver_failed`，没有以重试结果替换该槽位。

## NTO 的正常终止失败与碰撞后端异常

| 方法 / 原生接口码 | 原日志含义 | 数量 |
|---|---|---:|
| 距离 SNOPT 13 | nonlinear infeasibilities minimized | 63 |
| 距离 SNOPT 31 | iteration limit reached | 51 |
| 距离 SNOPT 32 | major iteration limit reached | 42 |
| 距离 SNOPT 41 | current point cannot be improved | 41 |
| 距离 SNOPT 43 | cannot satisfy the general constraints | 1 |
| 距离 Drake wrapper exception | BoxBoxGradient 断言 | 5 |
| 时间 Drake IpoptSolverDetails.status 1 | Max iteration exceeded | 13 |
| 时间 Drake IpoptSolverDetails.status 6 | Local infeasibility | 3 |
| 时间 Drake IpoptSolverDetails.status 10 | Restoration failure | 1 |

距离合计 203 次、时间合计 17 次 `solver_failed`。IPOPT 表中代码是本机 Drake 返回的详情接口，不能套用另一种 IPOPT 枚举解释。51 个 SNOPT 31 的完整日志全部确认为 **10000 total iterations**；42 个 SNOPT 32 全部确认为 **2000 major iterations**。这与冻结配置/实测默认值一致，不是日志修复额外降低了预算，也不能据此宣称增加预算必能成功。

五个距离 wrapper 异常文本完全相同：`distance_to_shape_touching.cc:122` 的 `BoxBoxGradient()` 条件 `(X_WA * p_ACa - X_WB * p_BCb).norm() < kEps` 失败。其 SNOPT 日志打印 INFO 71（函数求值期间终止），但 wrapper 没有返回原生求解结果对象，因此不能把打印码冒充正常返回的 solver result。例见 [Drake 原始异常](../../results/rss2025_formal_v1_main_s000_route_choice/jobs/test-route_choice-014-p01-NTO_PAPER-minimum_distance-69864806/attempts/0001/exception.txt)。case 与 seed 为：

- `test-route_choice-014-p01`，roadmap seed 0。
- `test-sparse_clutter-002-p04`，roadmap seed 1。
- `test-sparse_clutter-013-p01`，roadmap seed 2。
- `test-dense_clutter-004-p03`，roadmap seed 2。
- `test-bottleneck-008-p04`，roadmap seed 2。

这些属于 Drake 碰撞后端故障，和前述 DRM 进程断言是不同故障族。既有 dev 诊断曾在保存的最后评估构型上复现同类 BoxBoxGradient 断言并定位 `left_inner_finger_collision` 与 `table::Box`；正式这五例未保存出错调用的构型/几何对，不能推断它们都由该几何对引发。

## 对 baseline 使用的边界

[既有 readiness 审查](nto_baseline_readiness.md) 已说明当前 NTO 是公开论文配置的 Kinova 适配：距离 order 2、时间 order 4，使用原始 DRM 节点及已声明的时间端点重复；没有为结果加密控制点。`Nc=150` 沿公开归档实现约定实际施加 149 个内部约束点，这与论文“150 points”的文字差异需要持续披露。零 clearance、有限配置点约束不能保证约束点之间无碰撞，因而必须保留独立的密集数值验证。

既有 dev 距离案例已出现“求解器返回成功、有限约束满足，但密集验证碰撞”，支持将优化成功与验证成功分开报告；它不证明每个失败问题不可行，也不足以推出算法普遍优劣。[日志修复实跑验收](nto_logging_real_validation.md) 证明六个固定 dev 查询的输入和数值参数不变、已返回候选与历史一致；正式运行使用完整原生日志，未因弱结果更改迭代上限。以上失败、未决和已披露适配应随主结果保留，不能统称为“原算法无解”。
