# Drake BoxBoxGradient：只读后端调查

调查日期：2026-10-29。范围：读取已保存的本机最小复现、冻结源码和官方 Drake 源码／issue／PR；未执行新的几何查询、优化器或 GPU 作业，未安装依赖，未修改冻结 runtime、模型、碰撞过滤、目标或历史结果。

**结论：当前 double API 也会失败，有明确的源码原因。官方仓库已有精确针对该问题的候选补丁 [PR #25144](https://github.com/RobotLocomotion/drake/pull/25144)，但调查时仍未合并，不能称为已发布或本机已验证的修复。** 本机 Drake 1.51.1、最新发布版 1.58.0 和本次查询到的 master 中，两份直接相关实现逐字相同。现有公共 signed-distance API 没有跳过 normal 计算、仅返回标量距离的开关；单独改用 double、逐对查询或升级到当前 release，都没有源码依据可以保证消除这次故障。

## 本机已发生了什么

既有 [最小复现执行记录](../baseline_study/nto_refinement_cpu_v1/boxbox_minimal_reproduction_module.json) 显示，独立 CPU 进程调用 `QueryObject[double].ComputeSignedDistancePairClosestPoints`，退出码为 1，触发 `distance_to_shape_touching.cc:122` 中的 BoxBoxGradient 断言。该调用不依赖 SNOPT、IPOPT 或 AutoDiff。具体碰撞对为 `robotiq::left_inner_finger_collision` 与 `table::Box`，不是本次调查新增或替换的几何。

固定输入保存在 [last_evaluated_q.npy](../baseline_study/nto_backend_last_q_probe/last_evaluated_q.npy)，SHA256 为 `46faf400094085a1474cd9cfb3729d1eecb48143bcd4a5474e34741247ddcbb4`。对应 [diagnosis.json](../baseline_study/nto_backend_last_q_probe/diagnosis.json) 已明确：这是原优化异常后捕获的 context positions，不能证明它就是原失败 callback 的准确 collocation 点；但随后独立单对查询确实在此固定 q 复现了同一断言。

[既有官方 API 对照](../baseline_study/nto_refinement_cpu_v1/boxbox_official_apis/comparison.json) 在未改模型、未改碰撞对的条件下记录：

| double API | 实际返回 | 说明 |
|---|---|---|
| `ComputeSignedDistancePairClosestPoints` | 退出码 1，同一断言 | 逐对查询仍走相同 narrow phase |
| `ComputeSignedDistancePairwiseClosestPoints` | 退出码 1，同一断言 | 全候选对接口不能绕过它 |
| `ComputePointPairPenetration` | 退出码 0，0 个 penetration pair | 仅返回穿透信息；不能由此推出满足正的 clearance 下界 |

以上是先前实际运行的保存结果，本次没有重跑，也没有将诊断点作为新 benchmark 成绩。

## 具体触发链

1. `proximity_engine.cc` 的两个 signed-distance API 都设置 nearest points、signed distance、`GST_LIBCCD`，再调用同一个 `shape_distance::Callback`。
2. box–box 满足 `RequiresFallback`，进入 `CalcDistanceFallback<double>`。FCL 先算出标量 `result.min_distance` 和世界系 witness points，Drake 将 witness points 变换到各几何局部坐标。
3. 当 `abs(result.min_distance) < 1e-14` 时，即便调用者只需要 `.distance`，Drake 仍执行 `CalcGradientWhenTouching` 来填充返回结构的 normal；box–box 分支进入 `BoxBoxGradient`。
4. `BoxBoxGradient` 又把两个局部 witness points 变回世界系，要求二者欧氏距离严格小于同一个绝对阈值 `1e-14`。这一要求不能从第 3 步的标量距离判断推出：微小穿透本来就可产生不重合的 witness points，坐标往返变换还会增加浮点误差。

可逐行核查保存的 [fallback 实现](boxbox_source_evidence/installed_geometry__proximity__distance_to_shape_callback.cc)、[touching 实现](boxbox_source_evidence/installed_geometry__proximity__distance_to_shape_touching.cc) 和 [API 实现](boxbox_source_evidence/installed_geometry__proximity_engine.cc)。[官方 issue #21675](https://github.com/RobotLocomotion/drake/issues/21675) 报告相同的轨迹优化失败；[2026-10-27 的上游最小示例](https://github.com/RobotLocomotion/drake/issues/21675#issuecomment-6449437279) 中，FCL signed distance 约为 −9.77e−15，而重建 witness 间距约为 1.06e−14，因此落入 touching 分支后失败。这两个数值来自上游两盒示例，**不是本机 Kinova 碰撞对的实测距离**。

这是 double 后处理中的几何数值断言，不是“把 AutoDiff 改成有限差分”便能消失的问题。NTO 的官方 `MinimumDistanceLowerBoundConstraint` 需要相同距离后端；独立的本机 `Model.clearance()` 也查询 all-pairs double signed distance，所以故障可能发生在约束或验证阶段。需要保留具体 failed stage，不能一律解释为算法找不到路径。

## 发布版本和上游补丁核查

| 对象 | 本次固定身份 | 相关实现状态 |
|---|---|---|
| 本机 pydrake | 1.51.1；VERSION.TXT commit `778ac9e2147ae4b9053db8489196f9d38cd5d9ad` | 存在上述断言 |
| 官方最新 release | v1.58.0，2026-10-15 发布 | touching.cc 与 callback.cc 均与本机对应源码逐字相同 |
| 官方 master 快照 | `9726576eb689de7582ed81c4c6b8319aef0ae3da` | 同上 |
| PR #25144 | head `f77d3d6d8bce599ece27319b258f83db170063e2` | open，merged=false，非 draft；保存的 reviews 列表为空 |

三份 `distance_to_shape_touching.cc` 的 SHA256 均为 `61890282220f57c90c8e1267f27ac7510dab4802e3b971f541d983b088c22c49`；三份 `distance_to_shape_callback.cc` 均为 `6b57260cb7cf733610a52de7e9ba1b3d3eec9ed3ee3d01875a116193273259d6`。这证明关键逻辑相同，不等同于不同版本整体行为逐点相同：FCL 依赖版本已经变化，可能改变某些输入的浮点路径。没有实际安装／运行新版本，故不声称升级后本机每个案例必然仍失败；只能说升级不能作为已证实修复。

该 [候选补丁](https://github.com/RobotLocomotion/drake/pull/25144/files) 删除重复的 witness 重合断言；保留原 feature-based normal 路径；若无法判定接触 feature，则用盒体 15 个 separating-axis 候选中的最小重叠方向替代最终 unreachable。补丁修改 normal 构造，不修改 FCL 返回的标量 distance、几何尺寸、碰撞过滤或优化目标。保存的补丁还加入微小穿透、缺失 feature、edge-cross-edge、远离原点及 callback 回归示例。完整源码与补丁在 [证据目录](boxbox_source_evidence/)。

这是一条值得独立验证的上游实现路径，不是已有本机成功结果。官方 PR 尚在 review；本轮查询 commit CI status 时 GitHub 返回 HTTP 403 rate limit，已记录在 [investigation_fetch_status.json](boxbox_source_evidence/investigation_fetch_status.json)。因此既不声称上游 CI 已通过，也不声称补丁已解决本机案例。较早的 [PR #21266](https://github.com/RobotLocomotion/drake/pull/21266) 修的是另一处 BoxBoxGradient 问题，早已包含在当前源码，不能拿它作为本次断言已修复的证据。

## 能否使用准确的纯数值距离入口

| 候选 | 保持原问题的判断 | 目前证据与限制 |
|---|---|---|
| 当前公开 double pair/all-pairs signed-distance | 接口和模型可保持，但故障未消除 | 已保存的两条调用均真实失败；返回结构仍强制构造 normal |
| 更换为 penetration API | 不等价 | 不返回正间隙；无 penetration 不能证明 clearance ≥ 1 mm 等正下界 |
| point-to-shape 距离或顶点采样 | 不等价 | 一般盒体最近点可在边／面内部，不能把有限点样本当作 exact box–box signed distance |
| 修改 SceneGraph/FCL distance tolerance | 无可靠修复依据 | FCL 迭代容差与源码硬编码 touching 阈值是不同量；提高容差也改变数值行为，并不建立 witness 断言的逻辑保证 |
| 直接调用 FCL 标量 `min_distance` | 底层 C++ 存在该量，但无现成 Drake Python drop-in | 必须准确复现 geometry pose、signed penetration、候选／过滤对及梯度；现有公共 API 没有“只返回 scalar、不构造 normal”的选项。未经实现和验证不能称为官方可用替换 |
| 删除碰撞对、球化／缩小盒体、换 Convex 表达 | 本任务不采用 | 改变冻结几何或查询路径，不能解释为保持模型的后端修复 |
| 在隔离后端构建并验证 PR #25144 | 源码层面保持几何、过滤和标量距离目标 | 是具体候选路径；仍需实际编译、固定 q 回归、非接触距离及 normal／约束导数检查后才能采用 |

接口含义可核对 [官方 QueryObject 文档](https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_query_object.html)。另一个容易遗漏的差异是：pair API 的内部 callback data 不使用全局碰撞 filter。即使未来逐对构建替代 evaluator，也必须从 canonical 允许碰撞对清单出发，不能无意增加或删减约束。

## 可执行的下一步边界

当前 950-slot 批次使用冻结后端，继续如实保存该错误；本次调查不改变它。待批次结束，在**另一个可追踪的后端构建目录**固定 PR head 或其后正式合并 commit：先运行已有 `reproduce_isolated_pair` 和 pair/all/penetration 对照；再运行补丁自带的近接触回归，并检查保留的 geometry、filter、固定 q 及正常非接触距离。之后检查约束值和非光滑点之外的导数，再单独执行 CPU 开发案例。新后端结果必须另立 runtime 身份，不能覆盖或混入当前批次与旧 formal 数据。

上述顺序有明确可证伪标准：若补丁仍触发该 q 的异常、改变非近接触距离／过滤对，或产生不可接受的约束导数，就不能进入 baseline 主矩阵。接触处 normal 可能不唯一；即便修复不再异常，也不能由此声称优化器保证成功，更不能把有限采样验证解释为数学连续无碰撞证明。

原始网络读取 URL 和 SHA256 全部保存于 [sources.json](boxbox_source_evidence/sources.json)。该文件、已保存本机失败记录和本报告支持以上结论；没有新增规划成绩。
