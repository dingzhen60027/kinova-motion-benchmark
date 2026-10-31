# General Manipulator Motion Planning Benchmark v1.0

**用途**：针对 `wernerpe/csdecomp` 的通用运动规划研究基准。输入只包含机器人、已知静态障碍、固定关节起终点、运动约束和目标函数；不引入抓取、装配、末端修正、语义、视觉策略或训练数据。

**状态**：实施规范，尚非已生成/已验证的数据集；实际样本数、场景有效率与性能必须由本地运行得到。首次生成即版本化冻结测试集。

## 1. 与 RSS 2025 的关系

Werner et al., *Superfast Configuration-Space Convex Set Computation on GPUs for Online Motion Planning*, RSS 2025，VII-A、Appendix D：
- Forest：10×10的二维域、中心7×7区域内随机15个半径0.35的圆障碍，固定左下至右上的路径问题。
- Franka：FR3 7 DoF；桌面加随机20个球障碍；原文用33个机器人碰撞球，分别随机在桌面左右两侧生成起点和目标。
- 两类问题：minimum-distance PWL path；minimum-time smooth trajectory with joint velocity/acceleration limits and zero endpoint velocities。
- 10个环境种子×10个DRM种子×4档路网节点数，每一种求解方法400种环境-路网组合；Franka的路网规模3000/6000/9000/12000。
- 报告无条件求解率和给定DRM成功时的条件求解率，不混淆 offline roadmap time 与 online query time。

需要特别区分：本规范采用 Kinova Gen3（当前 csdecomp 直接提供完整示例）而不是论文 FR3 时，称为 **RSS-inspired Kinova-7D adaptation**，不得称为对作者 FR3 benchmark 的精确复现。若以后实现 FR3 再单独建 paper-fidelity suite。

## 2. 规范输入与机器人

一条问题精确定义为：

`(benchmark_version, robot_model_sha256, geometry_protocol_sha256, scene_id, q_start[7], q_goal[7], objective, roadmap_id, method_seed)`。

- 主机器人：优先采用 csdecomp 已有 Kinova Gen3 7DoF 及固定夹爪模型，不增加任务工具。
- 单一 canonical model：对比 `simple_kinova_sens_on_table.yml`、`kinova_sens_on_table.yml`，其当前示例可能使用不同简化 URDF；先确认7个活动关节、关节顺序、碰撞体、过滤对、关节上下限、基座、TCP/夹爪位置；建立唯一正式模型协议。DRM离线建图和在线碰撞必须是**同一个canonical模型**；不允许混用两套模型而不检查。
- 基础桌面和静态设备若纳入，则对全部环境保持一致；动态变化只来自下面的新增障碍物。
- 机器人姿态、四元数和世界坐标系全部采用固定约定并写入manifest；无需IK，起终点直接给定关节构型。
- 障碍物首先以真实几何定义（球／立方体／墙体等），再通过**统一的保守**球-体素表示喂给csdecomp。保存原始几何与已转换的球心、半径、体素数、坐标范围、转换误差。初版统一体素球半径原则上0.04m；如果由体素网格近似立方体，球半径至少覆盖该体素对角半径，不能留下漏碰撞缝隙。
- GPU/CPU碰撞模型需在同一组随机构型与边上交叉校验并统计不一致；性能测试使用统一冻结的主模型，碰撞验证另报更细检查，不能随方法切换模型。

## 3. 测试场景（通用，不限定任务）

正式测试集（test，固定一次生成，算法开发阶段只读development）：

| family | 场景数 | 每场景起终点对数 | 案例数 | 环境生成原则 |
|---|---:|---:|---:|---|
| sparse_clutter | 20 | 5 | 100 | 随机6–12个障碍物，分散布置，覆盖容易和被直线阻塞的问题 |
| dense_clutter | 20 | 5 | 100 | 随机20–40个球/球簇障碍物，中心采样于机械臂有效工作空间内，允许多种形态 |
| bottleneck | 20 | 5 | 100 | 若干平面/障碍簇之间有有限宽通行区域（无需声称C-space拓扑已知），改变间隙和朝向 |
| route_choice | 20 | 5 | 100 | 两簇障碍形成多种几何绕行可能性，改变间距、大小及摆放；不预先认定有多个C-space同伦类 |
| open_control | 4 | 5 | 20 | 基础模型，无新增障碍，用于分析额外计算和路径过度优化 |
| **合计** | **84** |  | **420** | |

独立开发集（dev）：4类×5个布局×3对起终点=60题，与测试集**不共享**随机种子、场景、起终点。

场景样本由种子决定，模型变更或碰撞体半径变更则创建新benchmark版本。环境默认世界坐标在机器人base固定下，新增障碍物ROI应由现有kinova场景的可达工作空间样本核验，不允许把所有障碍物采在机械臂不可达区。所有障碍族配置上限、实际可生成数量及拒绝次数在开发集校准并冻结，正式集不以任一被测方法的成功率筛题。

### 固定起终点规则

1. 从关节限制内部的配置池按公开RNG采样；仅筛选无自碰撞、对当前场景无环境碰撞、离关节限位有预设裕度的端点。
2. 以规范化关节距离分层（短／中／长），并用独立几何checker判定关节直线段是否碰撞；尽量包含直线可行和被阻塞两种情况。
3. 一经写入manifest，即所有方法、路网规模及随机种子都采用完全相同的端点。不准重新选择IK或取最好结果。
4. **不**把DRM/RRT/本文方法的成功作为数据保留条件；未知是否全局可行要标记为`feasibility_unknown`，不准把算法超时直接断言不可行。
5. 额外可建`witness_positive`开发套件（通过独立可行路径构造），只能作为工程测试，不并入独立测试集统计。

## 4. RSS参考轨道

- `forest2d_reference`：尽可能忠实于RSS 2025的二维Forest参数，用于区域计算机制可视化、数值验收、复杂度对照。不要把2D与7D混成一个成功率。
- `kinova7_generic_v1`：上述420题，正式主套件。考虑后续附加 `fr3_paper_fidelity` 作为可选独立轨道，保留原文机器人配置、球体几何与场景参数。
- `external_mbm_v1`：在主方法稳定后把MBM/Robometrics中公开Panda问题做独立外部测试；协议不一致时明确标注，不把它与自建集叠加假充独立数据源。

## 5. 两种独立优化问题（同一数据问题，不重新生成场景）

**minimum_distance**：求无碰撞PWL路径，从`q_start`到`q_goal`，最小化关节空间欧氏弧长`sum(||q_{k+1}-q_k||_2)`；关节单位rad，另报归一化长度。初版不引入任务语义代价。

**minimum_time**：连续时间曲线q(t)，无碰撞、端点位置固定、端点速度为0，所有关节的`|qdot_j|<=vmax_j`、`|qddot_j|<=amax_j`，最小化T；建议与原论文示例一致先使用每轴1 rad/s、1 rad/s^2，并保留第二种“URDF限制”配置作为独立实验协议；不能在方法间改变限幅值。

曲线优化使用csdecomp/SCSPlanning原有接口。不要把minimum_distance的折线路径时长拿来和minimum_time比较，也不能把不同目标函数下的cost直接排序。

## 6. 路网和随机性

- DRM的离线构造基于同一**无新增在线障碍**的canonical机器人场景；不同测试环境只更新在线障碍/查询，不修改模型。
- 先做10个smoke题，再做60个dev题；选择一个DRM规模例如6000节点，3个算法/路网随机种子。
- 正式420题每题3个配对planner/DRM种子，离线roadmap规模固定，全部方法完全配对；另从test按预先固定ID抽80题做路网规模消融（3000/6000/9000/12000）。
- 记录环境seed、起终点seed、DRM seed、算法seed和solver初始化seed，每类独立，不把不同种子隐式重复使用。
- 原论文10×10×4的400个样本组合可作为`paper_reference`轨道，而不是混入本文正式420题。

## 7. 必需算法接口与基线

正式生成benchmark时先只跑**原始未修改**CSDecomp：`DRM -> EI-ZO -> LSCS`用于距离问题；`DRM -> EI-ZO -> SCSTO`用于时间问题。必须验证本地安装了相应开源求解后端且许可证／版本实际可用。

阶段二加入：
- `DRM -> raw path` 诊断（不作为最小时间优化结果）；
- `DRM -> NTO`（Drake KinematicTrajectoryOptimization/Ipopt，公共初始化；原论文强基线）；
- `DRM -> EI-ZO, two paths -> LGCS`（可用时，记录Gurobi许可；不能运行则如实标记缺失，不造等价实现）；
- `EI-ZO + uniform / random refinement`、`EI-ZO + geometry-guided refinement` 与新方法进行等预算局部实验。

所有方法共享模型、端点、碰撞几何、目标定义、阈值、时间限额、种子与独立验证器。分别报告公共前端完成后与完整端到端性能。

## 8. 独立验证与指标

- 每次保存原始路径、简化路径、所有EI-ZO多面体(A,b)、轨迹控制点、各方法子阶段时长、GPU峰值显存、全部失败日志。
- 统一检查端点、关节上下限、节点/边连续碰撞的**数值近似**、曲线细采样碰撞、速度/加速度、端点速度为0。碰撞按数值步长检查，不能自动宣称连续碰撞已被确定性证明。
- 明确`success_validated_numerical`、`timeout`、`invalid_endpoint`、`collision_failed`、`limit_failed`、`solver_failed`、`frontend_failed`、`validation_undetermined`、`infrastructure_error`等状态。算法失败数据不删除。
- **主指标**：完整有效成功率（全部合法任务为分母）、给定DRM成功条件下成功率、端到端时长（含失败/超时处理）、成功案例路径长度或时间代价、最小碰撞距离、区域数/面数、路径修复次数、CUDA内存峰值。
- 必报p50/p90/p95耗时、成功/失败分层统计和配对置信区间；一个场景共享起终点和不同seed时做scene-level cluster bootstrap，不把同环境不同seed当独立环境。
- 时间拆分：offline DRM build（单列）、online collision-map/query、DRM path、EI-ZO inflation、trajectory optimization、collision recovery、validation、cold_e2e & warm_query；一次性GPU上下文初始化和预热分别记录。
- 同等总预算与同等新增碰撞查询／区域数预算都对比；算法之间不能只比较“相同尝试次数”。

## 9. 项目接口与产物

建议在csdecomp仓库或单独实验repo下**新增**`benchmarks/generic_motion_v1/`，不要改写上游原始示例和核心CUDA实现。

```text
benchmarks/generic_motion_v1/
  README.md
  configs/{robot.yaml, generator.yaml, evaluation.yaml}
  generator/{scene.py, endpoints.py, voxelize.py, manifest.py, audit.py}
  runners/{csdecomp_baseline.py, batch.py, validate.py, summarize.py, replay.py}
  tests/{test_determinism.py, test_scene_schema.py, test_model_consistency.py,
         test_endpoints.py, test_collision_adapter.py, test_result_integrity.py}
  manifests/{smoke.jsonl,dev.jsonl,test.jsonl}
  scenes/<scene_id>.json
  roadmaps/<roadmap-id>/ ...
  results/<run_id>/...
```

每题manifest必须包括:`benchmark_version`,`split`,`family`,`scene_id`,`problem_id`,`robot_sha256`,`collision_geometry_sha256`,`obstacle_geometry`,`voxels_xyz`,`voxel_radius_m`,`q_start`,`q_goal`,`straight_line_collision`,`start_goal_joint_distance`,`scene_seed`,`endpoint_seed`,`source_code_commit`,`validation_status`。输出附生成脚本 SHA 与配置 SHA。`scene.json`及MANIFEST的sha应与实际运行所用文件一致。

## 10. 验收门槛

**G0**：模型同一性：DRM和在线测试7个关节、限位、TCP、碰撞几何及所有过滤对明确一致；Drake/CSD边界碰撞抽样比对，差异统计且查明。若不一致不准跑论文基线。

**G1**：10个smoke真实生成、写入、重载、检查、可视化、原始CSDecomp一次完整最短路径/最短时间闭环，包含失败记录。

**G2**：60个dev每题manifest/hash一致，场景生成可复现，不以被测算法成功过滤问题，至少覆盖所有族；失败和不适用输入如实计数。验证器独立于规划器。

**G3**：冻结420个test问题与ID，但不根据测试成绩调参；在正式跑大规模前，先输出场景和起终点分布审计，包括各族障碍数、直连阻塞比例、模型穿透率、起终点距离、地图内存。

**G4**：运行原始EI-ZO及NTO开发基线，确认不同方法使用同一代价与时限。不得因局部失败而更换终点、seed或模型。

## 11. 范围声明

这是一般静态三维障碍环境中的**离线／快速在线无碰撞机械臂运动规划**。原论文假设规划及执行期间障碍不变化；本文benchmark按每个query内障碍静态处理，跨query允许环境变化。没有训练、视觉感知、抓取接触、强化学习、机器人操作语义。将来的创新方向必须独立于场景定义，benchmark不包含任何“对新方法有利”的筛选规则。
