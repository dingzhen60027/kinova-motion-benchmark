# Kinova-7D 通用规划基准：实施与实际验收

本报告对应 `kinova7_generic_v1`。这是 **RSS-inspired Kinova-7D adaptation**，不是原论文 FR3/33 球模型的精确复现。任务只有固定七关节起终点、已知静态三维障碍和独立的距离/时间目标；没有抓取、装配、视觉或任务奖励。

## 1. 来源与环境

- 审计所用上游源码提交（本地 `origin/main`）：`7661a97bb7c4931be73d2bc4f8c45c3094ff476e`；该 Git 引用不证明已安装 wheel 的构建提交，实际运行二进制另以 SHA 冻结。
- 本次开始的本地 main：`5a0647ac69c3be9df8892d69c767de7c18f7c10d`。
- 独立本地分支：`benchmark/generic-motion-v1`。本次未推送、未强推、未改写 main。
- 实现提交 `db86208`；共享预算修正 `c64ff67`；数值交接与输入修复 `a98f44d`；续跑互斥及未封存结果统计 `5dc135f`。每个运行另外冻结实际源文件 SHA 和源码副本，不能仅凭 commit 假定工作树相同。
- 沿用 Ubuntu 22.04.5、项目 `.conda/envs/csdecomp`，Python 3.10.22，CSDecomp 0.1.2，Drake 1.51.1，NumPy 2.2.6，Quadro RTX 8000，驱动 570.211.01。没有重建 Conda、迁移 uv 或修改第三方安装。

完整版本、二进制与 Python 模块 SHA、线程设置见 [environment.json](environment.json)。作者 SCSPlanning `d3f885aa2b74a10418de96e61ec2df0bf6ef16b8` 的当前源码本身使用 Drake Clarabel，安装文件与该源码逐字节一致；本次没有改写求解后端。相对论文 LSCS 的 MOSEK 配置仍须标记 **adapted**。pybezier 发行 metadata 为 0.1.4，模块内部旧 `__version__` 为 0.1.1，两者均记录。进入规划流程的合法 attempt 保存 Clarabel 可用性和小型实际求解探测；无效端点会提前拒绝。NTO 使用实际可用的 Ipopt。详细来源见 [baseline_implementation_audit.md](baseline_implementation_audit.md)。

## 2. Canonical 模型与碰撞核查（G0）

审计了上游 `examples/build_DRM.py`、`kinova_mintime_planning.py`、`test_eizo.py`、`utils.py`、两套 Kinova directives、roadmap builder/planner 和碰撞实现。

`build_DRM.py` 使用 reduced Kinova 与 collapsed Robotiq，在线例子使用 full directives。实际比较发现两者都有 7 个活动关节、20 个碰撞盒、15 个活动碰撞体、137 个检测对；64 个姿态的世界碰撞体/TCP 变换最大差约 `3.58e-7`。512 个随机姿态的碰撞分类一致。因此不能仅凭 URDF 文件名不同宣称物理模型不一致。本基准仍强制所有离线/在线方法统一从 full `kinova_sens_on_table.yml` 经 Drake 转换生成，拒绝身份未知的旧路网。

- 关节顺序 `kinova_joint_1_q` 至 `kinova_joint_7_q`；URDF 名义限位 ±[3.10, 2.24, 3.10, 2.57, 3.10, 2.09, 3.10] rad。
- base 焊接 world identity，右手世界坐标、米；TCP 沿用 `kinova::end_effector_link`，夹爪固定。
- 基座、桌子、两台 tripod、保护墙等固定几何保留。过滤同 link、相邻 link、全固定链及 world/world 的对；准确的检查/排除 pair 身份与尺寸/位姿全部保存在 [robot.json](../configs/robot.json)。
- robot SHA：`a25f447c8caf4aff6dc76196c3d57cc940f00e802367f48034f06a9973927c75`。
- collision geometry SHA：`896ab9f55c5d8081b843358bd85cde2daae1f432564593f86218bf61733ce4fa`。

实际 G0 使用独立 PCG64 seed 861730：无新增障碍与 30 个在线球两种情况各检查 512 个随机姿态和 528 个边采样点，另有 66 个接触邻域探测，分类分歧均为 0。部分接触探测被其他几何对碰撞遮蔽；它们并非 66 个独立的全局可行边界。固定 `2e-6 m` 接触带仍作为不确定区间。NTO SceneGraph 的球/盒距离与独立检查器另做实际 8 姿态比对，差小于 `1e-10 m`。原始数组与统计见 [model_audit](model_audit/audit.json) 和 [模型说明](model_audit_notes.md)。

## 3. 冻结的数据规则（G1/G2/G3）

使用 PCG64 和 SHA256 domain separation。scene、endpoint、roadmap、method、reserved solver 的 seed 字段分开；实际优化器从固定路径确定性初始化，reserved solver seed 标记未使用。全局生成 seed 为 `20261027`。同一 scene 的多对端点/路网 seed 不是独立环境。

世界 ROI 为 `[-0.2,-0.7,0.18]` 至 `[0.8,0.7,1.0] m`；实际基础模型 307 个无碰撞随机姿态的 TCP 范围约 `[-0.399,-0.871,0.073]` 至 `[0.911,0.864,1.212] m`。这是工作空间抽样核查，不是每个 ROI 点可达的证明。

| family | 固定生成规则 |
|---|---|
| sparse_clutter | 6–12 个球/定向盒，球半径 0.03–0.065 m，盒边长 0.055–0.11 m |
| dense_clutter | 20–40 个球，半径 0.025–0.055 m |
| bottleneck | 两个墙体，长 0.30 m、厚 0.065 m、高 0.40 m，间隙 0.22–0.34 m，随机位置/朝向 |
| route_choice | 两簇各 6 个球，簇间距 0.25–0.45 m、扩散尺度 0.065 m |
| open_control | 无新增障碍；多个 ID 共享同一基础世界，统计时合并为一个环境 cluster |

后两族只描述 workspace 几何，不声称已经知道 C-space 同伦类或可行性。场景可拒绝越界/固定世界重叠的几何提案，但不查询被测规划算法。保存真实形状、尺寸、姿态、拒绝数和最终实际 float32 球心。所有方法使用同一半径 `0.04 m` 的保守覆盖球；网格边长不大于 0.04 m，半对角不超过 0.034642 m，另核查量化误差，覆盖不会留缝。

端点直接采样全部 7 个关节，限位裕度 0.02 rad、独立碰撞间隙至少 `1e-4 m`，每题最多 4096 次提案。规范化距离为 `sqrt(mean(((q_goal-q_start)/(upper-lower))**2))`；short/medium/long 区间分别为 [0.1,0.3]、[0.3,0.45]、[0.45,0.75]，按题号循环分层。直连用最大 0.04 rad 步长独立数值检查，仅作属性；不按直连或任一算法成功筛题。耗尽提案的原始任务记录 `invalid_endpoint`，不替换场景。

smoke 共 10 题/5 布局：8 合法，2 个 dense 任务各耗尽 4096 提案，均保留；合法题直连碰撞 6/8。对 dense 原始 27 个真实球的额外 128 姿态诊断也全部碰撞，但不能据此声称全局无解。见 [诊断](smoke_dense_endpoint_diagnosis.json)。

dev 共 60 题/20 布局：每族 15 题、每个距离层 20 题，全部端点合法；直连碰撞 47/60，其中 sparse 14、dense 14、bottleneck 11、route_choice 8。距离 min/median/max 为 2.28335/5.79121/8.98337 rad；规范化为 0.151637/0.401857/0.596564，最小端点间隙 0.000166296 m。实际重生成的 82 个文件 SHA 全部一致，见 [重复生成证据](dev_actual_determinism.json)。smoke/dev 的 scene seed、endpoint seed、非 open 几何和有效端点对均无交集；有效端点穿透与 CSD/独立分类分歧均为 0。

数据文件见 [smoke.jsonl](../manifests/smoke.jsonl)、[dev.jsonl](../manifests/dev.jsonl)，逐题及逐场景核查见 [smoke_input_audit.json](smoke_input_audit.json)、[dev_input_audit.json](dev_input_audit.json)。所有题全局可行性仍标记 `feasibility_unknown`。

### test 冻结完成（G3）

验收 G0/G1/G2 的真实证据生成 [acceptance.json](acceptance.json) 后，test 只生成一次并独立审计，没有执行规划器。**420 题/84 布局 ID 中 395 合法、25 invalid_endpoint 原样保留**：sparse 5、dense 15、route_choice 5；这些任务没有替换场景、端点或 seed。合法题直连碰撞 290/395（73.42%）。84 个 ID 中有 80 个独立随机几何与 4 个相同 open control ID，实际物理几何为 81 种。

实际检查了 133504 个端点候选和 51463 个直线采样点，CSD/独立分类分歧及有效端点穿透均为 0；与 smoke/dev 的非 open 几何、场景/端点 seed、单独端点构型交集均为 0。生成耗时 100.125 s，输入审计 26.966 s，均不是规划耗时。逐族障碍数、体素数、距离/阻塞分布、无效原因及地图字节数见 [test_freeze_audit.md](test_freeze_audit.md)。

[test.jsonl](../manifests/test.jsonl) SHA256：`9fcbe5ccae28d8681586b1ac6c9986f51f7cc7b5cdc19352c101910e3b6ed03b`。配置、manifest、scene、原始样本及生成来源共 518 项校验封存于 [test.SHA256SUMS](../manifests/test.SHA256SUMS)，实际逐项校验全部通过。批量入口硬拒绝 test manifest；本次没有 test 成功率或优化代价分数。

## 4. 离线 DRM 与两种目标

三份 native DRM 使用 6000 个实际采样通过的节点、seed 0/1/2；移除孤立点后分别为 5981/5975/5981 节点、72210/71782/73662 有向邻接项。离线构建分别为 43.2270/43.1166/43.3692 s。每份保存采样提案、通过节点、构建前后 native map 和 SHA。地图中心 [0,0,0.55]、范围 [2.8,2.8,2.2] m，native `offline_voxel_resolution=0.04` 实际含义是半边长，离线格边长 0.08 m，34300 个离线格。离线世界没有新增在线障碍。离线时间完全单列，未计入/扣除任何在线算法耗时。

距离目标使用原生 DRM→EI-ZO→作者 LSCS，代价是 PWL 关节 L2 弧长（rad）。时间目标使用 DRM→EI-ZO→作者 SCSTO，6 次 Bezier、各轴速度 1 rad/s、加速度 1 rad/s²、端点速度为零，代价为秒。没有将两种目标混排。

EI-ZO 使用实际暴露的随机 seed、10000 particles、100 mixing steps、9 bisection steps、0.1 configuration margin，其他参数冻结于 [evaluation.json](../configs/evaluation.json)。NTO 用同一个 DRM 初始化与同一碰撞模型：距离采用控制点与 DRM 路径点对应的 order-2 PWL spline；时间采用端点重复的 cubic spline，并通过保形插结点扩至至少 16 控制点。两种 NTO 目标均使用 150 collocation points、Ipopt 最多 500 iterations。两种方法共享 60 s 外层进程截止时间，包含导入、模型和地图加载、优化及验证。

`warm_query_seconds` 是同一次冷启动执行中初始化后的查询片段，不是另做的一次热启动重复实验。失败/超时不从分母或冷耗时删去；显存为约 0.2 s 间隔的进程 nvidia-smi 采样峰值，不是精确 allocator 最大值。阶段计时包括 native map/query、DRM、EI-ZO、优化、恢复及验证，外部强停可能只保留已完成阶段和当前 phase。

## 5. 实际发现和修复

原生 DRM 输出 float32，而 manifest 固定端点为 float64。保存的失败案例显示约 `1e-7 rad` 的路径锚点舍入可导致端点在半空间外约 `3e-8`，官方凸优化因此报告不可行。已保存原始失败、逐数组 SHA、真实官方 LSCS/SCSTO 的 CPU 诊断，不把诊断求解算作性能结果。

修复保持 q_start/q_goal、碰撞模型和半空间 A 不变，只将 b 逐面补偿到包含原生路径锚点及固定端点。按每行法向量范数归一化的修正量不得超过固定 `1e-6 rad`，超限失败；所有初次/复用/恢复区域都保存修正前后数组及实际补偿。该操作是明确记录的数值适配，最终轨迹仍必须通过独立碰撞和运动约束验证。详见 [数值故障报告](native_solver_failure_diagnosis.md)。

还修复了 ragged/None 输入误分类、manifest 运动约束不一致未拒绝的问题；续跑现有 run 独占非阻塞锁，未封存的中断 attempt 保留原文件并建立新 attempt，已完成的失败也不重试。汇总不会把未封存或待重试任务当作完成。当前新增测试 **93 passed**，见 [真实测试日志](all_tests_final.log)。上游 CUDA、示例、URDF 与历史研究结果没有修改。

## 6. 运行版本与统计口径

| 运行 | 实际范围 | 用途 |
|---|---|---|
| smoke_preflight_v1 | 2 次 EI-ZO，距离/时间均成功 | 首次闭环与实际 replay |
| smoke_v1 | 20 次，6 成功/8 前端失败/2 求解失败/4 无效输入 | 初次验收；期间有 7.5 s 生成重复检查并发，故不作主计时 |
| smoke_v2_clean | 20 次，同上状态数 | 独占 GPU 的修复前复查 |
| dev_eizo_v1 | 360 次，159 成功/138 前端失败/41 求解失败/12 超时/10 infrastructure | 数值修复前诊断，完整保留，不能充当修复后的性能结果 |
| smoke_v3_fixed | 20 次，8 成功/8 前端失败/4 无效输入 | 修复后 smoke；每个目标 4/8 合法题成功 |
| dev_paired_v1 | 只执行 2 次，均前端失败；46 次未运行 | 续跑锁加入前预检，显式 superseded，不计入完整对照 |
| dev_paired_v2 | 48 次；EI-ZO/NTO × 两目标各 6 成功/6 前端失败 | 固定 ID 的开发配对对照 |

配对对照事先按每族第一个固定 problem ID 选择，共 4 个布局、4 个 short 端点对，3 份路网和两个目标；这不是 60 题总体性能排名。成功得到初始化的 12 对 DRM 路径逐元素完全一致，其余 12 对都没有前端路径。实际分段 resume 后最初两条 attempt 文件散列不变，完成后再次 resume 也没有改变任何 attempt。见 [续跑/配对验收](resume_and_pairing_acceptance.json) 与 [配对报告](paired_baseline_audit.md)。

汇总分母是全部声明的合法任务，另外给定 DRM 成功的条件分母；失效输入单列。bootstrap 固定 PCG64 seed、2000 次、按完整 scene cluster 重采样，并保留该场景全部端点/路网重复；成功代价差只在两方法共同成功的配对上计算。p50/p90/p95 同时给出成功和失败层；不同成功子集均值不用于排名。

### 修复后的完整 dev 结果

60 个固定题 × 3 份 roadmap × 2 个目标 = **360 次**，全部完成并封存。198 次通过数值验证、138 次前端失败、12 次超时、10 次 infrastructure_error、2 次求解失败；不存在未运行任务被填成成功。独立核对 8405 个证据文件 SHA。

| 目标 | 成功/全部合法运行 | scene-bootstrap 95% CI | DRM 成功条件下 | 冷 p50/p90/p95（s，含失败） |
|---|---:|---:|---:|---:|
| minimum_distance | 98/180 | 35.56–72.22% | 98/106 | 1.315/1.963/2.853 |
| minimum_time | 100/180 | 36.67–73.89% | 100/106 | 1.517/2.391/2.623 |

完整各族状态、成功/失败冷/热分位数、阶段耗时、实际地图内存、区域/面数/恢复/间隙见 [development_results.md](development_results.md) 和 [机器可读 summary](../results/dev_eizo_v2/summary/summary.json)。479 条区域补偿记录全部在固定上限内，最大 `3.05858e-7 rad`。这些结果来自冻结配置的完整新运行，未与修复前 360 次结果拼接。

计时期间独占规划 GPU；没有为工程验收隔离整个操作系统 CPU/I/O，后台报告写入及一次 Git 维护可能产生负载波动。这些是本机开发观察值，不是正式论文吞吐测量。外部截止后的进程回收小幅开销也保留在实际 wall time 中。

12 份真实回放已经生成，显示帧与原采样逐值对应，receipt 成员不变；实际浏览器核验了播放、暂停、终点定位及关节数值更新，没有浏览器 error 日志。见 [replay_acceptance.json](replay_acceptance.json)。

G0 模型核查、G1 smoke 闭环、G2 开发数据、G3 test 输入冻结均完成；G4 完成限定开发子集上的原生 EI-ZO/NTO 执行和配对检查。等新增碰撞查询预算、隐藏的 shortcut 前路径及正式 test 评测仍未完成，见下文限制，不能宣称整份研究评测已完成。

## 7. 复现命令与证据组织

所有命令从仓库根目录执行，继续使用现有 Conda launcher。已有输出目录不会被覆盖；重新实验需换 `--output`。实际 argv、开始/结束时间、退出码保存在 [cli_commands.jsonl](cli_commands.jsonl)，各运行另有 stdout/stderr、process deadline 和阶段日志。

```bash
# 模型实际核查与测试
bash local/python.sh -m benchmarks.generic_motion_v1.model
bash local/python.sh -m pytest benchmarks/generic_motion_v1/tests -q

# 数据生成；已有目录用新的根目录复查，原始结果不得覆盖
bash local/python.sh -m benchmarks.generic_motion_v1 generate --split smoke --seed 20261027 --output benchmarks/generic_motion_v1
bash local/python.sh -m benchmarks.generic_motion_v1 generate --split dev --seed 20261027 --output benchmarks/generic_motion_v1
bash local/python.sh -m benchmarks.generic_motion_v1 audit --manifest benchmarks/generic_motion_v1/manifests/smoke.jsonl --output benchmarks/generic_motion_v1/reports/smoke_input_audit.json
bash local/python.sh -m benchmarks.generic_motion_v1 audit --manifest benchmarks/generic_motion_v1/manifests/dev.jsonl --output benchmarks/generic_motion_v1/reports/dev_input_audit.json

# 验收之后只生成/审计 test，不运行 test 规划
bash local/python.sh -m benchmarks.generic_motion_v1 generate --split test --seed 20261027 --output benchmarks/generic_motion_v1 --acceptance benchmarks/generic_motion_v1/reports/acceptance.json
bash local/python.sh -m benchmarks.generic_motion_v1 audit --manifest benchmarks/generic_motion_v1/manifests/test.jsonl --output benchmarks/generic_motion_v1/reports/test_input_audit.json
sha256sum -c benchmarks/generic_motion_v1/manifests/test.SHA256SUMS

# 三份离线路网（实际分别运行 seed 0/1/2）
bash local/python.sh -m benchmarks.generic_motion_v1 build-roadmap --nodes 6000 --seed 0 --output benchmarks/generic_motion_v1/roadmaps/n6000_s000
bash local/python.sh -m benchmarks.generic_motion_v1 build-roadmap --nodes 6000 --seed 1 --output benchmarks/generic_motion_v1/roadmaps/n6000_s001
bash local/python.sh -m benchmarks.generic_motion_v1 build-roadmap --nodes 6000 --seed 2 --output benchmarks/generic_motion_v1/roadmaps/n6000_s002

# 修复后的固定 smoke 与 dev 网格
bash local/python.sh -m benchmarks.generic_motion_v1 batch --manifest benchmarks/generic_motion_v1/manifests/smoke.jsonl --roadmaps benchmarks/generic_motion_v1/roadmaps/n6000_s000 --methods EI_ZO --output benchmarks/generic_motion_v1/results/smoke_v3_fixed
bash local/python.sh -m benchmarks.generic_motion_v1 batch --manifest benchmarks/generic_motion_v1/manifests/dev.jsonl --roadmaps benchmarks/generic_motion_v1/roadmaps/n6000_s000 benchmarks/generic_motion_v1/roadmaps/n6000_s001 benchmarks/generic_motion_v1/roadmaps/n6000_s002 --methods EI_ZO --output benchmarks/generic_motion_v1/results/dev_eizo_v2

# 固定 first-per-family 对照，先执行2项，再真实续跑
bash local/python.sh -m benchmarks.generic_motion_v1 batch --manifest benchmarks/generic_motion_v1/manifests/dev.jsonl --roadmaps benchmarks/generic_motion_v1/roadmaps/n6000_s000 benchmarks/generic_motion_v1/roadmaps/n6000_s001 benchmarks/generic_motion_v1/roadmaps/n6000_s002 --methods EI_ZO NTO --selection first-per-family --output benchmarks/generic_motion_v1/results/dev_paired_v2 --prepare-only
bash local/python.sh -m benchmarks.generic_motion_v1 resume --run benchmarks/generic_motion_v1/results/dev_paired_v2 --max-jobs 2
bash local/python.sh -m benchmarks.generic_motion_v1 resume --run benchmarks/generic_motion_v1/results/dev_paired_v2
bash local/python.sh -m benchmarks.generic_motion_v1 summarize --run benchmarks/generic_motion_v1/results/dev_paired_v2
bash local/python.sh -m benchmarks.generic_motion_v1 verify --run benchmarks/generic_motion_v1/results/dev_paired_v2
```

单题入口为 `run --manifest ... --roadmaps ... --problem <problem_id> --output <new_directory>`；回放入口为 `replay --attempt <saved_attempt_directory> --output <new_directory>`。回放只消费已保存的真实数组，不重新规划，也不会把失败画成成功路径。每个 run 保存 `run_manifest.json`、源码快照、输入/路网 SHA；每个 attempt 保存请求、阶段、原始路径、半空间、控制点/持续时间、采样验证、日志及逐文件 SHA receipt。未到达某阶段时不会虚构对应数组。

## 8. 限制与下一阶段

1. 这是 Kinova 的一般规划套件。机器人碰撞盒、场景采样和端点规则与论文 FR3 的球模型不同，不能以这里的成功率声称复现论文表格。
2. 原生 DRM 的内部 shortcut 前路径没有通过现有 Python binding 暴露。保存的是实际返回路径，并明确标记缺项；没有拿返回路径复制冒充 raw-before-shortcut。
3. 原生碰撞查询计数器没有暴露。因此完成的是相同 60 s 墙钟预算的开发对照，**没有完成相同新增碰撞查询预算的比较**。有序区域数、facet 数、恢复次数均可审计，但复用区域也占有序槽位，不能把它们当作唯一膨胀次数或碰撞查询数。后续若新增计数器，应独立版本验证且不改本次分数。
4. 独立验证是有限采样：最大关节步长 0.005 rad，曲线至少 400 点、上限 100000；接触带返回未决。多项式导数控制包络检查速度/加速度，按固定数值容差判断。不能将通过称为数学连续无碰撞证明。
5. 某些合法输入触发上游 `drm_planner.cpp:249` 的 `closest_node >= 0` 断言退出。外层进程完整保存 native signal、stderr、输入及已产物，并分类 infrastructure_error；本任务不改上游 CUDA。路网失败、优化失败、超时均不证明问题无解。
6. NTO 的有限 collocation、样条控制点数和初值构成固定的开发转录，不是全局最优求解证明。当前 48 次配对只有 4 个 short 布局，成功子集两题均直连；后续比较应预先声明包含阻塞问题的 dev ID 网格，不能根据当前成功结果筛选。
7. 目前只有 6000 节点、3 个 seed，未做论文的 3000/6000/9000/12000 规模消融、LGCS 或任何新谱算法；正式 test 评分未获本次授权。
8. 原始历史运行和修复后运行全部保留。修复前故障记录用于解释 bug，主结果只能来自注明版本的完整修复后运行；不同版本不可拼接最优成绩。

下一阶段应先处理原生前端的断言与连通失败、扩大预先声明的困难 dev 配对、补可核查的查询计数与预算协议，再由用户明确授权进入冻结 test 的正式评测。不要重采端点、减小障碍球或基于 test 调参。
