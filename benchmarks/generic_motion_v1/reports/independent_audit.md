# Benchmark v1：只读交叉审计

审计时间：2026-10-27 09:20 UTC 起。审计者此前参与了 canonical model 和 reporting 实现；本轮对规范、冻结输入、另一实现者编写的 runner/batch、已封存原始结果进行只读核对，不将这称为外部独立复现。没有运行 GPU、生成新题、修改模型、修改已封存候选或启动额外规划。

**结论：现有证据支持 G0、G1、G2 通过，但只能按下面的实际范围理解。** 10 个 smoke 输入中 8 个合法、2 个生成失败原样保留；clean smoke 的 20 个方法任务全部结束，其中 6 个数值验证成功。60 个 dev 输入全部通过输入审计。开发完整评测仍在运行，本报告不把中途完成子集的成功率当作完整结果；G3、G4 仍需其对应最终证据。

## 1. 模型与碰撞协议（G0）

上游参考 commit 是 `7661a97bb7c4931be73d2bc4f8c45c3094ff476e`。新 benchmark 的完整 online Kinova 模型统一供离线 DRM、在线 EI-ZO、NTO 和 validator 使用。核对了 `model.py`、上游 bridge、CSD static-pair 构造和碰撞函数。137 个 static pairs 同时包括自碰撞和机器人与固定环境碰撞；不是只检查新增障碍。

- 7 个活动关节，顺序为 joint 1 至 joint 7；20 个碰撞盒、15 个运动几何体、137 对检查及 53 对排除均明确保存。
- CSD 与 Drake 原始碰撞候选集合逐对完全相同。固定 gripper 的关节折叠、邻接链和固定链过滤没有被方法切换。
- reduced/full 源文件不同，但不能据此宣称物理模型不一致：尺寸、限位和过滤对相同，64 个状态的世界变换最大差为 `3.5762786865e-7`，512 个状态的碰撞分类相同。新管线仍统一为 full 构造，并拒绝模型 hash 不一致的 map。
- G0 已保存 512 个随机状态、528 个边采样状态、无新增障碍和 30 个新增球两种条件，以及 66 个边界探针。所有组的 CSD/Drake 分类差异为 0。
- CSD 将固定基座算作 world，因此在线球不与固定基座做 robot-voxel 检查。该行为在所有方法间一致，生成器另行拒绝保守球覆盖与固定基座/桌面/设备交叠的布局。本次 smoke/dev 的这类交叠计数均为 0。
- NTO 使用同一 full model 加注册的在线球。额外对数为 `15 × voxel_count`，并按实际几何名称核对所有 pairs，避免只看数量。

当前模型源码 SHA256 为 `7ecde3366d182fa8efe19091aa26b8b3f7688f79fe28e2f292afba90ab108cfe`，与 G0 receipt 和所有输入协议一致。模型 source、robot 和 collision 三类 hash 分开保存。详见 `model_audit/audit.json`、`model_audit/raw_samples.npz`、`../configs/robot.json`。

## 2. 输入生成、保守覆盖与随机性（G1/G2 输入部分）

读取了 10 个 smoke 和 60 个 dev manifest 行，逐行重新计算 problem hash；核对生成 receipt 列出的 **99 个输入文件** SHA256。合法输入的起终点均在保存的原始 7 维均匀采样 proposals 中找到，未见事后规划器重选端点。所有检查没有发现不一致。

| 项目 | smoke | dev |
|---|---:|---:|
| 题数 | 10 | 60 |
| 场景布局数 | 5 | 20 |
| 合法输入 | 8 | 60 |
| 原样保留的 invalid_endpoint | 2 | 0 |
| 合法输入中直线数值检查碰撞 | 6/8 | 47/60 |
| 固定设备与保守覆盖交叠 | 0 | 0 |
| endpoint seed 重复 | 0 | 0 |
| 最小体素覆盖余量 | 5.6681 mm | 5.3981 mm |

Smoke 与 dev 的 scene seed、endpoint seed、非 open 场景几何和端点均不共享。每个 dev 布局有三对不同端点、五个布局/族，不把共享世界的端点或三个 roadmap seed 当成独立环境。`open_control` 的 scene IDs 明确共享同一个基础物理环境，aggregator 强制合成同一 bootstrap cluster，即使输入误给不同 cluster ID 也不会拆开。

PCG64 的 scene、endpoint、method 和 solver seed 由不同 SHA256 domain 派生；正式 test 还包含独立 split 标识。场景保留条件只有固定设备几何不相交，端点保留条件只有七关节限位/裕量、原生和独立碰撞检查、最小净距与预定规范化距离分层。源码没有调用任一规划方法来决定是否保留场景或端点。

保守体素化不是只采物体表面：box 保留全部 cells，sphere 保留与球相交的全部 cells，每个 cell 的半对角线加实际 float32 中心量化误差不大于 0.04 m 球半径。本次覆盖余量至少 5.398 mm。固定 0.04 m 的球协议未因方法改变。原始 sphere、旋转 box、生成参数、实际球坐标和覆盖证明界均保存。

Bottleneck/route_choice 只是 workspace 描述，manifest 明确 `cspace_topology_known=false`。所有问题保持 `feasibility_unknown`。ROI 由实际 sampled robot/TCP envelope 核验，但这个包围盒检查不能证明每一个障碍位置都可达；报告没有做该更强声明。

Dev 的真实重新生成 receipt `dev_actual_determinism.json` 显示 82 个 scene/raw-array/config/manifest 文件逐字节 SHA 相同。两个 smoke dense-clutter 输入各用完 4096 个端点候选而失败，记录没有被删除或替换；这不是“已证明不存在可行构型”。

## 3. 原生闭环与封存证据（G1）

`smoke_v2_clean` 的 20 个任务全部封存，`batch.verify` 返回 integrity passed、complete=true、errors=[]。状态是：

| 状态 | 任务数 |
|---|---:|
| success_validated_numerical | 6 |
| frontend_failed | 8 |
| solver_failed | 2 |
| invalid_endpoint | 4 |

合法任务分母为 16，所以本轮整体合法输入完成率是 6/16；这不是十道题全成功。四个 bottleneck 任务（两题 × 两目标）全部成功；open_control 中两个成功、两个 solver_failed。Sparse_clutter 和 route_choice 各四个任务都是 frontend_failed。两个 dense-clutter invalid inputs 在两个目标下产生四个 invalid_endpoint 记录。

原始 EI-ZO minimum-distance 和 minimum-time 均有真实成功闭环，并保存 native path、EI-ZO A/b、控制点、验证样本及回放。预检两条轨迹分别有 844 和 1326 个验证样本，回放读取这些数组，未重新规划。3D 图包含完整机器人碰撞盒和固定环境；minimum-distance 的横轴明确不是物理时间。

`smoke_v1` 曾与约 7.5 s 的输入确定性 GPU 复核可能重叠。`smoke_timing_note.json` 在 clean 重跑前明确排除了 v1 主计时并预先规定完整 20 个任务重跑。v1 与 v2 的端点、scene、seed 和方法参数相同，结果状态计数相同；不是挑选成功任务重跑。

## 4. 公平性、计时和验证

在 09:20:10 UTC 的只读检查中，四个 run 共 **402 个计划 job** 的内嵌 case 与原始 manifest 一致，method seed 与各自 roadmap seed 对应的 paired seed 一致。检查当时所有已封存 receipts、source snapshots、roadmap hashes 都通过；worker 保存的 input 与计划输入相同，endpoint_audit 没有改端点。正在执行的 unsealed attempt 没有被读取为终态结果。

新 map 基于无新增在线障碍的相同 canonical model，三个 seed 分别保留 5981、5975、5981 个原生节点，不能写成实际每张图恰好 6000 节点。各图离线总耗时约 43.227、43.117、43.369 s，独立列出；没有重复加入在线 query，也没有把 cache 查询吞进离线建图时间。

当前所有已检查任务采用固定目标、1 rad/s、1 rad/s²、相同数值 checker 和外层 60 s 预算。NTO 的程序构建、求解、验证处于相同外层预算内。局部求解参数和几何容差为显式适配，不等于作者未公开代码的逐位复现。

`cold_e2e_seconds` 主字段来自父进程，包含 import、model、CUDA、map、在线规划、验证和输出。`warm_query_seconds` 是**同一次冷启动 worker 准备完毕之后的在线段**，不是第二次重解，也不是测得常驻服务吞吐。GPU 峰值是约 0.2 s 间隔的进程显存采样，有遗漏瞬时峰值的可能。超时保留真实外层 wall time；watchdog 有轮询和退出清理开销，不能宣称操作系统精确在 60.000 s 完成回收。

所有方法使用同一 CSD + independent 数值终检；接触净距在 ±2e-6 m 内不计稳健成功。已封存成功记录没有 collision、contact-band 样本或假阳性验证状态。曲线采用细采样碰撞检查，速度/加速度用控制点凸包界；没有将这些有限碰撞样本称为连续无碰撞数学证明。

`summary` 分离目标和非法输入，主分母包含合法计划任务的失败/timeout，未完成时显式标记 pending；条件率只在明确 DRM 成功的任务上计算。按 scene cluster bootstrap，不按 seed repetition 独立 bootstrap。方法质量差仅比较同题、同 seed、同 roadmap、同目标的共同成功对；完整 EI-ZO 360 jobs 不能与后来选定的 NTO 24 jobs 直接做横向均值排名。

计划的 paired 开发诊断只有四族第一题 × 三 map × 两目标 × 两方法，即 48 jobs、四个布局。第一题均为 short 距离层，所以即使配对成立，也不能推广成对所有距离分层的总体优劣结论。

## 5. 失败与尚未完成的能力

以下是实际检查到的限制或缺陷，不作性能结论，也不修改冻结 run：

1. **原生 DRM 断言终止。** 已封存 dev 中观察到四个 `infrastructure_error`，发生在 dense_clutter-003 的 p00/p01 两目标，worker exit=-6。日志指出 `drm_planner.cpp:249` 的 `closest_node >= 0` 断言，阶段为 drm_path。父进程正确保留了失败与日志；这些不能写作问题无解，不能删除。未修改上游核心算法。
2. **不完整端点的错误分类。** `run_case` 在检查 `validation_status` 前就对 `[q_start,q_goal]` 做 NumPy float 转换；若 q_start 是七维而 q_goal=None，ragged 转换会抛错并被算作 infrastructure_error。当前两个 invalid smoke 都是 None/None，60 个 dev 都合法，因此本批证据未受影响。后续应先检查状态/逐端点形状，再 stack，保留失败输入。
3. **非封存中断的续跑不支持。** 当前 resume 可跳过已封存任务并继续未启动任务；若主机中断留下无 receipt 的 attempt，则 verify 拒绝继续，且 execute 固定写 0001。报告不能把这称作已支持任意断电恢复。应在后续补丁中保存中断记录并创建新 attempt，不能覆盖遗留目录。
4. **修改动态约束需更严格的协议核对。** CLI evaluation config 可以覆盖速度/加速度，但 runner 未断言其与 manifest 中的固定数组一致。当前实际配置均为 1，因此本次公平性没有失配；未来自定义配置应拒绝冲突或创建显式的新协议。
5. **官方接口没有暴露 shortcut 前路径。** `path.npy` 是原生 `DrmPlanner.Plan` 返回的路径，配置开启 shortcutting；`path_provenance.json` 明确写明内部 raw 路径不可得。没有伪造一个所谓 raw path。
6. **求解器适配。** LSCS/SCSTO 调用安装的作者 SCSPlanning 实现，实际 Clarabel 开源求解探针通过，但它替代商业求解后端；报告必须标 adapted。Kinova 也不能冒称作者 FR3 benchmark 的精确复现。
7. **当前没有等新增碰撞查询/区域数预算的算法对比。** 现有验收是共享外层 wall budget 和冻结输入的开发平台。不能把实现了总预算说成同时完成了 query-budget 等预算研究结论。

## 6. 审计方式与复核入口

本轮实际通过现有 Conda Python 执行了只读 Python 检查：重算 99 个输入文件/70 行 problem hashes，核对所有计划 jobs 的完整 case、seed、模型，逐项复算已封存 receipt/source/map 文件 SHA256，再检查成功记录的终检字段。未调用 Model.cuda 或规划器。当前 clean smoke 的完整性也实际调用了：

```python
from pathlib import Path
from benchmarks.generic_motion_v1.batch import verify
verify(Path('benchmarks/generic_motion_v1/results/smoke_v2_clean'))
# integrity_passed=True, complete=True, completed_jobs=20, planned_jobs=20, errors=[]
```

等价的公开复核入口：

```bash
bash local/python.sh -m benchmarks.generic_motion_v1 verify --run benchmarks/generic_motion_v1/results/smoke_v2_clean
bash local/python.sh -m benchmarks.generic_motion_v1 summarize --run benchmarks/generic_motion_v1/results/smoke_v2_clean
```

本报告是 G0/G1/G2 和协议/证据审计；最终 dev、paired、test freeze 的计数和 SHA 应以各自完成后的 receipt 和主审计报告为准。不要将本轮看到的 dev 已封存子集当作最终成功率。

## 7. a98f44d 修复后的增量审计

本节保留上文当时的缺陷记录，追加对 `a98f44d`（Bound native region rounding and preserve interrupted benchmark attempts）的复核。此轮只读取源码并执行 CPU 测试，没有构造 `Model`、调用 CUDA 或规划器，也没有读取正在执行的未封存候选作为终态结果。

此前第 5 节中的不完整端点分类、固定动力学约束核对和非封存中断续跑，已获得对应修复：

- `_input_endpoints` 先检查输入状态，再检查两个七维有限数组。缺失、ragged 或非有限端点返回 `invalid_endpoint`，不进入模型初始化，保留原始拒绝统计。
- `_assert_motion_limits` 在模型初始化前核对 manifest 的七维速度/加速度数组与实际配置完全一致，并核对端点零速度；不一致明确记录 `input_protocol_mismatch`，不悄悄改变目标。
- `resume` 保留中断目录及旧 result，封存 `coordinator_interrupted` 记录并标记 `retry_pending`，再用相同冻结任务创建新的编号 attempt。已封存的成功、失败和超时不会自动反复重试。`verify` 区分历史 source snapshot 的完整性和继续执行所需的当前源码兼容性；run manifest 的 identity hash 可发现计划被修改。

`cover_region_anchors` 对 EI-ZO 的 float32 半空间到 double 优化器的数值传递作显式、有限修正。各平面所需的 `b` 修正除以法向量二范数，固定上限为 `1e-6` joint-space 单位，不能通过缩放同一平面的表示逃避限制。`A`、机器人、障碍和任务端点均未修改。超过上限明确失败；修正前后 A/b、anchors、各平面修正量及最大残差落盘。精确原始起终点加入第一/末区域的 anchors，复用区域和 recovery 区域走同一核对。碰撞采样终检仍然必须通过，因此没有将此数值修正称为碰撞安全证明。

实际 CPU 测试包括：错误输入在 native/model 初始化前退出、配置与 manifest 冲突被拒绝、真实子进程超时保存、旧中断输出保留并另起 `0002`、修改 manifest/roadmap/封存文件被发现、平面缩放不改变 correction 判据、过大修正拒绝，以及 float32 端点舍入导致原始 LSCS 不可行而有限修正后的作者 LSCS/SCSTO 能保持精确端点求解。执行命令为：

```bash
bash local/python.sh -m pytest -q \
  benchmarks/generic_motion_v1/tests/test_batch.py \
  benchmarks/generic_motion_v1/tests/test_baseline_input_contract.py \
  benchmarks/generic_motion_v1/tests/test_baseline_region_coverage.py \
  benchmarks/generic_motion_v1/tests/test_reporting.py \
  -k 'not replay_preserves_failed_samples'
```

实际结果为 **54 passed, 1 deselected in 0.86 s**。排除的 replay 测试会构造 CPU `Model`，本轮按协调要求未执行；不能据此宣称重新验收了回放或 GPU 路径。CSV 汇总导出的换行已固定为 LF，并通过字节检查回归测试；这只影响派生汇总文件，没有改封存数组、运行源码快照或实验协议。仅 reporting 测试随后复测为 **7 passed, 1 deselected in 0.07 s**。

本轮另外识别了两个操作边界，并已向主协调者报告：

1. **同一 run 的并发 resume 尚缺独占锁。** 当前代码会把任何无 receipt 的 attempt 视作中断；如果第二个 coordinator 在第一个仍运行时进入，它可能封存正在写入的目录并启动第二个 attempt。应增加 run-level 独占锁或同等有效的存活进程防护。现有单 coordinator 串行执行不触发这个条件，但不能宣称任意并发 resume 安全。
2. **派生汇总未识别待重试中断。** 用明确标记为 synthetic 的临时 fixture 实测：`receipt.retry_pending=true`、result 为 `coordinator_interrupted` 时，`summarize` 当时仍返回 `complete=true` 和 `finished_jobs=1`；而 `batch.verify` 已正确视其为未完成。尚未启动的重试与未封存结果不应进入最终成功/失败统计。此检查不属于 benchmark 成功证据，未改真实结果。

运行仍在进行，修复后的 GPU smoke/dev 成功计数应由它们各自完成的 receipt 给出；本节没有用 CPU 回归测试替代端到端验收。

## 8. 续跑锁与汇总终态的后续修复

主协调者在全部 GPU worker 退出后授权修复上述两个边界；本轮没有与正在执行的 worker 同时改运行源码。

`resume` 现先对 run 根目录的 `.resume.lock` 获取非阻塞 `flock` 独占锁，锁覆盖完整 verify、遗留中断封存和任务执行过程。已有 coordinator 占用时立即明确失败，尚未触碰 attempt；正常退出或进程退出后操作系统释放锁。该文件不加入任何 attempt receipt，且保留文件 inode，避免删除后另建同名文件绕过旧锁。这保证同一 run 的公共 resume 入口互斥；它不是不同 run 之间的全局 GPU 调度锁，后者仍由主协调者串行安排。

`summarize` 现只把有 receipt 且非 `retry_pending` 的结果视作终态。无 receipt 的 attempt 即使已写 success result，仍显示 `running`；待重试中断显示 `retry_pending`，不计入 finished jobs、成功 cost 或已完成时间比较。CSV 另保留 `last_attempt_status`、`attempt_sealed` 和 `retry_pending` 供追溯。最新 attempt 的选择与 batch 使用同一四位数字目录约定，旁边的任意报告目录不能遮蔽实际结果。新 `0002` 完成后仍只计一个计划任务，不重复计数旧中断。

新增纯 CPU 回归覆盖：持锁时第二次 resume 立即失败且旧文件逐字节不变；释放锁后恢复；待重试 receipt 不计完成；未封存 success 不计成功或配对时间；新编号终态正确替换旧中断；无关目录不干扰最新 attempt。使用第 7 节相同命令，实际结果为 **58 passed, 1 deselected in 0.89 s**。未执行 Model/GPU 测试，未改已封存真实候选。新运行需要重新冻结包含锁修复的 batch 源码；原运行 source snapshot 和失败记录保留。
