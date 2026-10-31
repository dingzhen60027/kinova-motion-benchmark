# Benchmark v1 准入审计与修复记录

本轮确实发现了会导致错误实验判断的缺陷，不能把之前“工程验收通过”理解为完整正确性保证。现已修复可复现的输入、验证、溯源、预算及汇总缺陷，并完成下面列出的实际检验。当前结论是：**限定在已冻结的 Kinova 静态几何规划协议内，可用于受控开发；尚未批准正式 test 运行或论文总体算法排名。** 这不是零缺陷证明，也不是连续无碰撞或实机安全证明。

机器可读结论见 [qualification_status.json](qualification/qualification_status.json)。审计前 HEAD 为 `7205174f9d6b8e93e610ebbfb2ddbb3d1efc5c6d`；修复和复测保存在原独立分支 `benchmark/generic-motion-v1`，没有改 main 或推送。沿用现有 Ubuntu 22.04 / Conda / CSDecomp / Drake 环境，没有重装环境或更换第三方 CUDA 算法。

## 发现的问题与修复

| 问题 | 实际复现与影响 | 修复及验收 |
|---|---|---|
| 输入审计可能漏题 | 删去整族、删除原始 NPZ，甚至重新封装部分哈希后，旧审计仍可能通过 | 核对完整题单、布局、引用文件、状态、约束、种子派生和原始采样；重放首次接受端点规则 |
| 非法配置可能越限 | 负关节裕度可生成超出真实限位的端点 | 配置类型、有限数、范围和分层约束在采样前拒绝；真实限位独立检查 |
| 验证器假成功 | NaN 固定端点、把折线路径当最短时间曲线、无限端点容差、NaN 多面体等可能绕过检查 | 严格端点/目标/运动限制契约、有限值检查、样条连续性与导数界、碰撞返回值类型和样本步长检查 |
| 球覆盖与数值输入边界 | 极大 double 坐标转 float32 溢出；极大世界坐标可损失局部网格偏移 | 转换前后检查有限性、数组形状；显式约束变换舍入误差。冻结场景米级坐标不受这些极端反例影响 |
| 凭据与汇总未严格关联 | 空 receipt、缺 identity 可被视为完整；汇总可能计入坏哈希结果 | 要求完整身份、文件清单、请求/结果一致性、成功候选和独立 validation 证据；汇总先验证凭据 |
| 工作进程身份可被覆盖 | 返回错题号/错方法名后，协调器旧逻辑会直接填成预期身份 | 先保留错误原件，再标 infrastructure_error；verify 独立核对结果身份 |
| 重复实验及错误统计来源 | 重复方法参数产生相同 job ID；内嵌 validation 可能覆盖独立文件的 cost | 拒绝重复实验轴/配对键；统计使用已封存独立 validation；不以同名本地地图替代实际地图 |
| 截止时间检查可能延迟 | 慢 nvidia-smi 查询会阻塞主轮询，漏过截止时刻 | 主线程独立执行进程期限，显存查询移至旁路；真实子进程超时反例通过；NaN/Inf/负预算拒绝 |
| 执行期间代码改变 | 本轮第一份集成运行实际捕获这一情形 | 保存运行前后源码清单；标记基础设施错误并停止后续任务；改用稳定提交在新目录复测 |
| 证据覆盖风险 | 回放默认曾写入已封存 attempt；audit 输出可指向冻结文件 | 回放派生文件移至 attempt 外，回放先验凭据；CLI 拒绝覆盖冻结输入及验收凭据 |

修复前的反例日志没有删除：[输入反例](qualification/generator_adversarial_before.json)、[验证器反例](qualification/baseline_pre_repair.json)、[生命周期反例](qualification/lifecycle_prefixed_probes.json)、[独立身份/统计复审](qualification/model_lifecycle_readonly_before.json)。反例中的合成任务只是测试夹具，未计入任何 benchmark 成绩。

## 实际通过的检查

完整测试：**238 passed**，见 [最终 pytest 日志](qualification/final_pytest.log)。保留上游 Drake 弃用警告，没有隐去警告或失败过的中间日志。

| 冻结输入 | 记录数 / 布局 ID | 合法 / 已标记无效 | 重放端点候选 | 本次输入审计耗时 |
|---|---:|---:|---:|---:|
| smoke | 10 / 5 | 8 / 2 | 8,768 | 5.584 s |
| dev | 60 / 20 | 60 / 0 | 4,544 | 11.647 s |
| test | 420 / 84 | 395 / 25 | 133,504 | 123.138 s |

490 条记录全部通过**输入及其有效/无效标记的一致性审计**，不表示有 490 道合法题。实际重新计算了 146,816 个端点候选和 61,011 个直线样本的 CUDA / 独立碰撞结果，原始数组观测一致，合法端点穿透为零；场景几何重新生成一致。有效题中直连碰撞分别为 6、47、290 题。[执行凭据](qualification/generator_reaudit_receipt.json)、[详细输入报告](qualification/generator_findings.md)。

跨 split 未发现非 open_control 场景、端点或端点对重复。open_control 的 5 个布局 ID（smoke 1、test 4）共享同一基础物理世界，在统计中属于同一环境簇，不能作为 5 个独立环境。[跨 split 检查](qualification/cross_split_audit.json)。

### 模型与碰撞检查

canonical 模型仍是上游完整 Kinova Gen3，7 个关节、20 个碰撞盒（15 移动/5 固定）、137 个检查对和 53 个排除对。手写 XML/FK 与 Drake 的矩阵元素差最大 `5.552e-16`；CSD 与 Drake 的几何平移差最大 `2.219e-7 m`，旋转矩阵元素差最大 `2.771e-7`。冻结机器人和碰撞几何 hash 未变。

512 个新随机构型、300 个边上构型分别在基础环境和随机障碍环境检查；四类真实 dev 环境另外检查 260 个构型/边样本，未出现碰撞分类分歧。静态接触 108 个探针和球—盒 face/edge/corner 630 个探针中，**实际出现 30 次分歧，包括 14 次 CUDA false-free**。最深 false-free 约 `4.025e-8 m`；全部位于现有 `2e-6 m` 接触带内，带外分歧为零。接触带内不会被本验证器当作成功，但这只是本次采样证据，不是全域浮点误差上界。

另实证上游 CPU `CheckCollisionFreeVoxels` wrapper 会额外加入固定 base 的检查。当前 benchmark 使用 CUDA 和独立 Drake 路径，不调用该 CPU wrapper；不能未经核验把它当作等价回退。第三方源码未改。[完整模型报告与原始探针](qualification/model_audit.md)。

### 历史结果复核

对当前被采用的 smoke/dev/paired 历史运行：8＋198＋24＝**230 个成功候选**全部通过加强后的 CPU 契约及独立 SciPy 曲线检查，新生成的 q/time 样本与已保存数组逐元素相等。这项检查复用原碰撞观测，**没有把它写成新的 230 条碰撞复验**。[逐条结果](qualification/baseline_saved_candidate_reaudit.json)。

历史正式采用运行的哈希、原始数组和凭据通过新完整性检查；12 个超过 60 秒的任务全部已标 timeout，未发现超时却计作成功的情况。更早的 `smoke_preflight_v1` 缺少运行身份文件，不能通过新标准，保留原件并隔离，不补造旧凭据。旧源码缺陷不能反推所有旧结果必错；旧报告也不能替代此次更强检查。[历史完整性](qualification/historical_integrity_final.json)、[截止时间核查](qualification/historical_deadlines.json)。

## 新的真实端到端回归

固定 `dev-bottleneck-000-p01`，同一 `n6000_s000`、端点和碰撞模型，分别执行两方法、两目标。该题直连检测有碰撞，native 路径包含多段；它是有意选择的集成回归题，**不是随机抽样的算法排名实验**。运行计算源码为 `c6649d9`。

| 方法 / 目标 | 实际结果 | 成功代价 | 冷启动墙钟时间 |
|---|---|---:|---:|
| EI-ZO / minimum_distance | success_validated_numerical | 7.308662 rad | 1.817 s |
| EI-ZO / minimum_time | success_validated_numerical | 6.230161 s | 2.219 s |
| NTO / minimum_distance | solver_failed | 不计入成功代价 | 13.136 s |
| NTO / minimum_time | success_validated_numerical | 4.967795 s | 2.767 s |

EI-ZO 实际生成 3 个区域、113 个面。NTO 距离候选的数值净距约 `-0.106766 m`，求解器失败及碰撞候选原样保存，没有改终点或调参使其通过。该四次结果只证明接口、失败分类、预算、证据和终检实际运行，不能据此宣称方法总体优劣。离线 DRM 沿用已有地图，未混入在线延迟。

稳定回归目录为 `results/qualification_integration_v2`，4/4 完整封存并通过 verify。首轮 `qualification_integration_v1` 有 3 个已封存 attempt，其中一次因审计修复落盘改变源码而标 infrastructure_error，剩余任务未执行；原失败与运行中止日志保留。新的 EI-ZO 时间轨迹回放含 2,300 个实际验证样本，静态图已实际查看；本轮没有重复浏览器交互验收。[回放](qualification/integration_replay/replay.html)。

## 仍然不能据此声称什么

1. 这是自建、受控 Kinova 分布，尚未接入 MBM/Panda 外部题库，不是 FR3 精确复现或通用公开标准集。结构族的中心范围仍固定于源码，ROI 配置不统一控制所有族；不能声称全部分布参数都由 JSON 配置。改变这部分须建立明确的新协议，不能改已冻结输入。
2. bottleneck / route_choice 只描述 workspace 几何；端点没有强制位于通道两侧，不能把它们直接解释为已验证的 C-space 瓶颈或不同同伦类。旧四题配对成功子集可直连，不能用来支持复杂绕障排名。
3. 随机题的全局可行性仍 unknown。25 个无效 test 输入须单独统计；不能把生成失败计成规划失败，也不能用方法成功来筛选剩余题。
4. 原生 DRM 的 `closest_node` 断言退出在旧开发运行中确实存在，尚未改动上游算法或证明其根因已消除；它们仍按基础设施失败保留在合法任务分母，不能删掉。新增碰撞查询计数不可得，等查询预算比较仍 unsupported。
5. 有限碰撞采样不能证明连续安全；采样接触带不是严格浮点误差证书。当前 minimum_time 检查速度/加速度与端点速度，不构成力矩约束、控制跟踪或实机执行验证。

因此，此次工程检查通过的范围、已知限制和失败数据必须随以后算法结果一起引用。正式 test 规划继续由 CLI 锁定，本轮仅重审 test 输入，没有用于调参或刷分。

## 冻结证据与可复现命令

原 `test.SHA256SUMS` 不改写。其 518 项中，513 项当前文件逐字节保持；5 个被加强的 Python 源文件已将原版本按原 hash 保存在 `qualification/frozen_v1_source/`。所有原始场景、端点、输入 NPZ、配置、旧报告与生成凭据保持原 hash。直接在新代码树执行旧 sha256sum 会如实指出这 5 个源码差异，应使用下面的显式冻结源核查程序区分“代码修复”和“数据改变”。

以下为实际执行命令，均在仓库根目录，继续使用现有 Conda。重跑实验须换新的 output 目录；已封存目录不会被覆盖。

```bash
bash local/python.sh -m pytest benchmarks.generic_motion_v1/tests -q
bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.model_audit --output benchmarks/generic_motion_v1/reports/qualification/model_audit_current.json
bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.baseline_reaudit
bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.generator_reaudit
bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.cross_split_audit
bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.freeze_integrity --output benchmarks/generic_motion_v1/reports/qualification/freeze_final.json

bash local/python.sh -m benchmarks.generic_motion_v1 run \
  --manifest benchmarks/generic_motion_v1/manifests/dev.jsonl \
  --roadmaps benchmarks/generic_motion_v1/roadmaps/n6000_s000 \
  --problem dev-bottleneck-000-p01 --methods EI_ZO NTO \
  --objectives minimum_distance minimum_time --timeout 60 \
  --output benchmarks/generic_motion_v1/results/qualification_integration_v2
bash local/python.sh -m benchmarks.generic_motion_v1 verify --run benchmarks/generic_motion_v1/results/qualification_integration_v2
bash local/python.sh -m benchmarks.generic_motion_v1 summarize --run benchmarks/generic_motion_v1/results/qualification_integration_v2
bash local/python.sh -m benchmarks.generic_motion_v1 replay \
  --attempt benchmarks/generic_motion_v1/results/qualification_integration_v2/jobs/dev-bottleneck-000-p01-EI_ZO-minimum_time-79019c3f/attempts/0001 \
  --output benchmarks/generic_motion_v1/reports/qualification/integration_replay
bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.finalize_evidence
```

输入审计单独复跑可使用 `audit --manifest …/manifests/dev.jsonl --output /tmp/new_dev_audit.json`，输出到新文件；无需规划器，也不改变任何题目。实际日志、失败尝试、原始数组和校验值均位于 `reports/qualification/` 与上述两个新 results 目录。
