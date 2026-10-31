# 按论文补全复现：结果与未完成项

**结论：公开方法和参数的实现进一步补全，但完整原论文复现仍未完成。** 特别是现有 FR3 33 球拟合存在已实测证实的过包络伪障碍，不能代表作者原始模型。本轮所有运行均按冻结配置执行；FR3 数值作为明确标注的重建模型诊断，不作为论文结论的复现证据。

本报告对应 `paper_completion_v4`。旧 v1–v3 源码、输入、轨迹、报告和失败记录保持不变。新运行在同一 Conda 前缀、同一 `main` 工作树上进行，无其他研究项目改动。

## 从论文和原始源文件恢复的内容

- 论文 V-B 的在线顺序：原生碰撞地图、碰撞集、从 pose map 筛选近目标无碰撞配置、真实位姿 IK、原生 DRM。目标位姿固定，不因规划失败重新抽题。Forest 现在也真实构建离线体素碰撞地图，第二路线同样调用原生 DRM。
- 论文 VI-C：第二路线无路时停止扩展，保留第一路线。VI-B：按反例修改区域并核验整条原始路径覆盖。float32 半空间转换采用有误差界的向外舍入，原生包含检查通过后才继续；碰撞阈值未放宽。
- EI-ZO 正式表 III/IV/V 参数优先；原 arXiv TeX 注释补出 `max_iterations=30`。正文可支持 `tau=0.5`。源码注释中旧体素值与正式表格冲突，使用正式的 0.06 m。论文 IV-C 明确说明设置最大迭代次数可能使概率保证失效；本轮不能声称所有返回区域都取得 δ/ε 概率证书，最终使用保存的数值碰撞检查判定。
- NTO 保留 DRM 节点，取消至少八控制点的弧长重采样。距离使用二阶样条，时间使用四阶样条；首尾各重复一次用于静止端点，是公开声明的推断。原 TeX 注释给出内部 k/Nc 配点，共 Nc−1 个绑定；它与正式正文的 Nc 个点存在计数歧义，尚不能认定作者实际运行代码已恢复。
- LGCS 实现引用文献 Eq. 5.5 的透视流模型在分裂图上的严格代数投影，取消 big-M。`NonConvex=0` 保持凸模型；Gurobi 13 需 `Presolve=0` 避免实际测试中出现的 SOC 识别失败，这也是性能可比性差异。LSCS 仍用 Clarabel 替代未许可的 MOSEK。PDF 式 (19d) 印为每对相邻路径节点相等，结合不同的起终点会使问题不可行；实现采用正文所述连续 PWL SOCP 的解释，此项已列为待作者确认的排版歧义。
- 新模型使用论文前 2025-04-11 官方关节范围；2026 版本扩大了六个关节范围。运动学、八个手臂网格及手部网格经逐字节核对未变。球中心、半径和过滤仍是公开几何重建，绝非作者原始资产。

逐项来源、数值和优先级见 [参数追踪](parameter_trace.csv)、[源文件审计](source_audit.md)、[优化转录审计](optimization_audit.md) 与 [流水线审计](pipeline_audit.md)。

## 实际完成的运行

| 环境 | 预定题目 | 原生 DRM 成功 | 独立有效种子 | 运行状态 |
|---|---:|---:|---:|---|
| FOREST | 400 | 400 | 388 | completed |
| FR3 | 400 | 241 | 240 | completed |

400 题来自 10 个环境种子 × 10 个路线图种子 × 4 个规模。“运行完成”只表示全部预定题目已处理，包含规划失败、许可证失败和求解失败，不等于论文复现完成。

| 环境 | 方法 | 实际尝试 | 求解成功 | CUDA 检查通过 | 求解成功且 CUDA 通过 | 独立严格通过 / 预定题目 | TO+CS 均值 ± SD (ms) |
|---|---|---:|---:|---:|---:|---:|---:|
| FOREST | LSCS | 388 | 388 | 388 | 388 | 388 / 400 | 2.954 ± 1.366 |
| FOREST | LGCS | 388 | 388 | 388 | 388 | 385 / 400 | 62.055 ± 124.676 |
| FOREST | SCSTO | 388 | 387 | 387 | 387 | 387 / 400 | 45.435 ± 3.498 |
| FOREST | NTO_distance | 400 | 390 | 120 | 120 | 120 / 400 | 2.726 ± 3.218 |
| FOREST | NTO_time | 400 | 316 | 231 | 231 | 231 / 400 | 24.024 ± 165.826 |
| FR3 | LSCS | 240 | 240 | 240 | 240 | 240 / 400 | 66.387 ± 58.431 |
| FR3 | LGCS | 240 | 208 | 208 | 208 | 208 / 400 | 221.899 ± 329.767 |
| FR3 | SCSTO | 240 | 240 | 240 | 240 | 240 / 400 | 110.753 ± 67.690 |
| FR3 | NTO_distance | 241 | 195 | 75 | 74 | 74 / 400 | 953.721 ± 3819.108 |
| FR3 | NTO_time | 241 | 220 | 170 | 170 | 170 / 400 | 3845.326 ± 9840.791 |

论文表 I 的 TO SR/CFR 以原生 DRM 成功为条件；上表保留全 400 分母。对应条件率、求解成功样本代价、独立验证样本代价和论文参考数值见 [paper_vs_actual.csv](paper_vs_actual.csv)。原 TeX 表格注释明确提到 NTO 求解失败仍可能得到无碰撞轨迹，因此 CUDA CFR 不与求解成功作合取；两者合取另列。没有保存候选或检查的题目保留在原生 DRM 分母中，不算通过。CUDA 检查是本机发布版本的实际谓词，不代表已经恢复论文二进制。独立验证还检查端点、关节域、速度/加速度和几何；不将微小穿透改判成功。

计时为保存的构建和优化时间，包括失败尝试；最终候选独立验证、离线建图、IK、预热和外围预解/回放写入不在正常 TO+CS 计时内。备用地图序列化/加载、备用路径核验和恢复覆盖记录的写入实际发生在构建计时内，不能声称全部 IO 已被扣除。异常的墙钟回退单独标记；超时/进程中断且已开始的方法计入尝试数，但没有完整计时的尝试不进入时间均值。方法尝试数不同，不用边际均值比值声称加速。成对代价要求双方均独立通过，成对时间要求双方有已记录的计时。SD 使用 ddof=1；统计按环境聚类，400 题不视为独立重复。

## 反例恢复触发率

| 环境 | 方法 | 触发次数 | 实际尝试 | 给定尝试的触发率 | 给定原生 DRM 成功的触发率 |
|---|---|---:|---:|---:|---:|
| FOREST | LSCS | 19 | 388 | 0.049 | 0.048 |
| FOREST | LGCS | 15 | 388 | 0.039 | 0.037 |
| FOREST | SCSTO | 1 | 388 | 0.003 | 0.003 |
| FR3 | LSCS | 19 | 240 | 0.079 | 0.079 |
| FR3 | LGCS | 8 | 240 | 0.033 | 0.033 |
| FR3 | SCSTO | 0 | 240 | 0.000 | 0.000 |

论文 VII-A5 报告 Forest 距离/时间恢复率 5.6%/0.46%，Franka 为 40.22%/21.36%。正文未明确距离问题中 LSCS/LGCS 的池化和精确分母，因此这里按方法报告真实计数及不同条件率，不宣称数值具有相同分母。触发是保存的候选产生碰撞并需要修复，包含随后修复异常的尝试；完成的修改迭代数另记在 recovery_rates.csv。

## 失败和限制

**FOREST**：原始题目状态 `{'completed': 400}`。

- LSCS：`{'success_numerical': 388, 'invalid_seed_path': 12}`。
- LGCS：`{'success_numerical': 385, 'invalid_seed_path': 12, 'validation_failed': 3}`。
- SCSTO：`{'success_numerical': 387, 'invalid_seed_path': 12, 'error': 1}`。
- NTO_distance：`{'validation_failed': 270, 'success_numerical': 120, 'optimization_failed': 10}`。
- NTO_time：`{'optimization_failed': 84, 'validation_failed': 85, 'success_numerical': 231}`。

**FR3**：原始题目状态 `{'drm_failed': 157, 'ik_failed': 2, 'completed': 241}`。

- LSCS：`{'not_run_due_to_drm_failed': 157, 'not_run_due_to_ik_failed': 2, 'success_numerical': 240, 'invalid_seed_path': 1}`。
- LGCS：`{'not_run_due_to_drm_failed': 157, 'not_run_due_to_ik_failed': 2, 'blocked_license': 32, 'success_numerical': 208, 'invalid_seed_path': 1}`。
- SCSTO：`{'not_run_due_to_drm_failed': 157, 'not_run_due_to_ik_failed': 2, 'success_numerical': 240, 'invalid_seed_path': 1}`。
- NTO_distance：`{'not_run_due_to_drm_failed': 157, 'not_run_due_to_ik_failed': 2, 'validation_failed': 121, 'optimization_failed': 46, 'success_numerical': 74}`。
- NTO_time：`{'not_run_due_to_drm_failed': 157, 'not_run_due_to_ik_failed': 2, 'success_numerical': 170, 'optimization_failed': 21, 'validation_failed': 50}`。

本地流程增加了严格种子核验门槛：原生 0.1 步长 DRM 返回的路径若未通过独立检查，LSCS、LGCS、SCSTO 记录为 invalid_seed_path 并跳过膨胀和求解；NTO 仍使用该路径作为初值。这是当前重建的输入核验选择，不是已恢复的作者接受规则。所有这类原生 DRM 成功题目仍保留在条件率分母中，因此方法条件率不能直接视为论文精确复现。

Forest 的 e03_r03_n00400 / SCSTO 异常来自作者 `fixed_velocities.solve` 中的 `assert result.is_success()`，实际求解器未成功；原 traceback 和求解前区域均已保存。

两个预选旧适配失败已单独真实回归通过，旧 e08 边界目标的 float32 穿透仍按失败保留。旧 r01_n03000 地图在加入在线障碍之前就有 1512/1488 两个分量，在线 IK 不能保证改变这一事实。有限路线图断连不证明连续配置空间断连。

**球模型问题已有独立几何证据。** 常见 home 下，拟合 link5–hand 球对穿透 −35.1577 mm，而同一运动学下官方两体碰撞网格的凸包距离 +37.0481 mm。凸包包含原网格，其正间距证明这一几何对的球碰撞是伪障碍。固定其他关节的 31 点 q7 扫描中，22 点发生球碰撞，但官方两体凸包全部分离。这是特定几何对的诊断，不是整机连续安全证明。没有通过增加碰撞过滤来隐藏这些失败。

实际 FR3 为 33 个球、32 个活动球，179 个机器人内部候选对加 32 个桌面候选对，静态共 211 对；动态障碍有 640 对，总共 851 对。论文报告 1187 对，并将其中 527 对称为自碰撞；其余 660 对对应 33 球×20 障碍。桌面计数与过滤细节仍不完整。原球几何/过滤缺失，单凭总数无法恢复，不能为了凑数添加无意义检查。

原公开硬件部件组合也做了实际探针：作者 iiwa 8 盒加 WSG 3 盒可组装成 7DOF 的 11 盒模型，两个场景各 2000 个随机配置的 CUDA/Drake 核对一致；安装坐标和桌子是声明的重建选择。这不等于论文原 105 对模型，更不是三台 D415、真实 KUKA 和 15 条实际执行记录。先前安装坐标错误导致全部碰撞的尝试也保留。见 [硬件部件探针](hardware_probe_summary.json)。

还缺作者原始 FR3 球模型、过滤、随机任务输入及完整 NTO 转录；MOSEK 未许可，Gurobi 的含二次项免费模型受 200 变量限制；当前 csdecomp 0.1.2 二进制晚于论文，CPU/GPU/CUDA 版本也不同。不能声称复现了 17.1 倍加速、27.9% 可靠性提升或 823.4 ms 实机延迟。

## 附录吞吐量与历史核心

每次批量为 2,000,000 个配置，预热独立于五次同步计时。Forest/FR3 各有十个环境；公开 iiwa 只作一组已声明模型诊断。

| 模型 | 实际批次 | Mconfig/s 均值 ± SD | 独立核对点 | 不一致点 |
|---|---:|---:|---:|---:|
| forest | 50 | 23.489 ± 0.253 | 200000 | 0 |
| fr3 | 50 | 7.574 ± 0.066 | 200000 | 0 |
| public_iiwa | 5 | 11.637 ± 0.184 | 2000 | 0 |

吞吐计时包含公开同步包装器的分配与主机/设备传输；随机采样和文件写入不计。实际数组、碰撞标志、计时和源码身份可从 [吞吐原始索引](throughput_index.json) 定位。模型对数、设备和软件均与原论文不同，尤其公开 iiwa 组装不是原 105 对模型，不计算论文速度比。

论文前公开核心已生成编译产物；执行记录状态为 `passed`。是否通过各项功能探针须分别查看 [历史核心执行结果](paperdate_core_result.json)，不能由编译成功推断。

本次三项探针均实际通过：CUDA 与解析几何核对 3000 点、与 Drake 核对 300 点，差异均为 0；原 DRM 构建 300 节点地图并求得 3 节点路径；EI-ZO 返回 5 面集合并包含种子，2000 个独立样本无碰撞。原 179 文件和已安装 wheel 的前后校验一致。这些是构造场景的小型功能验证，不是原 800 题，也不构成连续空间安全证书。

构建说明见 [paperdate_build_readiness.md](paperdate_build_readiness.md)。隔离构建未替换主实验的 csdecomp 0.1.2。原核心缺少新版 GPU recovery 接口，即使核心构建和探针通过，也不等于本轮 800 题已改用原始二进制。

## 许可证阻塞的独立补充诊断

对主实验全部 32 道 `blocked_license` 题，使用保存的原区域图枚举简单路线，实际完成 2898 次走廊 SOCP；31 题在保存区域图上建立数值最优性，24 个最终候选通过独立几何验证，8 个未通过，0 题超时。同时满足数值最优且独立验证通过的为 23 题。仍有未决走廊的题目为 `['e04_r07_n12000']`，不声明其最优性。

这是避免专有许可证阻塞的补充数学诊断，并非论文的 Gurobi 实现。该诊断使用独立 Drake 几何检查，未调用 CUDA，也没有执行新的反例恢复；保存区域中的最优路径仍可能碰撞。主表保留原许可证失败；枚举耗时不纳入 Gurobi 速度比较。详见 [实际枚举结果](lgcs_enumeration_results.json) 和 [诊断说明](lgcs_enumeration_diagnostic.md)。

## 验证、命令与原始文件

- FOREST：独立重查 33371 个预定回放点；审计问题 `[]`。原始完整轨迹标志中存在 96 个 CUDA/独立几何判定不一致点，其最大绝对几何间距为 1.62e-07 m；这些边界差异按原严格判据保留，未调整阈值来增加成功数。
- FR3：独立重查 19890 个预定回放点；审计问题 `[]`。原始完整轨迹标志中存在 163 个 CUDA/独立几何判定不一致点，其最大绝对几何间距为 1.14e-07 m；这些边界差异按原严格判据保留，未调整阈值来增加成功数。

所有实际命令、退出码与日志在本目录 `*_commands.json` 和 `verification_commands.json`；环境清单沿用 [environment.json](../../environment.json)、[Conda 锁](../../conda-explicit.txt) 和 [Python 锁](../../requirements.lock.txt)。最终全套 pytest 实际通过 26 项（1.76 秒，61 条第三方弃用警告），其中包含 8 项吞吐记录器故障注入测试；主实验前的 18 项检查记录也保留。另有 8 项独立 CPU 分析审查通过，见 [分析审计](analysis_audit.md)。记录器的单独验证见 [记录器验证](throughput_recording_verification.json)。其中合成元数据仅测试统计记账，不进入主实验；首次失败日志也保留。

```bash
bash local/python.sh -m pytest -q reproduction/tests
bash local/python.sh -m reproduction.run_completion
bash local/python.sh -m reproduction.paper_completion.run resume --run-dir <RUN_DIR>
bash local/python.sh -m reproduction.paper_completion.analyze --run-dir <FOREST_RUN> <FR3_RUN> --output reproduction/reports/paper_completion --audit --collision-depths
bash local/python.sh -m reproduction.audit_paper_completion --run-dir <FOREST_RUN> <FR3_RUN> --output reproduction/reports/paper_completion/audit --resume
```

每个 run 保存源码快照、配置、完整地图、在线 IK 全部尝试、原生种子、每次求解前的区域/种子、优化系数、回放和失败。终态失败与成功都由续跑跳过；源码或依赖漂移会拒绝续跑。

## 达到完整复现还需要什么

1. 作者的 FR3 33 球中心/半径、安装变换、碰撞过滤和桌面模型；这决定是否在求解同一个几何问题。
2. 原始题目生成/种子、NTO 样条和求解器选项、验证阈值及计时脚本，消除当前可证实的歧义。
3. 相应求解器许可、作者完整运行代码和匹配的软件版本；本轮已完成历史公开核心的小型验证，完整流水线仍不等价。硬件时延需要匹配硬件才能直接比较。
4. KUKA 与三台 D415 的标定、原始感知和执行记录，以及可访问的真实设备，才能重做表 II 的实机实验。

以上缺项不会用合成数据或“近似成功率”补齐。资产请求草稿已保存在 [author_asset_request.md](author_asset_request.md)，尚未发送。
