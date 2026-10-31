# 研究平台实现与实测验收报告

日期：2026-10-26。范围：`csdecomp` 的可持续实验平台与可靠性修复。

本轮交付了可直接使用的统一配置、原生模型与基线适配、单题/批量执行、续跑、显式重试、证据核验、统计和轨迹回放。最终 **149 项回归测试全部通过**；实际执行了 5 个任务、30 个方法任务，另对一次数值失败做了显式重试。三份最终运行的任务清单均已完整执行，内容哈希核验通过。测试和这些小规模验收不用于算法优劣或论文性能结论。

历史代码、来源、模型审计、报告、原始结果和失败记录均保留。新增实现位于 `research_platform/`；只有 `main` 分支，本轮起始 HEAD 为 `9b869cb335d2077816bbf0b0fa36bc6c49ccbcf5`。运行发生在提交前工作树，manifest 如实记录 `git_dirty=true`；精确代码由每次运行的 `source_snapshot` 和 SHA-256 绑定，不能把起始 HEAD 当作新实现的完整代码版本。

## 使用入口

- [主 README](../../README.md)：环境安装、快速运行、续跑、验证和仓库结构。
- [实验协议](../../docs/research_protocol.md)：分母、成功标准、目标单位、预算、配对和证据解释。
- [扩展指南](../README.md)：实际算法接口、新 baseline 接入和配置字段。
- [最终命令清单](acceptance/commands.json)：21 条实际执行的 CLI 命令、退出码和实测耗时。
- [整体验收结果](acceptance/checks.json)、[Forest 报告](acceptance/forest/summary.html)、[FR3 报告](acceptance/fr3/summary.html)、[批量报告](acceptance/batch/summary.html)。

```bash
bash local/setup_research.sh
bash local/research.sh doctor --probe
bash local/python.sh -m pytest tests/research_platform -q
bash local/research.sh run --config configs/research/forest_smoke.json --output runs/my_first_study
bash local/research.sh verify --run runs/my_first_study
bash local/research.sh replay --run runs/my_first_study --case e000_r000_n00200
```

完整小规模验收流程也可直接复跑，`--label` 必须使用新名字：

```bash
bash local/python.sh -m research_platform.reports.acceptance_driver --label my_acceptance
```

## 环境核查

复用了原项目 Conda 环境 `.conda/envs/csdecomp`，没有新建或修改其他项目环境。`setup_research.sh` 的现有环境分支实际执行成功，`pip check` 无冲突。首次创建环境和 `--repair` 分支本轮未执行，不把它们记为已验收。

| 项目 | 实测值 |
| --- | --- |
| 系统 | Ubuntu 22.04.5 LTS，x86_64 |
| CPU | Intel Core i7-13700K，24 个逻辑 CPU |
| GPU | Quadro RTX 8000，49,152 MiB，驱动 570.211.01 |
| Python / csdecomp / Drake | 3.10.22 / 0.1.2 / 1.51.1 |
| NumPy / SciPy / CVXPY | 2.2.6 / 1.15.3 / 1.7.5 |
| Clarabel / gurobipy | 0.11.1 / 13.0.3 |
| 线程设置 | OPENBLAS=1、MKL=1、默认 OMP=4；每次运行保存实际值 |

完整实测清单在 [environment/](environment/)，包含 Conda explicit、pip freeze、CPU/GPU/OS 及采集命令。锁文件沿用 `reproduction/conda-explicit.txt` 和 `reproduction/requirements.lock.txt`，无需在多个位置维护互相偏离的版本。

[doctor_final.json](doctor_final.json) 的实际 CUDA 检测结果为 `[true, true, false]`，与两个可行端点及一个障碍中心的预期相符。Clarabel、SNOPT、IPOPT 和直接 `gurobipy` 小问题均求解成功。Drake 的 Gurobi adapter 不可用，LGCS 实际使用直接 `gurobipy`，两者分开记录。Gurobi 当前为受限许可证，小问题通过不表示大模型也获授权。

## 已修复的问题及证据

| 问题 | 修复与实际检验 |
| --- | --- |
| 只有分散脚本，环境入口容易漏装优化依赖 | 新增统一 CLI；使用完整的现有 Conda/pip 锁与依赖核查，提交必要 launcher |
| 配置错字、重复键/种子、NaN、错误边界可能被忽略 | 严格 schema、合并后的场景几何检查、明确默认值与解析后哈希；80 项配置测试 |
| 修改配置或代码后仍可能混用旧结果 | 冻结配置、源码、模型、数值二进制和 GPU 身份；不兼容续跑明确拒绝 |
| 重试覆盖日志、遗漏任务被当作已完成 | 每次独立 attempt、全部历史收据检查、按完整 job graph 判断完成 |
| 已封存的最新中断也曾误报 complete | 新增 `interrupted_jobs`；最新中断阻止 complete，后续成功重试解除阻塞并保留旧记录 |
| 上游重跑后下游结果可能仍被汇总 | 记录并递归验证依赖 receipt；变化的结果变为 stale，需重新运行 |
| 父进程先退出导致孙进程逃过超时清理 | 按整个进程组清理，即使 leader 已退出；SIGTERM 后最多 2 秒宽限，再 SIGKILL |
| 正常父进程退出或写 process.json 失败后泄漏子进程 | 所有 Popen 后操作纳入 finally；实际睡眠进程树回归通过 |
| 外部 SIGTERM 不执行清理 | 在执行边界转换为可清理的中断，恢复原 signal handler，并封存 interrupted 及原因 |
| LGCS 忽略配置的时间上限，且可受松弛可行性容差影响 | 局部适配传入 TimeLimit，收紧 feasibility/integrality 至 1e-9，模型 finally 释放；真实 Gurobi 测试 |
| SCSTO 初始化丢掉路径折点，可能离开走廊 | 保留所有原始折点，显式标注 numerical_variant；真实优化测试与 Forest/FR3 验收 |
| 修改 Forest domain 后 native map 仍固定为 10m | 根据实际 domain 计算离线 map 的中心和尺寸；矩形/偏移边界测试 |
| 过量采样或畸形候选导致异常/资源耗尽 | 候选维度、有限值、时长、结点检查；超过 200,000 验证点显式拒绝，不降精度冒充通过 |
| 路径长度或重试统计的命名会误导 | 二阶 NTO 使用精确折线长度；时间曲线长度标明 sampled；汇总字段改为 latest_attempts 并保留全部 history |
| 只比较幸存方法/混合不同输入与目标 | 全 planned denominator；严格成功三条件；配对要求同 case、input hash、目标和单位 |

最终全套测试记录为 `platform_tests_final_v4.log`：**149 passed in 10.03s**。其中包含真实小规模数值求解和真实 CPU 进程树，不把它们说成 GPU 基准测试。GPU 另由下面的实测任务检验。

修复前进程测试原临时目录 `pytest-197` 被 pytest 自动轮转删除，未伪称已归档原目录。使用首次真实运行冻结的旧源码重放后，实际得到 **3 failed / 1 passed**；另保存了 SIGTERM 遗留孙进程的修复前现场及修复后测试。见 [故障证据说明](process_failure_before_fix/README.md)。重放产生的未冻结 Python 缓存已精确清理，旧实验 identity 再次核验通过。失败重放、最终测试日志及原始文件均包含在证据包中。

## 原生模型与基线实测

最终运行目录为 `runs/platform_final_v4_{forest,fr3,batch}`。所有方法使用相同的场景、端点与已独立验证的 DRM 种子；FR3 在线 IK 每题只解析一次。默认 lazy edge step 为 0.01，独立验证步长为 0.002。每个独立 job 外部预算 120 秒，适用求解器每次调用预算 30 秒；LSCS/SCSTO 的 Clarabel 子问题没有单独的 30 秒内部限制，受整个 job 的外部预算约束。

| 方法 | Forest Figure 6（1 题） | FR3 public_table（1 题） | Forest random（3 题） |
| --- | --- | --- | --- |
| DRM | 1/1 通过 | 1/1 通过 | 3/3 通过 |
| LSCS | 1/1 通过 | 1/1 通过 | 3/3 通过 |
| LGCS | 1/1 通过 | 1/1 通过 | 3/3 通过 |
| SCSTO | 1/1 通过 | 1/1 通过 | 3/3 通过 |
| NTO_distance | 0/1，validation_failed | 1/1 通过 | 1/3，其余 validation_failed |
| NTO_time | 0/1，solver_failed | 1/1 通过 | 1/3，其余 validation_failed |

共 24/30 个方法任务严格成功，6 个失败。此处仅检验平台执行和记账是否正确；不能据此估计总体成功率或算法排序。Forest Figure 6 的 NTO_distance 显式重试仍为 validation_failed，两个 attempt 均保存。统计采用预先声明的最新 attempt，不择优选结果。

作为结果可核对示例，Figure 6 的距离目标 DRM/LSCS/LGCS 分别为 11.297010、11.174887、10.883124 m；SCSTO 的时间目标为 11.504054 s。FR3 单题的四个距离方法均为 5.219869 rad，SCSTO/NTO_time 为 4.985625/9.956917 s。这是一道较容易的 FR3 任务，不足以证明复杂障碍场景中的性能。

详细 cost、时间、失败原因和配对结果在各报告 CSV/JSON 中。DRM method 复用公共 plan，method 内 optimizer 时间为零；实际 DRM 搜索在 shared preparation 中，比较总时延必须按协议合计，不能直接把表里的零解释为免费规划。

## 工作流验收

最终 [checks.json](acceptance/checks.json) 与 [commands.json](acceptance/commands.json) 记录：

1. 批量 prepare-only 后有 25 个未执行 job，`verify` 返回 2。
2. 仅执行 case `e000_r000_n00200` 后完成 9 个 job，其余 16 个仍缺失；仍返回 2，所有方法的请求分母仍为 3。
3. 续跑补齐全部 25 个 job；先前 130 个文件内容哈希全部保持一致。
4. 已完成批量再次普通续跑，380 个任务文件完全不变，成功与失败均不自动重跑。
5. 显式重试 Figure 6 的 NTO_distance，新建 attempt 0002；原先 144 个任务文件逐字节不变。
6. 最终 Forest、FR3、batch 均 `integrity_passed=true, complete=true, verified=true`；Forest 和 batch 的 `all_valid=false` 正确反映数值失败，FR3 为 true。

首次开发运行、v2、v3 以及只 prepare 未执行的早期批量均保留。它们是开发/故障修复证据，不能混入最终统计。

## 回放与产物

- [Forest 回放](acceptance/forest/replay/e000_r000_n00200/replay.html)：成功轨迹实线、失败候选虚线，真实圆障碍及保存的端点。
- [FR3 回放](acceptance/fr3/replay/e000_r000_n03000/replay.html)：固定 33 球模型的关节/工作空间播放、旋转、缩放与时间/参数轴。
- [批量第三题回放](acceptance/batch/replay/e002_r000_n00200/replay.html)：保留 NTO 验证失败候选。

回放从保存的数组生成，不重新运行规划器。静态 PNG 已实际检查；交互脚本通过 Node VM/Canvas stub 行为测试和 JS/Python FK 数值一致性测试。本会话无可用浏览器自动化环境，未声称做过真实浏览器交互验收。

## 限制与后续重点

平台已可用于新实验，以下边界必须随结果披露：

- FR3 球模型、场景及任务分布为公开资产重建；作者原始 33 球参数、题单和 KUKA 实机记录仍不可得。当前实现不声称复现论文的精确排名、17.1× 加速或实机结果。
- 本轮只跑小规模验收；未启动新的正式论文评测，也未引入新的谱算法。
- FR3 碰撞检查采用有限采样与匹配球/盒几何，不是连续时间或真实非凸机器人网格的安全证明。
- NTO 点约束会漏掉约束点之间的碰撞；这是已显式检出的数值限制，未用改目标、静默替换算法或丢弃失败的方法隐藏。
- SIGKILL/掉电不能执行 Python finally；工作区锁只覆盖同一 checkout，不能协调其他目录的 GPU 程序。正式计时前须停止无关 GPU 任务，串行运行 doctor 探针。
- 当前环境及依赖核查实测通过；新机器的首次 Conda 重建尚未独立验证，网络镜像、CUDA 驱动和许可证也需要在目标机检查。
- 未来实验应先冻结任务分布、种子网格、预算、数值变体和成功条件，再收集结果。先扩充 FR3 难例与路径碰撞回归，明确统一的总规划时延统计，之后再开展正式比较。

所有原始运行、候选、轨迹、验证文件、命令日志和测试故障证据归档于 `evidence.tar.gz`。`evidence_manifest.json` 逐文件记录 SHA-256；`evidence_receipt.json` 记录压缩包哈希和逐成员读回核验结果。这些文件用于核对实测事实，不用于替代运行本身。
