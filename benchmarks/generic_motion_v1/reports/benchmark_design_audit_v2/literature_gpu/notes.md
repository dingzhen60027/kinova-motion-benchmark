# GPU 与优化规划文献的 benchmark 构造核查

检索日期：2026-10-30。范围为 cuRobo、MπNets、GCS、pRRTC 的原论文、补充材料和作者代码。此目录只保存只读文献核查；没有生成题目、调用规划器或修改本项目实验。下文“建议”是本次审查的判断，不冒充论文原文。

## 核心判断

公开且知名的 benchmark 也不是统一的金标准。目标可能是一个关节向量、一个末端位姿或多个 IK 解；场景可能独立，也可能是同一家具中的多组端点；有的题单按专家成功筛选，有的在线计时明确排除初始化。把这些差异删掉，再只比较成功率或毫秒数，会得到错误结论。

对本项目，最值得保留的是：冻结同一机器人/障碍/端点；按独立场景分割和统计；失败及拒绝原因不删；分开几何长度与有约束轨迹时长；共享最终验算器；明确离线、初始化、搜索、简化、验证的计时边界。**不应因为其他论文采用了成功筛选、事后改目标或放松碰撞模型，就在本项目照搬。**

## 文献对照

| 工作与核查版本 | 场景与目标 | 数据选择与独立性 | 对本项目的作用 |
|---|---|---|---|
| cuRobo，arXiv v2 (2023)，代码 v0.6.0 | Panda 7DoF；MotionBenchMaker 与 MπNets；输入起点关节配置和目标末端位姿 | 继承上游题单及其选择机制；原论文数量存在内部不一致 | 借鉴公开题单、分目标指标、时间分位数、热身披露；不能直接把 pose-goal 成功率对照固定 q_goal |
| MπNets，CoRL 2022 / PMLR 2023 正文与补充 | 随机 tabletop、cubby、dresser；目标姿态带任务方向；neutral 或 task-oriented 起点 | 正文说每个测试题有独立环境；公开三个集合按 global/hybrid/both 可解筛选 | 借鉴程序化几何、独立测试环境和失败分类；不照搬专家成功筛选、Hindsight Goal Revision |
| GCS，arXiv v1 (2022) 与作者2023仓库 | 7DoF KUKA iiwa；固定架子和两个箱子；固定/扰动 waypoint 配置 | 有手工 IRIS seeds、预建 PRM；主要是多查询结构化场景实验 | 借鉴离线/在线拆分及 PRM＋shortcut 强基线；不能把大量端点重复算成大量独立环境 |
| pRRTC，arXiv v2 (2025)，作者网站现标 ICRA 2026 | MotionBenchMaker；Panda/Fetch/Baxter；7/8/14DoF | Panda/Fetch 每类100题，7类；Baxter3类每类500题；代码按 valid 标志跳过输入 | 借鉴同分辨率、同扩展长度、优化碰撞后端、ECDF与分位数；冻结版本与有效输入分母 |

## cuRobo：必须分开论文版本和后续库版本

[原论文 v2，§6.1 与 Appendix B](https://arxiv.org/html/2310.17274v2)采用60秒预算及随机重启；成功判据包含无碰撞、关节限位、目标位姿误差。它分别报告关节路径长度、轨迹时长和 jerk 等指标，几何路径没有被直接当作可执行最短时间轨迹。Appendix B.3 明确有约2秒热身及 CUDA Graph 构建；TO seed 数还按场景类别选择。碰撞模型将圆柱包成盒体，导致5个问题在该表示下输入不合法。Appendix B.1 写800＋1600题，而§6.1 写总数2600；本审查保留这一冲突，不自行修正论文。

[官方 v0.7.6 benchmark 页面](https://nvlabs.github.io/curobo/v0.7.6/get_started/4_benchmarks.html)明确说论文结果来自 v0.6.0，页面结果是2024-11-22的v0.7.6，数量为800 MotionBenchMaker＋1800 MπNets。页面还说明后续碰撞距离、优化和运行时间改变。因此不能用页面新结果冒充2023论文的同一实现，也不能从这页数字推断原论文实际导出的题单哈希。

截至本次检索，[官方最新文档](https://nvlabs.github.io/curobo/latest/)已是2026年4月发布的 cuRoboV2/v0.8.0重写版；[技术报告索引](https://nvlabs.github.io/curobo/latest/technical_reports.html)说明其使用B-spline轨迹优化、可微逆动力学和新的GPU原生映射。故v0.7.6现在也属于legacy文档，不能标作当前最新版。这里未把V2新任务或新成绩混入2023方法的审查，更没有按新安装文档重建用户Conda环境。

针对论文使用的官方 tag v0.6.0，核查到 commit `f2eb5f937afef065214daab7e09a56e8f7880176`。其 [benchmark/curobo_benchmark.py](https://github.com/NVlabs/curobo/blob/f2eb5f937afef065214daab7e09a56e8f7880176/benchmark/curobo_benchmark.py) 有以下需要适配审计的细节：

- L184–186 将模型关节上下界各向外扩0.1 rad；本项目不得无记录照搬。
- L210 调用 warmup；L284–286 跳过 `collision_buffer_ik < 0` 条目。后者是可见输入过滤条件，不能直接解释成 cuRobo 本身成功筛选。
- L290–297 的参数为最多100次尝试、60秒、第三次开始启用图搜索。
- L302–325 使用 `start` 和 `goal_pose`，不是固定终点关节向量。
- L384–460 成功分支直接使用库的 success 并把 collision 等标为False；这个脚本自身不能代替跨实现独立验算。
- 函数默认 collision buffer 为−0.01 m，但 CLI 默认是−0.00 m并被传入；**不能仅看到函数签名就说默认 benchmark 缩小了碰撞球。**

另见[原论文 Appendix B.2](https://arxiv.org/html/2310.17274v2#A2.SS2)：作者披露了 Tesseract/Bullet 的负碰撞裕量与额外 OMPL 偏移。它提示复现者必须核对真实几何与裕量，不能仅对齐算法名字。本项目应坚持所有方法相同 canonical 几何和同一接受准则。

## MπNets：学习数据设计不能直接作为算法中立测试协议

[正文§4–5](https://proceedings.mlr.press/v205/fishman23a/fishman23a.pdf)说明测试有 global-solvable、hybrid-solvable、both 三类集合，每类1800题，三个环境族各600题；正文声称每个测试题的场景、起点和目标独立且未进入训练。Hybrid 数据采用 Hindsight Goal Revision，把目标修订为专家实际末态。它评价末端位姿误差、物理违规、成功率、平滑性；20秒 rollout 时限不是统一20秒离线求解预算。正文还直接报告离散平滑器可能漏碰撞。

[补充 Appendix E/H](https://proceedings.mlr.press/v205/fishman23a/fishman23a-supp.pdf)披露随机家具尺寸、位置、朝向与盒/圆柱障碍；端点先从方向合理的末端位姿生成，再通过IK，另加入扰动neutral姿态；merged cubby 会按候选端点合并隔间。它的最终碰撞判据要求多个不同表示的检查器全部判碰撞才记为碰撞，这是宽松的跨模型一致规则，并不等于本项目同一模型下任一检查器发现碰撞就拒绝。1 cm/15°与目标体积条件也不能替换固定关节终点约束。

当前官方代码固定到 commit `42ff07bb95bd75f7c7752bf4300eb53698ae099d`，不声称该提交等于2022论文运行版本。核查 [gen_data.py](https://github.com/NVlabs/motion-policy-networks/blob/42ff07bb95bd75f7c7752bf4300eb53698ae099d/mpinets/data_pipeline/gen_data.py)：

- L463–464：global 前向或后向无解就不给出该对结果。
- L571–638：场景 demo 两端必须在10秒内由 RRTConnect 解出，否则整个场景重新生成。
- L531–568：同场景可产生多组端点和正反方向；默认流程不能仅以记录行数估计独立场景数。
- L818–846：hybrid/both inference 先筛选 hybrid 解，再用其末态修订目标；both 再检查 global 解。
- L666–669：多进程生成调用无显式种子的 `np.random.seed()` / `random.seed()`；原样运行不能满足本项目确定性 `--seed` 要求。

[process_data.py L147–207](https://github.com/NVlabs/motion-policy-networks/blob/42ff07bb95bd75f7c7752bf4300eb53698ae099d/mpinets/data_pipeline/process_data.py)随机分割的是行索引。**这是公开通用处理函数的观察，不是已经证明原论文测试集泄漏**：论文测试题是单独生成，正文还明确声明测试环境独立。本项目仍应在自己manifest中按几何场景hash核查跨集合共享，而非借用该函数推断安全。

官方后续 [robometrics evaluator.py](https://github.com/fishbotics/robometrics/blob/81e3d1d605de84100d8ab880b43096aba221a48b/robometrics/evaluator.py) L85 的 success 只检查位置误差<1 cm和物理违规；不能把这个当前函数说成已经实现论文正文的全部姿态成功条件。公开库与论文指标需要逐项对照。

## GCS：结构化多查询例子与随机环境测试是两种问题

[arXiv v1 §7.4/Appendix C](https://arxiv.org/html/2205.04422v1)的7DoF实验在同一架子＋两个箱子的环境中；作者手工选8个IRIS seed，用图连通性辅助检查，主要比较5种 waypoint 间运动。IRIS与PRM的离线构建分别披露，PRM还用双向RRT搭骨架，再补随机样本；不只有裸PRM，也比较shortcut版本。论文明确实验IRIS采用数值非凸搜索，未给严格无碰撞认证。**可借鉴的是公开多查询假设、强后处理基线和离线成本；不应拿这一固定环境例子作为随机场景广泛泛化的证据。**

此处原文核查的是2022 arXiv v1；[作者仓库](https://github.com/RobotLocomotion/gcs-science-robotics)标记为2023论文复现，当前明确建议新工作使用改进后的Drake实现。不能把这三个版本混作一个执行版本。

当前固定 commit `3d781488136362c700b22db640b0a47806e3e38b` 的 [prm_comparison.ipynb](https://github.com/RobotLocomotion/gcs-science-robotics/blob/3d781488136362c700b22db640b0a47806e3e38b/reproduction/prm_comparison/prm_comparison.ipynb)可见手工seed、10,000节点PRM、固定场景的5种连接、另有1000次随机末端位姿扰动再IK的循环；这不是1000个独立环境。默认演示循环 N=5，注释说明论文用100，故跑默认notebook也不自动复现论文统计。源代码计算GCS时间时汇总 relaxation 与最大 rounded solve time（以可并行为依据），不能无说明等价为进程冷启动墙钟。

该脚本PRM模型为球近似 KUKA、网格0.04 m、边检查步长0.05；这些是当前代码设置，不是对本项目允许更粗验证的建议。[planning.py](https://github.com/RobotLocomotion/gcs-science-robotics/blob/3d781488136362c700b22db640b0a47806e3e38b/reproduction/prm_comparison/planning.py)

## pRRTC：强 RRTConnect 是合理基线，而不是异常

[arXiv v2 §III–IV](https://arxiv.org/html/2503.06757v2)使用与VAMP共享的优化碰撞后端增强OMPL-RRTConnect，统一检查分辨率和扩展范围；RRT族使用同一Halton序列，GPU方法保留其原采样器。计时排除环境设置；报告ECDF、成功率和多个时间/长度分位数。Panda与Fetch各7类×100题，Baxter3类×500题，不能把同族的100题未经几何核查都视为100个完全独立场景。机器人用离线球近似、环境用primitive，边做离散检查。论文正文未明确可复核的全局墙钟超时与每题随机重复次数，本审查不填造数值。

[作者网站](https://commalab.org/papers/pRRTC/)现标ICRA2026；arXiv当前v2为2025-10-07，与v1摘要的速度倍数不同，本笔记不混用。当前代码 commit `6a516f73438f3ff6f06206d01be1bc9afb84670a` 的 [evaluate_mbm.cpp L126–137](https://github.com/CoMMALab/pRRTC/blob/6a516f73438f3ff6f06206d01be1bc9afb84670a/scripts/evaluate_mbm.cpp)按 `valid` 跳过无效输入，接收一个start与goals数组。实际读取该提交 [panda_problems.json](https://github.com/CoMMALab/pRRTC/blob/6a516f73438f3ff6f06206d01be1bc9afb84670a/scripts/panda_problems.json)得到700条注册题，699条valid；每条goals数组长度均为1。无效条目来自table_pick（99/100）。这仅描述该代码快照，**不反推论文究竟用了哪一份输入或分母**。settings默认是最大样本/迭代次数，不等于本项目固定60秒预算。

## 对当前 Kinova 平台的具体审查清单

以下是从文献差异推出的检查项，不表示本项目已发现对应bug：

1. 固定 q_start/q_goal 的主任务继续独立于 pose-goal benchmark。未来若导入Panda公开数据，建立另一个具名套件，不用IK替换当前Kinova端点。
2. 一次注册的题目保留完整分母：生成失败、输入非法、规划失败、超时、验算拒绝分别保存。可行性unknown与已发现有效路径分开；规划失败不等于不可行。
3. 每题保存父scene_id及真实几何hash。分别报告场景数、端点对数、重复数；置信区间按场景聚类，不能用重复运行夸大样本量。
4. 端点分布与几何类别的匹配需检查：均匀全关节采样并不会自动穿过名义上的workspace缝隙。bottleneck、route_choice标签要由几何描述界定，再报告端点/直连实际覆盖，不能反过来按某方法失败改标签。
5. 长度比较保留共同成功配对及条件分母；端到端成功率与合法初始路径后的优化成功率并列。RRTConnect＋shortcut、PRM＋shortcut、优化器前端成本都应完整计算。
6. 延续canonical机器人和障碍覆盖证明/数值证据；同模型多检查器不一致时记未决，不能采用“全部检查器都碰才算碰”的宽松投票。
7. 冷启动墙钟、热态在线、离线roadmap、碰撞建模、最终验证分开。若方法只用10秒搜索，不能仅因外层60秒限额就说搜索预算已等同。
8. 最短时间轨迹必须统一速度/加速度、端点导数、连续性和采样验算精度；几何路径除非接入同一明确时间参数化程序，否则不进入同一个时间排行榜。

文献检索/下载状态、固定commit与源文件SHA256见 [sources.json](sources.json)。这轮未下载大型训练数据，也未安装任何新依赖。
