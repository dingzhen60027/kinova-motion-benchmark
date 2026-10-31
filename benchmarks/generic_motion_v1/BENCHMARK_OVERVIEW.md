# 机械臂运动规划 Benchmark 总览

更新于 2026-10-30。本文介绍 CSDecomp 原论文实验、可复用的公共基准和当前 Kinova v1，供选择后续实验协议使用。公共数据的文献与格式核查已经完成；其规划运行尚未接入当前 Kinova 管线。

一个完整的 benchmark 同时规定机器人、障碍、起终点、约束、数据选择规则、验证器、计算预算和统计指标。算法名称属于 baseline；场景与题单属于数据；评测工具负责运行和记录。三者应分别说明。

## 一眼看清三类来源

| 来源 | 机器人 / 状态空间 | 场景与任务 | 主要用途 | 与当前项目的关系 |
|---|---|---|---|---|
| CSDecomp Forest | 二维点路径 | 随机圆障碍，跨越二维区域 | 观察几何路径、凸集和优化行为 | 原论文低维方法实验 |
| CSDecomp Franka | 七自由度 FR3 | 桌前随机球障碍，右侧到左侧 | 比较机械臂规划管线的速度、可靠性和代价 | 当前论文方法 baseline 的研究来源 |
| CSDecomp KUKA 实机 | 七自由度 iiwa + 深度相机 | 改变桌面障碍后重新规划 | 验证感知、规划与执行系统 | 与纯仿真统计单列 |
| MotionBenchMaker / Panda 公共题 | 七自由度 Panda，也提供其他机器人 | 桌面、箱体、书架、笼状障碍 | 可复用的结构化场景比较 | 后续公开评测的优先候选 |
| MπNets 公共题 | 七自由度 Panda | 随机桌面、柜格、柜体环境，位姿目标 | 扩展场景分布与运动生成评测 | 可增加独立数据来源，需对齐目标定义 |
| 当前 Kinova v1 | 七自由度 Gen3，固定夹爪 | 四类随机障碍 + 无新增障碍对照，固定关节端点 | 受控算法比较、开发、消融与回归 | 本机已运行并保留完整证据的套件 |

上表中的原论文设计见 [CSDecomp §VII](https://arxiv.org/html/2504.10783v1#S7)；公共资源见 [MBM](https://github.com/KavrakiLab/motion_bench_maker)、[VAMP 资源说明](https://github.com/KavrakiLab/vamp/blob/main/resources/README.md)和 [MπNets](https://proceedings.mlr.press/v205/fishman23a.html)。它们服务于不同实验问题，题数和成功率不能直接混合。

## CSDecomp 作者如何安排实验

**先区分 Forest 和 Franka。** Forest 是二维环境：10×10 区域的中央 7×7 区域随机放置15个半径0.35的圆障碍，规划从左下到右上的路径。Franka 才使用 FR3：33个机器人碰撞球、20个环境球和箱形桌面，任务由右侧随机构型到左侧目标。两者都研究最短距离和最短时间。本文据此纠正此前概述中将两种环境合写造成的混淆。[原文 §VII-A1–2](https://arxiv.org/html/2504.10783v1#S7.SS1)

每种仿真环境采用10个环境 seed、10个 DRM seed和4种路网规模，共400个组合。Table I 后端结果以 DRM 成功为条件，代价另取求解成功样本。因此读表时必须同时看前端成功率、优化成功率、无碰撞比例、代价与阶段耗时。[原文 Table I](https://arxiv.org/html/2504.10783v1#S7.SS1.SSS5)

实机部分使用 KUKA iiwa 和3台深度相机，在改变桌面障碍后规划，报告15条轨迹。它验证系统运行，与随机仿真统计分别报告。[原文 §VII-B](https://arxiv.org/html/2504.10783v1#S7.SS2)

从实验安排可以理解作者的逻辑：先观察低维几何行为，再比较七维规划，最后验证实机系统。这里是对实验设计的解释。复用其方法时，应保留“共同前端下的后端比较”和“完整管线比较”两种口径。我们当前在 Kinova 上完成的比较属于另一明确模型与任务分布；原 FR3 实验资产的核查见[来源审计](reports/paper_algorithm/source_audit.md)。

## 哪些公共基准可以复用

### MotionBenchMaker：公共数据和生成工具

[MotionBenchMaker](https://github.com/KavrakiLab/motion_bench_maker)提供40个预生成数据集，也提供从名义场景扰动障碍、生成端点和运行规划器的工具。它覆盖多个机器人与结构化场景，适合检查规划器在不同空间布局中的表现。部分场景名称包含 pick，但可以评价其几何运动规划问题；这不要求我们引入抓取策略、装配或任务奖励。

[VAMP 发布的 MBM 资源](https://github.com/KavrakiLab/vamp/blob/main/resources/README.md)对 Panda 提供7类场景，每类100题，共700题：table_pick、table_under_pick、box、bookshelf_small、bookshelf_tall、bookshelf_thin、cage，并提供机器人资源及 JSON 转换工具。这是 MBM 的一个整理版本，不能把 VAMP 数据再计为独立来源。

原 MBM 的生成包含规划成功筛选，因此其公开题带有选择机制。公共集仍有比较价值，使用时应披露这一点，且不能把它的成功率与我们保留可行性未知题目的自建集直接横比。[MBM 原论文](https://www.kavrakilab.org/publications/chamzas2022-motion-bench-maker.pdf)

### MπNets：另一种场景来源和目标定义

[MπNets](https://proceedings.mlr.press/v205/fishman23a/fishman23a.pdf)使用程序化桌面、柜格和柜体环境，其原研究包含感知与学习，但后续工作可以复用几何任务评价规划器。题目以末端目标位姿为核心，测试子集经过专家可解性筛选。若我们复用几何题，应明确提供完整已知几何的评测协议，不将其等同于原论文的部分观测策略实验。

具体例子是 [cuRobo v0.7.6 的官方评测](https://nvlabs.github.io/curobo/v0.7.6/get_started/4_benchmarks.html)：800道 MBM 加1,800道 MπNets，共2,600题。这个版本的结果日期为2024-11-22，页面注明原论文使用 v0.6.0；它不是当前最新版的默认题单或成绩。800题 MBM 与 VAMP 的700题版本也应分别冻结。

### 数据入口与评测工具

| 资源 | 提供的功能 | 应如何使用 |
|---|---|---|
| [Robometrics](https://github.com/fishbotics/robometrics) | 机器人运动评测工具及 MBM/MπNets 整理数据 | 记录上游来源、版本和目标表示，不算第三个独立来源 |
| [OMPL Benchmark / Planner Arena](https://ompl.kavrakilab.org/benchmark.html) | 重复运行规划器、限制资源、保存日志和分析结果 | 仍须指定实际机器人、场景、起终点与验证规则 |

本项目已检查的 Robometrics 文件包含 `start`、`goal_pose` 和候选 `goal_ik`。其中800条 MBM 题对应700种不同障碍记录，箱体场景的目标变体复用了布局，不能把800题理解成800个独立环境。如果从多个 IK 解中选择一个固定 `q_goal`，应在运行算法前按统一规则冻结，并命名为派生协议。所有方法应使用同一个目标定义。[公开数据实查](reports/benchmark_design_audit_v2/literature_mbm/research_notes.md)

## 当前 Kinova v1 测的是什么

输入是固定的七维 `q_start/q_goal`、同一 Kinova Gen3 碰撞模型，以及静态已知障碍。场景与端点使用独立随机数流，端点直接采样全部7个关节；合法性检查包含限位、自碰撞和环境碰撞。题目在方法运行前冻结，不按某个规划算法的成功筛选。合法端点不代表已经证明存在连接路径。

| 场景族 | 几何构造 | 应如何解释 |
|---|---|---|
| sparse_clutter | 6–12个随机球 / 盒障碍 | 稀疏障碍布局 |
| dense_clutter | 20–40个随机球障碍 | 密集障碍布局 |
| bottleneck | 两个有限墙体及中间间隙 | 工作空间几何标签；不保证机械臂必须穿隙 |
| route_choice | 两簇球障碍 | 布局类别；不证明存在不同 C-space 同伦类别 |
| open_control | 保留机器人、桌面和固定设备，不新增障碍 | 仍可能有自碰撞及基础环境碰撞 |

新增随机障碍在规划中表示为半径0.04 m的保守球体素并集；桌面和固定设备仍使用 canonical 模型中的固定几何。源球/盒用于生成与描述，新增障碍的最终碰撞语义以冻结的覆盖模型为准；它与名义几何存在明显膨胀差异。[生成配置](configs/generator.json)、[几何审计](reports/benchmark_design_audit_v2/README.md)

| 划分 | 登记记录 | 合法题 | 生成失败 | 用途 |
|---|---:|---:|---:|---|
| smoke | 10 | 8 | 2 | 流程、验证和失败处理检查 |
| dev | 60 | 60 | 0 | 开发、对比和诊断 |
| test | 420 | 395 | 25 | 已评测的冻结比较集 |
| 合计 | 490 | 463 | 27 | 每条记录均保留 |

合法 test 包含75个随机障碍布局和一个共享基础环境。20道 open_control 题共享该基础环境；395道题不是395个独立环境。dev 的短/中/长距离比例为1:1:1，合法 test 为2:2:1。v1 test 已经曝光，继续使用时应称固定比较集，不能称未使用的盲测集。[完整审计与逐题注释](reports/benchmark_design_audit_v2/README.md)

当前两类目标分别报告：

| 目标 | 输出与代价 | 已接入的方法 |
|---|---|---|
| 最短距离 | 关节空间路径，长度以 rad 计 | DRM→EI-ZO→LSCS/LGCS、NTO distance、RRTConnect、RRT*、CHOMP、STOMP、TrajOpt |
| 最短时间 | 带时间参数的轨迹，时长以 s 计，含速度/加速度和静止端点约束 | DRM→EI-ZO→SCSTO、NTO time |

“最短”是任务目标，不表示每种算法证明了全局最优。CHOMP/STOMP/TrajOpt 等扩展方法的原生优化目标包含不同平滑或碰撞项，统一用最终有效路径评价时仍需保留这些差异。原 NTO 和 refined NTO 分别记名，不能混成一个结果。[论文方法协议](BASELINE_PROTOCOL.md)、[当前方法与运行入口](../../README.md#methods)

## 后续实验如何保持可比

| 比较维度 | 应固定或报告的内容 |
|---|---|
| 任务 | 同一机器人、碰撞几何、起终点和目标类型；固定关节终点与末端位姿目标分轨 |
| 成功 | 全部合法输入上的最终数值有效率，同时列前端、求解、碰撞、超时和依赖失败 |
| 质量 | 在同题共同成功集合上比较长度或时长，并给出该集合大小 |
| 计算时间 | 分离离线建图、启动/加载、搜索、优化、后处理和验证；求解耗时与机器人运动时长分开 |
| 预算 | 保留外层总截止时间及方法内部停止条件，不能把外层60秒解释为每种方法都搜索60秒 |
| 独立性 | 按物理场景聚类，端点对与随机重复单独计数 |
| 安全验证 | 所有方法采用共同验证规则；有限采样通过只支持数值验证结论 |

建议后续实验分工为：公开 MBM/Panda 用于与既有工作建立联系，现有 Kinova v1 用于受控比较与开发，另行准备未用于开发的独立任务用于泛化检验。接入 Panda 公共题时应使用其对应模型；换成 Kinova 并重采端点属于新适配集。现阶段公共题完成了来源与格式核查，不能把它们写成已经完成的本机规划实验。

## 文档与证据入口

- [运行说明](README.md)：环境、CLI、方法与结果入口。
- [数据规范](task_spec/BENCHMARK_SPEC.md)：最初需求和格式约定。
- [2026-10-30 审计](reports/benchmark_design_audit_v2/README.md)：实际输入、几何、分布与历史结果核查。
- [当前方法对比](reports/benchmark_v1_extended_baselines.md)：已运行的方法和结果口径。

勘误：上述审计报告首表的 CSDecomp 行将二维 Forest 与七维 Franka 的描述放在同一单元格，容易混淆两者。本总览按原文分别说明；已封存报告、SHA 和原始实验文件保持原样。该文字勘误不改变 Kinova 数据或任何历史成绩。
