# RSS 2025 论文对比算法核查

核查对象为 Werner 等人的 [RSS 正式论文](https://www.roboticsproceedings.org/rss21/p045.pdf)，并与保存的作者 arXiv TeX 逐项对照。审计基点是 `b73f2ddbcbcbb485a883d08b6b42641ddf1e6d81`。本文件记录方法覆盖范围，不产生新实验成绩。

**论文实际比较的是五个“方法 × 目标”组合：最短距离 NTO、LSCS、LGCS，最短时间 NTO、SCSTO。唯一作为外部 baseline 命名的是 NTO；LSCS、LGCS、SCSTO 是论文 EI-ZO 管线使用的三种优化后端。** 依据为正式 PDF 第 9 页 VII-A.3/4、第 10 页 Table I，以及第 14 页 Appendix D。已有 benchmark 包含全部五个组合的实现和历史结果。

| 目标与方法 | 论文规定 | 当前适配入口 |
| --- | --- | --- |
| minimum-distance NTO | 相同 DRM 路径初始化；Drake KinematicTrajectoryOptimization；Franka 使用 SNOPT | `runners/paper_nto.py`，方法名 `NTO_PAPER` |
| minimum-distance LSCS | 单条 DRM 路径经 EI-ZO 得到 SCS；路径长度 SOCP；论文用 MOSEK | `runners/csdecomp_baseline.py`，`EI_ZO` + `minimum_distance`；现有官方 SCSPlanning 后端使用 Clarabel |
| minimum-distance LGCS | 膨胀至多两条路径；图上混合整数凸最短路径；论文用 Gurobi | `runners/paper_lgcs.py`；`paper_lgcs_scip.py` 提供显式 SCIP 后端，并保存 gap 与独立可行性残差 |
| minimum-time NTO | DRM 节点初始化四阶 B-spline；Drake；Franka 使用 IPOPT | `runners/paper_nto.py`，`NTO_PAPER` + `minimum_time` |
| minimum-time SCSTO | EI-ZO 单路径 SCS；SCSTrajopt 的交替凸优化；Clarabel | `runners/csdecomp_baseline.py`，`EI_ZO` + `minimum_time` |

原始 Table I 以 DRM 成功为条件汇总后端数据，DRM 成功率、线段数、路径长度单列。独立报告原生 DRM 的原始路径质量是有用的前端对照，但应标为组件基线；DRM 没有解决论文定义的带静止端点和加速度约束的连续最短时间问题。

## 全文图表覆盖

已人工查看正式 PDF 全部 14 页的完整页面，并核对作者主 TeX 的实际 `input` 列表。第 12 页处于参考文献区，无实验章节。正式论文的图表范围如下：

| 图表 | 内容 | 是否新增被比较方法 |
| --- | --- | --- |
| Figures 1–3 | 实机/几何示意、EI-ZO 步骤、DRM 构造 | 否 |
| Figures 4–5 | 完整管线与碰撞恢复流程 | 否 |
| Figure 6 | Forest 与 Franka 场景 | 否 |
| Figure 7 | KUKA 实机障碍变化 | 否 |
| Table I | NTO、LSCS、LGCS、SCSTO 的仿真结果 | 是，完整列表即上表五组合 |
| Table II | 15 条实机轨迹的管线分项时间与辅助统计 | 否，没有额外竞争算法 |
| Tables III–VI | EI-ZO 与 DRM 参数 | 否 |

正式文稿未给出额外的 IRIS 或采样规划器比较图表。正文的性能陈述与 Table I 绑定，不能将参考文献中的方法名单解释为作者已实验比较的名单。

## 引用方法与实验 baseline 的区别

IRIS、IRIS-NP、IRIS-NP2、IRIS-ZO 与 Visibility Clique Cover 出现在 III 节背景、IV 节算法渊源中。STOMP、CHOMP、TrajOpt 出现在引言及参考文献；OMPL/PRM 和 Lazy PRM 用于背景或 DRM 构造说明。结论中的向量化采样规划、FCIT* 等是互补技术或未来方向。它们没有进入 Table I，也没有独立的本论文实验结果。若接入当前平台，应明确列为“扩展比较”，不得写为补齐作者已经报告的比较方法。

`NTO_REFINED` 同样属于当前平台的扩展：它保留 NTO 目标、求解器和控制点结构，额外采用固定安全裕度和有界配点细化。最新可靠性研究使用该名称；它不能替换 `NTO_PAPER` 的独立标签或覆盖其历史结果。若要重新建立论文方法的当前配对表，应在修复后的公共 DRM 上追加运行 `NTO_PAPER`，并与 `NTO_REFINED` 分列。

## 参数边界与证据

Appendix D 规定 Franka 的 NTO 为 `Nc=150`；公开正文未明确这 150 个配点是否包含端点。现有 `NTO_PAPER` 采用作者 TeX **注释**中的 `k/Nc, k=1,...,Nc-1`，即 149 个内部约束。四阶初始化的静止端点复制、具体容差、迭代次数和 60 秒外部预算也属于已记录的管线选择，不能称为论文逐项公布的参数。保留这些固定选择，不通过新的测试成绩倒推修改论文方法。

当前主套件继续是固定 Kinova Gen3、固定障碍球与端点；本核查不改变模型、数据、验证器或正式 test。论文 FR3 的 33 球模型与当前模型的区别已由既有模型审计单独记录。

正式 PDF SHA-256：`e61c094de19de72481ac480c37c1717bb9d145c675a9b3fc71066ea0fc726c57`，14 页、6,908,376 bytes。作者源文件、运行适配入口及本次查看页面的 SHA 清单见 [机器可读审计](paper_coverage_audit.json)。保留的正式页面证据见 [第 9 页](source_evidence/rss2025_page09.png)和[第 10 页](source_evidence/rss2025_page10.png)。[RSS 正式索引](https://www.roboticsproceedings.org/rss21/p045.html)与 [arXiv HTML](https://arxiv.org/html/2504.10783v1)也已在线核查。
