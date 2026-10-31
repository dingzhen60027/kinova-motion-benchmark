# 报告渲染脚本只读审查

对象：[render_report.py](render_report.py)，版本散列见 [review receipt](render_report_review_receipt.json)。未修改 renderer 或冻结 runtime，没有执行真实 metrics、规划、求解器或 bootstrap。只读取数据结构、冻结输入族分布和既有统计实现。

结论：当前真实数据设计支持每方法 dev 分母 180（60 固定合法题 × 3 路网），四族各 15 题、共 20 场景；共同成功代价配对和场景聚类方向正确。但报告发布前需加固证据绑定、修复重复运行会改写比较文件的问题，并纠正阶段计时表述。

## 应阻止发布的问题

1. **三份 `passed=true` 尚未与当前四块证据绑定**（原行 31–43）。门控不核查 `errors=[]`，也不核查完整覆盖、source、index、freeze 或 metrics 的 run-manifest SHA。三份无关的 `{"passed":true}` 可以通过当前第一层门控。建议最小绑定：
   - `integrity.complete=true`、950/940/10、`errors=[]`；其 source 等于冻结提交，`evidence.freeze/run_index` 的散列对应当前文件。
   - `integrity.run_manifest_sha256` 等于当前 index 的四块表，也等于 `metrics.runs` 中路径/manifest SHA 表；各 run 完成、没有多余或重复 run；metrics 来源提交等于冻结提交。
   - LGCS `index_sha256/freeze_sha256/new_source_commit` 一致、`complete=true`、`errors=[]`、新 LGCS 190 slots（含 2 个无效 smoke）、旧 dev 配对 180。
   - NTO `index_sha256` 一致、`errors=[]`、`planned_new_slots=950`、`new_nto_slots=380`、`paired_dev_nto_slots=360`。NTO 当前 schema 没有 `complete` 字段，不能不加适配地强制它存在。
   - 旧 metrics 要求 `comparable=true`，恰有原 dev run / 900 slots，绑定原 manifest SHA；把实际旧 metrics 文件 SHA 固定在比较证据中。原版和新版 source 本来不同，不应要求相等或混池。
   - 轻量核对 metrics 的状态计数与独立 integrity 审计组计数一致。不必重新做所有 bootstrap；对 CI 明确只绑定已冻结统计实现和输入，不冒称独立重算 CI。

2. **重复渲染先覆盖比较证据，再拒绝覆盖最终报告**（原行 82–84、161–168）。`historical_comparison.json` 先被 `write()` 改写，最后才检测最终 Markdown 已存在。第二次执行可能让旧 `report_receipt.json` 中的 comparison SHA 失效，即使最后确实抛错。应在任何写入之前检查最终 Markdown、comparison、receipt 的存在状态；所有校验和渲染先在内存完成，再写入新路径。失败产物应留存且不能覆盖旧证据。

## 分母、配对和数值口径

- **180 的实际依据成立，但脚本尚未验证。** 当前 groups 按 label 建 dict，会静默覆盖重复 group；pairing 只检查 `len==180`，重复某条并缺另一条也可能仍为 180。应拒绝重复 `(split, roadmap_nodes, scope, label)` 和重复 row key；验证旧新每方法 key 集合完全相同、每组 180 legal、20 scene clusters、当前整体 950 rows/940 legal/10 invalid。分母可从已核对的 `legal_jobs` 输出，避免声明字段与表格不一致。
- **配对输入已核对 endpoints/model/geometry/input/objective/budget，方向正确。** 补充 `input_legal`、`cluster_id`、`family`、直连标签一致。CI 目前只使用新版 cluster_id；若聚类标记改变，应直接拒绝。共同成功代价只用两个版本都成功的同一 case/map/seed，单位分离，不能解释为全任务最优性提升。
- **失败分类表可能漏项。** 批系统还允许 `invalid_endpoint`、`invalid_scene_geometry`、`model_consistency_failed`、`limit_failed`，当前表仅列七类。合法输入在运行中得到其中一种状态时仍必须占失败分母。建议动态列全部出现的非成功状态，或增加“其他（明确类别）”并断言成功数加所有失败数等于合法分母。
- **当前条件成功分母是 `drm_success=true` 的题。** “共同前端通过后成功”容易被理解成旧新共同通过；实际读取的是新版条件统计。建议称“本版 DRM 数值验收后成功”，并强调它是辅助条件结果，主表仍以全部 180 为分母。
- **历史比较缺少时延差。** 当前表只显示新版冷启动 p50/p95；历史 comparison 只计算成功差和共同成功成本差。若要判断修复代价，补充所有合法配对的冷启动差及共同成功题冷启动差，分别保留 scene-cluster CI；不把选中成功题的时间改善推及全体题。当前全合法 cold 统计包含失败/超时，这一处理正确。
- smoke/open_control 没混入 dev 主分母，处理正确。正文仅声明 smoke 总量，最好附简短按方法/控制族的验收状态表，或明确指向独立审计中已单列的 smoke 数据。

## 计时与报告措辞

- **`validation_seconds` 是多轮候选终检的累计时间。** LSCS/SCSTO 在每个 recovery round 使用 `+=`；不应将该列表头写为“最终验证”。建议“候选终检累计/s”。优化与碰撞修复也可能累计多轮。
- **stage 均值的单位是完整后端尝试，不是单次内部调用。** `completed_stage_times()` 只有在整个后端有 candidate/final validation、无异常或未完成状态时才保留后端计时；早期已经完成但后续失败的阶段也被排除。建议说明“有完整后端计时的尝试中，各阶段累计时长的均值，n 为尝试数；属于条件统计，不能相加当作全体线上均值”。当前“不填零”方向正确。
- LGCS 的 `alternate_drm_path_seconds` 未在阶段表列出，建议单列或说明表格非全部耗时拆解。外部 cold 包含启动/初始化/规划/验证，offline 另列；这些口径正确。
- NTO_REFINED 的正间隙与自适应配点是明确的工程增强，应继续保留该独立标签。将“保留 DRM 控制点”改为“保留 DRM 路径初始化规则”更准确，避免误称优化中控制点固定。
- 增加一句明确边界：“本轮是 Kinova 固定关节端点上的论文方法基线及公开工程增强；未完成原 FR3/Franka 仿真资产与 KUKA 实机实验的精确复现。”当前仅写 Kinova 轨道，边界不够直接。
- 全局完整性审计检查收据和路径，不重建 LSCS/SCSTO 曲线。LGCS/NTO 的独立数值审计与运行时统一 validator 要分别说明，不能将“三审计”概括成对所有算法都做了新的独立轨迹数值重建。

## 合成 fixture 的实际结果

[test_render_report_gate_review.py](test_render_report_gate_review.py) 实际运行 **21 passed in 0.05s**，日志：[render_report_gate_review_tests.log](render_report_gate_review_tests.log)。其中：

- 17 个拒绝测试证明：任一 audit 缺失/false/非严格 bool true，或者声明的完成性、可比性和数量不满足，当前脚本会拒绝。
- 4 个诊断测试证明：三份无来源的 true 审计、或者 true 与非空 errors / false complete / 错误 index 并存时，当前门控仍继续读取历史 metrics。它们通过代表漏洞被实际观察到，不代表这种行为正确。

fixture 在历史读取处主动停止，未读取真实 metrics 内容、未生成报告、未计算任何真实成绩。修正门控后应把这四项诊断改为拒绝回归，并补齐来源/manifest/分母绑定的正反 fixture。
