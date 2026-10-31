# 报告门控修复记录

本记录对应只读审查之后经主任务授权的修复；[此前审查](render_report_readonly_review.md) 与原测试记录保留，不覆盖。

已修改 `render_report.py`：

- 三份审计必须严格 `passed=true`、`errors=[]`；绑定本轮 source、index/freeze/completed SHA、四份 run-manifest SHA 和完整方法覆盖。旧 dev metrics 固定 SHA，并保留独立来源标签。
- 拒绝重复 run、row 或 group；检查 950/940/10、每方法 dev 180 合法记录/20 场景；从已绑定 rows 检查 group 分母、成功与失败分类、冷启动分位数，并与独立完整性审计计数一致。
- 所有十一种合法终态失败分类均显示；成功数与失败总数必须等于合法分母。条件统计明确标为本版 DRM 验收后的结果。
- 旧新配对还检查 cluster/family/legality/直连标签，导出全合法及共同成功冷时延差；不同来源方法不合并择优。
- 任何已有报告、comparison 或 receipt 均在读入指标前拒绝；全部验证和渲染完成后才以 exclusive creation 写出，保留并发/失败产物而不覆盖。
- 后端阶段表改为按完整后端尝试统计的累计时长，加入第二路 DRM；`validation_seconds` 称“候选终检累计”。明确条件选择与 CI 验证边界。
- 环境表直接读取 `execution_freeze.environment` 的 Python、Drake、CSDecomp、SCSPlanning、Clarabel、GPU；SCIP 指向实际审计版本记录。明确 Kinova 工程基线并非原 FR3/Franka 与 KUKA 实机精确复现。

实际测试：[render_report_binding_tests_v2.log](render_report_binding_tests_v2.log)，**52 passed in 0.24s**。包括缺审计/失败审计/错来源/错 index/错 manifest/不完整覆盖/重复观测/篡改分母与分位数/输出覆盖拒绝，以及一次完整合成元数据渲染。

全部 fixture 只使用构造的数据和临时输出；完整合成渲染替换 bootstrap 为小型统计 stub，没有计算真实研究指标。原只读诊断中的门控绕过测试已改为拒绝回归。未执行真实 renderer，未修改冻结 runtime/config/native，未提交 Git。
