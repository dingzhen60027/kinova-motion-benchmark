# 开发集完整性审计执行说明

工具：[audit_development_study.py](audit_development_study.py)。它只读取和比较保存的证据，不调用规划器、求解器、GPU 或运行时碰撞 validator。必须等本轮四块全部完成后运行：

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_repair_v2.audit_development_study \
  --planning-finished \
  > benchmarks/generic_motion_v1/reports/baseline_repair_v2/integrity_audit.log 2>&1
```

默认输出为本目录 `integrity_audit.json` 和 `integrity_audit.md`。同名 `integrity_audit/` 目录保存 950 个历史/新运行的逐方法 DRM 比较、190 个五方法共享查询组、全部新尝试的收据散列和错误堆栈。脚本拒绝覆盖已有报告；如审计失败，保留失败输出，修复审计代码后用 `--output /absolute/new_audit.json` 生成新版本，不能覆盖失败记录。

只有 `passed=true`、`complete=true`、`errors=[]` 才通过此项门控。检查内容：

- `execution_freeze.json`、`run_index.json`、`completed_blocks.json` 精确一致；50 smoke + 900 dev，不接受缩减到已成功或已完成的子集。
- 固定 `e6b7e372e091ce5b01417073e0880e7c3068910c` 的来源清单与运行快照；固定配置、预算、模型、全量输入记录、每张 6000-node 路网、对应随机种子和外部依赖。
- 四块全部历史尝试及最终收据可验证；最后一轮不得仍为 retry pending；无效输入不得进入规划。
- 只对旧 50 smoke + 900 dev 运行重验收据，不重新加载旧正式研究轨迹。将 `NTO_REFINED` 配对到旧 `NTO_PAPER` 只用于公共前端比较，方法标签和优化结果不合并。
- 新旧正常返回的 DRM 路径逐元素相同、前端接受状态相同。仅旧日志明确证明两个已评审出口之一导致 SIGABRT，且新记录仍为查询失败，才接受失败分类变化；未知崩溃不自动免责。
- 查询收据中的 fixed endpoints、球半径、二进制/wheel/source 散列和搜索次数不变。所有新方法在同一 case/map/seed 上共享相同返回路径及前端结果。
- 原正式 raw 目录、旧 smoke/dev raw 目录及 test 文件相对 `73356f1a0` 无 Git 修改或新增；test manifest、formal protocol、两份正式 metrics 的已归档 SHA 保持不变，并检查原 protocol 的 test/model/183 项依赖散列。通用 `cli_commands.jsonl` 可追加，不纳入不可变 formal artifact。
- 从收据独立重计各 split/方法/障碍族的所有状态；登记 950，其中合法 940、无效 10。合法输入成功率分母排除无效输入；open_control 单独保留族计数。不同 roadmap 的同一场景不被称为新增独立环境。

此项是完整性与前端一致性验收。它不替代独立 LSCS/SCSTO、LGCS、NTO 数值轨迹审计，也不重新生成置信区间、性能统计或正式 test 分数。运行依赖的验证边界是冻结文件、协调器按块完成记录和最终文件散列，不能证明系统在任意未记录时刻不存在外部修改。

准备阶段只执行合成 CPU fixture。实际 [测试日志](test_audit_development_study_final.log)：45 passed in 0.86s；首次测试导入错误和中间测试日志均保留。本阶段尚未执行完整 950-slot 审计，不能从测试通过推导真实运行验收通过。
