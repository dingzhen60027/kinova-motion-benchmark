# LGCS 开发修复批次的独立审计

脚本 `audit_lgcs_repair.py` 只使用 NumPy、标准库及已有只读审计函数，不导入运行时适配器，不调用规划器、GPU、碰撞器或优化求解器。必须等完整 950-slot 批次完成后运行；所有四个 block 的每个 attempt 都必须封存，且最后一个 attempt 不得是待续跑状态。2026-10-29 全批结束后已实际执行：190 个新 LGCS attempts 与 180 个旧 dev attempts 审计通过，退出码 0，`lgcs_audit.json` 中 `passed=true`、`errors=[]`。完整真实命令、脚本 SHA、起止 UTC 和退出码在 [lgcs_audit_execution.json](lgcs_audit_execution.json)，原始输出在 [lgcs_audit.log](lgcs_audit.log)。这项审计覆盖 LGCS；其余方法由各自独立审计处理。

在仓库根目录执行：

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_repair_v2.audit_lgcs_repair \
  --index benchmarks/generic_motion_v1/reports/baseline_repair_v2/run_index.json \
  --freeze benchmarks/generic_motion_v1/reports/baseline_repair_v2/execution_freeze.json \
  --old-dev benchmarks/generic_motion_v1/results/rss2025_full_dev_v1 \
  --output benchmarks/generic_motion_v1/reports/baseline_repair_v2/lgcs_audit \
  --planning-finished
```

产物为 `lgcs_audit.json`、`lgcs_audit.md` 和 `lgcs_audit_paired_dev.csv`，拒绝覆盖已有输出。JSON 顶层有 `passed: boolean`、`errors: list`、新旧 runtime commit、输入/index/freeze/审计脚本散列。只有 `passed=true` 才允许据此形成比较结论；启动前发现未完成批次会直接拒绝，不能解释成验收通过。

审计核对每个 LGCS attempt 的完整 receipt 文件集合与散列、运行前后源码清单、request/result/process 的身份和冷启动观测时间。完整执行冻结中的数据、roadmap 和外部二进制依赖也核对散列。对每个 SCIP round，核对原始 flow/transition/length/delta 与文本 solution、CIP 维度和二元变量、区域流与齐次多面体约束、未平方 SOC 残差、selected region sequence、丢弃的独立环以及轨迹长度。与最终候选、轨迹采样、保存的碰撞标签/clearance、frontend 两级门控、恢复见证映射和最终 validation 交叉核对。失败、不完整 round、超时及上一轮残留 validation 均单独保留，不计入成功。

SCIP 的 `getGap()` 定义为 `abs(primal-dual)/min(abs(primal),abs(dual))`；这不是 Gurobi 常用的 primal 分母。SCIP log 的 `Gap` 以百分数表示且只有有限小数，`0.00 %` 不证明真实 gap 为零。审计将 JSON 分数 gap 用同一个公式重算，并只按日志显示精度比较百分数。实际日志 footer 的 `optimal`、`gaplimit`、`timelimit` 等状态必须与 JSON 一致。合法 incumbent 可在限时状态下接受，但不得标成 `globally_optimal=True`；没有 incumbent 时禁止出现成功候选。[官方 PySCIPOpt Model API](https://pyscipopt.readthedocs.io/en/stable/api/model.html)

新旧统计各自保持一个冻结 runtime 和一个 solver backend，仅在 `case_id + native.map SHA + method_seed + solver_seed` 完全相同且 `problem_sha256`、输入合法性不变时配对。冷启动观测时间来自 process 记录，包含全部合法失败结果；代价差仅使用双方均通过最终验证的题。CSV 同时保留双方状态，失败不被共同成功子集掩盖。因为本轮还修复 DRM 查询，这种比较是整个流水线的配对变化，不能把全部变化归因于 SCIP。三个地图种子重复同一题，当前描述统计没有 IID 或置信区间声明。

审计不重新求解所有区域交集来证明邻接图完备性，也不重新运行碰撞检查。求解器的最优性只针对给定局部区域图并受数值容差约束。有限采样通过始终不等于数学连续无碰撞证明。

小测试命令：

```bash
bash local/python.sh -m pytest -q benchmarks/generic_motion_v1/reports/baseline_repair_v2/test_lgcs_repair_audit.py
```

实际小测试：`lgcs_audit_fixture_tests_v2.log` 为 30 passed。`lgcs_audit_saved_probe_compatibility.json` 记录了三个先前保存的 CPU probe 的只读核查，包含 optimal 与 gaplimit，未读取新批次的部分成绩。
