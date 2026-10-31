# 独立结果复核

复核通过：**True**。检查 1392 项，失败 0 项。仅读取封存结果与数组；未重跑算法、GPU 或碰撞检测。

固定实现：`3414dd08f165e0cee0856e86007b53d9b6e97720`。最终审计通过 222397 项检查，零错误。

开发集每种方法均为 **180 次合法运行 = 60 道固定题 × 3 个种子**，覆盖 20 个场景簇。不能将其写成 180 个独立环境。

| 方法 | 成功 / 180 | 前端失败 | 搜索预算耗尽 | 求解器拒绝 | 碰撞拒绝 | 验证未决 |
|---|---:|---:|---:|---:|---:|---:|
| LSCS | 110 | 70 | 0 | 0 | 0 | 0 |
| RRTConnect | 146 | 0 | 33 | 0 | 0 | 1 |
| RRT* | 120 | 0 | 60 | 0 | 0 | 0 |
| CHOMP | 109 | 70 | 0 | 1 | 0 | 0 |
| STOMP | 110 | 70 | 0 | 0 | 0 | 0 |
| TrajOpt | 97 | 70 | 0 | 0 | 13 | 0 |

四个局部管线的 70 次前端失败完全共享；它们获得相同的 110 条已验证 DRM 初始路径。固定路图未找到路径，不代表连续问题已被证明无解。

## 重点失败证据

- **CHOMP：1 次 solver_failed，同时数值验证为 collision_failed。** `dev-sparse_clutter-003-p01 / seed 1`，25 次迭代后 `line_search_stationary`，正常返回，未超时。2670 个验证样本中 7 个碰撞，两个检查器一致；最小间隙 −1.41255 mm，超出接触模糊带。路径长度由 13.391536 降至 12.809157 rad，但这条候选无效，汇总的成功路径成本中没有它。
- **RRTConnect：1 次 validation_undetermined。** `dev-dense_clutter-003-p01 / seed 1`，原生状态 `Exact solution`；3692 个样本中 2 个处于固定 ±2 μm 接触带，最小间隙 −1.03543 μm。两个检查器一致，端点误差为零、关节限位通过；不能描述为模型不一致或明确的带外碰撞。
- **RRTConnect / RRT*：33 / 60 次 planner_timeout。** 原始 OMPL 状态全部为 `Approximate solution`，未满足准确到达目标的接受条件；耗尽的是预先固定的 10 秒搜索预算，不是 60 秒外部进程超时。
- **TrajOpt：13 次 collision_failed。** 其中 12 次原生 `CONVERGED`，1 次 `PENALTY_ITERATION_LIMIT`；均存在接触带外碰撞，两种检查器无分歧。另有 6 条非收敛但通过统一验证的成功候选，必须与原生收敛单列。

## 证据与范围

全套 1140 条登记中保留 12 条 smoke 无效输入，1128 条合法输入；共 717 条数值验证成功。所有进程外部超时 0 次，非零退出 0 次。

仅纳入新建的 smoke_s000、dev_s000、dev_s001、dev_s002 四组运行；没有合并历史结果或正式 test 分数。所有失败均留在分母；拒绝候选的成本不混入成功质量统计。

几何验证来自保存的有限采样结果，不构成连续无碰撞数学证明。统一外部限时为 60 秒，各方法内部搜索或迭代上限按冻结配置分别报告。

Metrics SHA256：`a7f322c61dc9ba41c193c0a9880e739c56f3068f25cfd57b15a6ff16102a962d`。逐案例原始文件 SHA、分类细节和核对结果见 [JSON](final_results_review.json)。

## CHOMP / STOMP 的实际停止原因

主汇总保留 `solver_converged=unknown`：两种方法均未输出显式的该布尔字段；候选顶层也没有 `termination / solver_status / native_status`。因此主汇总中的 `reported_termination=unknown` 不表示原始停止记录缺失。以下从 `optimizer_result.json` 与候选内一致的 `optimizer_status` 提取，不追认或推断收敛。

| 集合 | 方法 | 候选 | 原始停止原因 | 迭代范围 |
|---|---|---:|---|---|
| smoke | CHOMP | 3 | {'line_search_stationary': 2, 'maximum_iterations': 1} | 6–40 |
| smoke | STOMP | 3 | {'native_STOMP_return': 3} | 6–6 |
| dev | CHOMP | 110 | {'maximum_iterations': 22, 'line_search_stationary': 77, 'relative_cost_tolerance': 11} | 1–40 |
| dev | STOMP | 110 | {'native_STOMP_return': 110} | 6–6 |

`native_STOMP_return` 说明原生 STOMP 正常返回；它不等于输出了显式的数值收敛证明。CHOMP 的具体停止原因与是否通过统一几何验证分别保存。70 次共享 dev 前端失败中，50 次保留原生“路图内无路径”异常，20 次为原生正常返回 `query_success=false`。
