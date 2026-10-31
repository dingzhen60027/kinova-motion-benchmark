# NTO 日志修复的真实开发任务验收

审计日期：2026-10-28。本报告核对主协调者在源码 `3edaca8fc5fd57fd8c4f25fbfe054ab80d4490d3` 上实际执行的6个固定开发查询，对照旧 `rss2025_full_dev_v1` 中相同 job_id 的记录。三个新运行均已封存完成；执行命令、退出码和运行完整性结果保存在 [actual dev checks](nto_logging_actual_dev_checks.json)。它们是日志修复验收，**不是新一轮基线评分，也不替换旧900条开发结果**。

只读配对审计通过：12个新旧attempt共核对285个文件哈希，无receipt不一致。6/6固定输入、roadmap、method seed、原始DRM path、初始控制点、knots、表示阶数、配点位置和固定数值求解参数一致；6/6最终分类一致。四个实际返回候选的查询，其全部raw solver数组及candidate数组与旧记录逐元素完全相等。异常和超时查询没有返回候选，不能把缺失数组视为数值相等。完整逐项结果在 [nto_logging_real_validation.json](nto_logging_real_validation.json)。

## 1. 新旧配对与实际日志

所有查询使用 `rss2025_n6000_s000`，外层预算60秒。固定数值参数比较包括SNOPT major limit=2000、两类major tolerance=1e-6；IPOPT max_iter=2000、tol=1e-6；模型、碰撞约束和目标没有改变。`Time limit`/`max_cpu_time` 是每次在共同端到端预算内重新计算的剩余时间，因此不能要求它们逐字相同：本次为58.193–59.182秒，历史对应值为58.206–59.188秒。文件输出路径也随新目录变化。

| 固定题 | 目标 | 历史分类 → 本次分类 | 本次原生日志证据 |
|---|---|---|---|
| dev-bottleneck-000-p00 | 距离 | success → success | SNOPT INFO1；0 total /0 major；26264字节 |
| dev-bottleneck-000-p00 | 时间 | success → success | IPOPT返回成功；有 Optimal Solution Found footer |
| dev-sparse_clutter-002-p02 | 距离 | solver_failed → solver_failed | SNOPT INFO31；10000 total /630 major；106387字节 |
| dev-sparse_clutter-002-p02 | 时间 | success → success | IPOPT返回成功；有 Optimal Solution Found footer |
| dev-route_choice-002-p00 | 距离 | solver_failed → solver_failed | SNOPT日志有EXIT70/INFO71；8150 total /257 major；62597字节；Drake随后报告同一backend异常 |
| dev-route_choice-002-p00 | 时间 | timeout → timeout | 没有最终IPOPT EXIT footer；新日志最后写到1923次，旧日志最后写到1926次 |

success在此均指已封存的 `success_validated_numerical`，不是连续无碰撞证明。最后已写出的迭代编号不能证明被杀进程实际执行的最后一轮；超时记录仍保持timeout。由于这6题是明确选取的接口诊断，不能把3成功/2求解失败/1超时当作新的总体成功率。

三份新SNOPT日志都已超出原先121字节版本头，实际包含参数列表、迭代记录、终止码和迭代统计。日志完整列出 **Iteration limit=10000、Major iterations limit=2000、Minor iterations limit=500**。sparse_clutter距离例的真实10000 total /630 major直接证实该例首先耗尽累计迭代限额，而不是2000次major或60秒；历史记录只能读取info31，本次重跑新增了可核查的迭代证据。旧记录保持原样，不能把新计数倒填到旧日志。

route_choice距离例也实际具有SNOPTA终止footer：EXIT70表示退出类别，INFO71记录函数求值期间终止。随后Drake抛出 `BoxBoxGradient` 异常，`nto_solver_result.json` 的状态是exception，没有native result或candidate返回。必须同时报告这两层证据；不能因为日志里有footer就声称Drake成功返回，也不能错误写成该日志没有footer。IPOPT超时例才是没有最终footer的实际情况。

这些真实机械臂查询使此前的CPU小程序验证落地：`CommonSolverOption.kPrintFileName`确实恢复完整日志，没有改变这四个有返回值的数值解；其余两个仍诚实保存原有失败类别。它不证明所有未来任务的wall time完全不受日志I/O影响，也不证明backend异常已修复。

## 2. 有限CPU追加诊断：已定位一个可独立复现的几何pair

主协调者随后授权一分钟以内的纯CPU追踪。本轮使用旧 `dev-route_choice-002-p00` 的同一保存路径、输入和配置，保留 `model.nto_plant()` 产生的native对象，调用现有paper_nto；没有调用DRM、GPU或正式test。约6.57秒后再次得到同一SNOPT包装的BoxBoxGradient异常，并保存native context中的七维位置为 [last_evaluated_q.npy](nto_backend_last_q_probe/last_evaluated_q.npy)。

这个值仍标为“异常之后context中的last_evaluated_q”，不保证恰是原solver失败回调的配置。随后在该q上执行独立CPU查询，取得更具体的事实：

1. 直接minimum-distance约束求值再次触发相同Drake断言，并以exit1结束进程；异常前保存的数据没有丢失。
2. 直接AutoDiff Box–Box pair查询报告此scalar/shape组合不支持。这个错误已保留，不能把它误认为原断言的碰撞pair证据。
3. double类型的pair扫描完成18个pair后，在finger/table查询前最后留下marker；该marker本身只用于选择独立复查对象。
4. 新CPU进程只执行一次 `ComputeSignedDistancePairClosestPoints`，pair为 **`robotiq::left_inner_finger_collision` 与 `table::Box`**，在保存的q上再次触发完全相同的BoxBoxGradient断言并exit1。这个单pair复查使定位不再仅依赖扫描marker。

捕获q为：

```text
[2.0755408865839566, 2.0011936907854526, -1.8898095919170692,
 1.9083356598539885, -1.4039254524900748, -1.0345121069462058,
 1.9903692918549658]
```

q文件SHA256为 `46faf400094085a1474cd9cfb3729d1eecb48143bcd4a5474e34741247ddcbb4`。关键证据包括 [捕获记录](nto_backend_last_q_probe/diagnosis.json)、[AutoDiff失败](nto_backend_last_q_pair_probe.log)、[double扫描](nto_backend_last_q_double_pairs.log)、[独立single-pair调用](nto_backend_last_q_isolated_pair.log) 和 [调用前输入记录](nto_backend_last_q_probe/isolated_pair_probe.json)。每个独立查询约0.6秒；诊断后已停止solver/几何查询，没有并发正式评测。

现已获得“同q、同模型、单pair可复现该backend断言”的证据；尚未证明它就是原solver某次回调的完整状态，也没有实施backend修复。该故障不能解释为规划任务无解，不应靠去掉finger/table过滤对或修改几何来规避。正式结果中应保留solver_failed并单独标明已复现的backend问题。

## 3. 复核与复现入口

6个真实运行的完整命令保存在 [execution receipt](nto_logging_actual_dev_checks.json)，本报告逐项链接到其相同job_id的旧记录。配对审计仅用Python/NumPy读取JSON、NPZ和日志：逐文件核对receipt SHA256，直接比较输入和数组，从SNOPTA而非SNMEMA行提取终止信息；没有重新碰撞验证或重算分数。

独立single-pair诊断是在inline Python中实际执行的，其等价操作已保存为 [reproduce_isolated_pair.py](nto_backend_last_q_probe/reproduce_isolated_pair.py)。如需之后定位backend，可从仓库根目录执行下面命令；这份脚本预期在当前环境重现native exit1，**本报告保存脚本后未再次执行它**，以避免与正式计时并发：

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_study.nto_backend_last_q_probe.reproduce_isolated_pair
```

没有修改runtime、历史正文、第三方环境或原有分数，也没有提交git。新增证据只支持日志修复验收和backend故障定位，不能被写作一次新的论文算法性能改进。
