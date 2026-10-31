# NTO 本轮开发结果与失败证据

完整独审 passed=True，1310 receipts / 42704 个文件，206 条候选重建最大采样差 2.66e-15 rad。

| 目标 | 旧成功 / 180 | 新成功 / 180 | 旧 blocked 成功 / 141 | 新 blocked 成功 / 141 |
|---|---:|---:|---:|---:|
| minimum_distance | 35 | 87 | 0 | 52 |
| minimum_time | 88 | 100 | 53 | 65 |

两种目标每种均有 180 次配对开发查询；blocked 标签有 141 次，仍不表示全局可行性已获证明。旧/新前端均验收且路径不同的配对数为 0。两种目标各有一条旧基础设施失败转为正常前端失败，前端和优化器版本差异均单独披露。

本轮新 NTO 记录的原生 exception 数为 0，所有 JSON/log 中 BoxBoxGradient 命中 0。这不等于底层问题已修复；旧开发构型的同模型 double 距离接口复现证据仍有效。
距离最终 solver_failed 的 SNOPT info 分布：{'13': 7, '41': 4}。原始失败、timeout 和失败候选均保留，没有用内部曾成功的轮次替换最后结果。

最终验证 205 条候选，保存数组的模型碰撞标签分歧总数 0。

## 最终碰撞失败

| case | roadmap | 目标 | native 成功 | 轮数 | 配点数 | 最小采样间隙/m | 停止原因 |
|---|---|---|---|---:|---:|---:|---|
| dev-route_choice-002-p00 | kinova7-full-n6000-s1-e258c5f9914c | minimum_time | True | 4 | 154 | -0.0009526180435059234 | native_solve_limit |
| dev-sparse_clutter-004-p02 | kinova7-full-n6000-s2-e258c5f9914c | minimum_time | True | 4 | 162 | -0.001413984146539829 | native_solve_limit |
| dev-route_choice-003-p01 | kinova7-full-n6000-s2-e258c5f9914c | minimum_distance | True | 4 | 154 | -0.0008532197398323227 | native_solve_limit |

这些候选即使 native solver 返回成功，也因统一终检发现碰撞而保持 collision_failed，不能作成功轨迹或连续安全证明。完整 case/attempt/原生码/验证摘要/证据 SHA 见同名 JSON。

## 审计器修正记录

首次审计误将 5 条首轮 watchdog 超时缺少顶层初值副本判为缺失数据。轮内 nto_initial.npz 均已保存并通过原始 path/controls 检验；程序被杀死前未执行返回后的顶层复制。首次失败 JSON、Markdown、log、命令和脚本保存为 nto_audit_initial_timeout_copy_failure*；审计器只针对 timeout、process.timed_out=true、首轮 running、无 solver_result/最终 candidate 的组合允许这一缺失，新增正/反 fixture 后 18 项全通过。运行结果没有修改。
