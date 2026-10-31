# 论文对比方法的现有证据与补测范围

本审计只读取已保存的输入、源码、协议和原始结果，没有调用规划器、优化器、碰撞检测器或 GPU。以开始审计时的 HEAD `b73f2ddbc` 为版本锚点；完整 HEAD、源文件与证据 SHA 在 [JSON](existing_evidence_audit.json)。

**目前并不缺少 NTO、LGCS、LSCS、SCSTO 的实现。实际缺口是：最新完整开发矩阵使用 `NTO_REFINED`，原始 `NTO_PAPER` 尚未在相同的 `safe_native` 前端下完整执行。** 这两个 NTO 配置必须保留各自结果，不能逐题取较好者后称为一个方法。论文究竟比较哪些方法，由本目录另行保存的原文核对负责；本文件只审计当前四类实现及其运行覆盖。

## 已有真实运行

旧开发矩阵是 `results/rss2025_full_dev_v1`，源码 `72984027030175b7bca78e2dcf592b38f0c1864b`。最新开发矩阵为 `results/rss2025_reliable_v2_dev_s000`、`s001`、`s002`，源码 `e6b7e372e091ce5b01417073e0880e7c3068910c`。二者均使用同一 60 题、20 个场景、三张 6000 节点路网与 60 秒外部预算；每个方法/目标的合法分母是 180。

| 方法 | 旧开发成功 | 最新开发成功 | 最新完整开发配置 |
|---|---:|---:|---|
| LSCS | 110/180 | 110/180 | 原生 DRM → EI-ZO → 作者 SCSPlanning |
| LGCS | 94/180 | 110/180 | 两路 DRM → EI-ZO → 混合整数图优化，显式 SCIP |
| NTO 距离 | 35/180 | 87/180 | 最新列为 NTO_REFINED；SNOPT，1 mm margin，有界配点细化 |
| SCSTO | 110/180 | 110/180 | 原生 DRM → EI-ZO → 作者 SCSTO |
| NTO 时间 | 88/180 | 100/180 | 最新列为 NTO_REFINED；IPOPT，1 mm margin，有界配点细化 |

NTO 两行的旧、新列是两个预先声明的协议，不能解释为同一次运行或同一配置。旧原始 NTO 距离有 69 前端失败、1 前端基础设施错误、45 碰撞失败、30 求解失败；时间有 69 前端失败、1 前端基础设施错误、18 碰撞失败、1 求解失败、3 超时。最新 NTO 距离有 70 前端失败、11 求解失败、11 超时、1 碰撞失败；时间有 70 前端失败、8 超时、2 碰撞失败。

审计重新核对旧 900 条及新 950 条的 **1850 份 receipt、62999 个封存文件哈希**，未发现错误。原始 NTO 的两目标分别与最新 LSCS 对齐 180 个查询：固定起终点、输入 hash、几何 hash、路网身份、方法种子与预算一致；各自 110 个通过验收的 DRM 种子文件全部逐字节相同。这个事实支持公平比较输入，不能代替原始 NTO 在新前端/日志实现下的实际计时。

正式测试也已有独立归档：`formal_protocol_v1.json` 冻结的主试验和路网规模试验合计 9900 登记组合、9300 合法调用。主试验每种方法有 1125 次合法障碍查询（375 题×3张图）及独立的60次合法 open_control 查询。旧正式障碍集成功数为 LSCS 776、LGCS 682、NTO 距离255、SCSTO775、NTO时间637；这些数字只用于说明既有覆盖，不用于选择本轮配置。本审计只读取正式汇总和协议，未重复校验正式原始 receipt，也未运行正式测试。

## 默认原始 NTO 与细化版本的区别

[`runners/paper_nto.py`](../../runners/paper_nto.py) 的默认 PROFILE 保持原始设置：距离 SNOPT/order 2，时间 IPOPT/order 4；Nc=150，实际内部位置 `k/150, k=1,...,149`；clearance=0；保留所有 DRM 节点，时间仅复制首尾控制点以初始化静止端点。距离显式 major=2000，实际后端默认累计10000/minor500；IPOPT max_iter=2000。外层总预算60秒覆盖初始化、规划和验证。

与旧源码比较，默认数值参数未改；新增的是可选细化入口，以及已经验证的 SNOPT common 日志接口。`NTO_REFINED` 显式启用1 mm margin、最多4轮/600配点、SNOPT累计100000；这些选项不会自动作用于默认 `NTO_PAPER`。当前原始 NTO 不需要再创建一个空实现。

LGCS 已完成显式 SCIP 后端，旧开发的16个 Gurobi许可规模限制现已全部得到实际数值验证成功。不可把 SCIP `gaplimit` 说成已经证明全局最优；原始/对偶界与 gap 已保存。两路种子、图约束和目标未因替换后端而更改。

## 建议冻结并执行的最小补测矩阵

| split | 全部登记题 | 路网 | 方法/目标 | 登记数 | 合法调用数 |
|---|---:|---|---|---:|---:|
| smoke | 10 | n6000, seed 0 | NTO_PAPER 距离/时间 | 20 | 16 |
| dev | 60 | n6000, seeds 0/1/2 | NTO_PAPER 距离/时间 | 360 | 360 |
| 合计 | | | | 380 | 376 |

配置使用现有 `configs/rss2025_reliable.json`，单 GPU 进程、相同60秒冷启动预算与数值 validator。先执行全部 smoke，再执行全部 dev；保留4条无效输入登记和所有失败。补测目录应使用新前缀，不能覆盖旧原始 NTO 或 refined 结果。建议矩阵不是已执行结果；执行状态应由新协议、commands、run manifest 与 receipts 提供。

在不修改 runtime/config 的前提下，这次补测可与可靠版五组结果使用相同全量 source inventory，并由原有 `study_metrics` 聚合为七个单独标签：LSCS、LGCS、NTO-distance、NTO-refined-distance、SCSTO、NTO-time、NTO-refined-time。若为其他方法新增 runtime/config 文件导致 inventory 改变，则标准汇总器会拒绝混合。应保留独立版本并审计跨版本参考，或在一次新冻结下重跑所需完整矩阵，不能绕过校验把历史结果伪装成同版本新执行。冷时延跨时段也不能单独归因于算法变更。

## 实际审计命令

以下命令已实际执行且退出0；仅使用 Python 标准库读取文件，不改变现有 Conda：

```bash
python3 benchmarks/generic_motion_v1/reports/paper_comparators_v1/audit_existing_evidence.py \
  > benchmarks/generic_motion_v1/reports/paper_comparators_v1/existing_evidence_audit.log 2>&1
```

脚本：[audit_existing_evidence.py](audit_existing_evidence.py)；原始输出：[existing_evidence_audit.log](existing_evidence_audit.log)；结构化数据与全部引用 SHA：[existing_evidence_audit.json](existing_evidence_audit.json)。已有数值验证是固定分辨率下的验证，未新增连续无碰撞证明。
