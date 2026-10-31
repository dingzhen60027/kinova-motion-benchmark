# NTO 同输入变体代码审查

审查版本：

- `reproduction/alignment_diagnosis/transcription.py` SHA-256 `a211b6c456dfff203202cba63c6a86b9574fe331c4828648cf5b3aba5604b900`
- `reproduction/alignment_diagnosis/ablation.py` SHA-256 `0af078c491ffa6f2d6b892f9583f4d3a2bc2c6c4ec8e103118da3e07b8ebb869`

只读审查；未修改上述文件。未发现阻断本轮敏感性实验的数学/API问题。

## 检查结果

1. 基线分支复用了原目标函数、端点条件、速度/加速度数值、Nc、求解器及其选项。所选 Forest/FR3 各 10 个冻结输入中，`seed_path.npz` 首尾与 `input.json` 的 start/goal 全部逐元素一致，没有因改用路径端点改变求解任务。
2. 每次选最长结点区间中点插结点，保持初始曲线。Drake v1.51.1 KTO 的轨迹构造函数直接保存传入 basis 的归一化结点，没有强制替换成均匀结点。源码证据见 [缓存来源](nto_source_evidence/source.json)。
3. 额外 CPU 接口探针对 2/3/4/5 个初始 DRM 节点分别插至 16/32 个控制点，重建 KTO 实际初值后比较：位置误差 ≤8.89e−16，速度误差 ≤1.28e−15，加速度误差 ≤1.86e−14，时长误差为零。结果在 [knot_insertion_api_probe.json](knot_insertion_api_probe.json)。
4. `AddVelocityBounds` / `AddAccelerationBounds` 仍约束同一物理导数数值范围。插结点后速度控制点包络变得更贴近真实速度范围，且重新优化时自由变量更多，因此有限维约束集和可表示轨迹族发生改变。报告应称“同输入、相同物理限值的转录敏感性实验”，不能称完全相同优化问题或已恢复作者控制点配置。
5. 采用原验证器，保留所有求解失败和验证失败，并在验证前保存候选。验证超时/异常也能追溯已保存的优化候选。FR3 的独立检查仍是声明的匹配球/盒几何，不应把本次严格通过解读为原始网格的连续证书。
6. 碰撞标量残差按 `EvalBinding - upper_bound` 计算，与 MinimumDistanceLowerBoundConstraint 的 upper-bound 可行性方向一致；它是聚合约束残差，不是物理穿透长度。

## 建议的结果核验

- 当前 `baseline_parity` 只比较状态和 cost。最终汇总时可补查原/新候选控制点、结点和 solver_success，以区分浮点求解差异与任务配置差异；不宜仅凭相同均值声明基线复现。
- `--resume` 启动时前置检查源码哈希，输入哈希到 summary 才核对。当前输入核对通过时不影响结论；今后长时间续跑可在开始前也校验输入，避免变化后继续消耗计算预算。
- 每任务 100 秒墙钟包含模型构建与验证，求解器限制则是 60 秒。若出现超时，应区分优化阶段还是验证阶段，不能一律归为 NTO 不收敛。

探针命令：

```bash
bash local/python.sh reproduction/reports/alignment_diagnosis/knot_insertion_api_probe.py > reproduction/reports/alignment_diagnosis/knot_insertion_api_probe.log 2>&1
```

时间代价机制的独立求解和固定 T 线性规划核验见 [NTO 源码审计](nto_source_audit.md)。该探针读取 793 条时间候选的 2379 个源文件哈希已再次核对一致，且脚本 SHA 与结果记录一致。
