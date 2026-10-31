# 新环境验收失败的 CPU 诊断

本诊断只读取 `run_final` 已保存的题目、路径、日志和验证数组，使用同一 FR3 球/桌/障碍模型的 Drake signed-distance 查询。没有运行规划、优化或 CUDA，没有重采样任务或调整参数。

最终运行完成 24/24 项，失败状态分布：`{'exception': 1, 'validation_failed': 1}`。

## fr3_e03: exception

在线 IK：`success`；端点 CPU 最小间距：`{'start': 0.03372923530651136, 'goal': 0.04365848484585008}` m。
日志先报告候选图路径长度 `[8, 9, 9, 9, 10]`，最终记录 `RuntimeError('[DRMPlanner] No collision-free path exists in roadmap.')`。没有最终输出路径或验证样本。
这是当前有限路线图及惰性检查流程的失败，不能解释为原连续空间问题无解，也不是已证实的端点或 IK 失败。

## fr3_e07: validation_failed

保存的 1790 个样本中，CUDA 与独立检查各标出 8 个碰撞点，分歧数 0；CPU 重查全部确认。
最深保存样本为索引 **1738**，路径第 **3** 段（0-based: 2），段内比例 `0.906250000000`。
几何对：`fr3::fr3_link6_sphere_0` / `fr3::obstacle_18`；穿透深度 **0.185748021 mm**。
其配置、几何名称、几何局部坐标中的接触见证，以及全部碰撞样本均保存在 JSON。该值是已保存样本中的最大值，不是连续全路径最大穿透的证明。

### 对原 0.1 rad 检查间隔的诊断

按公开 `SteerFunctionIsSafe` 的 float32 网格公式，只在保存的最终三条边上重新做 CPU 距离查询：
- 第 1 段：35 个粗网格点，最小间距 15.635317 mm。
- 第 2 段：29 个粗网格点，最小间距 25.600697 mm。
- 第 3 段：28 个粗网格点，最小间距 0.444092 mm。

这些粗网格点均无碰撞，但原 0.005 rad 验证网格已捕获上述穿透，支持‘粗采样之间漏碰撞’这一具体机制。原规划器内部 GPU 查询数组未保存，因此本项不是逐位重放原 GPU 内部调用。未为了通过验收而放松阈值、细化规划步长或覆盖失败。

## 输入完整性及重现

运行前后 `run_final` 全部 **240** 个文件 SHA-256 一致：`True`。模型代码及资产 SHA 与运行 manifest 一致。

```bash
bash local/python.sh reproduction/reports/environment_reconstruction/diagnose_failures.py
```

JSON 保存完整输入清单、失败日志、逐样本见证和 CPU 粗网格查询结果。环境变量 `CUDA_VISIBLE_DEVICES` 在导入模型前设为空；只构造 Drake plant，不构造 native FR3 规划器。
