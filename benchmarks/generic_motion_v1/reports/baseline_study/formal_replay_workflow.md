# 正式实验结束后的已保存轨迹回放

此流程在 24 个正式运行块全部完成后执行。准备脚本时只读既有代码，未读取部分
正式结果挑案例，未运行回放、规划器、数值验证器或测试。HEAD、runtime 与
`BASELINE_PROTOCOL.md` 保持冻结。脚本和测试只新增于本报告目录。

## 前置条件和执行顺序

1. 串行规划结束，`formal_completed_blocks.json` 明确 complete=true，24 个块、
   9900 个槽均由 coordinator 验证完成。
2. 按主任务的正式后验审查流程完成证据审查，并生成 **全部 6300 槽主实验**
   的 `study_metrics` 输出。不要使用部分结果、开发集或 80 题规模子集作为回放选择源。
3. 此时才运行下列轻量回归测试以及回放脚本。`--main-metrics` 用实际完整主结果路径。

```bash
bash local/python.sh -m pytest benchmarks/generic_motion_v1/reports/baseline_study/test_formal_replays.py -q
bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_study.make_formal_replays \
  --planning-finished \
  --main-metrics benchmarks/generic_motion_v1/reports/baseline_study/formal_main_metrics/metrics.json \
  --output benchmarks/generic_motion_v1/reports/baseline_study/formal_replays_v1 \
  > benchmarks/generic_motion_v1/reports/baseline_study/formal_replays_v1.log 2>&1
```

这里的 `formal_main_metrics/metrics.json` 是命令示例中的明确输出约定；若主任务
实际导出到另一路径，替换该参数即可，不重算实验、不重选任务。本准备阶段尚未
声称该文件或回放已生成。输出目录必须新建且位于本报告目录下，已有目录拒绝覆盖。
失败后保留旧目录和日志，排查渲染问题后使用新的输出目录；不重新调用规划器。

程序首先复用 `audit_formal_study.completion_barrier`：没有显式结束参数，或
24/9900 完成状态不匹配时，在读取指标行之前拒绝执行。之后核验冻结协议、完整
runtime 源码散列、15 个主运行块与其 manifest SHA、6300 槽完整指标、五组合、
test/6000 分组、公共前端一致性。所选运行再次完整核对 receipts，所选输入身份、
合法性、直连碰撞属性和保存数组逐件对照原始 receipt。回放自身仍使用现有
`runners/replay.py`，执行 CPU 正向运动学和绘图；不执行 CUDA 查询或优化。

## 预先固定的选择规则

- 五组合为 LSCS、LGCS、NTO-distance、SCSTO、NTO-time。先找全部组合都严格数值
  成功的相同 `(case_id, roadmap_id, method_seed)` 配对，优先输入标注直连碰撞的题，
  再优先非 open_control。其后按 case_id、roadmap seed、roadmap ID、method seed
  的字典序选第一组。没有共同成功组时，按同样规则为每组合单独选第一条严格成功。
- EI_ZO 最短距离和最短时间各额外提供一个直连碰撞成功条目；若已选条目满足条件，
  复用同一回放；否则从该组合直连碰撞成功集中按相同字典序选第一条。不将 unblocked
  命名为 blocked。若不存在，明确记录缺失，不改变条件。
- 另选一个合法失败且已有独立验证 DRM 种子的任务。先优先直连碰撞、非 open_control；
  再依次优先 collision_failed、solver_failed、unsupported、timeout、infrastructure_error；
  最后按同样任务字典序选择。失败页只展示 **verified seed-only**。
- 不按成本、速度、路径外观或最终图像挑案例。没有严格成功、没有受阻成功或没有
  合格失败种子时写明缺失，图库 complete=false；不制造或重新求解替代轨迹。

这些是结果说明图的选择规则，不是代表性抽样设计；不能用图库成功比例作为性能
统计。即便共同成功组用于展示，它也不取代全部合法输入的主统计分母。直连碰撞
来自已冻结的输入采样检测，仍不是连续空间拓扑或可行性的证明。

## 失败回放不会伪装成最终解

现有通用回放在没有根目录最终样本时会搜索子目录样本。为避免失败时意外展示某个
候选轨迹，本脚本不让它自行选择：把原失败任务的 `input.json`、`result.json`、
`process.json` 和 `frontend_validation` 中的种子 samples/validation 原封不动复制到
报告内的 `seed_source_projection/`，逐文件验证 SHA，并保存源路径和 receipt SHA。
这是带来源记录的展示副本，不是新实验 attempt。

HTML 页顶部、样本来源说明及图库均显著写明 **SEED ONLY — failed attempt**，
同时保留实际失败类别。数据 JSON 记录 `display_role=verified_frontend_seed_only`、
原 attempt、原样本、源 receipt 和全部投影文件 SHA。种子不是优化后的最终解，
也不是最短时间轨迹。成功回放则必须使用根目录已封存的最终
`trajectory_samples.npz`，拒绝子目录回退。全部原始 attempts 保持不变。

## 生成物和验收

- `formal_replays_v1/index.json`：选择规则、是否共同配对、实际身份、失败/缺失、
  源与生成文件 SHA、实际 Python 命令及完成标记。
- `formal_replays_v1/index.html`：离线图库，链接每个实际回放。
- 每个非复用条目有 `replay.html`、`replay.json`、`scene.png`、`joints.png` 和渲染日志。
- 两个 EI_ZO blocked 条目可引用已有回放而不重复渲染；仍明确记录源题和输入属性。
- 失败种子的原始投影和 provenance 独立保存，失败页不画任何虚构连接。

实际执行后应再查看至少两个 EI_ZO blocked 页面与失败 seed-only 页面，确认机器人
完整碰撞几何、原障碍与球覆盖、播放控件、时间轴及显著失败标记。有限保存样本的
回放不是新的连续无碰撞证明。准备完成不等于运行验收通过，真实结果将由日志与
index.json 给出。

## 计时结束后的隔离测试记录

收到主任务确认全部 24/24 块、9900 槽计时结束后，实际执行上述 pytest 命令。
最初七个 fixture 为 **7 passed in 0.06s**，日志为
`formal_replay_fixture_tests.log`。随后静态检查发现本报告工具的 HTML JSON 字符串
转义需要保留字面 `\u003c`，已仅在报告脚本修正，并加入 JSON 嵌入/seed-only 标签和
成功页拒绝 frontend fallback 的测试。扩大后的结果为 **9 passed in 0.09s**，日志为
`formal_replay_fixture_tests_v2.log`。这些均为临时目录和内存 fixture；没有读取正式
结果挑选案例，没有创建正式回放，也没有运行模型、CUDA 或求解器。冻结 runtime
和协议未变。等待完整主指标文件路径后才执行图库命令。

后续实际执行已完成：主任务提供完整 `formal_main_metrics/metrics.json` 后，执行
上述正式回放命令，退出 0，8 个条目全部生成。详细选择、260 个源工件散列复核、
24 个链接检查及实际限制见 `formal_replay_execution.md` 和
`formal_replays_v1/post_render_verification.json`；此前“等待”描述仅记录准备时状态。
