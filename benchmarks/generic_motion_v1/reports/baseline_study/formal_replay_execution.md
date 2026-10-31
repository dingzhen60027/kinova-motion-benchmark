# 正式已保存结果回放：实际执行记录

已在正式计时全部结束、独立审计和完整主指标成功后执行。命令退出码为 **0**，
图库 `complete=true`，共 **8 个实际条目**，missing=[]、failures=[]。未重跑规划器，
未修改 runtime、协议、输入或原始 attempts，未提交。

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_study.make_formal_replays --planning-finished --main-metrics benchmarks/generic_motion_v1/reports/baseline_study/formal_main_metrics/metrics.json --output benchmarks/generic_motion_v1/reports/baseline_study/formal_replays_v1 > benchmarks/generic_motion_v1/reports/baseline_study/formal_replays_v1.log 2>&1
```

实际 Python argv 另外保存在 `formal_replays_v1/index.json` 的 `command` 字段。
使用的是现有 `.conda/envs/csdecomp/bin/python`。

## 实际选择

| 条目 | 固定题 | DRM seed | 输入直连碰撞 | 实际展示 |
| --- | --- | ---: | --- | --- |
| LSCS | test-bottleneck-000-p00 | 0 | false | 严格数值成功的最终样本 |
| LGCS | test-bottleneck-000-p00 | 0 | false | 严格数值成功的最终样本 |
| NTO-distance | test-bottleneck-000-p00 | 0 | false | 严格数值成功的最终样本 |
| SCSTO | test-bottleneck-000-p00 | 0 | false | 严格数值成功的最终样本 |
| NTO-time | test-bottleneck-000-p00 | 0 | false | 严格数值成功的最终样本 |
| EI_ZO_distance_blocked | test-bottleneck-000-p01 | 0 | true | LSCS 最终成功样本 |
| EI_ZO_time_blocked | test-bottleneck-000-p01 | 0 | true | SCSTO 最终成功样本 |
| failure_seed_only | test-bottleneck-000-p01 | 1 | true | NTO-distance 的 collision_failed；只展示其验证通过的 DRM 种子 |

五方法共同成功组按预先声明规则选择，实际为 **直连未检出碰撞** 的 p00；没有将它
称为绕障成功。两个 EI-ZO 目标的直连受阻展示另选了 p01。失败条目保持真实
`collision_failed` 分类；HTML 页首、样本来源、图库及 JSON 都明确标注
**SEED ONLY / verified_frontend_seed_only**，没有展示失败优化器的候选曲线冒充解。

## 实际核验

`formal_replays_v1/post_render_verification.json` 保存完整核验结果，passed=true、
issues=[]：

- 重新读取 8 个源 attempts 的 receipt，**260 个原始工件 SHA 全部匹配**；源 receipt
  自身 SHA 与渲染前记录一致，没有增加未封存文件。
- 每个展示帧的 7 关节值与对应原始保存数组的索引逐项完全一致；最终样本/失败种子
  NPZ SHA 与回放来源 SHA 一致。没有重新求解或伪造中间帧。
- 8 页 HTML 嵌入的 JSON 与对应 `replay.json` 完全相同；24 个图库/页面本地链接均
  存在且位于图库目录内；生成文件 SHA 与 index 记录一致。
- 每页显示完整的 15 个运动碰撞盒和 5 个固定碰撞盒；实际 PNG 文件头、像素尺寸
  和帧数写入核验 JSON。

另已实际查看 EI_ZO 距离 blocked 的 scene.png、EI_ZO 时间 blocked 的 joints.png
以及失败 seed-only 的 scene.png：世界坐标、几何障碍、完整机器人和保存的 TCP
轨迹可见，时间目标关节图使用秒轴。初次核验只覆盖文件、链接和嵌入数据；随后已
补充下面记录的真实浏览器交互验收。

## 实际浏览器交互验收

2026-10-28 正式计时结束后，启动仅监听 `127.0.0.1` 的临时 HTTP 服务，系统分配
端口 **41273**。CUA 的实际探测结果为 `iab unavailable` 且 browsers=[]，因此使用
本机已有 Chrome 与 Codex 已有 bundled Playwright，建立不含用户个人资料的新
headless context；没有安装软件或修改 Conda。浏览器仅允许向该 loopback origin
发出请求，实际非本地请求数为 0。

```bash
bash local/python.sh -u -m http.server 0 --bind 127.0.0.1 --directory benchmarks/generic_motion_v1/reports/baseline_study/formal_replays_v1
node benchmarks/generic_motion_v1/reports/baseline_study/check_formal_replay_interaction.cjs http://127.0.0.1:41273 benchmarks/generic_motion_v1/reports/baseline_study/formal_replay_interaction_v1 > benchmarks/generic_motion_v1/reports/baseline_study/formal_replay_interaction_v1.log 2>&1
```

交互命令实际退出 **0**，`formal_replay_interaction_v1/interaction.json` 的
passed=true。先打开图库并确认 8 个链接，然后实际点击进入
`EI_ZO_distance_blocked/replay.html` 和 `failure_seed_only/replay.html`，两页分别完成：

| 实际动作 | 成功页与 SEED ONLY 页的观测 |
| --- | --- |
| 点击 Play | 按钮变为 Pause，帧从 `1 / 180` 变为 `2 / 180` |
| 点击 Pause | 按钮恢复 Play，等待 350 ms 后帧数保持不变 |
| 聚焦帧滑条并按 End | 滑条值变为 179，帧数到达 `180 / 180` |
| 检查 Canvas | 首帧与末帧截图 SHA 不同，确认画面实际改变 |
| 按 Home | 回到 `1 / 180` 和原始路径参数 0 |
| 点击 Full scene | 按钮变为 Focus robot，实际触发视图切换 |

失败页顶部 **SEED ONLY** 和 subtitle 中的真实 **collision_failed** 在浏览器实际
可见，成功页显示严格数值成功；没有把种子描述为最终解。JavaScript page_errors、
console_errors、failed_requests、nonlocal_requests 均为 **空列表**。保存 9 张实际
浏览器/Canvas 截图及其 SHA，包括图库、两页的首末帧。实际查看了成功末帧和失败
首帧截图，页面文字、状态和运动几何清晰可见。

交互前后 **268 个源文件（含 receipts）SHA 一致**。浏览器已关闭；向创建服务的
原 exec session 发送 Ctrl-C 后服务器退出码为 0。随后实际连接
`127.0.0.1:41273` 得到 `ConnectionRefusedError`，确认端口已关闭。完整服务器命令、
停止记录和关闭核验保存在 `formal_replay_interaction_v1/http_server_lifecycle.json`。
未重跑规划、未更改源 attempts 或冻结 runtime，未提交。

## 使用入口与限制

打开 `formal_replays_v1/index.html` 可进入全部回放；优先查看
`EI_ZO_distance_blocked/replay.html`、`EI_ZO_time_blocked/replay.html` 和
`failure_seed_only/replay.html`。

这是按固定规则选出的说明图库，不是代表性随机子样本，也不是性能排行。成功组的
直连未检出碰撞、失败的 seed-only 身份和两类目标的区别不能省略。通用 renderer
生成的失败 PNG 仅保留原失败状态，若单独引用该 PNG，必须附上“已验证 DRM 种子，
非最终解”的图注；完整 HTML 和图库已经显著标注该信息。TCP 投影、有限保存样本
和回放不能证明连续无碰撞，更不能将 workspace bottleneck 名称当成 C-space 拓扑
已知。失败 seed 的路径参数不是最短时间轨迹。
