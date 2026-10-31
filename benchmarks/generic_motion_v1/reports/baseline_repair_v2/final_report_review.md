# 最终报告只读复核

数值与证据核对通过：False。178 项检查，1 项失败。主报告 SHA：`2359ff6997eeef955fbaebe1a67e0cd33c9fa61e97ae32d6f5f9256eebc0aad8`。

复核 950 条登记观测、940 条合法输入、五方法各180条dev记录、四组同输入方法比较、五组新旧比较、各族与直连标签、完整失败分类、冷/温计时及阶段条件均值。所有正文数字、差值方向与已封存 metrics 和三份独立审计一致；历史绑定和全部本地正文链接有效。六个回放目录文件及来源收据 SHA 核对通过。置信区间只复核已绑定值、方向和分母，未重复 bootstrap；浏览器交互由主任务另验。

发布前说明：BoxBox 调查沿用历史构型最小复现；本轮新 NTO 原生异常为0、BoxBoxGradient日志命中为0，不应将其解释为当前11条solver_failed的原因（SNOPT info13=7、info41=4）。建议通过补充说明链接 [本轮NTO解释](nto_result_interpretation.md)，保留已封存主报告。

导航补齐：六视图文件和SHA已核对，但主报告尚未链接 [回放总览](development_replays_v1/index.html)；最终交付或README应直接提供入口。

本复核只新增本JSON/Markdown，不修改主报告、receipt、metrics、冻结runtime或原始结果，不运行GPU、规划器、求解器或碰撞查询。
