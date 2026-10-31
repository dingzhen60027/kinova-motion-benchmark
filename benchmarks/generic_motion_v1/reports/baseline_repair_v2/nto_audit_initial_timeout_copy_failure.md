# NTO_REFINED 独立结果审计

审计结论：未通过；错误 5 项。
新研究 950 个方法槽位；NTO 380 个；与旧dev配对 360 个。
共核对 1310 份新旧receipt、42704 个封存文件。SciPy独立重建 206 条最终曲线，最大采样差 2.66e-15 rad。

本脚本独立检查四轮/600点上限、固定1mm裕度、原生solver选项、轮间初值、共享预算、全部最终状态、最后一轮原生成功标志以及公共验证记录。按所有方法核对固定输入与已接受DRM路径；未调用Drake、GPU、碰撞查询或求解器。

## 新配置状态计数

| split | 目标 | 原始直连碰撞 | 最终状态 | 数量 |
|---|---|---|---|---:|
| dev | minimum_distance | False | frontend_failed | 4 |
| dev | minimum_distance | False | success_validated_numerical | 35 |
| dev | minimum_distance | True | collision_failed | 1 |
| dev | minimum_distance | True | frontend_failed | 66 |
| dev | minimum_distance | True | solver_failed | 11 |
| dev | minimum_distance | True | success_validated_numerical | 52 |
| dev | minimum_distance | True | timeout | 11 |
| dev | minimum_time | False | frontend_failed | 4 |
| dev | minimum_time | False | success_validated_numerical | 35 |
| dev | minimum_time | True | collision_failed | 2 |
| dev | minimum_time | True | frontend_failed | 66 |
| dev | minimum_time | True | success_validated_numerical | 65 |
| dev | minimum_time | True | timeout | 8 |
| smoke | minimum_distance | False | frontend_failed | 1 |
| smoke | minimum_distance | False | invalid_endpoint | 2 |
| smoke | minimum_distance | False | success_validated_numerical | 1 |
| smoke | minimum_distance | True | frontend_failed | 4 |
| smoke | minimum_distance | True | timeout | 2 |
| smoke | minimum_time | False | frontend_failed | 1 |
| smoke | minimum_time | False | invalid_endpoint | 2 |
| smoke | minimum_time | False | success_validated_numerical | 1 |
| smoke | minimum_time | True | frontend_failed | 4 |
| smoke | minimum_time | True | success_validated_numerical | 2 |

## 旧/新dev状态转移

旧NTO_PAPER与新NTO_REFINED是不同参数/源码协议，下面仅报告同case/roadmap配对转移，不混合成绩，也不逐题择优。原始直连碰撞标签不等于已证明不可行。

| 目标 | 原始直连碰撞 | 旧状态 | 新状态 | 数量 |
|---|---|---|---|---:|
| minimum_distance | False | frontend_failed | frontend_failed | 4 |
| minimum_distance | False | success_validated_numerical | success_validated_numerical | 35 |
| minimum_distance | True | collision_failed | collision_failed | 1 |
| minimum_distance | True | collision_failed | solver_failed | 2 |
| minimum_distance | True | collision_failed | success_validated_numerical | 42 |
| minimum_distance | True | frontend_failed | frontend_failed | 65 |
| minimum_distance | True | infrastructure_error | frontend_failed | 1 |
| minimum_distance | True | solver_failed | solver_failed | 9 |
| minimum_distance | True | solver_failed | success_validated_numerical | 10 |
| minimum_distance | True | solver_failed | timeout | 11 |
| minimum_time | False | frontend_failed | frontend_failed | 4 |
| minimum_time | False | success_validated_numerical | success_validated_numerical | 35 |
| minimum_time | True | collision_failed | collision_failed | 1 |
| minimum_time | True | collision_failed | success_validated_numerical | 12 |
| minimum_time | True | collision_failed | timeout | 5 |
| minimum_time | True | frontend_failed | frontend_failed | 65 |
| minimum_time | True | infrastructure_error | frontend_failed | 1 |
| minimum_time | True | solver_failed | timeout | 1 |
| minimum_time | True | success_validated_numerical | success_validated_numerical | 51 |
| minimum_time | True | success_validated_numerical | timeout | 2 |
| minimum_time | True | timeout | collision_failed | 1 |
| minimum_time | True | timeout | success_validated_numerical | 2 |

新旧均已接受但保存路径不同的配对：0；这些任务不能把所有变化单独归因于NTO细化。详细case、seed、路径和失败记录见同名JSON。

重复roadmap seed是配对查询，不是独立场景。碰撞数组核查和有限分辨率数值验证不构成连续安全证明。后端异常、超时和未决均保留在分母中。

## 审计错误

- `dev-route_choice-002-p01-NTO_REFINED-minimum_distance-60fb6dbf`：Malformed/incomplete NTO artifacts: FileNotFoundError: [Errno 2] No such file or directory: '/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/rss2025_reliable_v2_dev_s000/jobs/dev-route_choice-002-p01-NTO_REFINED-minimum_distance-60fb6dbf/attempts/0001/nto_initial.npz'
- `dev-route_choice-003-p01-NTO_REFINED-minimum_time-bb3a39b1`：Malformed/incomplete NTO artifacts: FileNotFoundError: [Errno 2] No such file or directory: '/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/rss2025_reliable_v2_dev_s001/jobs/dev-route_choice-003-p01-NTO_REFINED-minimum_time-bb3a39b1/attempts/0001/nto_initial.npz'
- `dev-bottleneck-000-p02-NTO_REFINED-minimum_time-dc253f28`：Malformed/incomplete NTO artifacts: FileNotFoundError: [Errno 2] No such file or directory: '/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/rss2025_reliable_v2_dev_s002/jobs/dev-bottleneck-000-p02-NTO_REFINED-minimum_time-dc253f28/attempts/0001/nto_initial.npz'
- `dev-route_choice-000-p02-NTO_REFINED-minimum_time-24c09a25`：Malformed/incomplete NTO artifacts: FileNotFoundError: [Errno 2] No such file or directory: '/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/rss2025_reliable_v2_dev_s002/jobs/dev-route_choice-000-p02-NTO_REFINED-minimum_time-24c09a25/attempts/0001/nto_initial.npz'
- `dev-route_choice-002-p00-NTO_REFINED-minimum_time-24b1f7ad`：Malformed/incomplete NTO artifacts: FileNotFoundError: [Errno 2] No such file or directory: '/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/rss2025_reliable_v2_dev_s002/jobs/dev-route_choice-002-p00-NTO_REFINED-minimum_time-24b1f7ad/attempts/0001/nto_initial.npz'
