# DRM 查询失败出口修复与验证

本轮在现有 Conda 环境完成了独立 CPU 扩展构建、原生崩溃复现和开发集路径对照。修复针对公共 DRM 查询的两个进程终止出口，未修改上游 CUDA、冻结模型、端点、障碍半径或搜索预算。已安装 wheel 保持原样。它让原本的进程崩溃变成可审计的前端查询失败；本次失败题没有因此得到一条新路径，也不能据此断言连续构型空间无解。

## 原因和改动范围

在上游 [`drm_planner.cpp`](../../../../csdecomp/src/planning/drm_planner.cpp) 中，`GetClosestNonCollidingConfigurationInRoadMap` 找不到未排除节点时仍执行 `assert(closest_node >= 0)`。调用者已有 `-1` 失败处理，但断言先终止了进程。端点连接失败还会持续增加 `collision_set`，所以仅在调用 `Plan` 前检查节点集合非空不能解决问题。

实际复现题 `dev-sparse_clutter-000-p01`、路网 seed 2：6000 个节点中最初排除 5847 个，剩余 153 个。原配置保留每轮最多 10 次端点连接和 20 轮规划；连接失败逐步耗尽节点，原生进程最终以 SIGABRT（returncode −6）退出。完整 [v2 子进程证据](drm_query_probes_v2/audit.json) 保存命令、输入散列、地图散列、stdout、stderr 和失败结果。

[`native_drm_query.py`](../../runners/native_drm_query.py) 复制该 C++ 翻译单元，仅应用下面两处精确替换。原文件及安装库不变；补丁锚点不匹配时拒绝构建。

| 原代码 | 副本中替换 | 含义 |
|---|---|---|
| `assert(closest_node >= 0);` | `if (closest_node < 0) return -1;` | 允许现有调用者处理候选耗尽 |
| `std::abort();`（A* frontier 耗尽出口） | `return false;` | 将搜索失败返回给现有控制流 |

其余原生 `Plan`、`_plan`、A*、延迟碰撞检查、路径捷径化和次数限制均保留。没有改用 Python 图搜索，没有缩短次数来规避崩溃。第二处 A* 出口的补丁通过源文本和 CPU 回归核查；本轮真实开发题触发的是第一处候选耗尽，未声称已独立覆盖第二处真实 GPU 失败分支。

## 构建、链接和兼容性证据

最终使用目录为 [`drm_safe_build_v2/`](drm_safe_build_v2/)，完整编译 argv、包含路径、源码/头文件散列及编译器版本见 [build.json](drm_safe_build_v2/build.json)。源代码基准提交为 `73356f1a0677a0a7e6fecaf4c25969055fb8d36b`；新 binding 另有文件散列，不能把该未提交 binding 冒称为原提交内容。

| 项目 | 实际值 |
|---|---|
| 编译器 | g++ (Ubuntu 9.5.0-1ubuntu1~22.04) 9.5.0 |
| 编译耗时 | 5.678869 秒，离线构建 |
| Python | 现有 Conda Python 3.10.22 |
| 编译 ABI | `-fabi-version=13`，实际 `__GXX_ABI_VERSION=1013` |
| pybind internals | `__pybind11_internals_v4_gcc_libstdcpp_cxxabi1013__` |
| 本地复用 pybind 头文件 | 2.12.0 |
| 原上游 CPP SHA256 | `159300e72e24c2ed398c2c1453a94c10f1f34024f20b014c06e89825380658d6` |
| 修补副本 SHA256 | `12ef03c722a7c7fbe67623bc2eff6b177b13c87c85deb1420e623ebdfb1a78c1` |
| 现有 wheel native library SHA256 | `0a048625a9d745ac409cff55791d360fe9e91ad1b8d7a3b82245a1a024c57738` |
| v2 sidecar SHA256 | `a3ca2a23e7ca84bfc90542a7e761b3f3f10b33c92889ff1ef85a477c902f8c83` |

[动态链接审计](drm_safe_linkage_audit.json) 的 8 项检查实际通过：`DT_NEEDED` 指向现有 native library 的绝对路径；`checkCollisionFreeVoxelsCuda` 仍是由该库提供的外部符号；修补的 `DrmPlanner::Plan` 是 sidecar 本地 text 符号且未导出；模块初始化符号正常导出；ABI key 和二进制散列一致。原始 `readelf`、`nm`、`strings` 输出及命令也保留在本目录。

编译使用隐藏符号和 `-Wl,-Bsymbolic`，没有全局符号覆盖或替换安装包。原生入口在加载 sidecar 后仍能复现原断言，这也支持两入口彼此独立。每次安全查询前，wrapper 还对比 native/sidecar 看到的关节数、节点数、排除节点数、初始化状态和主要查询选项，加载时核查库、扩展、原 C++、binding 与本地 native 头文件散列。

这里提供的是对当前安装组合的兼容性证据。wheel 内记录的编译器为 GCC 8.5.0，和本地编译器不同；共享 pybind key、显式编译 ABI、成员检查和有限成功路径对照不能构成任意环境下完整 ABI 等价证明。更换 wheel、源码或头文件后必须重新显式构建和验证，不能沿用旧二进制。

## 实际结果

只使用预先固定的 dev 输入，共 3 题、6 个独立子进程；未查询正式 test。原生和安全入口共用同一端点、地图和配置。

| dev case / map seed | 原生结果 | 安全入口结果 | 路径核查 |
|---|---|---|---|
| `sparse_clutter-000-p01` / 2 | SIGABRT −6 | 正常捕获原生 RuntimeError，`frontend_failed`；最终 6000/6000 节点排除 | 无候选路径，不能计为成功 |
| `bottleneck-000-p00` / 0 | 返回 2×7 路径 | 返回 2×7 路径 | `array_equal=true`，最大绝对误差 0 |
| `route_choice-002-p00` / 0 | 返回 5×7 路径 | 返回 5×7 路径 | `array_equal=true`，最大绝对误差 0 |

路径对照文件：[bottleneck](drm_query_probes_v2/dev-bottleneck-000-p00_comparison.json)、[route_choice](drm_query_probes_v2/dev-route_choice-002-p00_comparison.json)。实际查询计时随各子进程保存；本轮单次对照用于兼容性验证，不能作方法加速比或可靠性统计。表中“返回路径”只描述前端返回，不代替统一轨迹碰撞验证或优化器验收。

[CPU 回归日志](test_native_drm_query.log)：**25 passed，1 个既有 Drake deprecation warning**。覆盖补丁精确性、输入/选项预条件、布局不一致拒绝、异常前后证据保存、不覆盖旧查询证据、原生 CPU 空节点与正常最近节点选择。历史 v1 构建、失败探索和 v1 探针均保留，没有覆盖旧结果。

调用接口已支持 `query(..., build_dir=..., out=Path(out))`。进入原生查询之前保存 `build_receipt.json`、状态为 `running` 的 `query.json`；正常返回或原生异常均更新 elapsed 和布局，异常保存类型/原文并重新抛出，交由公共前端失败分类处理。模块加载或输入预检错误发生在原生调用之前，不应误标为已经执行的原生查询。

## 论文参数与剩余边界

本次只读复核 [`rss2025_eizo.json`](../../configs/rss2025_eizo.json)、[既有逐项论文来源审计](../paper_algorithm/source_audit.md) 和本地原论文全文。Table IV 的 δ=0.005、ε=0.005、Np=10000、Nf=10、混合步数 60、回退量 0.01 已配置。Table V 的 DRM 连接距离 10/10、邻居数 10、在线步长 0.1 已配置；`offline_voxel_resolution=0.03` 是半边长，上游 builder 使用 `2*resolution` 的网格间距，确实对应论文离线边长 0.06 m，不能误改成 0.06 参数值。

以下仍需明确披露，不能写成论文原实验完全复现：EI-ZO 最大迭代数 30 来自作者源码注释，9 次二分不是恢复得到的原实验参数；规划重试、工程恢复次数、碰撞见证上限属于有界实现配置。NTO 的求解器可用性及 Nc=150 分段位置约定、SCSTO 六次 Bézier等应继续引用相应方法审计，不能由本次 DRM 修复推导出它们全部等价。Kinova 固定关节端点、原论文 FR3/Franka 资产缺失和硬件实验缺失的边界未改变。

上游 `Steer` 仍采用有限离散采样，通常不含精确末端，零长度输入还存在除以距离的数值风险；本补丁未改变该正常路径以保持最小修复范围。所有返回种子仍须经公共严格端点/分段检查，所有最终轨迹仍须数值验证；有限采样通过不能声明数学连续无碰撞证明。未基于此风险改变障碍半径、机器人模型或端点。后续需要全 dev/smoke 统一配置回归，新增结果与原始冻结结果应分别保存。

## 实际执行命令

从仓库根目录运行，继续使用项目已有 Conda launcher。以下 v2 输出已经存在；重新复现实验时使用新的输出目录，构建器拒绝覆盖旧目录。

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.runners.native_drm_query build \
  --output benchmarks/generic_motion_v1/reports/baseline_repair_v2/drm_safe_build_v2

bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_repair_v2.run_drm_query_probes \
  --build-dir benchmarks/generic_motion_v1/reports/baseline_repair_v2/drm_safe_build_v2 \
  --output benchmarks/generic_motion_v1/reports/baseline_repair_v2/drm_query_probes_v2 \
  > benchmarks/generic_motion_v1/reports/baseline_repair_v2/drm_query_probes_v2.log 2>&1

bash local/python.sh -m pytest -q benchmarks/generic_motion_v1/tests/test_native_drm_query.py \
  > benchmarks/generic_motion_v1/reports/baseline_repair_v2/test_native_drm_query.log 2>&1
```

动态链接核查的精确命令与输出 SHA 保存在 [drm_safe_linkage_audit.json](drm_safe_linkage_audit.json)，未导入扩展或调用 GPU。本轮没有修改 Conda 安装、正式 test 样本或旧运行收据，没有提交 Git；GPU 在开发验证完成后已交还主任务。
