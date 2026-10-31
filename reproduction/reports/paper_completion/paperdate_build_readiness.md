# 论文时期公开核心：隔离构建与实际验证

状态：**实际构建成功，collision / DRM / EI-ZO 三项检查全部通过**。命令总进程 exit 0，稳定结果为 [`paperdate_core_result.json`](paperdate_core_result.json)。没有修改原算法文件，也没有安装或替换当前 Conda wheel。主实验和三组吞吐运行结束后才启动本构建。

## 固定来源与运行命令

原论文提交前 main 最新公开提交为 [`ee94f8145bb32ef6c08b1e575218335b9daf533f`](https://github.com/wernerpe/csdecomp/tree/ee94f8145bb32ef6c08b1e575218335b9daf533f)，提交日期 2025-03-12。实际使用 `git archive` 解包到 `reproduction/vendor/csdecomp_paperdate`，179 个原文件的 Git blob SHA 在配置后、构建前和验证后均全部匹配。没有创建分支或新工作树。完整清单为 `paperdate_source_inventory.json`。

实际执行：

```bash
python3 reproduction/prepare_paperdate_core.py --verify-only
bash local/python.sh -m reproduction.verify_paperdate_core --build
```

干净克隆须先执行 `python3 reproduction/prepare_paperdate_core.py`。该入口下载并校验固定 Bazel，恢复指定 Git 提交；本地提交缺失时只 fetch 指定 SHA，不创建分支，不运行编译或更改 Conda。既有源码不匹配时会失败并保存记录，不会覆盖差异。该脚本的 `--verify-only` 已实际通过；没有另建干净目录重试全部下载路径。

实际原始日志/数组保存在：

```
reproduction/reports/paper_completion/source_evidence/paperdate_core/20261025T100730.351287Z/
```

每次运行创建独立时间戳目录，并保留前次结果。`paperdate_core_result.json` 包括完整 build 命令、退出码、加载路径、二进制和日志 SHA、原文件校验、当前 wheel 的前后 SHA 与逐项结果。

## 构建结果

原 `.bazeliskrc` 指定 Bazel 7.2.1。实际下载的[官方 executable](https://github.com/bazelbuild/bazel/releases/tag/7.2.1) 为 71,527,976 字节，SHA-256 `80ccd1ecb4b88750fbe5d7622d67072fddcba9da7808f13356555e480bf67875`，与官方校验文件相同。

Bazel 构建退出码 0，耗时 20.55 秒，250 total actions（242 linux-sandbox）。采用原 `//csdecomp/src/pybind/pycsdecomp:pycsdecomp_bindings` 目标、独立 output base、4 jobs、CUDA 12.8 和 sm_75。未添加 fast-math 或更改数值算法的编译选项。编译日志中的 Eigen 注解与 C++17 structured-binding extension warnings 已保留，没有为消除警告改源码。

库位于 `reproduction/vendor/paperdate_python/pycsdecomp/pycsdecomp_bindings.so`，SHA-256 为 `d81bbc060fa3ed51ae775499eb4e4a74ad96cbd00cbbd8cb3641080e1f6e5d6d`。三项测试均在新 Python 进程中确认从这个路径加载，且 `csdecomp` 新包没有被导入。

当前 Conda 中 0.1.2 wheel 的 `.so` 前后 SHA 均为 `0a048625a9d745ac409cff55791d360fe9e91ad1b8d7a3b82245a1a024c57738`。没有运行 pip install，也没有运行含 apt 安装的原 `setup.sh`。

## 三项实际验证

场景是本次构造的二维点机器人加 3 个动态球形障碍，用于确认原核心可运行；不是论文的原始 Forest 题单。

| 检查 | 实际结果 | 范围 |
|---|---|---|
| CUDA 碰撞 | 3000 点对解析圆检查 0 差异；其中 300 点对 Drake 0 差异；原 Drake bridge 静态模型对 Drake 也 0 差异 | 验证动态 voxel 接口及原 bridge；不代替 FR3 原球模型审计 |
| 原 DRM | 300 节点、6832 有向边；100 个 collision-map entries；阻塞 15 节点；成功得到 3 节点路径 | 端点误差 4.77e-8；解析线段最小间隙 0.0646454；新建原版地图，未读新版地图 |
| 原 EI-ZO | 得到 5 面集合，两个种子端点满足约束；2000 个另行采样点的解析碰撞检查为 0 碰撞 | 最小样本间隙 0.0171808；仅是样本检查与种子包含，不是整个集合的确定性无碰撞证明 |

EI-ZO 使用原默认内部 seed 1337，验证采样 seed 20250415。配置为 margin 0.005、tau 0.5、delta 0.05、epsilon 0.01、bisection 5、particles 1000、每轮最多 10 个超平面、最多 30 轮、mixing 50。上限和概率配置不等于运行后自动获得严格证书。

## 实际兼容处理与保留失败

| 项目 | 原配置 | 本次实际处理/限制 |
|---|---|---|
| OS/Python ABI | Ubuntu 22.04 / Python 3.10 | 相同 OS 大版本与 ABI，使用现有 Conda Python 3.10 |
| CUDA | 作者 README 为 12.6 | 本机没有 12.6；实际使用 12.8.61，RTX 8000 sm_75，不能直接比较论文硬件耗时 |
| Bazel | 7.2.1 | 同版本 query/fetch/build 均实际通过 |
| rules_cuda | 作者 fork `1f8d4a0733226b246e799f3b17e0c76875a7997b` | 原 archive/integrity，无修改 |
| `.bazelrc` | 裸 `--test_timeout=10`，缺少命令前缀 | 使用 `--ignore_all_rc_files`；这是目前唯一绕过原错误配置的参数，没有修改文件 |
| Python 依赖 | NumPy 1.26.4、Drake 1.38.0 等 | 当前 NumPy 2.2.6 / Drake 1.51.1 实际通过上述小型检查；不是原始依赖环境的完整等价证明 |
| pybind11 | Bzlmod 2.12.0；WORKSPACE 另写 2.13.0 | 实际解析到 2.12.0。两个 WORKSPACE 下载 hash 不匹配记录在 `paperdate_dependency_downloads.json`，其声明没有被本次 Bzlmod 使用，未修改 hash |

配置阶段首次 query 因相对 executable 路径错误返回 127；改用绝对路径后 query 通过，fetch 也通过。fetch 日志为 0 processes / 0 total actions，它当时并不代表已编译。完整阶段区别和失败日志见 `paperdate_configure_results.json`。实际构建和三项 native 测试本次均首次通过，无算法修补。

## 已恢复范围与仍存在的差异

绑定名称集合比较见 `paperdate_api_diff.json`。包名为原 `pycsdecomp`。原核心没有 `EditRegionsCuda` / `EditRegionsOptions`，也没有 `EizoOptions.seed`；本次运行确认了接口缺失，未为兼容新版修改 kernel。原 `SceneInspector.robot_to_world_collision_pairs` 可用，原 Drake bridge 在本次小型测试中可运行。原 DRM 的 offline_voxel_resolution 表示半边长，球半径为 sqrt(3) 倍，本探针明确使用 0.2 半边长。

本次结果确认恢复了**论文日期已公开的核心实现**。当前完成的 800 题主实验仍使用 0.1.2；这三个小型检查没有把那些结果改称原核心结果。完整流水线若改用原核心，需要单独适配 seed、recovery 和地图，重新验证，不能无差别 alias 包名。

作者原 33 球中心/半径、原 NTO runner、原始题单和 KUKA 执行/传感记录仍未找到。历史核心构建成功不能填补这些缺失资产，也不能作为整篇论文精确复现完成的依据。
