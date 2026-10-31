# LGCS：SCIP 等价求解适配与开发走廊实测

这次改动为 LGCS 提供**显式选择的开源 SCIP 后端**，保留原来的局部 GCS 整数凸优化问题，没有把 LGCS 替换为首条走廊优化、连续松弛或启发式路径。默认 Gurobi 接口不变，没有自动 fallback。它仍是论文方法的本地重建（`adapted`），不是作者发布的 LGCS 源码，也不是全碰撞自由 C-space 的全局最短路证明。旧正式结果和输入未修改。

## 安装与可追溯依赖

复用 `.conda/envs/csdecomp/bin/python`，通过已授权的 `pip --no-deps --target` 安装到 `.local_setup/solver_probes/pyscipopt`；没有重建 Conda 环境或覆盖其中的包。实际版本为 PySCIPOpt **6.3.0**、SCIP **10.0.2**、glibc **2.35**。下载文件为 `pyscipopt-6.3.0-cp310-cp310-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl`，SHA256 为 `c1564f5fd88b2bf945cd931ebe1cc374010d4c1c1666355bc68e7c9d47c74f51`。

下载使用本机配置的清华 PyPI 镜像，随后与[官方 PyPI 6.3.0 元数据](https://pypi.org/pypi/pyscipopt/6.3.0/json)的同名 wheel 散列核对一致，完整记录在 `wheel_provenance.json` 和 `install.log`。官方文档说明这些二进制 wheel 包含 SCIP；本机实际导入和二元 SOC 求解验证见 `dependency_probe_final/`。[官方安装说明](https://pyscipopt.readthedocs.io/en/stable/install.html)

SCIP 自 8.0.3 起采用 Apache-2.0，PySCIPOpt wheel 中的 LICENSE 为 MIT；随附库保留各自许可。这是合法的独立求解器，没有改变 Gurobi 许可或规避其检查。[SCIP 官方许可说明](https://www.scipopt.org/)、[SCIP LICENSE](https://github.com/scipopt/scip/blob/master/LICENSE)

`dependency_files.json` 逐文件绑定 46 个实际安装文件（65,417,667 bytes），包括 Python/Cython 源、扩展模块、libscip/libgfortran/libquadmath、METADATA 与 LICENSE。`environment.json` 另存平台与动态链接库清单。运行时只导入这个显式目标内的 PySCIPOpt 6.3.0；目标缺失、版本不同或已加载其他位置的包会报 unsupported。`dependency_files()` 与 `dependency_receipt()` 供父级运行封存绑定实际二进制，而不是仅记录版本字符串。

## 数学问题与数值门控

与 `reproduction.paper_completion.optimization.lgcs` 相同：按闭合多面体相交构造有向图，二元边流满足起终点单位流与区域流守恒，区域出度不超过一；每条边的齐次转移点满足两个相邻区域的 `A z <= b phi`；向量流差是区域内位移；最小化各区域位移的欧氏范数之和。只合并完全相同法向量的重复行并取较紧 RHS，规则与原代码一致。起终点、区域系数、碰撞模型和图都不改。

原 Gurobi 写法为 `sum(delta**2) <= length**2, length >= 0`。最终 SCIP 写法是数学等价的 `sqrt(sum(delta**2)) <= length`，使用官方 SOC handler 支持的原始范数表达。这样可在长度接近零时按距离单位检查残差。[SCIP SOC handler](https://www.scipopt.org/doc/html/nlhdlr__soc_8c.php)、[PySCIPOpt 非线性表达](https://pyscipopt.readthedocs.io/en/stable/tutorials/expressions.html)

请求参数：`numerics/feastol=1e-9`、`numerics/dualfeastol=1e-9`、`limits/gap=1e-6`、求解线程和 LP 线程均为 1、随机 seed shift 为 0。模型构建和模型落盘扣除剩余在线预算后才设置求解时间；外层 worker 的总预算仍是最终约束。SCIP 的 LP 内部日志可能报告无 GMP 时将更小的内部容差限制为 1e-10，这些原生日志被保留；它不是任意精度证明。

返回前除了 `SCIP.checkSol(original=True)`，还用 NumPy 独立核查整数性、流平衡、出度、齐次区域约束、向量流及**未平方的范数残差**；再核查解码路径每段的两个端点归属、固定起终点及重算路径长度。阈值为 1e-7。只接受门控通过的 incumbent，原始 status、primal bound、dual bound、gap、node 数始终保留。只有 status 为 `optimal` 且候选门控通过，才标记 `globally_optimal=True`；`gaplimit` 或 `timelimit` 的可行候选不会改称 optimal。这里的最优性始终受求解器数值容差限制。

每次保存 CIP 原问题、全参数 `.set`、SCIP log、图/端点数组、全部 flow/transition/length/delta 数组、文本 solution、代数残差及候选元数据；失败也保存。最终元数据分别列 `solver_soc_constraints`、`solver_nonlinear_constraints` 与 `solver_quadratic_constraints=0`，如实说明原始范数的非线性表达。前两条范数开发探针早于这个纯标签澄清，仍把等价锥数放在旧 quadratic 字段；它们的 CIP 实际也是范数表达，后续代码已分开标记。

## 实际 CPU 验证

`pytest_final.log`：**22 passed**（原 LGCS 13 项 + SCIP 9 项）。验证包括：7D 单区域与 Gurobi 对照、2D 两走廊与穷举简单区域序列的独立 Clarabel oracle 对照、超过受限 Gurobi 二次变量数的 300 变量模型、断开图无解、构建预算耗尽、显式依赖缺失、错误后端拒绝，以及对损坏整数流和 SOC 残差的拒绝。穷举 oracle 在部分不可行序列上产生的两个 NumPy warning 保留在日志；测试没有因此忽略失败。

以下五条来自旧 dev 中已记录 Gurobi 10010 的真实区域池，选取覆盖四个障碍族，逐条使用原来的完整区域池和固定端点。所有原变量数、线性行数、等价 SOC 数及区域图尺寸核对一致。这里测的是保存走廊的 CPU 优化；没有重新生成 front end，也没有宣称完整规划成功率。时长是本次开发探针观测值，可能与同机测试共存，不作为正式基准耗时比较。

| dev case | 变量 / 线性行 / SOC | SCIP status | primal | dual | gap | CPU wall s | 代数与区域门控 |
|---|---:|---|---:|---:|---:|---:|---|
| dev-bottleneck-002-p01 | 172 / 2003 / 5 | optimal | 5.341673626 | 5.341673626 | 0 | 6.819 | yes |
| dev-bottleneck-000-p02 | 228 / 2287 / 8 | optimal | 9.958282628 | 9.958282628 | 0 | 13.694 | yes |
| dev-dense_clutter-002-p01 | 259 / 2750 / 10 | optimal | 6.127602530 | 6.127602530 | 0 | 6.505 | yes |
| dev-route_choice-002-p00 | 188 / 2421 / 7 | optimal | 6.473554290 | 6.473554290 | 0 | 5.585 | yes |
| dev-sparse_clutter-004-p01 | 260 / 2011 / 10 | gaplimit | 10.292534496 | 10.292534447 | 4.77e-09 | 5.123 | yes |

二元 SOC dependency probe 的实际最优值约为 2。7D 单区域长度为 sqrt(7)，两走廊 oracle 最短长度为 4.2。最终 300 变量对称 toy 在 5 秒预算内状态为 `timelimit`，实际 primal=1.32287565451、dual=1.3228593957、gap=1.22907e-05；其候选通过代数门控，但没有宣称最优 gap 已达到。见 `toy_norm_final_bounded_test/`。

## 保留的失败与修复证据

- `dependency_probe/` 保存初次 tiny probe：解已达到约 5e-10 gap，但错误地只接受 status=optimal，使 gaplimit 被判失败。依赖检查改为要求零 gap 的小问题，再次实际成功；没有改业务求解的 1e-6 gap。
- `dev_cpu_initial/` 与 `scip_squared_soc_initial.py` 保留平方表达的两条真实候选及完整模型。SCIP 虽报告 optimal/gaplimit，未平方范数残差约 **3e-5**，因此独立门控明确拒绝。`dev_cpu_norm/` 对同两条改用等价范数形式重测成功；范数残差降到约 1e-9。
- `pytest_norm.log` 保留 300 变量对称 toy 在 15 秒尚未达到 1e-6 gap 的测试失败。测试原先把“有预算的有效候选”错误地等同“必定证明目标 gap”，现改为检查实际 bound 是否包住已知解析最优值、是否诚实保留状态，不能无条件要求限时内最优。该次临时 pytest 目录已被后续 pytest 回收，保留的是完整测试日志；最终同类有限预算解的原始数组和模型已另存。

## 边界与下一步

以上五条新曲线只完成了 CPU 区域/代数校验。物理碰撞、数值连续采样、native DRM/EI-ZO、恢复回合和统一在线预算，需要父级固定 SCIP 的完整 smoke/dev 矩阵继续验证；不能用本表替代这些验证。没有用正式 test 调参或修改旧正式结果，没有为算法更改机器人、碰撞球、障碍、端点或任务。选择 open-source SCIP 后端消除了这里的商业许可规模阻断，但不保证任意走廊在 60 秒内证明最优；超时、有可行 incumbent、无 incumbent、数值失败必须分别保留。

最终适配器 SHA256：`935c45fe4aa61343efac1480e5aa7ea7071ad9a285ff878793a7133af0fb6332`。

## 可复现命令

在仓库根目录执行。目标输出必须使用新目录，探针脚本会拒绝覆盖已有目录。

```bash
bash local/python.sh -m pip download --no-deps --only-binary=:all: --dest .local_setup/solver_probes/wheels pyscipopt==6.3.0
bash local/python.sh -m pip install --no-deps --target .local_setup/solver_probes/pyscipopt .local_setup/solver_probes/wheels/pyscipopt-6.3.0-cp310-cp310-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
bash local/python.sh -m pytest -q benchmarks/generic_motion_v1/tests/test_paper_lgcs.py benchmarks/generic_motion_v1/tests/test_paper_lgcs_scip.py
bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_strengthening.lgcs_v1.probe_saved_corridors \
  --round benchmarks/generic_motion_v1/results/rss2025_full_dev_v1/jobs/dev-bottleneck-002-p01-LGCS-minimum_distance-7670fddf/attempts/0001/rounds/000 \
  --output benchmarks/generic_motion_v1/reports/baseline_strengthening/lgcs_v1/dev_cpu_replay_new \
  --time-limit 60
```

Python 调用为 `paper_lgcs.dependency_probe(out, backend='scip')` 和 `paper_lgcs.solve(regions, start, goal, time_limit=remaining, out=out, backend='scip')`。其余五题的精确源路径与请求在 `dev_cpu_norm/request.json`、`dev_cpu_other_families/request.json` 中，来源 SHA 逐条保存。安装命令只用于复现本次隔离安装；已有相同目标无需重复安装。
