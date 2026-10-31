# 原论文、原始 TeX 与公开模型取证

审计日期：2026-10-25。审计起点为本地 `main` 的 `ec6eaf8`。本轮只新增取证报告和独立硬件模型探针；原 v3 计算代码、输入、结果保持原样。

本轮找到两类此前漏查的实质证据：**arXiv 原始 TeX 中被注释的 NTO/EI-ZO 配置**，以及**当前官方 FR3 模型的关节限位晚于论文且已改变**。仍未找到作者实验使用的 33 球中心/半径、原始题单、完整实验脚本及 KUKA 传感器/执行记录，因此这些证据尚不能建立整篇论文的精确复现。

## 新取证范围

- 从 [arXiv v1 原始源文件](https://arxiv.org/src/2504.10783v1) 实际下载 9,103,858 字节归档，SHA-256 为 `18c4140e37eb05abe636dd4999612618f565c120ce57fbe966e39545b68f1986`。解包审查了全部 TeX、图表源和参考文献。归档中没有实验 runner、URDF、原始轨迹或机器可读题单。
- 重新查询 [CSDecomp 全部公开 refs](https://api.github.com/repos/wernerpe/csdecomp/git/matching-refs/)，核对 10 个 branch、3 个 tag，并补查此前可能遗漏的 8 个 PR head 的完整递归目录树；没有发现 FR3/Forest/NTO 原始实验文件。[Release 列表](https://api.github.com/repos/wernerpe/csdecomp/releases)仍为空。
- 核对 [SCSTO 三个分支](https://api.github.com/repos/TobiaMarcucci/scsplanning/branches?per_page=100)、作者 VAMP 的 main/hardwarestation、iiwa 与 WSG 模型仓库，以及目标化 GitHub code search。VAMP 是其他机器人/规划实验，不能作为本论文 FR3 的原始资产。
- [项目网站](https://sites.google.com/view/GPUPolytopes)仍只提供论文、代码、说明图和演示视频入口。原始源归档和连接器完整响应保存在本机 `source_evidence/`；它们不应整体加入 Git。

以上“未发现”仅描述已检查公开范围，不是证明资产在所有位置不存在。检索失败及工具返回也保留。

另外检查论文提交时 main 最新提交 [ee94f814](https://github.com/wernerpe/csdecomp/tree/ee94f8145bb32ef6c08b1e575218335b9daf533f)（2025-03-12）的完整 205 项目录树，并搜索所有 Python、Notebook、URDF、YAML 和 Markdown 文件。只发现 Kinova 场景引用其他 iiwa14 示例来源的四条注释，没有原论文实验文件。当前实际使用的 [v0.1.2](https://github.com/wernerpe/csdecomp/commit/0c26beafd99aa1eb5da39e79f9b9f6df1b404eb5) 发布提交为 2026-03-19，相隔 10 个提交。比较中 DRM planner、roadmap builder 和部分碰撞接口也有代码变化，不能把它称为论文时期的相同二进制。被删除路径仅为编辑器设置与旧构建文件，没有发现后来删掉的原始实验资产。详见 `csdecomp_version_gap.json`。

随后已从原 `ee94` 树实际隔离构建核心并执行三项小型验证：3000 点 CUDA/解析圆和 300 点 CUDA/Drake 检查均无差异；300 节点动态 DRM 得到经解析线段检查通过的路线；EI-ZO 包含输入种子，2000 个独立验证样本未出现碰撞。原 179 个文件前后 hash 一致，现有 Conda 0.1.2 wheel 未改变。实际构建为 CUDA 12.8 / sm_75，日志、库 SHA、加载路径和每项退出码见 `paperdate_core_result.json`，构建限制见 `paperdate_build_readiness.md`。这确认论文日期已公开核心可运行，不能替代缺失的原实验模型/runner，也没有把当前 800 题主实验自动转换成原核心结果。

## 可据以修正实现的证据

| 项目 | 本次证据 | 应如何使用 |
|---|---|---|
| NTO 碰撞采样位置 | 原 `sections/appendix.tex:175` 注释给出内点 `k/Ncol`，`k=1,…,Ncol−1` | 这是作者草稿源证据；与正式正文称 Nc 个点存在数量语义歧义，不能宣称找回原可执行代码 |
| NTO 最短距离 | 同文件 179–190 行注释描述二阶 B-spline、固定 M 控制点、T=1、相邻控制点距离和 | 支持距离转录；没有给出原 M 的选择规则。注释的 transition 索引自身也有歧义 |
| NTO 最短时间 | 正式 `experiments.tex:49` 以 DRM 节点初始化四阶 B-spline 控制点 | 没有补齐少控制点时的规则、首尾复制次数、初始 duration 或全套 solver 选项；源末尾仍留有未写完的时间转录注释 |
| EI-ZO 最大迭代 | 原 `appendix.tex` 的 Forest/Franka 参数注释均为 30 | 当前 v3 的 500 不是该草稿值；新协议可明确采用作者源注释的 30 |
| EI-ZO τ | `fci.tex:105` 对应正式 PDF 脚注给出通常采用 0.5 | 应单列为有发表依据，不能与完全未公开参数混为一项 |
| 离线 voxel 边长 | 正式 Table V 为 0.06 m；旧注释写 0.1 | 正式表格优先；保留冲突来源，不把注释覆盖正式表格 |
| 多路径 | 正式 §VI-C 在所选内部凸集内封禁节点，重新运行 A*；失败即终止增加路径 | 找不到第二条路径不应让已存在的第一条路径消失 |

TeX 中还存在曾建议选择 DRM 成功 seed 的草稿注释；正式结果仍报告 FR3 DRM SR=0.961。不能据此筛除新实验中的失败例，也不能把草稿备注当作已确认的作者题单采样算法。

## FR3 时代版本与碰撞计数

当前重建使用的 [官方 revision 7aeeddc](https://github.com/frankarobotics/franka_description/commit/7aeeddc449edf8d62b594f9e36a81da53e7796f9) 日期为 **2026-08-17**，晚于论文。论文提交前最后一版 [39f8c0b](https://github.com/frankarobotics/franka_description/commit/39f8c0bcea9bc48ae5334edcb2aa2a504f2d2c85) 日期为 **2025-04-11**。逐个下载并作 SHA-256 对比，FR3 的八个 arm collision mesh、hand mesh、kinematics.yaml 共十项均完全相同；SRDF 的禁用碰撞 link 对没有变化。此结论不能替代对自建球拟合的验证。

六个关节的限位已经变宽，J3 未变。原始 [2025-04-11 joint_limits.yaml](https://github.com/frankarobotics/franka_description/blob/39f8c0bcea9bc48ae5334edcb2aa2a504f2d2c85/robots/fr3/joint_limits.yaml) 为 738 字节，SHA-256 `ae3cc3148dc7d917a6f5317547b27831b37ee290dfd5c362aee8012904ef7767`。

| 关节 | 论文前官方范围（rad） | v3 使用范围（rad） |
|---|---|---|
| J1 | ±2.7437 | ±2.9007 |
| J2 | ±1.7837 | ±1.8361 |
| J3 | ±2.9007 | ±2.9007 |
| J4 | [−3.0421, −0.1518] | [−3.0770, −0.1169] |
| J5 | ±2.8065 | ±2.8763 |
| J6 | [0.5445, 4.5169] | [0.4398, 4.6216] |
| J7 | ±3.0159 | ±3.0508 |

采用论文前版本能去掉这一明确时代偏差，**不能证明作者使用的正是该官方模型**。

对 v3 的实际 Drake candidate 列表重新运行分解：**179 robot–robot + 32 robot–table = 211 静态对**；32 个活动球对 20 个动态障碍形成 640 对，共 851。桌子已存在，差异不是“遗漏桌子”。论文则给出 1187 总对、527 self 对，二者之差恰为 33×20；这支持原动态检查使用 33 个球的解释，但无法反推出所有静态几何或过滤规则。没有理由为了匹配数字增加/删除碰撞过滤。

## 公开 iiwa + WSG 模型实跑

新 helper：`reproduction/probe_public_iiwa.py`。它按固定 URL 与 SHA 下载作者的 [iiwa box URDF](https://github.com/wernerpe/iiwa_description/blob/967cf36204325302bf6726c5c6e65a70f87d6a04/urdf_output/iiwa7_box_collision.urdf) 和 [WSG welded-finger box URDF](https://github.com/wernerpe/schunk_wsg50_model/blob/7b3f5615bbaab066d29803e9386db0ddf1fd92c0/urdf/wsg50_110_welded_fingers_box_collision.urdf)，去除视觉 mesh 依赖后实际组装、载入 Drake/CSDecomp 并检验。只处理仿真模型，不连接硬件。

最终实际运行命令：

```bash
bash local/python.sh -m reproduction.probe_public_iiwa --output reproduction/reports/paper_completion/source_evidence/hardware_probe_pinned
```

| 检查 | 仅公开机器人组装 | 加显式示例桌子 |
|---|---:|---:|
| 自由度 | 7 | 7 |
| 碰撞几何数 | 11 boxes | 12 boxes |
| Native 活动几何 | 10 | 10 |
| 静态 candidate 对 | 42 | 52 |
| 随机配置数 | 2000 | 2000 |
| CUDA / Drake 均判自由 | 1012 | 909 |
| CUDA / Drake 不一致 | 0 | 0 |
| 30 个配置各 link FK 最大绝对差 | 2.1431×10⁻⁷ | 2.1431×10⁻⁷ |

失败过程也保留：最初两个探针因 Python binding 名称和带 model prefix 的 body 名称失败；随后发现 `manual_edit` iiwa 文件把 link7 collision 注释掉，真实只有 7 盒，不能凭文本 `<box>` 数量认定 8 盒。直接照搬另一 WSG 模型坐标系的安装旋转又使所有测试配置碰撞；作者 URDF 手指沿 −y，所以显式采用 `Rx(−π/2)` 将它映射到 flange +z 后重新运行。没有修改碰撞过滤。

最终 9 cm 安装平移、安装朝向和示例桌子均是有记录的重建选择，**并非找回了原始硬件校准**。碰撞对也不匹配论文 105。它证明公开部件可以在当前真实环境中加载并验证；不能替代论文的 15 条实机轨迹、相机观测或 Table II。

## 整篇实验的完整性判定

| 论文实验项 | 要完成的证据 | 当前仍缺什么 |
|---|---|---|
| Table I：Forest 两类目标、五方法、400 题 | 原始输入或有明确差异的重新生成；动态 DRM、native 第二路线、原转录、完整失败分母 | 精确 seed/端点及原 runner；先前静态 Forest 版本不能被称作原始 DRM 协议 |
| Table I：FR3 两类目标、五方法、400 题 | 同上，并有原 33 球、表格与过滤定义 | 原碰撞模型仍未找到；自拟合导致自由空间改变是实质限制 |
| 附录：三类碰撞吞吐 | Forest、FR3、KUKA 各 batch 2 million，明确硬件与计时边界 | 当前硬件不同；KUKA 原模型及观测缺失 |
| Table II、Fig.1/7：15 条硬件轨迹 | iiwa7、三台 D415、标定/同步、8 cm voxel 感知、35k DRM、完整日志 | 无原传感器/执行记录，也没有当前可操作的同一硬件系统；模型探针不能代替 |

证据优先级为正式论文正文/表格、作者源注释、版本固定的公开实现、显式重建假设。完整原始结果不可由调参匹配表格数字来证明。最终主报告应保留无法满足的项及确切原因。
