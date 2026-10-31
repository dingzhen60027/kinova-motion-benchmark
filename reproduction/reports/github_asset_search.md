# GitHub 实验资产检索

检索日期：2026-10-25（Asia/Shanghai）。目标是寻找 CSDecomp 论文未提供的
FR3 球碰撞模型、原始 Forest/FR3 题单、NTO 转录和调参代码，以及 KUKA 实机记录。
本轮只做检索、下载文本候选文件和结构核查，未改变既有实验输入或结果。

## 检索结论

找到了官方 FR3 模型、第三方 FR3 球近似、作者的 KUKA 模型仓库和 SCSTO 实现。
在本次检查的公开范围内，仍未找到能确认属于原论文的 33 球 FR3 模型、原始题单、
NTO 完整配置或实机传感器记录。“未找到”是本次检索结果，不代表能证明这些资产
在整个 GitHub 或其他位置不存在。

| 所需资产 | 公开资源及链接 | 核查结果 |
|---|---|---|
| FR3 运动学和本体模型 | [官方 franka_description](https://github.com/frankarobotics/franka_description/blob/7aeeddc449edf8d62b594f9e36a81da53e7796f9/robots/fr3/fr3.urdf.xacro) | 已下载并查看 Xacro；包含 FR3、手爪及碰撞几何配置入口。可作为重建基础，未证实是论文使用的球模型 |
| FR3 球碰撞近似 | [OpenReal2Sim 的 FR3 球配置](https://github.com/PointsCoder/OpenReal2Sim/blob/8f69a19e5dfdfe0aad906fdacad0decf862a7adf/openreal2sim/simulation/isaaclab/assets/franka_fr3/spheres/franka_fr3_mesh.yml) | 实际解析出 61 条球心/半径定义；配套配置还有附加物体、余量和碰撞过滤，不符合直接认定为原文 33 球资产的条件 |
| 其他 Franka 球模型 | [StanfordASL/frax 的 Panda 球 URDF](https://github.com/StanfordASL/frax/blob/cc0a066b5b539c729838308496ae1b8a1d6aeada/frax/assets/franka_panda/bubblify/panda_spherized.urdf) | XML 核查为 22 个球碰撞几何、7 个转动关节；这是 Panda，不能当作原文 FR3 |
| SCSTO 算法实现 | [TobiaMarcucci/scsplanning](https://github.com/TobiaMarcucci/scsplanning) | 原作者的 biconvex 实现公开；前一轮已经固定版本并实际使用。它不是 NTO 基线代码 |
| 公开端到端例程 | [CSDecomp Kinova 最短时间示例](https://github.com/wernerpe/csdecomp/blob/7661a97bb7c4931be73d2bc4f8c45c3094ff476e/examples/kinova_mintime_planning.py) | 公开了 Kinova 的 DRM→EI-ZO→SCSTO 例程；没有因此找到原文 FR3/Forest 全套基准 |
| KUKA iiwa 本体模型 | [作者的 iiwa_description](https://github.com/wernerpe/iiwa_description) | iiwa 7/14 的模型资源公开；模型文件不能替代相机标定、障碍点云、时间戳和实机运行日志 |
| 原始 400 题列表与随机输入 | 主库历史、分支、fork、作者其他相关仓库 | 本次未找到可确认的原始资产 |
| 原始 NTO 控制点、约束位置和调参配置 | GitHub 代码检索及主库历史 | 本次未找到；已有 Drake/SNOPT 的一般实现不能补全作者具体参数 |
| KUKA 相机和实机记录 | 主库、相关仓库、[论文项目页](https://sites.google.com/view/GPUPolytopes) | 项目页提供论文、代码和演示视频；本次未发现可下载的原始传感器/执行记录 |

这些可用模型允许进一步建设明确标注的 FR3 或 KUKA 仿真重建。若目标是匹配原文
表格，还需要相同球心/半径、碰撞过滤、起终点/障碍题单和 NTO 参数；仅获得同型号
机器人的 URDF，或自行拟合为 33 个球，都不能建立这种等价性。

## 实际检查范围

- 主库 [wernerpe/csdecomp](https://github.com/wernerpe/csdecomp)：10 个公开分支、
  3 个标签、144 个可达上游提交；API 返回的公开分支 SHA 与本地审计版本全部一致。
  检查了历史路径和相关内容，未查到原始 FR3/Forest/NTO 文件；没有 GitHub Release
  附件可补充这些资产。
- 3 个公开 fork：[ravvas/csdecomp](https://github.com/ravvas/csdecomp)、
  [yucnet/csdecomp](https://github.com/yucnet/csdecomp)、
  [ersolog/esolo_csdecomp](https://github.com/ersolog/esolo_csdecomp)。
  检查默认树及分支；额外分支是已有功能分支和 DRM 修复，没有确认到原始基准资产。
- 列出作者 42 个公开仓库，重点检查 `iris_benchmarks`、`cvisiris_examples`、
  `drake-franka-setup`、`robot_description_utils`、`cspace_utils`、`pybmtp`、
  `visiris`、`convex-obstacles-vcc` 等项目；再检查相关项目和 SCSTO 的
  20 个非默认分支树。
- `iris_benchmarks` 确实含有 IRIS 的题单和结果文件，但这些属于另一个实验协议；
  没有证据将它们认定为本论文的 Forest/FR3 数据。
- `robot_description_utils` 有 SDF→URDF 工具；`drake-franka-setup` 有接口、IK
  及子模块指针。这些有用代码不等同于原文实验资产。
- 使用已登录的 GitHub API 做 `fr3`、`franka`、`forest`、`pycsdecomp`、
  `KinematicTrajectoryOptimization`、`collision_spheres` 等代码/仓库/issue 检索，
  并用官方项目页和 GitHub 网页交叉核查。

GitHub 代码索引并不完整，也主要针对默认分支，所以对相关项目补查了目录树。
其他项目中的二进制结果文件没有被当作原论文数据载入；这轮也没有执行第三方
候选仓库中的代码。两个猜测的 README 路径返回了 404，随后通过目录树取得真实的
工具文件和子模块文件；失败请求同样保留在检索记录中。

## 已保存的证据

- [结构化结论和候选文件校验值](github_asset_search_summary.json)：来源 URL、固定
  revision、下载结果、SHA-256 和球数量核查。
- [完整 API/命令快照](github_asset_search_evidence.json.gz)：67 份 JSON 记录，解压
  后可复查查询语句、返回结果、仓库树、分支和失败请求。
- 本机原始响应：`reproduction/paper/github_asset_search/`。
- 本机候选源文件：`reproduction/vendor/github_assets/`，仅用于查看和结构核查。
  它们未替换主实验模型，也未加入环境依赖。

原先 400 题及补充实验的结果保持不变。本轮未新建分支、未向外部仓库提交内容，
也未联系作者。
