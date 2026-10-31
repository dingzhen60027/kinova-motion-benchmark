"""Generate the constructed-benchmark report from audited, saved observations."""
from pathlib import Path
import hashlib
import itertools
from reproduction.common import ROOT, atomic, read
from reproduction.paper_constructed.analyze import write_csv

REPORTS = ROOT / "reproduction/reports/paper_constructed"


def number(value):
    return "NA" if value is None else f"{value:.3f}"


def verify_completed_grid(summary, audit):
    """Require the actual full grid, not merely a completed pilot summary."""
    family = summary["environment"]
    run = Path(summary["run_dir"])
    manifest = read(run / "manifest.json")
    cfg = manifest["config"]
    expected = set(itertools.product(cfg["environment_seeds"],
                                    cfg["roadmap_seeds"],
                                    cfg["roadmap_sizes"][family]))
    actual = [(r["environment_seed"], r["roadmap_seed"], r["roadmap_size"])
              for r in manifest["cases"]]
    if (summary["manifest_status"] != "completed" or manifest["status"] != "completed"
            or summary["requested"] != 400 or len(actual) != 400
            or len(expected) != 400 or set(actual) != expected
            or len({r["id"] for r in manifest["cases"]}) != 400
            or summary["source_integrity_issues"] or summary["artifact_issues"]
            or audit["issues"] or audit["cases"] != 400
            or Path(audit["run_dir"]).resolve() != run.resolve()
            or audit["environment"] != family or manifest["environment"] != family
            or {r["method"] for r in summary["methods"]} != set(cfg["method_order"])
            or len(summary["methods"]) != len(cfg["method_order"])
            or any(r["requested"] != 400 for r in summary["methods"])):
        raise ValueError("Requires the complete, independently audited 400-case grid: " + family)
    return manifest


def verify_geometry_evidence(out, manifest):
    """Connect fit, audit, current asset and the measured snapshot by SHA-256."""
    out = Path(out)
    index = read(out / "geometry_index.json")
    construction = out / index["construction"]
    validation = out / index["geometry_validation"]
    fitting = read(construction / "manifest.json")
    geometry = read(validation / "summary.json")
    identity = read(validation / "input_identity.json")
    asset_name = "reproduction/paper_constructed/assets/fr3_33.json"
    snapshot = Path(manifest["run_dir"]) / "source_snapshot" / asset_name
    hashes = [index["asset_sha256"], fitting["final_asset_sha256"],
              geometry["asset_sha256"], identity["asset_sha256"],
              manifest["source_files"][asset_name]]
    hashes += [hashlib.sha256(p.read_bytes()).hexdigest() for p in
               [ROOT / asset_name, snapshot, construction / "selected_asset.json"]]
    if (len(set(hashes)) != 1 or fitting["status"] != "completed"
            or geometry["status"] != "completed"
            or not identity["geometry_configuration_matches_preregistered"]):
        raise ValueError("Fit, geometry audit and measured FR3 asset identities differ")
    return index, fitting, geometry


def main():
    out = REPORTS
    summaries = {f: read(out / f / "summary.json") for f in ("forest", "fr3")}
    audits = {f: read(out / "audit" / f / "audit.json") for f in summaries}
    index = {}
    manifests = {}
    resumes = {}
    for family, summary in summaries.items():
        run = Path(summary["run_dir"])
        manifest = verify_completed_grid(summary, audits[family])
        manifests[family] = dict(manifest, run_dir=str(run))
        resume = read(out / "audit" / family / "resume_check.json")
        if (resume["exit_code"] != 0 or resume["changed_case_files"]
                or resume["case_files"] != audits[family]["case_files"]):
            raise ValueError("Report requires an actual unchanged completed-run resume: " + family)
        resumes[family] = resume
        index[family] = dict(run_dir=str(run), cases=summary["requested"],
                             implementation_hash=manifest["implementation_hash"],
                             source_manifest_sha256=hashlib.sha256((run / "manifest.json").read_bytes()).hexdigest())
    atomic(out / "run_index.json", index)
    geometry_index, fitting, geometry = verify_geometry_evidence(out, manifests["fr3"])
    coverage = read(out / geometry_index["construction"] / "geometry_audit.json")["selected"]
    triangle_count = sum(row["subtriangles"] for row in coverage.values())
    uncovered_triangles = sum(row["uncovered_subtriangles"] for row in coverage.values())
    interior_uncovered = sum(row["solid_uncovered_inside_probes"] for row in coverage.values())
    if uncovered_triangles or not all(row["continuous_surface_coverage_certified"] for row in coverage.values()):
        raise ValueError("The surface-coverage claim requires successful serialized triangle checks")
    static_pairs = geometry["conditions"]["static_table"]["sphere_pairs"]
    total_pairs = geometry["conditions"]["online_20_spheres"]["sphere_pairs"]
    if total_pairs - static_pairs != 660:
        raise ValueError("The all33 robot /20 obstacle count is not supported by the geometry audit")
    static_groups = geometry["conditions"]["static_table"]["summary"]
    online_groups = geometry["conditions"]["online_20_spheres"]["summary"]
    if (static_groups["home"]["counts"] != {"both_free": 1}
            or static_groups["old_q7_scan"]["counts"] != {"both_free": 31}
            or online_groups["home"]["counts"] != {"both_collide": 1}):
        raise ValueError("Saved home/q7 findings do not support the fixed-pose report text")
    lines = ["# CSDecomp：按论文条件构造实验并复现", "",
        "本轮采用用户授权的自建模型和题单，完成 Forest、FR3 各400个预定案例的调度和结果记录。"
        "五种方法按各案例的前置条件实际尝试，未执行的方法保留明确原因。"
        "所有失败、未执行原因、原始输入、地图、区域、轨迹和验证结果均保留。"
        "这里复现的是按公开论文条件构造的实验，不冒充作者未公开的原始题单或 KUKA 实机记录。", "",
        "协议为 `paper_constructed_v5`，沿用现有 Ubuntu22.04 Conda 环境，只使用 `main`。"
        "v1–v4的代码和结果未覆盖。论文与构造选择逐项见[实验合同](experimental_contract.md)，"
        "执行方法见[运行指南](../../paper_constructed/README.md)。", "",
        "## 模型构造和独立核验", "",
        f"新模型保持7自由度和33个碰撞球。按官方碰撞几何选择球分配、中心和半径，选择不读取规划结果。"
        f"球体积之和从{fitting['baseline_sphere_volume_sum']:.6f}降到{fitting['sphere_volume_sum']:.6f}m³；"
        "它是球体积之和，包含重叠，不是机器人实体体积。四次构造尝试及其中两次失败全部保存。"
        "[构造过程](model_construction.md)给出每个链接的覆盖、最大及95分位外扩误差。", "",
        f"{triangle_count:,}个细分面片均被至少一个球完整覆盖；固定内部采样发现{interior_uncovered}个未覆盖点。"
        "这证明该离散三角网格的表面覆盖，尚不是连续实体体积覆盖证书。"
        "静态条件下，home与旧31点关节7扫描在两个模型中均无碰撞；"
        "加入固定在线障碍后home在两个模型中均碰撞。该结论限定于这些保存姿态，"
        "验收姿态未参与拟合选择。", "",
        f"固定基座仍为固定关节，但其碰撞球也参加动态障碍查询。实测{static_pairs}个静态候选对，"
        f"加上33×20=660个动态对，总计{total_pairs}；不人为增加碰撞对以凑论文的1187。"
        "官方SRDF及焊接过滤保留；详见[官方几何审计](geometry_audit.md)。", "",
        "| 固定姿态核验条件 | 双方无碰撞 | 双方碰撞 | 球模型额外报碰撞 | 仅官方凸包报碰撞的未决候选 | 查询失败 |",
        "|---|---:|---:|---:|---:|---:|"]
    for name, result in geometry["conditions"].items():
        c = result["summary"]["all"]["counts"]
        lines.append(f"| {name} | {c.get('both_free',0)} | {c.get('both_collide',0)} | {c.get('sphere_false_positive',0)} | {c.get('reference_false_negative_candidate',0)} | {result['failures']} |")
    lines += ["", "每种条件1184点：1024个固定均匀关节样本、左右工作区各64点、home及31点旧扫描。"
        "正的官方凸包距离能证明原网格分离；凸包穿透可能来自填平凹部，不能直接认定原非凸网格碰撞。"
        "因此最后一类保留为未决候选，不能把它们改写为零漏检。", "",
        "## 按论文规模执行的结果", "",
        "每个环境使用10个环境索引×10个路线图种子×4种规模。FR3路线图为3000/6000/9000/12000节点，"
        "Forest为200/400/800/1600节点。FR3左右端点各有独立随机流，目标是左工作区无碰撞关节样本的FK，"
        "随后每个路线图在线执行位姿IK；不因规划或优化失败重新抽题。固定环境索引不是盲测数据。", "",
        "| 环境 | 预定案例 | 原生DRM成功 | 严格有效种子 | 案例状态 |", "|---|---:|---:|---:|---|"]
    for family, s in summaries.items():
        lines.append(f"| {family.upper()} | {s['requested']} | {s['native_drm_successes']} | {s['strict_seed_valid']} | `{s['semantic_case_status_counts']}` |")
    lines += ["", "| 环境 | 方法 | 实际尝试 | 求解成功 | CUDA无碰撞候选 | 严格有效 / 全部案例 | 计时数 / 墙钟回退数 | 已记录计时均值±SD(ms) |",
              "|---|---|---:|---:|---:|---:|---:|---:|"]
    reference = read(ROOT / "reproduction/reports/paper_protocol/paper_reference.json")
    comparison = []
    for family, s in summaries.items():
        for r in s["methods"]:
            lines.append(f"| {family.upper()} | {r['method']} | {r['attempted']} | {r['solver_successes']} | {r['native_cuda_free_trajectories']} | {r['validated_successes']} / {r['requested']} | {r['attempts_with_completed_timing']} / {r['exception_wall_timings']} | {number(r['total_ms_mean_all_attempts'])} ± {number(r['total_ms_sd_all_attempts'])} |")
            ref = reference[family][r["method"]]
            comparison.append(dict(environment=family, method=r["method"], requested=r["requested"],
                native_drm_successes=r["native_drm_successes"], actual_attempts=r["attempted"],
                actual_solver_success_count=r["solver_successes"], actual_cuda_free_count=r["native_cuda_free_trajectories"],
                actual_strict_success_count=r["validated_successes"],
                actual_completed_timing_count=r["attempts_with_completed_timing"],
                actual_exception_wall_timing_count=r["exception_wall_timings"],
                actual_case_timeout_count=s["case_status_counts"].get("timeout", 0),
                actual_solver_success_given_drm=r["solver_success_rate_given_native_drm"],
                actual_cuda_cfr_given_drm=r["native_cuda_cfr_given_drm"], actual_strict_success_given_drm=r["validated_rate_given_native_drm"],
                paper_solver_success_given_drm=ref["solver_success_rate"], paper_cuda_cfr_given_drm=ref["collision_free_rate"],
                actual_cost_validated=r["cost_mean_validated"], actual_cost_solver_success=r["cost_mean_solver_success"],
                paper_cost=ref["cost"], paper_cost_population="solver_success_given_native_drm",
                cost_unit="s" if r["method"] in ["SCSTO", "NTO_time"] else ("m" if family == "forest" else "rad"),
                actual_total_ms=r["total_ms_mean_all_attempts"], paper_total_ms=ref["total_ms"],
                actual_timing_scope=s["timing_scope"],
                identical_original_inputs=False, directly_comparable_hardware=False))
    write_csv(out / "paper_vs_actual.csv", comparison)
    fallbacks = "; ".join(
        f"{family.upper()} {r['method']} {r['exception_wall_timings']}条"
        for family, summary in summaries.items() for r in summary["methods"]
        if r["exception_wall_timings"])
    timeout_cases = sum(s["case_status_counts"].get("timeout", 0) for s in summaries.values())
    interrupted_attempts = sum(r["interrupted_attempts_without_timing"]
                              for s in summaries.values() for r in s["methods"])
    lines += ["", f"计时表中的异常墙钟回退：{fallbacks or '0条'}。这些记录包含在上述均值中，"
              "不能将混合均值全部称为正常TO+CS时间。"
              f"本次实际案例超时{timeout_cases}例、没有完整计时的中断方法{interrupted_attempts}次；"
              "共享预算与固定顺序仍是协议限制，实际删失次数按这里的记录解释。", ""]
    lines += ["", "CUDA无碰撞候选可来自未成功的求解器；严格有效还要求求解成功、端点、关节域、"
        "独立几何以及适用的速度/加速度检查。轨迹的独立Drake检查采用同一声明的球模型，"
        "并非每条轨迹都经过官方非凸网格接触验证；1184姿态的官方几何审计是另外一项检查。"
        "主表保留全400分母，论文DRM条件率与参考数值另见"
        "[逐项对照](paper_vs_actual.csv)。[图表](../../figures/paper_constructed/)展示全部案例状态及每环境按编号选定的首个案例回放；"
        "其余轨迹保留在原始运行目录。", "",
        "计时为构建加优化，离线地图、在线IK、最终独立验证和外围回放写入不在正常TO+CS内；"
        "备用地图IO、备用路径检查与恢复记录写入实际包含在构建时间。异常墙钟回退单独标记。"
        "固定五方法顺序共享240秒案例预算。LGCS和NTO配置60秒求解器限制；"
        "LSCS与SCSTO没有单独的60秒限制，依赖案例watchdog。恢复最多20轮，可能再次调用求解器。"
        "超时后的未开始方法属于预算删失，"
        "不会伪记为求解器失败。均值/SD包含有完整计时的失败尝试，SD使用ddof=1；"
        "成对代价仅比较共同严格有效案例，见每环境paired.csv，不能用边际均值宣称论文加速倍数。", ""]
    review_md = out / "posthoc_result_review.md"
    if review_md.exists():
        review = read(out / "posthoc_result_review.json")
        if (review.get("passed") is not True or review.get("verification_exit_code") != 0
                or hashlib.sha256(review_md.read_bytes()).hexdigest() != review["markdown_sha256"]
                or any(hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != expected
                       for name, expected in review["evidence_sha256"].items())):
            raise ValueError("Posthoc result review is unsuccessful or its evidence has changed")
        lines += [review_md.read_text(), ""]
    lines += ["## 失败、续跑和回放核验", ""]
    for family, s in summaries.items():
        lines.append(f"**{family.upper()}**")
        lines.append("")
        for r in s["methods"]:
            lines.append(f"- {r['method']}：`{r['status_counts']}`；恢复触发{r['recovery_triggered']}次。")
        a = audits[family]
        lines += ["", f"按每个保存候选17个固定等间隔样点独立重查，共{a['independently_rechecked_fixed_points']}点；审计问题{len(a['issues'])}项。"
                  "这是密集验证结果之外的固定点复核，未重新验证每一条曲线的全部连续时刻。"
                  f"原始密集验证保留的模型边界分歧为{a['full_saved_model_disagreements']}个样点。"
                  f"完成后的续跑退出码为{resumes[family]['exit_code']}，逐个比较{resumes[family]['case_files']}个案例文件，变化0个。"
                  "这是已完成运行的续跑保持性检查，不能替代新的人为中断恢复实验。", ""]
    lines += ["NTO采用正式PDF给出的完整Nc点数（20/100/150），位置k/(Nc+1)是声明的构造选择。"
        "距离样条保留原始DRM节点；时间样条只增加静止端点所需的重复控制点。"
        "碰撞阈值不放宽，微小穿透也保留为失败。", "",
        "## 实际环境、命令和复用", "",
        "使用 `.conda/envs/csdecomp` 和 `bash local/python.sh`。环境见[清单](environment.json)，"
        "命令与退出码见本目录 `*_commands.json`；所有版本和计算源码哈希也冻结在各run manifest。"
        "[运行索引](run_index.json)给出主实验原始目录。模型构造、pilot、正式运行及补充诊断不混池。", "",
        "```bash", "bash local/python.sh -m reproduction.run_constructed",
        "bash local/python.sh -m reproduction.paper_constructed.run resume --run-dir <RUN_DIR>",
        "bash local/python.sh -m reproduction.paper_constructed.analyze --run-dir <FOREST_RUN> <FR3_RUN> --output reproduction/reports/paper_constructed --audit --collision-depths",
        "bash local/python.sh -m reproduction.audit_paper_constructed --run-dir <FOREST_RUN> <FR3_RUN> --output reproduction/reports/paper_constructed/audit --resume",
        "```", "",
        "## 解释边界与下一阶段重点", "",
        "自建资产已让公开实验流水线可以实际比较五种方法，但无法要求随机重建题单得到作者相同数值。"
        "主要差异包括33球的具体布局、障碍/目标分布、静态碰撞对、未公开的样条与求解细节、硬件与软件版本。"
        "v5相较v4同时修改球模型、固定基座动态检查、NTO采样网格和端点随机流；"
        "跨版本差异不构成只改变模型的消融，也不是同端点配对，不能将成功率变化全部归因于模型修复。"
        "Clarabel代替未许可的MOSEK；受限Gurobi许可证遇到超规模LGCS时保留失败。"
        "已构建的论文日期原生核心只作先前单独核验，本轮主实验仍使用csdecomp0.1.2。", "",
        "下一阶段优先核查保存轨迹上的凸包分歧、实体覆盖与NTO碰撞离散化差异，取得全尺寸LGCS求解能力，"
        "并在新预注册场景上检验方法结论。KUKA硬件需真实机械臂、标定和传感器数据；"
        "本轮iiwa完整规划仿真与碰撞吞吐诊断不能充当实机复现。", ""]
    extra = out / "supplementary_results.md"
    if extra.exists():
        lines += [extra.read_text(), ""]
    (out / "reproduction_report.md").write_text("\n".join(lines).rstrip() + "\n")
    print(out / "reproduction_report.md")


if __name__ == "__main__":
    main()
