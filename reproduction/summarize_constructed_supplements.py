"""Summarize separately executed diagnostics without changing primary outcomes."""
import argparse
from collections import Counter
import hashlib
from pathlib import Path
from reproduction.common import ROOT, atomic, read

OUT = ROOT / "reproduction/reports/paper_constructed"


def optional(path):
    return read(path) if Path(path).exists() else None


def evidence_files(run, names):
    return {name: hashlib.sha256((run / name).read_bytes()).hexdigest()
            for name in names if (run / name).is_file()}


def official_replay_section(run):
    summary = optional(run / "summary.json")
    failure = optional(run / "failure.json")
    recheck = optional(run / "input_reverification.json")
    item = dict(run_dir=str(run), summary=summary, failure=failure,
                input_reverification=recheck,
                evidence_sha256=evidence_files(run, ["summary.json", "failure.json",
                    "input_identity.json", "input_reverification.json", "trajectory_inventory.json"]))
    lines = ["### 对官方几何的保存轨迹复核", ""]
    if summary is None:
        lines += [f"未生成完整汇总；状态为 `{(failure or {}).get('status', 'missing_summary')}`。"
                  "部分记录不能当作已完成复核，失败详情保留在补充索引。", ""]
        return item, lines
    verified = (summary["status"] in {"completed", "completed_with_errors"}
                and failure is None and (recheck or {}).get("passed") is True)
    item["inputs_reverified"] = verified
    lines += [f"实际状态 `{summary['status']}`；输入结束复核{'通过' if verified else '未通过或缺失'}；"
              f"范围{'不完整' if summary['incomplete_scope'] else '覆盖所有保存候选文件'}。"
              "命令退出码0本身不等于查询无错误或轨迹无碰撞。", "",
              f"保存候选文件{summary['all_trajectory_files']}条，选中{summary['selected_trajectory_files']}条，"
              f"完成查询{summary['checked_trajectories']}条，查询错误{summary['errors']}条。"
              f"未保存轨迹的案例方法组合{summary['missing_trajectory_files']}个仍保留在清单。"
              f"每条采用17个均匀轴位置附近的真实保存行，按每条轨迹去重后累计{summary['unique_saved_rows']}行。"
              "距离路径使用归一化保存参数，时间轨迹使用秒；并非所有轴位置都是物理时刻。"
              "非有限时间的附加反例行排除，重复选择和偏移逐项保存。", ""]
    if not verified:
        lines += ["以下数值仅转录未完成身份复核的保存汇总，不作为已核验结论。", ""]
    lines += ["| 方法 | 完成查询候选 | 参考无负距候选 | 参考有负距候选 | 精确基元负距点 | 仅含凸包的未决点 | 近接触点 |",
              "|---|---:|---:|---:|---:|---:|---:|"]
    for method, row in summary["methods"].items():
        lines.append(f"| {method} | {row['checked_trajectories']} | {row['reference_free_trajectories']} | "
                     f"{row['reference_collision_trajectories']} | {row.get('exact_primitive_collision_points', '未记录')} | "
                     f"{row.get('hull_only_collision_candidate_points', '未记录')} | {row.get('near_contact_points', '未记录')} |")
    lines += ["", "点计数按17个选中位置累计，可能包含同一保存行的重复选择；近接触类别可与负距类别重叠。"
              "参考负距不是原非凸网格的真实碰撞率：凸包穿透可能来自凹部填充，相关情况保留为未决。"
              "这些是补充抽样结果，不是整条轨迹的连续安全证明；原CUDA/球模型验证标志保持不变。", ""]
    return item, lines


def hardware_section(run):
    manifest = optional(run / "manifest.json")
    summary = optional(run / "summary.json")
    roadmap = optional(run / "roadmap/roadmap.json")
    item = dict(run_dir=str(run), scope="constructed_simulation_only", manifest=manifest,
                summary=summary, roadmap=roadmap,
                evidence_sha256=evidence_files(run, ["manifest.json", "summary.json", "roadmap/roadmap.json"]))
    lines = ["### 论文硬件条件的构造仿真", ""]
    if manifest is None:
        lines += ["指定目录缺少运行manifest，不能确认任何已执行试验；保留该未完成状态。", ""]
        return item, lines
    trials = manifest.get("trials", [])
    cfg = manifest.get("config", {})
    rows = []
    for trial in trials:
        name = f"trials/{trial:02d}/result.json"
        row = optional(run / name)
        rows.append(row or dict(index=trial, status="missing_record", executed_in_virtual_state=False))
        item["evidence_sha256"].update(evidence_files(run, [name]))
    counts = dict(Counter(row.get("status", "missing_status") for row in rows))
    successes = sum(row.get("executed_in_virtual_state") is True for row in rows)
    full_declared = cfg.get("roadmap_nodes") == 35000 and trials == list(range(15))
    recorded = sum(row.get("status") != "missing_record" for row in rows)
    actual_nodes = (roadmap or {}).get("actual_nodes")
    consistency = []
    if manifest.get("matches_declared_full_size") != full_declared:
        consistency.append("manifest full-size flag disagrees with requested nodes/trials")
    if summary is not None and (summary.get("requested") != len(trials)
            or summary.get("strict_virtual_successes") != successes
            or summary.get("case_status_counts") != counts):
        consistency.append("summary differs from actual requested trial records")
    if manifest.get("physical_experiment") is not False:
        consistency.append("manifest lacks the explicit nonphysical scope")
    item.update(actual_requested_trials=len(trials), actual_recorded_trials=recorded,
                actual_strict_virtual_successes=successes, actual_trial_status_counts=counts,
                declared_full_profile=full_declared, consistency_issues=consistency)
    lines += [f"运行状态 `{manifest.get('status', 'missing_status')}`；"
              f"预定{len(trials)}个场景、已保存{recorded}个试验结果，严格验证后推进虚拟状态{successes}次。"
              f"请求路线图{cfg.get('roadmap_nodes', '未记录')}节点，实际建图节点{actual_nodes if actual_nodes is not None else '未完成/未记录'}，"
              f"建图状态 `{(roadmap or {}).get('status', 'missing_record')}`。", "",
              f"配置{'符合35000节点和顺序0–14的15场景预定规模' if full_declared else '是子集/烟测或其他非完整规模'}。"
              "预定完整规模、运行记录齐全和全部轨迹成功是三个不同条件，不能互相替代。"
              f"实际试验状态：`{counts}`。", ""]
    if consistency:
        lines += [f"汇总一致性问题：`{consistency}`；保存计数不作为完整通过结论。", ""]
    if summary is None:
        lines += ["没有完整summary，以上只转录现存manifest和逐试验结果，未宣称补充运行完成。", ""]
    lines += ["左右目标按固定日程交替；失败后虚拟状态保持不动，因此下一目标可能与当前状态同侧，"
              "不能将15个调度项称为15次完成的左右穿越。"
              "合成点云、公开iiwa/WSG模型和数值状态推进不代表RealSense感知、动力学执行或物理机械臂实测。"
              "配置与替代范围见[协议说明](../../hardware_constructed/README.md)，实际文件身份见补充索引。", ""]
    report_audit = optional(OUT / "hardware_simulation/report_audit.json")
    report_path = OUT / "hardware_simulation/report.md"
    if report_audit and Path(report_audit["run_dir"]).resolve() == run and report_path.exists():
        if hashlib.sha256(report_path.read_bytes()).hexdigest() != report_audit["report_sha256"]:
            raise ValueError("Hardware report no longer matches its saved audit")
        item["report_audit_status"] = report_audit["status"]
        lines += ["逐项状态推进、实体检查和计时分母见[硬件条件仿真报告](hardware_simulation/report.md)。"
                  f"保存记录审计状态为 `{report_audit['status']}`。", ""]
    region_metric = (summary or {}).get("metrics", {}).get("regions", {})
    if region_metric.get("recorded") == len(trials) and region_metric.get("mean") == 1 and region_metric.get("sample_sd") == 0:
        lines += ["本次所有试验最终都使用一个凸区域；它验证了完整流水线的执行，"
                  "不说明构造场景与论文实机任务具有相同难度或需要相同的绕障行为。", ""]
    return item, lines


def mesh_section(run):
    summary = optional(run / "summary.json")
    failure = optional(run / "failure.json")
    controls = optional(run / "synthetic_controls.json")
    recheck = optional(run / "input_reverification.json")
    item = dict(run_dir=str(run), summary=summary, failure=failure, synthetic_controls=controls,
                input_reverification=recheck,
                evidence_sha256=evidence_files(run, ["summary.json", "failure.json", "synthetic_controls.json",
                    "input_identity.json", "input_reverification.json", "frozen_input_inventory.json"]))
    lines = ["### 原始三角网格的局部数值诊断", ""]
    if summary is None:
        lines += [f"未生成完整诊断汇总；状态 `{(failure or {}).get('status', 'missing_summary')}`。"
                  "失败后的部分逐点结论不能用于消除原未决候选。", ""]
        return item, lines
    if summary.get("status") == "synthetic_controls_only":
        lines += [f"仅执行解析控制例；控制检查通过值为 `{(controls or {}).get('passed')}`，未评估真实网格候选。", ""]
        return item, lines
    verified = (failure is None and (controls or {}).get("passed") is True
                and (recheck or {}).get("passed") is True
                and summary.get("status") in {"completed_numerical_diagnostic", "completed_with_unresolved"})
    item["controls_and_inputs_verified"] = verified
    lines += [f"状态 `{summary.get('status', 'missing_status')}`；解析控制与输入复核{'通过' if verified else '未通过或缺失'}。"
              f"共{summary.get('unique_configurations', '未记录')}个固定配置，"
              f"对应{summary.get('original_candidate_occurrences', '未记录')}次原凸包候选出现，"
              f"未决{summary.get('unresolved_configurations', '未记录')}个。", "",
              "| 固定配置 | 记录状态 | 数值判据结论 | 表面距离(m) |",
              "|---|---|---|---:|"]
    for row in summary.get("results", []):
        distance = row.get("surface_distance", {}).get("distance")
        lines.append(f"| {row['sample_id']} | `{row['status']}` | `{row.get('conclusion', 'unresolved_exception')}` | "
                     f"{distance if distance is not None else '未完成'} |")
    lines += ["", "这些是自定义float64相交、拓扑与包含检查的局部结果，不是精确算术证书。"
              "近接触、拓扑/自交筛查异常、多组件实体语义和绕数模糊项保持未决；"
              "数值分离只适用于该对网格的这些固定配置，不证明全部场景或轨迹无漏检。"
              "原主实验和原凸包审计的标志均不改写。", ""]
    if not verified:
        lines += ["身份或控制检查未通过时，上表仅保留已写出的诊断记录，不采信其分离/相交结论。", ""]
    return item, lines


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--enumeration", type=Path)
    parser.add_argument("--official-replay", type=Path)
    parser.add_argument("--hardware", type=Path)
    parser.add_argument("--mesh-diagnostic", type=Path)
    args = parser.parse_args()
    ledger = read(OUT / "verification_commands.json")
    index = {}
    lines = ["## 独立补充实验", "", "以下结果不替换主表中的失败，也不与主实验计时混合。", "",
             "### 碰撞吞吐", "",
             "| 环境 | 实测批数 | 每批配置数 | 百万配置/秒：均值±SD | 独立复核点 | 分歧 |",
             "|---|---:|---:|---:|---:|---:|"]
    throughput = {}
    for family in ("forest", "fr3"):
        rows = [r for r in ledger if r["label"] == "throughput_" + family]
        if len(rows) != 1:
            raise ValueError("Expected one recorded throughput command for " + family)
        execution = rows[0]
        run = Path(execution["run_dir"])
        manifest = read(run / "manifest.json")
        item = dict(run_dir=str(run), status=manifest["status"], execution=execution)
        if execution["exit_code"] == 0 and manifest["status"] == "completed":
            s = read(run / "summary.json")
            item["summary"] = s
            lines.append(f"| {family.upper()} | {s['batches']} | {s['configurations_per_batch']} | {s['mean_million_configurations_per_second']:.3f} ± {s['sample_sd']:.3f} | {s['verified_points']} | {s['model_disagreements']} |")
        else:
            lines.append(f"| {family.upper()} | 未完整完成：{manifest['status']} | — | — | — | — |")
        throughput[family] = item
    atomic(OUT / "throughput_index.json", throughput)
    lines += ["", "包含公开同步接口的分配、主机/设备传输及布尔转换；采样、哈希、独立核验和文件写入不计入。"
              "每环境10个场景，每场景先独立预热再测5批。硬件和碰撞模型与论文不同，不据此宣称论文加速倍数。", ""]
    if args.enumeration:
        run = args.enumeration.resolve()
        m, rows = read(run / "manifest.json"), read(run / "results.json")
        valid = sum(r.get("result", {}).get("independent_validation_passed", False) for r in rows)
        solved = sum(r.get("result", {}).get("numerical_global_optimum_established", False) and r["exit_code"] == 0 for r in rows)
        both = sum(r.get("result", {}).get("numerical_global_optimum_established", False)
                   and r.get("result", {}).get("independent_validation_passed", False)
                   and r["exit_code"] == 0 for r in rows)
        index["lgcs_enumeration"] = dict(run_dir=str(run), attempted=len(rows), numerical_optimum=solved,
            independent_valid=valid, optimal_and_independent_valid=both, timeouts=m.get("timed_out"),
            original_results_unchanged=True)
        lines += ["### 许可证受限LGCS的独立检查", "",
            f"对{len(rows)}个已保存的受限问题枚举简单区域路径，再用Clarabel求各走廊SOCP。"
            f"{solved}个完成全部路径且未留下未决SOCP；{valid}个候选独立验证通过；两者同时满足{both}个。"
            f"超时{m.get('timed_out')}个。全部路径穷尽与数值可行性同时满足时才声明数值全局解。"
            "原主表的blocked_license不改写；该检查未重做区域膨胀/恢复，耗时不代表Gurobi性能。", ""]
        atomic(OUT / "lgcs_enumeration_results.json", index["lgcs_enumeration"])
    if args.official_replay:
        run = args.official_replay.resolve()
        index["official_replay"], section = official_replay_section(run)
        lines += section
    if args.hardware:
        run = args.hardware.resolve()
        index["hardware_simulation"], section = hardware_section(run)
        lines += section
    if args.mesh_diagnostic:
        index["mesh_diagnostic"], section = mesh_section(args.mesh_diagnostic.resolve())
        lines += section
    atomic(OUT / "supplementary_index.json", index)
    (OUT / "supplementary_results.md").write_text("\n".join(lines))
    print(OUT / "supplementary_results.md")


if __name__ == "__main__":
    main()
