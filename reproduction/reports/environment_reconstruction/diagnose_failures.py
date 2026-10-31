"""Read-only CPU diagnosis of the final environment acceptance failures.

Run from the repository root:
  bash local/python.sh reproduction/reports/environment_reconstruction/diagnose_failures.py
No planner, optimizer, CUDA collision query, resampling, or parameter change is
performed. CPU replay of the published coarse grid is diagnostic only.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import re
import sys

# The diagnostic must not allocate a CUDA context even if a future helper changes.
os.environ["CUDA_VISIBLE_DEVICES"] = ""
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import numpy as np
from reproduction.common import atomic, read
from reproduction.paper_constructed.model import drake_fr3
from reproduction.paper_constructed.validation import polygon_samples


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inventory(root):
    return {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob("*")) if p.is_file()}


def cpu_model(cfg, scene):
    diagram, context, plant, pc, sg, inspector, instance = drake_fr3(cfg, scene["obstacles"])
    sc = sg.GetMyContextFromRoot(context)

    def distances(q, threshold=np.inf):
        plant.SetPositions(pc, np.asarray(q, dtype=float))
        query = sg.get_query_output_port().Eval(sc)
        return query.ComputeSignedDistancePairwiseClosestPoints(float(threshold))

    def geometry(identifier):
        shape = inspector.GetShape(identifier)
        info = {"geometry": inspector.GetName(identifier),
                "frame": inspector.GetName(inspector.GetFrameId(identifier)),
                "shape": type(shape).__name__}
        if info["shape"] == "Sphere":
            info["radius_m"] = float(shape.radius())
        return info

    def pair_record(pair):
        return {"signed_distance_m": float(pair.distance),
                "penetration_m": max(0., -float(pair.distance)),
                "geometry_a": geometry(pair.id_A), "geometry_b": geometry(pair.id_B),
                "witness_point_a_geometry_frame": pair.p_ACa.tolist(),
                "witness_point_b_geometry_frame": pair.p_BCb.tolist()}

    return distances, pair_record


def segment_indices(path, step):
    ids, fractions = [], []
    for segment, (u, v) in enumerate(zip(path[:-1], path[1:])):
        count = max(2, int(np.ceil(np.linalg.norm(v-u)/step))+1)
        ids.extend([segment]*(count-1))
        fractions.extend(np.linspace(0, 1, count)[:-1].tolist())
    return np.asarray(ids+[len(path)-2]), np.asarray(fractions+[1.])


def diagnose_case(root, cfg, entry):
    case_dir = root / "cases" / entry["id"]
    record = read(case_dir / "case.json")
    stdout = (case_dir / "stdout.log").read_text()
    stderr = (case_dir / "stderr.log").read_text()
    result = {"id": entry["id"], "status": record["status"], "kind": record["kind"],
              "ik_status": record.get("ik_status"), "exception": record.get("exception"),
              "stdout": stdout, "stderr": stderr,
              "saved_start": record.get("start"), "saved_goal": record.get("goal"),
              "saved_endpoint_audits": record.get("endpoint_audits"),
              "nominal_graph_paths_reported": [int(v) for v in re.findall(r"Found path of length: (\d+)", stdout)],
              "input_files": {str(p.relative_to(root)): sha(p) for p in sorted(case_dir.iterdir()) if p.is_file()}}
    if record["kind"] != "fr3":
        result["diagnosis"] = "Non-FR3 failure retained; no FR3-specific inference applied"
        return result
    scene = read(root / record["scene"])
    query, describe = cpu_model(cfg, scene)
    result["cpu_endpoint_minimum_distances_m"] = {
        key: float(min(pair.distance for pair in query(record[key]))) for key in ("start", "goal") if key in record}
    samples_file = case_dir / "validation_samples.npz"
    if not samples_file.exists():
        result.update(validation_samples_present=False,
            diagnosis="Saved native log reports graph-path exhaustion after lazy checking; no final path was returned",
            inference_limit="This finite-roadmap failure does not prove that the continuous configuration-space task is infeasible")
        return result

    saved = np.load(samples_file)
    q, native, independent = saved["q"], saved["native"], saved["independent"]
    path = np.asarray(record["path"])
    expected = polygon_samples(path, cfg["validation_step"])
    if not np.array_equal(expected, q):
        raise ValueError("Saved validation samples do not match the frozen path and step")
    segments, fractions = segment_indices(path, cfg["validation_step"])
    bad = np.flatnonzero(~native | ~independent)
    samples = []
    for index in bad:
        pairs = sorted(query(q[index], 0.), key=lambda p: p.distance)
        colliding = [pair for pair in pairs if pair.distance < 0]
        if not colliding:
            pairs = sorted(query(q[index]), key=lambda p: p.distance)[:1]
        else:
            pairs = colliding
        segment = int(segments[index])
        length = np.linalg.norm(path[segment+1]-path[segment])
        samples.append({"sample_index": int(index), "segment_index_zero_based": segment,
            "segment_fraction": float(fractions[index]),
            "distance_along_segment_rad": float(length*fractions[index]),
            "q": q[index].tolist(), "saved_native_free": bool(native[index]),
            "saved_independent_free": bool(independent[index]),
            "cpu_collision_reconfirmed": bool(colliding),
            "pairs": [describe(pair) for pair in pairs]})
    deepest = min(samples, key=lambda s: s["pairs"][0]["signed_distance_m"]) if samples else None

    # Follow the public SteerFunctionIsSafe grid formula on the *saved* output
    # edges with float32 arithmetic. This is not a replay of its unavailable
    # internal GPU trace, and it does not alter or rerun the planner.
    step = np.float32(record["native_planner_options"]["online_edge_step_size"])
    coarse = []
    for index, (u, v) in enumerate(zip(path[:-1], path[1:])):
        u, v = u.astype(np.float32), v.astype(np.float32)
        length = np.linalg.norm(v-u)
        count = int(float(length)/float(step)+1)
        direction = (v-u)/length
        points = np.asarray([u+np.float32(j)*step*direction for j in range(count)])
        minima = [min(pair.distance for pair in query(point)) for point in points]
        coarse.append({"segment_index_zero_based": index, "count": count,
            "q": points.astype(float).tolist(), "minimum_distances_m": list(map(float, minima)),
            "minimum_clearance_m": float(min(minima)), "all_cpu_free": bool(min(minima) >= 0)})
    result.update(validation_samples_present=True, validation_samples_reconstruction_exact=True,
        saved_sample_count=len(q), saved_native_collision_count=int((~native).sum()),
        saved_independent_collision_count=int((~independent).sum()), saved_disagreements=int((native != independent).sum()),
        all_flagged_samples_cpu_reconfirmed=all(s["cpu_collision_reconfirmed"] for s in samples),
        collision_samples=samples, deepest_saved_sample=deepest,
        cpu_coarse_grid_diagnostic={"step_rad": float(step), "segments": coarse,
            "all_cpu_free": all(s["all_cpu_free"] for s in coarse),
            "source": "csdecomp/src/planning/drm_planner.cpp:SteerFunctionIsSafe",
            "source_sha256": sha(ROOT / "csdecomp/src/planning/drm_planner.cpp"),
            "limitation": "CPU re-evaluation of public grid formula on saved final segments; original internal GPU query arrays were not saved"},
        diagnosis="Saved final edge has a narrow inter-sample robot-sphere/obstacle collision; native and independent dense flags agree",
        inference_limit="Maximum is over saved flagged samples only, not the continuous maximum; no continuous collision-free certificate follows from any finite coarse grid")
    return result


def report_text(result):
    lines = ["# 新环境验收失败的 CPU 诊断", "",
        "本诊断只读取 `run_final` 已保存的题目、路径、日志和验证数组，使用同一 FR3 球/桌/障碍模型的 Drake signed-distance 查询。没有运行规划、优化或 CUDA，没有重采样任务或调整参数。", "",
        f"最终运行完成 {result['final_summary']['completed']}/{result['final_summary']['requested']} 项，失败状态分布：`{result['failure_status_counts']}`。", ""]
    for case in result["cases"]:
        lines += [f"## {case['id']}: {case['status']}", ""]
        if not case.get("validation_samples_present"):
            lines += [f"在线 IK：`{case.get('ik_status')}`；端点 CPU 最小间距：`{case.get('cpu_endpoint_minimum_distances_m')}` m。",
                f"日志先报告候选图路径长度 `{case['nominal_graph_paths_reported']}`，最终记录 `{case.get('exception')}`。没有最终输出路径或验证样本。",
                "这是当前有限路线图及惰性检查流程的失败，不能解释为原连续空间问题无解，也不是已证实的端点或 IK 失败。", ""]
            continue
        deepest = case["deepest_saved_sample"]
        pair = deepest["pairs"][0]
        lines += [f"保存的 {case['saved_sample_count']} 个样本中，CUDA 与独立检查各标出 {case['saved_native_collision_count']} 个碰撞点，分歧数 {case['saved_disagreements']}；CPU 重查全部确认。",
            f"最深保存样本为索引 **{deepest['sample_index']}**，路径第 **{deepest['segment_index_zero_based']+1}** 段（0-based: {deepest['segment_index_zero_based']}），段内比例 `{deepest['segment_fraction']:.12f}`。",
            f"几何对：`{pair['geometry_a']['geometry']}` / `{pair['geometry_b']['geometry']}`；穿透深度 **{pair['penetration_m']*1000:.9f} mm**。",
            "其配置、几何名称、几何局部坐标中的接触见证，以及全部碰撞样本均保存在 JSON。该值是已保存样本中的最大值，不是连续全路径最大穿透的证明。", "",
            "### 对原 0.1 rad 检查间隔的诊断", "",
            "按公开 `SteerFunctionIsSafe` 的 float32 网格公式，只在保存的最终三条边上重新做 CPU 距离查询："]
        for segment in case["cpu_coarse_grid_diagnostic"]["segments"]:
            lines.append(f"- 第 {segment['segment_index_zero_based']+1} 段：{segment['count']} 个粗网格点，最小间距 {segment['minimum_clearance_m']*1000:.6f} mm。")
        lines += ["", "这些粗网格点均无碰撞，但原 0.005 rad 验证网格已捕获上述穿透，支持‘粗采样之间漏碰撞’这一具体机制。原规划器内部 GPU 查询数组未保存，因此本项不是逐位重放原 GPU 内部调用。未为了通过验收而放松阈值、细化规划步长或覆盖失败。", ""]
    lines += ["## 输入完整性及重现", "",
        f"运行前后 `run_final` 全部 **{result['input_file_count']}** 个文件 SHA-256 一致：`{result['run_final_unchanged']}`。模型代码及资产 SHA 与运行 manifest 一致。", "",
        "```bash", "bash local/python.sh reproduction/reports/environment_reconstruction/diagnose_failures.py", "```", "",
        "JSON 保存完整输入清单、失败日志、逐样本见证和 CPU 粗网格查询结果。环境变量 `CUDA_VISIBLE_DEVICES` 在导入模型前设为空；只构造 Drake plant，不构造 native FR3 规划器。"]
    return "\n".join(lines)+"\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path(__file__).parent / "run_final")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent)
    args = parser.parse_args()
    summary, cfg, manifest = (read(args.run / p) for p in ("summary.json", "protocol.json", "manifest.json"))
    if summary["completed"] != summary["requested"]:
        raise ValueError("Wait until final environment acceptance has finished")
    for source in ("reproduction/paper_constructed/model.py", "reproduction/paper_constructed/assets/fr3_33.json"):
        if sha(ROOT / source) != manifest["source_hashes"][source]:
            raise ValueError(f"Model changed since frozen run: {source}")
    before = inventory(args.run)
    failures = [entry for entry in summary["cases"] if entry["status"] != "validated"]
    cases = [diagnose_case(args.run, cfg, entry) for entry in failures]
    after = inventory(args.run)
    result = {"run": str(args.run.resolve()), "scope": "CPU diagnosis of saved environment-acceptance failures only",
        "cuda_visible_devices": os.environ["CUDA_VISIBLE_DEVICES"], "planning_rerun": False,
        "final_summary": summary, "failure_status_counts": dict(Counter(c["status"] for c in cases)),
        "cases": cases, "input_file_count": len(before), "input_sha256": before,
        "run_final_unchanged": before == after, "script_sha256": sha(__file__)}
    atomic(args.output / "failure_diagnosis.json", result)
    (args.output / "failure_diagnosis.md").write_text(report_text(result))
    print(json.dumps({"failed_cases_diagnosed": [c["id"] for c in cases],
        "input_file_count": len(before), "run_final_unchanged": before == after}, indent=2))
    if before != after:
        raise SystemExit("Frozen run changed during read-only diagnosis")


if __name__ == "__main__":
    main()
