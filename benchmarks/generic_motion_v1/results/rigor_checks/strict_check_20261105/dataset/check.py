"""Fresh CPU audit of every frozen input; no planner or GPU invocation.

Run with the repository's local/python.sh.  The output path must be new.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[6]
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1 import dataset
from benchmarks.generic_motion_v1.generator.scene import generate_scene
from benchmarks.generic_motion_v1.generator.voxelize import voxelize
from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.reports.qualification.model_audit import ManualURDF, FreshDrake

BASE = ROOT / "benchmarks/generic_motion_v1"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def independent_cover(scene, config):
    """Rebuild source-intersecting cells with meshgrid, not voxelize helpers."""
    expected, stats = [], []
    for obj in scene["obstacle_geometry"]:
        center = np.asarray(obj["center_m"])
        sphere = obj["shape"] == "sphere"
        extent = np.repeat(2 * obj["radius_m"], 3) if sphere else np.asarray(obj["size_m"])
        counts = np.ceil(extent / config["maximum_voxel_cell_side_m"]).astype(int)
        sides = extent / counts
        axes = [(np.arange(counts[k]) + .5) * sides[k] - extent[k] / 2 for k in range(3)]
        local = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
        if sphere:
            closest = np.clip(np.abs(local) - sides / 2, 0, None)
            local = local[(closest * closest).sum(1) <= (obj["radius_m"] + 1e-14) ** 2]
        yaw = obj.get("yaw_rad", 0.)
        c, s = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        exact = np.einsum("ij,kj->ki", rotation, local) + center
        frozen = exact.astype(np.float32).astype(float)
        # Dot/einsum can differ at roundoff before float32; bound the actual
        # saved centres against these independently constructed cell centres.
        start = len(expected)
        count = len(local)
        saved = np.asarray(scene["voxels_xyz"], float).reshape(-1, 3)[start:start + count]
        require(saved.shape == frozen.shape, f"{scene['scene_id']}: missing cover cells")
        error = float(np.max(np.linalg.norm(saved - exact, axis=1), initial=0.))
        halfdiag = float(np.linalg.norm(sides / 2))
        transform_error = 32 * np.finfo(float).eps * float(np.linalg.norm(np.abs(center) + extent))
        margin = scene["voxel_radius_m"] - halfdiag - error - transform_error
        require(margin > 0, f"{scene['scene_id']}: uncovered source cell")
        stats.append(dict(obstacle_id=obj["obstacle_id"], cells=count,
                          cell_half_diagonal_m=halfdiag, centre_error_m=error,
                          conservative_coverage_margin_m=margin))
        expected.extend(frozen.tolist())
    require(len(expected) == len(scene["voxels_xyz"]), f"{scene['scene_id']}: extra/missing cells")
    return stats


def main(output):
    output = Path(output).resolve()
    require(not output.exists(), "Preserve previous evidence; choose a new output directory")
    output.mkdir(parents=True)
    started = time.perf_counter()
    report = dict(status="running", command=sys.argv,
                  git_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                  python=sys.executable, script_sha256=sha(__file__),
                  cuda_calls=0, optimizer_calls=0, errors=[], scenes=[], cases=[])
    protected = [BASE / item for item in ("problems.jsonl", "metadata.json", "splits.json", "features.csv", "configs/robot.json")]
    protected += sorted((BASE / "manifests").glob("*.jsonl"))
    protected += sorted((BASE / "receipts").glob("*_generation.json"))
    report["protected_files_before"] = {str(p.relative_to(ROOT)): sha(p) for p in protected}
    arrays = {}
    try:
        report["dataset_integrity"] = dataset.check(BASE, ROOT)
        rows = [r for r, _, _ in dataset.source_records(BASE)]
        configs = {split: dataset.read(BASE / f"configs/{split}_generator.json") for split in dataset.SPLITS}
        model, manual = Model(), ManualURDF()

        def forbidden_cuda(*args, **kwargs):
            raise AssertionError("CPU audit must never invoke CUDA")

        model.cuda = forbidden_cuda
        frozen_model = dataset.read(BASE / "configs/robot.json")
        require(model.inventory == frozen_model, "Current model differs from frozen complete inventory")
        names = [x["name"] for x in model.inventory["geometries"]]
        checked, excluded, world_links = manual.filter_pairs(names)
        require({tuple(x["pair"]) for x in checked} == {tuple(x) for x in model.pairs}, "Manual XML filter mismatch")
        require(model.joint_names == [x["name"] + "_q" for x in manual.active], "Joint order mismatch")
        require(np.allclose(model.lower, [x["lower"] for x in manual.active], atol=2e-7, rtol=0), "Manual joint limits mismatch")
        report["model"] = dict(robot_sha256=model.robot_sha256, collision_geometry_sha256=model.collision_geometry_sha256,
                               inventory_matches=True, xml_filter_pairs=len(checked), excluded_pairs=len(excluded),
                               primitives=len(names), moving=len(model.robot_geometry_ids),
                               joint_names=model.joint_names, periodic_joints=[False] * 7,
                               bounded_joint_metric="Euclidean L2 in R7 without wrapping")
        scene_map = {}
        for row in rows:
            scene_map.setdefault(row["scene_id"], dataset.read(BASE / row["scene_file"]))
        for sid, scene in scene_map.items():
            cfg = configs[scene["split"]]
            regenerated = generate_scene(scene["family"], scene["scene_seed"], cfg, model)
            for key, value in regenerated.items():
                require(value == scene[key], f"{sid}: scene regeneration differs at {key}")
            xyz, cover = voxelize(scene["obstacle_geometry"], scene["voxel_radius_m"], cfg["maximum_voxel_cell_side_m"])
            require(np.array_equal(xyz, np.asarray(scene["voxels_xyz"]).reshape(-1, 3)), f"{sid}: voxel regeneration differs")
            require(cover == scene["voxelization"], f"{sid}: cover certificate differs")
            cells = independent_cover(scene, cfg)
            report["scenes"].append(dict(scene_id=sid, split=scene["split"], category=scene["family"],
                                         scene_sha256=sha(BASE / f"scenes/{sid}.json"),
                                         obstacles=len(scene["obstacle_geometry"]), voxels=len(xyz),
                                         layout_rejections=scene["generation_rejected_layouts"],
                                         fixed_geometry_overlapping_voxels=scene["fixed_geometry_overlapping_voxels"],
                                         independent_cell_cover=cells, passed=True))
        proposal_count = line_count = endpoint_count = 0
        largest_clearance_error = 0.
        largest_native_oracle_error = 0.
        largest_fk_error = 0.
        # All stored proposals, including all rejected configurations, are
        # independently re-evaluated on CPU. Saved CUDA observations remain
        # historical and are compared to this fresh double-precision check.
        native_scenes = {}
        for row_index, row in enumerate(rows):
            identity = row["problem_id"]
            model.set_voxels(row["voxels_xyz"], row["voxel_radius_m"])
            with np.load(BASE / row["input_arrays_file"], allow_pickle=False) as archived:
                raw = {key: archived[key] for key in archived.files}
            result = dict(problem_id=identity, split=row["split"], category=row["family"],
                          input_status=row["validation_status"], endpoint_proposals=len(raw["endpoint_proposal_q"]))
            for prefix in ("endpoint_proposal", "straight_line"):
                if prefix + "_q" not in raw:
                    continue
                q = raw[prefix + "_q"]
                actual = model.clearance(q)
                saved = raw[prefix + "_independent_clearance_m"]
                err = float(np.max(np.abs(actual - saved), initial=0.))
                largest_clearance_error = max(largest_clearance_error, err)
                require(err <= 1e-10, f"{identity}: {prefix} clearance differs {err}")
                disagreements = (raw[prefix + "_cuda_free"] != (actual > 0)) & (np.abs(actual) > model.contact_band_m)
                require(not disagreements.any(), f"{identity}: historical CUDA vs fresh CPU disagreement")
                arrays[identity + "__" + prefix + "_clearance_m"] = actual
                result[prefix + "_clearance_max_error_m"] = err
                result[prefix + "_cuda_disagreements_outside_band"] = int(disagreements.sum())
                if prefix == "endpoint_proposal":
                    proposal_count += len(q)
                else:
                    line_count += len(q)
            available = [row[name] for name in ("q_start", "q_goal") if row[name] is not None]
            if available:
                sid = row["scene_id"]
                if sid not in native_scenes:
                    native_scenes[sid] = FreshDrake(row["voxels_xyz"])
                native = native_scenes[sid]
                endpoints = np.asarray(available)
                cpu = model.clearance(endpoints)
                oracle = np.asarray([native.distance(state) for state in endpoints])
                err = float(np.max(np.abs(cpu - oracle)))
                largest_native_oracle_error = max(largest_native_oracle_error, err)
                require(err <= 1e-10, f"{identity}: native SceneGraph sphere distance differs {err}")
                if row["validation_status"] == "valid_input":
                    require(len(endpoints) == 2 and np.min(cpu) > configs[row["split"]]["endpoint_clearance_m"], f"{identity}: invalid accepted endpoint")
                for state in endpoints:
                    manual_fk = manual.fk(state)
                    model.plant.SetPositions(model.plant_context, state)
                    query = model._query()
                    for name, gid in zip(names, model._drake_ids):
                        item = manual.geometry[name]
                        independent = manual_fk[item["link"]] @ item["X"]
                        actual = query.GetPoseInWorld(gid).GetAsMatrix4()
                        largest_fk_error = max(largest_fk_error, float(np.max(np.abs(actual - independent))))
                require(largest_fk_error <= 2e-12, f"{identity}: independent URDF geometry FK differs")
                endpoint_count += len(endpoints)
                result.update(endpoint_clearances_m=cpu.tolist(), native_scene_graph_max_error_m=err)
            result["passed"] = True
            report["cases"].append(result)
            if row_index % 20 == 0:
                print(json.dumps(dict(case=row_index + 1, total=len(rows), proposals=proposal_count, line_samples=line_count, seconds=time.perf_counter() - started)), flush=True)
        report["fresh_cpu_checks"] = dict(endpoint_proposals=proposal_count, straight_line_configurations=line_count,
                                           accepted_or_partial_endpoints=endpoint_count, native_scene_graphs=len(native_scenes),
                                           maximum_saved_clearance_error_m=largest_clearance_error,
                                           maximum_native_sphere_oracle_error_m=largest_native_oracle_error,
                                           maximum_manual_xml_geometry_pose_error=largest_fk_error)
        report["counts"] = {split: dict(registered=sum(r["split"] == split for r in rows),
                                           valid=sum(r["split"] == split and r["validation_status"] == "valid_input" for r in rows),
                                           generation_failed=sum(r["split"] == split and r["validation_status"] != "valid_input" for r in rows),
                                           statuses=dict(Counter(r["validation_status"] for r in rows if r["split"] == split))) for split in dataset.SPLITS}
        report["status"] = "passed"
    except Exception as exc:
        report["errors"].append(dict(type=type(exc).__name__, message=str(exc), traceback=traceback.format_exc()))
        report["status"] = "failed"
    finally:
        report["protected_files_after"] = {str(p.relative_to(ROOT)): sha(p) for p in protected}
        report["protected_files_unchanged"] = report["protected_files_before"] == report["protected_files_after"]
        if not report["protected_files_unchanged"]:
            report["status"] = "failed"
            report["errors"].append(dict(type="ProtectedInputChanged", message="Protected file changed during audit"))
        if arrays:
            np.savez_compressed(output / "clearances.npz", **arrays)
            report["clearances_sha256"] = sha(output / "clearances.npz")
        report["elapsed_seconds"] = time.perf_counter() - started
        (output / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(json.dumps({k: report.get(k) for k in ("status", "counts", "fresh_cpu_checks", "elapsed_seconds", "errors")}, indent=2), flush=True)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    raise SystemExit(main(parser.parse_args().output))
