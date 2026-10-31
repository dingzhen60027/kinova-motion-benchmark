"""Reload and independently check frozen inputs and geometry regeneration."""
from __future__ import annotations

from collections import Counter, defaultdict
import json
from pathlib import Path

import numpy as np

from .endpoints import normalized_distance, straight_line
from .manifest import _require_model_audit, file_sha256, json_text
from .scene import digest, generate_scene
from .voxelize import voxelize


def audit(manifest_path, *, model=None, config_path=None, regenerate=True, out=None):
    path = Path(manifest_path)
    root = path.parent.parent
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if not rows:
        raise ValueError("Manifest is empty")
    split = rows[0]["split"]
    receipt = json.loads((root / "receipts" / f"{split}_generation.json").read_text())
    config_path = Path(config_path or root / "configs" / f"{split}_generator.json")
    config = json.loads(config_path.read_text())
    if model is None:
        from ..model import Model
        model = Model()
    _require_model_audit(model)
    errors = []
    if file_sha256(config_path) != receipt["protocol"]["config_sha256"]:
        errors.append("generator_config_hash_mismatch")
    if file_sha256(path) != receipt["manifest_sha256"]:
        errors.append("manifest_hash_mismatch")
    for relative, expected in receipt["files_sha256"].items():
        file = root / relative
        if not file.exists() or file_sha256(file) != expected:
            errors.append(f"file_hash_mismatch:{relative}")
    seen_ids, scenes, cases = set(), {}, []
    significant_disagreements = 0
    for row in rows:
        case_errors = []
        identity = row["problem_id"]
        if identity in seen_ids:
            case_errors.append("duplicate_problem_id")
        seen_ids.add(identity)
        actual = dict(row)
        stored = actual.pop("problem_sha256")
        if digest(actual) != stored:
            case_errors.append("problem_hash_mismatch")
        if row["robot_sha256"] != model.robot_sha256 or row["collision_geometry_sha256"] != model.collision_geometry_sha256:
            case_errors.append("model_hash_mismatch")
        scene_id = row["scene_id"]
        if scene_id not in scenes:
            scene_path = root / row["scene_file"]
            scene = json.loads(scene_path.read_text())
            scene_errors = []
            if file_sha256(scene_path) != row["scene_sha256"]:
                scene_errors.append("scene_file_hash_mismatch")
            xyz, cover = voxelize(scene["obstacle_geometry"], row["voxel_radius_m"], config["maximum_voxel_cell_side_m"])
            if not np.array_equal(xyz, np.asarray(scene["voxels_xyz"]).reshape(-1, 3)):
                scene_errors.append("voxel_regeneration_mismatch")
            if cover != scene["voxelization"]:
                scene_errors.append("voxel_certificate_mismatch")
            if regenerate:
                regenerated = generate_scene(row["family"], row["scene_seed"], config, model)
                for field, value in regenerated.items():
                    if value != scene[field]:
                        scene_errors.append(f"scene_regeneration_mismatch:{field}")
            physical_overlap = int(np.sum(model.fixed_geometry_sphere_clearance(xyz, row["voxel_radius_m"]) <= 0))
            scenes[scene_id] = dict(scene_id=scene_id, family=row["family"],
                obstacle_count=len(scene["obstacle_geometry"]), voxel_count=len(xyz),
                voxel_xyz_float32_bytes=len(xyz) * 3 * 4,
                geometry_sha256=row["obstacle_geometry_sha256"],
                layout_rejections=scene["generation_rejected_layouts"],
                fixed_geometry_overlapping_voxels=physical_overlap,
                errors=scene_errors, passed=not scene_errors)
        scene = json.loads((root / row["scene_file"]).read_text())
        for key in ("obstacle_geometry", "voxels_xyz", "voxel_radius_m", "scene_seed", "family", "split"):
            if row[key] != scene[key]:
                case_errors.append(f"manifest_scene_mismatch:{key}")
        if digest(row["obstacle_geometry"]) != row["obstacle_geometry_sha256"]:
            case_errors.append("obstacle_geometry_hash_mismatch")
        if digest(dict(xyz=row["voxels_xyz"], radius=row["voxel_radius_m"])) != row["voxel_geometry_sha256"]:
            case_errors.append("voxel_geometry_hash_mismatch")
        model.set_voxels(row["voxels_xyz"], row["voxel_radius_m"])
        result = dict(problem_id=identity, family=row["family"], scene_id=scene_id,
                      validation_status=row["validation_status"], errors=case_errors)
        if row["q_start"] is not None and row["q_goal"] is not None:
            q = np.asarray([row["q_start"], row["q_goal"]])
            margin = config["endpoint_joint_margin_rad"]
            joint_limits_ok = bool(q.shape == (2, 7) and np.isfinite(q).all() and
                                   np.all(q >= model.lower + margin) and np.all(q <= model.upper - margin))
            if not joint_limits_ok:
                case_errors.append("endpoint_joint_limits")
            native = model.cuda(q)
            clearance = model.clearance(q)
            endpoints_valid = bool(native.all() and np.all(clearance > config["endpoint_clearance_m"]))
            if row["validation_status"] == "valid_input" and not endpoints_valid:
                case_errors.append("endpoint_collision_or_clearance")
            distance = float(np.linalg.norm(q[1] - q[0]))
            ndistance = normalized_distance(q[0], q[1], model.lower, model.upper)
            if not np.isclose(distance, row["start_goal_joint_distance"], atol=1e-12, rtol=0):
                case_errors.append("endpoint_distance_mismatch")
            if not np.isclose(ndistance, row["normalized_start_goal_joint_distance"], atol=1e-12, rtol=0):
                case_errors.append("normalized_endpoint_distance_mismatch")
            minimum, maximum = row["endpoint_sampling"]["distance_bin_bounds"]
            if not minimum <= ndistance < maximum:
                case_errors.append("distance_stratum_mismatch")
            line, _ = straight_line(model, q[0], q[1], config["straight_line_step_rad"])
            if line["straight_line_collision"] != row["straight_line_collision"]:
                case_errors.append("straight_line_classification_mismatch")
            disagreements = line["straight_line_disagreements_outside_contact_band"]
            significant_disagreements += disagreements
            if disagreements:
                case_errors.append("independent_checker_disagreement")
            result.update(endpoint_native_free=native.tolist(), endpoint_clearances_m=clearance.tolist(),
                          joint_limits_ok=joint_limits_ok, endpoints_valid=endpoints_valid,
                          start_goal_joint_distance=distance,
                          normalized_start_goal_joint_distance=ndistance,
                          distance_bin=row["endpoint_sampling"]["distance_bin"], **line)
        else:
            if row["validation_status"] == "valid_input":
                case_errors.append("missing_endpoint_for_valid_input")
            result["endpoints_valid"] = False
        result["passed"] = not case_errors
        cases.append(result)
    # This also catches accidentally copied layouts or endpoints across splits.
    other_layout_hashes, other_endpoint_hashes = {}, {}
    for other in sorted(path.parent.glob("*.jsonl")):
        if other == path:
            continue
        for line in other.read_text().splitlines():
            row = json.loads(line)
            if row["family"] != "open_control":
                other_layout_hashes[row["obstacle_geometry_sha256"]] = row["scene_id"]
            other_endpoint_hashes[digest([row["q_start"], row["q_goal"]])] = row["problem_id"]
    shared_layouts = []
    shared_endpoints = []
    for row in rows:
        if row["family"] != "open_control" and row["obstacle_geometry_sha256"] in other_layout_hashes:
            shared_layouts.append([row["scene_id"], other_layout_hashes[row["obstacle_geometry_sha256"]]])
        key = digest([row["q_start"], row["q_goal"]])
        if row["q_start"] is not None and row["q_goal"] is not None and key in other_endpoint_hashes:
            shared_endpoints.append([row["problem_id"], other_endpoint_hashes[key]])
    if shared_layouts:
        errors.append("non_open_scene_geometry_shared_across_splits")
    if shared_endpoints:
        errors.append("endpoints_shared_across_splits")
    families = {}
    for family in sorted({row["family"] for row in rows}):
        family_rows = [row for row in rows if row["family"] == family]
        valid = [row for row in family_rows if row["validation_status"] == "valid_input"]
        family_scenes = [scene for scene in scenes.values() if scene["family"] == family]
        distances = [row["start_goal_joint_distance"] for row in valid]
        families[family] = {
            "cases": len(family_rows), "layouts": len(family_scenes), "valid_inputs": len(valid),
            "independent_random_geometries": 0 if family == "open_control" else len({s["geometry_sha256"] for s in family_scenes}),
            "distinct_actual_geometries": len({s["geometry_sha256"] for s in family_scenes}),
            "straight_line_blocked": sum(row["straight_line_collision"] is True for row in valid),
            "straight_line_blocked_fraction": sum(row["straight_line_collision"] is True for row in valid) / len(valid) if valid else None,
            "distance_rad_min_median_max": np.quantile(distances, [0, .5, 1]).tolist() if distances else None,
            "normalized_distance_min_median_max": np.quantile([row["normalized_start_goal_joint_distance"] for row in valid], [0, .5, 1]).tolist() if valid else None,
            "distance_bin_counts": dict(Counter(row["endpoint_sampling"]["distance_bin"] for row in family_rows)),
            "obstacle_count_min_max": [min(s["obstacle_count"] for s in family_scenes), max(s["obstacle_count"] for s in family_scenes)],
            "voxel_count_min_max": [min(s["voxel_count"] for s in family_scenes), max(s["voxel_count"] for s in family_scenes)],
            "endpoint_collision_or_clearance_rejections": sum(row["endpoint_sampling"]["collision_or_clearance_rejections"] for row in family_rows),
            "endpoint_distance_bin_rejections": sum(row["endpoint_sampling"]["distance_bin_rejections"] for row in family_rows),
            "layout_rejections": sum(s["layout_rejections"] for s in family_scenes),
        }
    report = dict(split=split, manifest_sha256=file_sha256(path),
                  passed=not errors and all(case["passed"] for case in cases) and all(scene["passed"] for scene in scenes.values()),
                  errors=errors, problem_count=len(rows), scene_count=len(scenes),
                  valid_input_count=sum(row["validation_status"] == "valid_input" for row in rows),
                  invalid_input_count=sum(row["validation_status"] != "valid_input" for row in rows),
                  statuses=dict(Counter(row["validation_status"] for row in rows)),
                  independent_checker_disagreements_outside_contact_band=significant_disagreements,
                  endpoint_penetration_count=sum(any(d <= 0 for d in case.get("endpoint_clearances_m", [])) for case in cases),
                  shared_non_open_geometries_across_splits=shared_layouts,
                  shared_endpoints_across_splits=shared_endpoints,
                  numerical_collision_check_only=True,
                  unknown_global_feasibility_count=len(rows),
                  family_distribution=families, scenes=list(scenes.values()), cases=cases)
    if out:
        out = Path(out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json_text(report))
    return report
