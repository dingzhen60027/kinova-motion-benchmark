"""Frozen JSONL datasets and receipts, deterministic across output locations."""
from __future__ import annotations

import hashlib
import json
import numbers
from pathlib import Path
import subprocess

import numpy as np

from .scene import FAMILIES, digest, domain_seed, generate_scene
from .endpoints import generate_endpoints
from .configuration import validate_config

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
DEFAULT_CONFIG = HERE / "configs/generator.json"


def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def json_text(value):
    return json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + "\n"


def write_new(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        stream.write(content)


def source_hashes():
    return {str(path.relative_to(HERE)): file_sha256(path)
            for path in sorted(Path(__file__).parent.glob("*.py"))}


def _require_model_audit(model):
    # A subsequent qualification can certify stricter guards without replacing
    # the original receipt used to generate already-frozen datasets.
    failures = []
    for path in (HERE / "reports/qualification/model_audit_current.json",
                 HERE / "reports/model_audit/audit.json"):
        if not path.exists():
            continue
        try:
            receipt = json.loads(path.read_text())
            if receipt.get("passed") is not True:
                raise ValueError("G0 model audit did not pass")
            if (receipt["model"]["robot_sha256"] != model.robot_sha256 or
                    receipt["model"]["collision_geometry_sha256"] != model.collision_geometry_sha256 or
                    receipt["model_source_sha256"] != file_sha256(HERE / "model.py")):
                raise ValueError("Current model differs from the passing G0 audit")
            return dict(receipt, _receipt_file=str(path.relative_to(HERE)), _receipt_sha256=file_sha256(path))
        except (ValueError, KeyError, TypeError) as exc:
            failures.append(f"{path.name}: {exc}")
    raise RuntimeError("No passing G0 receipt matches the current model; " + "; ".join(failures))


def generate(split, seed, out_root, *, model=None, config_path=None,
             source_code_commit=None, unlock_test=False):
    """Generate an immutable split, with no call to any planning method.

    ``unlock_test`` is an orchestration gate: only the root CLI may enable it
    after persisted G1/G2 receipts have passed. Dataset generation is separate
    from running test baselines, which are not authorized at this stage.
    """
    if split not in ("smoke", "dev", "test"):
        raise ValueError("split must be smoke, dev or test")
    if isinstance(seed, bool) or not isinstance(seed, numbers.Integral) or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    if split == "test" and not unlock_test:
        raise RuntimeError("Test generation is locked until G1/G2 acceptance")
    config_path = Path(config_path or DEFAULT_CONFIG)
    config = json.loads(config_path.read_text())
    validate_config(config)
    out = Path(out_root)
    manifest_path = out / "manifests" / f"{split}.jsonl"
    if manifest_path.exists() or (out / "receipts" / f"{split}_generation.json").exists():
        raise FileExistsError(f"Frozen split already exists: {manifest_path}")
    if model is None:
        from ..model import Model
        model = Model()
    model_audit = _require_model_audit(model)
    validate_config(config, model)
    if source_code_commit is None:
        source_code_commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    config_hash = file_sha256(config_path)
    sources = source_hashes()
    protocol = dict(config_sha256=config_hash, generator_sources_sha256=sources,
                    robot_sha256=model.robot_sha256,
                    collision_geometry_sha256=model.collision_geometry_sha256,
                    model_source_sha256=file_sha256(HERE / "model.py"))
    config_copy = out / "configs" / f"{split}_generator.json"
    write_new(config_copy, config_path.read_text())
    workspace = model_audit["workspace"]
    low, high = np.array(config["roi_min_m"]), np.array(config["roi_max_m"])
    if np.any(low < workspace["moving_collision_geometry_min"]) or np.any(high > workspace["moving_collision_geometry_max"]):
        raise ValueError("Configured obstacle ROI lies outside the sampled moving-robot envelope")
    model_audit_hash = model_audit.get('_receipt_sha256')
    if model_audit_hash is None:
        model_audit_hash = file_sha256(HERE / "reports/model_audit/audit.json")
    receipt = {
        "benchmark_version": config["benchmark_version"], "split": split, "seed": int(seed),
        "rng": config["rng"], "source_code_commit": source_code_commit,
        "protocol": protocol, "geometry_protocol_sha256": digest(protocol),
        "workspace_roi_evidence": {"roi_min_m": low.tolist(), "roi_max_m": high.tolist(),
            "g0_workspace": workspace, "model_audit_sha256": model_audit_hash,
            "interpretation": "ROI overlaps measured reachable TCP and robot envelope; pointwise reachability is not proved"},
        "selection_policy": "Geometry-only fixed-equipment rejection and endpoint validity/distance bins; no planning algorithm invoked",
        "open_control_policy": "All open layouts have the identical base-only geometry; they are not independent random environments",
        "files_sha256": {}, "problem_count": 0, "scene_count": 0,
    }
    rows = []
    split_cfg = config["splits"][split]
    family_layouts = [(family, split_cfg["main_layouts_per_family"]) for family in FAMILIES]
    family_layouts.append(("open_control", split_cfg["open_layouts"]))
    for family, layout_count in family_layouts:
        for index in range(layout_count):
            scene_id = f"{split}-{family}-{index:03d}"
            scene_seed = domain_seed(seed, "scene", split, family, index)
            scene = generate_scene(family, scene_seed, config, model)
            scene.update(scene_id=scene_id, family=family, split=split, scene_seed=scene_seed,
                         world_frame=config["world_frame"], robot_sha256=model.robot_sha256,
                         collision_geometry_sha256=model.collision_geometry_sha256,
                         geometry_protocol_sha256=receipt["geometry_protocol_sha256"])
            scene["obstacle_geometry_sha256"] = digest(scene["obstacle_geometry"])
            scene["voxel_geometry_sha256"] = digest(dict(xyz=scene["voxels_xyz"], radius=scene["voxel_radius_m"]))
            scene_path = out / "scenes" / f"{scene_id}.json"
            write_new(scene_path, json_text(scene))
            scene_hash = file_sha256(scene_path)
            receipt["files_sha256"][str(scene_path.relative_to(out))] = scene_hash
            receipt["scene_count"] += 1
            model.set_voxels(scene["voxels_xyz"], scene["voxel_radius_m"])
            for pair_index in range(split_cfg["pairs_per_layout"]):
                problem_id = f"{scene_id}-p{pair_index:02d}"
                endpoint_seed = domain_seed(seed, "endpoint", split, family, index, pair_index)
                cycle = config["distance_bin_cycle"]
                distance_bin = cycle[pair_index % len(cycle)]
                endpoints, raw = generate_endpoints(model, endpoint_seed, distance_bin, config)
                if scene["generation_status"] != "valid":
                    endpoints["validation_status"] = "invalid_scene_geometry"
                row = dict(benchmark_version=config["benchmark_version"], split=split, family=family,
                           scene_id=scene_id, problem_id=problem_id,
                           robot_sha256=model.robot_sha256,
                           collision_geometry_sha256=model.collision_geometry_sha256,
                           geometry_protocol_sha256=receipt["geometry_protocol_sha256"],
                           obstacle_geometry=scene["obstacle_geometry"], voxels_xyz=scene["voxels_xyz"],
                           voxel_radius_m=scene["voxel_radius_m"], world_frame=config["world_frame"],
                           scene_seed=scene_seed, endpoint_seed=endpoint_seed,
                           source_code_commit=source_code_commit,
                           generator_config_sha256=config_hash,
                           generator_sources_sha256=sources,
                           scene_file=str(scene_path.relative_to(out)), scene_sha256=scene_hash,
                           obstacle_geometry_sha256=scene["obstacle_geometry_sha256"],
                           voxel_geometry_sha256=scene["voxel_geometry_sha256"],
                           objectives=["minimum_distance", "minimum_time"],
                           velocity_limits_rad_s=[1.0] * 7, acceleration_limits_rad_s2=[1.0] * 7,
                           endpoint_velocity_rad_s=[0.0] * 7,
                           paired_seeds=[dict(replicate=r,
                               drm_seed=r,
                               method_seed=domain_seed(seed, "method", split, family, index, pair_index, r) % 2**32,
                               solver_seed=domain_seed(seed, "solver", split, family, index, pair_index, r) % 2**32) for r in range(3)],
                           **endpoints)
                raw_path = out / "inputs" / f"{problem_id}.npz"
                raw_path.parent.mkdir(parents=True, exist_ok=True)
                if raw_path.exists():
                    raise FileExistsError(raw_path)
                np.savez_compressed(raw_path, **raw)
                row["input_arrays_file"] = str(raw_path.relative_to(out))
                row["input_arrays_sha256"] = file_sha256(raw_path)
                receipt["files_sha256"][str(raw_path.relative_to(out))] = row["input_arrays_sha256"]
                row["problem_sha256"] = digest(row)
                rows.append(row)
    write_new(manifest_path, "".join(json.dumps(row, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n" for row in rows))
    receipt["problem_count"] = len(rows)
    receipt["valid_input_count"] = sum(row["validation_status"] == "valid_input" for row in rows)
    receipt["invalid_input_count"] = len(rows) - receipt["valid_input_count"]
    receipt["manifest_path"] = str(manifest_path.relative_to(out))
    receipt["manifest_sha256"] = file_sha256(manifest_path)
    receipt["files_sha256"][str(manifest_path.relative_to(out))] = receipt["manifest_sha256"]
    receipt["files_sha256"][str(config_copy.relative_to(out))] = file_sha256(config_copy)
    write_new(out / "receipts" / f"{split}_generation.json", json_text(receipt))
    return dict(receipt, manifest_path=str(manifest_path))
