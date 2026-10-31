"""Workspace geometry families with no semantic or planner-dependent rules."""
from __future__ import annotations

import hashlib
import json
import numpy as np

from .voxelize import rotation_z, voxelize

FAMILIES = ("sparse_clutter", "dense_clutter", "bottleneck", "route_choice")


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def domain_seed(seed, domain, *identifiers):
    text = json.dumps([int(seed), str(domain), *identifiers], separators=(",", ":"))
    return int.from_bytes(hashlib.sha256(text.encode()).digest()[:8], "little")


def rng(seed):
    return np.random.Generator(np.random.PCG64(int(seed)))


def _proposal(family, random, config):
    low, high = np.array(config["roi_min_m"]), np.array(config["roi_max_m"])
    if family == "open_control":
        return []
    cfg = config[family]
    objects = []
    if family in ("sparse_clutter", "dense_clutter"):
        number = int(random.integers(cfg["obstacles_min"], cfg["obstacles_max"] + 1))
        for i in range(number):
            centre = random.uniform(low, high)
            if family == "sparse_clutter" and random.random() < 0.35:
                objects.append(dict(shape="box", center_m=centre.tolist(),
                                    size_m=random.uniform(*cfg["box_size_range_m"], 3).tolist(),
                                    yaw_rad=float(random.uniform(-np.pi, np.pi))))
            else:
                objects.append(dict(shape="sphere", center_m=centre.tolist(),
                                    radius_m=float(random.uniform(*cfg["sphere_radius_range_m"]))))
    elif family == "bottleneck":
        yaw = float(random.uniform(-np.pi, np.pi))
        rotation = rotation_z(yaw)
        centre = random.uniform([0.15, -0.22, 0.45], [0.48, 0.22, 0.72])
        gap = float(random.uniform(*cfg["gap_range_m"]))
        length = cfg["wall_length_m"]
        for sign in (-1, 1):
            offset = rotation @ np.array([sign * (gap + length) / 2, 0, 0])
            objects.append(dict(shape="box", center_m=(centre + offset).tolist(),
                                size_m=[length, cfg["wall_thickness_m"], cfg["wall_height_m"]],
                                yaw_rad=yaw, workspace_gap_m=gap))
    elif family == "route_choice":
        yaw = float(random.uniform(-np.pi, np.pi))
        direction = rotation_z(yaw)[:, 0]
        centre = random.uniform([0.18, -0.25, 0.4], [0.52, 0.25, 0.78])
        separation = float(random.uniform(*cfg["cluster_separation_range_m"]))
        for cluster, sign in enumerate((-1, 1)):
            for _ in range(cfg["spheres_per_cluster"]):
                location = centre + sign * separation * direction / 2
                location += random.uniform(-cfg["cluster_spread_m"], cfg["cluster_spread_m"], 3)
                objects.append(dict(shape="sphere", center_m=location.tolist(),
                                    radius_m=float(random.uniform(*cfg["sphere_radius_range_m"])),
                                    cluster=cluster))
    else:
        raise ValueError(f"Unknown geometry family {family}")
    for i, obj in enumerate(objects):
        obj["obstacle_id"] = f"o{i:03d}"
    return objects


def generate_scene(family, scene_seed, config, model):
    """Reject only geometry intersecting fixed equipment, never planner failures.

    Entire layouts are deterministic proposals. An exhausted geometric sampler
    retains its last layout as invalid; no planner or endpoint is consulted.
    """
    random = rng(scene_seed)
    rejection_count = 0
    for attempt in range(config["maximum_scene_proposals"]):
        objects = _proposal(family, random, config)
        xyz, cover = voxelize(objects, config["voxel_radius_m"], config["maximum_voxel_cell_side_m"])
        distances = model.fixed_geometry_sphere_clearance(xyz, config["voxel_radius_m"])
        overlap = int(np.sum(np.asarray(distances) <= 0))
        if not overlap:
            break
        rejection_count += 1
    return {"obstacle_geometry": objects, "voxels_xyz": xyz.tolist(),
            "voxel_radius_m": config["voxel_radius_m"], "voxelization": cover,
            "generation_rejected_layouts": rejection_count,
            "fixed_geometry_overlapping_voxels": overlap,
            "generation_status": "valid" if not overlap else "invalid_scene_geometry",
            "workspace_geometry_description_only": family in ("bottleneck", "route_choice"),
            "cspace_topology_known": False}
