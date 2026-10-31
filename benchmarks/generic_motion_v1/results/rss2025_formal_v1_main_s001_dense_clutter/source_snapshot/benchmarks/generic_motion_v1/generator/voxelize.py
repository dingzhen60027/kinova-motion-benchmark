"""Conservative ball covers of actual geometric obstacles.

Each intersected grid cell is enclosed in a radius-0.04 m ball. Cells use at
most 0.04 m sides, leaving a 5 mm reserve for float32 centre quantization.
This is a geometric coverage argument, not a trajectory safety certificate.
"""
from __future__ import annotations

import itertools
import numpy as np


def rotation_z(yaw):
    if not np.isfinite(yaw):
        raise ValueError('Obstacle yaw must be finite')
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def voxelize(obstacles, radius=0.04, cell_side=0.04):
    if radius != 0.04 or not 0 < cell_side <= 0.04:
        raise ValueError("v1 freezes radius=0.04 and cell sides at most 0.04 m")
    centres, records = [], []
    for obstacle in obstacles:
        centre = np.asarray(obstacle["center_m"], dtype=float)
        if centre.shape != (3,) or not np.isfinite(centre).all():
            raise ValueError("Obstacle centre must contain three finite coordinates")
        kind = obstacle["shape"]
        if kind == "sphere":
            r = float(obstacle["radius_m"])
            dimensions = np.full(3, 2 * r)
            rotation = np.eye(3)
        elif kind == "box":
            dimensions = np.asarray(obstacle["size_m"], dtype=float)
            rotation = rotation_z(float(obstacle.get("yaw_rad", 0)))
        else:
            raise ValueError(f"Unsupported obstacle shape {kind}")
        if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
            raise ValueError("Obstacle dimensions must be positive and finite")
        counts_float = np.ceil(dimensions / cell_side)
        if not np.isfinite(counts_float).all() or np.prod(counts_float) > 1000000:
            raise ValueError('Obstacle cover exceeds one million candidate cells')
        counts = counts_float.astype(int)
        sides = dimensions / counts
        axes = [(np.arange(n) + 0.5) * width - extent / 2
                for n, width, extent in zip(counts, sides, dimensions)]
        local = np.asarray(list(itertools.product(*axes)))
        if kind == "sphere":
            # Retain every cell intersecting the source sphere, including
            # tangent cells; the closest point of the box gives exact overlap.
            closest = np.maximum(np.abs(local) - sides / 2, 0)
            local = local[np.linalg.norm(closest, axis=1) <= r + 1e-14]
        original = local @ rotation.T + centre
        with np.errstate(over='ignore', invalid='ignore'):
            stored = original.astype(np.float32).astype(float)
        if not np.isfinite(stored).all():
            raise ValueError('Voxel centres are not finite in float32')
        error = float(np.max(np.linalg.norm(original - stored, axis=1), initial=0))
        half_diagonal = float(np.linalg.norm(sides / 2))
        # At very large, exactly representable world coordinates, addition can
        # erase local grid offsets even when the float32 cast itself is exact.
        # Reserve a conservative floating point transform-error budget before
        # accepting the cell enclosure. Frozen metre-scale covers are unchanged.
        transform_bound = 16 * np.finfo(float).eps * float(np.linalg.norm(
            np.abs(centre) + np.max(np.abs(local) @ np.abs(rotation.T), axis=0)))
        if half_diagonal + error + transform_bound > radius:
            raise ValueError("Float32 voxel centres no longer cover their cells")
        start = len(centres)
        centres.extend(stored.tolist())
        if len(centres) > 1000000:
            raise ValueError('Scene cover exceeds one million voxels')
        records.append({"obstacle_id": obstacle["obstacle_id"], "first_voxel": start,
                        "voxel_count": len(stored), "grid_cell_side_m": sides.tolist(),
                        "cell_half_diagonal_m": half_diagonal,
                        "maximum_centre_quantization_error_m": error,
                        "coverage_margin_m": radius - half_diagonal - error,
                        "maximum_outward_overapproximation_m": radius + half_diagonal + error})
    xyz = np.asarray(centres, dtype=float).reshape(-1, 3)
    report = {"policy": "cover every source-intersecting grid cell with one enclosing sphere",
              "voxel_radius_m": radius, "voxel_count": len(xyz),
              "float32_centres_frozen": True, "coverage_certified_by_cell_bound": True,
              "bounds_min_m": xyz.min(axis=0).tolist() if len(xyz) else None,
              "bounds_max_m": xyz.max(axis=0).tolist() if len(xyz) else None,
              "obstacles": records}
    return xyz, report
