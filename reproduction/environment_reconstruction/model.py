"""Fixed-input Forest geometry shared by native, Drake and analytic checks.

The scene is constructed once from explicit centers. No RNG is consulted and
no geometry is replaced after another collision representation is built.
"""
from __future__ import annotations

from copy import deepcopy

import csdecomp as csd
import numpy as np
from pydrake.all import RobotDiagramBuilder

from reproduction.common import config
from reproduction.forest import Forest, urdf
from reproduction.paper_constructed.model import ForestEnvironment


class _FixedForestGeometry(Forest):
    """Reuse analytic predicates without calling the random-scene constructor."""

    def __init__(self, cfg, centers):
        self.cfg = cfg
        self.centers = np.asarray(centers, dtype=float).copy()
        self.radius = float(cfg["obstacle_radius"] + cfg["point_radius"])
        self.source = urdf(cfg, self.centers)
        self.domain = csd.HPolyhedron()
        self.domain.MakeBox(np.asarray(cfg["domain_lower"]), np.asarray(cfg["domain_upper"]))


class FixedForest(ForestEnvironment):
    """ForestEnvironment adapter for an explicit, immutable input scene.

    The analytic and Drake models retain the supplied float64 centers. Native
    online voxels and planner endpoints use a declared float32 round trip, as
    required by the native API. Original endpoint coordinates are retained.
    """

    kind = "forest"
    dimension = 2
    dim = 2

    def __init__(self, protocol, scene):
        self.protocol = deepcopy(protocol)
        self.scene = deepcopy(scene)
        cfg = config()
        cfg.update({key: deepcopy(value) for key, value in protocol.items() if key in cfg})
        cfg["eizo"] = deepcopy(protocol["eizo"]["forest"])
        for name in ("domain_lower", "domain_upper", "obstacle_radius", "obstacle_center_lower", "obstacle_center_upper"):
            if name in scene:
                cfg[name] = deepcopy(scene[name])
        centers = np.asarray(scene["centers"], dtype=float)
        if centers.ndim != 2 or centers.shape[1] != 2 or len(centers) == 0 or not np.isfinite(centers).all():
            raise ValueError("Fixed Forest centers must be a finite nonempty N x 2 array")
        if "obstacle_centers" in scene and not np.array_equal(centers, np.asarray(scene["obstacle_centers"])):
            raise ValueError("Scene center aliases disagree")
        if "obstacles" in scene and scene["obstacles"] != len(centers):
            raise ValueError("Declared obstacle count differs from explicit centers")
        cfg["obstacles"] = len(centers)
        self.lower = np.asarray(cfg["domain_lower"], dtype=float)
        self.upper = np.asarray(cfg["domain_upper"], dtype=float)
        if (self.lower.shape != (2,) or self.upper.shape != (2,)
                or not np.isfinite([self.lower, self.upper]).all() or not np.all(self.lower < self.upper)):
            raise ValueError("Invalid Forest domain")
        if not np.isfinite([cfg["point_radius"], cfg["obstacle_radius"]]).all() or min(cfg["point_radius"], cfg["obstacle_radius"]) <= 0:
            raise ValueError("Both geometry radii must be positive and finite")
        self.original_start = np.asarray(scene["start"], dtype=float).copy()
        self.original_goal = np.asarray(scene["goal"], dtype=float).copy()
        for endpoint in (self.original_start, self.original_goal):
            if endpoint.shape != (2,) or not np.isfinite(endpoint).all():
                raise ValueError("Endpoint must contain two finite coordinates")
            if not np.all((endpoint >= self.lower) & (endpoint <= self.upper)):
                raise ValueError("Endpoint outside Forest domain")
        self.start = self.original_start.astype(np.float32).astype(float)
        self.goal = self.original_goal.astype(np.float32).astype(float)
        cfg.update(start=self.start.copy(), goal=self.goal.copy())
        self.cfg = cfg
        self.endpoint_representation = {
            "rule": "float32_roundtrip_without_endpoint_displacement",
            "original_start": self.original_start.tolist(), "original_goal": self.original_goal.tolist(),
            "canonical_start": self.start.tolist(), "canonical_goal": self.goal.tolist(),
            "start_l2_rounding_m": float(np.linalg.norm(self.start-self.original_start)),
            "goal_l2_rounding_m": float(np.linalg.norm(self.goal-self.original_goal)),
        }
        self.base = _FixedForestGeometry(cfg, centers)
        self.domain = self.base.domain

        # The offline/native plant has only the moving point. Explicit obstacles
        # enter through voxels; including them in both would double-count them.
        parser = csd.URDFParser()
        if not parser.parse_urdf_string(urdf(cfg, [])):
            raise ValueError("Fixed Forest empty-workspace URDF rejected")
        self.native = parser.build_plant()
        self.minimal = self.native.getMinimalPlant()
        self.geometry_ids = self.native.getRobotGeometryIds()
        self.voxels = csd.Voxels(np.vstack([centers.T, np.zeros(len(centers))]).astype(np.float32))
        self.voxel_radius = float(cfg["obstacle_radius"])

        # Build Drake from the same original centers before any queries. Merely
        # changing an existing random Forest.centers would leave stale geometry.
        builder = RobotDiagramBuilder()
        self.drake, self.sg = builder.plant(), builder.scene_graph()
        builder.parser().AddModelsFromString(self.base.source, "urdf")
        self.drake.Finalize()
        self.diagram = builder.Build()
        self.context = self.diagram.CreateDefaultContext()
        self.pc = self.drake.GetMyMutableContextFromRoot(self.context)

    def drake_clearances(self, q):
        """Actual full-scene signed distances for geometry audits, not planning."""
        rows = np.atleast_2d(np.asarray(q, dtype=float))
        if rows.shape[1] != 2 or not np.isfinite(rows).all():
            raise ValueError("Expected finite N x 2 configurations")
        result = []
        sg_context = self.sg.GetMyContextFromRoot(self.context)
        for row in rows:
            self.drake.SetPositions(self.pc, row)
            pairs = self.sg.get_query_output_port().Eval(sg_context).ComputeSignedDistancePairwiseClosestPoints()
            if len(pairs) != self.cfg["obstacles"]:
                raise RuntimeError("Unexpected Drake candidate-pair count")
            result.append(min(pair.distance for pair in pairs))
        return np.asarray(result)
