"""Published Forest geometry, released native roadmap, independent circle checks."""

from __future__ import annotations
from pathlib import Path
import time
import numpy as np
import csdecomp as csd
import networkx as nx
from .common import atomic


def obstacles(cfg, seed):
    return np.random.default_rng(seed).uniform(
        cfg["obstacle_center_lower"],
        cfg["obstacle_center_upper"],
        (cfg["obstacles"], 2),
    )


def urdf(cfg, centers):
    collisions = "".join(
        f'<collision name="circle_{i}"><origin xyz="{x:.17g} {y:.17g} 0"/><geometry><sphere radius="{cfg["obstacle_radius"]}"/></geometry></collision>'
        for i, (x, y) in enumerate(centers)
    )
    lo, hi = cfg["domain_lower"], cfg["domain_upper"]
    return f"""<robot name="forest">
<link name="obstacles">{collisions}</link>
<joint name="obstacle_weld" type="fixed"><parent link="world"/><child link="obstacles"/></joint>
<link name="slide"/><link name="point"><collision name="point"><geometry><sphere radius="{cfg['point_radius']}"/></geometry></collision></link>
<joint name="x" type="prismatic"><parent link="world"/><child link="slide"/><axis xyz="1 0 0"/><limit lower="{lo[0]}" upper="{hi[0]}" effort="1" velocity="1"/></joint>
<joint name="y" type="prismatic"><parent link="slide"/><child link="point"/><axis xyz="0 1 0"/><limit lower="{lo[1]}" upper="{hi[1]}" effort="1" velocity="1"/></joint>
</robot>"""


class Forest:
    def __init__(self, cfg, environment_seed):
        self.cfg = cfg
        self.centers = obstacles(cfg, environment_seed)
        self.radius = cfg["obstacle_radius"] + cfg["point_radius"]
        self.parser = csd.URDFParser()
        self.source = urdf(cfg, self.centers)
        if not self.parser.parse_urdf_string(self.source):
            raise ValueError("Forest URDF rejected")
        self.plant = self.parser.build_plant()
        self.minimal = self.plant.getMinimalPlant()
        self.geometry_ids = self.plant.getRobotGeometryIds()
        self.domain = csd.HPolyhedron()
        self.domain.MakeBox(
            np.array(cfg["domain_lower"]), np.array(cfg["domain_upper"])
        )
        self.empty_voxels = csd.Voxels(np.empty((3, 0), dtype=np.float32))

    def clearances(self, points):
        points = np.atleast_2d(points)
        delta = points[:, None, :] - self.centers[None, :, :]
        return np.sqrt((delta * delta).sum(axis=2)).min(axis=1) - self.radius

    def segment_clearances(self, path):
        """Exact closest point between every polygonal segment and every circle."""
        path = np.asarray(path, dtype=float)
        start, delta = path[:-1], np.diff(path, axis=0)
        length2 = (delta * delta).sum(axis=1)
        t = ((self.centers[None, :, :] - start[:, None, :]) * delta[:, None, :]).sum(
            axis=2
        ) / np.maximum(length2[:, None], 1e-30)
        closest = start[:, None, :] + np.clip(t, 0, 1)[:, :, None] * delta[:, None, :]
        distances = (
            np.linalg.norm(closest - self.centers[None, :, :], axis=2) - self.radius
        )
        indices = np.argmin(distances, axis=1)
        return (
            distances[np.arange(len(start)), indices],
            closest[np.arange(len(start)), indices],
        )

    def validate_polygon(self, path):
        path = np.asarray(path)
        margins, points = self.segment_clearances(path)
        endpoint_error = max(
            np.linalg.norm(path[0] - self.cfg["start"]),
            np.linalg.norm(path[-1] - self.cfg["goal"]),
        )
        bounds = bool(
            np.all(path >= np.array(self.cfg["domain_lower"]) - 1e-7)
            and np.all(path <= np.array(self.cfg["domain_upper"]) + 1e-7)
        )
        return dict(
            collision_free=bool(margins.min() >= 0),
            minimum_clearance=float(margins.min()),
            endpoint_error=float(endpoint_error),
            endpoint_pass=bool(endpoint_error <= self.cfg["endpoint_tolerance"]),
            limits_pass=bounds,
            check="analytic_line_circle",
            collision_points=points[margins < 0],
            collision_segments=np.flatnonzero(margins < 0),
        )

    def roadmap(self, seed, size, out):
        cfg = self.cfg
        rng = np.random.default_rng(np.random.SeedSequence([2025, seed]))
        accepted = []
        while len(accepted) < size:
            batch = rng.uniform(
                cfg["domain_lower"], cfg["domain_upper"], (max(size, 1024), 2)
            )
            accepted.extend(batch[self.clearances(batch) > 0])
        nodes = np.asarray(accepted[:size], dtype=np.float32)
        options = csd.RoadmapOptions()
        options.robot_map_size_x = 10.0
        options.robot_map_size_y = 10.0
        options.robot_map_size_z = cfg["offline_voxel_side"]
        options.map_center = np.array([0.0, 0.0, 0.0])
        options.offline_voxel_resolution = cfg["offline_voxel_side"]
        options.edge_step_size = cfg["lazy_edge_step"]
        options.max_task_space_distance_between_nodes = cfg["max_task_space_distance"]
        options.max_configuration_distance_between_nodes = cfg[
            "max_configuration_distance"
        ]
        options.nodes_processed_before_debug_statement = 100000
        builder = csd.RoadmapBuilder(self.plant, "point", options)
        started = time.perf_counter()
        builder.add_nodes_manual(nodes.T)
        builder.build_roadmap(cfg["roadmap_neighbors"])
        drm = builder.get_drm()
        builder.write(str(Path(out) / "native_roadmap.map"))
        build_seconds = time.perf_counter() - started
        po = csd.DrmPlannerOptions()
        po.online_edge_step_size = cfg["lazy_edge_step"]
        po.try_shortcutting = True
        po.voxel_padding = 0.0
        po.max_nodes_to_expand = 100000
        planner = csd.DrmPlanner(self.plant, po, True)
        planner.LoadRoadmap(str(Path(out) / "native_roadmap.map"))
        planner.BuildCollisionSet(self.empty_voxels)
        started = time.perf_counter()
        success, path = planner.Plan(
            np.asarray(cfg["start"], dtype=np.float32),
            np.asarray(cfg["goal"], dtype=np.float32),
            self.empty_voxels,
            1e-6,
        )
        plan_seconds = time.perf_counter() - started
        point_map = drm.id_to_node_map
        ids = sorted(point_map)
        node_array = np.array([point_map[i] for i in ids])
        edges = np.array(
            [(i, j) for i, adj in drm.node_adjacency_map.items() for j in adj],
            dtype=np.int32,
        )
        np.savez_compressed(
            Path(out) / "roadmap.npz",
            node_ids=ids,
            nodes=node_array,
            edges=edges,
            initial_samples=nodes,
            path=np.asarray(path),
        )
        record = dict(
            success=bool(success),
            requested_nodes=size,
            actual_nodes=len(ids),
            offline_build_seconds=build_seconds,
            online_seconds=plan_seconds,
            path_length=(
                float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
                if success
                else None
            ),
            segments=len(path) - 1 if success else None,
        )
        if success:
            record["validation"] = self.validate_polygon(path)
        atomic(Path(out) / "roadmap.json", record)
        return record, np.array(path), drm

    def alternate_path(self, drm, blocked_region):
        points = drm.id_to_node_map
        graph = nx.Graph()
        a, b = blocked_region.A(), blocked_region.b()
        blocked = {i for i, q in points.items() if np.all(a @ q <= b + 1e-7)}
        for i, q in points.items():
            if i not in blocked:
                graph.add_node(i)
        for i, adj in drm.node_adjacency_map.items():
            for j in adj:
                if i in graph and j in graph and i != j:
                    if (
                        self.segment_clearances(np.array([points[i], points[j]]))[0][0]
                        >= 0
                    ):
                        graph.add_edge(
                            i, j, weight=float(np.linalg.norm(points[i] - points[j]))
                        )
        local = dict(points)
        for node, q in [
            (0, np.array(self.cfg["start"])),
            (-1, np.array(self.cfg["goal"])),
        ]:
            local[node] = q
            graph.add_node(node)
            connected = 0
            for i in sorted(points, key=lambda i: np.linalg.norm(points[i] - q)):
                if i in blocked:
                    continue
                if self.segment_clearances(np.array([q, points[i]]))[0][0] >= 0:
                    graph.add_edge(node, i, weight=float(np.linalg.norm(points[i] - q)))
                    connected += 1
                    if connected == self.cfg["roadmap_neighbors"]:
                        break
        try:
            route = nx.astar_path(
                graph,
                0,
                -1,
                heuristic=lambda u, v: float(np.linalg.norm(local[u] - local[v])),
                weight="weight",
            )
        except nx.NetworkXNoPath:
            return None
        path = np.array([local[i] for i in route])
        simple = [path[0]]
        i = 0
        while i < len(path) - 1:
            for j in range(len(path) - 1, i, -1):
                if self.segment_clearances(np.array([path[i], path[j]]))[0][0] >= 0:
                    simple.append(path[j])
                    i = j
                    break
            else:
                return None
        return np.array(simple)
