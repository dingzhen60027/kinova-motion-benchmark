"""Paper Section V Forest DRM with online circular obstacles.

The same empty-workspace roadmap is used across all environment seeds. Native
voxel maps, lazy checks and shortcutting are exercised, as in the FR3 adapter.
No graph search is replaced by NetworkX.
"""
from pathlib import Path
import time
import numpy as np
import csdecomp as csd
from reproduction.common import atomic
from .model import ForestEnvironment


def build_roadmap(cfg, seed, size, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    f = ForestEnvironment(cfg, 0)
    rng = np.random.default_rng(np.random.SeedSequence([2025, seed]))
    nodes = rng.uniform(f.lower, f.upper, (size, 2)).astype(np.float32)
    np.save(out / "samples.npy", nodes)
    options = csd.RoadmapOptions()
    options.robot_map_size_x = 10.0
    options.robot_map_size_y = 10.0
    options.robot_map_size_z = cfg["offline_voxel_side"]
    options.map_center = np.zeros(3)
    options.offline_voxel_resolution = cfg["offline_voxel_side"] / 2
    options.max_task_space_distance_between_nodes = cfg["max_task_space_distance"]
    options.max_configuration_distance_between_nodes = cfg["max_configuration_distance"]
    options.edge_step_size = cfg["lazy_edge_step"]
    options.nodes_processed_before_debug_statement = 100000
    builder = csd.RoadmapBuilder(f.native, "point", options)
    tic = time.perf_counter()
    builder.add_nodes_manual(nodes.T)
    builder.build_roadmap(cfg["roadmap_neighbors"])
    builder.build_pose_map()
    builder.build_collision_map()
    builder.write(str(out / "native.map"))
    drm = builder.get_drm()
    ids = sorted(drm.id_to_node_map)
    np.savez_compressed(out / "roadmap.npz", node_ids=ids,
        nodes=np.array([drm.id_to_node_map[i] for i in ids]),
        edges=np.array([(i,j) for i, adj in drm.node_adjacency_map.items() for j in adj]))
    result = dict(seed=seed, requested_nodes=size, actual_nodes=len(ids),
        offline_seconds=time.perf_counter()-tic, status="ready",
        voxel_side=cfg["offline_voxel_side"],
        native_voxel_half_side=options.offline_voxel_resolution,
        collision_map_entries=len(drm.collision_map),
        active_robot_spheres=len(f.geometry_ids),
        static_candidate_pairs=f.minimal.num_collision_pairs,
        obstacles="online only; independent of offline roadmap")
    atomic(out / "roadmap.json", result)
    return result
