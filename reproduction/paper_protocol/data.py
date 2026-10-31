"""Frozen random scenes, real pose IK, and reusable native dynamic roadmaps."""

from pathlib import Path
import time
import numpy as np
import csdecomp as csd
from reproduction.common import atomic, read
from .model import FR3
from .validation import polygon_samples


def make_scene(cfg, seed, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(np.random.SeedSequence([250410783, seed]))
    s = cfg["fr3_scene"]
    centers = rng.uniform(
        s["obstacle_lower"], s["obstacle_upper"], (s["obstacle_count"], 3)
    )
    f = FR3(cfg, centers)
    right = []
    left = []
    tried = 0
    while (not right or len(left) < cfg["fr3_scene"]["ik_attempts"] + 1) and tried < s[
        "endpoint_sampling_limit"
    ]:
        q = rng.uniform(f.lower, f.upper, (2048, 7))
        tried += len(q)
        for point in q[f.cuda(q)]:
            pose = f.pose(point)
            xyz = pose[:3, 3]
            if np.all(xyz >= s["right_ee_lower"]) and np.all(
                xyz <= s["right_ee_upper"]
            ):
                right.append(point)
            if np.all(xyz >= s["left_ee_lower"]) and np.all(xyz <= s["left_ee_upper"]):
                left.append(point)
    record = dict(
        seed=seed,
        obstacles=centers,
        obstacle_radius=s["obstacle_radius"],
        configurations_sampled=tried,
        ik_attempts=[],
        status="endpoint_sampling_failed",
    )
    if right and len(left) > 1:
        start = right[0]
        target = f.pose(left[0])
        record.update(
            start=start, target_pose=target, target_generating_configuration=left[0]
        )
        for initial in left[1 : 1 + s["ik_attempts"]]:
            tic = time.perf_counter()
            ok, goal, status = f.ik(target, initial)
            check = bool(ok and f.cuda(goal[None])[0] and f.independent(goal[None])[0])
            pose = f.pose(goal)
            position_error = float(np.linalg.norm(pose[:3, 3] - target[:3, 3]))
            rotation_error = float(np.linalg.norm(pose[:3, :3] - target[:3, :3]))
            record["ik_attempts"].append(
                dict(
                    initial=initial,
                    solver_status=status,
                    solver_success=ok,
                    independently_valid=check,
                    position_error=position_error,
                    rotation_matrix_error=rotation_error,
                    seconds=time.perf_counter() - tic,
                    goal=goal,
                )
            )
            if check and position_error < 2e-5 and rotation_error < 2e-4:
                record.update(status="ready", goal=goal)
                break
        if record["status"] != "ready":
            record["status"] = "goal_ik_failed"
    atomic(out / "scene.json", record)
    return record


def planner(environment):
    options = csd.DrmPlannerOptions()
    options.online_edge_step_size = environment.protocol["lazy_edge_step"]
    options.max_nodes_to_expand = 20000
    options.max_number_planning_attempts = 20
    options.try_shortcutting = True
    options.voxel_padding = 0.0
    return csd.DrmPlanner(environment.native, options, True)


def build_roadmap(cfg, seed, size, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    f = FR3(cfg)
    rng = np.random.default_rng(np.random.SeedSequence([2025, seed]))
    selected = []
    while len(selected) < size:
        q = rng.uniform(f.lower, f.upper, (32768, 7)).astype(np.float32)
        selected.extend(q[f.cuda(q)])
    nodes = np.asarray(selected[:size])
    np.save(out / "samples.npy", nodes)
    options = csd.RoadmapOptions()
    s = cfg["fr3_scene"]
    options.robot_map_size_x, options.robot_map_size_y, options.robot_map_size_z = s[
        "offline_map_size"
    ]
    options.map_center = np.array(s["offline_map_center"])
    options.offline_voxel_resolution = cfg["offline_voxel_side"] / 2
    options.max_task_space_distance_between_nodes = cfg["max_task_space_distance"]
    options.max_configuration_distance_between_nodes = cfg["max_configuration_distance"]
    options.edge_step_size = cfg["lazy_edge_step"]
    options.nodes_processed_before_debug_statement = 100000
    builder = csd.RoadmapBuilder(f.native, "fr3::fr3_hand_tcp", options)
    tic = time.perf_counter()
    builder.add_nodes_manual(nodes.T)
    builder.build_roadmap(cfg["roadmap_neighbors"])
    builder.build_pose_map()
    builder.build_collision_map()
    builder.write(str(out / "native.map"))
    record = dict(
        seed=seed,
        requested_nodes=size,
        actual_nodes=len(builder.get_drm().id_to_node_map),
        offline_seconds=time.perf_counter() - tic,
        voxel_side=cfg["offline_voxel_side"],
        native_voxel_half_side=options.offline_voxel_resolution,
        robot_spheres=33,
        active_robot_spheres=len(f.geometry_ids),
        static_candidate_pairs=f.minimal.num_collision_pairs,
    )
    atomic(out / "roadmap.json", record)
    return record


def attach_alternate(environment, map_path, case_dir):
    environment.map_path = Path(map_path)
    environment.case_dir = Path(case_dir)

    def alternate(drm, region):
        copy = csd.DRM()
        copy.Read(str(environment.map_path))
        nodes = copy.id_to_node_map
        kept = {i: q for i, q in nodes.items() if not region.PointInSet(q, 1e-7)}
        copy.id_to_node_map = kept
        copy.node_adjacency_map = {
            i: [j for j in adj if j in kept]
            for i, adj in copy.node_adjacency_map.items()
            if i in kept
        }
        copy.id_to_pose_map = {
            i: pose for i, pose in copy.id_to_pose_map.items() if i in kept
        }
        target = environment.case_dir / "alternate_roadmap.map"
        copy.Write(str(target))
        plan = planner(environment)
        plan.LoadRoadmap(str(target))
        plan.BuildCollisionSet(environment.voxels)
        ok, path = plan.Plan(
            environment.start,
            environment.goal,
            environment.voxels,
            environment.voxel_radius,
        )
        path = np.asarray(path)
        if (
            ok
            and environment.independent(
                polygon_samples(path, environment.cfg["validation_step"])
            ).all()
        ):
            return path
        return None

    environment.alternate_path = alternate
