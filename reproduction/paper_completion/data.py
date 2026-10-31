"""Frozen random scenes, real pose IK, and reusable native dynamic roadmaps."""

from pathlib import Path
import time
import numpy as np
import csdecomp as csd
from reproduction.common import atomic, read
from .model import FR3
from .validation import polygon_samples


def endpoint_audit(environment, original):
    original = np.asarray(original, float)
    canonical = original.astype(np.float32).astype(float)
    pair = np.vstack([original, canonical])
    cuda = environment.cuda(pair)
    independent = environment.independent(pair)
    bounds = np.all((pair >= environment.lower) & (pair <= environment.upper), axis=1)
    return dict(original=original, canonical=canonical,
                representation="float32_roundtrip_without_endpoint_displacement",
                joint_l2_error=float(np.linalg.norm(original - canonical)),
                original_cuda_valid=bool(cuda[0]),
                original_independent_valid=bool(independent[0]),
                canonical_cuda_valid=bool(cuda[1]),
                canonical_independent_valid=bool(independent[1]),
                original_limits_valid=bool(bounds[0]), canonical_limits_valid=bool(bounds[1]),
                original_valid=bool(cuda[0] and independent[0] and bounds[0]),
                canonical_valid=bool(cuda[1] and independent[1] and bounds[1]))


def online_pose_ik(environment, drm, target_pose, out, collision_set=None):
    """Paper V-B: warm-start IK from the collision-free roadmap pose map.

    A fixed target pose is retained for every candidate and roadmap. Selection
    never depends on eventual planning or trajectory-optimization success.
    """
    started = time.perf_counter()
    out = Path(out)
    target = np.asarray(target_pose, float)
    blocked = set(collision_set or [])
    nodes, poses = drm.id_to_node_map, drm.id_to_pose_map
    ranked = []
    for node_id, pose in poses.items():
        if node_id in blocked or node_id not in nodes:
            continue
        pose = np.asarray(pose, float)
        translation = float(np.linalg.norm(pose[:3, 3] - target[:3, 3]))
        relative = target[:3, :3].T @ pose[:3, :3]
        angle = float(np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1)))
        ranked.append((translation + 0.2 * angle, node_id))
    ranked.sort()
    record = dict(status="ik_failed", target_pose=target, attempts=[],
                  collision_filter_rejections=[], goal=None,
                  candidate_selection=dict(
                      source="roadmap_pose_map_after_collision_set",
                      metric="translation_L2_plus_0.2_times_rotation_angle",
                      metric_source="released_native_measurePoseDistance",
                      position_cutoff=None, tie_break="ascending_node_id",
                      collision_set_nodes=len(blocked), ranked_nodes=len(ranked),
                      maximum_ik_attempts=environment.protocol["fr3_scene"]["ik_attempts"],
                      cutoff_and_attempt_limit="unpublished_reconstruction_choices"))
    maximum = environment.protocol["fr3_scene"]["ik_attempts"]
    for distance, node_id in ranked:
        initial = np.asarray(nodes[node_id], np.float32).astype(float)
        if not (environment.cuda(initial[None])[0] and
                environment.independent(initial[None])[0]):
            record["collision_filter_rejections"].append(node_id)
            continue
        tic = time.perf_counter()
        ok, raw_goal, status = environment.ik(target, initial)
        raw_goal = np.asarray(raw_goal, float)
        attempt = dict(node_id=node_id, pose_distance=distance, initial=initial,
                       solver_success=bool(ok), solver_status=status,
                       seconds=time.perf_counter() - tic)
        if np.isfinite(raw_goal).all():
            audit = endpoint_audit(environment, raw_goal)
            canonical = np.asarray(audit["canonical"])
            pose = environment.pose(canonical)
            position_error = float(np.linalg.norm(pose[:3, 3] - target[:3, 3]))
            rotation_error = float(np.linalg.norm(pose[:3, :3] - target[:3, :3]))
            accepted = bool(ok and audit["canonical_valid"] and
                            position_error < 2e-5 and rotation_error < 2e-4)
            attempt.update(endpoint_audit=audit, position_error=position_error,
                           rotation_matrix_error=rotation_error, accepted=accepted)
        else:
            accepted = False
            attempt.update(accepted=False, reason="nonfinite_solver_configuration")
        record["attempts"].append(attempt)
        record["seconds"] = time.perf_counter() - started
        if accepted:
            record.update(status="success", goal=canonical,
                          selected_node_id=node_id)
        atomic(out / "online_pose_ik.json", record)
        if accepted or len(record["attempts"]) >= maximum:
            break
    record["seconds"] = time.perf_counter() - started
    atomic(out / "online_pose_ik.json", record)
    return record


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
    if right and left:
        # Keep the sampled pose task fixed. IK is an ONLINE, per-roadmap step
        # in paper V-B; a preliminary random-configuration IK must not gate it.
        original_start = np.asarray(right[0], float)
        start = original_start.astype(np.float32).astype(float)
        target = f.pose(left[0])
        audit = endpoint_audit(f, original_start)
        record.update(start=start, original_start=original_start,
                      start_representation=audit, target_pose=target,
                      target_generating_configuration=left[0],
                      goal_selection="online_pose_map_IK_per_roadmap",
                      status="ready" if audit["canonical_valid"] else
                             "start_representation_invalid")
    atomic(out / "scene.json", record)
    return record


def planner(environment):
    options = csd.DrmPlannerOptions()
    options.online_edge_step_size = environment.protocol["lazy_edge_step"]
    # Keep upstream defaults (100000 A* expansions, 100 lazy attempts).
    # The paper does not specify a smaller reconstruction-specific cap.
    options.try_shortcutting = True
    options.voxel_padding = 0.0
    environment.native_planner_options = {
        name: getattr(options, name) for name in (
            "online_edge_step_size", "max_nodes_to_expand",
            "max_number_planning_attempts", "max_iterations_steering_to_node",
            "max_num_blocked_edges_before_discard_node", "try_shortcutting",
            "voxel_padding")}
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
        try:
            ok, path = plan.Plan(
                environment.start, environment.goal, environment.voxels,
                environment.voxel_radius)
        except RuntimeError as exc:
            if "No collision-free path exists in roadmap" not in str(exc):
                raise
            atomic(environment.case_dir / "alternate_path_search.json", dict(
                status="native_no_alternate_path", exception=repr(exc),
                blocked_nodes=len(nodes)-len(kept), first_path_retained=True))
            return None
        path = np.asarray(path)
        valid = False
        if ok and len(path) >= 2:
            samples = polygon_samples(path, environment.cfg["validation_step"])
            valid = bool(environment.cuda(samples).all() and
                         environment.independent(samples).all())
            if environment.kind == "forest":
                valid = bool(valid and environment.base.validate_polygon(path)["collision_free"])
        atomic(environment.case_dir / "alternate_path_search.json", dict(
            status="success" if valid else "no_valid_alternate_path",
            native_success=bool(ok), independently_valid=valid,
            blocked_nodes=len(nodes)-len(kept), first_path_retained=True))
        if valid:
            return path
        return None

    environment.alternate_path = alternate
