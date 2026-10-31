"""Released 7-DoF example pipeline, explicitly auxiliary to the missing FR3 data.

This reproduces the public DRM -> EI-ZO -> SCSTrajopt pipeline with the supplied
Kinova model. It is never labelled as Table I's FR3 or Table II's real hardware.
"""

from __future__ import annotations
import datetime
import hashlib
import json
from pathlib import Path
import time
import traceback
import numpy as np
import csdecomp as csd
from pydrake.all import (
    RobotDiagramBuilder,
    LoadModelDirectives,
    ProcessModelDirectives,
    SceneGraphCollisionChecker,
    RigidTransform,
    Sphere,
    CoulombFriction,
    HPolyhedron,
)
from scsplanning import biconvex
from .common import ROOT, atomic, read
from .optimization import bezier_values

ASSETS = ROOT / "csdecomp/tests/test_assets"
START = np.array(
    [-0.79714745, 1.8618815, -2.0683706, 0.36096397, -2.404603, 1.9386684, -1.823328]
)
GOAL = np.array([0.6, -0.5, 0.2])


def model(voxel_centers=None, voxel_radius=0.04):
    builder = RobotDiagramBuilder()
    parser = builder.parser()
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    parser.package_map().Add("test_assets", str(ASSETS))
    parser.package_map().Add(
        "kortex_description", str(ASSETS / "kinova/kortex_description")
    )
    parser.package_map().Add("robotiq_arg85_description", str(ASSETS / "robotiq"))
    directives = LoadModelDirectives(
        str(ASSETS / "directives/kinova_sens_on_table.yml")
    )
    ProcessModelDirectives(directives, plant, parser)
    if voxel_centers is not None:
        for i, point in enumerate(voxel_centers):
            plant.RegisterCollisionGeometry(
                plant.world_body(),
                RigidTransform(point),
                Sphere(voxel_radius),
                f"reproduction_voxel_{i}",
                CoulombFriction(0.5, 0.5),
            )
    plant.Finalize()
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    pc = plant.GetMyContextFromRoot(context)
    sc = scene_graph.GetMyContextFromRoot(context)
    inspector = scene_graph.get_query_output_port().Eval(sc).inspector()
    assert plant.num_positions() == 7
    robot_instances = [
        plant.GetModelInstanceByName("kinova"),
        plant.GetModelInstanceByName("robotiq"),
    ]
    return diagram, context, plant, pc, inspector, robot_instances


def align(region, u, v):
    a = np.asarray(region.A(), float)
    b = np.asarray(region.b(), float)
    correction = np.maximum(np.maximum(a @ u - b, a @ v - b), 0)
    if correction.max() > 1e-5:
        raise ValueError("EI-ZO endpoint containment failed")
    return HPolyhedron(a, b + correction + 1e-7)


def draw_voxels(seed):
    # Same distribution as released get_fake_voxel_map; all 15 seeds are frozen.
    random = np.random.RandomState(seed)
    return random.rand(20, 3) * np.array([1.3, 2.4, 1.5]) + np.array([-0.1, -1.2, 0.0])


def sample_path(path, step=0.005):
    pieces = []
    for u, v in zip(path[:-1], path[1:]):
        count = max(2, int(np.ceil(np.linalg.norm(v - u) / step)) + 1)
        pieces.append(np.linspace(u, v, count)[:-1])
    return np.vstack(pieces + [path[-1:]])


def run():
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    out = ROOT / "reproduction/runs" / (stamp + "_kinova_auxiliary")
    out.mkdir(parents=True)
    manifest = dict(
        scope="released_Kinova_7DOF_auxiliary_not_original_FR3_or_hardware",
        requested_cases=15,
        environment_seeds=list(range(15)),
        roadmap_nodes=3000,
        roadmap_seed=2025,
        voxel_radius=0.04,
        offline_voxel_side=0.06,
        goal_selection="released-example nearest roadmap node by end-effector pose, not full paper IK",
        velocity_limit=1.0,
        acceleration_limit=1.0,
        degree=6,
        eizo=dict(
            delta=0.005,
            epsilon=0.005,
            num_particles=10000,
            mixing_steps=60,
            configuration_margin=0.01,
            max_hyperplanes_per_iteration=10,
            bisection_steps=9,
        ),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        native_repository_commit="7661a97bb7c4931be73d2bc4f8c45c3094ff476e",
        status="running",
    )
    atomic(out / "manifest.json", manifest)
    print("AUX_RUN_DIR=" + str(out), flush=True)
    diagram, context, plant, pc, inspector, instances = model()
    converted = csd.convert_drake_plant_to_csd_plant(plant, pc, inspector)
    minimal = converted.getMinimalPlant()
    geometry_ids = converted.getRobotGeometryIds()
    tree = converted.getKinematicTree()
    domain = csd.HPolyhedron()
    domain.MakeBox(tree.get_position_lower_limits(), tree.get_position_upper_limits())
    lower = tree.get_position_lower_limits()
    upper = tree.get_position_upper_limits()
    rng = np.random.default_rng(2025)
    selected = []
    while len(selected) < 3000:
        candidates = rng.uniform(lower, upper, (6000, 7)).astype(np.float32)
        valid = np.array(csd.CheckCollisionFreeCuda(candidates.T, minimal), bool)
        selected.extend(candidates[valid])
    samples = np.asarray(selected[:3000])
    np.savez_compressed(
        out / "roadmap_samples.npz", samples=samples, lower=lower, upper=upper
    )
    options = csd.RoadmapOptions()
    options.robot_map_size_x = 1.5
    options.robot_map_size_y = 2.0
    options.robot_map_size_z = 1.0
    options.map_center = np.array([0.3, 0, 0.5])
    # The native API stores a half-side, whereas the paper calls s the voxel side.
    options.offline_voxel_resolution = 0.03
    options.max_task_space_distance_between_nodes = 10.0
    options.max_configuration_distance_between_nodes = 10.0
    options.edge_step_size = 0.1
    options.nodes_processed_before_debug_statement = 100000
    builder = csd.RoadmapBuilder(converted, "kinova::end_effector_link", options)
    tic = time.perf_counter()
    builder.add_nodes_manual(samples.T)
    builder.build_roadmap(10)
    builder.build_pose_map()
    builder.build_collision_map()
    builder.write(str(out / "kinova.map"))
    manifest["offline_seconds"] = time.perf_counter() - tic
    manifest["robot_collision_geometry_ids"] = len(geometry_ids)
    manifest["static_collision_pairs"] = minimal.num_collision_pairs
    atomic(out / "manifest.json", manifest)
    po = csd.DrmPlannerOptions()
    po.online_edge_step_size = 0.01
    po.max_nodes_to_expand = 10000
    po.max_number_planning_attempts = 20
    po.try_shortcutting = True
    po.voxel_padding = 0.01
    planner = csd.DrmPlanner(converted, po, True)
    planner.LoadRoadmap(str(out / "kinova.map"))
    domain_drake = HPolyhedron.MakeBox(lower, upper)
    velocity = HPolyhedron.MakeBox(-np.ones(7), np.ones(7))
    acceleration = HPolyhedron.MakeBox(-np.ones(7), np.ones(7))
    records = []
    for seed in range(15):
        dest = out / f"e{seed:02d}"
        dest.mkdir()
        record = dict(seed=seed, status="running")
        tic = time.perf_counter()
        voxels = draw_voxels(seed)
        voxel = csd.Voxels(voxels.T)
        np.savez_compressed(
            dest / "input.npz", voxels=voxels, start=START, goal_translation=GOAL
        )
        try:
            if not csd.CheckCollisionFreeVoxelsCuda(
                START.reshape(7, 1), voxel, 0.04, minimal, geometry_ids
            )[0]:
                record.update(
                    status="invalid_start",
                    reason="frozen start collides with this unmodified voxel scene",
                )
                continue
            planner.BuildCollisionSet(voxel)
            target = np.eye(4)
            target[:3, 3] = GOAL
            goals = planner.GetClosestNonCollidingConfigurationsByPose(target, 100, 0.2)
            if not len(goals):
                record.update(status="goal_not_found")
                continue
            goal = np.asarray(goals[0])
            ok, path = planner.Plan(START, goal, voxel, 0.04)
            path = np.asarray(path)
            if not ok:
                record.update(status="planner_failed")
                continue
            np.savez_compressed(dest / "path.npz", path=path, goal=goal)
            (
                check_diagram,
                check_context,
                check_plant,
                check_pc,
                check_inspector,
                check_instances,
            ) = model(voxels)
            checker = SceneGraphCollisionChecker(
                model=check_diagram,
                robot_model_instances=check_instances,
                edge_step_size=0.005,
                env_collision_padding=0.0,
                self_collision_padding=0.0,
            )
            dense = sample_path(path)
            if not np.array(
                checker.CheckConfigsCollisionFree(dense, parallelize=True), bool
            ).all():
                record.update(status="invalid_seed_path")
                continue
            record["drm_seconds"] = time.perf_counter() - tic
            eizo = csd.EizoOptions()
            for name, value in manifest["eizo"].items():
                setattr(eizo, name, value)
            eizo.seed = 1337
            eizo.verbose = False
            eizo.track_iteration_information = False
            inflator = csd.CudaEdgeInflator(minimal, geometry_ids, eizo, domain)
            regions = []
            edges = []
            construction = 0.0
            optimization = 0.0
            validation_seconds = 0.0
            history = []

            def inflate_missing():
                for u, v in zip(path[:-1], path[1:]):
                    if not any(
                        r.PointInSet(u, 1e-6) and r.PointInSet(v, 1e-6) for r in regions
                    ):
                        region = align(
                            inflator.inflateEdge(u, v, voxel, 0.04, False), u, v
                        )
                        regions.append(region)
                        edges.append((u, v))
                order = []
                for u, v in zip(path[:-1], path[1:]):
                    found = next(
                        i
                        for i, r in enumerate(regions)
                        if r.PointInSet(u, 1e-6) and r.PointInSet(v, 1e-6)
                    )
                    if not order or order[-1] != found:
                        order.append(found)
                return [regions[i] for i in order], [edges[i] for i in order]

            stage = time.perf_counter()
            regions, edges = inflate_missing()
            construction += time.perf_counter() - stage
            for iteration in range(21):
                stage = time.perf_counter()
                curve = biconvex(START, goal, regions, velocity, acceleration, 6, 1e-2)
                optimization += time.perf_counter() - stage
                controls = np.array([segment.points for segment in curve.curves], float)
                durations = np.array(
                    [segment.duration for segment in curve.curves], float
                )
                np.savez_compressed(
                    dest / f"bezier_{iteration:02d}.npz",
                    controls=controls,
                    durations=durations,
                )
                for i, region in enumerate(regions):
                    np.savez_compressed(
                        dest / f"region_{iteration:02d}_{i:02d}.npz",
                        A=region.A(),
                        b=region.b(),
                    )
                stage = time.perf_counter()
                v = 6 * np.diff(controls, axis=1) / durations[:, None, None]
                a = 5 * np.diff(v, axis=1) / durations[:, None, None]
                count = max(
                    2001,
                    int(
                        np.ceil(
                            durations.sum() * np.linalg.norm(v, axis=2).max() / 0.005
                        )
                    )
                    + 1,
                )
                q, segments = bezier_values(
                    controls, durations, np.linspace(0, durations.sum(), count)
                )
                cuda = np.array(
                    csd.CheckCollisionFreeVoxelsCuda(
                        q.T, voxel, 0.04, minimal, geometry_ids
                    ),
                    bool,
                )
                independent = np.array(
                    checker.CheckConfigsCollisionFree(q, parallelize=True), bool
                )
                endpoint_error = max(
                    np.linalg.norm(q[0] - START), np.linalg.norm(q[-1] - goal)
                )
                bound_ok = bool(
                    np.max(np.abs(v)) <= 1 + 1e-5 and np.max(np.abs(a)) <= 1 + 1e-5
                )
                region_residual = max(
                    np.max(r.A() @ c.T - r.b()[:, None])
                    for r, c in zip(regions, controls)
                )
                passed = bool(
                    independent.all()
                    and cuda.all()
                    and bound_ok
                    and endpoint_error < 1e-5
                    and region_residual < 1e-5
                )
                validation = dict(
                    passed=passed,
                    samples=len(q),
                    cuda_collisions=int((~cuda).sum()),
                    independent_drake_collisions=int((~independent).sum()),
                    disagreements=int((cuda != independent).sum()),
                    velocity_bound=float(np.abs(v).max()),
                    acceleration_bound=float(np.abs(a).max()),
                    endpoint_error=float(endpoint_error),
                    region_residual=float(region_residual),
                    guarantee="independent_numerical_validation_not_continuous_certificate",
                )
                atomic(dest / f"validation_{iteration:02d}.json", validation)
                np.savez_compressed(
                    dest / f"samples_{iteration:02d}.npz",
                    q=q,
                    cuda=cuda,
                    independent=independent,
                )
                validation_seconds += time.perf_counter() - stage
                history.append(validation)
                if passed or (independent & cuda).all() or iteration == 20:
                    break
                if np.any((~independent) & cuda):
                    record["unresolved_model_disagreement"] = True
                    break
                stage = time.perf_counter()
                hits = np.flatnonzero(~cuda)
                options_edit = csd.EditRegionsOptions()
                options_edit.configuration_margin = 0.01
                options_edit.bisection_steps = 9
                options_edit.verbose = False
                edited, _ = csd.EditRegionsCuda(
                    np.asarray(q[hits].T, np.float32),
                    segments[hits].astype(np.uint32).tolist(),
                    np.asarray([u for u, vv in edges], np.float32).T,
                    np.asarray([vv for u, vv in edges], np.float32).T,
                    [
                        csd.HPolyhedron(
                            np.asarray(r.A(), np.float32), np.asarray(r.b(), np.float32)
                        )
                        for r in regions
                    ],
                    minimal,
                    geometry_ids,
                    voxel,
                    0.04,
                    options_edit,
                )
                regions = [
                    HPolyhedron(np.asarray(r.A(), float), np.asarray(r.b(), float))
                    for r in edited
                ]
                regions, edges = inflate_missing()
                construction += time.perf_counter() - stage
            record.update(
                status="success_numerical" if passed else "validation_failed",
                duration=float(durations.sum()),
                regions=len(regions),
                recovery_iterations=len(history) - 1,
                construction_seconds=construction,
                optimization_seconds=optimization,
                validation_seconds=validation_seconds,
                validation=validation,
            )
        except Exception as error:
            traceback.print_exc()
            record.update(
                status="error", reason=repr(error), traceback=traceback.format_exc()
            )
        finally:
            record["wall_seconds"] = time.perf_counter() - tic
            records.append(record)
            atomic(dest / "result.json", record)
            atomic(out / "results.json", records)
            print(
                "Kinova",
                seed,
                record["status"],
                f"{record['wall_seconds']:.3f}s",
                flush=True,
            )
    manifest["status"] = "completed"
    atomic(out / "manifest.json", manifest)
    summary = dict(
        requested=15,
        status_counts={
            status: sum(row["status"] == status for row in records)
            for status in sorted({row["status"] for row in records})
        },
        scope=manifest["scope"],
        source_sha256=manifest["source_sha256"],
    )
    atomic(out / "summary.json", summary)
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    run()
