"""Isolate and verify the unmodified paper-date public native core.

The small constructed scene tests the recovered implementation; it is not an
original paper benchmark. Each native test gets a separate Python process so a
native failure is retained and cannot erase the other test records.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reproduction/reports/paper_completion"
OUTPUT = REPORT / "source_evidence/paperdate_core"
PACKAGE = ROOT / "reproduction/vendor/paperdate_python"
REVISION = "ee94f8145bb32ef6c08b1e575218335b9daf533f"
RESULT = REPORT / "paperdate_core_result.json"
SECTIONS = ("collision", "drm", "eizo")


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_check():
    records = []
    listing = subprocess.check_output(
        ["git", "ls-tree", "-r", REVISION], cwd=ROOT, text=True
    )
    source = ROOT / "reproduction/vendor/csdecomp_paperdate"
    for line in listing.splitlines():
        meta, name = line.split("\t", 1)
        expected = meta.split()[2]
        data = (source / name).read_bytes()
        actual = hashlib.sha1(
            b"blob " + str(len(data)).encode() + b"\0" + data
        ).hexdigest()
        records.append({"path": name, "expected": expected, "actual": actual})
    return {
        "original_files": len(records),
        "all_original_files_match": all(x["expected"] == x["actual"] for x in records),
        "changed_original_files": [x for x in records if x["expected"] != x["actual"]],
    }


def installed_wheel_manifest():
    distribution = importlib.metadata.distribution("csdecomp")
    libraries = []
    for item in distribution.files or []:
        if str(item).endswith(".so"):
            path = Path(distribution.locate_file(item)).resolve()
            libraries.append({"path": str(path), "sha256": digest(path)})
    return {"version": distribution.version, "native_libraries": libraries}


def test_scene(csd, np):
    radius = 0.0001
    centers = np.array([[0.0, 0.0], [-0.6, 0.6], [0.8, -0.5]])
    obstacle_radius = 0.35
    robot = f'''<link name="slide"/>
<link name="point"><collision name="point"><geometry><sphere radius="{radius}"/></geometry></collision></link>
<joint name="x" type="prismatic"><parent link="world"/><child link="slide"/><axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint>
<joint name="y" type="prismatic"><parent link="slide"/><child link="point"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint>'''
    obstacles = "".join(
        f'<link name="obstacle_{i}"><collision name="obstacle_{i}"><geometry><sphere radius="{obstacle_radius}"/></geometry></collision></link><joint name="obstacle_{i}_weld" type="fixed"><parent link="world"/><child link="obstacle_{i}"/><origin xyz="{x} {y} 0"/></joint>'
        for i, (x, y) in enumerate(centers)
    )
    urdf = '<robot name="probe">' + robot + '</robot>'
    full_urdf = '<robot name="probe">' + robot + obstacles + '</robot>'
    (OUTPUT / "robot.urdf").write_text(urdf)
    (OUTPUT / "full_scene.urdf").write_text(full_urdf)
    parser = csd.URDFParser()
    if not parser.parse_urdf_string(urdf):
        raise RuntimeError("Original parser rejected constructed 2D robot")
    native = parser.build_plant()
    domain = csd.HPolyhedron()
    domain.MakeBox(np.array([-2.0, -2.0]), np.array([2.0, 2.0]))
    voxels = csd.Voxels(np.vstack([centers.T, np.zeros(len(centers))]).astype(np.float32))

    def clearance(q):
        return np.linalg.norm(np.atleast_2d(q)[:, None] - centers[None], axis=2).min(axis=1) - obstacle_radius - radius

    def cuda(q):
        return np.asarray(csd.CheckCollisionFreeVoxelsCuda(
            np.asarray(q, np.float32).T, voxels, obstacle_radius,
            native.getMinimalPlant(), native.getRobotGeometryIds()
        ), dtype=bool)

    def segment_clearances(path):
        start = path[:-1]
        delta = np.diff(path, axis=0)
        t = np.sum((centers[None] - start[:, None]) * delta[:, None], axis=2)
        t /= np.maximum(np.sum(delta**2, axis=1)[:, None], 1e-30)
        closest = start[:, None] + np.clip(t, 0, 1)[:, :, None] * delta[:, None]
        return np.linalg.norm(closest - centers[None], axis=2) - obstacle_radius - radius

    return native, domain, voxels, centers, obstacle_radius, full_urdf, clearance, cuda, segment_clearances


def run_section(section):
    sys.path.insert(0, str(PACKAGE))
    import numpy as np
    import pycsdecomp as csd
    import pycsdecomp.pycsdecomp_bindings as bindings

    if "csdecomp" in sys.modules:
        raise RuntimeError("Modern csdecomp must not be imported in old-core process")
    if not Path(bindings.__file__).resolve().is_relative_to(PACKAGE):
        raise RuntimeError("Wrong native library loaded")
    scene = test_scene(csd, np)
    native, domain, voxels, centers, radius, full_urdf, clearance, cuda, segment_clearances = scene
    record = {
        "section": section, "native_library": str(Path(bindings.__file__).resolve()),
        "native_library_sha256": digest(Path(bindings.__file__)),
        "python_executable": sys.executable, "numpy_version": np.__version__,
        "modern_core_loaded": "csdecomp" in sys.modules,
        "source_revision": REVISION,
        "scene": "constructed 2D point plus 3 online sphere obstacles; not original paper data",
        "configuration_dimension": native.numConfigurationVariables(),
        "robot_geometry_count": len(native.getRobotGeometryIds()),
        "has_gpu_recovery_interface": hasattr(csd, "EditRegionsCuda"),
        "has_eizo_seed_option": hasattr(csd.EizoOptions(), "seed"),
    }
    rng = np.random.default_rng(20250415)
    started = time.perf_counter()
    if section == "collision":
        from pydrake.all import RobotDiagramBuilder
        builder = RobotDiagramBuilder()
        plant, sg = builder.plant(), builder.scene_graph()
        builder.parser().AddModelsFromString(full_urdf, "urdf")
        plant.Finalize()
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        pc = plant.GetMyMutableContextFromRoot(context)
        query_context = sg.GetMyContextFromRoot(context)
        q = rng.uniform(-2, 2, (3000, 2))
        analytic = clearance(q) >= 0
        actual = cuda(q)
        drake = []
        for point in q[:300]:
            plant.SetPositions(pc, point)
            query = sg.get_query_output_port().Eval(query_context)
            drake.append(not any(d.distance < 0 for d in query.ComputeSignedDistancePairwiseClosestPoints(0.0)))
        bridge = csd.convert_drake_plant_to_csd_plant(plant, pc, query.inspector())
        bridge_actual = np.asarray(csd.CheckCollisionFreeCuda(q[:300].astype(np.float32).T, bridge.getMinimalPlant()), bool)
        np.savez_compressed(OUTPUT / "collision.npz", configurations=q, analytic=analytic, original_cuda=actual, drake=drake, original_bridge_cuda=bridge_actual)
        record.update(samples=len(q), cuda_analytic_disagreements=int(np.sum(actual != analytic)),
                      drake_samples=len(drake), cuda_drake_disagreements=int(np.sum(actual[:300] != drake)),
                      bridge_drake_disagreements=int(np.sum(bridge_actual != drake)),
                      collision_free_samples=int(actual.sum()),
                      minimum_absolute_boundary_distance=float(np.min(np.abs(clearance(q)))))
        record["passed"] = all(record[key] == 0 for key in ("cuda_analytic_disagreements", "cuda_drake_disagreements", "bridge_drake_disagreements"))
    elif section == "drm":
        options = csd.RoadmapOptions()
        options.robot_map_size_x = 4.0
        options.robot_map_size_y = 4.0
        options.robot_map_size_z = 0.4
        options.map_center = np.zeros(3)
        options.offline_voxel_resolution = 0.2
        options.edge_step_size = 0.02
        options.max_task_space_distance_between_nodes = 1.5
        options.max_configuration_distance_between_nodes = 1.5
        options.nodes_processed_before_debug_statement = 10000
        nodes = rng.uniform(-2, 2, (300, 2)).astype(np.float32)
        builder = csd.RoadmapBuilder(native, "point", options)
        builder.add_nodes_manual(nodes.T)
        builder.build_roadmap(20)
        builder.build_pose_map()
        builder.build_collision_map()
        map_path = OUTPUT / "original_core_roadmap.map"
        builder.write(str(map_path))
        drm = builder.get_drm()
        planner_options = csd.DrmPlannerOptions()
        planner_options.online_edge_step_size = 0.01
        planner_options.try_shortcutting = True
        planner_options.voxel_padding = 0.0
        planner_options.max_nodes_to_expand = 10000
        planner = csd.DrmPlanner(native, planner_options, True)
        planner.LoadRoadmap(str(map_path))
        planner.BuildCollisionSet(voxels)
        start, goal = np.array([-1.8, -1.5]), np.array([1.8, 1.5])
        success, path = planner.Plan(start, goal, voxels, radius)
        path = np.asarray(path)
        np.savez_compressed(OUTPUT / "drm.npz", initial_nodes=nodes, path=path, centers=centers)
        margins = segment_clearances(path) if success and len(path) > 1 else None
        endpoint_error = float(max(np.linalg.norm(path[0] - start), np.linalg.norm(path[-1] - goal))) if success and len(path) else None
        record.update(requested_nodes=300, actual_nodes=len(drm.id_to_node_map),
                      directed_edges=sum(map(len, drm.node_adjacency_map.values())),
                      collision_map_entries=len(drm.collision_map), blocked_nodes=len(planner.collision_set),
                      plan_success=bool(success), path_nodes=len(path), endpoint_error=endpoint_error,
                      exact_segment_minimum_clearance=float(margins.min()) if margins is not None else None,
                      map_sha256=digest(map_path), original_offline_voxel_half_side=0.2,
                      original_offline_voxel_radius=float(options.get_offline_voxel_radius()))
        record["passed"] = bool(success and endpoint_error < 1e-5 and margins is not None and margins.min() >= 0)
    elif section == "eizo":
        options = csd.EizoOptions()
        options.configuration_margin = 0.005
        options.tau = 0.5
        options.delta = 0.05
        options.epsilon = 0.01
        options.bisection_steps = 5
        options.num_particles = 1000
        options.max_hyperplanes_per_iteration = 10
        options.max_iterations = 30
        options.mixing_steps = 50
        options.verbose = False
        options.track_iteration_information = False
        edge = np.array([[-1.8, -1.5], [-1.0, -1.5]])
        inflator = csd.CudaEdgeInflator(native.getMinimalPlant(), native.getRobotGeometryIds(), options, domain)
        region = inflator.inflateEdge(edge[0], edge[1], voxels, radius, False)
        a, b = np.asarray(region.A()), np.asarray(region.b())
        samples = np.asarray(csd.UniformSampleInHPolyhedraCuda([region], region.ChebyshevCenter(), 2000, 50, 20250415)[0]).T
        margins = clearance(samples)
        seed_excess = float(np.max(a @ edge.T - b[:, None]))
        np.savez_compressed(OUTPUT / "eizo.npz", A=a, b=b, seed=edge, samples=samples, independent_clearance=margins)
        record.update(faces=len(b), seed_maximum_constraint_excess=seed_excess,
                      independent_validation_samples=len(samples), independent_collisions=int(np.sum(margins < 0)),
                      empirical_collision_fraction=float(np.mean(margins < 0)),
                      minimum_sample_clearance=float(margins.min()),
                      original_eizo_sampling_seed=1337, validation_sampling_seed=20250415,
                      options={name: getattr(options, name) for name in ("configuration_margin", "tau", "delta", "epsilon", "bisection_steps", "num_particles", "max_hyperplanes_per_iteration", "max_iterations", "mixing_steps")})
        record["passed"] = bool(seed_excess <= 1e-5 and np.mean(margins < 0) <= options.epsilon)
        record["validation_limit"] = "Empirical sampling and seed containment; not a deterministic certificate of the full region."
    record["elapsed_seconds"] = time.perf_counter() - started
    return record


def main():
    global OUTPUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true", help="Build original core before isolated verification")
    parser.add_argument("--section", choices=SECTIONS, help=argparse.SUPPRESS)
    parser.add_argument("--output", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.section:
        if args.output is None:
            parser.error("Internal section execution requires --output")
        OUTPUT = args.output
    else:
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        OUTPUT = OUTPUT / stamp
    OUTPUT.mkdir(parents=True, exist_ok=True)
    if args.section:
        try:
            result = run_section(args.section)
        except Exception:
            result = {"section": args.section, "passed": False, "error": traceback.format_exc()}
        write(OUTPUT / (args.section + ".json"), result)
        print(json.dumps(result, indent=2))
        return 0 if result["passed"] else 1
    if RESULT.exists():
        (OUTPUT / "previous_result.json").write_bytes(RESULT.read_bytes())
    result = {
        "status": "running", "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "source_revision": REVISION, "source_url": "https://github.com/wernerpe/csdecomp/tree/" + REVISION,
        "scope": "Original public core build and small constructed collision/DRM/EI-ZO verification; not original full experiment data",
        "conda_wheel_replaced": False, "source_before": source_check(), "checks": {},
        "configured": True, "compiled": False, "build_performed_this_attempt": args.build,
        "raw_output_directory": str(OUTPUT.relative_to(ROOT)),
        "installed_wheel_before": installed_wheel_manifest(),
    }
    write(RESULT, result)
    if not result["source_before"]["all_original_files_match"]:
        result.update(status="failed", error="Original algorithm/source file mismatch before build")
        write(RESULT, result)
        return 1
    if args.build:
        command = ["bash", str(ROOT / "reproduction/build_paperdate_core.sh")]
        printed = subprocess.check_output(command + ["--print-command"], cwd=ROOT, text=True).strip()
        result["build"] = {"argv": command, "resolved_shell_command": printed, "status": "running"}
        write(RESULT, result)
        log = OUTPUT / "build.log"
        began = time.perf_counter()
        with log.open("w") as stream:
            completed = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        result["build"].update(returncode=completed.returncode, status="passed" if completed.returncode == 0 else "failed", elapsed_seconds=time.perf_counter() - began, log=str(log.relative_to(ROOT)), log_sha256=digest(log))
        write(RESULT, result)
        if completed.returncode:
            result.update(status="build_failed", source_after=source_check(), installed_wheel_after=installed_wheel_manifest())
            result["installed_wheel_unchanged"] = result["installed_wheel_before"] == result["installed_wheel_after"]
            write(RESULT, result)
            return 1
    library = PACKAGE / "pycsdecomp/pycsdecomp_bindings.so"
    result["load_path"] = str(library)
    if not library.exists():
        result.update(status="failed", error="Original native library has not been built")
        write(RESULT, result)
        return 1
    result["library_sha256"] = digest(library)
    result["compiled"] = True
    for section in SECTIONS:
        command = [sys.executable, str(Path(__file__).resolve()), "--section", section, "--output", str(OUTPUT)]
        log = OUTPUT / (section + ".log")
        environment = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")
        began = time.perf_counter()
        with log.open("w") as stream:
            try:
                completed = subprocess.run(command, cwd=ROOT, env=environment, stdout=stream, stderr=subprocess.STDOUT, timeout=180)
                returncode = completed.returncode
            except subprocess.TimeoutExpired:
                returncode = 124
        child = OUTPUT / (section + ".json")
        check = json.loads(child.read_text()) if child.exists() else {"passed": False, "error": "Native process returned without JSON; inspect retained log"}
        check.update(argv=command, returncode=returncode, process_elapsed_seconds=time.perf_counter() - began, log=str(log.relative_to(ROOT)), log_sha256=digest(log))
        result["checks"][section] = check
        write(RESULT, result)
    result["source_after"] = source_check()
    result["installed_wheel_after"] = installed_wheel_manifest()
    result["installed_wheel_unchanged"] = result["installed_wheel_before"] == result["installed_wheel_after"]
    passed = all(x.get("passed") and x["returncode"] == 0 for x in result["checks"].values())
    result["status"] = "passed" if passed and result["source_after"]["all_original_files_match"] and result["installed_wheel_unchanged"] else "verification_failed"
    result["finished_at_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    write(RESULT, result)
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
