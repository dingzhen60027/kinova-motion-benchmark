"""Isolated, resumable paper-parameter Forest and FR3 experiments."""

import argparse
from collections import Counter
import datetime
import hashlib
import itertools
from pathlib import Path
import os
import shutil
import signal
import subprocess
import sys
import time
import traceback
import numpy as np
from reproduction.common import ROOT, atomic, digest, read
from reproduction.run import runtime_versions

PACKAGE = Path(__file__).parent


def identity():
    paths = [
        p
        for p in PACKAGE.rglob("*")
        if p.is_file()
        and p.suffix in [".py", ".json", ".yaml"]
        and p.name not in ["analyze.py", "plot.py", "throughput.py"]
        and "__pycache__" not in p.parts
    ]
    paths += [
        ROOT / "local/python.sh",
        ROOT / "reproduction/common.py",
        ROOT / "reproduction/forest.py",
        ROOT / "reproduction/optimization.py",
        ROOT / "reproduction/pipeline.py",
        ROOT / "reproduction/configs/forest.json",
    ]
    return {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(paths)
    }


def execute_child(cmd, dest, timeout):
    dest.mkdir(parents=True, exist_ok=True)
    with (dest / "stdout.log").open("w") as stdout, (dest / "stderr.log").open(
        "w"
    ) as stderr:
        process = subprocess.Popen(
            cmd, cwd=ROOT, stdout=stdout, stderr=stderr, start_new_session=True
        )
        try:
            process.wait(timeout=timeout)
            return dict(exit_code=process.returncode, timed_out=False)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            return dict(exit_code=process.returncode, timed_out=True)
        except KeyboardInterrupt:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
            raise


def new_run(args):
    cfg = read(PACKAGE / "config.json")
    cases = [
        dict(
            id=f"e{e:02d}_r{r:02d}_n{n:05d}",
            environment_seed=e,
            roadmap_seed=r,
            roadmap_size=n,
        )
        for e, r, n in itertools.product(
            args.environments or cfg["environment_seeds"],
            args.roadmap_seeds or cfg["roadmap_seeds"],
            args.sizes or cfg["roadmap_sizes"][args.environment],
        )
    ]
    if len({r["id"] for r in cases}) != len(cases):
        raise ValueError("Duplicate cases")
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    root = Path(args.output_root or ROOT / "reproduction/runs") / (
        stamp + "_paper_constructed_" + args.environment
    )
    root.mkdir(parents=True)
    files = identity()
    for name in files:
        dest = root / "source_snapshot" / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / name, dest)
    atomic(
        root / "manifest.json",
        dict(
            schema_version=2,
            environment=args.environment,
            paper=cfg["paper"],
            fidelity=cfg["fidelity"],
            created_utc=stamp,
            status="pending",
            config=cfg,
            config_hash=digest(cfg),
            cases=cases,
            planned_cases=len(cases),
            source_files=files,
            implementation_hash=digest(files),
            runtime_versions=runtime_versions(),
            git_commit=subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
            ).strip(),
            working_tree_dirty=bool(
                subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT)
            ),
            threads={
                k: os.getenv(k)
                for k in ["OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"]
            },
        ),
    )
    if args.cache_from:
        reuse_cache(root, Path(args.cache_from).resolve(), cfg, files)
    with (root / "gpu_before.txt").open("w") as stream:
        subprocess.run(["nvidia-smi", "-q"], stdout=stream, check=True)
    return root


def reuse_cache(root, previous, cfg, files):
    """Reuse only compatible frozen FR3 input assets, never method outcomes."""
    old = read(previous / "manifest.json")
    new = read(root / "manifest.json")
    if old["environment"] != "fr3" or new["environment"] != "fr3":
        raise ValueError("Cache reuse is only supported for FR3")
    keys = [
        "fr3_scene",
        "roadmap_neighbors",
        "max_task_space_distance",
        "max_configuration_distance",
        "offline_voxel_side",
        "lazy_edge_step",
    ]
    if any(cfg[k] != old["config"][k] for k in keys):
        raise ValueError("Scene or roadmap settings changed; refuse cache reuse")
    sources = [
        name
        for name in files
        if name.endswith(
            ("/model.py", "/data.py", "/build_assets.py", "/assets/fr3_33.json")
        )
    ]
    if any(files[name] != old["source_files"].get(name) for name in sources):
        raise ValueError("Model or input generation source changed; refuse cache reuse")
    if new["runtime_versions"] != old["runtime_versions"]:
        raise ValueError("Dependency versions changed; refuse cache reuse")
    inventory = {}
    for folder in ["scenes", "roadmaps"]:
        for source in sorted((previous / folder).rglob("*")):
            if not source.is_file():
                continue
            relative = source.relative_to(previous)
            target = root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            expected = hashlib.sha256(source.read_bytes()).hexdigest()
            if hashlib.sha256(target.read_bytes()).hexdigest() != expected:
                raise ValueError("Cache copy hash mismatch: " + str(relative))
            inventory[str(relative)] = dict(
                sha256=expected, bytes=source.stat().st_size
            )
    if not inventory:
        raise ValueError("Empty cache")
    atomic(
        root / "cache_provenance.json",
        dict(
            source_run=str(previous),
            source_implementation_hash=old["implementation_hash"],
            compared_config_keys=keys,
            compared_source_files=sources,
            files=inventory,
            scope="input scenes and offline maps only; no case outcomes reused",
        ),
    )


def require_identity(manifest):
    if (
        digest(identity()) != manifest["implementation_hash"]
        or runtime_versions() != manifest["runtime_versions"]
    ):
        raise ValueError(
            "Computational sources or numerical dependencies changed; refuse mixed run"
        )


def case_worker(root, case_id):
    from .model import FR3, ForestEnvironment
    from .data import planner, attach_alternate, online_pose_ik, endpoint_audit
    from .pipeline import Pipeline
    from .optimization import nto
    from .validation import validate, polygon_samples
    from reproduction.pipeline import save_candidate

    root = Path(root)
    m = read(root / "manifest.json")
    require_identity(m)
    cfg = m["config"]
    entry = next(c for c in m["cases"] if c["id"] == case_id)
    dest = root / "cases" / case_id
    dest.mkdir(parents=True, exist_ok=True)
    record = dict(
        **entry,
        status="running",
        implementation_hash=m["implementation_hash"],
        methods={},
    )
    tic = time.perf_counter()

    def checkpoint(stage):
        atomic(dest / "case.json", record)
        atomic(
            dest / "progress.json",
            dict(stage=stage, elapsed_seconds=time.perf_counter() - tic),
        )

    try:
        checkpoint("model_and_roadmap")
        if m["environment"] == "fr3":
            scene = read(
                root / "scenes" / f"e{entry['environment_seed']:02d}" / "scene.json"
            )
            if scene["status"] != "ready":
                record.update(status="scene_failed", reason=scene["status"])
                return
            f = FR3(cfg, scene["obstacles"])
            f.start = np.asarray(scene["start"], np.float32).astype(float)
            map_dir = (
                root
                / "roadmaps"
                / f"r{entry['roadmap_seed']:02d}_n{entry['roadmap_size']:05d}"
            )
            map_record = read(map_dir / "roadmap.json")
            if map_record.get("status") == "failed":
                record.update(status="offline_failed", reason=map_record)
                return
            p = planner(f)
            p.LoadRoadmap(str(map_dir / "native.map"))
            started = time.perf_counter()
            p.BuildCollisionSet(f.voxels)
            drm = p.drm
            collision_set_seconds = time.perf_counter() - started
            ik = online_pose_ik(f, drm, np.asarray(scene["target_pose"]), dest,
                                collision_set=p.collision_set)
            if ik["status"] != "success":
                record.update(status="ik_failed", ik=ik)
                return
            f.goal = np.asarray(ik["goal"], float)
            f.cfg.update(start=f.start, goal=f.goal)
            attach_alternate(f, map_dir / "native.map", dest)
            atomic(
                dest / "input.json",
                dict(
                    **entry,
                    scene=str(
                        root
                        / "scenes"
                        / f"e{entry['environment_seed']:02d}"
                        / "scene.json"
                    ),
                    start=f.start,
                    original_scene_start=scene["start"],
                    goal=f.goal,
                    obstacles=scene["obstacles"],
                    robot_spheres=33,
                    active_robot_spheres=len(f.geometry_ids),
                    static_pairs=f.minimal.num_collision_pairs,
                    obstacle_pairs=len(f.geometry_ids) * len(scene["obstacles"]),
                    target_pose=scene["target_pose"],
                    target_ik=ik,
                ),
            )
        else:
            f = ForestEnvironment(cfg, entry["environment_seed"])
            f.start = np.array(f.cfg["start"])
            f.goal = np.array(f.cfg["goal"])
            map_dir = root / "roadmaps" / f"r{entry['roadmap_seed']:02d}_n{entry['roadmap_size']:05d}"
            map_record = read(map_dir / "roadmap.json")
            if map_record.get("status") == "failed":
                record.update(status="offline_failed", reason=map_record)
                return
            p = planner(f)
            p.LoadRoadmap(str(map_dir / "native.map"))
            started = time.perf_counter()
            p.BuildCollisionSet(f.voxels)
            collision_set_seconds = time.perf_counter() - started
            drm = p.drm
            attach_alternate(f, map_dir / "native.map", dest)
            atomic(
                dest / "input.json",
                dict(**entry, centers=f.base.centers, start=f.start, goal=f.goal),
            )
        atomic(dest / "endpoints.json", dict(
            start=endpoint_audit(f, f.start), goal=endpoint_audit(f, f.goal)))
        started = time.perf_counter()
        try:
            success, path = p.Plan(f.start, f.goal, f.voxels, f.voxel_radius)
        except RuntimeError as error:
            if "No collision-free path exists in roadmap" not in str(error):
                raise
            record.update(status="drm_failed", reason=repr(error),
                roadmap=dict(**map_record, success=False,
                    online_seconds=time.perf_counter()-started,
                    collision_set_seconds=collision_set_seconds))
            return
        online = time.perf_counter() - started
        path = np.asarray(path, float)
        roadmap = dict(**map_record, success=bool(success), online_seconds=online,
                       collision_set_seconds=collision_set_seconds,
                       native_planner_options=f.native_planner_options)
        if m["environment"] == "fr3":
            roadmap["ik_seconds"] = ik["seconds"]
        if not success:
            record.update(status="drm_failed", roadmap=roadmap)
            return
        np.savez_compressed(dest / "seed_path.npz", path=path)
        dense = polygon_samples(path, cfg["validation_step"])
        seed_cuda = f.cuda(dense)
        seed_independent = f.independent(dense)
        exact = f.base.validate_polygon(path) if f.kind == "forest" else None
        valid = bool(
            seed_cuda.all()
            and seed_independent.all()
            and (exact is None or exact["collision_free"])
        )
        roadmap.update(
            path_length=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
            segments=len(path) - 1,
            independent_validation=dict(
                passed=valid,
                samples=len(dense),
                disagreements=int((seed_cuda != seed_independent).sum()),
                exact_circle_check=exact,
            ),
        )
        record.update(roadmap=roadmap, drm_validated_success=valid)
        checkpoint("eizo_warmup")
        if valid:
            warm = time.perf_counter()
            try:
                Pipeline(f, 1337).inflate(path[0], path[1])
            except Exception as error:
                record["eizo_warmup_failure"] = repr(error)
            record["eizo_warmup_seconds"] = time.perf_counter() - warm
        for method in cfg["method_order"]:
            checkpoint(method)
            out = dest / method
            out.mkdir(exist_ok=True)
            started = time.perf_counter()
            try:
                if not method.startswith("NTO") and not valid:
                    result = dict(
                        status="invalid_seed_path",
                        solver_success=False,
                        validation_passed=False,
                        collision_free=False,
                        reason="Native 0.1-step seed failed independent validation; no invalid edge is inflated",
                    )
                elif method.startswith("NTO"):
                    candidate = nto(path, f, method == "NTO_time")
                    elapsed = time.perf_counter() - started
                    save_candidate(out, candidate)
                    check_started = time.perf_counter()
                    v = validate(candidate, f)
                    samples = v.pop("_samples")
                    np.savez_compressed(out / "trajectory_samples.npz", **samples)
                    atomic(out / "validation.json", v)
                    result = dict(
                        status=(
                            "success_numerical"
                            if candidate["solver_success"] and v["passed"]
                            else (
                                "validation_failed"
                                if candidate["solver_success"]
                                else "optimization_failed"
                            )
                        ),
                        solver_success=candidate["solver_success"],
                        solver_status=candidate["solver_status"],
                        solver=candidate["solver"],
                        validation_passed=v["passed"],
                        collision_free=v["collision_free"],
                        native_cuda_collision_free=v["native_cuda_collision_free"],
                        independent_collision_free=v["independent_collision_free"],
                        cost=v["cost"],
                        minimum_clearance=v["minimum_clearance"],
                        optimization_seconds=elapsed,
                        construction_seconds=0.0,
                        total_seconds=elapsed,
                        validation_seconds=time.perf_counter() - check_started,
                        model_disagreements=v["model_disagreements"],
                    )
                else:
                    pipe = Pipeline(f, 1337)
                    answer = pipe.solve(method, path, drm, out)
                    candidate = answer.pop("candidate")
                    regions = answer.pop("regions")
                    v = answer.pop("validation", None)
                    if v:
                        samples = v.pop("_samples", None)
                        if samples:
                            np.savez_compressed(
                                out / "trajectory_samples.npz", **samples
                            )
                        atomic(out / "validation.json", v)
                    result = dict(
                        **answer,
                        solver_success=candidate["solver_success"],
                        solver_status=candidate["solver_status"],
                        solver=candidate.get("solver"),
                        validation_passed=bool(v and v["passed"]),
                        collision_free=v["collision_free"] if v else None,
                        native_cuda_collision_free=v["native_cuda_collision_free"] if v else None,
                        independent_collision_free=v["independent_collision_free"] if v else None,
                        cost=v["cost"] if v else None,
                        minimum_clearance=v["minimum_clearance"] if v else None,
                        regions=len(regions),
                        model_disagreements=v["model_disagreements"] if v else None,
                    )
            except Exception as error:
                traceback.print_exc()
                result = dict(
                    status=(
                        "blocked_license"
                        if getattr(error, "errno", None) == 10010
                        else "error"
                    ),
                    solver_success=False,
                    validation_passed=False,
                    collision_free=None,
                    cost=None,
                    total_seconds=time.perf_counter() - started,
                    reason=repr(error),
                    optimization_metadata=getattr(error, "optimization_metadata", None),
                    traceback=traceback.format_exc(),
                )
            result.update(
                method=method,
                case_id=case_id,
                elapsed_wall_seconds=time.perf_counter() - started,
            )
            atomic(out / "result.json", result)
            record["methods"][method] = result
            checkpoint(method + "_saved")
        record.update(status="completed", elapsed_seconds=time.perf_counter() - tic)
    except Exception as error:
        traceback.print_exc()
        record.update(
            status="error", reason=repr(error), traceback=traceback.format_exc()
        )
    finally:
        atomic(dest / "case.json", record)


def execute(root):
    root = Path(root).resolve()
    m = read(root / "manifest.json")
    require_identity(m)
    m["status"] = "running"
    atomic(root / "manifest.json", m)
    print("RUN_DIR=" + str(root), flush=True)
    cfg = m["config"]
    started = time.perf_counter()
    if m["environment"] in ["fr3", "forest"]:
        jobs = [
            ("_scene", f"e{e:02d}", dict(environment_seed=e))
            for e in sorted({c["environment_seed"] for c in m["cases"]})
        ] if m["environment"] == "fr3" else []
        jobs += [
            ("_map", f"r{r:02d}_n{n:05d}", dict(roadmap_seed=r, roadmap_size=n))
            for r, n in sorted(
                {(c["roadmap_seed"], c["roadmap_size"]) for c in m["cases"]}
            )
        ]
        for command, label, entry in jobs:
            dest = root / ("scenes" if command == "_scene" else "roadmaps") / label
            target = dest / ("scene.json" if command == "_scene" else "roadmap.json")
            if target.exists():
                continue
            dest.mkdir(parents=True, exist_ok=True)
            cmd = [
                sys.executable,
                "-m",
                "reproduction.paper_constructed.run",
                command,
                "--run-dir",
                str(root),
                "--case-id",
                label,
            ]
            result = execute_child(cmd, dest, cfg["case_timeout_seconds"])
            if not target.exists():
                atomic(target, dict(status="failed", **entry, **result))
            print(
                "PREPARE",
                label,
                read(target).get("status", "ready"),
                f"{time.perf_counter()-started:.1f}s",
                flush=True,
            )
    for index, case in enumerate(m["cases"], 1):
        dest = root / "cases" / case["id"]
        path = dest / "case.json"
        if path.exists() and read(path).get("status") in [
            "completed",
            "drm_failed",
            "scene_failed",
            "offline_failed",
            "ik_failed",
            "error",
            "timeout",
            "process_error",
        ]:
            continue
        outcome = execute_child(
            [
                sys.executable,
                "-m",
                "reproduction.paper_constructed.run",
                "_case",
                "--run-dir",
                str(root),
                "--case-id",
                case["id"],
            ],
            dest,
            cfg["case_timeout_seconds"],
        )
        if outcome["timed_out"] or outcome["exit_code"] != 0:
            record = read(path) if path.exists() else dict(**case, methods={})
            record.update(
                status="timeout" if outcome["timed_out"] else "process_error", **outcome
            )
            if (dest / "progress.json").exists():
                record["failure_stage"] = read(dest / "progress.json")["stage"]
            atomic(path, record)
        record = read(path)
        print(
            f"{index}/{len(m['cases'])}",
            case["id"],
            record["status"],
            {k: v["status"] for k, v in record.get("methods", {}).items()},
            f"{time.perf_counter()-started:.1f}s",
            flush=True,
        )
        atomic(
            root / "progress.json",
            dict(
                completed_cases=index,
                planned_cases=len(m["cases"]),
                elapsed_seconds=time.perf_counter() - started,
                last_case=case["id"],
            ),
        )
    counts = Counter(
        read(root / "cases" / case["id"] / "case.json")["status"] for case in m["cases"]
    )
    m.update(status="completed", case_status_counts=dict(counts))
    atomic(root / "manifest.json", m)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    with (root / f"gpu_after_{stamp}.txt").open("w") as stream:
        subprocess.run(["nvidia-smi", "-q"], stdout=stream, check=True)
    return 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("command", choices=["run", "resume", "_case", "_map", "_scene"])
    p.add_argument("--environment", choices=["forest", "fr3"], default="fr3")
    p.add_argument("--run-dir")
    p.add_argument("--case-id")
    p.add_argument("--environments", type=int, nargs="+")
    p.add_argument("--roadmap-seeds", type=int, nargs="+")
    p.add_argument("--sizes", type=int, nargs="+")
    p.add_argument("--output-root")
    p.add_argument("--cache-from", help="Verified FR3 input scene/roadmap cache")
    a = p.parse_args()
    if a.command == "_case":
        case_worker(a.run_dir, a.case_id)
        return 0
    if a.command in ["_map", "_scene"]:
        from .data import make_scene, build_roadmap

        root = Path(a.run_dir)
        m = read(root / "manifest.json")
        require_identity(m)
        if a.command == "_scene":
            make_scene(m["config"], int(a.case_id[1:]), root / "scenes" / a.case_id)
        else:
            if m["environment"] == "forest":
                from .forest import build_roadmap
            r, n = a.case_id.split("_n")
            build_roadmap(
                m["config"], int(r[1:]), int(n), root / "roadmaps" / a.case_id
            )
        return 0
    return execute(new_run(a) if a.command == "run" else a.run_dir)


if __name__ == "__main__":
    raise SystemExit(main())
