"""Freeze reconstructed scenes, then exercise real native DRM and pose-map IK.

The prepare phase cannot see planning outcomes. Every declared scene/case stays
in the denominator. Each child is bounded; resume verifies frozen inputs and
sources and keeps completed failures instead of silently trying new tasks.
"""
import argparse
from collections import Counter
import datetime
import hashlib
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
import traceback

import numpy as np

from reproduction.common import ROOT, atomic, read, digest

PACKAGE = Path(__file__).parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sources():
    names = [
        "local/python.sh", "reproduction/common.py", "reproduction/run.py",
        "reproduction/forest.py", "reproduction/configs/forest.json",
        "reproduction/environment_reconstruction/run.py",
        "reproduction/environment_reconstruction/model.py",
        "reproduction/environment_reconstruction/profile.json",
        "reproduction/paper_constructed/config.json",
        "reproduction/paper_constructed/model.py",
        "reproduction/paper_constructed/data.py",
        "reproduction/paper_constructed/forest.py",
        "reproduction/paper_constructed/validation.py",
        "reproduction/paper_constructed/run.py",
        "reproduction/paper_constructed/assets/fr3_33.json",
        "csdecomp/tests/test_assets/table.obj",
        "csdecomp/tests/test_assets/table.urdf",
    ]
    return {name: sha(ROOT / name) for name in names}


def receipt(folder):
    return {str(p.relative_to(folder)): sha(p) for p in sorted(Path(folder).rglob("*"))
            if p.is_file()}


def job_receipt(folder):
    return {k: v for k, v in receipt(folder).items() if k != "execution.json"}


def verify_job(folder):
    record = read(folder / "execution.json")
    if record.get("output_hashes") != job_receipt(folder):
        raise ValueError("Completed job outputs changed: " + str(folder))
    return record


def preserve_partial(folder):
    """Seal an interrupted attempt without replacing any existing artifact."""
    record = dict(status="interrupted_preserved", exit_code=None, timed_out=False,
                  reason="Existing artifacts without a completion receipt; not automatically retried",
                  output_hashes=job_receipt(folder))
    atomic(folder / "execution.json", record)
    return record


def case_record(root, case):
    folder = root / "cases" / case["id"]
    execution = read(folder / "execution.json")
    if execution.get("status") == "interrupted_preserved":
        return dict(**case, status="interrupted_preserved")
    return read(folder / "case.json")


def check_run(root):
    manifest = read(root / "manifest.json")
    if manifest["source_hashes"] != sources():
        raise ValueError("Computational sources changed; use a separate prepared run")
    if sha(root / "protocol.json") != manifest["protocol_sha256"]:
        raise ValueError("Frozen protocol changed")
    if receipt(root / "scenes") != manifest["scene_hashes"]:
        raise ValueError("Frozen input scenes changed")
    from reproduction.run import runtime_versions
    if runtime_versions() != manifest["runtime_versions"]:
        raise ValueError("Numerical environment changed")
    return manifest


def prepare(root):
    from reproduction.run import runtime_versions
    from reproduction.paper_constructed.data import make_scene, endpoint_audit
    from reproduction.paper_constructed.model import FR3
    from .model import FixedForest

    root.mkdir(parents=True, exist_ok=False)
    profile = read(PACKAGE / "profile.json")
    cfg = read(ROOT / "reproduction/paper_constructed/config.json")
    cfg["protocol"] = profile["name"]
    cfg["fidelity"] = "figure_evidence_plus_explicit_public_asset_hypotheses"
    cfg["fr3_scene"].update(profile["fr3_scene_overrides"])
    cfg["assumptions"] = profile["evidence"]
    reference = read(ROOT / profile["forest_reference"])
    cfg.update(start=reference["start"], goal=reference["goal"])
    atomic(root / "protocol.json", cfg)
    atomic(root / "profile.json", profile)
    source = sources()
    for name in source:
        target = root / "source_snapshot" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, target)
    manifest = dict(status="preparing", created_utc=datetime.datetime.now(
        datetime.timezone.utc).isoformat(), scope=profile["scope"],
        git_head=subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        source_hashes=source, runtime_versions=runtime_versions(),
        system=dict(platform=platform.platform(), python=sys.version,
                    executable=sys.executable, conda_prefix=sys.prefix),
        protocol_sha256=sha(root / "protocol.json"), profile=profile,
        no_planning_before_scene_freeze=True)
    atomic(root / "manifest.json", manifest)
    for command, name in [(["nvidia-smi", "-q"], "gpu.txt"),
                          (["uname", "-a"], "kernel.txt"),
                          ([sys.executable, "-m", "pip", "freeze"], "pip-freeze.txt")]:
        with (root / name).open("w") as stream:
            subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, check=True)
    atomic(root / "scenes/forest/figure6.json", reference)
    count = profile["collision_audit_points_per_scene"]
    audits = []
    for seed in profile["environment_seeds"]:
        # Reuse v5 obstacle realizations; only the endpoint condition changes.
        centers = np.random.default_rng(seed).uniform(-3.5, 3.5, (15, 2))
        scene = {k: v for k, v in reference.items() if k not in (
            "reference_path", "reference_path_semantics", "source_member", "source_url")}
        scene.update(scene_id=f"forest_constructed_{seed:02d}", centers=centers,
                     obstacle_centers=centers, environment_seed=seed,
                     evidence_class="constructed_random_scene_with_figure6_endpoints")
        atomic(root / f"scenes/forest/e{seed:02d}.json", scene)
    for scene_path in sorted((root / "scenes/forest").glob("*.json")):
        scene = read(scene_path)
        env = FixedForest(cfg, scene)
        rng = np.random.default_rng(250410783)
        points = rng.uniform(env.lower, env.upper, (count, 2))
        native, exact = env.cuda(points), env.independent(points)
        audit = dict(kind="forest", id=scene["scene_id"],
                     samples=count, disagreements=int(np.sum(native != exact)),
                     obstacle_count=len(env.base.centers),
                     start=endpoint_audit(env, env.start), goal=endpoint_audit(env, env.goal),
                     straight_line=env.base.validate_polygon(np.array([env.start, env.goal])))
        audits.append(audit)
        np.savez_compressed(root / "scenes/forest" / (scene_path.stem + "_collision_audit.npz"),
                            q=points, native=native, independent=exact)
    for seed in profile["environment_seeds"]:
        out = root / f"scenes/fr3/e{seed:02d}"
        try:
            scene = make_scene(cfg, seed, out)
            env = FR3(cfg, scene["obstacles"])
            rng = np.random.default_rng(np.random.SeedSequence([250410783, 15, seed, 3]))
            points = rng.uniform(env.lower, env.upper, (count, 7)).astype(np.float32)
            native, independent = env.cuda(points), env.independent(points)
            np.savez_compressed(out / "collision_audit.npz", q=points,
                                native=native, independent=independent)
            s = cfg["fr3_scene"]
            obstacles = np.array(scene["obstacles"])
            audit = dict(kind="fr3", id=f"e{seed:02d}", scene_status=scene["status"],
                samples=count, disagreements=int(np.sum(native != independent)),
                native_free=int(native.sum()), independent_free=int(independent.sum()),
                robot_spheres=len(env.geometry_ids),
                static_pairs=env.minimal.num_collision_pairs,
                online_pairs=len(env.geometry_ids)*len(obstacles),
                drake_total_pairs=len(env.inspector.GetCollisionCandidates()),
                obstacle_count=len(obstacles),
                obstacle_centers_in_box=bool(np.all(obstacles >= s["obstacle_lower"]) and
                                             np.all(obstacles <= s["obstacle_upper"])),
                table_intersecting_spheres=int(np.sum(obstacles[:, 2] < s["obstacle_radius"])),
                minimum_sphere_bottom=float(obstacles[:, 2].min()-s["obstacle_radius"]))
            if scene["status"] == "ready":
                start_pose = env.pose(scene["start"])
                target = np.asarray(scene["target_pose"])
                audit.update(start=endpoint_audit(env, scene["start"]),
                    target_witness=endpoint_audit(env, scene["target_generating_configuration"]),
                    start_tcp=start_pose[:3,3], target_tcp=target[:3,3],
                    start_in_right_box=bool(np.all(start_pose[:3,3]>=s["right_ee_lower"]) and
                                            np.all(start_pose[:3,3]<=s["right_ee_upper"])),
                    target_in_left_box=bool(np.all(target[:3,3]>=s["left_ee_lower"]) and
                                            np.all(target[:3,3]<=s["left_ee_upper"])),
                    target_fk_residual=float(np.linalg.norm(target-env.pose(
                        scene["target_generating_configuration"]))))
            atomic(out / "audit.json", audit)
            audits.append(audit)
        except Exception as exc:
            atomic(out / "preparation_failure.json", dict(status="failed", exception=repr(exc),
                                                         traceback=traceback.format_exc()))
            audits.append(dict(kind="fr3", id=f"e{seed:02d}", scene_status="failed", error=repr(exc)))
    atomic(root / "environment_audits.json", audits)
    cases = [dict(id=f"forest_n{n:05d}", kind="forest", scene="scenes/forest/figure6.json",
                  roadmap=f"forest_n{n:05d}", size=n) for n in profile["forest_figure_roadmap_sizes"]]
    cases += [dict(id=f"forest_e{seed:02d}", kind="forest", scene=f"scenes/forest/e{seed:02d}.json",
                   roadmap=f"forest_n{profile['forest_random_roadmap_size']:05d}",
                   size=profile["forest_random_roadmap_size"]) for seed in profile["environment_seeds"]]
    cases += [dict(id=f"fr3_e{seed:02d}", kind="fr3", scene=f"scenes/fr3/e{seed:02d}/scene.json",
                   roadmap=f"fr3_n{profile['fr3_roadmap_size']:05d}", size=profile["fr3_roadmap_size"])
              for seed in profile["environment_seeds"]]
    manifest.update(status="prepared", scene_hashes=receipt(root / "scenes"), cases=cases,
                    scenes_frozen_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    scene_preparation_statuses=dict(Counter(a.get("scene_status", "ready") for a in audits)))
    atomic(root / "manifest.json", manifest)
    print(dict(run=str(root), status="prepared", scenes=len(audits), cases=len(cases)), flush=True)


def build_map(root, case):
    from reproduction.paper_constructed.data import build_roadmap as fr3_map
    from reproduction.paper_constructed.forest import build_roadmap as forest_map
    m = check_run(root)
    out = root / "roadmaps" / case["roadmap"]
    if (out / "native.map").exists() or (out / "roadmap.json").exists():
        raise ValueError("Refuse to overwrite a prior roadmap; use a new run")
    (fr3_map if case["kind"] == "fr3" else forest_map)(
        read(root / "protocol.json"), m["profile"]["roadmap_seed"], case["size"], out)


def run_case(root, case):
    from reproduction.paper_constructed.data import planner, online_pose_ik, endpoint_audit
    from reproduction.paper_constructed.model import FR3
    from reproduction.paper_constructed.validation import polygon_samples
    from .model import FixedForest
    check_run(root)
    out = root / "cases" / case["id"]
    if (out / "case.json").exists() or (out / "online_pose_ik.json").exists():
        raise ValueError("Refuse to overwrite prior case evidence; use a new run")
    verify_job(root / "roadmaps" / case["roadmap"])
    cfg = read(root / "protocol.json")
    result = dict(**case, status="running", started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    atomic(out / "case.json", result)
    tic = time.perf_counter()
    try:
        scene = read(root / case["scene"])
        if case["kind"] == "fr3":
            if scene["status"] != "ready":
                result.update(status="scene_failed", reason=scene["status"])
                return
            env = FR3(cfg, scene["obstacles"])
            env.start = np.asarray(scene["start"])
        else:
            env = FixedForest(cfg, scene)
        p = planner(env)
        p.LoadRoadmap(str(root / "roadmaps" / case["roadmap"] / "native.map"))
        p.BuildCollisionSet(env.voxels)
        if case["kind"] == "fr3":
            ik = online_pose_ik(env, p.drm, np.asarray(scene["target_pose"]), out, p.collision_set)
            result["ik_status"] = ik["status"]
            if ik["status"] != "success":
                result["status"] = "ik_failed"
                return
            env.goal = np.asarray(ik["goal"])
            result["target_pose"] = scene["target_pose"]
        result.update(start=env.start, goal=env.goal,
                      endpoint_audits=dict(start=endpoint_audit(env, env.start),
                                           goal=endpoint_audit(env, env.goal)))
        if not all(a["canonical_valid"] for a in result["endpoint_audits"].values()):
            result["status"] = "endpoint_invalid"
            return
        start, goal = (np.asarray(q, np.float32).astype(float) for q in [env.start, env.goal])
        started = time.perf_counter()
        ok, raw_path = p.Plan(start, goal, env.voxels, env.voxel_radius)
        result.update(native_success=bool(ok), online_plan_seconds=time.perf_counter()-started,
                      native_planner_options=env.native_planner_options)
        path = np.asarray(raw_path, float)
        result["path"] = path
        if not ok or len(path) < 2:
            result["status"] = "drm_failed"
            return
        samples = polygon_samples(path, cfg["validation_step"])
        native, independent = env.cuda(samples), env.independent(samples)
        exact = env.base.validate_polygon(path) if case["kind"] == "forest" else None
        endpoint_error = float(max(np.linalg.norm(path[0]-start), np.linalg.norm(path[-1]-goal)))
        bounds = bool(np.all(samples >= env.lower-1e-7) and np.all(samples <= env.upper+1e-7))
        passed = bool(native.all() and independent.all() and bounds and endpoint_error < 1e-5 and
                      (exact is None or exact["collision_free"]))
        np.savez_compressed(out / "validation_samples.npz", q=samples, native=native, independent=independent)
        result.update(status="validated" if passed else "validation_failed", segments=len(path)-1,
            joint_path_length=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
            validation=dict(passed=passed, samples=len(samples), native_valid=bool(native.all()),
                independent_valid=bool(independent.all()), disagreements=int(np.sum(native != independent)),
                limits_valid=bounds, endpoint_error=endpoint_error, exact_forest=exact,
                scope="analytic continuous polygon/circle for Forest; 0.005-rad sampled sphere/box for FR3, not a continuous mesh certificate"))
    except Exception as exc:
        result.update(status="exception", exception=repr(exc), traceback=traceback.format_exc())
    finally:
        result["elapsed_seconds"] = time.perf_counter()-tic
        atomic(out / "case.json", result)


def smoke(root):
    from reproduction.paper_constructed.run import execute_child
    m = check_run(root)
    timeout = m["profile"]["child_timeout_seconds"]
    maps = {}
    for case in m["cases"]:
        maps[case["roadmap"]] = case
    for name, case in maps.items():
        dest = root / "roadmaps" / name
        if (dest / "execution.json").exists():
            verify_job(dest)
            continue
        if dest.exists() and any(dest.iterdir()):
            preserve_partial(dest)
            print(name, "interrupted_preserved", flush=True)
            continue
        cmd = [sys.executable, "-m", "reproduction.environment_reconstruction.run", "map",
               "--run", str(root), "--case", case["id"]]
        r = execute_child(cmd, dest, timeout)
        atomic(dest / "execution.json", dict(**r, command=cmd, output_hashes=job_receipt(dest)))
        print(name, r, flush=True)
    for case in m["cases"]:
        dest = root / "cases" / case["id"]
        if (dest / "execution.json").exists():
            verify_job(dest)
            continue
        if dest.exists() and any(dest.iterdir()):
            preserve_partial(dest)
            print(case["id"], "interrupted_preserved", flush=True)
            continue
        map_exec = read(root / "roadmaps" / case["roadmap"] / "execution.json")
        if map_exec["exit_code"] != 0 or map_exec["timed_out"]:
            atomic(dest / "case.json", dict(**case, status="roadmap_failed"))
            atomic(dest / "execution.json", dict(skipped="roadmap_failed", output_hashes=job_receipt(dest)))
            continue
        cmd = [sys.executable, "-m", "reproduction.environment_reconstruction.run", "case",
               "--run", str(root), "--case", case["id"]]
        r = execute_child(cmd, dest, timeout)
        if r["timed_out"] or r["exit_code"] != 0:
            partial = read(dest / "case.json") if (dest / "case.json").exists() else case
            atomic(dest / "case.json", dict(**{k:v for k,v in partial.items() if k != "status"},
                status="timeout" if r["timed_out"] else "worker_failed", worker=r))
        atomic(dest / "execution.json", dict(**r, command=cmd, output_hashes=job_receipt(dest)))
        print(case["id"], read(dest / "case.json")["status"], flush=True)
    records = [case_record(root, c) for c in m["cases"]]
    summary = dict(requested=len(m["cases"]), completed=len(records),
        by_environment={k:dict(Counter(r["status"] for r in records if r["kind"]==k)) for k in ["forest","fr3"]},
        cases=[{k:v for k,v in r.items() if k in ["id","status","segments","joint_path_length","elapsed_seconds","ik_status"]} for r in records],
        input_hashes_unchanged=receipt(root / "scenes") == m["scene_hashes"],
        scope=m["scope"])
    atomic(root / "summary.json", summary)
    print(summary["by_environment"], flush=True)


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("action", choices=["prepare","smoke","map","case","verify"])
    p.add_argument("--run", type=Path, required=True)
    p.add_argument("--case")
    args = p.parse_args()
    root = args.run.resolve()
    if args.action == "prepare":
        prepare(root)
    elif args.action == "smoke":
        smoke(root)
    elif args.action == "verify":
        m = check_run(root)
        jobs = list(root.glob("roadmaps/*/execution.json")) + list(root.glob("cases/*/execution.json"))
        for job in jobs:
            verify_job(job.parent)
        print(dict(verified=True, source_files=len(m["source_hashes"]),
                   input_files=len(m["scene_hashes"]), planned_cases=len(m["cases"]),
                   verified_jobs=len(jobs)))
    else:
        m = check_run(root)
        case = next(c for c in m["cases"] if c["id"] == args.case)
        (build_map if args.action == "map" else run_case)(root, case)


if __name__ == "__main__":
    main()
