"""Audit saved experiments, independently recheck fixed samples, test resume integrity."""

import argparse
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from reproduction.common import ROOT, atomic, digest, read
from reproduction.paper_completion.run import require_identity
from reproduction.paper_completion.model import FR3, ForestEnvironment
from reproduction.paper_completion.analyze import source_integrity_issues


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def case_inventory(run):
    return {
        str(p.relative_to(run)): dict(bytes=p.stat().st_size, sha256=sha(p))
        for p in sorted((run / "cases").rglob("*"))
        if p.is_file()
    }


def clearance_capped(env, q):
    if env.kind == "forest":
        return float(env.base.clearances([q])[0])
    env.drake.SetPositions(env.pc, q)
    query = env.sg.get_query_output_port().Eval(
        env.sg.GetMyContextFromRoot(env.context)
    )
    return min(
        (p.distance for p in query.ComputeSignedDistancePairwiseClosestPoints(0.02)),
        default=0.02,
    )


def check_online_inputs(case_id, scene, inputs, ik, issues, env=None):
    """Validate the saved fixed-pose task and raw→canonical IK handoff."""
    original_start = np.asarray(scene.get("original_start", scene["start"]), float)
    canonical_start = np.asarray(scene["start"], float)
    raw_goal = None
    if not np.array_equal(canonical_start, original_start.astype(np.float32).astype(float)):
        issues.append(case_id + ": scene start is not the float32 original start")
    if ik is not None:
        if not np.array_equal(ik.get("target_pose"), scene["target_pose"]):
            issues.append(case_id + ": online IK changed the frozen target pose")
        accepted = [a for a in ik.get("attempts", []) if a.get("accepted")]
        if ik.get("status") == "success":
            if len(accepted) != 1:
                issues.append(case_id + ": successful IK must have one accepted candidate")
            else:
                attempt = accepted[0]
                audit = attempt.get("endpoint_audit", {})
                raw_goal = np.asarray(audit.get("original", []), float)
                canonical_goal = np.asarray(audit.get("canonical", []), float)
                if raw_goal.shape != canonical_start.shape or canonical_goal.shape != canonical_start.shape:
                    issues.append(case_id + ": missing/malformed accepted IK endpoints")
                    raw_goal = None
                    return original_start, raw_goal
                if not np.array_equal(canonical_goal, raw_goal.astype(np.float32).astype(float)):
                    issues.append(case_id + ": IK canonical goal is not a float32 round trip")
                if not np.array_equal(ik.get("goal"), canonical_goal):
                    issues.append(case_id + ": IK selected goal differs from accepted candidate")
                if ik.get("selected_node_id") != attempt.get("node_id"):
                    issues.append(case_id + ": IK selected roadmap node mismatch")
                if not attempt.get("solver_success") or not audit.get("canonical_valid"):
                    issues.append(case_id + ": accepted IK lacks solver/endpoint validity")
    if inputs is not None:
        for field in ["obstacles", "target_pose"]:
            if not np.array_equal(inputs[field], scene[field]):
                issues.append(case_id + ": input does not match frozen scene: " + field)
        if not np.array_equal(inputs["start"], canonical_start):
            issues.append(case_id + ": input start differs from canonical scene start")
        if ik is None or ik.get("status") != "success":
            issues.append(case_id + ": planning input lacks successful online IK evidence")
        elif not np.array_equal(inputs["goal"], ik["goal"]):
            issues.append(case_id + ": planning input goal differs from selected IK goal")
        if "target_ik" in inputs and inputs["target_ik"] != ik:
            issues.append(case_id + ": copied IK record differs from online IK artifact")
        if env is not None:
            pose = env.pose(np.asarray(inputs["goal"], float))
            target = np.asarray(scene["target_pose"], float)
            if (np.linalg.norm(pose[:3, 3] - target[:3, 3]) >= 2e-5 or
                    np.linalg.norm(pose[:3, :3] - target[:3, :3]) >= 2e-4):
                issues.append(case_id + ": selected goal misses the frozen target pose")
    return original_start, raw_goal


def audit(run, out, resume=False):
    run, out = Path(run).resolve(), Path(out)
    out.mkdir(parents=True, exist_ok=True)
    m = read(run / "manifest.json")
    require_identity(m)
    if m["status"] != "completed":
        raise ValueError("Expected a completed primary run")
    issues, failures, lgcs, case_failures, boundary_rows, seed_rows = (
        [],
        [],
        [],
        [],
        [],
        [],
    )
    issues.extend(source_integrity_issues(run, m))
    sample_points, trajectories, maximum_step, disagreements = 0, 0, 0.0, 0
    exact_checks, exact_valid = 0, 0
    current_seed, env = None, None
    for entry in m["cases"]:
        directory = run / "cases" / entry["id"]
        record = read(directory / "case.json")
        inputs = None
        raw_start = raw_goal = None
        if record["status"] != "completed":
            from reproduction.paper_completion.analyze import semantic_case_status

            case_failures.append(
                dict(
                    case_id=entry["id"],
                    raw_status=record["status"],
                    semantic_status=semantic_case_status(record),
                    reason=record.get("reason"),
                )
            )
        if record.get("implementation_hash") != m["implementation_hash"]:
            issues.append(entry["id"] + ": computational identity")
        if current_seed != entry["environment_seed"]:
            current_seed = entry["environment_seed"]
            if m["environment"] == "fr3":
                scene = read(run / "scenes" / f"e{current_seed:02d}" / "scene.json")
                env = FR3(m["config"], scene["obstacles"])
            else:
                env = ForestEnvironment(m["config"], current_seed)
        if m["environment"] == "fr3":
            inputs = read(directory / "input.json") if (directory / "input.json").exists() else None
            ik = read(directory / "online_pose_ik.json") if (directory / "online_pose_ik.json").exists() else None
            if scene.get("status") == "ready":
                raw_start, raw_goal = check_online_inputs(entry["id"], scene, inputs, ik, issues, env)
        if (
            record.get("roadmap", {}).get("success")
            and (directory / "seed_path.npz").exists()
        ):
            with np.load(directory / "seed_path.npz") as arrays:
                seed_path = arrays["path"]
            if env.kind == "fr3" and (inputs is None or raw_goal is None):
                issues.append(entry["id"] + ": successful DRM lacks original IK endpoint evidence")
                continue
            original_start = np.array(raw_start if env.kind == "fr3" else env.cfg["start"])
            original_goal = np.array(raw_goal if env.kind == "fr3" else env.cfg["goal"])
            input_start = np.array(inputs["start"] if env.kind == "fr3" else env.cfg["start"])
            input_goal = np.array(inputs["goal"] if env.kind == "fr3" else env.cfg["goal"])
            if not np.array_equal(seed_path[0], input_start.astype(np.float32).astype(float)):
                issues.append(entry["id"] + ": native path start differs from planning input")
            if not np.array_equal(seed_path[-1], input_goal.astype(np.float32).astype(float)):
                issues.append(entry["id"] + ": native path goal differs from planning input")
            a, b, c, d = [
                clearance_capped(env, q)
                for q in [original_start, original_goal, seed_path[0], seed_path[-1]]
            ]
            seed_rows.append(
                dict(
                    case_id=entry["id"],
                    strict_seed_valid=record.get("drm_validated_success"),
                    original_goal_source="accepted_online_IK_double" if env.kind == "fr3" else "configured_goal",
                    original_start_clearance_capped_0p02=a,
                    original_goal_clearance_capped_0p02=b,
                    native_path_start_clearance_capped_0p02=c,
                    native_path_goal_clearance_capped_0p02=d,
                    start_representation_error=float(
                        np.linalg.norm(seed_path[0] - original_start)
                    ),
                    goal_representation_error=float(
                        np.linalg.norm(seed_path[-1] - original_goal)
                    ),
                    goal_equals_float32_original=bool(
                        np.array_equal(
                            seed_path[-1],
                            original_goal.astype(np.float32).astype(float),
                        )
                    ),
                    failure_mechanism=(
                        (
                            "original_endpoint_collision"
                            if min(a, b) < 0
                            else (
                                "endpoint_representation_collision"
                                if min(c, d) < 0
                                else "interior_collision_or_native_boundary_disagreement"
                            )
                        )
                        if not record.get("drm_validated_success")
                        else "valid_seed"
                    ),
                )
            )
        for method, r in record.get("methods", {}).items():
            dest = directory / method
            if r != read(dest / "result.json"):
                issues.append(entry["id"] + ": result checkpoint: " + method)
            vp = dest / "validation.json"
            v = read(vp) if vp.exists() else None
            if r.get("solver_success") and (v is None or not (dest / "trajectory_samples.npz").exists()):
                issues.append(entry["id"] + ": solver-success result lacks validation/replay: " + method)
            if r["status"] != "success_numerical":
                failures.append(
                    dict(
                        case_id=entry["id"],
                        method=method,
                        status=r["status"],
                        solver_success=r.get("solver_success"),
                        collision_free=v.get("collision_free") if v else None,
                        endpoint_error=v.get("endpoint_error") if v else None,
                        derivative_pass=v.get("derivative_pass") if v else None,
                        reason=r.get("reason"),
                    )
                )
            if method == "LGCS":
                paths = sorted(dest.glob("optimization_*.json"))
                if paths:
                    o = read(paths[-1])
                    lgcs.append(
                        dict(
                            case_id=entry["id"],
                            status=r["status"],
                            solver_status=o.get("solver_status"),
                            gap=o.get("gap"),
                            variables=o.get("solver_variables"),
                            graph_edges=o.get("graph_edges"),
                        )
                    )
            replay = dest / "trajectory_samples.npz"
            if replay.exists() and v is None:
                issues.append(entry["id"] + ": replay exists without validation metadata: " + method)
                continue
            if replay.exists():
                with np.load(replay) as arrays:
                    q = arrays["q"]
                    actual = arrays["independent"]
                    native = arrays["cuda"]
                    times = arrays["time"]
                trajectories += 1
                if len(q) != v["samples"] or not (len(q) == len(actual) == len(native)):
                    issues.append(entry["id"] + ": replay sample counts: " + method)
                if (
                    int((~actual).sum()) != v["independent_collisions"]
                    or int((~native).sum()) != v["cuda_collisions"]
                ):
                    issues.append(
                        entry["id"] + ": saved collision flag counts: " + method
                    )
                if int((actual != native).sum()) != v["model_disagreements"]:
                    issues.append(
                        entry["id"] + ": saved disagreement counts: " + method
                    )
                if v.get("native_cuda_collision_free") is not bool(native.all()):
                    issues.append(entry["id"] + ": native CUDA free flag differs from replay: " + method)
                for field in ["native_cuda_collision_free", "independent_collision_free", "collision_free"]:
                    if r.get(field) != v.get(field):
                        issues.append(entry["id"] + ": result/validation flag mismatch: " + method + ": " + field)
                disagreements += int((actual != native).sum())
                different = np.flatnonzero(actual != native)
                if len(different):
                    distances = [clearance_capped(env, point) for point in q[different]]
                    boundary_rows.append(
                        dict(
                            case_id=entry["id"],
                            method=method,
                            points=len(different),
                            native_free_independent_collision=int(
                                (native[different] & ~actual[different]).sum()
                            ),
                            native_collision_independent_free=int(
                                (~native[different] & actual[different]).sum()
                            ),
                            minimum_independent_clearance=min(distances),
                            maximum_independent_clearance=max(distances),
                            maximum_absolute_independent_clearance=max(
                                abs(v) for v in distances
                            ),
                        )
                    )
                indices = np.unique(
                    np.linspace(0, len(q) - 1, min(17, len(q))).astype(int)
                )
                checked = env.independent(q[indices])
                sample_points += len(indices)
                if not np.array_equal(checked, actual[indices]):
                    issues.append(
                        entry["id"] + ": independent replay mismatch: " + method
                    )
                maximum_step = max(maximum_step, v["maximum_sample_joint_step"])
                if r["status"] == "success_numerical":
                    if not v["passed"] or not actual.all() or not native.all():
                        issues.append(
                            entry["id"] + ": inconsistent validated success: " + method
                        )
                    if (
                        v["maximum_sample_joint_step"]
                        > m["config"]["validation_step"] + 1e-6
                    ):
                        issues.append(
                            entry["id"] + ": excessive validation step: " + method
                        )
            if m["environment"] == "forest" and method == "NTO_distance":
                candidates = sorted(dest.glob("candidate_*.npz"))
                if candidates:
                    with np.load(candidates[-1]) as arrays:
                        check = env.base.validate_polygon(arrays["controls"])
                    exact_checks += 1
                    exact_valid += int(check["collision_free"])
                    if (
                        r["status"] == "success_numerical"
                        and not check["collision_free"]
                    ):
                        issues.append(entry["id"] + ": exact NTO segment collision")
    cache = run / "cache_provenance.json"
    if cache.exists():
        provenance = read(cache)
        for name, expected in provenance["files"].items():
            if sha(run / name) != expected["sha256"]:
                issues.append("reused input changed: " + name)
    before = case_inventory(run)
    (out / "case_inventory.json.gz").write_bytes(
        gzip.compress((json.dumps(before, indent=2) + "\n").encode(), mtime=0)
    )
    if resume:
        with (out / "resume.stdout.log").open("w") as stream:
            result = subprocess.run(
                [
                    "bash",
                    "local/python.sh",
                    "-m",
                    "reproduction.paper_completion.run",
                    "resume",
                    "--run-dir",
                    str(run),
                ],
                cwd=ROOT,
                stdout=stream,
                stderr=subprocess.STDOUT,
            )
        after = case_inventory(run)
        changed = [k for k in set(before) | set(after) if before.get(k) != after.get(k)]
        atomic(
            out / "resume_check.json",
            dict(
                exit_code=result.returncode,
                case_files=len(before),
                changed_case_files=changed,
            ),
        )
        if result.returncode or changed:
            issues.append("resume failed or altered terminal case files")
    from reproduction.paper_completion.analyze import write_csv

    write_csv(out / "failures.csv", failures)
    write_csv(
        out / "case_failures.csv",
        case_failures,
        fields=["case_id", "raw_status", "semantic_status", "reason"],
    )
    write_csv(out / "boundary_disagreements.csv", boundary_rows)
    write_csv(out / "seed_endpoint_audit.csv", seed_rows)
    write_csv(out / "lgcs_solver_states.csv", lgcs)
    result = dict(
        environment=m["environment"],
        run_dir=str(run),
        cases=len(m["cases"]),
        replayed_trajectories=trajectories,
        independently_rechecked_fixed_points=sample_points,
        full_saved_model_disagreements=disagreements,
        maximum_absolute_independent_clearance_on_disagreements=max(
            (r["maximum_absolute_independent_clearance"] for r in boundary_rows),
            default=0,
        ),
        maximum_saved_sample_step=maximum_step,
        exact_forest_NTO_segment_checks=exact_checks,
        exact_collision_free_NTO_paths=exact_valid,
        case_files=len(before),
        case_bytes=sum(r["bytes"] for r in before.values()),
        case_inventory_hash=digest(before),
        failure_counts=dict(Counter(r["status"] for r in failures)),
        case_failure_counts=dict(Counter(r["semantic_status"] for r in case_failures)),
        seed_failure_mechanisms=dict(
            Counter(r["failure_mechanism"] for r in seed_rows)
        ),
        lgcs_solver_status_counts=dict(Counter(str(r["solver_status"]) for r in lgcs)),
        lgcs_maximum_recorded_gap=max(
            [r["gap"] for r in lgcs if r["gap"] is not None], default=None
        ),
        verification_scope="17 fixed evenly spaced samples per saved candidate rechecked in independent geometry; all original dense flags and metadata reconciled; Forest order-2 NTO additionally checked by exact segments",
        issues=issues,
    )
    atomic(out / "audit.json", result)
    print(result, flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", nargs="+", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    rows = [
        audit(
            run,
            Path(args.output) / read(Path(run) / "manifest.json")["environment"],
            args.resume,
        )
        for run in args.run_dir
    ]
    return int(any(row["issues"] for row in rows))


if __name__ == "__main__":
    raise SystemExit(main())
