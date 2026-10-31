"""CPU-only forensic audit of saved v5 inputs, paths and Forest NTO candidates.

This does not plan, optimize, load CUDA, change old artifacts, or tune scenes.
It recomputes line/circle intersections analytically and replays the documented
native edge sampling pattern in NumPy. Native samples are a numeric emulation,
not a new invocation of the native collision checker.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import io
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FOREST = ROOT / "reproduction/runs/20261026T025110.685774Z_paper_constructed_forest"
DEFAULT_FR3 = ROOT / "reproduction/runs/20261026T025846.407684Z_paper_constructed_fr3"


def point_clearance(points, centers, radius):
    return np.linalg.norm(np.asarray(points)[:, None] - centers[None], axis=2).min(axis=1) - radius


def segment_circle_audit(points, centers, radius, parameters=None):
    """Return exact closest distances and open intersection intervals in float64."""
    points, centers = np.asarray(points, float), np.asarray(centers, float)
    if parameters is None:
        parameters = np.linspace(0, 1, len(points))
    result = []
    for j, (start, end) in enumerate(zip(points[:-1], points[1:])):
        delta = end - start
        length = float(np.linalg.norm(delta))
        if length == 0:
            projection = np.zeros(len(centers))
            distances = np.linalg.norm(centers - start, axis=1)
        else:
            projection = ((centers - start) @ delta) / length**2
            closest = start + np.clip(projection, 0, 1)[:, None] * delta
            distances = np.linalg.norm(centers - closest, axis=1)
        for obstacle in np.flatnonzero(distances < radius):
            if length == 0:
                left, right = 0., 1.
            else:
                foot = start + projection[obstacle] * delta
                height2 = float((centers[obstacle] - foot) @ (centers[obstacle] - foot))
                half = np.sqrt(max(radius**2 - height2, 0.)) / length
                left = max(0., float(projection[obstacle] - half))
                right = min(1., float(projection[obstacle] + half))
            s0, s1 = parameters[j:j+2]
            result.append(dict(segment=j, obstacle=int(obstacle),
                clearance_m=float(distances[obstacle] - radius),
                fraction_entry=left, fraction_exit=right,
                parameter_entry=float(s0 + left * (s1-s0)),
                parameter_exit=float(s0 + right * (s1-s0)),
                collision_arc_length_m=float((right-left)*length)))
    # Calculate the nonnegative minimum too, independently of intersection list.
    minimum = float("inf")
    for start, end in zip(points[:-1], points[1:]):
        delta = end-start
        t = np.clip(((centers-start) @ delta) / max(float(delta@delta), 1e-30), 0, 1)
        minimum = min(minimum, float((np.linalg.norm(start+t[:, None]*delta-centers, axis=1)-radius).min()))
    return dict(minimum_clearance_m=minimum, intersections=result)


def native_pattern(points, centers, radius, step):
    """Emulate drm_planner.cpp SteerFunctionIsSafe float32 sample locations."""
    minimum = float("inf")
    count = 0
    by_segment = []
    for start, end in zip(points[:-1].astype(np.float32), points[1:].astype(np.float32)):
        delta = end-start
        length = float(np.linalg.norm(delta))
        if length == 0:
            q = start[None]
        else:
            direction = delta / np.float32(length)
            n = int(length / float(step) + 1)
            q = start + ((np.arange(n, dtype=np.float32)*np.float32(step))[:, None]*direction)
        clear = point_clearance(q.astype(float), centers, radius)
        by_segment.append(dict(samples=len(q), minimum_clearance_m=float(clear.min())))
        minimum = min(minimum, float(clear.min()))
        count += len(q)
    return dict(samples=count, minimum_clearance_m=minimum, by_segment=by_segment)


def controls():
    circle = np.array([[0., 0.]])
    through = segment_circle_audit(np.array([[-2., 0.], [2., 0.]]), circle, 1.)
    assert through["minimum_clearance_m"] == -1
    assert through["intersections"][0]["fraction_entry"] == .25
    assert through["intersections"][0]["fraction_exit"] == .75
    tangent = segment_circle_audit(np.array([[-2., 1.], [2., 1.]]), circle, 1.)
    assert tangent["minimum_clearance_m"] == 0 and not tangent["intersections"]
    miss = segment_circle_audit(np.array([[-2., 2.], [2., 2.]]), circle, 1.)
    assert miss["minimum_clearance_m"] == 1 and not miss["intersections"]
    return ["diameter_entry_exit", "tangency", "separated_line"]


def stats(values):
    a = np.asarray(values, float)
    return dict(count=len(a), mean=float(a.mean()), std_population=float(a.std()),
                minimum=float(a.min()), median=float(np.median(a)), maximum=float(a.max())) if len(a) else {}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--forest", type=Path, default=DEFAULT_FOREST)
    parser.add_argument("--fr3", type=Path, default=DEFAULT_FR3)
    parser.add_argument("--output", type=Path, default=ROOT / "reproduction/reports/alignment_diagnosis/task_distribution")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    hashes = {}

    def read_bytes(path):
        data = path.read_bytes()
        hashes[str(path.relative_to(ROOT))] = hashlib.sha256(data).hexdigest()
        return data

    def read_json(path):
        return json.loads(read_bytes(path))

    def read_npz(path):
        with np.load(io.BytesIO(read_bytes(path)), allow_pickle=False) as archive:
            return {k: archive[k] for k in archive.files}

    def save(name, content):
        (args.output / name).write_text(json.dumps(content, indent=2, allow_nan=False) + "\n")

    def save_csv(name, rows):
        with (args.output/name).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    forest_cfg = read_json(ROOT / "reproduction/configs/forest.json")
    radius = forest_cfg["obstacle_radius"] + forest_cfg["point_radius"]
    case_rows, seeds, nto, seed_failures, environment_summary = [], [], [], [], {}
    geometric_failures = []
    for environment, run in [("forest", args.forest), ("fr3", args.fr3)]:
        manifest = read_json(run / "manifest.json")
        local_rows = []
        for entry in manifest["cases"]:
            directory = run / "cases" / entry["id"]
            case = read_json(directory / "case.json")
            inp = read_json(directory / "input.json")
            roadmap = case.get("roadmap", {})
            row = dict(environment=environment, case_id=entry["id"],
                environment_seed=entry["environment_seed"], roadmap_seed=entry["roadmap_seed"],
                roadmap_size=entry["roadmap_size"], native_drm_success=bool(roadmap.get("success")),
                independently_valid_seed=bool(case.get("drm_validated_success")),
                segments=roadmap.get("segments"), vertices=None,
                path_length=roadmap.get("path_length"),
                endpoint_distance=float(np.linalg.norm(np.asarray(inp["goal"])-inp["start"])))
            for name in manifest["config"]["method_order"]:
                method = case.get("methods", {}).get(name, {})
                row[name+"_status"] = method.get("status", "not_run")
                row[name+"_strict_success"] = method.get("status") == "success_numerical"
                if name in ["LSCS", "LGCS", "SCSTO"] and method.get("status") not in [None, "success_numerical"]:
                    failure = dict(environment=environment, case_id=entry["id"], method=name,
                        status=method["status"], reason=method.get("reason"),
                        optimization_metadata=method.get("optimization_metadata"))
                    validation_path = directory / name / "validation.json"
                    if validation_path.exists():
                        validation = read_json(validation_path)
                        failure["saved_validation"] = {k: validation[k] for k in [
                            "passed", "collision_free", "native_cuda_collision_free",
                            "independent_collision_free", "endpoint_error", "limits_pass",
                            "region_residual", "derivative_pass", "samples"]}
                    geometric_failures.append(failure)
            if roadmap.get("success"):
                path = read_npz(directory / "seed_path.npz")["path"]
                row["vertices"] = len(path)
                assert row["segments"] == len(path)-1
                assert abs(row["path_length"]-np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) < 1e-9
                if environment == "forest":
                    centers = np.asarray(inp["centers"])
                    exact = segment_circle_audit(path, centers, radius)
                    straight = segment_circle_audit(np.asarray([inp["start"], inp["goal"]]), centers, radius)
                    row["direct_clearance_m"] = straight["minimum_clearance_m"]
                    step = roadmap["native_planner_options"]["online_edge_step_size"]
                    sampled = native_pattern(path, centers, radius, step)
                    saved_clearance = roadmap["independent_validation"]["exact_circle_check"]["minimum_clearance"]
                    assert abs(exact["minimum_clearance_m"]-saved_clearance) < 1e-12
                    record = dict(case_id=entry["id"], exact=exact, native_sampling_emulation=sampled)
                    seeds.append(record)
                    if exact["minimum_clearance_m"] < 0:
                        seed_failures.append(record)
                    method_dir = directory / "NTO_distance"
                    candidate = read_npz(method_dir / "candidate_00.npz")
                    optimization = read_json(method_dir / "optimization_00.json")
                    assert optimization["spline_order"] == 2
                    q, knots = candidate["controls"], candidate["knots"][1:-1]
                    assert len(q) == len(knots)
                    s = np.asarray(optimization["collocation_locations"])
                    checks = np.column_stack([np.interp(s, knots, q[:, i]) for i in range(2)])
                    clearance = point_clearance(checks, centers, radius)
                    continuous = segment_circle_audit(q, centers, radius, knots)
                    sample_nodes = np.sort(np.unique(np.r_[0., s, 1.]))
                    samples = np.column_stack([np.interp(sample_nodes, knots, q[:, i]) for i in range(2)])
                    lengths = np.linalg.norm(np.diff(q, axis=0), axis=1)
                    # Exact arc length between constraints for an order-2 spline.
                    arc = np.r_[0., np.cumsum(lengths)]
                    gap_arc = np.diff(np.interp(sample_nodes, knots, arc))
                    nto.append(dict(case_id=entry["id"], environment_seed=entry["environment_seed"],
                        status=case["methods"]["NTO_distance"]["status"],
                        solver_success=optimization["solver_success"], control_count=len(q),
                        collocation_points=len(s), collocation_minimum_clearance_m=float(clearance.min()),
                        continuous_minimum_clearance_m=continuous["minimum_clearance_m"],
                        largest_constraint_gap_arc_length_m=float(gap_arc.max()),
                        largest_constraint_gap_chord_length_m=float(np.linalg.norm(np.diff(samples, axis=0), axis=1).max()),
                        continuous_intersections=continuous["intersections"]))
            case_rows.append(row)
            local_rows.append(row)
        successful = [r for r in local_rows if r["native_drm_success"]]
        environment_summary[environment] = dict(cases=len(local_rows), drm_successes=len(successful),
            valid_seeds=sum(r["independently_valid_seed"] for r in local_rows),
            segments=stats([r["segments"] for r in successful]),
            vertices=stats([r["vertices"] for r in successful]),
            path_length=stats([r["path_length"] for r in successful]),
            endpoint_distance=stats([r["endpoint_distance"] for r in successful]),
            segment_histogram=dict(sorted(Counter(r["segments"] for r in successful).items())),
            nto_distance_by_seed_segments={str(k): dict(Counter(
                r["NTO_distance_status"] for r in successful if r["segments"] == k))
                for k in sorted({r["segments"] for r in successful})},
            method_statuses={name:dict(Counter(r[name+"_status"] for r in local_rows))
                for name in manifest["config"]["method_order"]})
    scene_rows = []
    for environment in ["forest", "fr3"]:
        for scene in range(10):
            rows = [r for r in case_rows if r["environment"] == environment and r["environment_seed"] == scene]
            valid = [r for r in rows if r["native_drm_success"]]
            scene_row = dict(environment=environment, environment_seed=scene, cases=len(rows),
                drm_successes=len(valid), valid_seeds=sum(r["independently_valid_seed"] for r in rows),
                segments_mean=float(np.mean([r["segments"] for r in valid])),
                vertices_mean=float(np.mean([r["vertices"] for r in valid])),
                path_length_mean=float(np.mean([r["path_length"] for r in valid])),
                endpoint_distance_mean=float(np.mean([r["endpoint_distance"] for r in valid])),
                direct_clearance_m=rows[0].get("direct_clearance_m"))
            for name in ["LSCS", "LGCS", "SCSTO", "NTO_distance", "NTO_time"]:
                scene_row[name+"_strict_successes"] = sum(r[name+"_strict_success"] for r in rows)
            scene_rows.append(scene_row)
    failed_nto = [r for r in nto if r["status"] == "validation_failed"]
    summary = dict(schema_version=1, command=" ".join(sys.argv), controls=controls(),
        execution="CPU-only NumPy analysis of frozen artifacts; no native or optimizer calls",
        environments=environment_summary, forest_invalid_seed_cases=len(seed_failures),
        forest_invalid_seed_penetration_m=stats([-r["exact"]["minimum_clearance_m"] for r in seed_failures]),
        forest_invalid_seed_emulated_lazy_samples_all_clear=all(r["native_sampling_emulation"]["minimum_clearance_m"] > 0 for r in seed_failures),
        forest_invalid_seed_collision_arc_lengths_m=stats([x["collision_arc_length_m"] for r in seed_failures for x in r["exact"]["intersections"]]),
        forest_nto_solver_success_but_validation_failed=len(failed_nto),
        forest_nto_failed_collocation_clearance_m=stats([r["collocation_minimum_clearance_m"] for r in failed_nto]),
        forest_nto_failed_all_constraint_points_within_1e_6_m=all(r["collocation_minimum_clearance_m"] >= -1e-6 for r in failed_nto),
        forest_nto_failed_continuous_penetration_m=stats([-r["continuous_minimum_clearance_m"] for r in failed_nto]),
        forest_nto_failed_max_gap_arc_m=stats([r["largest_constraint_gap_arc_length_m"] for r in failed_nto]))
    # Frozen inputs must remain unchanged throughout this read-only audit.
    for relative, expected in hashes.items():
        assert hashlib.sha256((ROOT/relative).read_bytes()).hexdigest() == expected, relative
    summary["input_files_verified_unchanged"] = len(hashes)
    save("summary.json", summary)
    save("input_sha256.json", hashes)
    save("forest_seed_checks.json", seeds)
    save("forest_invalid_seeds.json", seed_failures)
    save("forest_nto_distance_checks.json", nto)
    save("geometric_failures.json", geometric_failures)
    for row in case_rows:
        row.setdefault("direct_clearance_m", None)
    save_csv("cases.csv", case_rows)
    save_csv("scenes.csv", scene_rows)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
