"""Separate license-independent diagnostic for saved LGCS inputs.

Default mode only inventories blocked inputs. --execute explicitly enables
bounded all-simple-path enumeration and the existing Clarabel corridor SOCP.
Original results and frozen numerical sources are never modified.
"""

from __future__ import annotations

import argparse
from collections import Counter
import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

import numpy as np
from reproduction.common import ROOT, atomic, read, serial


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def simple_paths(adjacency, source, target):
    """Deterministic DFS; each complete simple path appears once."""
    path, visited = [source], {source}
    stack = [iter(adjacency[source])]
    while stack:
        node = next(stack[-1], None)
        if node is None:
            stack.pop()
            visited.remove(path.pop())
        elif node not in visited:
            if node == target:
                yield path + [target]
            else:
                path.append(node)
                visited.add(node)
                stack.append(iter(adjacency[node]))


def inventory(base):
    manifest = read(base / "manifest.json")
    cases = []
    for result_path in sorted((base / "cases").glob("*/LGCS/result.json")):
        result = read(result_path)
        if result.get("status") != "blocked_license":
            continue
        directory = result_path.parent
        pre_solve = sorted(directory.glob("pre_solve_[0-9][0-9].json"))
        item = dict(case_id=directory.parent.name, source_result=str(result_path),
                    source_result_sha256=sha(result_path),
                    optimization_metadata=result.get("optimization_metadata"),
                    saved_problem_available=bool(pre_solve))
        if pre_solve:
            iteration = int(pre_solve[-1].stem.rsplit("_", 1)[1])
            regions = sorted(directory.glob(f"pre_solve_region_{iteration:02d}_*.npz"))
            seed_path = directory / f"pre_solve_seeds_{iteration:02d}.npz"
            with np.load(seed_path) as data:
                endpoints = data["path"][[0, -1]].copy()
            item.update(iteration=iteration, region_count=len(regions),
                        start=endpoints[0], goal=endpoints[1],
                        pre_solve=str(pre_solve[-1]),
                        source_files={str(p): sha(p) for p in [*regions, seed_path, pre_solve[-1]]},
                        region_files=[str(p) for p in regions])
        cases.append(item)
    return dict(base_run=str(base), base_status=manifest.get("status"),
                environment=manifest["environment"], blocked_inputs=cases,
                blocked_count=len(cases),
                graph_size_histogram=dict(Counter(str((x.get("optimization_metadata") or {}).get("graph_vertices"))
                                                 for x in cases)),
                graph_intersections_evaluated=False, solver_calls=0, gpu_calls=0)


def independent_validate(candidate, regions, problem, manifest, case_input, destination):
    from reproduction.paper_constructed.validation import polygon_samples
    points = candidate["points"]
    cfg = manifest["config"]
    q = polygon_samples(points, cfg["validation_step"])
    if manifest["environment"] == "fr3":
        from reproduction.paper_constructed.model import drake_fr3
        _, context, plant, pc, sg, _, _ = drake_fr3(cfg, case_input["obstacles"])
        free = []
        for sample in q:
            plant.SetPositions(pc, sample)
            distances = sg.get_query_output_port().Eval(sg.GetMyContextFromRoot(context)).ComputeSignedDistancePairwiseClosestPoints(0.0)
            free.append(not any(x.distance < 0 for x in distances))
        free = np.asarray(free, dtype=bool)
        lower, upper = plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits()
        continuous_circle_free = None
    else:
        from reproduction.common import config
        forest_cfg = config()
        forest_cfg.update({key: value for key, value in cfg.items() if key in forest_cfg})
        centers = np.asarray(case_input["centers"])
        radius = forest_cfg["obstacle_radius"]
        free = (np.linalg.norm(q[:, None] - centers[None], axis=2) >= radius).all(axis=1)
        u, v = points[:-1], points[1:]
        delta = v - u
        squared = (delta * delta).sum(axis=1)
        parameter = ((centers[None] - u[:, None]) * delta[:, None]).sum(axis=2)
        parameter = np.divide(parameter, squared[:, None], out=np.zeros_like(parameter), where=squared[:, None] > 0)
        nearest = u[:, None] + np.clip(parameter, 0, 1)[..., None] * delta[:, None]
        continuous_circle_free = bool((np.linalg.norm(nearest - centers[None], axis=2) >= radius).all())
        lower, upper = np.asarray(forest_cfg["domain_lower"]), np.asarray(forest_cfg["domain_upper"])
    residual = max(float(np.max(regions[r].A() @ points[j:j + 2].T - regions[r].b()[:, None]))
                   for j, r in enumerate(candidate["segment_regions"]))
    endpoint_error = float(max(np.linalg.norm(points[0] - problem["start"]),
                               np.linalg.norm(points[-1] - problem["goal"])))
    bounds = bool((q >= lower - cfg["feasibility_tolerance"]).all()
                  and (q <= upper + cfg["feasibility_tolerance"]).all())
    np.savez_compressed(destination / "independent_samples.npz", q=q, independent_collision_free=free)
    passed = (free.all() and continuous_circle_free is not False and bounds
              and endpoint_error <= cfg["endpoint_tolerance"]
              and residual <= cfg["feasibility_tolerance"])
    return dict(passed=bool(passed), independent_collision_free=bool(free.all()),
                samples=len(q), collisions=int((~free).sum()), continuous_circle_free=continuous_circle_free,
                region_residual=residual, endpoint_error=endpoint_error, limits_pass=bounds,
                native_cuda_evaluated=False,
                scope="Independent numerical geometry validation; no new inflation or recovery; not continuous FR3 certificate")


def worker(base, destination, case_id, path_limit):
    from pydrake.all import HPolyhedron
    from reproduction.paper_constructed.optimization import lscs
    from reproduction.paper_constructed.run import require_identity
    from reproduction.pipeline import save_candidate

    manifest = read(base / "manifest.json")
    require_identity(manifest)
    problem = next(x for x in read(destination.parent / "inventory.json")["blocked_inputs"]
                   if x["case_id"] == case_id)
    if not problem["saved_problem_available"]:
        raise ValueError("Blocked solver input was not saved")
    for name, expected in problem["source_files"].items():
        if sha(name) != expected:
            raise ValueError("Saved optimization input changed: " + name)
    regions = []
    for name in problem["region_files"]:
        with np.load(name) as archive:
            regions.append(HPolyhedron(archive["A"], archive["b"]))
    start, goal = np.asarray(problem["start"]), np.asarray(problem["goal"])
    n = len(regions)
    source, target = n, n + 1
    adjacency = {i: [] for i in range(n + 2)}
    begun = time.perf_counter()
    atomic(destination / "progress.json", dict(stage="graph_construction", elapsed_seconds=0))
    for i, region in enumerate(regions):
        if not region.IsBounded():
            raise ValueError("Saved region is unbounded; Eq5.5 projection precondition not satisfied")
        for j in range(i + 1, n):
            if region.IntersectsWith(regions[j]):
                adjacency[i].append(j)
                adjacency[j].append(i)
        if region.PointInSet(start, 1e-6):
            adjacency[source].append(i)
        if region.PointInSet(goal, 1e-6):
            adjacency[i].append(target)
    adjacency = {i: sorted(v) for i, v in adjacency.items()}
    atomic(destination / "graph.json", dict(adjacency=adjacency, source=source, target=target,
            region_vertices=n, directed_edges=sum(map(len, adjacency.values())),
            source_tolerance=1e-6, construction_seconds=time.perf_counter() - begun,
            same_edge_tests_as_frozen_LGCS=True))
    solved, unresolved, count, best, complete = 0, 0, 0, None, True
    with (destination / "corridors.jsonl").open("w") as ledger:
        for path in simple_paths(adjacency, source, target):
            if count >= path_limit:
                complete = False
                break
            sequence = path[1:-1]
            started = time.perf_counter()
            row = dict(index=count, sequence=sequence)
            try:
                answer = lscs([regions[i] for i in sequence], start, goal)
                row.update(seconds=time.perf_counter() - started,
                           solver_success=answer["solver_success"], solver_status=answer["solver_status"],
                           objective=answer["objective"])
                if answer["solver_success"]:
                    answer["segment_regions"] = sequence
                    points = answer["points"]
                    residual = max(float(np.max(regions[r].A() @ points[j:j + 2].T - regions[r].b()[:, None]))
                                   for j, r in enumerate(sequence))
                    measured = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
                    finite = bool(np.isfinite(points).all() and np.isfinite(answer["objective"]))
                    endpoint_error = float(max(np.linalg.norm(points[0] - start),
                                               np.linalg.norm(points[-1] - goal)))
                    numerical_pass = (finite and residual <= manifest["config"]["feasibility_tolerance"]
                                      and endpoint_error <= manifest["config"]["endpoint_tolerance"]
                                      and abs(measured - answer["objective"]) <= 1e-5 * max(1, measured))
                    row.update(region_residual=residual, measured_length=measured,
                               endpoint_error=endpoint_error, numerical_pass=numerical_pass)
                    np.savez_compressed(destination / f"corridor_{count:06d}.npz", points=points,
                                        sequence=np.asarray(sequence), objective=answer["objective"])
                    if numerical_pass:
                        solved += 1
                        if best is None or answer["objective"] < best["objective"]:
                            best = answer
                            best["method"] = "LGCS_all_simple_path_Clarabel_diagnostic"
                            save_candidate(destination / "best", best)
                    else:
                        unresolved += 1
                elif "kInfeasibleConstraints" not in answer["solver_status"]:
                    unresolved += 1
            except Exception as error:
                unresolved += 1
                row.update(seconds=time.perf_counter() - started, error=repr(error), traceback=traceback.format_exc())
            ledger.write(json.dumps(row, default=serial, allow_nan=False) + "\n")
            ledger.flush()
            os.fsync(ledger.fileno())
            count += 1
            atomic(destination / "progress.json", dict(stage="enumerating", completed_corridors=count,
                    successful_corridors=solved, unresolved_corridors=unresolved,
                    best_objective=best["objective"] if best else None, elapsed_seconds=time.perf_counter() - begun))
    result = dict(case_id=case_id, status="complete" if complete and not unresolved else "unresolved",
                  all_simple_paths_enumerated=complete, corridors_attempted=count,
                  successful_corridors=solved, unresolved_corridors=unresolved,
                  enumeration_limit=path_limit, numerical_global_optimum_established=bool(complete and not unresolved and best),
                  objective=best["objective"] if best else None,
                  enumeration_and_SOCP_seconds=time.perf_counter() - begun,
                  not_Gurobi_timing=True, not_paper_algorithm=True, main_results_unchanged=True)
    atomic(destination / "result.json", result)
    if best:
        atomic(destination / "progress.json", dict(stage="independent_validation", completed_corridors=count,
                elapsed_seconds=time.perf_counter() - begun))
        independent = independent_validate(best, regions, problem, manifest,
                read(base / "cases" / case_id / "input.json"), destination / "best")
        atomic(destination / "best" / "validation.json", independent)
        result["independent_validation_passed"] = independent["passed"]
        result["total_diagnostic_seconds"] = time.perf_counter() - begun
        atomic(destination / "result.json", result)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--case-timeout", type=float, default=120)
    parser.add_argument("--path-limit", type=int, default=10000)
    parser.add_argument("--worker-case", help=argparse.SUPPRESS)
    args = parser.parse_args()
    base = args.run_dir.resolve()
    if args.worker_case:
        try:
            worker(base, args.output, args.worker_case, args.path_limit)
        except BaseException as error:
            atomic(args.output / "worker_failure.json", dict(error=repr(error), traceback=traceback.format_exc()))
            raise
        return
    if args.case_timeout <= 0 or args.path_limit <= 0:
        raise ValueError("Positive timeout and path limit required")
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    out = (args.output or ROOT / "reproduction/runs" / (stamp + "_constructed_LGCS_enumeration")).resolve()
    out.mkdir(parents=True, exist_ok=False)
    inputs = inventory(base)
    atomic(out / "inventory.json", inputs)
    metadata = dict(status="inventory_only", execute=args.execute, driver_sha256=sha(__file__),
                    base_run=str(base), base_manifest_sha256=sha(base / "manifest.json"),
                    scope="license-independent all-simple-path SOCP diagnostic on saved blocked inputs",
                    original_results_unchanged=True, gpu_calls=0,
                    case_timeout_seconds=args.case_timeout, path_limit=args.path_limit,
                    exact_enumeration_claim_requires="all simple paths exhausted with no unresolved SOCP/numerical failures",
                    time_comparison="Diagnostic wall time is not Gurobi time and is not pooled into Table I.")
    atomic(out / "manifest.json", metadata)
    print("ENUMERATION_DIR=" + str(out), flush=True)
    print("blocked inputs:", inputs["blocked_count"], "graph sizes:", inputs["graph_size_histogram"], flush=True)
    if not args.execute:
        return
    if inputs["base_status"] != "completed":
        raise ValueError("Execute only after the primary run finishes")
    metadata["status"] = "running"
    atomic(out / "manifest.json", metadata)
    rows = []
    for entry in inputs["blocked_inputs"]:
        destination = out / entry["case_id"]
        destination.mkdir()
        command = [sys.executable, "-m", "reproduction.enumerate_constructed_lgcs", "--run-dir", str(base),
                   "--output", str(destination), "--worker-case", entry["case_id"], "--path-limit", str(args.path_limit)]
        started = time.perf_counter()
        with (destination / "stdout.log").open("w") as stdout, (destination / "stderr.log").open("w") as stderr:
            process = subprocess.Popen(command, cwd=ROOT, stdout=stdout, stderr=stderr, start_new_session=True)
            timed_out = False
            try:
                process.wait(timeout=args.case_timeout)
            except subprocess.TimeoutExpired:
                timed_out = True
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        row = dict(case_id=entry["case_id"], command=command, exit_code=process.returncode,
                   timed_out=timed_out, elapsed_seconds=time.perf_counter() - started)
        if (destination / "result.json").exists():
            row["result"] = read(destination / "result.json")
        if (destination / "progress.json").exists():
            row["last_progress"] = read(destination / "progress.json")
        if (destination / "worker_failure.json").exists():
            row["worker_failure"] = read(destination / "worker_failure.json")
        atomic(destination / "execution.json", row)
        rows.append(row)
        atomic(out / "results.json", rows)
    metadata.update(status="completed_diagnostic_with_failures_retained", attempted_cases=len(rows),
                    timed_out=sum(x["timed_out"] for x in rows),
                    numerically_solved=sum(x.get("result", {}).get("numerical_global_optimum_established", False)
                                           and x["exit_code"] == 0 for x in rows))
    atomic(out / "manifest.json", metadata)


if __name__ == "__main__":
    main()
