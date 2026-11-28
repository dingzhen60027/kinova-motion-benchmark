"""Command line: generate, check, run, verify and summarize the recovery benchmark."""
from __future__ import annotations

import argparse
import csv
from concurrent.futures import ProcessPoolExecutor
import json
import multiprocessing
import os
from pathlib import Path
import platform
import sys

import numpy as np

from . import BENCHMARK
from .generator import DATA, ScheduledSubmission, generate, load_tasks
from .harness import (HERE, ROOT, InProcessPlanner, WorkerPlanner, digest, load_config, replay, run_episode,
                      sha, source_hashes, write_json)
from .policies import REGISTRY

ORACLE = "witness"


def resolve(path):
    path = Path(path)
    return path if path.is_absolute() else (ROOT / path)


# ----------------------------------------------------------------------------- data checks

def check_data(data=DATA):
    """Integrity of frozen task files: hashes, identities, split disjointness, witness evidence."""
    manifest = json.loads((Path(data) / "manifest.json").read_text())
    report, layouts = dict(passed=True, splits={}), {}
    for split, entry in manifest["splits"].items():
        path = Path(data) / entry["file"]
        problems = []
        if sha(path) != entry["sha256"]:
            problems.append("task file hash differs from manifest")
        tasks = load_tasks(split, data, accepted_only=False)
        ids = [t["task_id"] for t in tasks]
        if len(ids) != len(set(ids)):
            problems.append("duplicate task identifiers")
        accepted = [t for t in tasks if t["generation"]["status"] == "accepted"]
        for t in accepted:
            validation = t["oracle"].get("witness_validation", {})
            if validation.get("status") != "success":
                problems.append(t["task_id"] + ": witness lacks a successful closed-loop validation")
            if t["split"] != split:
                problems.append(t["task_id"] + ": split label differs")
            layouts.setdefault(digest(t["public"]["scene"]["static_primitives"]), set()).add(split)
        report["splits"][split] = dict(registered=len(tasks), accepted=len(accepted), problems=problems)
        report["passed"] &= not problems
    shared = [sorted(s) for s in layouts.values() if len(s) > 1]
    report["layouts_shared_between_splits"] = shared
    report["passed"] &= not shared
    return report


# ----------------------------------------------------------------------------- runs

def _episode(job):
    """Run one (task, method) episode in a fresh process; return its terminal status."""
    task, method, directory = job
    directory = Path(directory)
    if (directory / "receipt.json").exists():
        return task["task_id"], method, json.loads((directory / "result.json").read_text())["status"], "reused"
    if directory.exists():
        # An interrupted attempt is kept for inspection and never silently overwritten.
        os.replace(directory, directory.with_name(directory.name + f".interrupted-{os.getpid()}"))
    if method == ORACLE:
        planner = InProcessPlanner(ScheduledSubmission(task["oracle"]["witness"]["submission"]))
        spec = dict(name=ORACLE, privileged=True)
    else:
        planner = WorkerPlanner(REGISTRY[method], {}, directory.parent / f".{method}_workdir")
        spec = dict(name=method, target=REGISTRY[method])
    result = run_episode(task, planner, directory, method=spec)
    return task["task_id"], method, result["status"], "new"


def run(split, methods, output, tasks=None, workers=1, data=DATA):
    output = resolve(output)
    selected = load_tasks(split, data)
    if tasks:
        selected = [t for t in selected if t["task_id"] in set(tasks)]
    manifest_path = output / "run_manifest.json"
    manifest = dict(benchmark=BENCHMARK, split=split, methods=list(methods), tasks=[t["task_id"] for t in selected],
                    task_file_sha256=sha(Path(data) / f"{split}.jsonl"), config_sha256=sha(HERE / "config.json"),
                    source_hashes=source_hashes(), workers=workers, host=platform.node(), cpu_count=os.cpu_count(),
                    python=platform.python_version(),
                    latency_note="Planner latency is wall-clock time; concurrent episodes share the CPU.")
    if manifest_path.exists():
        previous = json.loads(manifest_path.read_text())
        for key in ("split", "task_file_sha256", "config_sha256", "source_hashes"):
            if previous[key] != manifest[key]:
                raise SystemExit(f"Run {output} was started with a different {key}; use a new output directory")
        manifest["methods"] = sorted(set(previous["methods"]) | set(methods))
        manifest["tasks"] = sorted(set(previous["tasks"]) | set(manifest["tasks"]))
    write_json(manifest_path, manifest)
    jobs = [(t, m, str(output / "episodes" / t["task_id"] / m)) for t in selected for m in methods]
    if workers > 1:
        with ProcessPoolExecutor(workers, mp_context=multiprocessing.get_context("spawn")) as pool:
            outcomes = list(pool.map(_episode, jobs))
    else:
        outcomes = [_episode(job) for job in jobs]
    for task_id, method, status, origin in outcomes:
        print(f"{task_id:48s} {method:14s} {status:28s} {origin}")
    return outcomes


def verify(output, replays=0, data=DATA):
    """Check every episode's receipt and artifacts; optionally replay a fixed subset."""
    output = resolve(output)
    manifest = json.loads((output / "run_manifest.json").read_text())
    tasks = {t["task_id"]: t for t in load_tasks(manifest["split"], data)}
    report = dict(planned=0, complete=0, problems=[], replays=[])
    for task_id in manifest["tasks"]:
        for method in manifest["methods"]:
            report["planned"] += 1
            directory = output / "episodes" / task_id / method
            if not (directory / "receipt.json").exists():
                report["problems"].append(f"{task_id}/{method}: missing")
                continue
            result = json.loads((directory / "result.json").read_text())
            receipt = json.loads((directory / "receipt.json").read_text())
            if sha(directory / "result.json") != receipt["result_sha256"]:
                report["problems"].append(f"{task_id}/{method}: result hash differs from receipt")
            for name, value in result.get("artifacts", {}).items():
                if not (directory / name).is_file() or sha(directory / name) != value:
                    report["problems"].append(f"{task_id}/{method}: artifact {name} changed")
            if result["task_sha256"] != digest(tasks[task_id]):
                report["problems"].append(f"{task_id}/{method}: task differs from the frozen task file")
            if result["status"] == "infrastructure_error":
                report["problems"].append(f"{task_id}/{method}: infrastructure error")
            report["complete"] += 1
    if replays:
        # Evenly spaced tasks for every replayable method, so each method's episodes are re-simulated.
        methods = sorted(m for m in manifest["methods"] if m != ORACLE)
        tasks_sorted = sorted(manifest["tasks"])
        per_method = -(-replays // max(1, len(methods)))
        chosen = []
        for k, method in enumerate(methods):
            picks = np.linspace(0, len(tasks_sorted) - 1, per_method).round().astype(int)
            chosen += [(tasks_sorted[(int(i) + k) % len(tasks_sorted)], method) for i in picks]
        for task_id, method in chosen:
            target = output / "replays" / task_id / method
            if target.exists():
                import shutil
                shutil.rmtree(target)
            comparison = replay(tasks[task_id], output / "episodes" / task_id / method, target)
            report["replays"].append(dict(task_id=task_id, method=method, **comparison))
            if not (comparison["identical_states"] and comparison["identical_outcome"]):
                report["problems"].append(f"{task_id}/{method}: replay differs")
    report["integrity_passed"] = not report["problems"]
    write_json(output / "verification.json", report)
    return report


# ----------------------------------------------------------------------------- summary

FIELDS = ("task_id", "layout_id", "workspace", "family", "method", "status", "success", "completed",
          "completion_time_s", "witness_completion_time_s", "time_ratio_to_witness", "collision",
          "interventions", "minimum_clearance_m", "minimum_dynamic_clearance_m", "limit_violation",
          "torque_saturation_steps", "calls", "submissions", "rejected_submissions", "deadline_misses",
          "latency_p50_s", "latency_p95_s", "latency_max_s", "planner_errors", "executed_path_length_rad")


def records(output, data=DATA):
    output = resolve(output)
    manifest = json.loads((output / "run_manifest.json").read_text())
    tasks = {t["task_id"]: t for t in load_tasks(manifest["split"], data)}
    rows = []
    for task_id in manifest["tasks"]:
        task = tasks[task_id]
        witness_time = task["oracle"]["witness_validation"]["completion_time_s"]
        for method in manifest["methods"]:
            path = output / "episodes" / task_id / method / "result.json"
            if not path.exists():
                continue
            r = json.loads(path.read_text())
            p, latency = r.get("planner", {}), (r.get("planner", {}).get("latency_s") or {})
            outcomes = p.get("submission_outcomes", {})
            rows.append(dict(
                task_id=task_id, layout_id=task["layout_id"], workspace=task["workspace"], family=task["family"],
                method=method, status=r["status"], success=r.get("success", False), completed=r.get("completed", False),
                completion_time_s=r.get("completion_time_s"), witness_completion_time_s=witness_time,
                time_ratio_to_witness=(r["completion_time_s"] / witness_time) if r.get("completion_time_s") else None,
                collision=r.get("collision", {}).get("occurred"), interventions=r.get("interventions", {}).get("count"),
                minimum_clearance_m=r.get("clearance", {}).get("minimum_m"),
                minimum_dynamic_clearance_m=r.get("clearance", {}).get("minimum_dynamic_m"),
                limit_violation=r.get("limits", {}).get("violated"),
                torque_saturation_steps=r.get("limits", {}).get("torque_saturation_steps"),
                calls=p.get("calls"), submissions=p.get("submissions"),
                rejected_submissions=sum(v for k, v in outcomes.items() if k != "accepted"),
                deadline_misses=p.get("deadline_misses"), latency_p50_s=latency.get("p50"),
                latency_p95_s=latency.get("p95"), latency_max_s=latency.get("max"), planner_errors=p.get("errors"),
                executed_path_length_rad=r.get("executed", {}).get("path_length_rad")))
    return rows


def _cluster_interval(rows, value, rng, resamples=2000):
    """95% percentile interval of a mean, resampling layouts with replacement."""
    layouts = sorted({r["layout_id"] for r in rows})
    groups = {l: [value(r) for r in rows if r["layout_id"] == l] for l in layouts}
    estimates = []
    for _ in range(resamples):
        picked = rng.choice(len(layouts), len(layouts))
        values = [v for i in picked for v in groups[layouts[i]]]
        estimates.append(np.mean(values))
    return [float(np.percentile(estimates, 2.5)), float(np.percentile(estimates, 97.5))]


def aggregate(rows):
    rng = np.random.default_rng(0)
    rate = lambda key: float(np.mean([bool(r[key]) for r in rows]))
    status = lambda name: float(np.mean([r["status"] == name for r in rows]))
    done = [r["completion_time_s"] for r in rows if r["completed"]]
    ratio = [r["time_ratio_to_witness"] for r in rows if r["completed"]]
    calls = sum(r["calls"] or 0 for r in rows)
    submissions = sum(r["submissions"] or 0 for r in rows)
    return dict(
        tasks=len(rows), layouts=len({r["layout_id"] for r in rows}),
        safe_completion_rate=rate("success"),
        safe_completion_rate_ci95=_cluster_interval(rows, lambda r: float(r["success"]), rng),
        completion_rate=rate("completed"), collision_rate=status("collision"), timeout_rate=status("timeout"),
        intervention_rate=float(np.mean([(r["interventions"] or 0) > 0 for r in rows])),
        completed_with_intervention_rate=status("completed_with_intervention"),
        limit_violation_rate=status("limit_violation"), infrastructure_error_rate=status("infrastructure_error"),
        median_completion_time_s=float(np.median(done)) if done else None,
        median_time_ratio_to_witness=float(np.median(ratio)) if ratio else None,
        median_minimum_dynamic_clearance_m=float(np.median([r["minimum_dynamic_clearance_m"] for r in rows
                                                            if r["minimum_dynamic_clearance_m"] is not None])) if rows else None,
        deadline_miss_rate=(sum(r["deadline_misses"] or 0 for r in rows) / submissions) if submissions else 0.0,
        rejected_submission_rate=(sum(r["rejected_submissions"] or 0 for r in rows) / submissions) if submissions else 0.0,
        latency_p95_s_max=max((r["latency_p95_s"] or 0) for r in rows), planner_calls=calls)


def summarize(output, data=DATA):
    output = resolve(output)
    rows = records(output, data)
    with (output / "records.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    summary = dict(benchmark=BENCHMARK, denominator="every accepted task of the split; failures are included",
                   methods={}, by_family={}, by_workspace={})
    for method in sorted({r["method"] for r in rows}):
        mine = [r for r in rows if r["method"] == method]
        summary["methods"][method] = aggregate(mine)
        summary["by_family"][method] = {f: aggregate([r for r in mine if r["family"] == f])
                                        for f in sorted({r["family"] for r in mine})}
        summary["by_workspace"][method] = {w: aggregate([r for r in mine if r["workspace"] == w])
                                           for w in sorted({r["workspace"] for r in mine})}
    write_json(output / "summary.json", summary)
    return summary


# ----------------------------------------------------------------------------- entry point

def main(argv=None):
    parser = argparse.ArgumentParser(prog="dynamic_recovery", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    g = sub.add_parser("generate", help="generate frozen task splits (method-independent)")
    g.add_argument("--splits", nargs="+", default=["smoke", "dev", "test"])
    g.add_argument("--workers", type=int, default=1)
    sub.add_parser("check", help="verify frozen task files")
    r = sub.add_parser("run", help="run methods on a split")
    r.add_argument("--split", required=True)
    r.add_argument("--methods", nargs="+", required=True, choices=sorted(REGISTRY) + [ORACLE])
    r.add_argument("--output", required=True)
    r.add_argument("--tasks", nargs="*")
    r.add_argument("--workers", type=int, default=1)
    v = sub.add_parser("verify", help="check run integrity and replay determinism")
    v.add_argument("--run", required=True)
    v.add_argument("--replays", type=int, default=3)
    s = sub.add_parser("summarize", help="write records.csv and summary.json")
    s.add_argument("--run", required=True)
    args = parser.parse_args(argv)
    if args.command == "generate":
        result = generate(args.splits, workers=args.workers)
        print(json.dumps({k: {x: v[x] for x in ("registered_tasks", "accepted_tasks", "generation_failures", "families")}
                          for k, v in result["splits"].items()}, indent=2))
    elif args.command == "check":
        result = check_data()
        print(json.dumps(result, indent=2))
        return 0 if result["passed"] else 1
    elif args.command == "run":
        run(args.split, args.methods, args.output, args.tasks, args.workers)
    elif args.command == "verify":
        result = verify(args.run, args.replays)
        print(json.dumps({k: result[k] for k in ("planned", "complete", "integrity_passed", "problems")}, indent=2))
        return 0 if result["integrity_passed"] else 1
    elif args.command == "summarize":
        print(json.dumps(summarize(args.run)["methods"], indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
