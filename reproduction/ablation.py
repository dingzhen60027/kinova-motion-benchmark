"""Sensitivity to an unreleased NTO transcription setting, on identical inputs.

This is a separate post-benchmark ablation, not a replacement of the frozen run.
Only the distance B-spline's control count changes: 20 arc-length-resampled warm
start points instead of the original DRM-node count. Nc remains 20. All cases,
including the nine invalid DRM seed paths, retain their original identities.
"""

from __future__ import annotations
import argparse
import datetime
import hashlib
from pathlib import Path
import time
import traceback
import numpy as np
from .common import ROOT, atomic, read
from .forest import Forest
from .optimization import nto
from .pipeline import save_candidate, validate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    args = parser.parse_args()
    base = Path(args.run_dir).resolve()
    manifest = read(base / "manifest.json")
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    out = ROOT / "reproduction/runs" / (stamp + "_nto_control_count_ablation")
    out.mkdir(parents=True)
    meta = dict(
        scope="post_benchmark_sensitivity_not_original_paper_artifact",
        base_run=str(base),
        changed_parameter="NTO_distance controls=20; same Nc=20",
        base_implementation_hash=manifest["implementation_hash"],
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        requested_cases=len(manifest["cases"]),
        status="running",
    )
    atomic(out / "manifest.json", meta)
    print("ABLATION_DIR=" + str(out), flush=True)
    rows = []
    for index, entry in enumerate(manifest["cases"], 1):
        case = read(base / "cases" / entry["id"] / "case.json")
        dest = out / entry["id"]
        dest.mkdir()
        record = dict(
            **entry, method="NTO_distance_20_controls", status="not_run_due_to_drm"
        )
        try:
            if not case.get("drm_validated_success"):
                continue
            forest = Forest(manifest["config"], entry["environment_seed"])
            path = np.load(base / "cases" / entry["id"] / "roadmap.npz")["path"]
            distances = np.r_[
                0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))
            ]
            dense = np.column_stack(
                [
                    np.interp(np.linspace(0, distances[-1], 20), distances, path[:, i])
                    for i in range(2)
                ]
            )
            np.savez_compressed(
                dest / "warm_start.npz", original_path=path, dense_controls=dense
            )
            started = time.perf_counter()
            candidate = nto(dense, forest, False)
            elapsed = time.perf_counter() - started
            candidate["method"] = "NTO_distance"
            save_candidate(dest, candidate)
            validation = validate(candidate, forest)
            samples = validation.pop("_samples", None)
            if samples:
                np.savez_compressed(dest / "samples.npz", **samples)
            atomic(dest / "validation.json", validation)
            record.update(
                status=(
                    "success_numerical"
                    if candidate["solver_success"] and validation["passed"]
                    else (
                        "validation_failed"
                        if candidate["solver_success"]
                        else "optimization_failed"
                    )
                ),
                solver_success=candidate["solver_success"],
                validation_passed=validation["passed"],
                collision_free=validation["collision_free"],
                cost=validation["cost"],
                minimum_clearance=validation["minimum_clearance"],
                optimization_seconds=elapsed,
            )
        except Exception as error:
            traceback.print_exc()
            record.update(
                status="error", reason=repr(error), traceback=traceback.format_exc()
            )
        finally:
            atomic(dest / "result.json", record)
            rows.append(record)
            atomic(out / "results.json", rows)
            print(index, entry["id"], record["status"], flush=True)
    meta["status"] = "completed"
    atomic(out / "manifest.json", meta)
    summary = dict(
        requested=len(rows),
        drm_eligible=sum(r["status"] != "not_run_due_to_drm" for r in rows),
        status_counts={
            status: sum(r["status"] == status for r in rows)
            for status in sorted({r["status"] for r in rows})
        },
        observed_mean_solver_ms=float(
            np.mean(
                [
                    r["optimization_seconds"] * 1000
                    for r in rows
                    if "optimization_seconds" in r
                ]
            )
        ),
        scope=meta["scope"],
    )
    atomic(out / "summary.json", summary)
    print(summary, flush=True)


if __name__ == "__main__":
    main()
