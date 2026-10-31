"""Audit saved artifacts without rerunning or changing benchmark outcomes."""

from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import gzip
from pathlib import Path
import numpy as np
from .common import atomic, digest, read, sources
from .forest import Forest
from .run import runtime_versions


def file_hash(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    run, out = Path(args.run_dir).resolve(), Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    manifest = read(run / "manifest.json")
    issues, invalid_seeds, exact_distance = [], [], []
    statuses = Counter()
    calls = 0
    maximum_step = 0.0
    missing_candidates = []
    for entry in manifest["cases"]:
        directory = run / "cases" / entry["id"]
        case = read(directory / "case.json")
        f = Forest(manifest["config"], entry["environment_seed"])
        if case["implementation_hash"] != manifest["implementation_hash"]:
            issues.append(entry["id"] + ": changed source identity")
        saved_input = read(directory / "input.json")
        if not np.array_equal(saved_input["centers"], f.centers):
            issues.append(entry["id"] + ": obstacle identity mismatch")
        with np.load(directory / "roadmap.npz") as a:
            path = a["path"]
        seed_check = f.validate_polygon(path)
        if case["drm_validated_success"] != bool(
            seed_check["collision_free"] and seed_check["endpoint_pass"]
        ):
            issues.append(entry["id"] + ": seed validation mismatch")
        if not case["drm_validated_success"]:
            invalid_seeds.append(dict(id=entry["id"], **seed_check))
        for method, result in case["methods"].items():
            calls += 1
            statuses[(method, result["status"])] += 1
            dest = directory / method
            if result != read(dest / "result.json"):
                issues.append(entry["id"] + ": checkpoint/result mismatch: " + method)
            candidates = sorted(dest.glob("candidate_*.npz"))
            if not candidates:
                missing_candidates.append([entry["id"], method])
            validation_path = dest / "validation.json"
            if validation_path.exists():
                v = read(validation_path)
                maximum_step = max(maximum_step, v.get("maximum_sample_joint_step", 0))
                if result["status"] == "success_numerical" and not v["passed"]:
                    issues.append(entry["id"] + ": inconsistent validation: " + method)
            if method == "NTO_distance" and candidates:
                # Order-two clamped B-splines are polygonal. Independently check
                # every full segment, including portions between dense samples.
                with np.load(candidates[-1]) as a:
                    check = f.validate_polygon(a["controls"])
                exact_distance.append(
                    dict(
                        id=entry["id"],
                        status=result["status"],
                        exact_collision_free=check["collision_free"],
                        exact_minimum_clearance=check["minimum_clearance"],
                    )
                )
                if (
                    result["status"] == "success_numerical"
                    and not check["collision_free"]
                ):
                    issues.append(
                        entry["id"] + ": sampled NTO passed but exact segment failed"
                    )
    current_hash = digest(sources())
    if current_hash != manifest["implementation_hash"]:
        issues.append("frozen computational sources changed")
    if runtime_versions() != manifest["runtime_versions"]:
        issues.append("numerical runtime versions changed")
    inventory = {}
    for path in sorted((run / "cases").rglob("*")):
        if path.is_file():
            inventory[str(path.relative_to(run))] = dict(
                bytes=path.stat().st_size, sha256=file_hash(path)
            )
    atomic(out / "case_file_inventory.json", inventory)
    (out / "case_file_inventory.json.gz").write_bytes(
        gzip.compress((out / "case_file_inventory.json").read_bytes(), mtime=0)
    )
    atomic(out / "invalid_seed_paths.json", invalid_seeds)
    atomic(out / "nto_distance_exact_segment_checks.json", exact_distance)
    summary = dict(
        requested=len(manifest["cases"]),
        eligible=len(manifest["cases"]) - len(invalid_seeds),
        actual_method_calls=calls,
        missing_candidate_files=missing_candidates,
        case_files=len(inventory),
        case_bytes=sum(v["bytes"] for v in inventory.values()),
        case_inventory_hash=digest(inventory),
        frozen_implementation_hash=current_hash,
        source_and_runtime_match=not any("changed" in issue for issue in issues),
        maximum_saved_spline_sample_step=maximum_step,
        exact_nto_distance_segment_checks=len(exact_distance),
        exact_nto_distance_passes=sum(
            r["exact_collision_free"] for r in exact_distance
        ),
        status_counts=[
            dict(method=m, status=s, count=n) for (m, s), n in sorted(statuses.items())
        ],
        issues=issues,
    )
    atomic(out / "audit.json", summary)
    print(summary)
    return int(bool(issues or missing_candidates))


if __name__ == "__main__":
    raise SystemExit(main())
