"""Read pinned public YAMLs; inspect schema/duplicates without running a planner.

Run from repo root with the existing environment:
bash local/python.sh benchmarks/generic_motion_v1/reports/benchmark_design_audit_v2/literature_mbm/inspect_public_datasets.py --output /tmp/robometrics_inspection.json
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import urllib.request

import yaml

COMMIT = "81e3d1d605de84100d8ab880b43096aba221a48b"


def digest(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    rows = []
    for name in ["mb_set", "mpinets_set"]:
        url = f"https://raw.githubusercontent.com/fishbotics/robometrics/{COMMIT}/robometrics/content/dataset/{name}.yaml"
        raw = urllib.request.urlopen(url, timeout=60).read()
        data = yaml.load(raw, Loader=yaml.CSafeLoader)
        probs = [p for group in data.values() for p in group]
        obstacles = {k: {digest(p["obstacles"]) for p in v} for k, v in data.items()}
        overlaps = [
            {"group_a": a, "group_b": b, "shared_exact_obstacle_records": len(obstacles[a] & obstacles[b])}
            for i, a in enumerate(obstacles) for b in list(obstacles)[i + 1:]
            if obstacles[a] & obstacles[b]
        ]
        rows.append({
            "url": url, "sha256": hashlib.sha256(raw).hexdigest(), "bytes": len(raw),
            "groups": {k: len(v) for k, v in data.items()}, "total": len(probs),
            "unique_obstacle_records": len({digest(p["obstacles"]) for p in probs}),
            "unique_start_records": len({digest(p["start"]) for p in probs}),
            "goal_ik_count_distribution": dict(collections.Counter(len(p["goal_ik"]) for p in probs)),
            "world_frames": dict(collections.Counter(p["world_frame"] for p in probs)),
            "record_keys": sorted(probs[0]), "cross_group_obstacle_overlap": overlaps,
            "note": "Exact-record uniqueness does not prove statistical independence. No collision validation, environment modification or planning run.",
        })
    args.output.write_text(json.dumps(rows, indent=2) + "\n")
    print(json.dumps({"output": str(args.output), "datasets": [{"total": x["total"], "unique_obstacles": x["unique_obstacle_records"], "overlap": x["cross_group_obstacle_overlap"]} for x in rows]}, indent=2))


if __name__ == "__main__":
    main()
