"""One deterministic adapter engineering check; not benchmark aggregate data."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import load_config, write
from benchmarks.generic_motion_v1.runners.trajopt_baseline import solve
from benchmarks.generic_motion_v1.runners.validate import validate

parser = argparse.ArgumentParser()
parser.add_argument("--out", type=Path, required=True)
args = parser.parse_args()
base = ROOT / "benchmarks/generic_motion_v1"
run = base / "results/rss2025_reliable_v2_smoke_s000"
manifest = json.loads((run / "run_manifest.json").read_text())
job = next(j for j in manifest["jobs"] if j["case_id"] == "smoke-bottleneck-000-p00"
           and j["method"] == "EI_ZO" and j["objective"] == "minimum_distance")
seed_path = run / "jobs" / job["job_id"] / "attempts/0001/path.npy"
case = job["case"]
points = np.load(seed_path)
endpoints = np.array([case["q_start"], case["q_goal"]])
error = float(np.max(np.abs(points[[0, -1]]-endpoints)))
if error > 2e-7:
    raise ValueError("Historical DRM path exceeds fixed native roundtrip tolerance")
points[[0, -1]] = endpoints
model = Model(case["voxels_xyz"], case["voxel_radius_m"])
config = load_config(base / "configs/extended_baselines.json")
args.out.mkdir(parents=True, exist_ok=False)
write(args.out / "probe_source.json", dict(
    purpose="adapter engineering probe; not a timed comparative observation",
    source_seed=str(seed_path.relative_to(ROOT)),
    source_seed_sha256=hashlib.sha256(seed_path.read_bytes()).hexdigest(),
    endpoint_float32_error=error,
    endpoint_policy="restore immutable manifest endpoints within 2e-7 native roundtrip",
    problem_id=case["problem_id"], case=case))
started = time.perf_counter()
candidate = solve(points, model, case, config, args.out, started+55)
seconds = time.perf_counter()-started
write(args.out / "candidate.json", candidate)
report = validate(candidate, model, case, config, out=args.out)
summary = dict(solve_seconds=seconds, solver_status=candidate["solver_status"],
               passed=report["passed"], cost=report["cost"])
write(args.out / "probe_summary.json", summary)
print(json.dumps(summary, indent=2))
