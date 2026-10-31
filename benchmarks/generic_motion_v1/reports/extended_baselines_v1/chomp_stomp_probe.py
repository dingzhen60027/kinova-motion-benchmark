"""Explicit CPU-only preflight on archived smoke seeds, not a benchmark score."""
import argparse
import json
from pathlib import Path
import time
import numpy as np

from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import load_config, write
from benchmarks.generic_motion_v1.runners.chomp_stomp import solve
from benchmarks.generic_motion_v1.runners.validate import sample_candidate

ROOT = Path(__file__).resolve().parents[4]
BASE = ROOT / "benchmarks/generic_motion_v1"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--problem", required=True)
    parser.add_argument("--method", choices=["CHOMP", "STOMP"], required=True)
    parser.add_argument("--output-suffix", default="v1")
    args = parser.parse_args()
    source = next((BASE / "results/rss2025_reliable_v2_smoke_s000/jobs").glob(
        f"{args.problem}-EI_ZO-minimum_distance-*/attempts/0001/path.npy"))
    cases = [json.loads(line) for line in (BASE / "manifests/smoke.jsonl").read_text().splitlines()]
    case = next(case for case in cases if case["problem_id"] == args.problem)
    case["_objective"] = "minimum_distance"
    path = np.load(source)
    model = Model(case["voxels_xyz"], case["voxel_radius_m"])
    out = Path(__file__).resolve().parent / f"chomp_stomp_probe_{args.output_suffix}" / args.problem / args.method
    if out.exists():
        raise ValueError(f"Refusing to overwrite existing probe {out}")
    start = time.perf_counter()
    candidate = solve(path, model, case, {}, out, start+50., 1961, args.method)
    seconds = time.perf_counter()-start
    samples = sample_candidate(candidate, load_config())
    clearance = model.clearance(samples["q"])
    np.savez_compressed(out / "cpu_validation.npz", **samples["derivatives"],
                        q=samples["q"], clearance=clearance)
    summary = dict(problem_id=args.problem, method=args.method,
                    probe_only=True, source_seed_path=str(source),
                    optimizer_seconds=seconds, solver_success=candidate["solver_success"],
                    dense_cpu_valid=bool(np.all(clearance > model.contact_band_m)),
                    dense_minimum_clearance_m=float(clearance.min()),
                    dense_samples=len(clearance), common_length_rad=samples["cost"],
                    gpu_used=False, full_common_validator_still_required=True)
    write(out / "probe_summary.json", summary)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
