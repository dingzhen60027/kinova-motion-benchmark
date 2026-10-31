"""Two-million-configuration batches on the reconstructed FR3 scenes."""

import argparse
import datetime
import hashlib
from pathlib import Path
import time
import numpy as np
from reproduction.common import ROOT, atomic, read, digest
from .model import FR3
from .run import identity, runtime_versions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    args = parser.parse_args()
    base = Path(args.run_dir)
    source = read(base / "manifest.json")
    if source["environment"] != "fr3":
        raise ValueError("Expected a FR3 run")
    if digest(identity()) != source["implementation_hash"]:
        raise ValueError("Source identity changed")
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    out = ROOT / "reproduction/runs" / (stamp + "_paper_fr3_throughput")
    out.mkdir(parents=True)
    manifest = dict(
        status="running",
        scope="FR3_33_sphere_reconstruction_not_original_author_geometry",
        base_run=str(base.resolve()),
        configurations_per_batch=2000000,
        repeats=5,
        environments=source["config"]["environment_seeds"],
        computational_identity=source["implementation_hash"],
        runtime_versions=runtime_versions(),
        driver_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        timing="synchronous public wrapper including allocations and host/device transfers; samples and IO excluded",
    )
    atomic(out / "manifest.json", manifest)
    print("THROUGHPUT_DIR=" + str(out), flush=True)
    rows = []
    for seed in manifest["environments"]:
        scene = read(base / "scenes" / f"e{seed:02d}" / "scene.json")
        env = FR3(source["config"], scene["obstacles"])
        dest = out / f"e{seed:02d}"
        dest.mkdir()
        q = (
            np.random.default_rng(np.random.SeedSequence([2504, seed]))
            .uniform(env.lower, env.upper, (2000000, 7))
            .astype(np.float32)
        )
        np.save(dest / "configurations.npy", q)
        tic = time.perf_counter()
        warm = env.cuda(q)
        warm_seconds = time.perf_counter() - tic
        timings = []
        for repeat in range(5):
            tic = time.perf_counter()
            actual = env.cuda(q)
            seconds = time.perf_counter() - tic
            if not np.array_equal(actual, warm):
                raise ValueError("Repeated collision batches disagree")
            timings.append(
                dict(
                    repeat=repeat,
                    seconds=seconds,
                    configurations_per_second=len(q) / seconds,
                )
            )
        indices = np.linspace(0, len(q) - 1, 20000).astype(int)
        independent = env.independent(q[indices])
        different = indices[actual[indices] != independent]
        np.savez_compressed(
            dest / "flags.npz",
            collision_free=actual,
            verification_indices=indices,
            independent=independent,
            disagreement_indices=different,
        )
        record = dict(
            seed=seed,
            warmup_seconds=warm_seconds,
            verified_points=len(indices),
            model_disagreements=len(different),
            static_pairs=env.minimal.num_collision_pairs,
            obstacle_pairs=len(env.geometry_ids) * len(scene["obstacles"]),
            measurements=timings,
        )
        rows.append(record)
        atomic(dest / "result.json", record)
        atomic(out / "results.json", rows)
        print(
            seed,
            "Mconfig/s",
            np.mean([x["configurations_per_second"] for x in timings]) / 1e6,
            "disagreements",
            len(different),
            flush=True,
        )
    values = [
        x["configurations_per_second"] / 1e6 for r in rows for x in r["measurements"]
    ]
    summary = dict(
        batches=len(values),
        configurations_per_batch=2000000,
        mean_million_configurations_per_second=float(np.mean(values)),
        sample_sd=float(np.std(values, ddof=1)),
        verified_points=sum(r["verified_points"] for r in rows),
        model_disagreements=sum(r["model_disagreements"] for r in rows),
        original_paper_reference_million_per_second=8.6,
        comparison="different hardware and reconstructed collision geometry; no direct speedup claim",
    )
    atomic(out / "summary.json", summary)
    manifest["status"] = "completed"
    atomic(out / "manifest.json", manifest)
    print(summary, flush=True)


if __name__ == "__main__":
    main()
