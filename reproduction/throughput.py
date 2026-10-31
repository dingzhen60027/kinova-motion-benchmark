"""Paper's two-million-configuration GPU batch size on reconstructed Forests."""

from __future__ import annotations
import datetime
import hashlib
from pathlib import Path
import time
import numpy as np
import csdecomp as csd
from .common import ROOT, atomic, config
from .forest import Forest


def main():
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    out = ROOT / "reproduction/runs" / (stamp + "_forest_throughput")
    out.mkdir(parents=True)
    cfg = config()
    rows = []
    meta = dict(
        scope="Forest_throughput_reconstruction_different_hardware",
        batch_configurations=2000000,
        repeats=5,
        environment_seeds=cfg["environment_seeds"],
        timing="synchronous public CheckCollisionFreeCuda wrapper including allocations and host/device transfers; sampling and file IO excluded",
        reference="paper Appendix D: 19.9 million configurations/s on RTX 3090",
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        status="running",
    )
    atomic(out / "manifest.json", meta)
    print("THROUGHPUT_DIR=" + str(out), flush=True)
    for seed in cfg["environment_seeds"]:
        f = Forest(cfg, seed)
        rng = np.random.default_rng(np.random.SeedSequence([2504, seed]))
        q = np.asfortranarray(rng.uniform(-5, 5, (2, 2000000)).astype(np.float32))
        destination = out / f"e{seed:02d}"
        destination.mkdir()
        np.save(destination / "configurations.npy", q)
        # Warm-up of the exact batch size is explicitly timed and retained.
        started = time.perf_counter()
        warm = np.asarray(csd.CheckCollisionFreeCuda(q, f.minimal), bool)
        warm_seconds = time.perf_counter() - started
        timings = []
        for repeat in range(5):
            started = time.perf_counter()
            result = np.asarray(csd.CheckCollisionFreeCuda(q, f.minimal), bool)
            elapsed = time.perf_counter() - started
            assert np.array_equal(result, warm)
            timings.append(
                dict(
                    repeat=repeat,
                    seconds=elapsed,
                    configurations_per_second=2000000 / elapsed,
                )
            )
        sample = np.linspace(0, 1999999, 20000).astype(int)
        analytic = f.clearances(q[:, sample].T) >= 0
        disagreements = int(np.count_nonzero(result[sample] != analytic))
        np.savez_compressed(
            destination / "results.npz",
            collision_free=result,
            verification_indices=sample,
            analytic_collision_free=analytic,
        )
        record = dict(
            seed=seed,
            warmup_seconds=warm_seconds,
            configuration_sha256=hashlib.sha256(q.tobytes()).hexdigest(),
            verified_points=len(sample),
            analytic_disagreements=disagreements,
            measurements=timings,
        )
        rows.append(record)
        atomic(destination / "result.json", record)
        atomic(out / "results.json", rows)
        print(
            seed,
            "mean Mconfig/s",
            np.mean([r["configurations_per_second"] for r in timings]) / 1e6,
            "disagreements",
            disagreements,
            flush=True,
        )
    rates = [
        r["configurations_per_second"] for row in rows for r in row["measurements"]
    ]
    summary = dict(
        measured_batches=len(rates),
        configurations_per_batch=2000000,
        mean_million_configurations_per_second=float(np.mean(rates) / 1e6),
        sd_million_configurations_per_second=float(np.std(rates, ddof=1) / 1e6),
        median_million_configurations_per_second=float(np.median(rates) / 1e6),
        independent_verified_points=sum(r["verified_points"] for r in rows),
        analytic_disagreements=sum(r["analytic_disagreements"] for r in rows),
        reference_million_configurations_per_second=19.9,
        hardware="Quadro RTX 8000; reference RTX 3090",
        scope=meta["scope"],
    )
    atomic(out / "summary.json", summary)
    meta["status"] = "completed"
    atomic(out / "manifest.json", meta)
    print(summary, flush=True)


if __name__ == "__main__":
    main()
