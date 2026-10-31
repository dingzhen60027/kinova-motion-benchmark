"""Derive paired and supplementary metrics solely from saved measurements."""

from __future__ import annotations
import argparse
from collections import Counter
import csv
from pathlib import Path
import numpy as np
from .common import atomic, read


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--forest-summary", required=True)
    parser.add_argument("--kinova", required=True)
    parser.add_argument("--throughput", required=True)
    parser.add_argument("--ablation", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    out = Path(args.output)
    rows = list(
        csv.DictReader((Path(args.forest_summary) / "paired_comparisons.csv").open())
    )
    paired = []
    for alternative, baseline in sorted(
        {(r["alternative"], r["baseline"]) for r in rows}
    ):
        selection = [
            r
            for r in rows
            if r["alternative"] == alternative and r["baseline"] == baseline
        ]
        common = [r for r in selection if r["common_validated_success"] == "True"]
        ratios = [
            float(r["paired_runtime_speed_ratio"])
            for r in selection
            if r["paired_runtime_speed_ratio"]
        ]
        paired.append(
            dict(
                alternative=alternative,
                baseline=baseline,
                common_validated=len(common),
                mean_cost_ratio=(
                    float(np.mean([float(r["cost_ratio"]) for r in common]))
                    if common
                    else None
                ),
                mean_cost_difference=(
                    float(np.mean([float(r["cost_difference"]) for r in common]))
                    if common
                    else None
                ),
                runtime_pairs=len(ratios),
                mean_paired_baseline_over_alternative_time_ratio=(
                    float(np.mean(ratios)) if ratios else None
                ),
                median_paired_baseline_over_alternative_time_ratio=(
                    float(np.median(ratios)) if ratios else None
                ),
            )
        )
    atomic(out / "paired_summary.json", paired)
    kinova = Path(args.kinova)
    records = read(kinova / "results.json")
    successes = [r for r in records if r["status"] == "success_numerical"]
    checks = []
    for record in successes:
        dest = kinova / f"e{record['seed']:02d}"
        with np.load(sorted(dest.glob("bezier_*.npz"))[-1]) as arrays:
            cp, durations = arrays["controls"], arrays["durations"]
            velocity = 6 * np.diff(cp, axis=1) / durations[:, None, None]
        residual = max(
            [0.0]
            + [float(abs(cp[i, -1] - cp[i + 1, 0]).max()) for i in range(len(cp) - 1)]
            + [
                float(abs(velocity[i, -1] - velocity[i + 1, 0]).max())
                for i in range(len(cp) - 1)
            ]
        )
        checks.append(
            dict(
                seed=record["seed"],
                C0_C1_residual=residual,
                rest_endpoint_velocity=float(
                    max(abs(velocity[0, 0]).max(), abs(velocity[-1, -1]).max())
                ),
                velocity_bound=float(abs(velocity).max()),
                drake_samples=record["validation"]["samples"],
                disagreements=record["validation"]["disagreements"],
            )
        )
    metrics = dict(
        requested=len(records),
        status_counts=dict(Counter(r["status"] for r in records)),
        successful_cases=len(successes),
        mean_construction_ms=(
            float(np.mean([r["construction_seconds"] * 1000 for r in successes]))
            if successes
            else None
        ),
        mean_optimization_ms=(
            float(np.mean([r["optimization_seconds"] * 1000 for r in successes]))
            if successes
            else None
        ),
        mean_duration_seconds=(
            float(np.mean([r["duration"] for r in successes])) if successes else None
        ),
        posthoc_continuity_checks=checks,
        failures=[r for r in records if r["status"] != "success_numerical"],
    )
    atomic(out / "kinova_auxiliary.json", metrics)
    for name, directory in [
        ("forest_throughput", args.throughput),
        ("nto_control_count_ablation", args.ablation),
    ]:
        atomic(
            out / (name + ".json"),
            dict(
                run_dir=str(Path(directory).resolve()),
                **read(Path(directory) / "summary.json"),
            ),
        )
    print(f"Saved paired and supplementary metrics to {out}")


if __name__ == "__main__":
    main()
