"""Plot every saved case; figures cannot launch or select new experiments."""

from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Patch
from matplotlib.colors import ListedColormap, BoundaryNorm
from .common import ROOT, atomic, read
from .optimization import clipped_polygon, bezier_values
from .pipeline import validate
from .forest import Forest
from pydrake.geometry.optimization import HPolyhedron
from pydrake.trajectories import BsplineTrajectory
from pydrake.math import BsplineBasis

COLORS = {
    "LSCS": "#40789D",
    "LGCS": "#6F9FAD",
    "SCSTO": "#4F8767",
    "NTO_distance": "#8D6C65",
    "NTO_time": "#AC785B",
}
LABELS = {
    "LSCS": "LSCS",
    "LGCS": "LGCS",
    "SCSTO": "SCSTO",
    "NTO_distance": "NTO distance",
    "NTO_time": "NTO time",
}
plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
        "font.size": 8,
        "axes.labelsize": 8,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 7,
        "axes.titlesize": 9,
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.7,
        "legend.frameon": False,
    }
)


def export(fig, out, name):
    svg_path = out / (name + ".svg")
    fig.savefig(svg_path, bbox_inches="tight")
    svg_path.write_text(
        "\n".join(line.rstrip() for line in svg_path.read_text().splitlines()) + "\n"
    )
    fig.savefig(out / (name + ".pdf"), bbox_inches="tight")
    fig.savefig(
        out / (name + ".tiff"),
        dpi=600,
        bbox_inches="tight",
        pil_kwargs={"compression": "tiff_lzw"},
    )
    fig.savefig(out / (name + ".png"), dpi=300, bbox_inches="tight")
    plt.close(fig)


def panel(ax, letter, title):
    ax.set_title(title, loc="left", pad=10)
    ax.text(
        -0.12,
        1.045,
        letter,
        transform=ax.transAxes,
        fontsize=10,
        fontweight="bold",
        va="bottom",
    )


def candidate_points(path, metadata):
    with np.load(path, allow_pickle=False) as arrays:
        if "points" in arrays:
            return arrays["points"]
        if "durations" in arrays:
            controls = arrays["controls"]
            durations = arrays["durations"]
            q, _ = bezier_values(
                controls, durations, np.linspace(0, durations.sum(), 1000)
            )
            return q
        trajectory = BsplineTrajectory(
            BsplineBasis(metadata["spline_order"], arrays["knots"].tolist()),
            [point.reshape(2, 1) for point in arrays["controls"]],
        )
        return trajectory.vector_values(
            np.linspace(trajectory.start_time(), trajectory.end_time(), 1000)
        ).T


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--summary-dir", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    run = Path(args.run_dir).resolve()
    summary = Path(args.summary_dir).resolve()
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    manifest = read(run / "manifest.json")
    results = read(summary / "summary.json")
    data = pd.read_csv(summary / "methods.csv")
    selection = []
    timing_counts = {}
    # One fixed ordering rule determines the geometry example before outcomes.
    first = sorted(manifest["cases"], key=lambda row: row["id"])[0]
    case = run / "cases" / first["id"]
    forest = Forest(manifest["config"], first["environment_seed"])
    fig, axes = plt.subplots(
        2, 2, figsize=(7.2, 6.2), gridspec_kw={"hspace": 0.52, "wspace": 0.42}
    )
    ax = axes[0, 0]
    panel(ax, "a", "Frozen Forest example")
    for center in forest.centers:
        ax.add_patch(
            Circle(
                center, forest.radius, color="#B0B2B5", alpha=0.85, lw=0.5, ec="#666666"
            )
        )
    regions = sorted((case / "LSCS").glob("region_00_*.npz"))
    for path in regions:
        with np.load(path) as array:
            polygon = clipped_polygon(HPolyhedron(array["A"], array["b"]))
        if len(polygon) > 2:
            ax.add_patch(
                Polygon(
                    polygon,
                    facecolor=COLORS["LSCS"],
                    edgecolor=COLORS["LSCS"],
                    alpha=0.13,
                    lw=0.6,
                )
            )
    with np.load(case / "roadmap.npz") as arrays:
        path = arrays["path"]
    ax.plot(path[:, 0], path[:, 1], color="#565B63", ls=":", lw=1.0, label="DRM seed")
    for method in ["LSCS", "SCSTO", "NTO_distance"]:
        directory = case / method
        if not directory.exists():
            continue
        candidates = sorted(directory.glob("candidate_*.npz"))
        if not candidates:
            continue
        candidate = candidates[-1]
        metadata = read(
            directory / ("optimization_" + candidate.stem.split("_")[-1] + ".json")
        )
        points = candidate_points(candidate, metadata)
        record = read(directory / "result.json")
        passed = record["status"] == "success_numerical"
        label = LABELS[method] + (" (invalid)" if not passed else "")
        ax.plot(
            points[:, 0],
            points[:, 1],
            color=COLORS[method] if passed else "#B85252",
            ls="-" if passed else "--",
            lw=1.3,
            label=label,
        )
        selection.append(
            dict(
                case_id=first["id"],
                method=method,
                artifact=str(candidate.relative_to(run)),
            )
        )
    ax.scatter(*manifest["config"]["start"], s=16, color="#252525", zorder=5)
    ax.scatter(
        *manifest["config"]["goal"],
        s=55,
        marker="*",
        color="#D9AE48",
        edgecolor="#665025",
        lw=0.5,
        zorder=5,
    )
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="upper left", fontsize=6)
    ax = axes[0, 1]
    panel(ax, "b", "Independent validation: all cases")
    methods = manifest["config"]["method_order"]
    values = [
        next(row for row in results["method_summaries"] if row["method"] == method)
        for method in methods
    ]
    rates = [row["validated_successes"] / row["requested"] for row in values]
    ax.barh(
        np.arange(len(methods)), rates, color=[COLORS[m] for m in methods], height=0.63
    )
    ax.set_yticks(np.arange(len(methods)), [LABELS[m] for m in methods])
    ax.invert_yaxis()
    ax.set_xlim(0, 1.19)
    ax.set_xlabel("Validated / requested")
    for i, row in enumerate(values):
        ax.text(
            rates[i] + 0.02,
            i,
            f"{row['validated_successes']}/{row['requested']}",
            va="center",
            fontsize=7,
        )
    ax.set_xticks([0, 0.5, 1])
    ax.grid(axis="x", color="#E2E4E6", lw=0.5)
    ax.set_axisbelow(True)
    for ax, letter, problem, title in [
        (axes[1, 0], "c", "distance", "Distance solver + construction"),
        (axes[1, 1], "d", "time", "Time solver + construction"),
    ]:
        panel(ax, letter, title)
        for method in methods:
            part = data[(data.method == method) & (data.problem == problem)]
            if part.empty:
                continue
            raw_timing = part.total_ms.to_numpy()
            observed = raw_timing[np.isfinite(raw_timing) & (raw_timing > 0)]
            timing_counts[method] = dict(
                before=len(part),
                after=len(observed),
                missing_or_nonpositive=len(part) - len(observed),
            )
            assert np.all(observed > 0)
            if not len(observed):
                continue
            observed = np.sort(observed)
            cdf = np.arange(1, len(observed) + 1) / len(observed)
            assert np.all(np.diff(observed) >= 0)
            ax.step(
                observed,
                cdf,
                where="post",
                color=COLORS[method],
                lw=1.3,
                label=LABELS[method] + f" (n={len(observed)})",
            )
        ax.set_xscale("log")
        from matplotlib.ticker import FuncFormatter

        ax.xaxis.set_major_formatter(
            FuncFormatter(lambda value, position: f"{value:g}")
        )
        ax.set_ylim(0, 1.04)
        ax.set_xlabel("Measured TO + CS time (ms)")
        ax.set_ylabel("Observed cumulative fraction")
        ax.legend(loc="lower right", fontsize=6)
        ax.grid(color="#E5E5E5", lw=0.4)
    fig.suptitle("Forest reconstruction on Quadro RTX 8000", fontsize=11, y=1.01)
    export(fig, out, "forest_results")
    # Full case-status grid: no missing case, failure, or size is removed.
    status_order = [
        "success_numerical",
        "validation_failed",
        "optimization_failed",
        "not_run_due_to_drm",
        "not_run",
        "error",
        "blocked_license",
    ]
    status_colors = [
        "#4F8767",
        "#D9A054",
        "#9B6E73",
        "#A8ADB4",
        "#E5E7EA",
        "#433C46",
        "#7364A1",
    ]
    envs = sorted({row["environment_seed"] for row in manifest["cases"]})
    seeds = sorted({row["roadmap_seed"] for row in manifest["cases"]})
    sizes = sorted({row["roadmap_size"] for row in manifest["cases"]})
    columns = [(s, n) for s in seeds for n in sizes]
    fig, axes = plt.subplots(
        len(methods), 1, figsize=(9.0, 8.0), gridspec_kw={"hspace": 0.60}
    )
    for ax, method in zip(axes, methods):
        matrix = np.full((len(envs), len(columns)), 4, dtype=int)
        for i, e in enumerate(envs):
            for j, (s, n) in enumerate(columns):
                selected = data[
                    (data.method == method)
                    & (data.environment_seed == e)
                    & (data.roadmap_seed == s)
                    & (data.roadmap_size == n)
                ]
                assert len(selected) == 1
                status = selected.iloc[0].status
                matrix[i, j] = (
                    status_order.index(status) if status in status_order else 5
                )
        ax.imshow(
            matrix,
            aspect="auto",
            interpolation="nearest",
            cmap=ListedColormap(status_colors),
            norm=BoundaryNorm(
                np.arange(-0.5, len(status_order) + 0.5), len(status_order)
            ),
        )
        ax.set_yticks([0, len(envs) - 1], [envs[0], envs[-1]])
        ax.set_ylabel("Env. seed")
        ax.set_title(LABELS[method], loc="left", fontsize=9, pad=3)
        ax.set_xticks(
            np.arange(0, len(columns), len(sizes)) + (len(sizes) - 1) / 2,
            [str(s) for s in seeds],
        )
        ax.tick_params(length=0)
        for x in np.arange(len(sizes) - 0.5, len(columns), len(sizes)):
            ax.axvline(x, color="white", lw=0.7)
    fig.subplots_adjust(top=0.94, bottom=0.065, left=0.065, right=0.99)
    axes[-1].set_xlabel("Roadmap seed; within each group: 200, 400, 800, 1600 nodes")
    present = [status for status in status_order if status in set(data.status)]
    handles = [
        Patch(
            facecolor=status_colors[status_order.index(status)],
            label={
                "success_numerical": "Validated success",
                "validation_failed": "Failed validation",
                "optimization_failed": "Solver failure",
                "not_run_due_to_drm": "No valid DRM seed",
                "not_run": "Not run",
                "error": "Program error",
                "blocked_license": "License unavailable",
            }[status],
        )
        for status in present
    ]
    fig.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.0),
        ncol=min(4, len(handles)),
        fontsize=7,
    )
    export(fig, out, "all_case_statuses")
    atomic(
        out / "figure_sources.json",
        dict(
            run_dir=str(run),
            source_csvs=[
                str(summary / name)
                for name in [
                    "methods.csv",
                    "summary.csv",
                    "jobs.csv",
                    "paired_comparisons.csv",
                ]
            ],
            selected_geometry_artifacts=selection,
            quantitative_exclusions="none; missing timing values visible in status grid and counts",
            figure_1_uncertainty="all-case observed proportions and full empirical CDF; no iid error bars",
            figure_2_case_count=len(manifest["cases"]),
            exports=["editable SVG", "editable PDF", "600dpi LZW TIFF", "300dpi PNG"],
            timing_observation_counts=timing_counts,
            minimum_font_size_points=6,
        ),
    )
    print("Saved figures to", out)


if __name__ == "__main__":
    main()
