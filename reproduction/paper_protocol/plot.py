"""Replay measured Forest/FR3 results, with complete denominators and failures."""

import argparse
import hashlib
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch
from scipy.spatial.transform import Rotation
from reproduction.common import atomic, read

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
        "font.sans-serif": ["DejaVu Sans"],
        "font.size": 8,
        "axes.labelsize": 8,
        "axes.titlesize": 9,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 6.5,
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "legend.frameon": False,
    }
)


def export(fig, out, name):
    fig.savefig(out / (name + ".pdf"), bbox_inches="tight")
    svg = out / (name + ".svg")
    fig.savefig(svg, bbox_inches="tight")
    svg.write_text("\n".join(x.rstrip() for x in svg.read_text().splitlines()) + "\n")
    fig.savefig(out / (name + ".png"), dpi=300, bbox_inches="tight")
    fig.savefig(
        out / (name + ".tiff"),
        dpi=600,
        bbox_inches="tight",
        pil_kwargs={"compression": "tiff_lzw"},
    )
    plt.close(fig)


def panel(ax, letter, title):
    ax.set_title(title, loc="left", pad=10)
    ax.text(-0.14, 1.05, letter, transform=ax.transAxes, fontweight="bold", fontsize=10)


def transforms(q, asset):
    result = {"fr3_link0": np.eye(4)}
    x = np.eye(4)
    for i in range(1, 9):
        a = asset["kinematics"][f"joint{i}"]["kinematic"]
        fixed = np.eye(4)
        fixed[:3, :3] = Rotation.from_euler(
            "xyz", [a["roll"], a["pitch"], a["yaw"]]
        ).as_matrix()
        fixed[:3, 3] = [a["x"], a["y"], a["z"]]
        x = x @ fixed
        if i < 8:
            turn = np.eye(4)
            turn[:3, :3] = Rotation.from_rotvec([0, 0, q[i - 1]]).as_matrix()
            x = x @ turn
        result[f"fr3_link{i}"] = x.copy()
    hand = np.eye(4)
    hand[:3, :3] = Rotation.from_rotvec([0, 0, -np.pi / 4]).as_matrix()
    result["fr3_hand"] = x @ hand
    for name, xyz, yaw in [
        ("fr3_leftfinger", [0, 0.04, 0.0584], 0),
        ("fr3_rightfinger", [0, -0.04, 0.0584], np.pi),
        ("fr3_hand_tcp", [0, 0, 0.1034], 0),
    ]:
        fixed = np.eye(4)
        fixed[:3, :3] = Rotation.from_rotvec([0, 0, yaw]).as_matrix()
        fixed[:3, 3] = xyz
        result[name] = result["fr3_hand"] @ fixed
    return result


def sphere(ax, center, radius, color, alpha):
    u = np.linspace(0, 2 * np.pi, 16)
    v = np.linspace(0, np.pi, 10)
    ax.plot_surface(
        center[0] + radius * np.outer(np.cos(u), np.sin(v)),
        center[1] + radius * np.outer(np.sin(u), np.sin(v)),
        center[2] + radius * np.outer(np.ones_like(u), np.cos(v)),
        color=color,
        alpha=alpha,
        linewidth=0,
        rasterized=True,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary-dir", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    base = Path(args.summary_dir)
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    summaries = {env: read(base / env / "summary.json") for env in ["forest", "fr3"]}
    data = {env: pd.read_csv(base / env / "methods.csv") for env in summaries}
    pairs = {env: pd.read_csv(base / env / "paired.csv") for env in summaries}
    for summary in summaries.values():
        if (
            summary["manifest_status"] != "completed"
            or summary["source_integrity_issues"]
            or summary["artifact_issues"]
        ):
            raise ValueError("Only complete audited summaries may be plotted")
    figure_sources = dict(
        source_files={},
        selection="all requested cases; fixed lexicographically first FR3 case for scene replay",
        hardware="i7-13700K / Quadro RTX 8000; paper hardware differs",
        guarantee="independent numerical validation, not continuous certification",
        fidelity="published parameters; reconstructed FR3 sphere fit and unpublished random inputs",
        timing="all recorded attempts including failures; construction + optimization, with explicitly flagged method-wall-time fallbacks for caught exceptions",
        paired_cost_display="all common validated observations; median and interquartile range, no iid confidence interval",
    )
    for env in data:
        for filename in ["methods.csv", "paired.csv", "summary.json"]:
            p = base / env / filename
            figure_sources["source_files"][str(p.resolve())] = hashlib.sha256(
                p.read_bytes()
            ).hexdigest()
    fig, axes = plt.subplots(
        3, 2, figsize=(7.2, 8.7), gridspec_kw={"hspace": 0.65, "wspace": 0.48}
    )
    for column, env in enumerate(["forest", "fr3"]):
        summary = summaries[env]
        methods = [r["method"] for r in summary["methods"]]
        ax = axes[0, column]
        panel(ax, "ab"[column], env.upper() + " independent validation")
        rates = [r["validated_successes"] / r["requested"] for r in summary["methods"]]
        ax.barh(
            range(len(methods)), rates, color=[COLORS[m] for m in methods], height=0.62
        )
        ax.set_yticks(range(len(methods)), [LABELS[m] for m in methods])
        ax.invert_yaxis()
        ax.set_xlim(0, 1.22)
        ax.set_xticks([0, 0.5, 1])
        ax.set_xlabel("Validated / requested")
        for i, r in enumerate(summary["methods"]):
            ax.text(
                rates[i] + 0.02,
                i,
                f"{r['validated_successes']}/{r['requested']}",
                va="center",
                fontsize=7,
            )
        ax.grid(axis="x", color="#E2E4E6", lw=0.5)
        ax.set_axisbelow(True)
        ax = axes[1, column]
        panel(ax, "cd"[column], env.upper() + " observed runtime")
        for method in methods:
            values = data[env].loc[data[env].method == method, "total_ms"].to_numpy()
            if np.any(values[np.isfinite(values)] <= 0):
                raise ValueError(
                    "Runtime log axis requires strictly positive observations"
                )
            observed = np.sort(values[np.isfinite(values)])
            if len(observed):
                ax.step(
                    observed,
                    np.arange(1, len(observed) + 1) / len(observed),
                    where="post",
                    lw=1.2,
                    color=COLORS[method],
                    label=f"{LABELS[method]} (n={len(observed)})",
                )
        ax.set_xscale("log")
        ax.tick_params(axis="x", labelsize=8)
        ax.set_ylim(0, 1.03)
        ax.set_xlabel("Recorded method time (ms)")
        ax.set_ylabel("Observed cumulative fraction")
        ax.legend(loc="lower right", fontsize=6)
        ax.grid(color="#E4E6E8", lw=0.4)
        ax = axes[2, column]
        panel(ax, "ef"[column], env.upper() + " paired validated cost")
        groups = [
            ("LSCS", "NTO_distance"),
            ("LGCS", "NTO_distance"),
            ("SCSTO", "NTO_time"),
        ]
        ticklabels = []
        for i, (alternative, baseline) in enumerate(groups):
            selection = pairs[env][
                (pairs[env].alternative == alternative)
                & (pairs[env].baseline == baseline)
                & (pairs[env].common_validated == True)
            ]
            values = selection.cost_ratio.to_numpy()
            values = values[np.isfinite(values)]
            ticklabels.append(f"{LABELS[alternative]}\n(n={len(values)})")
            if len(values):
                jitter = 0.14 * np.sin(np.arange(len(values)) * 2.399963229728653)
                ax.scatter(
                    i + jitter,
                    values,
                    s=5,
                    alpha=0.3,
                    color=COLORS[alternative],
                    edgecolors="none",
                    rasterized=True,
                )
                ax.plot(
                    [i - 0.19, i + 0.19],
                    [np.median(values)] * 2,
                    color=COLORS[alternative],
                    lw=2,
                )
                q25, median, q75 = np.quantile(values, [0.25, 0.5, 0.75])
                ax.errorbar(
                    i,
                    median,
                    yerr=np.array([[median - q25], [q75 - median]]),
                    color=COLORS[alternative],
                    capsize=3,
                    lw=1,
                )
        ax.axhline(1, color="#555555", lw=0.6, ls="--")
        ax.set_xticks(range(3), ticklabels)
        ax.set_ylabel("Cost / matching NTO baseline")
        ax.set_xlim(-0.5, 2.5)
        ax.grid(axis="y", color="#E4E6E8", lw=0.4)
    fig.suptitle(
        "Paper-parameter reconstruction: all requested cases", fontsize=11, y=1.01
    )
    export(fig, out, "paper_results")
    # Full case grids retain every failure and every absent solver output.
    classes = [
        "Validated success",
        "Failed validation",
        "Solver failure",
        "Invalid DRM seed",
        "License blocked",
        "DRM / scene unavailable",
        "Other error / timeout",
    ]
    colors = [
        "#4F8767",
        "#D9A054",
        "#9B6E73",
        "#A8ADB4",
        "#7364A1",
        "#D2D6DB",
        "#433C46",
    ]
    fig, axes = plt.subplots(
        5, 2, figsize=(9.4, 7.5), gridspec_kw={"hspace": 0.65, "wspace": 0.18}
    )
    for column, env in enumerate(["forest", "fr3"]):
        manifest = read(Path(summaries[env]["run_dir"]) / "manifest.json")
        sizes = sorted({r["roadmap_size"] for r in manifest["cases"]})
        seeds = sorted({r["roadmap_seed"] for r in manifest["cases"]})
        environments = sorted({r["environment_seed"] for r in manifest["cases"]})
        for row, method in enumerate([r["method"] for r in summaries[env]["methods"]]):
            ax = axes[row, column]
            grid = np.zeros((len(environments), len(seeds) * len(sizes)), int)
            for record in data[env][data[env].method == method].itertuples():
                status = record.status
                value = (
                    0
                    if status == "success_numerical"
                    else (
                        1
                        if status == "validation_failed"
                        else (
                            2
                            if status == "optimization_failed"
                            else (
                                3
                                if status == "invalid_seed_path"
                                else (
                                    4
                                    if status == "blocked_license"
                                    else (
                                        5
                                        if status.startswith("not_run_due_to_drm")
                                        or status.startswith("not_run_due_to_scene")
                                        or status.startswith("not_run_due_to_offline")
                                        else 6
                                    )
                                )
                            )
                        )
                    )
                )
                grid[
                    environments.index(record.environment_seed),
                    seeds.index(record.roadmap_seed) * len(sizes)
                    + sizes.index(record.roadmap_size),
                ] = value
            ax.imshow(
                grid,
                aspect="auto",
                interpolation="nearest",
                cmap=ListedColormap(colors),
                norm=BoundaryNorm(np.arange(-0.5, len(colors) + 0.5), len(colors)),
            )
            ax.set_title(
                (env.upper() + ": " if row == 0 else "") + LABELS[method],
                loc="left",
                fontsize=8,
                pad=4,
            )
            ax.set_yticks(
                [0, len(environments) - 1], [environments[0], environments[-1]]
            )
            ax.set_ylabel("Env. seed")
            ax.set_xticks(
                np.arange(len(seeds)) * len(sizes) + (len(sizes) - 1) / 2, seeds
            )
            if row == 4:
                ax.set_xlabel("Roadmap seed; 4 ascending sizes per group")
            for x in np.arange(1, len(seeds)) * len(sizes) - 0.5:
                ax.axvline(x, color="white", lw=0.5)
    fig.legend(
        handles=[Patch(facecolor=c, label=label) for c, label in zip(colors, classes)],
        loc="upper center",
        bbox_to_anchor=(0.5, 1.03),
        ncol=4,
        fontsize=6.5,
    )
    export(fig, out, "paper_case_statuses")
    # Geometry illustration uses an ID rule, never a success-based selection.
    run = Path(summaries["fr3"]["run_dir"])
    manifest = read(run / "manifest.json")
    first = sorted(manifest["cases"], key=lambda r: r["id"])[0]
    case = run / "cases" / first["id"]
    scene = read(run / "scenes" / f"e{first['environment_seed']:02d}" / "scene.json")
    asset = read(Path(__file__).parent / "assets/fr3_33.json")
    fig = plt.figure(figsize=(7.5, 5.2))
    ax = fig.add_subplot(111, projection="3d")
    for q, color, alpha in [
        (scene["start"], "#40789D", 0.33),
        (scene["goal"], "#4F8767", 0.18),
    ]:
        frames = transforms(q, asset)
        for name, balls in asset["spheres"].items():
            x = frames[name]
            for ball in balls:
                sphere(
                    ax,
                    x[:3, :3] @ ball["center"] + x[:3, 3],
                    ball["radius"],
                    color,
                    alpha,
                )
        joints = np.array([frames[f"fr3_link{i}"][:3, 3] for i in range(9)])
        ax.plot(*joints.T, color=color, lw=1.5)
    for center in scene["obstacles"]:
        sphere(ax, np.array(center), scene["obstacle_radius"], "#666666", 0.6)
    replay = []
    for method in ["LSCS", "SCSTO", "NTO_time"]:
        path = case / method / "trajectory_samples.npz"
        if not path.exists():
            continue
        with np.load(path) as arrays:
            q = arrays["q"]
        indices = np.unique(np.linspace(0, len(q) - 1, min(300, len(q))).astype(int))
        tcp = np.array(
            [transforms(point, asset)["fr3_hand_tcp"][:3, 3] for point in q[indices]]
        )
        status = read(case / method / "result.json")["status"]
        passed = status == "success_numerical"
        ax.plot(
            *tcp.T,
            color=COLORS[method] if passed else "#B85252",
            ls="-" if passed else "--",
            lw=1.6,
            label=LABELS[method] + (" (invalid)" if not passed else ""),
        )
        replay.append(
            dict(
                method=method,
                source=str(path.relative_to(run)),
                validated=status,
                plotted_samples=len(indices),
                available_samples=len(q),
            )
        )
    x, y = np.meshgrid([-0.2, 1.2], [-0.8, 0.8])
    ax.plot_surface(x, y, np.zeros_like(x), color="#B2B8BD", alpha=0.15)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_xlim(-0.35, 1.2)
    ax.set_ylim(-0.85, 0.85)
    ax.set_zlim(0, 1.35)
    ax.set_box_aspect([1.55, 1.7, 1.35])
    ax.view_init(24, -122)
    ax.set_title(
        "FR3 sphere-fit reconstruction: first frozen case\nStart / goal geometry and saved TCP trajectories",
        fontsize=10,
    )
    ax.legend(loc="upper left", fontsize=7)
    export(fig, out, "fr3_scene_replay")
    figure_sources["fr3_replay"] = dict(
        case_id=first["id"],
        selection="lexicographically first case before outcome filtering",
        paths=replay,
        scene=str(
            (
                run / "scenes" / f"e{first['environment_seed']:02d}" / "scene.json"
            ).resolve()
        ),
        geometry="33-sphere fit, not original authors geometry; TCP curves illustrate saved joint trajectories",
    )
    atomic(out / "figure_sources.json", figure_sources)
    print("Saved all-case figures and actual FR3 replay to", out)


if __name__ == "__main__":
    main()
