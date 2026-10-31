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
from matplotlib.patches import Patch, Circle
from scipy.spatial.transform import Rotation
from reproduction.common import atomic, read
from reproduction.report_constructed import verify_completed_grid, verify_geometry_evidence

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


def trajectory_rows(path):
    """Select actual ordered trajectory samples, excluding appended witnesses."""
    with np.load(path) as arrays:
        q, times = arrays["q"], arrays["time"]
    if len(q) != len(times):
        raise ValueError("Trajectory/time row count mismatch: " + str(path))
    keep = np.isfinite(times)
    return q[keep], dict(
        raw_rows=len(q), plotted_trajectory_rows=int(keep.sum()),
        excluded_nonfinite_time_witness_rows=int((~keep).sum()),
        selection="finite saved times only; appended collision witnesses are not trajectory edges")


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
    manifests = {}
    for env, summary in summaries.items():
        audit_path = base / "audit" / env / "audit.json"
        manifest = verify_completed_grid(summary, read(audit_path))
        manifests[env] = dict(manifest, run_dir=summary["run_dir"])
        expected_ids = sorted(entry["id"] for entry in manifest["cases"])
        for method in [r["method"] for r in summary["methods"]]:
            actual_ids = sorted(data[env].loc[data[env].method == method, "id"].tolist())
            if actual_ids != expected_ids:
                raise ValueError(f"Missing or duplicate plotted cases: {env}/{method}")
    geometry_index, fitting, reference = verify_geometry_evidence(base, manifests["fr3"])
    figure_sources = dict(
        source_files={},
        selection="all400requested cases per environment; lexicographically first case per environment for scene replay; FR3 TCP replay displays LSCS,SCSTO,NTO_time only",
        hardware="i7-13700K / Quadro RTX 8000; paper hardware differs",
        guarantee="independent numerical validation of the declared sphere model, not continuous or official nonconvex-mesh trajectory certification; separate official-geometry audit uses1184fixed poses",
        fidelity="published parameters; reconstructed FR3 sphere fit and unpublished random inputs",
        timing="all completed timing records including failures; interrupted attempts without completed timing excluded, not zero; fixed method order shares240second case budget; construction + optimization includes alternate-roadmap IO/validation and recovery coverage writes; final-candidate validation and surrounding pre-solve/replay writes excluded; caught exceptions retain flagged method-wall-time fallbacks",
        paired_cost_display="all common validated observations; median and interquartile range, no iid confidence interval",
        outcome_bars="solver success, solver-success AND CUDA-free, and strict validated success; CUDA CFR without the solver conjunction is a separate report/CSV metric",
    )
    for env in data:
        for p in [*(base / env / filename for filename in ["methods.csv", "paired.csv", "summary.json"]),
                  base / "audit" / env / "audit.json"]:
            figure_sources["source_files"][str(p.resolve())] = hashlib.sha256(
                p.read_bytes()
            ).hexdigest()
    fig, axes = plt.subplots(
        3, 2, figsize=(7.2, 9.2), gridspec_kw={"hspace": 0.72, "wspace": 0.48}
    )
    for column, env in enumerate(["forest", "fr3"]):
        summary = summaries[env]
        methods = [r["method"] for r in summary["methods"]]
        ax = axes[0, column]
        panel(ax, "ab"[column], env.upper() + " outcome checks")
        for offset, field, label, color in [
            (-0.23, "solver_successes", "Solver success", "#A1A5AA"),
            (0, "native_cuda_validated_successes", "Solver + CUDA free", "#40789D"),
            (0.23, "validated_successes", "Strict independent validation", "#4F8767"),
        ]:
            rates = [r[field] / r["requested"] for r in summary["methods"]]
            ax.barh(np.arange(len(methods)) + offset, rates,
                    color=color, height=0.2, label=label)
        ax.set_yticks(range(len(methods)), [LABELS[m] for m in methods])
        ax.invert_yaxis()
        ax.set_xlim(0, 1.22)
        ax.set_xticks([0, 0.5, 1])
        ax.set_xlabel(f"Fraction of all {summary['requested']} requested cases")
        ax.legend(loc="upper left", bbox_to_anchor=(-0.02, -0.22), fontsize=6)
        for i, r in enumerate(summary["methods"]):
            ax.text(
                max(r[k] for k in ["solver_successes", "native_cuda_validated_successes", "validated_successes"]) / r["requested"] + 0.02,
                i,
                f"{r['validated_successes']} strict",
                va="center",
                fontsize=7,
            )
        ax.grid(axis="x", color="#E2E4E6", lw=0.5)
        ax.set_axisbelow(True)
        ax = axes[1, column]
        panel(ax, "cd"[column], env.upper() + " completed timing records")
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
    fig.text(0.5, 0.01,
             "FR3 uses a geometry-audited constructed 33-sphere model.\n"
             "Fixed inputs; interrupted timings are excluded, and later methods can be budget-censored.",
             ha="center", fontsize=7)
    export(fig, out, "paper_results")
    # Full case grids retain every failure and every absent solver output.
    classes = [
        "Validated success",
        "Failed validation",
        "Solver failure",
        "Invalid DRM seed",
        "License blocked",
        "Input / IK / DRM unavailable",
        "Interrupted / other error",
        "Not started after timeout",
    ]
    colors = [
        "#4F8767",
        "#D9A054",
        "#9B6E73",
        "#A8ADB4",
        "#7364A1",
        "#D2D6DB",
        "#433C46",
        "#C9B7D8",
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
                                        or status.startswith("not_run_due_to_ik")
                                        else (7 if status == "not_run_due_to_timeout" else 6)
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
    fig.text(0.5, 0.015, "FR3: constructed geometry and fixed random tasks; all failures retained.",
             ha="center", fontsize=7)
    export(fig, out, "paper_case_statuses")
    # Geometry illustration uses an ID rule, never a success-based selection.
    run = Path(summaries["fr3"]["run_dir"])
    manifest = read(run / "manifest.json")
    first = sorted(manifest["cases"], key=lambda r: r["id"])[0]
    case = run / "cases" / first["id"]
    scene = read(run / "scenes" / f"e{first['environment_seed']:02d}" / "scene.json")
    case_record = read(case / "case.json")
    inputs = read(case / "input.json") if (case / "input.json").exists() else {}
    asset_name = "reproduction/paper_constructed/assets/fr3_33.json"
    asset_path = run / "source_snapshot" / asset_name
    if hashlib.sha256(asset_path.read_bytes()).hexdigest() != manifest["source_files"][asset_name]:
        raise ValueError("Frozen FR3 geometry differs from the measured source")
    asset = read(asset_path)
    figure_sources["source_files"][str(asset_path.resolve())] = hashlib.sha256(asset_path.read_bytes()).hexdigest()
    fig = plt.figure(figsize=(10.2, 5.2))
    forest_run = Path(summaries["forest"]["run_dir"])
    forest_manifest = read(forest_run / "manifest.json")
    forest_first = sorted(forest_manifest["cases"], key=lambda r: r["id"])[0]
    forest_case = forest_run / "cases" / forest_first["id"]
    forest_input_path = forest_case / "input.json"
    forest_input = read(forest_input_path) if forest_input_path.exists() else {}
    forest_cfg = read(forest_run / "source_snapshot/reproduction/configs/forest.json")
    for source in [forest_case / "case.json", case / "case.json",
                   forest_run / "source_snapshot/reproduction/configs/forest.json",
                   forest_run / "manifest.json", run / "manifest.json",
                   run / "scenes" / f"e{first['environment_seed']:02d}" / "scene.json"]:
        figure_sources["source_files"][str(source.resolve())] = hashlib.sha256(source.read_bytes()).hexdigest()
    if forest_input_path.exists():
        figure_sources["source_files"][str(forest_input_path.resolve())] = hashlib.sha256(forest_input_path.read_bytes()).hexdigest()
    if (case / "input.json").exists():
        figure_sources["source_files"][str((case / "input.json").resolve())] = hashlib.sha256((case / "input.json").read_bytes()).hexdigest()
    forest_ax = fig.add_subplot(121)
    for center in forest_input.get("centers", []):
        forest_ax.add_patch(Circle(center, forest_cfg["obstacle_radius"],
                                   color="#A1A5AA", alpha=0.65))
    forest_replay = []
    for method in LABELS:
        path = forest_case / method / "trajectory_samples.npz"
        if not path.exists():
            continue
        q, selection = trajectory_rows(path)
        figure_sources["source_files"][str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
        result = read(forest_case / method / "result.json")
        result_path = forest_case / method / "result.json"
        figure_sources["source_files"][str(result_path.resolve())] = hashlib.sha256(result_path.read_bytes()).hexdigest()
        passed = result["status"] == "success_numerical"
        failure_label = {"optimization_failed": "solver failed", "validation_failed": "validation failed"}.get(result["status"], "unaccepted")
        forest_ax.plot(q[:, 0], q[:, 1], color=COLORS[method],
                       ls="-" if passed else "--", lw=1.3,
                       label=LABELS[method] + ("" if passed else " (" + failure_label + ")"))
        forest_replay.append(dict(method=method, status=result["status"],
                                  source=str(path.resolve()), samples=len(q),
                                  trajectory_selection=selection))
    for name, marker in [("start", "o"), ("goal", "*")]:
        if name not in forest_input:
            continue
        point = forest_input[name]
        forest_ax.scatter(*point, marker=marker, color="#333333", s=30, zorder=8, label=name.capitalize())
    forest_ax.set(xlim=[forest_cfg["domain_lower"][0], forest_cfg["domain_upper"][0]],
                  ylim=[forest_cfg["domain_lower"][1], forest_cfg["domain_upper"][1]],
                  xlabel="x (m)", ylabel="y (m)")
    forest_ax.set_aspect("equal")
    forest_ax.set_title("Forest: first frozen case\n" + forest_first["id"], fontsize=10)
    if forest_input:
        forest_ax.legend(loc="upper left", fontsize=7)
    else:
        forest_ax.text(.5, .5, "Input unavailable for this failed case", transform=forest_ax.transAxes,
                       ha="center", fontsize=8)
    figure_sources["forest_replay"] = dict(case_id=forest_first["id"], paths=forest_replay,
                                           input=str((forest_case/"input.json").resolve()))
    ax = fig.add_subplot(122, projection="3d")
    configurations = [(scene["start"], "#40789D", 0.33)] if "start" in scene else []
    if "goal" in inputs:
        configurations.append((inputs["goal"], "#4F8767", 0.18))
    for q, color, alpha in configurations:
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
        q, selection = trajectory_rows(path)
        figure_sources["source_files"][str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
        indices = np.unique(np.linspace(0, len(q) - 1, min(300, len(q))).astype(int))
        tcp = np.array(
            [transforms(point, asset)["fr3_hand_tcp"][:3, 3] for point in q[indices]]
        )
        status = read(case / method / "result.json")["status"]
        result_path = case / method / "result.json"
        figure_sources["source_files"][str(result_path.resolve())] = hashlib.sha256(result_path.read_bytes()).hexdigest()
        passed = status == "success_numerical"
        failure_label = {"optimization_failed": "solver failed", "validation_failed": "validation failed"}.get(status, "unaccepted")
        ax.plot(
            *tcp.T,
            color=COLORS[method] if passed else "#B85252",
            ls="-" if passed else "--",
            lw=1.6,
            label=LABELS[method] + (" (" + failure_label + ")" if not passed else ""),
        )
        replay.append(
            dict(
                method=method,
                source=str(path.relative_to(run)),
                validated=status,
                plotted_samples=len(indices),
                available_samples=len(q),
                trajectory_selection=selection,
            )
        )
    table = manifest["config"]["fr3_scene"]
    center, size = np.asarray(table["table_center"]), np.asarray(table["table_size"])
    x, y = np.meshgrid([center[0] - size[0]/2, center[0] + size[0]/2],
                       [center[1] - size[1]/2, center[1] + size[1]/2])
    ax.plot_surface(x, y, np.full_like(x, center[2] + size[2]/2), color="#B2B8BD", alpha=0.15)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_xlim(-0.35, 1.2)
    ax.set_ylim(-0.85, 0.85)
    ax.set_zlim(0, 1.35)
    ax.set_box_aspect([1.55, 1.7, 1.35])
    ax.view_init(24, -122)
    ax.set_title(
        "FR3 sphere-fit reconstruction: first frozen case\nCase status: " + case_record["status"],
        fontsize=10,
    )
    if replay:
        ax.legend(loc="upper left", fontsize=7)
    else:
        ax.text2D(0.02, 0.94, "No saved trajectory for this case", transform=ax.transAxes, fontsize=8)
    geometry_legend = ([Patch(facecolor="#40789D", label="Start configuration")]
                       if "start" in scene else [])
    if "goal" in inputs:
        geometry_legend.append(Patch(facecolor="#4F8767", label="IK goal configuration"))
    geometry_legend.append(Patch(facecolor="#666666", label="Obstacles"))
    ax.add_artist(ax.legend(handles=geometry_legend, loc="lower center",
                            bbox_to_anchor=(0.5, -0.06), ncol=3, fontsize=6.5))
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
    geometry_path = base / geometry_index["construction"] / "geometry_audit.json"
    reference_path = base / geometry_index["geometry_validation"] / "summary.json"
    geometry = read(geometry_path)
    links = list(geometry["selected"])
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.5), gridspec_kw={"wspace": .5})
    panel(axes[0], "a", "Geometric overreach on every link")
    for i, name in enumerate(links):
        values = [1000 * geometry[k][name]["sphere_surface_outward_max_m"]
                  for k in ["baseline", "selected"]]
        axes[0].plot(values, [i, i], color="#BEC4C7", linewidth=1)
        axes[0].scatter(values[0], i, s=15, color="#8D6C65", label="Old model" if i == 0 else None)
        axes[0].scatter(values[1], i, s=15, color="#40789D", label="Constructed model" if i == 0 else None)
    axes[0].set_yticks(range(len(links)), [x.replace("fr3_", "") for x in links])
    axes[0].invert_yaxis()
    axes[0].set_xlabel("Maximum sampled exterior distance (mm)")
    axes[0].legend(loc="lower right", fontsize=6)
    panel(axes[1], "b", "Independent official-geometry comparison")
    conditions = ["static_table", "online_20_spheres"]
    labels = ["Static table", "+ 20 obstacles"]
    bottom = np.zeros(2)
    categories = [("both_free", "Both free", "#93B0C0"),
                  ("both_collide", "Both collide", "#AAB5AF"),
                  ("sphere_false_positive", "Sphere-only collision", "#AC785B"),
                  ("reference_false_negative_candidate", "Hull-only collision*", "#766B83")]
    for key, label, color in categories:
        values = [reference["conditions"][c]["summary"]["all"]["counts"].get(key, 0) for c in conditions]
        axes[1].bar(labels, values, bottom=bottom, color=color, label=label, width=.58)
        bottom += values
    failures = np.asarray([reference["conditions"][c]["failures"] for c in conditions])
    if failures.any():
        axes[1].bar(labels, failures, bottom=bottom, color="#433C46", label="Query failed", width=.58)
        bottom += failures
    if list(bottom.astype(int)) != [reference["conditions"][c]["requested"] for c in conditions]:
        raise ValueError("Geometry plot does not account for every preregistered configuration")
    axes[1].set_ylabel("All preregistered configurations (count)")
    axes[1].set_ylim(0, max(bottom) * 1.12)
    for i, total in enumerate(bottom):
        axes[1].text(i, total + max(bottom) * .02, f"n={int(total)}", ha="center", fontsize=7)
    axes[1].legend(loc="upper center", bbox_to_anchor=(.5, -.16), ncol=2, fontsize=6)
    fig.subplots_adjust(bottom=.28)
    fig.text(.5, .015, "* Hull penetration may fill mesh concavities; surface coverage is not a continuous solid-volume certificate.",
             ha="center", fontsize=6)
    export(fig, out, "fr3_model_diagnostic")
    figure_sources["model_diagnostic"] = dict(
        source_files={str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in [geometry_path, reference_path]},
        links=len(links), conditions={c: reference["conditions"][c]["requested"] for c in conditions},
        selection="all11links and all1184preregistered configurations in both conditions",
        scope="sampled geometric approximation error and independent convex-hull comparison; no continuous solid certificate")
    atomic(out / "figure_sources.json", figure_sources)
    print("Saved all-case figures and actual FR3 replay to", out)


if __name__ == "__main__":
    main()
