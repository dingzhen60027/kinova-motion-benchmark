"""Recover the Forest example from the author's Figure 6 vector artwork.

This recovers a published illustration, not the unpublished benchmark seeds.
Run with ``bash local/python.sh -m reproduction.environment_reconstruction.forest_reference``.
The existing benchmark configuration and all historical runs remain untouched.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tarfile
import time
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SOURCE_ARCHIVE = ROOT / "reproduction/reports/paper_completion/source_evidence/arxiv_2504.10783v1.tar.gz"
DEFAULT_OUTPUT = ROOT / "reproduction/reports/environment_reconstruction/forest_evidence"
ARCHIVE_SHA256 = "18c4140e37eb05abe636dd4999612618f565c120ce57fbe966e39545b68f1986"
MEMBER = "figures/franka_benchmark_gradient.pdf"
NS = "{http://www.w3.org/2000/svg}"


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def path_points(element):
    # These selected source paths contain only M, L, C and Z. Cubic circle
    # paths have cardinal extrema among their coordinates, so their bounding
    # boxes are exact. This is not a general SVG path parser.
    data = element.attrib["d"]
    commands = set(re.findall("[A-Za-z]", data))
    if not commands <= {"M", "L", "C", "Z"}:
        raise ValueError(f"Unsupported SVG commands: {commands}")
    return np.asarray([float(x) for x in re.findall(r"[-+]?(?:\d*\.\d+|\d+)", data)]).reshape(-1, 2)


def minimum_segment_circle_distances(path, centers, radius):
    """Exact distances for this extracted polygonal artwork, including endpoints."""
    path, centers = np.asarray(path), np.asarray(centers)
    delta = np.diff(path, axis=0)
    length2 = np.sum(delta * delta, axis=1)
    fractions = np.clip(np.sum((centers[None] - path[:-1, None]) * delta[:, None], axis=2)
                        / np.maximum(length2[:, None], 1e-30), 0, 1)
    nearest = path[:-1, None] + fractions[:, :, None] * delta[:, None]
    return np.linalg.norm(nearest - centers[None], axis=2) - radius


def extract(svg):
    root = ET.parse(svg).getroot()
    paths = list(root.iter(NS + "path"))
    borders = [p for p in paths if "stroke-opacity:0.7" in p.attrib.get("style", "")]
    circles = [p for p in paths if "fill:rgb(94.116211%,50.195312%,50.195312%)" in p.attrib.get("style", "")]
    trajectories = [p for p in paths if "fill:none" in p.attrib.get("style", "")
                    and p.attrib["d"].count(" L ") > 50]
    if len(borders) != 1 or len(circles) != 15 or len(trajectories) != 1:
        raise ValueError("Source artwork structure changed; do not infer a different layout silently")
    border, trace = borders[0], trajectories[0]
    expected_transform = border.attrib.get("transform")
    if not expected_transform or any(p.attrib.get("transform") != expected_transform for p in circles + [trace]):
        raise ValueError("Source objects do not share the same affine transform")
    raw_box = path_points(border)
    lo, hi = raw_box.min(axis=0), raw_box.max(axis=0)

    def world(points):
        # The paper gives side length 10 but no coordinate origin. Centering
        # at zero is an explicit harmless coordinate-frame convention.
        return (np.asarray(points) - lo) * 10 / (hi - lo) - 5

    raw_centers, centers, radii = [], [], []
    for circle in circles:
        points = path_points(circle)
        low, high = points.min(axis=0), points.max(axis=0)
        raw_centers.append(((low + high) / 2).tolist())
        centers.append(world((low + high) / 2).tolist())
        radii.append(((high - low) * 5 / (hi - lo)).tolist())
    path = world(path_points(trace))
    stars = [p for p in paths if "stroke-width:2.241528;" in p.attrib.get("style", "")]
    if len(stars) != 1 or stars[0].attrib.get("transform") != expected_transform:
        raise ValueError("Cannot identify goal star")
    star_vertices = path_points(stars[0])
    if len(star_vertices) != 11 or not np.array_equal(star_vertices[0], star_vertices[-1]):
        raise ValueError("Unexpected goal-star structure")
    # Five outer tips of a regular star give its rotational center. Its
    # bounding-box midpoint is vertically displaced and must not set the goal.
    star_center = world(star_vertices[:-1][::2].mean(axis=0))
    scene = {
        "schema_version": 1,
        "scene_id": "forest_paper_figure6_vector",
        "evidence_class": "author_illustration_reconstruction_not_original_benchmark_tasks",
        "source_url": "https://arxiv.org/src/2504.10783v1",
        "source_member": MEMBER,
        "coordinate_convention": "Figure frame mapped affinely to [-5,5]^2, y upwards; original origin unpublished",
        "domain_lower": [-5.0, -5.0],
        "domain_upper": [5.0, 5.0],
        "obstacle_center_lower": [-3.5, -3.5],
        "obstacle_center_upper": [3.5, 3.5],
        "obstacles": 15,
        "obstacle_radius": 0.35,
        "obstacle_centers": centers,
        "centers": centers,
        "start": path[0].tolist(),
        "goal": path[-1].tolist(),
        "reference_path": path.tolist(),
        "reference_path_semantics": "90 vertices from published display polyline; not recovered DRM nodes, spline controls, or timing",
        "point_robot_semantics": "mathematical point in 2D; practical finite-radius adapters must declare their radius",
        "endpoint_semantics": "Endpoints of the published black trajectory; not evidence of endpoints used for every benchmark seed",
        "radius_semantics": "0.35 taken from formal text, not rounded artwork radius",
        "unknowns": ["original environment RNG and seeds", "full 10-environment task list", "original coordinate origin",
                     "all-case endpoint rule", "pre-publication artwork edits", "timing/velocity/acceleration of the displayed trajectory"],
    }
    raw = {
        "shared_svg_transform": expected_transform,
        "raw_frame_lower": lo.tolist(), "raw_frame_upper": hi.tolist(),
        "raw_circle_centers": raw_centers,
        "raw_trace_vertices": path_points(trace).tolist(),
        "normalized_circle_radii_xy": radii,
        "raw_goal_star_vertices": star_vertices.tolist(),
        "goal_star_outer_tip_center": star_center.tolist(),
        "goal_star_all_vertex_center": world(star_vertices[:-1].mean(axis=0)).tolist(),
        "goal_star_bbox_center": world((star_vertices.min(axis=0) + star_vertices.max(axis=0))/2).tolist(),
        "goal_star_center_minus_trace_endpoint_m": (star_center-path[-1]).tolist(),
        "selected_svg_paths": [{"kind": "frame", **border.attrib}]
            + [{"kind": "obstacle", **p.attrib} for p in circles]
            + [{"kind": "trajectory", **trace.attrib}],
    }
    return scene, raw


def search_common_rng_seeds(scene, count=10000):
    """Test only predeclared generators/layouts using geometry, never outcomes."""
    from scipy.optimize import linear_sum_assignment
    centers = np.asarray(scene["obstacle_centers"])
    results = []
    started = time.perf_counter()
    for family in ["default_rng", "RandomState"]:
        for layout in ["15x2", "2x15_transposed"]:
            best = None
            for seed in range(count):
                rng = np.random.default_rng(seed) if family == "default_rng" else np.random.RandomState(seed)
                shape = (15, 2) if layout == "15x2" else (2, 15)
                candidates = rng.uniform(-3.5, 3.5, shape)
                if layout == "2x15_transposed":
                    candidates = candidates.T
                cost = np.sum((candidates[:, None] - centers[None])**2, axis=2)
                rows, columns = linear_sum_assignment(cost)
                rmse = float(np.sqrt(cost[rows, columns].mean()))
                if best is None or rmse < best["matched_center_rmse_m"]:
                    best = {"generator": family, "layout": layout, "seed": seed,
                            "matched_center_rmse_m": rmse,
                            "max_matched_center_error_m": float(np.sqrt(cost[rows, columns].max())),
                            "candidate_to_figure_assignment": columns.tolist()}
            results.append(best)
    return {"scope": "Finite geometry-only hypothesis test; not exhaustive recovery of original RNG",
            "generator_families": ["numpy.default_rng (PCG64)", "numpy.RandomState (MT19937)"],
            "seed_range_inclusive": [0, count-1], "draw_layouts": ["15x2", "2x15 transposed"],
            "sampling": "independent uniform [-3.5,3.5] coordinates; no rejection sampling",
            "assignment": "Hungarian minimum total squared Euclidean center distance",
            "declared_exact_match_max_error_m": .001, "tested_candidates": 4*count,
            "found_exact_match": any(item["max_matched_center_error_m"] <= .001 for item in results),
            "best_per_family_layout": results, "elapsed_seconds": time.perf_counter()-started}


def audit(scene, raw):
    centers, q = np.array(scene["obstacle_centers"]), np.array(scene["reference_path"])
    radii = np.array(raw["normalized_circle_radii_xy"])
    distances = minimum_segment_circle_distances(q, centers, scene["obstacle_radius"])
    straight = minimum_segment_circle_distances(q[[0, -1]], centers, scene["obstacle_radius"])
    # Independent dense samples check the analytic closest-point calculation.
    t = np.linspace(0, 1, 1001)
    dense_minima = []
    for start, end in zip(q[:-1], q[1:]):
        samples = start[None] + t[:, None] * (end - start)[None]
        dense_minima.append(np.min(np.linalg.norm(samples[:, None] - centers[None], axis=2) - .35))
    dense_minima = np.asarray(dense_minima)
    analytic_minima = distances.min(axis=1)
    checks = {
        "exactly_15_circle_paths": len(centers) == 15,
        "all_centers_in_published_central_square": bool(np.all((centers >= -3.5) & (centers <= 3.5))),
        "all_extracted_radii_within_0_0001_of_published_radius": bool(np.max(np.abs(radii - .35)) < 1e-4),
        "source_path_inside_domain": bool(np.all((q >= -5) & (q <= 5))),
        "source_polyline_collision_free_analytic": bool(distances.min() >= 0),
        "dense_samples_respect_analytic_lower_bound": bool(np.min(dense_minima - analytic_minima) >= -1e-12),
        "dense_samples_agree_with_analytic_minima_within_1um": bool(np.max(dense_minima - analytic_minima) < 1e-6),
        "straight_line_is_obstructed": bool(straight.min() < 0),
        "star_rotation_center_agrees_with_trace_endpoint_within_0_0001m": bool(np.linalg.norm(raw["goal_star_center_minus_trace_endpoint_m"]) < 1e-4),
    }
    return {
        "passed": all(checks.values()), "checks": checks,
        "path_vertex_count": len(q), "path_length_m": float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum()),
        "minimum_polyline_clearance_m": float(distances.min()),
        "nearest_segment_and_circle_indices": list(map(int, np.unravel_index(distances.argmin(), distances.shape))),
        "straight_line_minimum_clearance_m": float(straight.min()),
        "endpoint_minimum_clearances_m": (np.linalg.norm(q[[0, -1], None] - centers[None], axis=2).min(axis=1) - .35).tolist(),
        "radius_max_residual_m": float(np.max(np.abs(radii - .35))),
        "radius_xy_max_difference_m": float(np.max(np.abs(radii[:, 0] - radii[:, 1]))),
        "dense_points_checked": int((len(q)-1)*len(t)),
        "dense_minus_analytic_minimum_range_m": [float(np.min(dense_minima-analytic_minima)), float(np.max(dense_minima-analytic_minima))],
        "old_constructed_start_minus_figure_start_m": (np.array([-4., -4.]) - q[0]).tolist(),
        "old_constructed_goal_minus_figure_goal_m": (np.array([4., 3.]) - q[-1]).tolist(),
        "star_center_minus_trace_endpoint_m": raw["goal_star_center_minus_trace_endpoint_m"],
        "star_bbox_center_minus_trace_endpoint_m": (np.asarray(raw["goal_star_bbox_center"])-q[-1]).tolist(),
        "precision_statement": "Observed vector normalization residual is ~4e-5 m; this is not an error bound relative to the unreleased simulation. Artwork may have been edited.",
        "scope": "Geometry of one published example only; no solver, GPU, benchmark or speed result produced",
    }


def plot(scene, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    q = np.asarray(scene["reference_path"])
    fig, ax = plt.subplots(figsize=(7, 7), constrained_layout=True)
    ax.add_patch(Rectangle((-3.5, -3.5), 7, 7, fc="none", ec="#a3aab3", ls="--", lw=1, label="Published obstacle-center support"))
    for index, center in enumerate(scene["obstacle_centers"]):
        ax.add_patch(Circle(center, .35, fc="#ed8080", ec="black", lw=1))
        ax.text(*center, str(index), ha="center", va="center", fontsize=8)
    ax.plot(*q.T, color="black", lw=1.7, label="Extracted display polyline")
    ax.scatter(*q[0], marker="o", s=70, color="black", zorder=3)
    ax.scatter(*q[-1], marker="*", s=220, color="#f1d650", edgecolors="black", zorder=3)
    ax.set(xlim=(-5, 5), ylim=(-5, 5), aspect="equal", xlabel="x [m]", ylabel="y [m]",
           title="Forest: author Figure 6 reconstructed from vector artwork")
    ax.legend(loc="upper left", framealpha=.95, fontsize=8)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-archive", type=Path, default=SOURCE_ARCHIVE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--search-seeds", type=int, default=10000,
                        help="Number of integer seeds tested per generator/layout, default 10000")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if sha256(args.source_archive) != ARCHIVE_SHA256:
        raise ValueError("Unexpected source archive hash")
    pdf, svg = args.output / "author_figure6.pdf", args.output / "author_figure6.svg"
    with tarfile.open(args.source_archive) as archive:
        pdf.write_bytes(archive.extractfile(MEMBER).read())
    commands = [["/usr/bin/pdftocairo", "-svg", str(pdf), str(svg)],
                ["/usr/bin/pdftoppm", "-scale-to", "2000", "-singlefile", "-png", str(pdf), str(args.output / "author_figure6")]]
    executions = []
    for command in commands:
        result = subprocess.run(command, capture_output=True, text=True)
        executions.append({"command": command, "returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr})
        write_json(args.output / "execution.json", executions)
        result.check_returncode()
    scene, raw = extract(svg)
    validation = audit(scene, raw)
    write_json(args.output / "forest_reference_scene.json", scene)
    write_json(args.output / "scene.json", scene)
    write_json(args.output / "vector_extraction.json", raw)
    write_json(args.output / "geometry_audit.json", validation)
    write_json(args.output / "seed_search.json", search_common_rng_seeds(scene, args.search_seeds))
    plot(scene, args.output / "forest_reference_scene.png")
    write_json(args.output / "source_manifest.json", {
        "source_url": "https://arxiv.org/src/2504.10783v1", "archive_path": str(args.source_archive.relative_to(ROOT)),
        "archive_sha256": ARCHIVE_SHA256, "member": MEMBER,
        "script_sha256": sha256(__file__),
        "artifacts": [{"path": str(path.relative_to(args.output)), "size": path.stat().st_size, "sha256": sha256(path)}
                      for path in sorted(args.output.iterdir()) if path.is_file()
                      and path.name != "source_manifest.json" and path.suffix != ".log"],
    })
    print(json.dumps(validation, indent=2))
    if not validation["passed"]:
        raise SystemExit("Figure reconstruction audit failed; retain diagnostics")


if __name__ == "__main__":
    main()
