"""Read-only dataset indexing, integrity checks and feature export.

The original manifests, scenes and input arrays remain the authoritative frozen
inputs. The compact index references them and never resamples or rewrites them.
No collision library or GPU is loaded by this module.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import io
import json
from pathlib import Path
import os
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

from .outcomes import CANDIDATE_STATUSES, REQUIRED_FILES, assess_saved_output

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[1]
SPLITS = ("smoke", "dev", "test")
PRODUCTS = ("problems.jsonl", "splits.json", "features.csv", "metadata.json")
SUCCESS = {"success_validated_numerical", "success_numerical"}
FEATURE_COLUMNS = (
    "problem_id",
    "scene_id",
    "environment_cluster",
    "split",
    "category",
    "scene_seed",
    "query_seed",
    "distance_rad",
    "normalized_distance",
    "distance_bin",
    "endpoints_valid",
    "input_status",
    "straight_line_collision",
    "straight_line_step_rad",
    "obstacle_count",
    "voxel_count",
    "voxel_radius_m",
    "feasibility",
    "witness_kind",
    "witness_result",
    "input_failure_reason",
    "endpoint_drawn_configurations",
    "endpoint_collision_rejections",
    "endpoint_distance_rejections",
)


def read(path):
    def reject(value):
        raise ValueError(f"Nonfinite JSON value: {value}")

    return json.loads(Path(path).read_text(), parse_constant=reject)


def digest(value):
    return hashlib.sha256(
        json.dumps(
            value, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode()
    ).hexdigest()


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for data in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(data)
    return h.hexdigest()


def safe_path(base, reference):
    relative = Path(reference)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"Expected a relative dataset reference: {reference}")
    path = (Path(base) / relative).resolve()
    if not path.is_relative_to(Path(base).resolve()):
        raise ValueError(f"Reference escapes dataset root: {reference}")
    return path


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def source_records(base=BASE):
    """Return (original row, manifest reference, one-based line) in split order."""
    base = Path(base)
    records, identities = [], set()
    for split in SPLITS:
        reference = f"manifests/{split}.jsonl"
        for line_number, line in enumerate(
            (base / reference).read_text().splitlines(), 1
        ):
            if not line.strip():
                continue
            row = json.loads(line)
            identity = row["problem_id"]
            _require(identity not in identities, f"Duplicate problem: {identity}")
            _require(row["split"] == split, f"Wrong split: {identity}")
            identities.add(identity)
            records.append((row, reference, line_number))
    return records


def _check_saved_arrays(row, base, config, robot):
    """Replay seeded proposals and saved decisions, without collision/FK calls."""
    with np.load(
        safe_path(base, row["input_arrays_file"]), allow_pickle=False
    ) as arrays:
        raw = {key: arrays[key] for key in arrays.files}
    identity = row["problem_id"]
    q = raw["endpoint_proposal_q"]
    free = raw["endpoint_proposal_cuda_free"]
    clearance = raw["endpoint_proposal_independent_clearance_m"]
    low, high = np.asarray(robot["lower"]), np.asarray(robot["upper"])
    margin = config["endpoint_joint_margin_rad"]
    _require(
        q.ndim == 2
        and q.shape[1] == 7
        and 0 < len(q) <= config["endpoint_max_attempts"],
        f"Invalid proposal array: {identity}",
    )
    expected = np.random.Generator(np.random.PCG64(row["endpoint_seed"])).uniform(
        low + margin, high - margin, q.shape
    )
    _require(np.array_equal(q, expected), f"Seeded proposals differ: {identity}")
    _require(
        free.shape == clearance.shape == (len(q),)
        and free.dtype.kind == "b"
        and np.isfinite(clearance).all(),
        f"Malformed proposal observations: {identity}",
    )
    _require(
        not np.any(
            (free != (clearance > 0)) & (np.abs(clearance) > robot["contact_band_m"])
        ),
        f"Saved checker disagreement: {identity}",
    )
    start = goal = None
    rejected_collision = rejected_distance = 0
    lo, hi = row["endpoint_sampling"]["distance_bin_bounds"]
    for considered, (state, is_free, distance) in enumerate(zip(q, free, clearance), 1):
        if not is_free or distance <= config["endpoint_clearance_m"]:
            rejected_collision += 1
        elif start is None:
            start = state
        elif lo <= np.sqrt(np.mean(((state - start) / (high - low)) ** 2)) < hi:
            goal = state
            break
        else:
            rejected_distance += 1
    for key, accepted in (("q_start", start), ("q_goal", goal)):
        _require(
            (accepted is None and row[key] is None)
            or (accepted is not None and np.array_equal(accepted, row[key])),
            f"First accepted endpoint differs: {identity}/{key}",
        )
    sampling = row["endpoint_sampling"]
    observed = dict(
        drawn_configurations=len(q),
        considered_configurations=considered,
        unused_batch_tail_configurations=len(q) - considered,
        collision_or_clearance_rejections=rejected_collision,
        distance_bin_rejections=rejected_distance,
    )
    _require(
        all(sampling[k] == v for k, v in observed.items()),
        f"Rejection counters differ: {identity}",
    )
    expected_draws = min(
        config["endpoint_max_attempts"],
        ((considered - 1) // config["endpoint_batch_size"] + 1)
        * config["endpoint_batch_size"],
    )
    _require(
        len(q) == expected_draws
        and (goal is not None or len(q) == config["endpoint_max_attempts"]),
        f"Truncated generation: {identity}",
    )
    if goal is not None:
        n = max(
            2,
            int(
                np.ceil(np.linalg.norm(goal - start) / config["straight_line_step_rad"])
            )
            + 1,
        )
        line = raw["straight_line_q"]
        flags = raw["straight_line_cuda_free"]
        distances = raw["straight_line_independent_clearance_m"]
        _require(
            np.array_equal(line, np.linspace(start, goal, n)),
            f"Straight-line interpolation differs: {identity}",
        )
        _require(
            flags.shape == distances.shape == (n,)
            and flags.dtype.kind == "b"
            and np.isfinite(distances).all(),
            f"Straight-line observations malformed: {identity}",
        )
        _require(
            not np.any(
                (flags != (distances > 0))
                & (np.abs(distances) > robot["contact_band_m"])
            ),
            f"Saved straight-line checker disagreement: {identity}",
        )
        _require(
            row["straight_line_collision"]
            is bool(not (flags.all() and (distances > 0).all())),
            f"Straight-line classification differs: {identity}",
        )
        _require(
            np.allclose(
                distances[[0, -1]], row["endpoint_clearances_m"], atol=1e-10, rtol=0
            ),
            f"Saved endpoint clearance differs: {identity}",
        )
    else:
        _require(
            row["straight_line_collision"] is None,
            f"Missing endpoints have straight-line classification: {identity}",
        )


def check_sources(base=BASE, repo_root=ROOT):
    """Verify frozen bytes, model references, endpoint arrays and split independence.

    This is a CPU integrity check. It does not repeat the existing native/Drake
    collision audit; the latter is linked in metadata and run via the audit CLI.
    """
    base, repo_root = Path(base), Path(repo_root)
    records = source_records(base)
    from .generator.integrity import preflight

    robot = read(base / "configs/robot.json")
    configs = {
        split: read(base / f"configs/{split}_generator.json") for split in SPLITS
    }
    for name, expected in robot["source_files_sha256"].items():
        _require(
            sha(safe_path(repo_root, name)) == expected, f"Robot source changed: {name}"
        )
    scenes, checked_files = {}, set()
    geometry_splits, endpoint_splits, scene_seeds, query_seeds = {}, {}, {}, set()
    for split in SPLITS:
        receipt = read(base / f"receipts/{split}_generation.json")
        _require(
            sha(base / f"manifests/{split}.jsonl") == receipt["manifest_sha256"],
            f"Frozen {split} manifest changed",
        )
        _require(
            sha(base / f"configs/{split}_generator.json")
            == receipt["protocol"]["config_sha256"],
            f"Frozen {split} generator config changed",
        )
        preflight(
            base / f"manifests/{split}.jsonl",
            [row for row, _, _ in records if row["split"] == split],
            configs[split],
            receipt,
            sha,
        )
        for reference, expected in receipt["files_sha256"].items():
            _require(
                sha(safe_path(base, reference)) == expected,
                f"Frozen file changed: {reference}",
            )
            checked_files.add(reference)
    for row, _, _ in records:
        identity = row["problem_id"]
        unhashed = {k: v for k, v in row.items() if k != "problem_sha256"}
        _require(
            digest(unhashed) == row["problem_sha256"],
            f"Problem hash mismatch: {identity}",
        )
        for key in ("robot_sha256", "collision_geometry_sha256"):
            _require(row[key] == robot[key], f"Model mismatch: {identity}/{key}")
        path = safe_path(base, row["scene_file"])
        _require(sha(path) == row["scene_sha256"], f"Scene hash mismatch: {identity}")
        scene = scenes.setdefault(row["scene_id"], read(path))
        for key in (
            "scene_id",
            "split",
            "family",
            "scene_seed",
            "obstacle_geometry",
            "voxels_xyz",
            "voxel_radius_m",
            "robot_sha256",
            "collision_geometry_sha256",
        ):
            _require(
                row[key] == scene[key], f"Scene reference mismatch: {identity}/{key}"
            )
        _require(
            digest(row["obstacle_geometry"]) == row["obstacle_geometry_sha256"],
            f"Obstacle hash mismatch: {identity}",
        )
        _require(
            digest(dict(xyz=row["voxels_xyz"], radius=row["voxel_radius_m"]))
            == row["voxel_geometry_sha256"],
            f"Voxel hash mismatch: {identity}",
        )
        arrays_path = safe_path(base, row["input_arrays_file"])
        _require(
            sha(arrays_path) == row["input_arrays_sha256"],
            f"Input hash mismatch: {identity}",
        )
        _check_saved_arrays(row, base, configs[row["split"]], robot)
        if row["family"] != "open_control":
            previous = geometry_splits.setdefault(
                row["obstacle_geometry_sha256"], row["split"]
            )
            _require(
                previous == row["split"],
                f"Random geometry shared across splits: {identity}",
            )
        previous = scene_seeds.setdefault(row["scene_seed"], row["scene_id"])
        _require(previous == row["scene_id"], f"Scene seed reused: {identity}")
        _require(
            row["endpoint_seed"] not in query_seeds, f"Query seed reused: {identity}"
        )
        query_seeds.add(row["endpoint_seed"])
        if row["validation_status"] == "valid_input":
            q = np.asarray([row["q_start"], row["q_goal"]], dtype=float)
            _require(
                q.shape == (2, 7) and np.isfinite(q).all(),
                f"Invalid endpoints: {identity}",
            )
            low, high = np.asarray(robot["lower"]), np.asarray(robot["upper"])
            _require(
                np.all(q >= low) and np.all(q <= high),
                f"Joint limit violation: {identity}",
            )
            distance = float(np.linalg.norm(q[1] - q[0]))
            normalized = float(np.sqrt(np.mean(((q[1] - q[0]) / (high - low)) ** 2)))
            _require(
                abs(distance - row["start_goal_joint_distance"]) < 1e-12,
                f"Distance mismatch: {identity}",
            )
            _require(
                abs(normalized - row["normalized_start_goal_joint_distance"]) < 1e-12,
                f"Normalized distance mismatch: {identity}",
            )
            for endpoint in q:
                previous = endpoint_splits.setdefault(
                    digest(endpoint.tolist()), row["split"]
                )
                _require(
                    previous == row["split"],
                    f"Endpoint shared across splits: {identity}",
                )
        else:
            _require(
                row["q_start"] is None
                or row["q_goal"] is None
                or row["validation_status"] == "invalid_scene_geometry",
                f"Unexplained invalid input: {identity}",
            )
    return dict(
        passed=True,
        registered=len(records),
        valid=sum(r["validation_status"] == "valid_input" for r, _, _ in records),
        scenes=len(scenes),
        frozen_files_checked=len(checked_files),
        collision_rechecked=False,
    )


def numerical_witness(result_path, row, base=BASE, *, evidence_cache=None):
    """Accept only an input-bound, receipt-bound independently validated artifact.

    This imports a past numerical witness; it neither trusts solver success
    alone nor repeats collision checking or claims continuous certification.
    """
    path, base = Path(result_path), Path(base)
    try:
        assessment = assess_saved_output(path.parent, case=row, evidence_cache=evidence_cache)
        if not assessment["accepted"]:
            return None
        result = read(path)
        receipt = read(path.parent / "receipt.json")["files"]
        bound = {str((path.parent / name).relative_to(base)): receipt[name]
                 for name in sorted(REQUIRED_FILES)}
        run_manifest = path.parents[4] / "run_manifest.json"
        bound[str(run_manifest.relative_to(base))] = sha(run_manifest)
        return dict(
            kind="validated_planner_trajectory", result=str(path.relative_to(base)),
            method=result.get("method"), objective=result.get("objective"),
            files_sha256=bound,
            receipt_file=str((path.parent / "receipt.json").relative_to(base)),
            receipt_sha256=sha(path.parent / "receipt.json"),
            certification="finite_numerical_sampling", raw_status=result.get("status"),
            status_basis=assessment["status_basis"],
        )
    except (OSError, ValueError, KeyError, TypeError):
        return None


def collect_witnesses(rows, base=BASE):
    base = Path(base)
    lookup = {
        r["problem_id"]: r for r in rows if r["validation_status"] == "valid_input"
    }
    witnesses = {}
    evidence_cache = {}
    stats = Counter()
    for path in sorted((base / "results").rglob("result.json")):
        try:
            result = read(path)
        except (ValueError, OSError):
            stats["unreadable_result_records"] += 1
            continue
        stats["result_records_scanned"] += 1
        if not isinstance(result, dict):
            stats["unreadable_result_records"] += 1
            continue
        identity = result.get("problem_id")
        if not isinstance(identity, str) or not isinstance(result.get("status"), str):
            stats["unreadable_result_records"] += 1
            continue
        if (
            identity not in lookup
            or identity in witnesses
            or result.get("status") not in CANDIDATE_STATUSES
        ):
            continue
        evidence = numerical_witness(path, lookup[identity], base, evidence_cache=evidence_cache)
        if evidence:
            witnesses[identity] = evidence
        else:
            stats["unaccepted_saved_output_records"] += 1
    return witnesses, dict(stats)


def _feature(row, witness):
    valid = row["validation_status"] == "valid_input"
    sampling = row.get("endpoint_sampling", {})
    failure = None
    if not valid:
        failure = (
            "invalid_scene_geometry"
            if row["validation_status"] == "invalid_scene_geometry"
            else (
                "no_legal_start_within_generation_budget"
                if row["q_start"] is None
                else "no_legal_goal_within_generation_budget"
            )
        )
    # Generation's coarser line classification is a feature, not a substitute
    # for the common trajectory validator's numerical feasibility witness.
    kind = witness["kind"] if witness else None
    return dict(
        problem_id=row["problem_id"],
        scene_id=row["scene_id"],
        environment_cluster="geometry:" + row["obstacle_geometry_sha256"],
        split=row["split"],
        category=row["family"],
        scene_seed=row["scene_seed"],
        query_seed=row["endpoint_seed"],
        distance_rad=row.get("start_goal_joint_distance"),
        normalized_distance=row.get("normalized_start_goal_joint_distance"),
        distance_bin=sampling.get("distance_bin"),
        endpoints_valid=valid,
        input_status="valid_input" if valid else "generation_failed",
        straight_line_collision=row.get("straight_line_collision"),
        straight_line_step_rad=row.get("straight_line_step_rad"),
        obstacle_count=len(row["obstacle_geometry"]),
        voxel_count=len(row["voxels_xyz"]),
        voxel_radius_m=row["voxel_radius_m"],
        feasibility=(
            "numerical_path_witness"
            if kind
            else ("unknown_feasibility" if valid else "invalid_input")
        ),
        witness_kind=kind,
        witness_result=witness["result"] if witness else None,
        input_failure_reason=failure,
        endpoint_drawn_configurations=sampling.get("drawn_configurations"),
        endpoint_collision_rejections=sampling.get("collision_or_clearance_rejections"),
        endpoint_distance_rejections=sampling.get("distance_bin_rejections"),
    )


def _model_metadata(base, repo_root):
    robot = read(base / "configs/robot.json")
    urdf = ET.parse(repo_root / "csdecomp/tests/test_assets/kinova_sens.urdf").getroot()
    active = [j for j in urdf.findall("joint") if j.get("type") != "fixed"]
    _require(
        len(active) == 7 and all(j.get("type") == "revolute" for j in active),
        "Canonical model no longer has seven bounded revolute joints",
    )
    keys = (
        "robot_sha256",
        "collision_geometry_sha256",
        "canonical_directive",
        "joint_names",
        "lower",
        "upper",
        "base_transform_world",
        "tcp",
        "world_frame",
        "filter_policy",
        "contact_band_m",
        "independent_checker",
        "source_files_sha256",
    )
    return dict(
        **{key: robot[key] for key in keys},
        inventory_file="configs/robot.json",
        inventory_sha256=sha(base / "configs/robot.json"),
        joint_types=[j.get("type") for j in active],
        periodic_joints=[False] * 7,
        joint_metric="Euclidean L2 in bounded R7, radians; no angular wrapping or endpoint normalization",
        gripper="Robotiq 2F85 welded finger URDF; no additional active gripper joints",
        fixed_environment="Table, two camera/tripod assemblies and protective box in the canonical directive",
        checked_collision_pair_count=len(robot["checked_pairs"]),
        excluded_collision_pair_count=len(robot["excluded_pairs"]),
        robot_geometry_count=len(robot["robot_geometry_ids"]),
        scene_geometry_count=len(robot["geometries"]),
    )


def _atomic(path, text):
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(text)
    os.replace(tmp, path)


def build(base=BASE, repo_root=ROOT):
    base, repo_root = Path(base).resolve(), Path(repo_root).resolve()
    source_check = check_sources(base, repo_root)
    records = source_records(base)
    witnesses, witness_scan = collect_witnesses([r for r, _, _ in records], base)
    index, features = [], []
    split_index = {
        s: {
            "problem_ids": [],
            "valid_problem_ids": [],
            "generation_failed_problem_ids": [],
        }
        for s in SPLITS
    }
    source_hashes = {
        reference: sha(base / reference)
        for reference in ("configs/robot.json", "configs/generator.json")
    }
    for row, manifest, line in records:
        witness = witnesses.get(row["problem_id"])
        feature = _feature(row, witness)
        features.append(feature)
        index.append(
            dict(
                **feature,
                problem_sha256=row["problem_sha256"],
                source_manifest=manifest,
                source_line=line,
                scene_file=row["scene_file"],
                scene_sha256=row["scene_sha256"],
                input_arrays_file=row["input_arrays_file"],
                input_arrays_sha256=row["input_arrays_sha256"],
                robot_sha256=row["robot_sha256"],
                collision_geometry_sha256=row["collision_geometry_sha256"],
                paired_planner_seeds=row["paired_seeds"],
                witness=witness,
            )
        )
        split = split_index[row["split"]]
        split["problem_ids"].append(row["problem_id"])
        split[
            (
                "valid_problem_ids"
                if feature["endpoints_valid"]
                else "generation_failed_problem_ids"
            )
        ].append(row["problem_id"])
        for reference in (manifest, row["scene_file"], row["input_arrays_file"]):
            if reference not in source_hashes:
                source_hashes[reference] = sha(base / reference)
    for split in SPLITS:
        for reference in (
            f"receipts/{split}_generation.json",
            f"configs/{split}_generator.json",
        ):
            source_hashes[reference] = sha(base / reference)
    text_json = (
        lambda value: json.dumps(value, indent=2, sort_keys=True, allow_nan=False)
        + "\n"
    )
    _atomic(
        base / "problems.jsonl",
        "".join(
            json.dumps(r, sort_keys=True, allow_nan=False, separators=(",", ":")) + "\n"
            for r in index
        ),
    )
    _atomic(base / "splits.json", text_json(split_index))
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=FEATURE_COLUMNS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(features)
    _atomic(base / "features.csv", stream.getvalue())
    counts = {}
    for split in SPLITS:
        subset = [r for r in features if r["split"] == split]
        valid = [r for r in subset if r["endpoints_valid"]]
        counts[split] = dict(
            registered=len(subset),
            valid=len(valid),
            generation_failed=len(subset) - len(valid),
            scene_ids=len({r["scene_id"] for r in subset}),
            valid_environment_clusters=len({r["environment_cluster"] for r in valid}),
            category_valid=dict(sorted(Counter(r["category"] for r in valid).items())),
            distance_bin_valid=dict(
                sorted(Counter(r["distance_bin"] for r in valid).items())
            ),
            numerical_witnesses=sum(
                r["feasibility"] == "numerical_path_witness" for r in valid
            ),
            straight_line_collision_count=sum(
                r["straight_line_collision"] is True for r in valid
            ),
        )
    first = records[0][0]
    generator = read(base / "configs/generator.json")
    metadata = dict(
        schema="benchmark_reference_index",
        dataset="benchmark",
        source_check=source_check,
        counts=counts,
        total_registered=len(features),
        total_valid=sum(r["endpoints_valid"] for r in features),
        original_inputs_modified=False,
        index_semantics="Reference view of existing frozen inputs; no resampling, no best-of selection, no planner-success task filtering",
        source_files_sha256=source_hashes,
        products_sha256={
            name: sha(base / name) for name in PRODUCTS if name != "metadata.json"
        },
        model=_model_metadata(base, repo_root),
        motion_limits=dict(
            velocity_rad_s=first["velocity_limits_rad_s"],
            acceleration_rad_s2=first["acceleration_limits_rad_s2"],
            endpoint_velocity_rad_s=first["endpoint_velocity_rad_s"],
            endpoint_acceleration="Not constrained to zero; same bounded acceleration limits apply",
            time_continuity="C1 with numerical tolerance; Bezier/B-spline derivative control hull bounds",
        ),
        geometry=dict(
            representation="Union of frozen float32-centred obstacle spheres; conservative cover of source primitives",
            voxel_radius_m=generator["voxel_radius_m"],
            maximum_voxel_cell_side_m=generator["maximum_voxel_cell_side_m"],
            contact_band_m=2e-6,
            endpoint_clearance_m=generator["endpoint_clearance_m"],
            planner_padding="Planner-specific conservative search margins recorded in evaluation config; common validation geometry does not change",
            source_primitive_inflation="Nonuniform conservative cover; see each scene voxelization certificate, not a uniform radius expansion",
        ),
        generation=dict(
            rng=generator["rng"],
            scene_seed="SHA256 domain-separated obstacle layout seed",
            query_seed="Original endpoint_seed; independent domain from scene_seed",
            planner_seed="Run-level paired method_seed; never a new scene or query",
            configs={split: f"configs/{split}_generator.json" for split in SPLITS},
            seeds={
                split: read(base / f"receipts/{split}_generation.json")["seed"]
                for split in SPLITS
            },
            selection_policy="Geometry validity and endpoint validity/distance bins only; no algorithm-success selection",
            open_control="All open_control IDs share one base-only physical environment across splits",
        ),
        categories=dict(
            sparse_clutter="Mixed boxes and spheres; not a density-only control against dense_clutter",
            dense_clutter="Random spheres, different size distribution from sparse_clutter",
            bottleneck="Two finite walls and a gap; mandatory C-space passage not established",
            route_choice="Two sphere clusters; C-space homotopy classes not established",
            open_control="No added obstacles; fixed table/equipment and robot self collision remain",
        ),
        known_data_issues=[
            f"{len(features) - sum(r['endpoints_valid'] for r in features)} original generation failures remain registered; legal planner failures are not input failures",
            "dev short/medium/long ratio is 1:1:1, legal test is 2:2:1",
            "dev has no open_control queries",
            "Several problem IDs share one scene; use environment_cluster for correlated statistics",
            "The test split has historical evaluation exposure and is not an untouched holdout; tuning provenance must accompany new comparisons",
            "Conservative voxel cover can appreciably alter source primitive clearance",
        ],
        witnesses=dict(
            policy="First lexicographic receipt-bound validated artifact establishes existence only; no best-of metric selection; solver success alone rejected",
            scan=witness_scan,
            planner_witness_count=len(witnesses),
            continuous_collision_certificate=False,
        ),
        historical_collision_audit="reports/benchmark_design_audit_v2/geometry_run_v3/report.json",
        build=dict(
            module="benchmarks.generic_motion_v1.dataset",
            source_sha256=sha(Path(__file__)),
            git_commit=subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=repo_root, text=True
            ).strip(),
        ),
    )
    _atomic(base / "metadata.json", text_json(metadata))
    return dict(
        passed=True,
        counts=counts,
        planner_witness_count=len(witnesses),
        products=list(PRODUCTS),
    )


def load_index(base=BASE):
    return [
        json.loads(line)
        for line in (Path(base) / "problems.jsonl").read_text().splitlines()
        if line
    ]


def resolve_problem(problem_id, base=BASE):
    """Load the exact original full manifest record for existing planner APIs."""
    base = Path(base)
    matches = [r for r in load_index(base) if r["problem_id"] == problem_id]
    _require(len(matches) == 1, f"Unknown or duplicated problem: {problem_id}")
    reference = matches[0]
    lines = safe_path(base, reference["source_manifest"]).read_text().splitlines()
    row = json.loads(lines[reference["source_line"] - 1])
    _require(
        row["problem_id"] == problem_id
        and row["problem_sha256"] == reference["problem_sha256"],
        f"Index does not resolve to original problem: {problem_id}",
    )
    _require(
        digest({k: v for k, v in row.items() if k != "problem_sha256"})
        == row["problem_sha256"],
        f"Original problem modified: {problem_id}",
    )
    return row


def check(base=BASE, repo_root=ROOT):
    base = Path(base)
    source = check_sources(base, repo_root)
    metadata = read(base / "metadata.json")
    for reference, expected in {
        **metadata["source_files_sha256"],
        **metadata["products_sha256"],
    }.items():
        _require(
            sha(safe_path(base, reference)) == expected,
            f"Index source/product changed: {reference}",
        )
    rows = load_index(base)
    _require(
        len(rows) == source["registered"],
        "Index length differs from frozen registration",
    )
    _require(
        len({r["problem_id"] for r in rows}) == len(rows), "Index duplicates problems"
    )
    source_by_id = {r["problem_id"]: r for r, _, _ in source_records(base)}
    witness_count = 0
    for row in rows:
        original = source_by_id[row["problem_id"]]
        if row["witness"]:
            witness = row["witness"]
            result = numerical_witness(
                safe_path(base, witness["result"]), original, base
            )
            _require(
                result == witness,
                f"Witness no longer matches validated evidence: {row['problem_id']}",
            )
            witness_count += 1
        expected = _feature(original, row["witness"])
        _require(
            all(row.get(k) == v for k, v in expected.items()),
            f"Feature mismatch: {row['problem_id']}",
        )
    return dict(
        **source,
        index_rows=len(rows),
        witness_artifacts_checked=witness_count,
        result="passed",
        collision_scope="Existing numerical evidence imported; run audit to repeat GPU/Drake collision checks",
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("build", "check", "show"))
    parser.add_argument(
        "--root", type=Path, default=BASE, help="Existing benchmark directory"
    )
    parser.add_argument("--problem", help="Problem ID to inspect")
    parser.add_argument("--split", choices=SPLITS)
    args = parser.parse_args(argv)
    if args.command == "build":
        output = build(args.root)
    elif args.command == "check":
        output = check(args.root)
    elif args.problem:
        output = next(
            (r for r in load_index(args.root) if r["problem_id"] == args.problem), None
        )
        if output is None:
            parser.error(f"Unknown problem: {args.problem}")
    else:
        output = read(args.root / "metadata.json")["counts"]
        if args.split:
            output = output[args.split]
    print(json.dumps(output, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
