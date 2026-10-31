"""Supplementary CPU-only official-geometry replay of saved FR3 trajectories.

Run after timed experiments stop. Each saved trajectory, including failed
candidates, gets 17 uniform targets on its saved parameter/time axis and the
nearest real saved rows. Polygon and distance-NTO axes are not physical time.
This is sampled reference evidence, not a continuous collision certificate.
The original run and its flags are never modified.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import importlib.util
import importlib.metadata
from pathlib import Path
import sys
import time
import traceback

import numpy as np

from reproduction.common import ROOT, atomic, digest, read
from reproduction.validate_constructed_geometry import (
    DEFAULT_OUT, GeometryOracle, INTERPRETATION, description, prepare,
)


def sha(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            value.update(chunk)
    return value.hexdigest()


def frozen_file(run, manifest, relative):
    path = run / "source_snapshot" / relative
    expected = manifest["source_files"].get(relative)
    if expected is None or sha(path) != expected:
        raise ValueError("Missing/mismatched frozen source: " + relative)
    return path


def nearest_saved_rows(times, q):
    times = np.asarray(times, float)
    q = np.asarray(q, float)
    if times.ndim != 1 or q.ndim != 2 or q.shape != (len(times), 7):
        raise ValueError("Expected aligned time[N] and q[N,7] arrays")
    original = np.flatnonzero(np.isfinite(times))
    if not len(original):
        raise ValueError("No finite trajectory times")
    finite_times = times[original]
    if np.any(np.diff(finite_times) < 0):
        raise ValueError("Saved finite times are not monotonic")
    if not np.all(np.isfinite(q[original])):
        raise ValueError("A finite-time trajectory row has nonfinite configuration")
    targets = np.linspace(finite_times[0], finite_times[-1], 17)
    upper = np.clip(np.searchsorted(finite_times, targets, side="left"), 0, len(original) - 1)
    lower = np.maximum(upper - 1, 0)
    # Deterministic lower-index tie break, including duplicate saved timestamps.
    selected = np.where(abs(finite_times[lower] - targets) <= abs(finite_times[upper] - targets), lower, upper)
    selected = np.searchsorted(finite_times, finite_times[selected], side="left")
    indices = original[selected]
    return dict(target_time=targets, actual_time=times[indices], time_offset=times[indices] - targets,
                source_row_indices=indices, q=q[indices], finite_rows=len(original),
                excluded_nonfinite_time_rows=len(times) - len(original),
                unique_selected_rows=len(np.unique(indices)), duplicate_selections=17 - len(np.unique(indices)))


AXIS_SEMANTICS = {
    "LSCS": "normalized saved polygon sample-index parameter [0,1]; not physical seconds",
    "LGCS": "normalized saved polygon sample-index parameter [0,1]; not physical seconds",
    "NTO_distance": "fixed normalized spline parameter [0,1]; not optimized physical seconds",
    "SCSTO": "Bezier duration parameter, seconds under the declared motion bounds",
    "NTO_time": "optimized spline duration parameter, seconds under the declared motion bounds",
}


def sampling_controls():
    """Analytic bookkeeping checks, run only when the replay itself runs."""
    repeated = nearest_saved_rows(np.array([0., 0., 1.]), np.zeros((3, 7)))
    witness_q = np.zeros((3, 7)); witness_q[1] = np.nan
    witness = nearest_saved_rows(np.array([0., np.nan, 1.]), witness_q)
    tests = [dict(name="duplicate_timestamp_earliest_row_tie", passed=int(repeated["source_row_indices"][8]) == 0),
             dict(name="nonfinite_time_witness_excluded", passed=witness["excluded_nonfinite_time_rows"] == 1 and 1 not in witness["source_row_indices"]),
             dict(name="short_replay_duplicates_are_recorded", passed=repeated["unique_selected_rows"] == 2 and repeated["duplicate_selections"] == 15)]
    for name, times, q in [("nonmonotonic_axis_rejected", np.array([0., 1., .5]), np.zeros((3, 7))),
                           ("nonfinite_finite_time_configuration_rejected", np.array([0., 1., 2.]), witness_q)]:
        try:
            nearest_saved_rows(times, q)
            passed = False
        except ValueError:
            passed = True
        tests.append(dict(name=name, passed=passed))
    return dict(passed=all(test["passed"] for test in tests), tests=tests)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--oracle-inputs", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--trajectory-limit", type=int, help="Smoke test only; selection is lexical and incomplete scope is explicitly recorded")
    args = parser.parse_args()
    run = args.run_dir.resolve()
    output = (args.output or (ROOT / "reproduction/reports/paper_constructed" / ("trajectory_geometry_" + run.name))).resolve()
    if output == run or output.is_relative_to(run):
        raise ValueError("Supplementary output must be outside the original run")
    if args.trajectory_limit is not None and args.trajectory_limit <= 0:
        raise ValueError("trajectory-limit must be positive")
    output.mkdir(parents=True, exist_ok=False)
    started = time.time()
    script_identity = sha(Path(__file__))
    helper_path = Path(__file__).with_name("validate_constructed_geometry.py")
    helper_identity = sha(helper_path)
    manifest_path = run / "manifest.json"
    manifest_identity = sha(manifest_path)
    input_inventory = {str(manifest_path): manifest_identity}
    def remember(path, required=True):
        path = Path(path).resolve()
        current = sha(path) if path.is_file() else None
        if current is None and required:
            raise FileNotFoundError(path)
        if str(path) in input_inventory and input_inventory[str(path)] != current:
            raise ValueError("Input changed while replay was reading it: " + str(path))
        input_inventory[str(path)] = current
        return current
    try:
        manifest = read(manifest_path)
        if manifest["environment"] != "fr3" or manifest["status"] != "completed":
            raise ValueError("Replay requires a completed FR3 run; do not overlap timing")
        controls = sampling_controls()
        atomic(output / "sampling_controls.json", controls)
        if not controls["passed"]:
            raise ValueError("Saved-axis sampling controls failed")
        if digest(manifest["source_files"]) != manifest["implementation_hash"]:
            raise ValueError("Manifest source map does not match implementation hash")
        if digest(manifest["config"]) != manifest["config_hash"]:
            raise ValueError("Manifest configuration does not match config hash")
        for relative in manifest["source_files"]:
            remember(frozen_file(run, manifest, relative))
        if not (args.oracle_inputs / "preregistration.json").exists():
            raise FileNotFoundError("Prepare and retain fixed official oracle inputs first")
        prereg = prepare(args.oracle_inputs)
        remember(args.oracle_inputs / "preregistration.json")
        for name in prereg["frozen_files"]:
            remember(args.oracle_inputs / name)
        for source in prereg["sources"]:
            remember(source["path"])
        desc = description(args.oracle_inputs)
        prefix = "reproduction/paper_constructed/"
        model_path = frozen_file(run, manifest, prefix + "model.py")
        asset_path = frozen_file(run, manifest, prefix + "assets/fr3_33.json")
        config_path = frozen_file(run, manifest, prefix + "config.json")
        asset, config = read(asset_path), read(config_path)
        if config != manifest["config"]:
            raise ValueError("Frozen configuration differs from run manifest")
        if config["fr3_scene"] != read(args.oracle_inputs / "scene_configuration.json"):
            raise ValueError("Run geometry settings differ from the fixed oracle inputs")
        if asset["kinematics"] != desc["kinematics"] or asset["limits"] != desc["limits"]:
            raise ValueError("Run kinematics/limits differ from official pinned description")
        actual_filters = {frozenset(pair) for pair in asset["disabled_collision_links"]}
        official_filters = {frozenset((row["first"], row["second"])) for row in desc["disabled"]}
        if actual_filters != official_filters:
            raise ValueError("Run filters differ from independently parsed official SRDF")
        spec = importlib.util.spec_from_file_location("frozen_fr3_geometry_replay", model_path)
        model = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(model)
        # ASSETS resolves relative to the frozen model.py, not today's package.
        chain = model.fr3_urdf()
        cases = {entry["id"]: entry for entry in manifest["cases"]}
        all_paths = sorted((run / "cases").rglob("trajectory_samples.npz"))
        selected_paths = all_paths if args.trajectory_limit is None else all_paths[:args.trajectory_limit]
        identities = dict(run_dir=str(run), manifest_sha256=manifest_identity,
                          run_implementation_hash=manifest["implementation_hash"],
                          frozen_model_sha256=sha(model_path), frozen_asset_sha256=sha(asset_path),
                          frozen_config_sha256=sha(config_path), replay_script_sha256=script_identity,
                          oracle_script_sha256=helper_identity,
                          preregistration_sha256=sha(args.oracle_inputs / "preregistration.json"),
                          command=sys.argv, input_geometry_sources=prereg["sources"],
                          runtime_versions=dict(drake=importlib.metadata.version("drake"), numpy=np.__version__),
                          interpretation=INTERPRETATION,
                          sampling="17 fixed uniform targets on the saved parameter/time axis; nearest finite-axis saved row, earliest duplicate/tie index; duplicates and offsets retained; field names retain the stored time convention",
                          axis_semantics_by_method=AXIS_SEMANTICS,
                          flags_policy="all saved candidates regardless of solver success; no original flags are modified",
                          all_trajectory_files=len(all_paths), selected_trajectory_files=len(selected_paths),
                          trajectory_limit=args.trajectory_limit,
                          incomplete_scope=args.trajectory_limit is not None and len(selected_paths) < len(all_paths))
        atomic(output / "input_identity.json", identities)
        oracles, scene_records, records = {}, {}, []
        for replay in selected_paths:
            relative = replay.relative_to(run / "cases")
            case_id = relative.parts[0]
            method = relative.parts[1] if len(relative.parts) > 2 else "unknown"
            dest = output / "trajectories" / relative.parent
            dest.mkdir(parents=True, exist_ok=False)
            record = dict(case_id=case_id, method=method, saved_axis_semantics=AXIS_SEMANTICS.get(method, "saved axis; physical units unestablished"), replay_path=str(replay), replay_sha256=remember(replay))
            try:
                entry = cases[case_id]
                seed = entry["environment_seed"]
                case_input_path = run / "cases" / case_id / "input.json"
                scene_path = run / "scenes" / f"e{seed:02d}" / "scene.json"
                record.update(case_input_sha256=remember(case_input_path), scene_sha256=remember(scene_path))
                case_input, scene_record = read(case_input_path), read(scene_path)
                if not np.array_equal(np.asarray(case_input["obstacles"]), np.asarray(scene_record["obstacles"])):
                    raise ValueError("Case obstacles differ from frozen scene inputs")
                if seed not in oracles:
                    oracles[seed] = GeometryOracle(desc, args.oracle_inputs, chain, config["fr3_scene"], case_input["obstacles"], official=True)
                    atomic(output / "scenes" / f"e{seed:02d}_pair_metadata.json", oracles[seed].pair_metadata)
                    scene_records[seed] = dict(scene_path=str(scene_path), scene_sha256=record["scene_sha256"], obstacles=case_input["obstacles"], candidate_pairs=len(oracles[seed].pairs))
                elif record["scene_sha256"] != scene_records[seed]["scene_sha256"]:
                    raise ValueError("Scene changed during replay")
                oracle = oracles[seed]
                with np.load(replay) as saved:
                    samples = nearest_saved_rows(saved["time"], saved["q"])
                    flags = {}
                    for name in ("cuda", "independent"):
                        if name in saved:
                            if len(saved[name]) != len(saved["time"]):
                                raise ValueError("Saved flag array is not aligned: " + name)
                            flags["saved_" + name] = np.asarray(saved[name], bool)[samples["source_row_indices"]]
                distances = np.array([oracle.distances(q) for q in samples["q"]])
                minimum = distances.min(axis=1)
                pair_indices = distances.argmin(axis=1)
                result_path = replay.parent / "result.json"
                result_identity = remember(result_path, required=False)
                original_result = read(result_path) if result_path.exists() else None
                primitive_pairs = np.array(["official_mesh_convex_hull" not in (pair["first_kind"], pair["second_kind"]) for pair in oracle.pair_metadata], bool)
                exact_primitive_hits = np.any(distances[:, primitive_pairs] < 0, axis=1)
                hull_only_candidates = (minimum < 0) & ~exact_primitive_hits
                point_records = []
                for i in range(17):
                    point_records.append(dict(target_time=samples["target_time"][i], actual_time=samples["actual_time"][i], source_row=int(samples["source_row_indices"][i]), minimum_distance=float(minimum[i]), exact_primitive_collision=bool(exact_primitive_hits[i]), hull_only_collision_candidate=bool(hull_only_candidates[i]), near_contact=bool(np.any(abs(distances[i]) <= 1e-8)), deepest_pair=oracle.pair_metadata[int(pair_indices[i])], collisions=[dict(oracle.pair_metadata[j], distance=float(distances[i, j])) for j in np.argsort(distances[i]) if distances[i, j] < 0]))
                np.savez_compressed(dest / "official_replay.npz", **{key: value for key, value in samples.items() if isinstance(value, np.ndarray)}, **flags, reference_pair_distances=distances, reference_collision_free=minimum >= 0)
                record.update(status="checked", targets=17, finite_saved_rows=samples["finite_rows"], excluded_nonfinite_time_rows=samples["excluded_nonfinite_time_rows"], unique_selected_rows=samples["unique_selected_rows"], duplicate_selections=samples["duplicate_selections"], maximum_time_offset=float(abs(samples["time_offset"]).max()), official_hull_collision_free_at_selected_rows=bool(np.all(minimum >= 0)), minimum_distance=float(minimum.min()), deepest_pair=oracle.pair_metadata[int(pair_indices[int(np.argmin(minimum))])], reference_collision_samples=int((minimum < 0).sum()), exact_primitive_collision_samples=int(exact_primitive_hits.sum()), hull_only_collision_candidate_samples=int(hull_only_candidates.sum()), reference_near_contact_samples=int(np.any(abs(distances) <= 1e-8, axis=1).sum()), original_result=original_result, original_result_sha256=result_identity, saved_selected_flags={key: value.tolist() for key, value in flags.items()}, points=point_records)
                if sha(replay) != record["replay_sha256"]:
                    raise ValueError("Saved trajectory changed while checking")
            except Exception as error:
                record.update(status="error", exception=repr(error), traceback=traceback.format_exc())
            atomic(dest / "record.json", record)
            records.append(record)
            atomic(output / "frozen_input_inventory.json", input_inventory)
            atomic(output / "progress.json", dict(checked=sum(row["status"] == "checked" for row in records), errors=sum(row["status"] == "error" for row in records), selected=len(selected_paths)))
        inventory = []
        for case_id, entry in cases.items():
            case_path = run / "cases" / case_id / "case.json"
            remember(case_path, required=False)
            case_result = read(case_path) if case_path.exists() else {}
            for method in config["method_order"]:
                replay_path = run / "cases" / case_id / method / "trajectory_samples.npz"
                method_path = replay_path.parent / "result.json"
                remember(method_path, required=False)
                method_result = read(method_path) if method_path.exists() else {}
                inventory.append(dict(case_id=case_id, method=method, case_status=case_result.get("status", "missing"), method_status=method_result.get("status", "not_recorded"), trajectory_present=replay_path.exists(), selected_for_check=replay_path in selected_paths))
        checked = [row for row in records if row["status"] == "checked"]
        by_method = {}
        for method in config["method_order"]:
            chosen = [row for row in checked if row["method"] == method]
            by_method[method] = dict(checked_trajectories=len(chosen), reference_free_trajectories=sum(row["official_hull_collision_free_at_selected_rows"] for row in chosen), reference_collision_trajectories=sum(not row["official_hull_collision_free_at_selected_rows"] for row in chosen), selected_reference_collision_points=sum(row["reference_collision_samples"] for row in chosen), exact_primitive_collision_points=sum(row["exact_primitive_collision_samples"] for row in chosen), hull_only_collision_candidate_points=sum(row["hull_only_collision_candidate_samples"] for row in chosen), near_contact_points=sum(row["reference_near_contact_samples"] for row in chosen), original_method_status_counts=dict(Counter((row["original_result"] or {}).get("status", "missing") for row in chosen)))
        if sha(manifest_path) != manifest_identity or sha(Path(__file__)) != script_identity or sha(helper_path) != helper_identity:
            raise ValueError("Run manifest or replay implementation changed during validation")
        atomic(output / "frozen_input_inventory.json", input_inventory)
        drift = [path for path, expected in input_inventory.items() if (sha(path) if Path(path).is_file() else None) != expected]
        files_changed = sorted((run / "cases").rglob("trajectory_samples.npz")) != all_paths
        atomic(output / "input_reverification.json", dict(passed=not drift and not files_changed, changed_or_missing=drift, trajectory_file_set_changed=files_changed))
        if drift or files_changed:
            raise ValueError("Replay input inventory changed; partial results remain unverified: " + repr(drift))
        atomic(output / "trajectory_inventory.json", inventory)
        atomic(output / "scene_inputs.json", scene_records)
        atomic(output / "summary.json", dict(status="completed" if len(checked) == len(selected_paths) else "completed_with_errors", all_trajectory_files=len(all_paths), selected_trajectory_files=len(selected_paths), checked_trajectories=len(checked), errors=len(records)-len(checked), cases=len(cases), requested_case_methods=len(inventory), missing_trajectory_files=sum(not row["trajectory_present"] for row in inventory), incomplete_scope=identities["incomplete_scope"], target_samples=17*len(checked), unique_saved_rows=sum(row["unique_selected_rows"] for row in checked), excluded_nonfinite_time_rows=sum(row["excluded_nonfinite_time_rows"] for row in checked), duplicate_selections=sum(row["duplicate_selections"] for row in checked), methods=by_method, elapsed_seconds=time.time()-started, interpretation=INTERPRETATION, modified_main_results=False))
        print(output / "summary.json", flush=True)
    except BaseException as error:
        atomic(output / "failure.json", dict(status="failed", command=sys.argv, exception=repr(error), traceback=traceback.format_exc()))
        raise


if __name__ == "__main__":
    main()
