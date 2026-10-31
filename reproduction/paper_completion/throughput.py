"""Paper-sized collision batches on the actual saved reconstruction scenes."""

import argparse
import datetime
from pathlib import Path
import time
import traceback
import numpy as np
from reproduction.common import ROOT, atomic, read, digest
from reproduction.throughput_records import catch_termination, file_hash, measure_batches, save_input
from .model import FR3, ForestEnvironment
from .run import identity, runtime_versions


def run(base):
    base = Path(base)
    source = read(base / "manifest.json")
    family = source["environment"]
    if family not in {"forest", "fr3"} or source["status"] != "completed":
        raise ValueError("Expected a completed Forest or FR3 run")
    if digest(identity()) != source["implementation_hash"]:
        raise ValueError("Source identity changed")
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    out = ROOT / "reproduction/runs" / (stamp + f"_paper_completion_{family}_throughput")
    out.mkdir(parents=True, exist_ok=False)
    manifest = dict(
        status="running", stage="initialization", environment=family,
        scope=("FR3_33_sphere_reconstruction_with_known_false_obstacle"
               if family == "fr3" else "Forest_declared_random_scene_reconstruction"),
        base_run=str(base.resolve()), base_manifest_sha256=file_hash(base / "manifest.json"),
        configurations_per_batch=2000000, repeats=5, warmups=1,
        independent_points_per_environment=20000,
        environments=source["config"]["environment_seeds"],
        computational_identity=source["implementation_hash"],
        driver_sha256=file_hash(__file__),
        recording_helper_sha256=file_hash(Path(__file__).resolve().parents[1] / "throughput_records.py"),
        timing="synchronous public wrapper including allocations, host/device transfers and bool conversion; sampling, hashing and IO excluded",
        completed_environments=0,
    )
    atomic(out / "manifest.json", manifest)
    print("THROUGHPUT_DIR=" + str(out), flush=True)
    rows = []
    record, dest = None, None
    try:
        with catch_termination():
            manifest["runtime_versions"] = runtime_versions()
            atomic(out / "manifest.json", manifest)
            atomic(out / "results.json", rows)
            for seed in manifest["environments"]:
                dest = out / f"e{seed:02d}"
                dest.mkdir()
                record = dict(seed=seed, status="running", stage="model",
                              verified_points=0, measurements=[], batches=[])
                rows.append(record)

                def persist():
                    atomic(dest / "result.json", record)
                    atomic(out / "results.json", rows)
                    manifest.update(active_environment=seed, stage=record["stage"])
                    atomic(out / "manifest.json", manifest)

                persist()
                if family == "fr3":
                    scene_path = base / "scenes" / f"e{seed:02d}" / "scene.json"
                    scene = read(scene_path)
                    record["scene_sha256"] = file_hash(scene_path)
                    env = FR3(source["config"], scene["obstacles"])
                    obstacle_count = len(scene["obstacles"])
                else:
                    env = ForestEnvironment(source["config"], seed)
                    obstacle_count = len(env.base.centers)
                record.update(static_pairs=int(env.minimal.num_collision_pairs),
                              obstacle_pairs=len(env.geometry_ids) * obstacle_count,
                              sampling_seed_sequence=[2504, seed], stage="input_generation")
                persist()
                q = np.random.default_rng(np.random.SeedSequence([2504, seed])).uniform(
                    env.lower, env.upper, (2000000, env.dimension)).astype(np.float32)
                record.update(save_input(dest, q))
                record["stage"] = "input_saved"
                persist()
                actual = measure_batches(env.cuda, q, dest, record, persist, repeats=5)
                indices = np.linspace(0, len(q) - 1, 20000).astype(int)
                verification_path = dest / "verification_inputs.npz"
                np.savez_compressed(verification_path, indices=indices, configurations=q[indices])
                record.update(stage="independent_check", verification_input_sha256=file_hash(verification_path))
                persist()
                started = time.perf_counter()
                independent = np.asarray(env.independent(q[indices]), dtype=bool)
                record["independent_check_seconds"] = time.perf_counter() - started
                valid_shape = independent.shape == indices.shape
                different = indices[actual[indices] != independent] if valid_shape else np.array([], dtype=int)
                np.savez_compressed(dest / "flags.npz", collision_free=actual,
                                    verification_indices=indices, independent=independent,
                                    disagreement_indices=different)
                record.update(verified_points=len(indices) if valid_shape else 0,
                              independent_valid_shape=valid_shape,
                              model_disagreements=len(different) if valid_shape else None,
                              verification_flags_sha256=file_hash(dest / "flags.npz"),
                              stage="independent_check_saved")
                persist()
                if not valid_shape or len(different):
                    raise ValueError("Independent collision check disagreed or returned wrong shape; raw checks retained")
                record.update(status="completed", stage="completed")
                manifest["completed_environments"] += 1
                persist()
                print(seed, "Mconfig/s", np.mean([x["configurations_per_second"]
                      for x in record["measurements"]]) / 1e6, "disagreements", len(different), flush=True)
            values = [x["configurations_per_second"] / 1e6 for r in rows for x in r["measurements"]]
            summary = dict(
                batches=len(values), configurations_per_batch=2000000,
                mean_million_configurations_per_second=float(np.mean(values)),
                sample_sd=float(np.std(values, ddof=1)),
                verified_points=sum(r["verified_points"] for r in rows),
                model_disagreements=sum(r["model_disagreements"] for r in rows),
                original_paper_reference_million_per_second=(8.6 if family == "fr3" else 19.9),
                comparison="different hardware and reconstructed collision geometry; no direct speedup claim",
            )
            atomic(out / "summary.json", summary)
            manifest.update(status="completed", stage="completed")
            atomic(out / "manifest.json", manifest)
            print(summary, flush=True)
            return out
    except BaseException as error:
        failure = dict(error=repr(error), traceback=traceback.format_exc())
        state = "interrupted" if isinstance(error, (KeyboardInterrupt, InterruptedError)) else "failed"
        # Terminalize the run first, even if a secondary partial-record write
        # subsequently encounters an IO failure.
        manifest.update(status=state, **failure)
        atomic(out / "manifest.json", manifest)
        if record is not None and record.get("status") != "completed":
            record.update(status=state, **failure)
            atomic(dest / "result.json", record)
        atomic(out / "results.json", rows)
        raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    args = parser.parse_args()
    run(args.run_dir)


if __name__ == "__main__":
    main()
