"""Two-million-configuration diagnostic on the declared public iiwa assembly.

This is not the original 105-pair hardware model or a physical experiment.
Use the existing Conda launcher; no dependency installation is performed.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import importlib.metadata
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback

import csdecomp as csd
import numpy as np
from pydrake.all import RobotDiagramBuilder, Role

from reproduction.common import ROOT, atomic
from reproduction.throughput_records import catch_termination, file_hash, measure_batches, save_input
from reproduction.paper_protocol.model import matched_native
from reproduction.probe_public_iiwa import BASE, SOURCES, assemble, fetch_assets


BATCH_SIZE = 2_000_000
REPEATS = 5
SEED = 250410783


def command_record(command):
    try:
        result = subprocess.run(command, text=True, capture_output=True, timeout=30)
        return dict(command=command, exit_code=result.returncode,
                    stdout=result.stdout, stderr=result.stderr)
    except Exception as error:
        return dict(command=command, error=repr(error))


def runtime_identity():
    native_dir = Path(csd.__file__).resolve().parent
    loaded_binaries = {
        str(Path(module.__file__).resolve()): file_hash(module.__file__)
        for name, module in tuple(sys.modules.items())
        if name.startswith(("csdecomp", "pycsdecomp"))
        and (getattr(module, "__file__", "") or "").endswith(".so")
    }
    # Native extension modules do not always appear in sys.modules under the
    # package name. Also retain all shared libraries beside the actual import.
    for path in native_dir.glob("*.so"):
        loaded_binaries.setdefault(str(path), file_hash(path))
    source_paths = [Path(__file__), ROOT / "reproduction/probe_public_iiwa.py",
                    ROOT / "reproduction/paper_protocol/model.py",
                    ROOT / "reproduction/common.py", ROOT / "reproduction/throughput_records.py",
                    ROOT / "local/python.sh"]
    return dict(
        python=sys.version, executable=sys.executable, platform=platform.platform(),
        packages={name: importlib.metadata.version(name)
                  for name in ["csdecomp", "numpy", "drake"]},
        csdecomp_import=str(Path(csd.__file__).resolve()),
        native_binary_sha256=loaded_binaries,
        source_sha256={str(p.relative_to(ROOT)): file_hash(p) for p in source_paths},
        environment_flags={name: os.environ.get(name) for name in [
            "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
            "CUDA_VISIBLE_DEVICES", "CUDA_MODULE_LOADING"]},
        build_flags=dict(status="not_inferred_from_installed_binary",
                         limitation="Package metadata and binary hashes are retained; unavailable build flags are not invented."),
        gpu=command_record(["nvidia-smi", "-q"]),
        git=command_record(["git", "rev-parse", "HEAD"]),
    )


def create_model(urdf):
    builder = RobotDiagramBuilder()
    plant, sg = builder.plant(), builder.scene_graph()
    instance = builder.parser().AddModelsFromString(urdf, "urdf")[0]
    plant.Finalize()
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    pc = plant.GetMyMutableContextFromRoot(context)
    inspector = sg.get_query_output_port().Eval(sg.GetMyContextFromRoot(context)).inspector()
    parts = diagram, context, plant, pc, sg, inspector, instance
    native = matched_native(parts)
    pairs = sorted((sorted([inspector.GetName(a), inspector.GetName(b)])
                    for a, b in inspector.GetCollisionCandidates()))
    table_pairs = [pair for pair in pairs if any("::probe_table" in x for x in pair)]
    metadata = dict(
        positions=plant.num_positions(),
        robot_box_geometries=11,
        total_geometries_including_declared_table=inspector.NumGeometriesWithRole(Role.kProximity),
        active_native_robot_geometries=len(native.getRobotGeometryIds()),
        static_candidate_pairs_including_table=len(pairs),
        robot_robot_candidate_pairs=len(pairs) - len(table_pairs),
        robot_table_candidate_pairs=len(table_pairs),
        native_collision_pairs=int(native.getMinimalPlant().num_collision_pairs),
        pair_names=pairs,
        lower=plant.GetPositionLowerLimits(), upper=plant.GetPositionUpperLimits(),
        collision_filter="Drake candidate pairs copied exactly by matched_native; no outcome-based exclusions",
    )
    if metadata["native_collision_pairs"] != len(pairs):
        raise ValueError("Native and independent candidate pair counts differ")
    return parts, native, metadata


def run(args):
    if args.verification_samples < 2000 or args.verification_samples > BATCH_SIZE:
        raise ValueError("verification-samples must be between 2000 and 2000000")
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    out = (args.output or ROOT / "reproduction/runs" / (stamp + "_public_iiwa_throughput")).resolve()
    out.mkdir(parents=True, exist_ok=False)
    manifest = dict(
        status="running", stage="initialization", created_utc=stamp, command=[sys.executable, *sys.argv],
        scope="public_11_box_iiwa_plus_declared_table_collision_throughput_diagnostic",
        configurations_per_batch=BATCH_SIZE, warmups=1, repeats=REPEATS,
        verification_samples=args.verification_samples, seed=SEED,
        original_author_model_recovered=False, physical_experiment=False,
        paper_reference_million_configurations_per_second=6.9,
        direct_reference_speed_ratio_permitted=False,
        reason="Different model/filter/hardware: published hardware has 105 self pairs; this declared public assembly has its own measured candidate-pair count.",
        timing="Synchronous public CheckCollisionFreeCuda wrapper (device synchronization and host return); includes allocations, transfers and conversion to bool; excludes sampling, independent checking and file IO.",
        gripper_mount=dict(xyz=[0, 0, .09], rpy=[-np.pi / 2, 0, np.pi / 2],
                           provenance="explicit public-URDF frame adaptation, not paper calibration"),
        table=dict(size=[1.4, 1.6, .1], center=[.5, 0, -.05],
                   provenance="explicit reconstruction assumption"),
        sources=[dict(name=name, url=url, sha256=expected) for name, url, expected in SOURCES],
        measurements=[],
    )
    atomic(out / "manifest.json", manifest)
    print("THROUGHPUT_DIR=" + str(out), flush=True)
    try:
        with catch_termination():
            atomic(out / "runtime_identity.json", runtime_identity())
            fetch_assets(args.asset_dir)
            urdf = assemble(args.asset_dir, table=True)
            (out / "assembled.urdf").write_text(urdf)
            parts, native, model = create_model(urdf)
            model["assembled_urdf_sha256"] = file_hash(out / "assembled.urdf")
            atomic(out / "model.json", model)
            diagram, context, plant, pc, sg, inspector, instance = parts
            q = np.random.default_rng(SEED).uniform(model["lower"], model["upper"],
                                                    (BATCH_SIZE, 7)).astype(np.float32)
            manifest.update(save_input(out, q))
            manifest["stage"] = "input_saved"
            atomic(out / "manifest.json", manifest)
            # The public wrapper synchronizes and returns host flags. The shared
            # recorder persists input identity and every attempt before the next call.
            minimal = native.getMinimalPlant()
            flags = measure_batches(
                lambda samples: csd.CheckCollisionFreeCuda(samples.T, minimal), q, out,
                manifest, lambda: atomic(out / "manifest.json", manifest), repeats=REPEATS)
            indices = np.linspace(0, BATCH_SIZE - 1, args.verification_samples, dtype=int)
            verification_inputs = out / "verification_inputs.npz"
            np.savez_compressed(verification_inputs, indices=indices, configurations=q[indices])
            manifest.update(stage="independent_check", verification_input_sha256=file_hash(verification_inputs))
            atomic(out / "manifest.json", manifest)
            independent, minimum = [], []
            for index in indices:
                # Check the exact float32 input values in float64 Drake arithmetic.
                plant.SetPositions(pc, q[index].astype(float))
                query = sg.get_query_output_port().Eval(sg.GetMyContextFromRoot(context))
                distances = query.ComputeSignedDistancePairwiseClosestPoints()
                distance = min((x.distance for x in distances), default=float("inf"))
                independent.append(distance >= 0)
                minimum.append(distance)
                if len(independent) % 250 == 0:
                    manifest["independent_points_completed"] = len(independent)
                    atomic(out / "manifest.json", manifest)
            independent = np.asarray(independent, dtype=bool)
            different = indices[flags[indices] != independent]
            np.savez_compressed(out / "independent_verification.npz", indices=indices,
                                configurations=q[indices], cuda_free=flags[indices],
                                drake_free=independent, minimum_signed_distance=minimum,
                                disagreement_indices=different)
            rates = np.array([row["configurations_per_second"] for row in manifest["measurements"]
                              if not row["warmup"]]) / 1e6
            summary = dict(
                scope=manifest["scope"], original_author_model_recovered=False,
                physical_experiment=False, direct_reference_speed_ratio_permitted=False,
                measured_batches=len(rates), configurations_per_batch=BATCH_SIZE,
                mean_million_configurations_per_second=float(np.mean(rates)),
                sample_sd_million_configurations_per_second=float(np.std(rates, ddof=1)),
                median_million_configurations_per_second=float(np.median(rates)),
                independently_verified_points=len(indices), model_disagreements=len(different),
                static_pairs_including_table=model["static_candidate_pairs_including_table"],
                robot_robot_pairs=model["robot_robot_candidate_pairs"],
                robot_table_pairs=model["robot_table_candidate_pairs"],
                verification_file_sha256=file_hash(out / "independent_verification.npz"),
            )
            atomic(out / "summary.json", summary)
            if len(different):
                raise ValueError("CUDA/Drake disagreements; measurements and failing input indices retained")
            manifest.update(status="completed_diagnostic", stage="completed")
            atomic(out / "manifest.json", manifest)
            print(summary, flush=True)
            return out
    except BaseException as error:
        manifest.update(status="interrupted" if isinstance(error, (KeyboardInterrupt, InterruptedError)) else "failed",
                        error=repr(error), traceback=traceback.format_exc())
        atomic(out / "manifest.json", manifest)
        if "independent" in locals() and "indices" in locals():
            count = len(independent)
            partial = out / "independent_partial.npz"
            np.savez_compressed(partial, indices=indices[:count], configurations=q[indices[:count]],
                                cuda_free=flags[indices[:count]], drake_free=independent,
                                minimum_signed_distance=minimum)
            manifest.update(independent_points_completed=count,
                            independent_partial_sha256=file_hash(partial))
        atomic(out / "manifest.json", manifest)
        raise


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--asset-dir", type=Path, default=BASE / "hardware_assets")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--verification-samples", type=int, default=2000)
    run(parser.parse_args())


if __name__ == "__main__":
    main()
