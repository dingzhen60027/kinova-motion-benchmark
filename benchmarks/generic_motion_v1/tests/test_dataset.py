"""CPU tests of the compact index and strict numerical-witness import."""

import json
from pathlib import Path
import shutil

import numpy as np
import pytest
from benchmarks.generic_motion_v1.tests.evidence_fixtures import save_numeric_output
from benchmarks.generic_motion_v1.common import BASE, read

from benchmarks.generic_motion_v1 import dataset


@pytest.fixture(scope="module")
def row():
    return dataset.source_records()[0][0]


def _write_json(path, value):
    path.write_text(json.dumps(value, allow_nan=False))


@pytest.fixture
def witness_bundle(tmp_path, row):
    folder = tmp_path / "results/run/jobs/task/attempts/0001"
    folder.mkdir(parents=True)
    job = dict(job_id="task", case=row, case_id=row["problem_id"], method="fixture",
               objective="minimum_distance", method_seed=0,
               evaluation_config=dict(endpoint_tolerance_rad=1e-5,validation_step_rad=.005))
    _write_json(folder.parents[3] / "run_manifest.json", dict(jobs=[job],timeout_seconds=60))
    _write_json(folder / "request.json",job)
    _write_json(folder / "process.json",dict(exit_code=0,timed_out=False,timeout_seconds=60,cold_e2e_seconds=1.))
    _write_json(
        folder / "result.json",
        dict(
            status="success_validated_numerical",
            problem_id=row["problem_id"], scene_id=row["scene_id"], family=row["family"],
            method_seed=0, budget_seconds=60,
            method="fixture",
            objective="minimum_distance",
        ),
    )
    _write_json(
        folder / "validation.json",
        dict(
            passed=True, status="success_validated_numerical",cost=1.,cost_unit="rad",
            collision_free=True,
            joint_limits_passed=True,
            derivative_limits_passed=True,
            native_collisions=0,
            independent_collisions=0,
            model_disagreements=0,
            contact_band_samples=0,
            endpoint_error_rad=0,
            sample_step_bound_rad=0.005,
            minimum_sampled_clearance_m=0.01,
        ),
    )
    _write_json(folder / "input.json", row)
    save_numeric_output(folder,row,job['evaluation_config'])
    _seal(folder)
    return folder


def _seal(folder):
    _write_json(
        folder / "receipt.json",
        {
            "status": dataset.read(folder / "result.json")["status"],
            "files": {
                p.name: dataset.sha(p)
                for p in folder.iterdir()
                if p.name != "receipt.json"
            }
        },
    )


def test_import_requires_complete_bound_numerical_evidence(
    witness_bundle, row, tmp_path
):
    witness = dataset.numerical_witness(witness_bundle / "result.json", row, tmp_path)
    assert witness["certification"] == "finite_numerical_sampling"
    assert len(witness["files_sha256"]) == 9
    assert not Path(witness["result"]).is_absolute()


@pytest.mark.parametrize(
    "key,value",
    [
        ("passed", False),
        ("independent_collisions", 1),
        ("joint_limits_passed", False),
        ("derivative_limits_passed", False),
        ("model_disagreements", 1),
        ("contact_band_samples", 1),
        ("endpoint_error_rad", 0.1),
        ("sample_step_bound_rad", 0.04),
        ("minimum_sampled_clearance_m", 0),
    ],
)
def test_solver_success_cannot_override_validation_failure(
    witness_bundle, row, tmp_path, key, value
):
    path = witness_bundle / "validation.json"
    validation = dataset.read(path)
    validation[key] = value
    _write_json(path, validation)
    _seal(witness_bundle)
    assert (
        dataset.numerical_witness(witness_bundle / "result.json", row, tmp_path) is None
    )


def test_modified_artifact_rejects_witness(witness_bundle, row, tmp_path):
    (witness_bundle / "candidate.npz").write_bytes(b"changed")
    assert (
        dataset.numerical_witness(witness_bundle / "result.json", row, tmp_path) is None
    )


def test_other_endpoints_reject_witness_even_with_matching_problem_id(
    witness_bundle, row, tmp_path
):
    source = dataset.read(witness_bundle / "input.json")
    source["q_goal"][0] += 0.1
    _write_json(witness_bundle / "input.json", source)
    _seal(witness_bundle)
    assert (
        dataset.numerical_witness(witness_bundle / "result.json", row, tmp_path) is None
    )


def test_coarse_straight_check_is_not_common_validator_witness(row):
    row = dict(
        row,
        straight_line_collision=False,
        straight_line_disagreements_outside_contact_band=0,
    )
    feature = dataset._feature(row, None)
    assert feature["feasibility"] == "unknown_feasibility"
    assert feature["witness_kind"] is None


def test_open_ids_share_physical_cluster():
    rows = [r for r, _, _ in dataset.source_records() if r["family"] == "open_control"]
    features = [dataset._feature(r, None) for r in rows]
    assert len({r["scene_id"] for r in rows}) > 1
    assert len({r["environment_cluster"] for r in features}) == 1


def test_invalid_inputs_retain_null_metrics():
    row = next(
        r
        for r, _, _ in dataset.source_records()
        if r["validation_status"] != "valid_input"
    )
    feature = dataset._feature(row, None)
    assert feature["input_status"] == "generation_failed"
    assert feature["feasibility"] == "invalid_input"
    assert feature["distance_rad"] is None
    assert feature["input_failure_reason"] == "no_legal_start_within_generation_budget"


def test_actual_frozen_inputs_pass_cpu_integrity():
    result = dataset.check_sources()
    assert result["registered"] == 490
    assert result["valid"] == 463
    assert result["scenes"] == 109
    assert result["collision_rechecked"] is False


def test_tampered_manifest_is_detected_before_indexing(tmp_path):
    for name in ("configs", "receipts", "scenes", "inputs"):
        (tmp_path / name).symlink_to(dataset.BASE / name, target_is_directory=True)
    shutil.copytree(dataset.BASE / "manifests", tmp_path / "manifests")
    path = tmp_path / "manifests" / "smoke.jsonl"
    path.write_text(path.read_text() + "\n")
    with pytest.raises(ValueError, match="Frozen smoke manifest changed"):
        dataset.check_sources(tmp_path)


@pytest.mark.parametrize("reference", ["../outside", "/tmp/outside"])
def test_reference_cannot_escape_dataset(tmp_path, reference):
    with pytest.raises(ValueError, match="relative dataset reference"):
        dataset.safe_path(tmp_path, reference)


def test_index_resolves_original_record_without_array_duplication(row):
    resolved = dataset.resolve_problem(row["problem_id"])
    assert resolved == row
    index = dataset.load_index()
    assert len(index) == 490
    assert all(
        "q_start" not in record and "voxels_xyz" not in record for record in index
    )


@pytest.mark.parametrize("clearance_sign", [1, -1])
def test_cpu_source_check_rejects_saved_straight_line_checker_disagreement(tmp_path, clearance_sign):
    row = next(r for r, _, _ in dataset.source_records()
               if r["validation_status"] == "valid_input" and r["straight_line_collision"])
    config = dataset.read(dataset.BASE / f"configs/{row['split']}_generator.json")
    robot = dataset.read(dataset.BASE / "configs/robot.json")
    with np.load(dataset.BASE / row["input_arrays_file"], allow_pickle=False) as stored:
        arrays = {name: stored[name].copy() for name in stored.files}
    distances = arrays["straight_line_independent_clearance_m"]
    index = np.flatnonzero(clearance_sign * distances > robot["contact_band_m"])[0]
    arrays["straight_line_cuda_free"][index] = not (distances[index] > 0)
    path = tmp_path / row["input_arrays_file"]
    path.parent.mkdir(parents=True)
    np.savez_compressed(path, **arrays)
    # The overall line remains blocked, so its Boolean classification alone
    # cannot expose the newly inconsistent per-sample observations.
    assert not (arrays["straight_line_cuda_free"].all() and (distances > 0).all())
    with pytest.raises(ValueError, match="Saved straight-line checker disagreement"):
        dataset._check_saved_arrays(row, tmp_path, config, robot)
