"""CPU-only fault injection; no native model, GPU or solver imports."""

import ast
import datetime
from pathlib import Path
import signal
import sys
import traceback
from types import SimpleNamespace

import numpy as np
import pytest

from reproduction.common import ROOT, atomic, digest, read
from reproduction.throughput_records import catch_termination, file_hash, measure_batches, save_input


def test_batch_identity_and_all_six_returned_flag_arrays(tmp_path):
    q = np.arange(24, dtype=np.float32).reshape(8, 3)
    record = save_input(tmp_path, q)
    persist = lambda: atomic(tmp_path / "record.json", record)
    flags = measure_batches(lambda x: x[:, 0] % 2 == 0, q, tmp_path, record, persist)
    saved = read(tmp_path / "record.json")
    assert saved["configuration_file_sha256"] == file_hash(tmp_path / "configurations.npy")
    assert len(saved["batches"]) == 6 and len(saved["measurements"]) == 5
    assert all(row["status"] == "completed" for row in saved["batches"])
    for row in saved["batches"]:
        path = tmp_path / row["flags_file"]
        assert file_hash(path) == row["flags_file_sha256"]
        assert np.array_equal(np.load(path)["collision_free"], flags)
        assert row["seconds"] >= 0
    assert all(row["matches_warmup"] for row in saved["measurements"])


@pytest.mark.parametrize("failure", ["exception", "mismatch", "wrong_shape"])
def test_partial_batches_survive_failure(tmp_path, failure):
    q = np.zeros((8, 2), dtype=np.float32)
    record = save_input(tmp_path, q)
    calls = 0

    def callback(values):
        nonlocal calls
        calls += 1
        if calls == 3:
            if failure == "exception":
                raise RuntimeError("injected callback failure")
            return np.zeros(8 if failure == "mismatch" else 7, dtype=bool)
        return np.ones(8, dtype=bool)

    with pytest.raises((RuntimeError, ValueError)):
        measure_batches(callback, q, tmp_path, record,
                        lambda: atomic(tmp_path / "record.json", record))
    saved = read(tmp_path / "record.json")
    assert len(saved["batches"]) == 3
    assert [row["status"] for row in saved["batches"]] == ["completed", "completed", "failed"]
    assert saved["batches"][-1]["seconds"] >= 0
    if failure == "exception":
        assert "injected callback failure" in saved["batches"][-1]["traceback"]
        assert not saved["batches"][-1]["returned_flags"]
    else:
        last = saved["batches"][-1]
        assert (tmp_path / last["flags_file"]).exists()
        assert file_hash(tmp_path / last["flags_file"]) == last["flags_file_sha256"]


def load_run_without_native_imports(path, namespace):
    """Execute the actual run function, excluding its GPU/model import layer."""
    tree = ast.parse(path.read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "run")
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), "exec"), namespace)
    return namespace["run"]


@pytest.mark.parametrize("driver", ["paper_completion/throughput.py", "throughput_public_iiwa.py"])
def test_driver_failure_manifest_is_terminal(tmp_path, driver):
    source_path = ROOT / "reproduction" / driver

    def injected_failure():
        raise RuntimeError("injected pre-model failure; no GPU or solver")

    namespace = dict(Path=Path, ROOT=tmp_path, __file__=str(source_path), datetime=datetime,
                     np=np, sys=sys, traceback=traceback, atomic=atomic, read=read,
                     digest=digest, identity=lambda: {}, file_hash=file_hash,
                     catch_termination=catch_termination, runtime_versions=injected_failure,
                     runtime_identity=injected_failure, BATCH_SIZE=2000000, REPEATS=5,
                     SEED=250410783, SOURCES=[])
    run = load_run_without_native_imports(source_path, namespace)
    if driver.startswith("paper_completion"):
        base = tmp_path / "source"
        atomic(base / "manifest.json", dict(environment="forest", status="completed",
               implementation_hash=digest({}), config=dict(environment_seeds=list(range(10)))))
        arguments = base
    else:
        arguments = SimpleNamespace(output=tmp_path / "public", asset_dir=tmp_path / "assets",
                                    verification_samples=2000)
    with pytest.raises(RuntimeError, match="injected pre-model failure"):
        run(arguments)
    output_manifests = [p for p in tmp_path.rglob("manifest.json") if p.parent.name != "source"]
    assert len(output_manifests) == 1
    manifest = read(output_manifests[0])
    assert manifest["status"] == "failed"
    assert "injected pre-model failure" in manifest["traceback"]


def test_sigterm_becomes_recordable_python_exception():
    original = signal.getsignal(signal.SIGTERM)
    with pytest.raises(InterruptedError, match="Received signal"):
        with catch_termination():
            signal.raise_signal(signal.SIGTERM)
    assert signal.getsignal(signal.SIGTERM) == original


def test_public_driver_retains_partial_independent_checks(tmp_path):
    source_path = ROOT / "reproduction/throughput_public_iiwa.py"
    queries = 0

    def distances():
        nonlocal queries
        queries += 1
        if queries == 4:
            raise RuntimeError("injected independent checker failure")
        return [SimpleNamespace(distance=0.1)]

    query = SimpleNamespace(ComputeSignedDistancePairwiseClosestPoints=distances)
    port = SimpleNamespace(Eval=lambda context: query)
    scene = SimpleNamespace(get_query_output_port=lambda: port,
                            GetMyContextFromRoot=lambda context: None)
    plant = SimpleNamespace(SetPositions=lambda context, q: None)
    parts = (None, None, plant, None, scene, None, None)
    native = SimpleNamespace(getMinimalPlant=lambda: None)
    model = dict(lower=np.zeros(7), upper=np.ones(7))
    namespace = dict(Path=Path, ROOT=tmp_path, __file__=str(source_path), datetime=datetime,
                     np=np, sys=sys, traceback=traceback, atomic=atomic, file_hash=file_hash,
                     catch_termination=catch_termination, runtime_identity=lambda: {},
                     BATCH_SIZE=2000, REPEATS=5, SEED=250410783, SOURCES=[],
                     fetch_assets=lambda path: None, assemble=lambda path, table: "<mock/>",
                     create_model=lambda text: (parts, native, model), save_input=save_input,
                     measure_batches=measure_batches,
                     csd=SimpleNamespace(CheckCollisionFreeCuda=lambda q, minimal: np.ones(q.shape[1], dtype=bool)))
    run = load_run_without_native_imports(source_path, namespace)
    out = tmp_path / "mock_public_run"
    args = SimpleNamespace(output=out, asset_dir=tmp_path / "assets", verification_samples=2000)
    with pytest.raises(RuntimeError, match="injected independent checker failure"):
        run(args)
    manifest = read(out / "manifest.json")
    assert manifest["status"] == "failed"
    assert manifest["independent_points_completed"] == 3
    assert len(manifest["batches"]) == 6
    assert all(row["status"] == "completed" for row in manifest["batches"])
    assert len(np.load(out / "independent_partial.npz")["indices"]) == 3
    assert manifest["independent_partial_sha256"] == file_hash(out / "independent_partial.npz")
