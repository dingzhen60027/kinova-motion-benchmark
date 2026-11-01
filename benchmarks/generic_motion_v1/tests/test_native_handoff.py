"""Actual historical float32 handoff regressions; CPU only, no planning."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import csdecomp as csd
from pydrake.all import HPolyhedron

from benchmarks.generic_motion_v1.runners.native_handoff import NativeHandoffError, native_region_with_anchors

BASE = Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def no_gpu(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("Native handoff must not invoke GPU")
    for name in ("CudaEdgeInflator", "EditRegionsCuda", "CheckCollisionFreeCuda", "CheckCollisionFreeVoxelsCuda"):
        monkeypatch.setattr(csd, name, forbidden)


@pytest.mark.parametrize("job", [
    "dev-route_choice-002-p01-EI_ZO-minimum_distance-5b8f2947",
    "dev-route_choice-004-p00-EI_ZO-minimum_distance-bb98a3c8"])
def test_historical_recovery_failures_are_repaired_without_moving_anchors(tmp_path, job):
    source = BASE / "results/dev_eizo_v2/jobs" / job / "attempts/0001"
    result = json.loads((source / "result.json").read_text())
    assert result["status"] == "solver_failed" and "not contained" in result["exception"]
    path = np.load(source / "path.npy")
    failures_before = 0
    with np.load(source / "regions.npz") as data:
        for index, anchors in enumerate(zip(path[:-1], path[1:])):
            A, b = data[f"A_{index:04d}"], data[f"b_{index:04d}"]
            anchors = np.array(anchors)
            untouched = anchors.copy()
            old = csd.HPolyhedron(A, b)
            failures_before += sum(not old.PointInSet(q, 1e-9) for q in anchors)
            fixed = native_region_with_anchors(HPolyhedron(A, b), anchors, 1e-6, tmp_path / str(index))
            assert all(fixed.PointInSet(q, 0.) for q in anchors)
            np.testing.assert_array_equal(anchors, untouched)
            np.testing.assert_array_equal(fixed.A(), np.asarray(A, np.float32))
            receipt = json.loads((tmp_path / str(index) / "handoff.json").read_text())
            assert receipt["status"] == "passed"
            assert receipt["maximum_normalized_correction"] <= 1e-6
            assert receipt["maximum_total_normalized_offset_change_from_input"] <= 1e-6
            with np.load(tmp_path / str(index) / "original.npz") as saved:
                np.testing.assert_array_equal(saved["b"], b)
                np.testing.assert_array_equal(saved["anchors"], anchors)
    assert failures_before > 0, "The actual historical defect must be reproduced first"


def test_already_interior_region_does_not_change(tmp_path):
    region = HPolyhedron.MakeBox(-np.ones(7), np.ones(7))
    result = native_region_with_anchors(region, np.zeros((2, 7)), 0., tmp_path)
    np.testing.assert_array_equal(result.A(), region.A())
    np.testing.assert_array_equal(result.b(), region.b())
    assert json.loads((tmp_path / "handoff.json").read_text())["corrected_planes"] == 0


@pytest.mark.parametrize("scale", [1., 1e-4, 1e4])
def test_normalized_guard_cannot_be_evaded_by_row_scaling(tmp_path, scale):
    A = np.vstack([np.eye(7), -np.eye(7)]) * scale
    region = HPolyhedron(A, np.ones(14) * scale)
    near = np.zeros((2, 7)); near[1, 0] = 1. + 2e-7
    result = native_region_with_anchors(region, near, 1e-6, tmp_path / "near")
    assert all(result.PointInSet(q, 0.) for q in near)
    far = near.copy(); far[1, 0] = 1.01
    with pytest.raises(NativeHandoffError, match="exceeds fixed tolerance") as error:
        native_region_with_anchors(region, far, 1e-6, tmp_path / "far")
    assert error.value.code == "correction_exceeds_tolerance"
    assert error.value.evidence_path == str(tmp_path / "far/handoff.json")
    assert json.loads((tmp_path / "far/handoff.json").read_text())["status"] == "rejected"


@pytest.mark.parametrize("tol", [np.nan, np.inf, -1., True, [1e-6]])
def test_invalid_tolerance_rejected(tmp_path, tol):
    with pytest.raises(ValueError, match="tolerance"):
        native_region_with_anchors(HPolyhedron.MakeBox(-np.ones(7), np.ones(7)), np.zeros((2, 7)), tol, tmp_path)
    assert json.loads((tmp_path / "handoff.json").read_text())["status"] == "rejected"


@pytest.mark.parametrize("anchors", [np.zeros(7), np.zeros((0, 7)), np.zeros((2, 6)),
                                     np.full((2, 7), np.nan), np.full((2, 7), 1e300)])
def test_invalid_anchor_arrays_rejected(tmp_path, anchors):
    with pytest.raises(ValueError):
        native_region_with_anchors(HPolyhedron.MakeBox(-np.ones(7), np.ones(7)), anchors, 1e-6, tmp_path)


def test_zero_normal_infeasibility_cannot_be_repaired(tmp_path):
    region = SimpleNamespace(A=lambda: np.zeros((1, 7)), b=lambda: np.array([-1.]))
    with pytest.raises(ValueError, match="zero-normal"):
        native_region_with_anchors(region, np.zeros((2, 7)), 1e-6, tmp_path)


def test_coefficient_quantization_underflow_rejected(tmp_path):
    A = np.eye(7) * 1e-300
    region = SimpleNamespace(A=lambda: A, b=lambda: np.ones(7))
    with pytest.raises(ValueError, match="unrepresentable"):
        native_region_with_anchors(region, np.zeros((2, 7)), 1e-6, tmp_path)


def _cancellation_fixture():
    source = (BASE / "results/repaired_dev_all_seeds/jobs/"
        "dev-dense_clutter-002-p01-LGCS-minimum_distance-83e6f638/attempts/0001/"
        "recovery/000/native_handoff/0005")
    with np.load(source / "original.npz") as data:
        arrays = {key: data[key] for key in ("A", "b", "anchors")}
    return source, arrays


def test_real_sixteen_ulp_failure_uses_minimal_passing_offset_inside_same_bound(tmp_path):
    source, arrays = _cancellation_fixture()
    old_record = json.loads((source / "handoff.json").read_text())
    assert old_record["status"] == "rejected" and old_record["native_refinement_rounds"] == 16
    with np.load(source / "corrected.npz") as old:
        failed = csd.HPolyhedron(old["A"], old["b"])
        assert not all(failed.PointInSet(q, 0.) for q in old["anchors"])
    untouched = {key: value.copy() for key, value in arrays.items()}
    repaired = native_region_with_anchors(HPolyhedron(arrays["A"], arrays["b"]),
        arrays["anchors"], 1e-6, tmp_path)
    assert all(repaired.PointInSet(q, 0.) for q in arrays["anchors"])
    record = json.loads((tmp_path / "handoff.json").read_text())
    assert record["status"] == "passed"
    assert record["maximum_total_normalized_offset_change_from_input"] < 1.8e-7
    assert record["maximum_normalized_correction"] <= 1e-6
    search, = record["bounded_native_offset_search"]
    assert search["row"] == 14 and search["native_predicate_calls"] <= 34
    # The predecessor fails with the same complete A shape and other b values.
    preceding = repaired.b().copy(); preceding[search["row"]] = search["preceding_offset"]
    assert not all(csd.HPolyhedron(repaired.A(), preceding).PointInSet(q, 0.) for q in arrays["anchors"])
    for key in arrays:
        np.testing.assert_array_equal(arrays[key], untouched[key])
    np.testing.assert_array_equal(repaired.A(), np.asarray(arrays["A"], np.float32))


def test_real_cancellation_cannot_escape_a_tighter_bound(tmp_path):
    _, arrays = _cancellation_fixture()
    with pytest.raises(NativeHandoffError, match="cannot satisfy the fixed normalized correction bound") as error:
        native_region_with_anchors(HPolyhedron(arrays["A"], arrays["b"]),
            arrays["anchors"], 1.3e-7, tmp_path)
    record = json.loads((tmp_path / "handoff.json").read_text())
    assert record["status"] == "rejected"
    assert record["error_code"] == error.value.code == "bounded_predicate_rejected"
    assert error.value.evidence_path == str(tmp_path / "handoff.json")
    assert record["error"] == repr(error.value)
    assert record["maximum_total_normalized_offset_change_from_input"] <= 1.3e-7
    assert record["bounded_native_offset_search"][0]["status"] == "no_passing_offset_within_bound"


def test_recorded_normal_handoffs_are_byte_identical_and_need_no_fallback(tmp_path):
    source, _ = _cancellation_fixture()
    for index in range(5):
        original = source.parent / f"{index:04d}"
        with np.load(original / "original.npz") as data:
            repaired = native_region_with_anchors(HPolyhedron(data["A"], data["b"]),
                data["anchors"], 1e-6, tmp_path / str(index))
        with np.load(original / "corrected.npz") as old:
            np.testing.assert_array_equal(repaired.A(), old["A"])
            np.testing.assert_array_equal(repaired.b(), old["b"])
        assert json.loads((tmp_path / str(index) / "handoff.json").read_text()) == json.loads((original / "handoff.json").read_text())


@pytest.mark.parametrize('target', [0., 1., -1.])
def test_bounded_search_terminates_at_least_float32_offset_for_each_sign(tmp_path, monkeypatch, target):
    # A monotone predicate with an explicit known accumulation discrepancy
    # exercises near-zero/subnormal and both float32 integer-order branches.
    threshold = np.float32(target + 2.5e-6)

    class MonotoneNative:
        def __init__(self, A, b):
            self._A, self._b = np.array(A, dtype=np.float32), np.array(b, dtype=np.float32)
        def PointInSet(self, q, tol):
            assert tol == 0.
            return self._b[0] >= threshold
        def A(self):
            return self._A
        def b(self):
            return self._b

    monkeypatch.setattr(csd, 'HPolyhedron', MonotoneNative)
    A = np.zeros((1, 7)); A[0, 0] = 1.
    anchors = np.zeros((2, 7)); anchors[:, 0] = target
    fixed = native_region_with_anchors(HPolyhedron(A, np.array([target])), anchors, 1e-5, tmp_path)
    assert fixed.b()[0] == threshold
    record = json.loads((tmp_path/'handoff.json').read_text())
    assert record['maximum_total_normalized_offset_change_from_input'] <= 1e-5
    search, = record['bounded_native_offset_search']
    assert search['native_predicate_calls'] <= 34
    assert np.float32(search['preceding_offset']) < threshold


def test_later_row_rejection_preserves_completed_bounded_correction_evidence(tmp_path, monkeypatch):
    thresholds = np.array([2e-7, 2e-6], dtype=np.float32)

    class MonotoneNative:
        def __init__(self, A, b):
            self.b = np.asarray(b, dtype=np.float32)
        def PointInSet(self, q, tol):
            return bool(np.all(self.b >= thresholds))

    monkeypatch.setattr(csd, 'HPolyhedron', MonotoneNative)
    A = np.eye(7)[:2]
    with pytest.raises(NativeHandoffError, match='cannot satisfy'):
        native_region_with_anchors(HPolyhedron(A, np.zeros(2)), np.zeros((2, 7)), 1e-6, tmp_path)
    record = json.loads((tmp_path/'handoff.json').read_text())
    first, second = record['bounded_native_offset_search']
    assert first['status'] == 'minimal_passing_offset_found'
    assert second['status'] == 'no_passing_offset_within_bound'
    with np.load(tmp_path/'corrected.npz') as saved:
        assert saved['b'][0] == thresholds[0] and saved['b'][1] < thresholds[1]
        assert float(np.max(saved['normalized_total_offset_changes'])) == record['maximum_total_normalized_offset_change_from_input']
        assert record['maximum_total_normalized_offset_change_from_input'] <= 1e-6
