"""Actual historical float32 handoff regressions; CPU only, no planning."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import csdecomp as csd
from pydrake.all import HPolyhedron

from benchmarks.generic_motion_v1.runners.native_handoff import native_region_with_anchors

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
    with pytest.raises(ValueError, match="exceeds fixed tolerance"):
        native_region_with_anchors(region, far, 1e-6, tmp_path / "far")
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
