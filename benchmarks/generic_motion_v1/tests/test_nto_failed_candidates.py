"""Unusable native failure returns stay auditable; no GPU operations."""
import importlib
import json
import time
from types import SimpleNamespace

import numpy as np
import pytest

from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import HERE, load_config
from benchmarks.generic_motion_v1.runners.csdecomp_baseline import (
    _nto, _save_candidate, _validate_optimizer_candidate,
)
from benchmarks.generic_motion_v1.runners.nto_collision_constraint import NativeFirstCollisionSolve
from benchmarks.generic_motion_v1.runners.paper_nto import paper_nto
from benchmarks.generic_motion_v1.runners.paper_nto_refined import paper_nto_refined


@pytest.fixture(scope="module")
def canonical():
    return Model()


def failed_return(monkeypatch, start, field):
    calls = []

    class FailedResult:
        def GetSolution(self, variables):
            shape = np.shape(variables)
            if not shape:
                return np.nan if field == "duration" else (0. if field == "zero_duration" else 1.)
            if len(shape) == 2:
                return (np.full(shape, np.nan) if field == "controls"
                        else np.tile(start[:, None], (1, shape[1])))
            return np.full(shape, np.nan)

        def is_success(self): return False
        def get_solution_result(self): return "failed_nonfinite_fixture"
        def get_optimal_cost(self): return np.nan
        def get_solver_details(self): return SimpleNamespace(status=-1)

    def solve(self, *args, **kwargs):
        calls.append(self)
        self.recovery_state["solver_invocations"] += 1
        return FailedResult()

    monkeypatch.setattr(NativeFirstCollisionSolve, "solve", solve)
    return calls


@pytest.mark.parametrize("adapter", [_nto, paper_nto, paper_nto_refined])
@pytest.mark.parametrize("field", ["controls", "duration", "zero_duration"])
def test_failed_nonfinite_candidate_retains_native_evidence_without_geometry(
        canonical, monkeypatch, tmp_path, adapter, field):
    cfg = load_config()
    case = json.loads((HERE / "manifests/dev.jsonl").read_text().splitlines()[0])
    start = np.asarray(case["q_start"])
    goal = start.copy(); goal[0] += .02
    case.update(q_goal=goal.tolist(), _objective="minimum_time")
    calls = failed_return(monkeypatch, start, field)

    def forbidden(*args):
        pytest.fail("An unusable failed solver return must not query collision geometry")

    model = SimpleNamespace(lower=canonical.lower, upper=canonical.upper,
                            nto_plant=canonical.nto_plant, clearance=forbidden, cuda=forbidden)
    candidate = adapter(np.array([start, goal]), model, case, cfg, tmp_path,
                        time.perf_counter() + 20)
    assert len(calls) == 1
    assert candidate["solver_success"] is False
    assert candidate["solver_status"] == "failed_nonfinite_fixture"
    with np.load(tmp_path / "nto_solver_raw.npz", allow_pickle=False) as raw:
        if field == "zero_duration":
            assert raw["duration"] == 0.
        else:
            assert not np.isfinite(raw[field]).all()
    assert json.loads((tmp_path / "nto_solver_result.json").read_text())["solver_success"] is False
    _save_candidate(tmp_path, candidate)
    assert _validate_optimizer_candidate(candidate, model, case, cfg, tmp_path) is None
    with np.load(tmp_path / "candidate.npz", allow_pickle=False) as raw:
        if field == "zero_duration":
            np.testing.assert_array_equal(raw["knots"], 0.)
        else:
            key = "knots" if field == "duration" else "controls"
            assert not np.isfinite(raw[key]).all()
    validation = json.loads((tmp_path / "validation.json").read_text())
    assert validation["passed"] is False and validation["solver_success"] is False
    assert ("Nonpositive B-spline duration" if field == "zero_duration"
            else "Malformed B-spline") in validation["candidate_sampling_rejection"]
    if adapter is paper_nto_refined:
        record = json.loads((tmp_path / "nto_refinement.json").read_text())
        assert record["stopping_reason"] == "unusable_failed_solver_candidate"
        assert len(record["rounds"]) == record["solver_invocations"] == 1
        assert record["rounds"][0]["geometry_guard"]["geometry_checked"] is False
        assert candidate["refinement_final_geometry_guard_passed"] is False


@pytest.mark.parametrize("error", [ValueError("different geometry failure"), RuntimeError("native fault")])
def test_refined_unknown_geometry_errors_after_failed_return_propagate(monkeypatch, tmp_path, error):
    module = importlib.import_module("benchmarks.generic_motion_v1.runners.paper_nto_refined")
    monkeypatch.setattr(module, "paper_nto", lambda *a, **k: dict(
        kind="bspline", objective="minimum_distance", solver_success=False,
        solver_status="failed", controls=np.zeros((2, 7)),
        knots=np.array([0., 0., 1., 1.]), duration=1., spline_order=2))

    def fail(*args): raise error

    monkeypatch.setattr(module, "_geometry_guard", fail)
    with pytest.raises(type(error), match=str(error)):
        module.paper_nto_refined(np.zeros((2, 7)), object(), {"_objective": "minimum_distance"},
                                load_config(), tmp_path, time.perf_counter() + 10)
    record = json.loads((tmp_path / "nto_refinement.json").read_text())
    assert record["stopping_reason"] == "exception"


@pytest.mark.parametrize("success,error", [
    (True, ValueError("Malformed B-spline")),
    (False, ValueError("Malformed B-spline")),
    (False, ValueError("different geometry failure")),
    (False, RuntimeError("native fault")),
])
def test_common_validation_does_not_swallow_success_or_unknown_faults(monkeypatch, tmp_path, success, error):
    module = importlib.import_module("benchmarks.generic_motion_v1.runners.csdecomp_baseline")

    def fail(*args, **kwargs): raise error

    # A failure after successful sampling is never swallowed, even if an
    # unrelated backend happens to reuse a declared sampler error message.
    monkeypatch.setattr(module, "sample_candidate", lambda *a: {})
    monkeypatch.setattr(module, "validate", fail)
    with pytest.raises(type(error), match=str(error)):
        module._validate_optimizer_candidate({"solver_success": success}, object(), {}, {}, tmp_path)
    assert not (tmp_path / "validation.json").exists()
