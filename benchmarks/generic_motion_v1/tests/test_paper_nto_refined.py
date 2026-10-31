"""CPU regressions for the distinct refinement policy and its failure records."""
import json
import re
import time
from types import SimpleNamespace

import numpy as np
import pytest

from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import HERE, load_config
from benchmarks.generic_motion_v1.runners.paper_nto import PROFILE as ORIGINAL
from benchmarks.generic_motion_v1.runners.paper_nto_refined import (
    PROFILE, paper_nto_refined, refinement_locations,
)


def test_refinement_uses_interval_minima_and_preserves_global_cap():
    existing = np.array([.25, .5, .75])
    t = np.array([0., .1, .2, .25, .3, .4, .5, .6, .7, .9, 1.])
    d = np.array([-1., .0004, -.01, -1., .002, .003, -1., -.02, -.01, .004, -1.])
    np.testing.assert_array_equal(refinement_locations(existing, t, d), [.2, .6])
    existing = np.arange(1, 600) / 600
    t = (np.r_[0., existing] + np.r_[existing, 1.]) / 2
    added = refinement_locations(existing, t, -np.arange(len(t), dtype=float))
    assert len(existing) + len(added) == 600
    assert added[0] == t[-1]


@pytest.mark.parametrize("objective", ["minimum_distance", "minimum_time"])
def test_real_cpu_refined_solver_artifacts_and_unmodified_original(tmp_path, objective):
    model = Model()
    frozen = json.loads((HERE / "manifests/dev.jsonl").read_text().splitlines()[0])
    start = np.array(frozen["q_start"])
    goal = start.copy()
    goal[0] += .02
    case = dict(q_start=start.tolist(), q_goal=goal.tolist(), _objective=objective)
    candidate = paper_nto_refined(np.array([start, goal]), model, case, load_config(),
                                 tmp_path, time.perf_counter()+30)
    assert candidate["solver_success"]
    assert candidate["refinement_final_geometry_guard_passed"]
    assert candidate["refinement_native_solves"] == 1
    assert candidate["profile"] == PROFILE["name"]
    assert ORIGINAL["clearance_m"] == 0
    assert "snopt_total_iterations" not in ORIGINAL
    record = json.loads((tmp_path / "nto_refinement.json").read_text())
    assert record["final_validation_required"]
    assert record["rounds"][0]["collocation_points"] == 149
    assert (tmp_path / "refinement/round_00/candidate.npz").exists()
    assert (tmp_path / "refinement/round_00/refinement_geometry.npz").exists()
    params = json.loads((tmp_path / "nto_parameters.json").read_text())
    if objective == "minimum_distance":
        assert params["parameters"]["Iterations limit"] == 100000
        assert params["parameters"]["Minor iterations limit"] == 500
        assert re.search(r"Iteration limit\.+\s+100000\b", (tmp_path / "snopt.log").read_text())


def test_expired_refinement_budget_records_failure_before_model_construction(tmp_path):
    model = SimpleNamespace(nto_plant=lambda: pytest.fail("Must not construct expired solve"))
    with pytest.raises(TimeoutError, match="shared online budget"):
        paper_nto_refined(np.zeros((2, 7)), model, {}, load_config(), tmp_path, time.perf_counter()-1)
    record = json.loads((tmp_path / "nto_refinement.json").read_text())
    assert record["stopping_reason"] == "exception"
    assert record["rounds"] == []


def test_actual_failed_native_candidate_is_not_relabelled_as_success(tmp_path):
    model = Model()
    start = np.array(json.loads((HERE / "manifests/dev.jsonl").read_text().splitlines()[0])["q_start"])
    goal = start.copy()
    goal[0] += .02
    upper = model.upper.copy()
    upper[0] = start[0] - .1
    impossible = SimpleNamespace(lower=model.lower, upper=upper, nto_plant=model.nto_plant,
                                 clearance=model.clearance)
    case = dict(q_start=start.tolist(), q_goal=goal.tolist(), _objective="minimum_distance")
    candidate = paper_nto_refined(np.array([start, goal]), impossible, case, load_config(),
                                 tmp_path, time.perf_counter()+30)
    assert not candidate["solver_success"]
    saved = json.loads((tmp_path / "nto_solver_result.json").read_text())
    assert not saved["solver_success"]
    with np.load(tmp_path / "nto_solver_raw.npz") as arrays:
        np.testing.assert_array_equal(arrays["controls"], candidate["controls"])
