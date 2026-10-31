"""CPU regressions for numerical contracts, evidence gates and solver adapters."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from research_platform import backend


def environment(cap=200000):
    return SimpleNamespace(dimension=2, cfg={"validation_max_samples": cap,
        "validation_step": .002, "validation_minimum_samples": 2001})


def polygon(points):
    return {"kind": "polygon", "points": np.asarray(points, dtype=float), "segment_regions": [0]*(len(points)-1)}


def test_preflight_retains_normal_paths_and_rejects_nonfinite_values():
    candidate = polygon([[0, 0], [1, 0], [1, 1]])
    before = deepcopy(candidate)
    audit = backend.candidate_preflight(candidate, environment())
    assert audit["expected_samples"] == 1001
    assert np.array_equal(candidate["points"], before["points"])
    candidate["points"][1, 0] = np.nan
    with pytest.raises(ValueError, match="finite configurations"):
        backend.candidate_preflight(candidate, environment())


@pytest.mark.parametrize("duration", [0., -1., np.inf, np.nan])
def test_bezier_nonpositive_nonfinite_duration_rejected_before_queries(duration):
    candidate = {"kind": "bezier", "controls": np.zeros((1, 4, 2)),
                 "durations": np.array([duration]), "segment_regions": [0]}
    with pytest.raises(ValueError, match="strictly positive durations"):
        backend.candidate_preflight(candidate, environment())


def test_bezier_sample_cap_prevents_uncapped_legacy_allocation():
    candidate = {"kind": "bezier", "controls": np.array([[[0, 0], [100, 0], [100, 0], [100, 0]]]),
                 "durations": np.array([1.]), "segment_regions": [0]}
    with pytest.raises(ValueError, match="exceeding the declared cap"):
        backend.candidate_preflight(candidate, environment(cap=10000))


def test_bspline_duration_and_knot_contract():
    candidate = {"kind": "bspline", "controls": np.array([[0, 0], [1, 1]]),
                 "spline_order": 2, "knots": np.array([0., 0., 1., 1.]), "duration": 1.}
    assert backend.candidate_preflight(candidate, environment())["expected_samples"] == 2001
    candidate["duration"] = 2.
    with pytest.raises(ValueError, match="disagrees"):
        backend.candidate_preflight(candidate, environment())


def test_region_alignment_guard_runs_before_legacy_validation():
    candidate = polygon([[0, 0], [1, 1]])
    candidate["segment_regions"] = [3]
    with pytest.raises(ValueError, match="nonexistent region"):
        backend.validate_candidate(candidate, environment(), [object()])


def test_failed_shared_seed_blocks_nto_without_model_construction(tmp_path, monkeypatch):
    plan = tmp_path / "plan"
    plan.mkdir()
    (plan / "validation.json").write_text('{"passed": false}')
    def forbidden(*args, **kwargs):
        raise AssertionError("Blocked method must not initialize geometry")
    monkeypatch.setattr(backend, "_environment", forbidden)
    result = backend.execute_job({"kind": "method", "method": "NTO_distance", "case_id": "case"},
        {"scene_definition": {"kind": "forest"}}, {"plan": plan}, tmp_path / "attempt")
    assert result["status"] == "blocked"
    assert not result["solver_success"] and not result["validation_passed"]
    assert (plan / "validation.json").read_text() == '{"passed": false}'


def test_attempt_cannot_write_inside_dependency(tmp_path):
    dependency = tmp_path / "dependency"
    dependency.mkdir()
    with pytest.raises(ValueError, match="immutable dependency"):
        backend.execute_job({"kind": "method"}, {}, {"plan": dependency}, dependency / "child")
    assert not (dependency / "child").exists()


def test_actual_lgcs_model_strict_parameters_budget_and_global_isolation():
    from pydrake.all import HPolyhedron
    from reproduction.paper_constructed import optimization
    import gurobipy
    original_import = __import__("builtins").__import__
    original_model, original_solver = gurobipy.Model, optimization.lgcs
    start, goal = np.array([-.5, -.5]), np.array([.5, .5])
    result = backend.lgcs_configured([HPolyhedron.MakeBox(-np.ones(2), np.ones(2))], start, goal, 7.)
    assert result["solver_success"]
    assert result["objective"] == pytest.approx(np.sqrt(2), abs=2e-6)
    assert result["solver_parameters"]["TimeLimit"] == 7.
    assert result["solver_parameters"]["IntFeasTol"] == 1e-9
    assert result["solver_quality"]["IntVio"] <= 1e-9
    assert gurobipy.Model is original_model and optimization.lgcs is original_solver
    assert __import__("builtins").__import__ is original_import


def test_actual_scsto_retains_corner_without_mutating_author_module():
    import importlib
    from pydrake.all import HPolyhedron
    from reproduction.alignment_diagnosis.geometric_solvers import scsto_preserve_knots
    module = importlib.import_module("scsplanning.biconvex")
    original = module.polygonal
    regions = [HPolyhedron.MakeBox(np.array([0., 0.]), np.array([1., .2])),
               HPolyhedron.MakeBox(np.array([.8, 0.]), np.array([1., 1.]))]
    result = scsto_preserve_knots(regions, np.array([0., 0.]), np.array([1., 1.]),
        {"velocity_limit": 1., "acceleration_limit": 1., "bezier_degree": 6,
         "scsto_relative_tolerance": .01, "feasibility_tolerance": 1e-5})
    assert result["solver_success"] and result["removed_knot_count"] == 0
    assert result["initial_region_residual"] <= 1e-5
    # Clarabel's scalar time initialization is numerical, not an exact symbolic
    # endpoint operation. Match the author's 1e-8 containment precision.
    assert result["initial_transition_error"] <= 1e-8
    assert result["controls"].shape[0] == 2
    assert module.polygonal is original


def test_distance_metrics_exclude_auxiliary_collision_witness_and_duration():
    validation = {"cost": 2., "duration": 1., "_samples": {
        "time": np.array([0., .5, 1., np.nan]),
        "q": np.array([[0, 0], [1, 0], [1, 1], [99, 99]])}}
    result = backend._metrics({}, validation, "NTO_distance")
    assert result["cost"] == result["path_length"] == 2.
    assert result["cost_type"] == "distance" and result["duration"] is None


def test_order_two_bspline_length_uses_knots_not_shortcut_sample_polygon():
    candidate = {"kind": "bspline", "spline_order": 2,
                 "controls": np.array([[0, 0], [1, 0], [1, 1]])}
    validation = {"cost": np.sqrt(2), "duration": 1., "_samples": {
        "time": np.array([0., 1.]), "q": np.array([[0, 0], [1, 1]])}}
    result = backend._metrics(candidate, validation, "NTO_distance")
    assert result["cost"] == result["path_length"] == 2.
    assert result["path_length_source"] == "exact_order_two_bspline_length"


def test_forest_offline_map_follows_shifted_non_square_domain():
    extent, center = backend.forest_map_geometry({"domain_lower": [1., 2.],
        "domain_upper": [4., 10.], "offline_voxel_side": .06})
    assert np.array_equal(extent, [3., 8., .06])
    assert np.array_equal(center, [2.5, 6., 0.])
