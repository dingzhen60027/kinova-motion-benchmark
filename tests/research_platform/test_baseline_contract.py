"""Cross-component contracts for comparable strengthened baselines."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from research_platform import backend
from research_platform.config import ConfigError, STRONG_NTO_DEFAULTS, validate_config, resolve_config
from research_platform.reporting import METHOD_OBJECTIVES, _row


def config():
    return dict(version=1, name="baseline_contract", environment="forest",
                environment_seeds=[0], roadmap_seeds=[0], roadmap_sizes=[200],
                methods=["NTO_distance_strong", "NTO_time_strong"], scene_profile="figure6")


def test_strong_defaults_are_explicit_frozen_and_do_not_change_legacy_identity():
    raw = config()
    before = deepcopy(raw)
    canonical = validate_config(raw)
    assert raw == before and canonical["strong_nto"] == STRONG_NTO_DEFAULTS
    resolved = resolve_config(raw)
    assert resolved["protocol"]["strong_nto"] == STRONG_NTO_DEFAULTS
    raw["strong_nto"] = {"minimum_control_points": 20}
    assert resolve_config(raw)["resolved_sha256"] != resolved["resolved_sha256"]
    raw = config()
    raw["methods"] = ["NTO_distance", "NTO_time"]
    assert "strong_nto" not in validate_config(raw)
    raw["strong_nto"] = {}
    with pytest.raises(ConfigError, match="require"):
        validate_config(raw)


@pytest.mark.parametrize("options", [
    {"clearence": .001}, {"clearance": -1}, {"clearance": True},
    {"minimum_control_points": 3}, {"maximum_control_points": 513},
    {"minimum_control_points": 65, "maximum_control_points": 64},
    {"initial_collocation_points": 1001, "maximum_collocation_points": 1000},
    {"maximum_refinements": -1}, {"maximum_new_points": 0},
])
def test_strong_invalid_or_unbounded_options_fail_before_work(options):
    raw = config()
    raw["strong_nto"] = options
    with pytest.raises(ConfigError):
        validate_config(raw)


def numerical_environment():
    return SimpleNamespace(kind="fr3", dimension=2, start=np.array([0., 0.]),
        goal=np.array([1., 0.]), lower=np.array([-2., -2.]), upper=np.array([2., 2.]),
        cfg={"validation_step": .01, "validation_minimum_samples": 101,
             "feasibility_tolerance": 1e-5, "endpoint_tolerance": 1e-5,
             "velocity_limit": 1., "acceleration_limit": 1.},
        cuda=lambda q: np.ones(len(q), dtype=bool),
        independent=lambda q: np.ones(len(q), dtype=bool))


def test_distance_alias_does_not_impose_stationary_endpoint_constraints():
    candidate = dict(kind="bspline", method="NTO_distance_strong", spline_order=2,
        controls=np.array([[0., 0.], [1., 0.]]), knots=np.array([0., 0., 1., 1.]), duration=1.)
    answer = backend.validate_candidate(candidate, numerical_environment())
    assert answer["passed"] and answer["endpoint_velocity"] == 1.
    assert candidate["method"] == "NTO_distance_strong"
    metric = backend._metrics(candidate, answer, candidate["method"])
    assert metric["cost_type"] == "distance" and metric["duration"] is None
    assert metric["cost"] == pytest.approx(1.)


@pytest.mark.parametrize("method", ["NTO_time", "NTO_time_strong"])
@pytest.mark.parametrize("duration,passed", [(1., False), (4., True)])
def test_time_alias_never_skips_derivative_acceptance(method, duration, passed):
    candidate = dict(kind="bspline", method=method, spline_order=4,
        controls=np.array([[0., 0.], [0., 0.], [1., 0.], [1., 0.]]),
        knots=np.array([0.]*4+[duration]*4), duration=duration)
    answer = backend.validate_candidate(candidate, numerical_environment())
    assert answer["passed"] == passed and answer["derivative_pass"] == passed
    assert backend._metrics(candidate, answer, method)["cost_type"] == "time"
    assert backend._metrics(candidate, answer, method)["duration"] == duration


@pytest.mark.parametrize("method,objective", [("NTO_distance_strong", "distance"), ("NTO_time_strong", "time")])
def test_pending_and_failed_new_methods_keep_objective_and_denominator(method, objective):
    assert METHOD_OBJECTIVES[method] == objective
    missing = backend._metrics({}, None, method)
    assert missing["cost"] is None and missing["cost_type"] == objective
    manifest = {"config": {"environment": "forest"}}
    job = {"id": "job", "case_id": "case", "method": method}
    for result in [{}, {"status": "budget_exhausted", "solver_success": True, "validation_passed": True}]:
        row = _row(manifest, job, result)
        assert row["cost_type"] == objective and not row["strict_success"]


def test_continuous_witness_is_shared_with_recovery_not_only_strong_nto():
    env = numerical_environment()
    env.kind = "forest"
    centers, radius = np.array([[.4, 0.]]), .000001
    env.cfg.update(obstacle_radius=radius/2, point_radius=radius/2,
                   validation_step=.5, validation_minimum_samples=2)
    env.base = SimpleNamespace(centers=centers, radius=radius)
    env.cuda = env.independent = lambda q: np.linalg.norm(q-centers[0], axis=1) >= radius
    controls = np.array([[[0., 0.], [0., 0.], [1., 0.], [1., 0.]]])
    candidate = dict(kind="bezier", method="SCSTO", controls=controls,
                     durations=np.array([4.]), segment_regions=[0])
    check = backend.validate_candidate(candidate, env)
    assert not check["passed"] and not check["collision_free"]
    assert check["normalized_collision_parameters"]
    assert len(check["collision_points"]) >= 1
    assert len(check["collision_points"]) == len(check["collision_regions"])
    assert np.isnan(check["_samples"]["time"][-1])
    assert check["minimum_clearance"] < 0
