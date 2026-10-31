"""Geometry identity and representation regressions; no planning or optimization."""
from copy import deepcopy

import numpy as np
import pytest

from reproduction.common import ROOT, read
from reproduction.environment_reconstruction.model import FixedForest


def inputs():
    protocol = read(ROOT / "reproduction/paper_constructed/config.json")
    scene = read(ROOT / "reproduction/reports/environment_reconstruction/forest_evidence/scene.json")
    return protocol, scene


def test_explicit_geometry_reaches_all_collision_representations_without_rng(monkeypatch):
    import reproduction.forest as forest_module
    def fail_rng(*args, **kwargs):
        raise AssertionError("Fixed input adapter must never sample obstacles")
    monkeypatch.setattr(forest_module, "obstacles", fail_rng)
    protocol, scene = inputs()
    original = deepcopy(scene)
    env = FixedForest(protocol, scene)
    centers = np.asarray(scene["centers"])
    q = np.vstack([centers, np.asarray(scene["reference_path"]), env.start, env.goal])
    analytic = env.base.clearances(q)
    drake = env.drake_clearances(q)
    assert np.allclose(drake, analytic, atol=2e-12, rtol=0)
    assert np.array_equal(env.cuda(q), analytic >= 0)
    assert not env.cuda(centers).any()
    assert np.array_equal(env.independent(q), analytic >= 0)
    assert env.minimal.num_collision_pairs == 0  # Obstacles are online only.
    assert len(env.geometry_ids) == 1
    assert env.base.radius == pytest.approx(.350001)
    assert scene == original


def test_float32_endpoint_contract_preserves_source_and_polygon_validation():
    protocol, scene = inputs()
    env = FixedForest(protocol, scene)
    assert np.array_equal(env.original_start, scene["start"])
    assert np.array_equal(env.original_goal, scene["goal"])
    assert np.array_equal(env.start, np.asarray(scene["start"], np.float32).astype(float))
    assert np.array_equal(env.goal, np.asarray(scene["goal"], np.float32).astype(float))
    assert np.array_equal(env.cfg["start"], env.start)
    assert np.array_equal(env.cfg["goal"], env.goal)
    q = np.array(scene["reference_path"])
    q[[0, -1]] = np.array([env.start, env.goal])
    check = env.base.validate_polygon(q)
    assert check["endpoint_pass"] and check["limits_pass"] and check["collision_free"]
    assert check["endpoint_error"] == 0
    assert check["minimum_clearance"] == pytest.approx(.00266691060003585, abs=1e-12)
    assert env.cuda(np.array([env.start, env.goal])).all()
    assert env.endpoint_representation["goal_l2_rounding_m"] < 3e-7


def test_drake_nto_constraint_uses_fixed_obstacles():
    protocol, scene = inputs()
    env = FixedForest(protocol, scene)
    constraint = env.constraint()
    free = constraint.Eval(env.start)
    blocked = constraint.Eval(np.asarray(scene["centers"][0]))
    assert np.all(free >= constraint.lower_bound()) and np.all(free <= constraint.upper_bound())
    assert np.any(blocked < constraint.lower_bound()) or np.any(blocked > constraint.upper_bound())


def test_reject_contradictory_geometry_aliases_before_building():
    protocol, scene = inputs()
    scene["centers"][0][0] += .1
    with pytest.raises(ValueError, match="aliases disagree"):
        FixedForest(protocol, scene)
