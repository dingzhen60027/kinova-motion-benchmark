"""Regression checks for the declared 33-sphere online collision contract."""

from pathlib import Path
import numpy as np
from reproduction.common import read
from reproduction.paper_constructed.model import FR3, ASSETS


def protocol():
    return read(Path(__file__).parents[1] / "paper_constructed/config.json")


def test_all_robot_spheres_participate_in_online_collision_queries():
    cfg = protocol()
    empty = FR3(cfg)
    centers = np.random.default_rng(90210).uniform(
        cfg["fr3_scene"]["obstacle_lower"],
        cfg["fr3_scene"]["obstacle_upper"], (20, 3))
    scene = FR3(cfg, centers)
    assert sum(map(len, read(ASSETS / "fr3_33.json")["spheres"].values())) == 33
    assert len(set(empty.geometry_ids)) == 33
    assert len(scene.inspector.GetCollisionCandidates()) == (
        len(empty.inspector.GetCollisionCandidates()) + 33 * 20)
    q = np.random.default_rng(2048).uniform(empty.lower, empty.upper, (2048, 7))
    q = q.astype(np.float32).astype(float)
    np.testing.assert_array_equal(scene.cuda(q), scene.independent(q))


def test_anchored_base_is_not_silently_ignored_online():
    cfg = protocol()
    empty = FR3(cfg)
    q = np.random.default_rng(416).uniform(empty.lower, empty.upper, (4096, 7))
    q = q.astype(np.float32).astype(float)
    free = q[empty.cuda(q)]
    assert len(free), "Model must have a nonempty sampled free space"
    start = free[0:1]
    assert empty.independent(start)[0]
    base = read(ASSETS / "fr3_33.json")["spheres"]["fr3_link0"][0]
    scene = FR3(cfg, [base["center"]])
    assert not scene.cuda(start)[0]
    assert not scene.independent(start)[0]
