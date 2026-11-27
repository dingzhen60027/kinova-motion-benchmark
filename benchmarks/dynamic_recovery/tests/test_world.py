import numpy as np
import pytest

from benchmarks.dynamic_recovery.world import (DynamicObstacle, GeometryChecker, Motion, World, build_simulation,
                                                quantize)


def test_motion_is_c2_and_rests_at_waypoints():
    motion = Motion([0.5, 1.5, 2.0, 3.0], [[0, 0, 0], [0.4, 0, 0.1], [0.4, 0, 0.1], [0, 0.3, 0.1]])
    for t in (0.0, 0.5, 1.5, 1.75, 2.0, 3.0, 9.0):  # before, at waypoints, during the dwell, after
        assert np.allclose(motion.at(t)[1], 0) and np.allclose(motion.at(t)[2], 0)
    h = 1e-6
    for t in np.linspace(0.51, 2.99, 40):
        p0, v0, a0 = motion.at(t - h)
        p1, v1, a1 = motion.at(t + h)
        assert np.allclose((p1 - p0) / (2 * h), motion.at(t)[1], atol=1e-5)
        assert np.allclose((v1 - v0) / (2 * h), motion.at(t)[2], atol=1e-3)
    assert motion.moving_interval() == (0.5, 3.0)
    assert np.isclose(motion.peak_speed(), 1.875 * np.linalg.norm([-0.4, 0.3, 0.0]) / 1.0)  # fastest interval


def test_fast_checker_equals_canonical_clearance(model):
    checker = GeometryChecker(type(model)())
    rng = np.random.default_rng(3)
    for _ in range(40):
        q = rng.uniform(model.lower * 0.8, model.upper * 0.8)
        xyz = quantize(rng.uniform([-0.3, -0.7, 0.05], [0.9, 0.7, 1.1], (50, 3)))
        model.set_voxels(xyz)
        expected = model.clearance(q)[0]
        assert abs(checker.clearance(q, xyz) - expected) < 1e-12
        cut = checker.clearance(q, xyz, cutoff=0.05)
        assert cut == pytest.approx(min(expected, 0.05), abs=1e-12) or expected > 0.05
    model.set_voxels()


def test_mujoco_world_contacts_agree_with_drake(model, tmp_path):
    import mujoco
    from benchmarks.generic_motion_v1.simulation.execution import configuration
    q = np.array([0.0, 0.5, 0.0, 1.8, 0.0, 0.8, 0.0])
    checker = GeometryChecker(type(model)())
    _, translations, _ = checker.poses(q)
    centre = translations[7]
    for offset, colliding in ((0.0, True), (0.35, False)):
        obstacle = DynamicObstacle("o0", dict(shape="box", size_m=[0.08, 0.08, 0.08], yaw_rad=0.0),
                                   Motion([0.0], [centre + [offset, 0, 0]]))
        world = World([dict(shape="box", center_m=[0.6, -0.4, 0.1], size_m=[0.1, 0.1, 0.2], yaw_rad=0.3)], [obstacle])
        metadata = build_simulation(world, tmp_path / f"m{offset}", configuration(),
                                    dict(problem_id="fixture", scene_id="fixture"))
        mj = mujoco.MjModel.from_xml_path(metadata["xml_path"])
        data = mujoco.MjData(mj)
        data.qpos[metadata["qpos_ids"]] = q
        data.mocap_pos[metadata["obstacle_mocap_ids"][0]] = obstacle.motion.at(0.0)[0]
        mujoco.mj_forward(mj, data)
        cover = set(metadata["obstacle_cover_geom_ids"][0])
        hits = [c for c in data.contact[:data.ncon] if c.dist < -2e-6 and {c.geom1, c.geom2} & cover]
        clearance = checker.clearance(q, world.dynamic_xyz(0.0), fixed=False)
        assert bool(hits) == colliding == (clearance < 0)
