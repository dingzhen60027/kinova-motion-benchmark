"""Physical/model and genuine solver checks for the new paper-parameter run."""

import numpy as np
from scipy.spatial.transform import Rotation
from pydrake.all import HPolyhedron
from reproduction.common import ROOT, read
from reproduction.paper_protocol.model import FR3, ForestEnvironment
from reproduction.paper_protocol.optimization import lscs, lgcs, nto
from reproduction.paper_protocol.validation import validate


def protocol():
    return read(ROOT / "reproduction/paper_protocol/config.json")


def test_fr3_cuda_matches_independent_drake_with_dynamic_obstacles():
    cfg = protocol()
    s = cfg["fr3_scene"]
    rng = np.random.default_rng(392)
    centers = rng.uniform(s["obstacle_lower"], s["obstacle_upper"], (20, 3))
    env = FR3(cfg, centers)
    points = rng.uniform(env.lower, env.upper, (10000, 7))
    native = env.cuda(points)
    independent = env.independent(points)
    assert native.any() and (~native).any()
    assert np.array_equal(native, independent)
    # The optimizer must see the dynamic obstacles too, rather than only the
    # static robot/table plant used by the native offline roadmap.
    static_free = FR3(cfg).cuda(points)
    dynamic_collision = static_free & ~native
    assert dynamic_collision.any()
    constraint = env.constraint()
    free_value = constraint.Eval(points[native][0])
    blocked_value = constraint.Eval(points[dynamic_collision][0])
    assert np.all(free_value >= constraint.lower_bound())
    assert np.all(free_value <= constraint.upper_bound())
    assert np.any(blocked_value < constraint.lower_bound()) or np.any(
        blocked_value > constraint.upper_bound()
    )


def test_fr3_official_kinematics_against_independent_matrix_chain():
    env = FR3(protocol())
    asset = read(ROOT / "reproduction/paper_protocol/assets/fr3_33.json")
    for q in np.random.default_rng(26).uniform(env.lower, env.upper, (30, 7)):
        x = np.eye(4)
        for i in range(1, 9):
            a = asset["kinematics"][f"joint{i}"]["kinematic"]
            fixed = np.eye(4)
            fixed[:3, :3] = Rotation.from_euler(
                "xyz", [a["roll"], a["pitch"], a["yaw"]]
            ).as_matrix()
            fixed[:3, 3] = [a["x"], a["y"], a["z"]]
            x = x @ fixed
            if i < 8:
                turn = np.eye(4)
                turn[:3, :3] = Rotation.from_rotvec([0, 0, q[i - 1]]).as_matrix()
                x = x @ turn
        hand = np.eye(4)
        hand[:3, :3] = Rotation.from_rotvec([0, 0, -np.pi / 4]).as_matrix()
        x = x @ hand
        tool = np.eye(4)
        tool[2, 3] = 0.1034
        x = x @ tool
        assert np.allclose(x, env.pose(q), atol=1e-12, rtol=0)


def test_seven_dimensional_conic_planners_attain_known_optimum():
    region = HPolyhedron.MakeBox(-np.ones(7) * 2, np.ones(7) * 2)
    start = -np.ones(7) * 0.5
    goal = np.ones(7) * 0.5
    for solver in [lscs, lgcs]:
        c = solver([region], start, goal)
        assert c["solver_success"] and abs(c["objective"] - np.sqrt(7)) < 1e-5


def test_genuine_drake_nto_constraint_and_ipopt_on_fr3():
    env = FR3(protocol())
    # A free-space optimizer test requires a free segment, rather than assuming
    # that a familiar Panda home pose is free for this FR3 sphere approximation.
    points = np.random.default_rng(572).uniform(
        env.lower + 0.1, env.upper - 0.1, (1024, 7)
    )
    env.start = points[env.cuda(points)][0]
    env.goal = env.start.copy()
    env.goal[0] += 0.05
    assert env.independent(np.linspace(env.start, env.goal, 100)).all()
    env.cfg.update(start=env.start, goal=env.goal)
    for timed in [False, True]:
        c = nto(np.array([env.start, env.goal]), env, timed)
        assert c["solver_success"] and validate(c, env)["passed"]
        assert c["solver"] == ("IPOPT" if timed else "SNOPT")


def test_forest_real_distance_constraint_finds_free_space_solution():
    env = ForestEnvironment(protocol(), 1)
    env.start = np.array([-4.0, -4.0])
    env.goal = np.array([-4.0, 4.0])
    env.cfg.update(start=env.start, goal=env.goal)
    c = nto(np.array([env.start, env.goal]), env, False)
    result = validate(c, env)
    assert c["solver_success"] and result["passed"] and abs(result["cost"] - 8) < 1e-5
