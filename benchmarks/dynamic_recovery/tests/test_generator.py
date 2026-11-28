import numpy as np
import pytest

from benchmarks.generic_motion_v1.generator.scene import domain_seed, rng as make_rng
from benchmarks.dynamic_recovery import generator as G
from benchmarks.dynamic_recovery.harness import digest, load_config


def _inside(points, primitive):
    offset = points - np.asarray(primitive["center_m"])
    if primitive["shape"] == "sphere":
        return np.linalg.norm(offset, axis=1) <= primitive["radius_m"]
    local = offset @ G.rz(primitive["yaw_rad"])
    return np.all(np.abs(local) <= np.asarray(primitive["size_m"]) / 2, axis=1)


def test_overlap_test_matches_point_sampling():
    rng = np.random.default_rng(0)
    for _ in range(300):
        shapes = []
        for _ in range(2):
            if rng.random() < 0.7:
                shapes.append(dict(shape="box", center_m=rng.uniform(-0.2, 0.2, 3).tolist(),
                                   size_m=rng.uniform(0.05, 0.3, 3).tolist(), yaw_rad=float(rng.uniform(-np.pi, np.pi))))
            else:
                shapes.append(dict(shape="sphere", center_m=rng.uniform(-0.2, 0.2, 3).tolist(),
                                   radius_m=float(rng.uniform(0.03, 0.15))))
        a, b = shapes
        points = rng.uniform(-0.45, 0.45, (20000, 3))
        sampled = bool(np.any(_inside(points, a) & _inside(points, b)))
        if sampled:  # sampling can miss a thin intersection but never invent one
            assert G.overlap(a, b)
    box = dict(shape="box", center_m=[0, 0, 0], size_m=[0.2, 0.2, 0.2], yaw_rad=0.0)
    diamond = dict(box, center_m=[0.235, 0, 0], yaw_rad=np.pi / 4)  # corner at 0.235 - 0.1*sqrt(2) = 0.094
    assert G.overlap(box, diamond) and not G.overlap(box, dict(diamond, center_m=[0.25, 0, 0]))
    assert not G.overlap(box, dict(box, center_m=[0.21, 0, 0])) and G.overlap(box, dict(box, center_m=[0.21, 0, 0]), 0.02)
    sphere = dict(shape="sphere", center_m=[0.18, 0.18, 0], radius_m=0.1)  # corner distance 0.113
    assert not G.overlap(box, sphere) and G.overlap(box, dict(sphere, radius_m=0.12))


@pytest.mark.parametrize("workspace", G.WORKSPACES)
def test_layouts_are_deterministic_and_split_specific(workspace, model):
    cfg = load_config()
    fixed = G.fixed_primitives(model)
    seed = cfg["generator"]["seed"]
    make = lambda split: G.make_layout(workspace, make_rng(domain_seed(seed, f"{G.BENCHMARK}/{split}/{workspace}/layout", 0)),
                                       cfg, fixed)
    assert digest(make("dev")["static_primitives"]) == digest(make("dev")["static_primitives"])
    assert digest(make("dev")["static_primitives"]) != digest(make("test")["static_primitives"])
    for primitive in make("dev")["static_primitives"]:
        assert G.plausible(primitive, [], fixed)


def test_frozen_task_files_pass_integrity_checks():
    from benchmarks.dynamic_recovery.cli import check_data
    if not (G.DATA / "manifest.json").exists():
        pytest.skip("No generated task data")
    report = check_data()
    assert report["passed"], report
    for split in report["splits"].values():
        assert split["accepted"] > 0
