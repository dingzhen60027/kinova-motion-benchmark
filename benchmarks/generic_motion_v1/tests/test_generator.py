"""CPU tests of deterministic inputs and conservative obstacle coverage."""
from __future__ import annotations

import importlib
import json
from pathlib import Path

import numpy as np
import pytest

from benchmarks.generic_motion_v1.generator.endpoints import generate_endpoints
from benchmarks.generic_motion_v1.generator.manifest import DEFAULT_CONFIG, generate
from benchmarks.generic_motion_v1.generator.scene import FAMILIES, domain_seed, generate_scene
from benchmarks.generic_motion_v1.generator.voxelize import rotation_z, voxelize


class FakeModel:
    """Deterministic geometry-free stand-in, only for CPU schema unit tests."""
    lower = np.full(7, -np.pi)
    upper = np.full(7, np.pi)
    robot_sha256 = "unit-test-robot"
    collision_geometry_sha256 = "unit-test-geometry"
    contact_band_m = 2e-6

    def set_voxels(self, xyz=None, radius=.04):
        self.voxels_xyz = np.asarray(xyz or []).reshape(-1, 3)
        return self

    def fixed_geometry_sphere_clearance(self, xyz, radius=.04):
        return np.ones(len(xyz))

    def cuda(self, q):
        return np.ones(len(np.atleast_2d(q)), dtype=bool)

    def clearance(self, q):
        return np.ones(len(np.atleast_2d(q)))

    def tcp(self, q):
        return np.asarray(q)[:, :3] / 10


@pytest.fixture
def config():
    return json.loads(DEFAULT_CONFIG.read_text())


@pytest.mark.parametrize("shape", ["box", "sphere"])
def test_conservative_voxel_cover(shape):
    random = np.random.default_rng(72)
    obj = dict(obstacle_id="o", shape=shape, center_m=[.234, -.345, .7])
    if shape == "box":
        obj.update(size_m=[.113, .073, .271], yaw_rad=1.37)
        samples = random.uniform(-.5, .5, (3000, 3)) * obj["size_m"]
        samples = samples @ rotation_z(obj["yaw_rad"]).T
    else:
        obj["radius_m"] = .087
        direction = random.normal(size=(3000, 3))
        direction /= np.linalg.norm(direction, axis=1, keepdims=True)
        # Include source-sphere boundary as the most difficult coverage points.
        samples = direction * obj["radius_m"]
    samples += obj["center_m"]
    xyz, certificate = voxelize([obj])
    distances = np.linalg.norm(samples[:, None] - xyz[None], axis=2)
    assert np.max(np.min(distances, axis=1)) <= .04
    assert certificate["obstacles"][0]["coverage_margin_m"] > .005
    assert np.array_equal(xyz, xyz.astype(np.float32).astype(float))


def test_empty_voxel_cover():
    xyz, report = voxelize([])
    assert xyz.shape == (0, 3)
    assert report["voxel_count"] == 0


@pytest.mark.parametrize("family", [*FAMILIES, "open_control"])
def test_scene_determinism_and_domain_separation(config, family):
    model = FakeModel()
    seed = domain_seed(123, "scene", "dev", family, 0)
    first = generate_scene(family, seed, config, model)
    assert first == generate_scene(family, seed, config, model)
    if family != "open_control":
        other = generate_scene(family, domain_seed(123, "scene", "test", family, 0), config, model)
        assert first["obstacle_geometry"] != other["obstacle_geometry"]
    assert first["generation_status"] == "valid"


@pytest.mark.parametrize("distance_bin", ["short", "medium", "long"])
def test_endpoint_all_joints_and_distance_bin(config, distance_bin):
    row, arrays = generate_endpoints(FakeModel(), 123, distance_bin, config)
    assert row["validation_status"] == "valid_input"
    low, high = config["normalized_distance_bins"][distance_bin]
    assert low <= row["normalized_start_goal_joint_distance"] < high
    assert len(row["q_start"]) == len(row["q_goal"]) == 7
    assert not row["straight_line_collision"]
    assert np.all(np.std(arrays["endpoint_proposal_q"], axis=0) > 0)
    assert row["problem_feasibility"] == "feasibility_unknown"


def test_invalid_endpoints_preserved(config):
    model = FakeModel()
    model.cuda = lambda q: np.zeros(len(np.atleast_2d(q)), dtype=bool)
    config["endpoint_max_attempts"] = 16
    row, arrays = generate_endpoints(model, 1, "short", config)
    assert row["validation_status"] == "invalid_endpoint"
    assert row["q_start"] is None and row["q_goal"] is None
    assert row["endpoint_sampling"]["collision_or_clearance_rejections"] == 16
    assert len(arrays["endpoint_proposal_q"]) == 16


def test_split_sizes_and_native_seed_contract(config):
    assert {split: (4 * cfg["main_layouts_per_family"] + cfg["open_layouts"]) * cfg["pairs_per_layout"]
            for split, cfg in config["splits"].items()} == {"smoke": 10, "dev": 60, "test": 420}
    assert len({domain_seed(42, domain, "dev") for domain in ("scene", "endpoint", "method", "solver")}) == 4


def test_test_generation_is_locked(tmp_path):
    with pytest.raises(RuntimeError, match="locked"):
        generate("test", 42, tmp_path, model=FakeModel())


def test_manifest_byte_determinism_and_immutable_output(tmp_path, monkeypatch):
    module = importlib.import_module("benchmarks.generic_motion_v1.generator.manifest")
    workspace = dict(moving_collision_geometry_min=[-2, -2, -2], moving_collision_geometry_max=[2, 2, 2])
    monkeypatch.setattr(module, "_require_model_audit", lambda model: {"workspace": workspace})
    first = generate("smoke", 12, tmp_path / "a", model=FakeModel(), source_code_commit="f" * 40)
    second = generate("smoke", 12, tmp_path / "b", model=FakeModel(), source_code_commit="f" * 40)
    assert first["manifest_sha256"] == second["manifest_sha256"]
    assert first["files_sha256"] == second["files_sha256"]
    assert first["problem_count"] == 10 and first["scene_count"] == 5
    rows = [json.loads(line) for line in Path(first["manifest_path"]).read_text().splitlines()]
    assert {r["paired_seeds"][0]["drm_seed"] for r in rows} == {0}
    assert all(s["method_seed"] < 2**32 and s["solver_seed"] < 2**32 for r in rows for s in r["paired_seeds"])
    with pytest.raises(FileExistsError):
        generate("smoke", 12, tmp_path / "a", model=FakeModel())
