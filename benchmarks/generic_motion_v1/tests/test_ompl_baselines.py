"""Actual official OMPL regression solves, isolated from the benchmark GPU runs."""
import json
from pathlib import Path
import subprocess
import sys
import textwrap

import numpy as np
import pytest

from benchmarks.generic_motion_v1.runners.ompl_baselines import (
    DEPENDENCY_DIR, dependency_probe, motion_samples, settings, native_seed,
)


def test_official_native_dependency_is_isolated_and_hashed():
    result = dependency_probe()
    assert result["version"] == "2.0.1"
    assert result["environment_modified"] is False
    assert len(result["files_sha256"]) >= 3
    assert any("libompl.so" in name for name in result["files_sha256"])


@pytest.mark.parametrize("a,b", [(np.zeros(7), np.ones(7)),
                                     (np.zeros(7), np.zeros(7)),
                                     (-np.ones(7)*3, np.ones(7)*3)])
def test_motion_samples_bound_euclidean_step_and_preserve_endpoints(a, b):
    values = motion_samples(a, b, .01)
    np.testing.assert_array_equal(values[0], a)
    np.testing.assert_array_equal(values[-1], b)
    assert np.max(np.linalg.norm(np.diff(values, axis=0), axis=1)) <= .01 + 1e-12


@pytest.mark.parametrize("step", [0, -.1, .0101, np.nan, np.inf])
def test_unsafe_or_nonfinite_motion_resolution_rejected(step):
    with pytest.raises(ValueError):
        motion_samples(np.zeros(7), np.ones(7), step)


@pytest.mark.parametrize("bad", [dict(motion_step_rad=.02), dict(planning_seconds=-1),
                                   dict(goal_bias=1), dict(rewire_factor=1),
                                   dict(simplification_max_steps=2.1), dict(range_rad=True),
                                   dict(unknown=1)])
def test_invalid_ompl_settings_rejected(bad):
    with pytest.raises(ValueError):
        settings(dict(ompl=bad))


def native_solve(tmp_path, method, barrier=False, seed=73, progress=False):
    """Each process initializes OMPL's global RNG exactly once, as production."""
    script = textwrap.dedent(f"""
        import json,time
        import numpy as np
        from types import SimpleNamespace
        from benchmarks.generic_motion_v1.runners.ompl_baselines import solve,OmplNoExactSolution
        from benchmarks.generic_motion_v1.roadmap import write
        path={str(tmp_path)!r}
        model=SimpleNamespace(lower=np.full(7,-1.),upper=np.ones(7),
            robot_sha256='regression-fixture',collision_geometry_sha256='regression-fixture',
            cuda=lambda q: np.abs(q[:,0])>.05 if {barrier!r} else np.ones(len(q),bool),
            clearance=lambda q: np.abs(q[:,0])-.05 if {barrier!r} else np.ones(len(q)))
        start=np.zeros(7); goal=np.zeros(7); start[0]=-.5; goal[0]=.5
        try:
            case=dict(q_start=start,q_goal=goal,velocity_limits_rad_s=[1.]*7,
                acceleration_limits_rad_s2=[1.]*7,endpoint_velocity_rad_s=[0.]*7)
            candidate=solve(model,case,
                dict(ompl=dict(planning_seconds=.15,record_validated_progress={progress!r},progress_poll_seconds=.03)),path,time.perf_counter()+8,{seed},{method!r})
            write(path+'/candidate.json',candidate)
        except OmplNoExactSolution as exc:
            write(path+'/expected_failure.json',dict(solver_status=exc.solver_status))
    """)
    result = subprocess.run([sys.executable, "-c", script], text=True, capture_output=True, timeout=15)
    (tmp_path / "process.log").write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout + result.stderr
    return json.loads((tmp_path / "ompl_result.json").read_text())


@pytest.mark.parametrize("method", ["RRTCONNECT", "RRTSTAR"])
def test_actual_native_solution_and_artifacts(tmp_path, method):
    result = native_solve(tmp_path, method)
    assert result["solver_success"]
    assert result["native_status"] == "Exact solution"
    candidate = json.loads((tmp_path / "candidate.json").read_text())
    points = np.array(candidate["points"])
    np.testing.assert_allclose(points[[0,-1],0], [-.5,.5], atol=1e-12)
    assert np.max(np.abs(points)) <= 1
    assert result["simplified_path_length_rad"] <= result["raw_path_length_rad"] + 1e-12
    graph = np.load(tmp_path / "ompl_search_graph.npz")
    assert graph["vertices"].shape[1] == 7
    assert graph["edges"].shape[1] == 2
    assert result["collision_counters"]["motion_calls"] > 0
    assert result["collision_counters"]["collision_configurations"] > 0


@pytest.mark.parametrize("method", ["RRTCONNECT", "RRTSTAR"])
def test_full_space_barrier_never_returns_exact_success(tmp_path, method):
    result = native_solve(tmp_path, method, barrier=True)
    assert result["solver_success"] is False
    assert result["has_exact_solution"] is False
    assert result["status"] == "no_exact_solution"
    assert not (tmp_path / "candidate.json").exists()
    assert (tmp_path / "expected_failure.json").exists()
    assert result["collision_counters"]["invalid_motion_calls"] > 0


def test_rrtconnect_fixed_seed_repeats_raw_native_tree_and_path(tmp_path):
    first, second = tmp_path / "first", tmp_path / "second"
    first.mkdir(); second.mkdir()
    native_solve(first, "RRTCONNECT")
    native_solve(second, "RRTCONNECT")
    for name in ("ompl_raw_path.npz", "ompl_simplified_path.npz", "ompl_search_graph.npz"):
        a, b = np.load(first / name), np.load(second / name)
        assert a.files == b.files
        for key in a.files:
            np.testing.assert_array_equal(a[key], b[key])


def test_zero_and_one_requested_seeds_are_distinct_native_streams(tmp_path):
    assert native_seed(0) == 2**32-1
    assert native_seed(1) == 1
    first, second = tmp_path / "zero", tmp_path / "one"
    first.mkdir(); second.mkdir()
    native_solve(first, "RRTCONNECT", seed=0)
    native_solve(second, "RRTCONNECT", seed=1)
    a = np.load(first / "ompl_search_graph.npz")["vertices"]
    b = np.load(second / "ompl_search_graph.npz")["vertices"]
    assert not np.array_equal(a, b)


@pytest.mark.parametrize("bad", [True, -1, 2**32-1, 2**32, .5])
def test_reserved_or_invalid_requested_seed_rejected(bad):
    with pytest.raises(ValueError):
        native_seed(bad)


@pytest.mark.parametrize("method", ["RRTCONNECT", "RRTSTAR"])
def test_native_progress_requires_actual_common_validator_pass(tmp_path, method):
    result = native_solve(tmp_path, method, progress=True)
    summary = result["validated_progress"]
    assert summary["validated_observations"] >= 2
    assert summary["first_valid_seconds"] > 0
    rows = [json.loads(line) for line in (tmp_path / "validated_progress/records.jsonl").read_text().splitlines()]
    passed = [r for r in rows if r["within_budget"] and r["cost_rad"] is not None]
    assert len(passed) == summary["validated_observations"]
    assert all(r["validation_status"] == "success_validated_numerical" for r in passed)
    assert np.all(np.diff([r["best_validated_cost_rad"] for r in passed]) <= 0)
    assert summary["first_valid_seconds"] == passed[0]["validation_completed_seconds"]
    if method == "RRTSTAR":
        assert any(r["phase"] == "native_continuation" for r in rows)
        assert result["native_continuation_calls"] > 1
        assert result["progress_observation_policy"]["tree_reset"] is False
        assert result["progress_observation_policy"]["random_stream_reset"] is False
    for row in rows:
        points = np.load(tmp_path / "validated_progress" / f"candidate_{row['index']:04d}.npz")["points"]
        np.testing.assert_allclose(points[[0, -1], 0], [-.5, .5])


def test_progress_is_opt_in_and_strict_boolean():
    assert settings({})["record_validated_progress"] is False
    with pytest.raises(ValueError):
        settings(dict(ompl=dict(record_validated_progress=1)))
