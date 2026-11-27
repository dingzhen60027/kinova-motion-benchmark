import json
from pathlib import Path
import time

import numpy as np
import pytest

from benchmarks.generic_motion_v1.runners import phase_search as phase
from benchmarks.generic_motion_v1.runners.csdecomp_baseline import _save_candidate
from . import cohort
from . import paired as p
from .excursion_worker import admit, install


class Blocked:
    lower = np.full(7, -7.0)
    upper = np.full(7, 7.0)

    def __init__(self):
        self.submitted = []

    def cuda(self, q):
        self.submitted.append(np.copy(q))
        return np.zeros(len(q), dtype=bool)

    def clearance(self, q):
        return np.zeros(len(q))


def test_regeneration_of_later_scales_and_deadline():
    start, goal = np.zeros(7), np.array([3.0, 0.4, -0.7, 0.6, 1.5, 0.9, 1.8])
    velocity, accel = np.ones(7), np.ones(7)
    line = np.linspace(start, goal, 33)
    mask = np.ones(33, bool)
    mask[16] = False
    trace = {}
    original = phase.schedules
    states = []
    for arm in ("guided", "excursion"):
        checker = Blocked()
        rng = np.random.default_rng(334)
        with install(arm, 334, trace):
            _, grad = phase.collision_witness(checker, line, mask)
            assert not list(
                phase.schedules(
                    checker,
                    start,
                    goal,
                    velocity,
                    accel,
                    rng,
                    grad,
                    {},
                    time.perf_counter() + 5,
                )
            )
            assert len(checker.submitted) == 4
            assert not list(
                phase.schedules(
                    checker,
                    start,
                    goal,
                    velocity,
                    accel,
                    rng,
                    grad,
                    {},
                    time.perf_counter() - 1,
                )
            )
            assert len(checker.submitted) == 4
        states.append((checker.submitted, rng.uniform()))
    assert original is phase.schedules and trace["first_layer_count"] == 25
    assert states[0][1] == states[1][1]
    assert not np.array_equal(states[0][0][0], states[1][0][0])
    for a, b in zip(states[0][0][1:], states[1][0][1:]):
        np.testing.assert_array_equal(a, b)


@pytest.mark.parametrize(
    "bad",
    [
        {"method": "PHASE_SEARCH", "objective": "minimum_distance"},
        {
            "method": "PHASE_SEARCH",
            "objective": "minimum_time",
            "case_id": "test-hidden",
            "replica_seed": 0,
        },
        {
            "method": "PHASE_SEARCH",
            "objective": "minimum_time",
            "case_id": "dev-hidden",
            "replica_seed": 2,
        },
    ],
)
def test_worker_rejects_other_matrices(bad):
    with pytest.raises(ValueError):
        admit(bad)


def cpu_integration(directory):
    directory = Path(directory)
    directory.mkdir(exist_ok=True, parents=True)
    pid = "dev-bottleneck-004-p00"
    case = next(c for c in p.select() if c["problem_id"] == pid)
    case = {**case, "_objective": "minimum_time"}
    seed = case["paired_seeds"][0]["method_seed"]
    cfg = p.load_config(p.ROOT / "configs/evaluation.json")
    scene = p.Scene(case)
    # Exercise the unmodified solve + real world geometry with a CPU-only
    # broad-phase. A timed GPU worker never takes this test-only path.
    scene.model.cuda = scene.mask
    trace = {}
    with install("excursion", seed, trace):
        candidate = phase.solve(
            scene.model,
            case,
            cfg,
            directory,
            time.perf_counter() + 60,
            seed,
            "PHASE_SEARCH",
        )
    _save_candidate(directory, candidate)
    loaded = json.loads((directory / "candidate.json").read_text())
    with np.load(directory / "candidate.npz") as saved:
        loaded.update({k: saved[k] for k in saved.files})
    sample = p.sample_candidate(loaded, cfg)
    lower = float(
        max(
            p.rest_times(
                np.asarray(case["q_goal"]) - case["q_start"],
                np.asarray(case["velocity_limits_rad_s"]),
                np.asarray(case["acceleration_limits_rad_s2"]),
            )[0]
        )
    )
    assert abs(lower - sample["duration"]) < 1e-8
    assert cohort.validate_dynamics(sample, case, scene.model, cfg, lower)
    clearance = scene.model.clearance(sample["q"])
    assert min(clearance) > scene.model.contact_band_m
    assert (
        trace["first_layer_count"] == 25
        and trace["first_layer_returned_slot"] is not None
    )
    rec = dict(
        problem_id=pid,
        seed=seed,
        status="cpu_integration_passed",
        cuda_used=False,
        lower_bound_s=lower,
        saved_trajectory_sha256=p.digest(
            loaded["controls"].tobytes() + loaded["durations"].tobytes()
        ),
        validated_positions=len(clearance),
        min_clearance_m=float(min(clearance)),
        bank=trace,
    )
    p.save(directory / "cpu_integration.json", rec)
    return rec


def test_real_cpu_integration(tmp_path):
    cpu_integration(tmp_path)
