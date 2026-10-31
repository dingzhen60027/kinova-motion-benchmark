"""Unvalidated or late incumbents cannot become an anytime success."""
import json
import time
from types import SimpleNamespace

import numpy as np

from benchmarks.generic_motion_v1.runners.validated_progress import ValidatedProgress


def observer(tmp_path, *, colliding=False, deadline=None):
    model = SimpleNamespace(lower=np.full(7, -2.), upper=np.full(7, 2.),
        cuda=lambda q: np.full(len(q), not colliding),
        clearance=lambda q: np.full(len(q), -.1 if colliding else 1.))
    points = np.zeros((2, 7)); points[-1, 0] = 1.
    case = dict(q_start=points[0], q_goal=points[-1],
        velocity_limits_rad_s=[1.]*7, acceleration_limits_rad_s2=[1.]*7,
        endpoint_velocity_rad_s=[0.]*7)
    started = time.perf_counter()
    return ValidatedProgress(model, case, {}, tmp_path, started,
        started+10 if deadline is None else deadline, origin="test"), points


def test_native_candidate_cost_is_not_itself_a_validated_cost(tmp_path):
    progress, points = observer(tmp_path, colliding=True)
    row = progress.record(points, "native_continuation", 1.)
    assert row["validation_status"] == "collision_failed"
    assert row["cost_rad"] is None
    assert progress.summary["first_valid_seconds"] is None
    assert progress.summary["best_validated_cost_rad"] is None
    assert progress.summary["validated_observations"] == 0
    assert (tmp_path / "validated_progress/candidate_0000.npz").exists()


def test_expired_budget_never_enters_validator_or_reports_cost(tmp_path):
    progress, points = observer(tmp_path, deadline=time.perf_counter()-1.)
    progress.model.cuda = lambda q: (_ for _ in ()).throw(AssertionError("must not validate"))
    row = progress.record(points, "native_continuation", 1.)
    assert row["validation_status"] == "not_checked_budget_exhausted"
    assert row["cost_rad"] is None and not row["within_budget"]
    assert progress.summary["validated_observations"] == 0


def test_best_validated_cost_retained_without_faking_monotonic_raw_costs(tmp_path):
    progress, straight = observer(tmp_path)
    bend = (straight[0]+straight[-1])/2; bend[1] = 1.
    longer = np.vstack([straight[0], bend, straight[-1]])
    first = progress.record(straight, "native_continuation", 1.)
    second = progress.record(longer, "native_continuation", 2.3)
    assert second["cost_rad"] > first["cost_rad"]
    assert second["best_validated_cost_rad"] == first["cost_rad"]
    assert progress.summary["first_valid_seconds"] == first["validation_completed_seconds"]
    rows = [json.loads(line) for line in (tmp_path / "validated_progress/records.jsonl").read_text().splitlines()]
    assert len(rows) == 2


def test_identical_points_share_validation_across_phases(tmp_path):
    progress, points = observer(tmp_path)
    calls = []
    native = progress.model.cuda
    progress.model.cuda = lambda q: (calls.append(len(q)), native(q))[1]
    first = progress.record(points, "native_continuation", 1.)
    elapsed = progress.summary["validation_seconds"]
    second = progress.record(points.copy(), "native_return", 1.)
    third = progress.record(np.asfortranarray(points), "native_simplified_return", 1.)
    assert len(calls) == 1
    assert progress.summary["validation_calls"] == 1
    assert progress.summary["validation_cache_hits"] == 2
    assert progress.summary["validation_seconds"] == elapsed
    assert progress.summary["first_valid_seconds"] == first["evidence_available_seconds"]
    for row in (second, third):
        assert row["validation_reused"]
        assert row["validation_source_index"] == first["index"]
        assert row["validation_source_path"] == "validation_0000.json"
        assert row["validation_completed_seconds"] == first["validation_completed_seconds"]
        assert row["evidence_available_seconds"] >= row["observed_seconds"]
        assert row["candidate_content_sha256"] == first["candidate_content_sha256"]
    assert len(list((tmp_path / "validated_progress").glob("validation_*.json"))) == 1


def test_changed_array_and_distinct_observer_require_new_validation(tmp_path):
    progress, points = observer(tmp_path / "first")
    first = progress.record(points, "native_continuation", 1.)
    middle = (points[0]+points[-1])/2; middle[1] = .001
    changed = np.vstack([points[0], middle, points[-1]])
    second = progress.record(changed, "native_continuation", 1.)
    assert not second["validation_reused"]
    assert second["candidate_content_sha256"] != first["candidate_content_sha256"]
    assert progress.summary["validation_calls"] == 2
    separate, same = observer(tmp_path / "separate", colliding=True)
    failed = separate.record(same, "native_return", 1.)
    assert not failed["validation_reused"] and failed["cost_rad"] is None


def test_cached_collision_remains_failure(tmp_path):
    progress, points = observer(tmp_path, colliding=True)
    first = progress.record(points, "native_continuation", 1.)
    second = progress.record(points.copy(), "native_return", 1.)
    assert first["validation_status"] == second["validation_status"] == "collision_failed"
    assert second["validation_reused"] and second["cost_rad"] is None
    assert progress.summary["first_valid_seconds"] is None
    assert progress.summary["validation_calls"] == 1


def test_late_validation_cannot_be_retroactively_backfilled(tmp_path, monkeypatch):
    from benchmarks.generic_motion_v1.runners import validated_progress as module
    clock = [10.]
    monkeypatch.setattr(module.time, "perf_counter", lambda: clock[0])
    progress, points = observer(tmp_path)
    original = module.validate
    def delayed(*args, **kwargs):
        answer = original(*args, **kwargs)
        clock[0] = 12.
        return answer
    monkeypatch.setattr(module, "validate", delayed)
    first = progress.record(points, "native_continuation", 1., admissible_deadline=11.)
    assert first["validation_completed_seconds"] == 2.
    assert not first["within_budget"] and progress.summary["first_valid_seconds"] is None
    clock[0] = 13.
    second = progress.record(points.copy(), "native_return", 1.)
    assert second["validation_reused"]
    assert second["validation_completed_seconds"] == 2.
    assert second["evidence_available_seconds"] == 3.
    assert progress.summary["first_valid_seconds"] == 3.
    assert first["within_budget"] is False


def test_cache_does_not_authorize_observation_past_current_deadline(tmp_path):
    progress, points = observer(tmp_path)
    progress.record(points, "native_continuation", 1.)
    second = progress.record(points.copy(), "native_return", 1., admissible_deadline=0.)
    assert second["validation_status"] == "not_checked_budget_exhausted"
    assert not second["within_budget"] and not second["validation_reused"]
    assert second["cost_rad"] is None
