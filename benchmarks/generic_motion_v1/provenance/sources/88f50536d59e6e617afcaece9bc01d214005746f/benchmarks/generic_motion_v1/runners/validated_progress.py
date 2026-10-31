"""Measured incumbent evidence using the same final numerical validator.

This diagnostic observer consumes the query budget and is disabled by default
in OMPL settings. Main baseline timing does not enable it implicitly.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time

import numpy as np

from ..roadmap import write
from .validate import validate


class ValidatedProgress:
    """Record actual candidates; only in-budget validation updates the curve.

    Validation runs synchronously in the query process and consumes its shared
    budget. No interpolation or retrospectively inferred first-solution time is
    emitted. The observer never substitutes a different algorithm's candidate.
    Exact repeated point arrays reuse prior evidence within this observer's
    fixed model, task and protocol; phase changes do not invalidate that evidence.
    """

    def __init__(self, model, case, cfg, out, started, deadline, *, origin):
        self.model, self.case, self.cfg = model, case, cfg
        self.out = Path(out) / "validated_progress"
        self.out.mkdir(parents=True, exist_ok=True)
        self.started, self.deadline = started, deadline
        self._validated = {}
        self.summary = dict(time_origin=origin, validation_charged_to_budget=True,
            numerical_validation_not_continuous_certificate=True, observations=0,
            first_valid_is_observed_upper_bound=True,
            validated_observations=0, first_valid_seconds=None,
            validation_calls=0, validation_cache_hits=0,
            best_validated_cost_rad=None, validation_seconds=0.)
        write(self.out / "summary.json", self.summary)

    def record(self, points, phase, native_cost, *, admissible_deadline=None):
        deadline = self.deadline if admissible_deadline is None else min(
            self.deadline, admissible_deadline)
        observed = time.perf_counter()
        index = self.summary["observations"]
        row = dict(index=index, phase=phase, native_cost_rad=float(native_cost),
            observed_seconds=observed-self.started, validation_completed_seconds=None,
            evidence_available_seconds=None, validation_reused=False,
            validation_source_index=None, validation_source_path=None,
            validation_status=None, within_budget=False, cost_rad=None,
            best_validated_cost_rad=self.summary["best_validated_cost_rad"])
        self.summary["observations"] += 1
        points = np.ascontiguousarray(points, dtype=np.float64)
        digest = hashlib.sha256(json.dumps(list(points.shape)).encode()+b"\0"+points.tobytes()).hexdigest()
        row["candidate_content_sha256"] = digest
        candidate = dict(kind="polygon", objective="minimum_distance", points=points)
        np.savez_compressed(self.out / f"candidate_{index:04d}.npz", points=points)
        if observed >= deadline:
            row["validation_status"] = "not_checked_budget_exhausted"
        else:
            tic = time.perf_counter()
            try:
                cached = self._validated.get(digest)
                if cached is not None:
                    # A cache hit is an observation made NOW, not a new check
                    # or retroactive proof at the candidate's earlier timestamp.
                    result = cached["result"]
                    completed = cached["completed"]
                    available = time.perf_counter()
                    source_index = cached["index"]
                    row["validation_reused"] = True
                    self.summary["validation_cache_hits"] += 1
                else:
                    self.summary["validation_calls"] += 1
                    result = validate(candidate, self.model, self.case, self.cfg)
                    completed = time.perf_counter()
                    available = completed
                    source_index = index
                    self.summary["validation_seconds"] += completed-tic
                    write(self.out / f"validation_{index:04d}.json",
                          {key: value for key, value in result.items() if key != "_samples"})
                    self._validated[digest] = dict(index=index, completed=completed,
                        result={key: result[key] for key in ("status", "cost", "passed")})
                row.update(validation_completed_seconds=completed-self.started,
                    evidence_available_seconds=available-self.started,
                    validation_source_index=source_index,
                    validation_source_path=f"validation_{source_index:04d}.json",
                    validation_status=result["status"], within_budget=available <= deadline,
                    cost_rad=float(result["cost"]) if result["passed"] else None)
                if result["passed"] and row["within_budget"]:
                    self.summary["validated_observations"] += 1
                    if self.summary["first_valid_seconds"] is None:
                        self.summary["first_valid_seconds"] = available-self.started
                    best = self.summary["best_validated_cost_rad"]
                    self.summary["best_validated_cost_rad"] = result["cost"] if best is None else min(best, result["cost"])
                    row["best_validated_cost_rad"] = self.summary["best_validated_cost_rad"]
            except Exception as exc:
                row.update(validation_status="validation_exception", error=repr(exc))
                self._save(row)
                raise
        self._save(row)
        return row

    def _save(self, row):
        with (self.out / "records.jsonl").open("a") as stream:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
        write(self.out / "summary.json", self.summary)
