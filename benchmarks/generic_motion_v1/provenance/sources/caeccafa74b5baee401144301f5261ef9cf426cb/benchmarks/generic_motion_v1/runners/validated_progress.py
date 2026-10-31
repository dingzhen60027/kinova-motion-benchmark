"""Measured incumbent evidence using the same final numerical validator."""
from __future__ import annotations

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
    """

    def __init__(self, model, case, cfg, out, started, deadline, *, origin):
        self.model, self.case, self.cfg = model, case, cfg
        self.out = Path(out) / "validated_progress"
        self.out.mkdir(parents=True, exist_ok=True)
        self.started, self.deadline = started, deadline
        self.summary = dict(time_origin=origin, validation_charged_to_budget=True,
            numerical_validation_not_continuous_certificate=True, observations=0,
            first_valid_is_observed_upper_bound=True,
            validated_observations=0, first_valid_seconds=None,
            best_validated_cost_rad=None, validation_seconds=0.)
        write(self.out / "summary.json", self.summary)

    def record(self, points, phase, native_cost, *, admissible_deadline=None):
        deadline = self.deadline if admissible_deadline is None else min(
            self.deadline, admissible_deadline)
        observed = time.perf_counter()
        index = self.summary["observations"]
        row = dict(index=index, phase=phase, native_cost_rad=float(native_cost),
            observed_seconds=observed-self.started, validation_completed_seconds=None,
            validation_status=None, within_budget=False, cost_rad=None,
            best_validated_cost_rad=self.summary["best_validated_cost_rad"])
        self.summary["observations"] += 1
        points = np.asarray(points, float)
        candidate = dict(kind="polygon", objective="minimum_distance", points=points)
        np.savez_compressed(self.out / f"candidate_{index:04d}.npz", points=points)
        if observed >= deadline:
            row["validation_status"] = "not_checked_budget_exhausted"
        else:
            tic = time.perf_counter()
            try:
                result = validate(candidate, self.model, self.case, self.cfg)
                completed = time.perf_counter()
                self.summary["validation_seconds"] += completed-tic
                write(self.out / f"validation_{index:04d}.json",
                      {key: value for key, value in result.items() if key != "_samples"})
                row.update(validation_completed_seconds=completed-self.started,
                    validation_status=result["status"], within_budget=completed <= deadline,
                    cost_rad=float(result["cost"]) if result["passed"] else None)
                if result["passed"] and row["within_budget"]:
                    self.summary["validated_observations"] += 1
                    if self.summary["first_valid_seconds"] is None:
                        self.summary["first_valid_seconds"] = completed-self.started
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
