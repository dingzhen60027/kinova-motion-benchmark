"""Exact saved time-trajectory references for torque-controlled simulation.

This module evaluates the optimizer's original coefficients.  Validation and
receipt acceptance belong to the simulation entry point; a parsed reference is
not itself a claim that planning succeeded.  PWL paths have no physical clock
and are intentionally rejected rather than silently time-parameterized.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import BSpline
from scipy.special import comb


CONTINUITY_TOLERANCE = 1e-5


def _array(value, name):
    def contains_boolean(item):
        if isinstance(item, (bool, np.bool_)):
            return True
        if isinstance(item, (list, tuple)):
            return any(contains_boolean(child) for child in item)
        return isinstance(item, np.ndarray) and item.dtype.kind == "b"
    array = np.asarray(value)
    if array.dtype.kind not in "fiu" or contains_boolean(value):
        raise ValueError(f"{name} must contain real numeric values, not booleans or strings")
    result = array.astype(float, copy=True)
    if not np.isfinite(result).all():
        raise ValueError(f"{name} must be finite")
    result.setflags(write=False)
    return result


def _scalar(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)):
        raise ValueError(f"{name} must be a finite real scalar")
    array = _array(value, name)
    if array.ndim != 0:
        raise ValueError(f"{name} must be a finite real scalar")
    return float(array)


def _bezier(controls, s):
    degree = len(controls) - 1
    k = np.arange(degree + 1)
    return (comb(degree, k) * s ** k * (1 - s) ** (degree - k)) @ controls


@dataclass(frozen=True)
class Reference:
    """A reference clock starts at zero, including nonzero B-spline knot origins."""

    duration: float
    metadata: dict
    _evaluate: object
    _start: np.ndarray
    _goal: np.ndarray

    def at(self, time):
        t = _scalar(time, "Reference time")
        if t < 0:
            return self._start.copy(), np.zeros(7), np.zeros(7)
        if t > self.duration:
            return self._goal.copy(), np.zeros(7), np.zeros(7)
        result = tuple(np.asarray(item, dtype=float).copy() for item in self._evaluate(t))
        if any(item.shape != (7,) or not np.isfinite(item).all() for item in result):
            raise ValueError("Reference evaluation produced malformed or nonfinite state")
        return result


def _from_candidate(candidate, source_metadata):
    if candidate.get("objective") != "minimum_time":
        raise ValueError("Simulation requires a minimum_time candidate; a minimum_distance path needs an explicit common time-parameterization protocol")
    kind = candidate.get("kind")
    metadata = dict(source_metadata, kind=kind, objective="minimum_time",
                    evaluation="original polynomial coefficients, analytic first and second derivatives",
                    time_units="seconds", time_origin="simulation starts at zero",
                    outside_time_domain="hold endpoint with zero velocity and acceleration; endpoints themselves use original derivatives")
    if kind == "bezier":
        controls = _array(candidate.get("controls"), "Bezier controls")
        durations = _array(candidate.get("durations"), "Bezier durations")
        if (controls.ndim != 3 or controls.shape[0] < 1 or controls.shape[1] < 3 or controls.shape[2] != 7
                or durations.shape != (len(controls),) or np.any(durations <= 0)):
            raise ValueError("Malformed Bezier controls or nonpositive segment durations")
        degree = controls.shape[1] - 1
        velocity = degree * np.diff(controls, axis=1) / durations[:, None, None]
        acceleration = (degree - 1) * np.diff(velocity, axis=1) / durations[:, None, None]
        if not np.isfinite(velocity).all() or not np.isfinite(acceleration).all():
            raise ValueError("Nonfinite Bezier derivatives")
        residual_q = float(np.max(np.abs(controls[:-1, -1] - controls[1:, 0]), initial=0))
        residual_v = float(np.max(np.abs(velocity[:-1, -1] - velocity[1:, 0]), initial=0))
        if max(residual_q, residual_v) > CONTINUITY_TOLERANCE:
            raise ValueError("Bezier segments violate required C1 position/velocity continuity")
        offsets = np.r_[0., np.cumsum(durations)]
        duration = float(offsets[-1])
        if not np.isfinite(duration):
            raise ValueError("Nonfinite total Bezier duration")

        def evaluate(t):
            index = min(int(np.searchsorted(offsets, t, side="right")) - 1, len(controls) - 1)
            parameter = float(np.clip((t - offsets[index]) / durations[index], 0., 1.))
            return (_bezier(controls[index], parameter), _bezier(velocity[index], parameter),
                    _bezier(acceleration[index], parameter))

        metadata.update(degree=int(degree), segments=int(len(controls)),
                        maximum_position_continuity_residual_rad=residual_q,
                        maximum_velocity_continuity_residual_rad_s=residual_v)
    elif kind == "bspline":
        controls = _array(candidate.get("controls"), "B-spline controls")
        knots = _array(candidate.get("knots"), "B-spline knots")
        order = candidate.get("spline_order")
        if isinstance(order, (bool, np.bool_)) or not isinstance(order, (int, np.integer)):
            raise ValueError("B-spline order must be an integer")
        if (order < 3 or controls.ndim != 2 or controls.shape[1] != 7 or len(controls) < order
                or knots.ndim != 1 or len(knots) != len(controls) + order or np.any(np.diff(knots) < 0)):
            raise ValueError("Malformed time B-spline coefficients, knots or order")
        start, end = float(knots[order - 1]), float(knots[-order])
        if end <= start:
            raise ValueError("Nonpositive B-spline duration")
        if np.any(knots[:order] != start) or np.any(knots[-order:] != end):
            raise ValueError("Only clamped B-splines are supported")
        interior = knots[order:-order]
        _, counts = np.unique(interior, return_counts=True)
        if np.any(interior <= start) or np.any(interior >= end) or np.any(counts > order - 2):
            raise ValueError("B-spline interior knot multiplicity violates C1 continuity")
        spline = BSpline(knots, controls, order - 1, extrapolate=False)
        first, second = spline.derivative(1), spline.derivative(2)
        if not np.isfinite(first.c).all() or not np.isfinite(second.c).all():
            raise ValueError("Nonfinite B-spline derivatives")
        duration = end - start

        def evaluate(t):
            parameter = float(np.clip(start + t, start, end))
            return spline(parameter), first(parameter), second(parameter)

        metadata.update(spline_order=int(order), control_points=int(len(controls)),
                        original_knot_start_seconds=start, original_knot_end_seconds=end,
                        knot_time_scaling="already physical seconds; no duration multiplication")
    else:
        raise ValueError(f"Unsupported minimum_time candidate kind: {kind!r}")
    if "duration" in candidate:
        declared = _scalar(candidate["duration"], "Declared trajectory duration")
        if declared <= 0 or not np.isclose(declared, duration, rtol=1e-10, atol=1e-10):
            raise ValueError("Declared duration differs from polynomial time domain")
    start_q, start_v, _ = evaluate(0.)
    goal_q, goal_v, _ = evaluate(duration)
    metadata.update(duration_seconds=float(duration),
                    endpoint_velocity_max_abs_rad_s=float(np.max(np.abs([start_v, goal_v]))))
    reference = Reference(float(duration), metadata, evaluate, np.asarray(start_q), np.asarray(goal_q))
    reference.at(0.)
    reference.at(duration)
    return reference


def load_reference(attempt):
    """Load candidate.json + candidate.npz without changing or resampling them."""
    directory = Path(attempt).expanduser().resolve()
    metadata_path, arrays_path = directory / "candidate.json", directory / "candidate.npz"
    candidate = json.loads(metadata_path.read_text())
    if not isinstance(candidate, dict):
        raise ValueError("Candidate metadata must be a JSON object")
    with np.load(arrays_path, allow_pickle=False) as saved:
        for key in saved.files:
            if key in candidate:
                raise ValueError(f"Candidate key appears in both JSON and NPZ: {key}")
            candidate[key] = saved[key].copy()
    source = {str(path.name): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in (metadata_path, arrays_path)}
    return _from_candidate(candidate, {"attempt_path": str(directory), "source_files_sha256": source})
