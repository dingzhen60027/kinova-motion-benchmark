"""Timed joint trajectories, online splicing and protective stops.

A trajectory is the repository's timed candidate: ``kind='bezier'`` with
controls ``[S, d+1, 7]`` and positive durations ``[S]``, or a clamped time
B-spline (``kind='bspline'``). Evaluation reuses
``simulation.reference._from_candidate`` and derivative bounds reuse
``runners.validate.sample_candidate``; both use ``objective='minimum_time'``
as the tag of a physically timed curve, so that tag is attached here.

A :class:`Plan` is the commanded reference. Each segment has an absolute start
time; the latest segment whose start has been reached is active, and a
finished segment holds its final configuration at rest. A submission replaces
the plan from its start time onward and must connect to the plan there in
position and velocity.
"""
from __future__ import annotations

import numpy as np

from benchmarks.generic_motion_v1.runners.phase_search import line_curve
from benchmarks.generic_motion_v1.runners.validate import sample_candidate
from benchmarks.generic_motion_v1.simulation.reference import _from_candidate

TIMED = "minimum_time"


def normalize(candidate):
    """Copy a submitted candidate into float arrays with the timed-curve tag."""
    if not isinstance(candidate, dict) or candidate.get("kind") not in ("bezier", "bspline"):
        raise ValueError("Trajectory must be a dict of kind 'bezier' or 'bspline'")
    value = {"kind": candidate["kind"], "objective": TIMED}
    if candidate["kind"] == "bezier":
        value["controls"] = np.array(candidate["controls"], dtype=float)
        value["durations"] = np.array(candidate["durations"], dtype=float).reshape(-1)
    else:
        value["controls"] = np.array(candidate["controls"], dtype=float)
        value["knots"] = np.array(candidate["knots"], dtype=float).reshape(-1)
        order = candidate.get("spline_order")
        if isinstance(order, (bool, np.bool_)) or not isinstance(order, (int, np.integer)):
            raise ValueError("B-spline order must be an integer")
        value["spline_order"] = int(order)
    return value


def serializable(candidate):
    return {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in candidate.items()}


class Segment:
    def __init__(self, start_time, candidate):
        self.start = float(start_time)
        if not np.isfinite(self.start):
            raise ValueError("Segment start time must be finite")
        self.candidate = normalize(candidate)
        self.reference = _from_candidate(self.candidate, {})
        self.duration = float(self.reference.duration)
        self.end = self.start + self.duration

    def at(self, t):
        return self.reference.at(float(t) - self.start)

    def to_dict(self):
        return dict(start_time_s=self.start, trajectory=serializable(self.candidate))


class Plan:
    """Commanded reference built from spliced segments (immutable)."""

    def __init__(self, segments):
        segments = list(segments)
        if not segments or any(b.start <= a.start for a, b in zip(segments, segments[1:])):
            raise ValueError("A plan needs segments with strictly increasing start times")
        self.segments = segments
        self._starts = np.asarray([s.start for s in segments])

    def active(self, t):
        index = int(np.searchsorted(self._starts, float(t), side="right")) - 1
        if index < 0:
            raise ValueError("Plan queried before its first segment")
        return self.segments[index]

    def at(self, t):
        return self.active(t).at(t)

    def spliced(self, segment):
        return Plan([s for s in self.segments if s.start < segment.start] + [segment])

    def future(self, t):
        """Segments that define the reference on [t, infinity)."""
        index = max(0, int(np.searchsorted(self._starts, float(t), side="right")) - 1)
        return self.segments[index:]

    def final_configuration(self):
        last = self.segments[-1]
        return last.at(last.end)[0]

    def sample(self, t0, t1, dt):
        times = np.arange(float(t0), float(t1) + 0.5 * dt, dt)
        states = [self.at(t) for t in times]
        return times, np.asarray([s[0] for s in states]), np.asarray([s[1] for s in states])

    def to_list(self, t=None):
        segments = self.segments if t is None else self.future(t)
        return [s.to_dict() for s in segments]

    @classmethod
    def from_list(cls, items):
        return cls([Segment(item["start_time_s"], item["trajectory"]) for item in items])


def check_submission(plan, start_time, candidate, limits, earliest_start):
    """Validate a submission against the protocol; return (segment, reason, details).

    ``limits`` holds lower/upper joint bounds, velocity/acceleration limits,
    the shared numerical tolerance and the caps on duration and pieces. A
    rejected submission never alters the commanded reference.
    """
    details = {}
    try:
        start_time = float(start_time)
        segment = Segment(start_time, candidate)
    except Exception as exc:  # malformed content is a method error, never an infrastructure error
        return None, "malformed", dict(error=f"{type(exc).__name__}: {exc}")
    tol = limits["tolerance"]
    details.update(start_time_s=start_time, duration_s=segment.duration)
    if start_time < earliest_start - 1e-12:
        details["earliest_start_s"] = earliest_start
        return None, "deadline_miss", details
    pieces = len(segment.candidate["durations"]) if segment.candidate["kind"] == "bezier" else len(segment.candidate["knots"])
    if segment.duration > limits["maximum_duration_s"] or pieces > limits["maximum_pieces"]:
        return None, "too_long", details
    try:
        bounds = sample_candidate(segment.candidate)
    except Exception as exc:
        return None, "malformed", dict(details, error=f"{type(exc).__name__}: {exc}")
    q0, v0, _ = segment.at(start_time)
    qp, vp, _ = plan.at(start_time)
    details.update(position_jump_rad=float(np.max(np.abs(q0 - qp))), velocity_jump_rad_s=float(np.max(np.abs(v0 - vp))))
    if details["position_jump_rad"] > tol or details["velocity_jump_rad_s"] > tol:
        return None, "discontinuous", details
    end_velocity = segment.reference.at(segment.duration)[1]
    details["end_speed_rad_s"] = float(np.max(np.abs(end_velocity)))
    if details["end_speed_rad_s"] > tol:
        return None, "not_at_rest", details
    controls = np.asarray(bounds["position_controls"]).reshape(-1, 7)
    if np.any(controls < limits["lower"] - tol) or np.any(controls > limits["upper"] + tol):
        return None, "joint_limits", details
    derivatives = bounds["derivatives"]
    details.update(velocity_bound_rad_s=float(np.max(derivatives["velocity_per_joint"] / limits["velocity"])),
                   acceleration_bound_rad_s2=float(np.max(derivatives["acceleration_per_joint"] / limits["acceleration"])))
    if np.any(derivatives["velocity_per_joint"] > limits["velocity"] + tol):
        return None, "velocity_limits", details
    if np.any(derivatives["acceleration_per_joint"] > limits["acceleration"] + tol):
        return None, "acceleration_limits", details
    return segment, "accepted", details


def hold(q, duration=1e-3):
    q = np.asarray(q, dtype=float)
    return dict(kind="bezier", objective=TIMED, controls=np.broadcast_to(q, (1, 3, 7)).copy(),
                durations=np.array([float(duration)]))


def braking(q, v, acceleration):
    """Stop along the current joint velocity direction with uniform deceleration.

    The slowest admissible common stopping time T = max_i |v_i| / a_i keeps
    every joint within its acceleration limit and the motion on the tangent
    line, which is the current path for straight-line joint motions.
    """
    q, v = np.asarray(q, float), np.asarray(v, float)
    duration = float(np.max(np.abs(v) / np.asarray(acceleration, float)))
    if duration < 1e-6:
        return hold(q)
    end = q + 0.5 * duration * v
    return dict(kind="bezier", objective=TIMED, controls=np.stack([q, end, end])[None],
                durations=np.array([duration]))


def concatenate(candidates):
    """Join rest-to-rest or C1-matched quadratic Bezier pieces."""
    controls = [np.asarray(c["controls"], float) for c in candidates]
    if any(c.shape[1] != 3 for c in controls):
        raise ValueError("Only quadratic pieces are concatenated")
    return dict(kind="bezier", objective=TIMED, controls=np.concatenate(controls),
                durations=np.concatenate([np.asarray(c["durations"], float) for c in candidates]))


def point_to_point(waypoints, velocity, acceleration, arrivals=False):
    """Rest-to-rest straight joint-space motions through waypoints (phase-synchronized).

    With ``arrivals=True`` also return the arrival time of every waypoint.
    """
    points = np.asarray(waypoints, dtype=float).reshape(-1, 7)
    pieces, times = [], [0.0]
    for a, b in zip(points[:-1], points[1:]):
        if np.max(np.abs(b - a)) > 1e-9:
            pieces.append(line_curve(a, b, np.asarray(velocity, float), np.asarray(acceleration, float)))
            times.append(times[-1] + float(np.sum(pieces[-1]["durations"])))
        else:
            times.append(times[-1])
    candidate = concatenate(pieces) if pieces else hold(points[0])
    return (candidate, np.asarray(times)) if arrivals else candidate
