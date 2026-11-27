"""Rest-to-rest joint excursions through a collision-witness time plane."""

from dataclasses import dataclass

import numpy as np

from benchmarks.generic_motion_v1.runners.phase_search import clock_state


def radius(t, velocity, acceleration):
    """Maximum rest-to-rest displacement at time t with axiswise box limits."""
    v, a = np.broadcast_arrays(
        np.asarray(velocity, float), np.asarray(acceleration, float)
    )
    if (
        not np.isfinite(t)
        or t < 0
        or not np.isfinite([v, a]).all()
        or np.any(v <= 0)
        or np.any(a <= 0)
    ):
        raise ValueError("Expected nonnegative time and positive finite limits")
    return np.where(t <= 2 * v / a, 0.25 * a * t * t, v * (t - v / a))


def via_intervals(start, goal, T, t, lower, upper, velocity, acceleration):
    if not 0 < t < T:
        raise ValueError("The via time must be strictly internal")
    before, after = radius(t, velocity, acceleration), radius(
        T - t, velocity, acceleration
    )
    return (
        np.maximum.reduce([start - before, goal - after, lower]),
        np.minimum.reduce([start + before, goal + after, upper]),
    )


def _minimum_peak_ramp(delta, span, vmax):
    # With fixed duration, a symmetric triangular velocity minimizes peak
    # acceleration until the velocity bound forces a nonzero cruise interval.
    ramp = np.minimum(span * 0.5, span - np.abs(delta) / vmax)
    if np.any(ramp <= 0):
        raise ValueError("A rest-to-rest segment has no feasible switching time")
    return ramp


@dataclass(frozen=True)
class Motion:
    start: np.ndarray
    goal: np.ndarray
    T: float
    t: float
    via: np.ndarray
    active: np.ndarray
    velocity: np.ndarray
    delay: np.ndarray
    span: np.ndarray
    ramp: np.ndarray

    def parts(self):
        at = self.active
        return [
            (self.start, self.goal, self.delay, self.span, self.ramp, ~at, 0.0),
            (
                self.start,
                self.via,
                np.zeros(7),
                np.full(7, self.t),
                _minimum_peak_ramp(
                    np.where(at, self.via - self.start, 0.0), self.t, self.velocity
                ),
                at,
                0.0,
            ),
            (
                self.via,
                self.goal,
                np.zeros(7),
                np.full(7, self.T - self.t),
                _minimum_peak_ramp(
                    np.where(at, self.goal - self.via, 0.0),
                    self.T - self.t,
                    self.velocity,
                ),
                at,
                self.t,
            ),
        ]

    def state(self, times):
        times = np.asarray(times, float)
        q = np.tile(self.start, (len(times), 1))
        v = np.zeros_like(q)
        for index, (a, b, delay, span, ramp, axes, origin) in enumerate(self.parts()):
            if not np.any(axes):
                continue
            ids = np.flatnonzero(times >= origin)
            if index == 1:
                ids = ids[times[ids] <= self.t]
            if not len(ids):
                continue
            target = np.ix_(ids, axes)
            q[target], v[target] = clock_state(
                times[ids, None] - origin,
                a[axes],
                b[axes] - a[axes],
                delay[axes],
                span[axes],
                ramp[axes],
            )
        return q, v

    def curve(self):
        switches = [0.0, self.T]
        for _, _, delay, span, ramp, axes, origin in self.parts():
            if np.any(axes):
                end = delay + span
                switches.extend(
                    (
                        origin
                        + np.r_[
                            delay[axes],
                            (delay + ramp)[axes],
                            (end - ramp)[axes],
                            end[axes],
                        ]
                    ).tolist()
                )
        times = np.sort(np.clip(switches, 0, self.T))
        near = 64 * np.finfo(float).eps * max(1.0, self.T)
        times = times[np.r_[True, np.diff(times) > near]]
        times[0], times[-1] = 0.0, self.T
        q, v = self.state(times)
        q[0], q[-1] = self.start, self.goal
        dt = np.diff(times)
        controls = np.stack(
            [q[:-1], q[:-1] + 0.5 * dt[:, None] * v[:-1], q[1:]], axis=1
        )
        return dict(
            kind="bezier", objective="minimum_time", controls=controls, durations=dt
        )


def proposals(
    start,
    goal,
    T,
    t,
    lower,
    upper,
    velocity,
    acceleration,
    gradient,
    base,
    ramp0,
    width,
    shift,
    rng,
):
    """Propose individual and simultaneous reversals plus interior via points.

    The first fourteen slots alternate signed excursions per eligible axis.
    The remaining slots vary axis subsets and the depth of the excursion.
    """
    low, high = via_intervals(start, goal, T, t, lower, upper, velocity, acceleration)
    epsilon = 1e-8
    # Use exact envelope edges. An artificial inward offset produces nearly
    # coincident switch times that make floating Bézier second differences unstable.
    valid = high - low > 2 * epsilon
    qmin, qmax = np.minimum(start, goal), np.maximum(start, goal)
    directions = []
    for j in np.flatnonzero(valid):
        for sign, z in [(-1, low[j]), (1, high[j])]:
            extends = z < qmin[j] - epsilon or z > qmax[j] + epsilon
            directions.append(
                (extends, sign * gradient[j] * (high[j] - low[j]), j, sign)
            )
    directions.sort(reverse=True)
    for i, (w, s) in enumerate(zip(width, shift)):
        span = np.where(base > 0, base + w * (T - base), 0.0)
        ramp = np.divide(ramp0, base, out=np.zeros(7), where=base > 0) * span
        delay = s * (T - span)
        active = np.zeros(7, dtype=bool)
        z = start.copy()
        if directions:
            if i < 14:
                _, _, j, sign = directions[i % len(directions)]
                active[j] = True
                z[j] = high[j] if sign == 1 else low[j]
            else:
                axes = np.flatnonzero(valid)
                subset = rng.choice(
                    axes, size=rng.integers(1, len(axes) + 1), replace=False
                )
                active[subset] = True
                z[subset] = rng.uniform(low[subset], high[subset])
        yield Motion(start, goal, T, t, z, active, velocity, delay, span, ramp)
