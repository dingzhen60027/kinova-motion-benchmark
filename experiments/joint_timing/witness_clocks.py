"""Local collision repair at a fixed arrival time, in joint-clock coordinates.

Each moving joint has two variables in [0, 1]: how much of its slack to
spend stretching the profile and where to place the remaining wait. The
critical joint has zero slack. This module only proposes a schedule. Every
candidate still needs the model's bounds, derivative and collision checks.
"""

import numpy as np

from benchmarks.generic_motion_v1.runners.phase_search import clock_state


def clock_sensitivity(start, goal, duration, base, ramp0, width, shift, witness_time):
    """Return the diagonal position Jacobians for stretch and delay fractions."""
    slack = np.maximum(duration - base, 0.0)
    span = np.where(base > 0, base + width * slack, 0.0)
    delay = shift * (duration - span)
    ramp = np.divide(ramp0, base, out=np.zeros_like(span), where=base > 0) * span
    phase = np.divide(
        witness_time - delay, span, out=np.zeros_like(span), where=span > 0
    )
    _, speed = clock_state(witness_time, start, goal - start, delay, span, ramp)
    return speed * (shift - phase) * slack, -speed * (duration - span)


def propose(
    start,
    goal,
    T,
    base,
    ramp0,
    width,
    shift,
    time_at_witness,
    gradient,
    *,
    clearance_m=0.0,
    target_margin_m=0.002,
    trust_radius=0.35
):
    """Smallest boxed clock correction giving the requested linearized clearance gain.

    If the requested gain exceeds the local box's support, return the box point
    with maximum predicted gain instead. Each of the two parameter arrays has
    one entry for every joint. A violated constraint with zero derivative gives
    the original schedule.
    """
    start, goal, base, ramp0, width, shift, gradient = (
        np.asarray(arg, dtype=float)
        for arg in (start, goal, base, ramp0, width, shift, gradient)
    )
    arrays = (start, goal, base, ramp0, width, shift, gradient)
    scalars = (T, time_at_witness, clearance_m, target_margin_m, trust_radius)
    if (
        any(x.shape != (7,) or not np.isfinite(x).all() for x in arrays)
        or not np.isfinite(scalars).all()
        or T <= 0
        or not 0 <= time_at_witness <= T
        or not 0 < trust_radius <= 1
        or target_margin_m < 0
        or np.any(base < 0)
        or np.any(base > T + 1e-12)
        or np.any(ramp0 < 0)
        or np.any(2 * ramp0 > base + 1e-12)
    ):
        raise ValueError("Invalid rest-to-rest parameters or collision witness")
    x = np.r_[width, shift]
    if np.any((x < 0) | (x > 1)):
        raise ValueError("Clock fractions must lie between zero and one")
    dw, ds = clock_sensitivity(
        start, goal, T, base, ramp0, width, shift, time_at_witness
    )
    normal = np.r_[gradient * dw, gradient * ds]
    lower = np.maximum(-trust_radius, -x)
    upper = np.minimum(trust_radius, 1.0 - x)
    best = np.where(normal > 0, upper, np.where(normal < 0, lower, 0.0))
    gain = target_margin_m - clearance_m
    support = float(normal @ best)
    if gain <= 0 or support <= 1e-12:
        return width.copy(), shift.copy()
    if gain >= support:
        step = best
    else:
        active = normal != 0
        lo, hi = 0.0, float(np.max(best[active] / normal[active]))
        for _ in range(48):
            mid = 0.5 * (lo + hi)
            if normal @ np.clip(mid * normal, lower, upper) >= gain:
                hi = mid
            else:
                lo = mid
        step = np.clip(hi * normal, lower, upper)
    proposed = np.clip(x + step, 0.0, 1.0)
    return proposed[:7], proposed[7:]
