"""Collision-guided search over the timing slack of independent joint motions.

A rest-to-rest joint has an analytic minimum travel time. Slower joints fix the
earliest arrival. Other joints may wait, stretch their motion, or finish early.
All schedules use trapezoidal or triangular velocity profiles. Their union of
switch times gives exact C1 quadratic Bezier segments. A geometric detour and
OMPL seed extend the search to motions requiring a joint reversal.
"""

from __future__ import annotations

from pathlib import Path
import time

import numpy as np

from ..roadmap import write
from .validate import sample_candidate
from .phase_refinement import clearance_gradient, refine_path, refine_time

PROFILE = {
    "duration_scales": (1.0, 1.25, 1.6, 2.1),
    "schedules_per_scale": 25,
    "coarse_stations": 33,
    "detour_samples_per_round": 96,
    "detour_rounds": 3,
    "local_budget_s": 4.0,
    "graph_budget_s": 3.5,
    "projection_budget_s": 1.2,
    "validation_reserve_s": 5.0,
}


def rest_times(delta, velocity, acceleration):
    distance = np.abs(delta)
    ramp = np.minimum(np.sqrt(distance / acceleration), velocity / acceleration)
    return (
        ramp
        + np.divide(
            distance, acceleration * ramp, out=np.zeros_like(ramp), where=ramp > 0
        ),
        ramp,
    )


def scalar_motion(start, goal, velocity, acceleration):
    """Fastest rest-to-rest motion constrained to this straight line."""
    delta = goal - start
    moving = np.abs(delta) > 0
    if not np.any(moving):
        return np.full_like(delta, 1e-3), np.full_like(delta, 0.5e-3)
    speed = np.min(velocity[moving] / np.abs(delta[moving]))
    accel = np.min(acceleration[moving] / np.abs(delta[moving]))
    duration, ramp = rest_times(np.asarray([1.0]), speed, accel)
    return np.full_like(delta, duration.item()), np.full_like(delta, ramp.item())


def clock_state(times, start, delta, delay, span, ramp):
    """Evaluate a bank of schedules at normalized or physical sample times."""
    local = np.clip(times - delay, 0, span)
    safe_ramp = np.where(ramp > 0, ramp, 1.0)
    cruise_speed = np.divide(
        delta, span - ramp, out=np.zeros_like(span), where=span > ramp
    )
    before = np.minimum(local, safe_ramp)
    after = np.minimum(span - local, safe_ramp)
    travel = np.where(
        local <= ramp,
        before**2 / (2 * safe_ramp),
        np.where(
            local >= span - ramp,
            span - ramp - after**2 / (2 * safe_ramp),
            local - 0.5 * ramp,
        ),
    )
    position = start + cruise_speed * travel
    speed = cruise_speed * np.minimum(before, after) / safe_ramp
    return position, speed


def clock_curve(start, goal, duration, delay, span, ramp):
    end = delay + span
    events = np.sort(
        np.clip(
            np.r_[0.0, duration, delay, delay + ramp, end - ramp, end], 0.0, duration
        )
    )
    # Coincident switches and machine-scale intervals have no resolvable
    # physical duration. The independent derivative-hull test still applies.
    knots = [0.0]
    for event in events:
        if event - knots[-1] > 1e-5 and duration - event > 1e-5:
            knots.append(float(event))
    knots.append(float(duration))
    knots = np.array(knots)
    q, v = clock_state(knots[:, None], start, goal - start, delay, span, ramp)
    q[0], q[-1] = start, goal
    intervals = np.diff(knots)
    controls = np.stack(
        (q[:-1], q[:-1] + 0.5 * intervals[:, None] * v[:-1], q[1:]), axis=1
    )
    return dict(
        kind="bezier", objective="minimum_time", controls=controls, durations=intervals
    )


def line_curve(start, goal, velocity, acceleration):
    span, ramp = scalar_motion(start, goal, velocity, acceleration)
    return clock_curve(start, goal, span[0], np.zeros_like(start), span, ramp)


def polygon(points):
    return dict(
        kind="polygon", objective="minimum_distance", points=np.asarray(points, float)
    )


def length(points):
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def dense_free(model, candidate, cfg):
    return bool(np.all(model.cuda(sample_candidate(candidate, cfg)["q"])))


def collision_witness(model, line, mask):
    """Use the middle of the longest blocked interval to steer joint clocks."""
    ids = np.flatnonzero(~mask)
    groups = np.split(ids, np.flatnonzero(np.diff(ids) > 1) + 1)
    longest = max(groups, key=len)
    index = longest[len(longest) // 2]
    q = line[index]
    _, gradient = clearance_gradient(model, q)
    direction = line[-1] - line[0]
    return float((q - line[0]) @ direction / (direction @ direction)), gradient


def schedules(model, start, goal, vlim, alim, rng, gradient, cfg, deadline):
    base, ramp0 = rest_times(goal - start, vlim, alim)
    lower = float(base.max())
    n = PROFILE["schedules_per_scale"]
    coarse = np.linspace(0, 1, PROFILE["coarse_stations"])
    lead = (gradient * (goal - start) < 0).astype(float)
    for scale in PROFILE["duration_scales"]:
        if time.perf_counter() >= deadline:
            break
        duration = lower * scale
        width = rng.uniform(0, 1, (n, 7))
        shift = rng.uniform(0, 1, (n, 7))
        width[:3] = 0
        shift[:3] = [lead, 1 - lead, np.full(7, 0.5)]
        # Proposals guided by the measured clearance derivative coexist with
        # unbiased timing samples, since other collisions can reverse its sign.
        shift[3 : n // 2] = 0.7 * lead + 0.3 * shift[3 : n // 2]
        span = np.where(base > 0, base + width * (duration - base), 0.0)
        ramps = np.divide(ramp0, base, out=np.zeros(7), where=base > 0) * span
        delay = shift * (duration - span)
        q, _ = clock_state(
            duration * coarse[None, :, None],
            start,
            goal - start,
            delay[:, None],
            span[:, None],
            ramps[:, None],
        )
        free = model.cuda(q.reshape(-1, 7)).reshape(n, -1).all(axis=1)
        sizes = np.linalg.norm(np.diff(q, axis=1), axis=2).sum(axis=1)
        for idx in np.argsort(sizes):
            if not free[idx] or time.perf_counter() >= deadline:
                continue
            curve = clock_curve(
                start, goal, duration, delay[idx], span[idx], ramps[idx]
            )
            if dense_free(model, curve, cfg):
                yield curve, q[idx], scale
                break


def via_point(
    model,
    start,
    goal,
    frac,
    gradient,
    rng,
    velocity,
    acceleration,
    objective,
    cfg,
    deadline,
):
    """Refine a batch of two-edge detours around measured collision intervals."""
    n = PROFILE["detour_samples_per_round"]
    u = np.linspace(0, 1, PROFILE["coarse_stations"])[:, None]
    centre = start + frac * (goal - start)
    elite = None
    for iteration in range(PROFILE["detour_rounds"]):
        if time.perf_counter() >= deadline:
            break
        locations = np.clip(rng.normal(frac, 0.22, (n, 1)), 0.05, 0.95)
        centres = start + locations * (goal - start)
        scales = rng.choice([0.22, 0.55, 1.0, 1.7], (n, 1))
        proposals = centres + scales * rng.normal(size=(n, 7))
        if elite is not None:
            proposals[: n // 2] = (
                elite[rng.integers(len(elite), size=n // 2)]
                + scales[: n // 2] * rng.normal(size=(n // 2, 7)) * 0.6
            )
        if np.linalg.norm(gradient) > 0:
            proposals[:8] = centre + np.linspace(0.15, 2.5, 8)[
                :, None
            ] * gradient / np.linalg.norm(gradient)
        proposals = np.clip(proposals, model.lower, model.upper)
        left = start + u[None] * (proposals - start)[:, None]
        right = proposals[:, None] + u[None] * (goal - proposals)[:, None]
        masks = model.cuda(
            np.concatenate((left, right), axis=1).reshape(-1, 7)
        ).reshape(n, -1)
        blocked = np.count_nonzero(~masks, axis=1)
        cost = np.linalg.norm(proposals - start, axis=1) + np.linalg.norm(
            proposals - goal, axis=1
        )
        if objective == "minimum_time":
            cost = rest_times(proposals - start, velocity, acceleration)[0].max(
                axis=1
            ) + rest_times(goal - proposals, velocity, acceleration)[0].max(axis=1)
        order = np.lexsort((cost, blocked))
        elite = proposals[order[:8]]
        for idx in order:
            if blocked[idx] or time.perf_counter() >= deadline:
                break
            path = np.array([start, proposals[idx], goal])
            if dense_free(model, polygon(path), cfg):
                return path
    return None


def visibility_path(model, start, goal, rng, deadline, cfg):
    """Search an online visibility graph containing local and global samples."""
    from scipy.spatial import cKDTree
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra

    bank = np.array([start, goal])
    for n in (600, 1400, 2000):
        if time.perf_counter() >= deadline:
            break
        mix = rng.uniform(0, 1, (n, 1))
        centres = start + mix * (goal - start)
        centres[: n // 4] = start
        centres[n // 4 : n // 2] = goal
        scale = rng.choice([0.2, 0.5, 1.0, 1.7], (n, 1))
        proposals = centres + rng.normal(size=(n, 7)) * scale
        proposals[-n // 5 :] = rng.uniform(
            model.lower, model.upper, (len(proposals[-n // 5 :]), 7)
        )
        inside = np.all((proposals >= model.lower) & (proposals <= model.upper), axis=1)
        proposals = proposals[inside]
        bank = np.vstack((bank, proposals[model.cuda(proposals)]))
        if len(bank) < 4:
            continue
        tree = cKDTree(bank)
        neighbours = tree.query(bank, k=min(16, len(bank)))[1][:, 1:]
        pairs = np.column_stack(
            (np.repeat(np.arange(len(bank)), neighbours.shape[1]), neighbours.ravel())
        )
        targets = tree.query(bank[:2], k=min(48, len(bank)))[1][:, 1:]
        pairs = np.vstack(
            (
                pairs,
                np.column_stack((np.repeat([0, 1], targets.shape[1]), targets.ravel())),
            )
        )
        pairs = np.unique(np.sort(pairs, axis=1), axis=0)
        av, bv = bank[pairs[:, 0]], bank[pairs[:, 1]]
        alive = np.zeros(len(pairs), bool)
        u = np.linspace(0.1, 0.9, 9)[None, :, None]
        for idx in range(0, len(pairs), 128):
            if time.perf_counter() >= deadline:
                return None
            end = idx + 128
            q = av[idx:end, None] + u * (bv[idx:end, None] - av[idx:end, None])
            alive[idx:end] = (
                model.cuda(q.reshape(-1, 7)).reshape(len(q), -1).all(axis=1)
            )
        weights = np.linalg.norm(bv - av, axis=1)
        lookup = {tuple(row): i for i, row in enumerate(pairs)}
        for _ in range(35):
            if time.perf_counter() >= deadline:
                return None
            edges = pairs[alive]
            if not len(edges):
                break
            graph = coo_matrix(
                (weights[alive], edges.T), shape=(len(bank), len(bank))
            ).tocsr()
            dist, parent = dijkstra(
                graph, directed=False, indices=0, return_predecessors=True
            )
            if not np.isfinite(dist[1]):
                break
            route = [1]
            while route[-1] != 0:
                route.append(int(parent[route[-1]]))
            route = route[::-1]
            valid = True
            for a, b in zip(route, route[1:]):
                if not dense_free(model, polygon(bank[[a, b]]), cfg):
                    alive[lookup[tuple(sorted((a, b)))]] = False
                    valid = False
            if valid:
                return bank[route]
    return None


def shorten(model, points, cfg, deadline):
    """Cut verified corners at interior edge points as well as stored vertices."""
    points = np.asarray(points)
    for _ in range(3):
        changed = False
        for i in range(len(points) - 2, 0, -1):
            if time.perf_counter() >= deadline:
                return points
            a, b, c = points[i - 1 : i + 2]
            cuts = np.array([1.0, 0.8, 0.6, 0.4, 0.2])
            left, right = b + cuts[:, None] * (a - b), b + cuts[:, None] * (c - b)
            tests = (
                left[:, None]
                + np.linspace(0, 1, 17)[None, :, None] * (right - left)[:, None]
            )
            valid = model.cuda(tests.reshape(-1, 7)).reshape(len(cuts), -1).all(axis=1)
            for k in np.flatnonzero(valid):
                arc = length([left[k], b, right[k]])
                if arc - np.linalg.norm(right[k] - left[k]) < 1e-3:
                    break
                if dense_free(model, polygon([left[k], right[k]]), cfg):
                    replacement = (
                        np.empty((0, 7)) if k == 0 else np.array([left[k], right[k]])
                    )
                    points = np.vstack((points[:i], replacement, points[i + 1 :]))
                    changed = True
                    break
        if not changed or len(points) > 45:
            break
    return points


def blended(points, velocity, acceleration, fraction):
    """Round polygon corners with C1 constant-acceleration transitions."""
    delta = np.diff(points, axis=0)
    travel = np.max(np.abs(delta) / velocity, axis=1) / fraction
    speeds = delta / travel[:, None]
    junctions = np.vstack((np.zeros(7), speeds, np.zeros(7)))
    half = 0.5 * np.max(np.abs(np.diff(junctions, axis=0)) / acceleration, axis=1)
    dilation = max(1.0, np.sqrt(np.max((half[:-1] + half[1:]) / travel)))
    speeds /= dilation
    travel *= dilation
    half /= dilation
    controls, durations = [], []

    def add(a, b, c, duration):
        if duration > 1e-9:
            controls.append([a, b, c])
            durations.append(duration)

    zero = np.zeros(7)
    for i, p in enumerate(points):
        previous = speeds[i - 1] if i else zero
        following = speeds[i] if i < len(speeds) else zero
        add(p - half[i] * previous, p, p + half[i] * following, 2 * half[i])
        if i < len(speeds):
            a = p + half[i] * following
            b = points[i + 1] - half[i + 1] * following
            add(a, 0.5 * (a + b), b, travel[i] - half[i] - half[i + 1])
    return dict(
        kind="bezier",
        objective="minimum_time",
        controls=np.array(controls),
        durations=np.array(durations),
    )


def time_path(model, points, velocity, acceleration, cfg, deadline):
    points = points[
        np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-10]
    ]
    segments = [
        line_curve(a, b, velocity, acceleration)
        for a, b in zip(points[:-1], points[1:])
    ]
    incumbent = dict(
        kind="bezier",
        objective="minimum_time",
        controls=np.concatenate([s["controls"] for s in segments]),
        durations=np.concatenate([s["durations"] for s in segments]),
    )
    for fraction in (1.0, 0.7, 0.45):
        if time.perf_counter() >= deadline:
            break
        curve = blended(points, velocity, acceleration, fraction)
        if curve["durations"].sum() < incumbent["durations"].sum() and dense_free(
            model, curve, cfg
        ):
            incumbent = curve
    return incumbent


def solve(model, case, cfg, out, deadline, seed, method):
    import osqp

    start_time = time.perf_counter()
    out = Path(out)
    start, goal = np.asarray(case["q_start"]), np.asarray(case["q_goal"])
    velocity = np.asarray(case["velocity_limits_rad_s"])
    acceleration = np.asarray(case["acceleration_limits_rad_s2"])
    timed = case["_objective"] == "minimum_time"
    rng = np.random.Generator(np.random.PCG64(seed))
    lower = (
        float(rest_times(goal - start, velocity, acceleration)[0].max())
        if timed
        else length([start, goal])
    )
    trace = dict(
        profile=PROFILE,
        osqp_version=osqp.__version__,
        joint_clocks=cfg.get("phase_search", {}).get("joint_clocks", True),
        method=method,
        method_seed=int(seed),
        objective_lower_bound=lower,
        objective=case["_objective"],
        stages=[],
    )

    def record(stage):
        trace["stages"].append(
            dict(stage=stage, elapsed_s=time.perf_counter() - start_time)
        )
        write(out / "phase_search.json", trace)

    def finish(candidate, origin):
        cost = (
            float(candidate["durations"].sum())
            if timed
            else length(candidate["points"])
        )
        trace.update(
            candidate_source=origin,
            candidate_cost=cost,
            unvalidated_gap_to_bound=cost - lower,
        )
        record("candidate_returned")
        return dict(
            **candidate, solver_success=True, solver="phase search", provenance=origin
        )

    direct = (
        line_curve(start, goal, velocity, acceleration)
        if timed
        else polygon([start, goal])
    )
    if dense_free(model, direct, cfg):
        return finish(direct, "straight")
    coarse = np.linspace(start, goal, PROFILE["coarse_stations"])
    mask = model.cuda(coarse)
    if np.all(mask):
        coarse = sample_candidate(polygon([start, goal]), cfg)["q"]
        mask = model.cuda(coarse)
    if np.all(mask):
        # Direct line and timed sampling can resolve different thin contacts.
        coarse = sample_candidate(direct, cfg)["q"]
        mask = model.cuda(coarse)
    frac, gradient = collision_witness(model, coarse, mask)
    trace["blocked_chord_fraction"] = frac
    record("blocked_chord")
    local_deadline = min(
        deadline - PROFILE["validation_reserve_s"],
        start_time + PROFILE["local_budget_s"],
    )
    best = None
    curve_seed = None
    origin = "joint_clocks"
    trials = (
        schedules(
            model,
            start,
            goal,
            velocity,
            acceleration,
            rng,
            gradient,
            cfg,
            local_deadline,
        )
        if trace["joint_clocks"]
        else ()
    )
    for curve, samples, scale in trials:
        if timed:
            if float(curve["durations"].sum()) <= lower * (1 + 1e-8):
                return finish(curve, origin)
            curve_seed = curve
            best = samples
            break
        if dense_free(model, polygon(samples), cfg) and (
            best is None or length(samples) < length(best)
        ):
            best = samples
    path = best
    if path is None:
        record("via_point_search")
        path = via_point(
            model,
            start,
            goal,
            frac,
            gradient,
            rng,
            velocity,
            acceleration,
            case["_objective"],
            cfg,
            local_deadline,
        )
        origin = "sampled_waypoint"
    if path is None:
        record("visibility_graph")
        graph_deadline = min(
            deadline - PROFILE["validation_reserve_s"],
            time.perf_counter() + PROFILE["graph_budget_s"],
        )
        path = visibility_path(model, start, goal, rng, graph_deadline, cfg)
        origin = "visibility_graph"
    if path is None:
        from .ompl_baselines import solve as ompl_solve

        record("rrt_seed")
        seed_case = dict(case, _objective="minimum_distance")
        seed_path = ompl_solve(
            model, seed_case, cfg, out / "rrt_seed", deadline - 1.5, seed, "RRTCONNECT"
        )
        path = seed_path["points"]
        origin = "rrt_seed"
    refine_deadline = min(
        time.perf_counter() + PROFILE["projection_budget_s"],
        deadline - PROFILE["validation_reserve_s"],
    )
    path = np.asarray(path)
    if timed:
        candidate = curve_seed or time_path(
            model, path, velocity, acceleration, cfg, refine_deadline
        )
        if candidate["durations"].sum() > lower * (1.0 + 1e-7):
            candidate, observations = refine_time(
                model,
                path,
                candidate,
                lower,
                velocity,
                acceleration,
                cfg,
                refine_deadline,
            )
            trace["duration_projection"] = observations
    else:
        path = shorten(
            model, path, cfg, min(refine_deadline, time.perf_counter() + 0.35)
        )
        path, observations = refine_path(model, path, cfg, refine_deadline)
        trace["length_projection"] = observations
        candidate = polygon(path)
    return finish(candidate, origin)
