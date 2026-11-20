"""Fixed-duration projection in joint velocity coordinates.

The variables are the joint velocities at internal time knots, multiplied by
one time step. Integrated positions, position control points and both derivative
bounds are linear at a fixed duration. Collision witnesses supply local cuts.
Failure of the local projection is never a lower bound on feasible duration.
"""

import time

import numpy as np
import osqp
from scipy import sparse


def arc_points(path, fractions):
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    keep = np.r_[True, np.diff(arc) > 0.0]
    return np.stack(
        [np.interp(fractions * arc[-1], arc[keep], p[keep]) for p in path.T], axis=1
    )


def velocity_basis(intervals):
    velocities = np.eye(intervals + 1)[:, 1:-1]
    positions = np.vstack(
        (
            np.zeros(intervals - 1),
            np.cumsum(0.5 * (velocities[:-1] + velocities[1:]), axis=0),
        )
    )
    return velocities, positions


def at_times(parameters, velocities, positions):
    s = np.clip(np.asarray(parameters), 0, 1) * (len(velocities) - 1)
    i = np.minimum(s.astype(int), len(velocities) - 2)
    f = (s - i)[:, None]
    return (
        positions[i]
        + f * velocities[i]
        + 0.5 * f * f * (velocities[i + 1] - velocities[i])
    )


def refine_time(model, path, candidate, lower, velocity, acceleration, cfg, deadline):
    from .validate import sample_candidate

    n = 14
    vel, pos = velocity_basis(n)
    start, goal = path[0], path[-1]
    delta = goal - start
    grid = np.linspace(0.0, 1.0, 2 * n + 1)
    basis = at_times(grid, vel, pos)
    ref = arc_points(path, grid**2 * (3.0 - 2.0 * grid)) - start
    deriv = np.diff(vel, axis=0)
    objective = basis.T @ basis + 0.009 * deriv.T @ deriv + 1e-6 * np.eye(n - 1)
    eye = sparse.eye(7, format="csc")
    P = sparse.triu(sparse.kron(objective, eye), format="csc")
    linear = -(basis.T @ ref).ravel()
    position_bounds = np.vstack((pos[1:-1], pos[:-1] + 0.5 * vel[:-1]))
    constraints = sparse.kron(
        np.vstack((pos[-1:], vel[1:-1], deriv, position_bounds)), eye, format="csc"
    )
    incumbent = candidate
    upper = float(incumbent["durations"].sum())
    probe_low = lower
    observations = []
    last_w = None
    for _ in range(5):
        if time.perf_counter() >= deadline or upper - lower <= 0.01 * max(lower, 0.01):
            break
        target = probe_low + 0.42 * (upper - probe_low)
        dt = target / n
        vl = np.tile(velocity * dt, n - 1)
        al = np.tile(acceleration * dt * dt, n)
        lo = np.r_[delta, -vl, -al, np.tile(model.lower - start, len(position_bounds))]
        hi = np.r_[delta, vl, al, np.tile(model.upper - start, len(position_bounds))]
        extra = []
        cut_lower = []
        record = dict(target_s=float(target), collision_cuts=0, accepted=False)
        observations.append(record)
        for _ in range(6):
            remaining = deadline - time.perf_counter()
            if remaining < 0.015:
                break
            A = (
                sparse.vstack([constraints, *extra], format="csc")
                if extra
                else constraints
            )
            solver = osqp.OSQP()
            solver.setup(
                P=P,
                q=linear,
                A=A,
                l=np.r_[lo, cut_lower],
                u=np.r_[hi, np.full(len(cut_lower), np.inf)],
                verbose=False,
                eps_abs=1e-7,
                eps_rel=1e-7,
                max_iter=3000,
                time_limit=min(0.15, remaining),
            )
            if last_w is not None:
                solver.warm_start(x=last_w.ravel())
            solution = solver.solve(raise_error=False)
            record["solver_status"] = solution.info.status
            if solution.x is None or solution.info.status_val not in (1, 2):
                break
            w = np.asarray(solution.x).reshape(n - 1, 7)
            # Enforce the endpoint equality to roundoff before forming curves.
            if np.max(np.abs(w.sum(axis=0) - delta)) > 2e-5:
                break
            w = w + (delta - w.sum(axis=0)) / (n - 1)
            last_w = w
            knots = start + pos @ w
            mid = knots[:-1] + 0.5 * (vel[:-1] @ w)
            controls = np.stack((knots[:-1], mid, knots[1:]), axis=1)
            if np.any(controls < model.lower - 1e-7) or np.any(
                controls > model.upper + 1e-7
            ):
                break
            scale = max(
                1.0,
                float(np.max(np.abs(vel @ w) / (dt * velocity))),
                float(np.sqrt(np.max(np.abs(deriv @ w) / (dt**2 * acceleration)))),
            )
            duration = target * scale
            if duration >= upper - 1e-5:
                break
            curve = dict(
                kind="bezier",
                objective="minimum_time",
                controls=controls,
                durations=np.full(n, duration / n),
            )
            sample = sample_candidate(curve, cfg)
            bad = np.flatnonzero(~model.cuda(sample["q"]))
            if not len(bad):
                incumbent = curve
                upper = duration
                record.update(accepted=True, duration_s=duration)
                break
            sections = np.split(bad, np.flatnonzero(np.diff(bad) > 1) + 1)
            for section in sorted(sections, key=len, reverse=True)[:4]:
                idx = section[len(section) // 2]
                q = sample["q"][idx]
                clearance, grad = clearance_gradient(model, q)
                if np.linalg.norm(grad) < 1e-7:
                    continue
                phi = at_times([sample["time"][idx] / duration], vel, pos)[0]
                extra.append(sparse.csr_matrix(np.outer(phi, grad).reshape(1, -1)))
                cut_lower.append(0.002 - clearance + grad @ (q - start))
            if len(extra) == record["collision_cuts"]:
                break
            record["collision_cuts"] = len(extra)
        if not record["accepted"]:
            probe_low = target
    return incumbent, observations


def refine_path(model, path, cfg, deadline):
    """Successive weighted-length projections with sampled collision cuts."""
    from .validate import sample_candidate

    n = 13
    positions = np.eye(n)[:, 1:-1]
    differences = np.diff(positions, axis=0)
    fixed = np.zeros((n, 7))
    fixed[0], fixed[-1] = path[0], path[-1]
    reference = arc_points(path, np.linspace(0, 1, n))
    incumbent = path
    best = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    eye = sparse.eye(7, format="csc")
    box = sparse.eye((n - 2) * 7, format="csc")
    rows = []
    cut_lo = []
    observations = []
    for _ in range(10):
        remaining = deadline - time.perf_counter()
        if remaining < 0.015:
            break
        weights = 1 / np.maximum(
            np.linalg.norm(np.diff(reference, axis=0), axis=1), 0.025
        )
        prox = 0.065
        objective = differences.T @ (weights[:, None] * differences) + prox * np.eye(
            n - 2
        )
        P = sparse.triu(sparse.kron(objective, eye), format="csc")
        linear = (
            differences.T @ (weights[:, None] * np.diff(fixed, axis=0))
            - prox * reference[1:-1]
        )
        A = sparse.vstack([box, *rows], format="csc") if rows else box
        solver = osqp.OSQP()
        solver.setup(
            P=P,
            q=linear.ravel(),
            A=A,
            l=np.r_[np.tile(model.lower, n - 2), cut_lo],
            u=np.r_[np.tile(model.upper, n - 2), np.full(len(rows), np.inf)],
            verbose=False,
            eps_abs=1e-6,
            eps_rel=1e-6,
            max_iter=3000,
            time_limit=min(0.1, remaining),
        )
        solver.warm_start(x=reference[1:-1].ravel())
        solution = solver.solve(raise_error=False)
        if solution.x is None or solution.info.status_val not in (1, 2):
            break
        points = np.vstack(
            (
                path[0],
                np.clip(solution.x.reshape(-1, 7), model.lower, model.upper),
                path[-1],
            )
        )
        candidate = dict(kind="polygon", objective="minimum_distance", points=points)
        sample = sample_candidate(candidate, cfg)
        bad = np.flatnonzero(~model.cuda(sample["q"]))
        cost = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        observations.append(dict(cost_rad=cost, collision_samples=len(bad)))
        if not len(bad):
            if cost >= best - 1e-5:
                break
            incumbent = points
            best = cost
            reference = points
            continue
        sections = np.split(bad, np.flatnonzero(np.diff(bad) > 1) + 1)
        before = len(rows)
        for section in sorted(sections, key=len, reverse=True)[:5]:
            idx = section[len(section) // 2]
            seg = sample["segment"][idx]
            q = sample["q"][idx]
            diff = points[seg + 1] - points[seg]
            blend = np.clip((q - points[seg]) @ diff / max(diff @ diff, 1e-12), 0, 1)
            basis = (1 - blend) * positions[seg] + blend * positions[seg + 1]
            offset = (1 - blend) * fixed[seg] + blend * fixed[seg + 1]
            clearance, grad = clearance_gradient(model, q)
            if np.linalg.norm(grad) < 1e-7:
                continue
            rows.append(sparse.csr_matrix(np.outer(basis, grad).reshape(1, -1)))
            cut_lo.append(0.002 - clearance + grad @ (q - offset))
        if len(rows) == before:
            break
    return incumbent, observations


def clearance_gradient(model, q):
    plus = np.minimum(model.upper, q + np.eye(7) * 1e-4)
    minus = np.maximum(model.lower, q - np.eye(7) * 1e-4)
    distance = model.clearance(np.vstack((q, plus, minus)))
    return distance[0], (distance[1:8] - distance[8:]) / np.diag(plus - minus)
