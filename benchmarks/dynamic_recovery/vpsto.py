"""B5: VP-STO and CC-VPSTO (via-point stochastic trajectory optimization), re-implemented from the papers.

VP-STO (Jankowski et al., ICRA 2023) represents a trajectory by a few via points between the current state and
the goal; the trajectory through them is the minimum-acceleration (cubic) spline with the boundary velocities
fixed, and its duration is the shortest one that satisfies the velocity and acceleration limits. CMA-ES optimizes
the via points for duration plus penalties; in the online (MPC) version the optimization restarts from the
latest state at every control step, warm-started with the previous solution, and the best trajectory replaces the
commanded one. CC-VPSTO (Brudermüller et al., 2024) replaces the deterministic collision penalty by a chance
constraint evaluated on sampled obstacle predictions (here: Monte-Carlo velocity perturbations).

Implementation notes. The spline is the clamped cubic B-spline interpolating [q_s, via_1..via_n, q_goal] at
uniform normalized times with velocity v_s at the start and rest at the end; its coefficients are linear in the
data, and its derivative control points (the hull the benchmark checks) scale as 1/T and 1/T^2 except for the
start-velocity term, so the minimum duration is found in closed form as the largest feasible x = 1/T of linear
and quadratic inequalities. The candidate is submitted as ``kind='bspline'`` (order 4). Moving obstacles are
predicted at constant velocity with the same growing envelope as the other methods; the static cost compares
with the clearance of the nominal route (see ``RouteReference``).
"""
from __future__ import annotations

import time

import numpy as np

from .recovery import GOAL_TOLERANCE_RAD, VOXEL_RADIUS_M, Obstacles, RouteReference, common_setup, validate
from .trajectory import TIMED, Plan


def _spline_matrices(n_via):
    """Unit-duration cubic interpolation: knots, coefficient map M (C x (n_via + 4)) and derivative maps."""
    from scipy.interpolate import BSpline, make_interp_spline
    m = n_via + 2
    sites = np.linspace(0.0, 1.0, m)
    columns = []
    for k in range(m + 2):
        y = np.zeros((m, 1))
        d0, d1 = np.zeros(1), np.zeros(1)
        if k < m:
            y[k, 0] = 1.0
        elif k == m:
            d0[0] = 1.0
        else:
            d1[0] = 1.0
        spline = make_interp_spline(sites, y, k=3, bc_type=([(1, d0)], [(1, d1)]))
        columns.append(spline.c[:, 0])
    knots = spline.t
    M = np.stack(columns, axis=1)
    count = M.shape[0]

    def derivative_map(order):
        rows = []
        for i in range(count):
            e = np.zeros(count)
            e[i] = 1.0
            rows.append(BSpline(knots, e, 3).derivative(order).c[: count - order])
        return np.stack(rows, axis=1)

    return knots, M, derivative_map(1), derivative_map(2)


class VPSTO:
    def __init__(self, budget_s=0.1, via_points=3, population=16, sigma_rad=0.15, samples=32, margin_m=0.05,
                 w_obstacle=200.0, w_static=200.0, w_limits=60.0, w_contact=50.0, chance=False, chance_samples=8,
                 sigma_v=0.1, epsilon=0.05, w_chance=50.0, finish_s=0.25, seed=0):
        self.budget, self.n_via, self.population, self.sigma0 = budget_s, via_points, population, sigma_rad
        self.samples, self.margin = samples, margin_m
        self.w = dict(obstacle=w_obstacle, static=w_static, limits=w_limits, chance=w_chance, contact=w_contact)
        self.chance, self.chance_samples, self.sigma_v, self.epsilon = chance, chance_samples, sigma_v, epsilon
        self.finish, self.seed = finish_s, seed

    def setup(self, problem):
        from scipy.interpolate import BSpline
        common_setup(self, problem)
        self.reference = RouteReference(self.route, self.arm, self.scene)
        self.knots, self.M, self.D1, self.D2 = _spline_matrices(self.n_via)
        self.s = np.linspace(0.0, 1.0, self.samples)
        count = self.M.shape[0]
        self.B = BSpline.design_matrix(self.s, self.knots, 3).toarray()
        self.dB = np.stack([BSpline(self.knots, np.eye(count)[i], 3).derivative(1)(self.s) for i in range(count)], 1)
        self.rng = np.random.default_rng(self.seed + int.from_bytes(problem["task_id"].encode()[-4:], "little"))
        self.previous = None
        return None

    # -- trajectory family
    def _coefficients(self, qs, vs, via, T):
        """Spline coefficients (P, C, 7) for via points (P, n, 7) and durations T (P,)."""
        P = len(via)
        data = np.concatenate([np.broadcast_to(qs, (P, 1, 7)), via, np.broadcast_to(self.goal, (P, 1, 7))], axis=1)
        m = self.n_via + 2
        c = np.einsum("ck,pkj->pcj", self.M[:, :m], data)
        return c + T[:, None, None] * self.M[:, m][None, :, None] * vs[None, None, :]

    def _durations(self, qs, vs, via):
        """Shortest feasible duration per candidate under the derivative-hull velocity/acceleration bounds."""
        P = len(via)
        data = np.concatenate([np.broadcast_to(qs, (P, 1, 7)), via, np.broadcast_to(self.goal, (P, 1, 7))], axis=1)
        m = self.n_via + 2
        cq = np.einsum("ck,pkj->pcj", self.M[:, :m], data)
        cv = self.M[:, m][:, None] * vs[None, :]
        a, b = np.einsum("dc,pcj->pdj", self.D1, cq), (self.D1 @ cv)[None]
        c2, d2 = np.einsum("dc,pcj->pdj", self.D2, cq), (self.D2 @ cv)[None]
        vmax, amax = self.limits.velocity, self.limits.acceleration
        with np.errstate(divide="ignore", invalid="ignore"):
            xv = np.where(a > 1e-12, (vmax - b) / a, np.where(a < -1e-12, (vmax + b) / -a, np.inf))
            bounds = [xv.reshape(P, -1).min(axis=1)]
            for sign in (1.0, -1.0):
                gamma = -sign * amax
                alpha, beta = c2, np.broadcast_to(d2, c2.shape)
                disc = beta ** 2 - 4.0 * alpha * gamma
                root = np.sqrt(np.maximum(disc, 0.0))
                r1 = np.where(np.abs(alpha) > 1e-12, (-beta + root) / (2 * alpha), -gamma / np.where(np.abs(beta) > 1e-12, beta, np.nan))
                r2 = np.where(np.abs(alpha) > 1e-12, (-beta - root) / (2 * alpha), np.nan)
                valid = lambda r: np.where((disc >= 0) & (r > 1e-9), r, np.inf)
                bounds.append(np.minimum(valid(r1), valid(r2)).reshape(P, -1).min(axis=1))
        x = np.min(np.stack(bounds), axis=0)
        x = np.where(np.isfinite(x), 0.995 * x, 1.0 / 0.05)
        return np.maximum(1.0 / np.maximum(x, 1e-6), 0.05)

    def _evaluate(self, qs, vs, via, obstacles, perturbations):
        T = self._durations(qs, vs, via)
        c = self._coefficients(qs, vs, via, T)
        Q = np.einsum("kc,pcj->pkj", self.B, c)
        V = np.einsum("kc,pcj->pkj", self.dB, c) / T[:, None, None]
        times = self.s[None] * T[:, None]
        w = self.w
        cost = T.copy()
        e, _, ref = self.reference.project(Q)
        centres = self.arm.spheres(Q.reshape(-1, 7)).reshape(Q.shape[:-1] + (-1, 3))
        static = self.scene.distance(centres, self.arm.sphere_radius).min(axis=-1)
        ds = 1.0 / (self.samples - 1)
        cost += w["static"] * (np.maximum(ref - static, 0.0) * ds).sum(axis=1)
        cost += w["contact"] * (static < ref - 0.03).any(axis=1)
        low = np.maximum(self.limits.lower + 0.02 - c, 0.0) + np.maximum(c - self.limits.upper + 0.02, 0.0)
        cost += w["limits"] * low.sum(axis=(1, 2))
        collision = np.zeros(len(via))
        if len(obstacles):
            worst = np.full(Q.shape[:2], np.inf)
            draws = []
            for j, item in enumerate(obstacles.items):
                predicted = item["p"] + times[..., None] * item["v"]
                d = obstacles.surface_distance(j, centres, predicted) - VOXEL_RADIUS_M - self.arm.sphere_radius
                worst = np.minimum(worst, d.min(axis=-1) - obstacles.inflation(times, j))
                if self.chance:
                    for dv in perturbations[j]:
                        moved = predicted + times[..., None] * dv
                        dd = obstacles.surface_distance(j, centres, moved) - VOXEL_RADIUS_M - self.arm.sphere_radius
                        draws.append(dd.min(axis=(-1, -2)) < 0.0)
            violation = np.maximum(self.margin - worst, 0.0)
            cost += w["obstacle"] * (violation * ds).sum(axis=1)
            collision = (worst < 0.0).any(axis=1).astype(float)
            cost += w["contact"] * collision  # VP-STO's large constant penalty for a colliding trajectory
            if self.chance and draws:
                probability = np.mean(np.stack(draws), axis=0)
                cost += w["chance"] * np.maximum(probability - self.epsilon, 0.0)
        return cost, T, c, collision

    def _warm_start(self, qs, t_s):
        """Previous solution re-sampled at the new via times (so progress carries over); else the route."""
        if self.previous is not None:
            spline, start, duration = self.previous
            elapsed = t_s - start
            if 0.0 <= elapsed < duration - 1e-3:
                fractions = np.linspace(0.0, 1.0, self.n_via + 2)[1:-1]
                return spline(elapsed + fractions * (duration - elapsed))
        return self._initial_mean(qs)

    def _initial_mean(self, qs):
        """Via points spread along the remaining nominal route (equal arc length)."""
        _, s0, _ = self.reference.project(qs[None])
        targets = np.linspace(float(s0[0]), self.reference.length, self.n_via + 2)[1:-1]
        index = np.searchsorted(self.reference.arc, targets).clip(0, len(self.reference.points) - 1)
        return self.reference.points[index].copy()

    def step(self, observation):
        start = time.perf_counter()
        t = observation["time_s"]
        if observation["supervisor"]["protective_stop"]:
            return None
        plan = Plan.from_list(observation["plan"])
        q_now = np.asarray(observation["robot"]["q"], float)
        to_goal = np.max(np.abs(plan.final_configuration() - self.goal)) <= 1e-6
        if to_goal and np.max(np.abs(q_now - self.goal)) <= GOAL_TOLERANCE_RAD:
            return None
        t_s = t + self.budget
        qs, vs, _ = plan.at(t_s)
        if to_goal and plan.segments[-1].end - t_s < self.finish:
            return None  # the last fraction of a second: keep the committed approach to the goal
        obstacles = Obstacles(observation["obstacles"])
        perturbations = [self.rng.normal(0.0, self.sigma_v, (self.chance_samples, 3)) for _ in obstacles.items]
        dim = self.n_via * 7
        mean = self._warm_start(qs, t_s).reshape(-1)
        # CMA-ES (Hansen's standard settings, full covariance).
        lam = self.population
        mu = lam // 2
        weights = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1))
        weights /= weights.sum()
        mueff = 1.0 / np.sum(weights ** 2)
        cc, cs = (4 + mueff / dim) / (dim + 4 + 2 * mueff / dim), (mueff + 2) / (dim + mueff + 5)
        c1 = 2 / ((dim + 1.3) ** 2 + mueff)
        cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((dim + 2) ** 2 + mueff))
        damps = 1 + 2 * max(0.0, np.sqrt((mueff - 1) / (dim + 1)) - 1) + cs
        chi = np.sqrt(dim) * (1 - 1 / (4 * dim) + 1 / (21 * dim ** 2))
        C, pc, ps, sigma = np.eye(dim), np.zeros(dim), np.zeros(dim), self.sigma0
        best = None
        generation, last = 0, 0.0
        while best is None or time.perf_counter() - start + 1.2 * last < 0.9 * self.budget:
            began = time.perf_counter()
            eigval, B = np.linalg.eigh(C)
            D = np.sqrt(np.maximum(eigval, 1e-20))
            z = self.rng.standard_normal((lam, dim))
            y = (z * D) @ B.T
            X = mean[None] + sigma * y
            if generation == 0:
                X[0] = mean
            via = np.clip(X.reshape(lam, self.n_via, 7), self.limits.lower, self.limits.upper)
            cost, T, c, collision = self._evaluate(qs, vs, via, obstacles, perturbations)
            order = np.argsort(cost)
            if best is None or cost[order[0]] < best[0]:
                best = (float(cost[order[0]]), X[order[0]].copy(), float(T[order[0]]), c[order[0]], collision[order[0]])
            old = mean
            mean = weights @ X[order[:mu]]
            yw = (mean - old) / sigma
            invsqrt = B @ np.diag(1 / D) @ B.T
            ps = (1 - cs) * ps + np.sqrt(cs * (2 - cs) * mueff) * (invsqrt @ yw)
            hsig = np.linalg.norm(ps) / np.sqrt(1 - (1 - cs) ** (2 * (generation + 1))) / chi < 1.4 + 2 / (dim + 1)
            pc = (1 - cc) * pc + hsig * np.sqrt(cc * (2 - cc) * mueff) * yw
            artmp = (X[order[:mu]] - old) / sigma
            C = ((1 - c1 - cmu) * C + c1 * (np.outer(pc, pc) + (1 - hsig) * cc * (2 - cc) * C)
                 + cmu * (artmp.T * weights) @ artmp)
            sigma *= np.exp((cs / damps) * (np.linalg.norm(ps) / chi - 1))
            generation += 1
            last = time.perf_counter() - began
        from scipy.interpolate import BSpline
        _, x_best, T_best, coefficients, _ = best
        self.info = dict(t=t, T=round(T_best, 3), cost=round(best[0], 3), generations=generation,
                         collision=bool(best[4]), remaining=round(plan.segments[-1].end - t_s, 3))
        candidate = dict(kind="bspline", objective=TIMED, controls=np.asarray(coefficients).tolist(),
                         knots=(self.knots * T_best).tolist(), spline_order=4)
        elapsed = time.perf_counter() - start
        if elapsed > self.budget - 0.002:
            return None
        reason, _ = validate(plan, t_s, candidate, self.limits, t + elapsed)
        if reason != "accepted":
            return None
        self.previous = (BSpline(self.knots * T_best, np.asarray(coefficients), 3), t_s, T_best)
        return dict(start_time_s=t_s, trajectory=candidate)


class CCVPSTO(VPSTO):
    """CC-VPSTO: VP-STO with a sampled chance constraint on collision with the predicted obstacles."""

    def __init__(self, **kwargs):
        kwargs.setdefault("chance", True)
        super().__init__(**kwargs)
