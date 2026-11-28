"""R3 (Refuge, Rent, Reroute): online recovery planning from an in-motion state.

At every observation R3

1. fixes the start time t_s = t + L (the latency budget ``budget_s``) and plans from the state the commanded plan
   reaches then, so every submission splices exactly in position and velocity;
2. predicts each obstacle under three hypotheses: constant velocity preceded by a short constant-acceleration
   phase estimated from consecutive observations (``cv``), decelerate to rest (``stop``) and stay (``hold``);
   moving obstacles get an envelope that grows with look-ahead, resting ones keep a small fixed one;
3. keeps the commanded plan when it reaches the goal and keeps the monitor margin (with a small hysteresis band);
   while a detour or a wait is executing, switches to a direct continuation once that is safe and arrives clearly
   earlier;
4. otherwise builds waiting places reachable from the braking point: braking in place, refuges (static roadmap
   nodes with the largest predicted clearance), escapes (clearance-gradient ascent out of the moving obstacles'
   predicted sweep) and retreats back along the executed path. Each place yields a *stay* (go there and stop) and
   *resumptions* (wait, then continue: along the route, or from a refuge along the roadmap back to the goal);
5. scores candidates by monitor-aware margins: separation minus the external monitor's protective distance (none
   while the robot is at rest, since the monitor never stops a resting robot), under ``cv`` over the horizon,
   under every hypothesis over the commitment window (one period, the budget and a stop), and against resting
   obstacles beyond the horizon, so that a resumption is never judged on a truncated horizon;
6. decides: when an obstacle at rest blocks the way, the rent-or-buy rule waits while the time spent blocked (the
   rent) is below the reroute's extra duration (the price), then reroutes (deterministic, 2-competitive). Reroutes
   come from a static roadmap built at setup (blocked edges from the obstacles' predicted positions, shortcut), and
   from OMPL RRT-Connect while waiting at rest. Otherwise the earliest-arriving candidate that is safe over the
   horizon is chosen, else a safe stay (held, re-deciding at every observation). If nothing keeps the protective
   distance, collision avoidance comes first: candidates are compared by their clearance under a monitor-consistent
   rollout (followed until the monitor would stop them, then braked and held);
7. confirms the chosen candidate's new segments with exact checks (self-collision and fixed equipment always, the
   static scene when the fast sphere model is borderline) and the benchmark's own submission check.

R3 uses only the public problem and observations. ``ablate`` switches off single components for the ablation
study: ``no_refuge``, ``no_escape``, ``no_retreat``, ``no_reroute``, ``no_ompl``, ``no_improve``, ``no_fallback``,
``no_accel``, ``always_wait``, ``reroute_now``, ``timeout`` (fixed 2 s patience), ``randomized`` (randomized
rent threshold), ``cv_only``, ``clearance_only`` and ``no_latency``. Set ``R3_DEBUG`` in the environment to write
one JSON line per step to ``r3_debug.jsonl`` in the worker directory.
"""
from __future__ import annotations

import json
import os
import time

import numpy as np

from .recovery import (GOAL_TOLERANCE_RAD, HYPOTHESES, VOXEL_RADIUS_M, ArmModel, Limits, Obstacles, Pieces,
                       StaticScene, brake, cached_roadmap, evaluate, fixed_equipment, monitor_margins, plan_state,
                       remaining_route, validate)
from .trajectory import Plan


class _Scaled:
    """Limits with scaled velocity and acceleration, for slower candidate variants."""

    def __init__(self, limits, scale):
        self.velocity = limits.velocity * scale
        self.acceleration = limits.acceleration * scale


def moving_conflict(obstacles, rest_speed):
    """Any obstacle still moving (resting ones are handled by rerouting)."""
    return any(obstacles.speed(j) >= rest_speed for j in range(len(obstacles)))


class R3:
    def __init__(self, budget_s=0.15, horizon_s=3.0, sample_s=0.08, waits_s=(0.0, 0.4, 1.0), place_waits_s=(0.0, 0.5),
                 hold_s=0.2, refuges=5, retreats_rad=(0.15, 0.3), speed_scales=(1.0, 0.6), rest_speed_m_s=0.02,
                 rest_time_s=0.3, hysteresis_s=0.1, static_margin_m=0.01, exact_margin_m=0.005, relax_m=0.04,
                 monitor_buffer_m=0.005, exact_tries=3, roadmap_nodes=450, reroute_margins_m=(0.12, 0.08, 0.05, 0.03),
                 ompl_seconds=0.4, improve_s=0.3, reroute_scales=(1.0, 0.6, 0.35), keep_tolerance_m=0.01, seed=0,
                 ablate=()):
        self.budget, self.horizon, self.dt = budget_s, horizon_s, sample_s
        self.waits, self.place_waits = tuple(waits_s), tuple(place_waits_s)
        self.hold, self.refuges = float(hold_s), int(refuges)
        self.retreats, self.scales = tuple(retreats_rad), tuple(speed_scales)
        self.rest_speed, self.rest_time, self.hysteresis = rest_speed_m_s, rest_time_s, hysteresis_s
        self.static_margin, self.exact_margin, self.relax = static_margin_m, exact_margin_m, relax_m
        self.buffer, self.exact_tries, self.roadmap_nodes = monitor_buffer_m, exact_tries, roadmap_nodes
        self.reroute_margins, self.ompl_seconds, self.seed = tuple(reroute_margins_m), ompl_seconds, seed
        self.improve, self.reroute_scales, self.keep_tolerance = improve_s, tuple(reroute_scales), keep_tolerance_m
        self.ablate = set(ablate)

    # ------------------------------------------------------------------ setup
    def setup(self, problem):
        self.arm = ArmModel()
        self.limits = Limits(problem, self.arm.lower, self.arm.upper)
        self.voxels = np.asarray(problem["scene"]["static_voxels_xyz"], float)
        self.scene = StaticScene(self.voxels, fixed_equipment(self.arm.model))
        self.supervisor = problem["protocol"]["supervisor"]
        self.period = float(problem["protocol"]["observation_period_s"])
        self.goal = np.asarray(problem["goal"]["q"], float)
        route = [np.asarray(w, float) for w in problem["nominal_waypoints"]]
        if np.max(np.abs(route[-1] - self.goal)) > 1e-9:
            route.append(self.goal)
        self.path, self.nominal, self.detour = route, list(route), False
        self.roadmap = cached_roadmap(problem, self.arm, self.scene, self.limits,
                                      dict(random_nodes=self.roadmap_nodes), cache_dir="r3_roadmaps")
        self.goal_node = self.roadmap.route_count - 1
        self.roadmap.goal_tree(self.goal_node)
        self.refuge_routes, self.self_ok, self.prev_velocity = {}, {}, {}
        self.next_ompl = -np.inf
        self.hypotheses = ("cv",) if "cv_only" in self.ablate else HYPOTHESES
        self.rest_since, self.blocked_since, self.threshold = {}, None, None
        self.rng = np.random.default_rng(self.seed + int.from_bytes(problem["task_id"].encode()[-4:], "little"))
        self.tau = np.arange(0.0, self.horizon + 1e-9, self.dt)
        self.window = self.period + self.budget + float(np.max(self.limits.velocity / self.limits.acceleration))
        self.segment_cache, self.waiting, self.ompl_calls, self.reroute_cache, self.edge_cache = {}, None, 0, {}, {}
        self.ompl_found = {}
        self.debug = os.environ.get("R3_DEBUG") is not None
        return dict(roadmap_nodes=int(len(self.roadmap.nodes)), roadmap_edges=int(len(self.roadmap.edges)))

    # ------------------------------------------------------------------ helpers
    def _rest_update(self, obstacles, t):
        resting = []
        for j, item in enumerate(obstacles.items):
            if obstacles.speed(j) < self.rest_speed:
                self.rest_since.setdefault(item["id"], t)
                if t - self.rest_since[item["id"]] >= self.rest_time:
                    resting.append(j)
            else:
                self.rest_since.pop(item["id"], None)
        return resting

    def _segment_static(self, a, b):
        """Fast static clearance of the straight segment a -> b (cached)."""
        key = (tuple(np.round(a, 6)), tuple(np.round(b, 6)))
        if key not in self.segment_cache:
            n = max(2, int(np.ceil(np.max(np.abs(np.subtract(b, a))) / 0.04)) + 1)
            samples = np.linspace(a, b, n)
            d = self.scene.distance(self.arm.spheres(samples), self.arm.sphere_radius).min()
            self.segment_cache[key] = (float(d), samples)
        return self.segment_cache[key]

    def _exact_ok(self, segments):
        for a, b in segments:
            fast, samples = self._segment_static(a, b)
            key = (tuple(np.round(a, 6)), tuple(np.round(b, 6)))
            if key not in self.self_ok:
                self.self_ok[key] = bool(self.arm.self_and_fixed_clearance(samples[1:]).min() > self.exact_margin)
            if not self.self_ok[key]:
                return False
            if fast >= self.static_margin:
                continue
            sparse = samples[:: max(1, len(samples) // 6)]
            if self.arm.exact_clearance(sparse, self.voxels, fixed=True, cutoff=0.1).min() < self.exact_margin:
                return False
        return True

    def _back_along_path(self, q, distance):
        """The point ``distance`` rad behind ``q`` along the current path polyline (stops at its start)."""
        path = self.path
        best, index, s_best = np.inf, 0, 0.0
        for i, (a, b) in enumerate(zip(path[:-1], path[1:])):
            d = b - a
            s = np.clip((q - a) @ d / max(d @ d, 1e-12), 0.0, 1.0)
            e = np.linalg.norm(a + s * d - q)
            if e < best:
                best, index, s_best = e, i, s
        point = path[index] + s_best * (path[index + 1] - path[index])
        left = distance
        while left > 1e-9:
            seg = point - path[index]
            length = np.linalg.norm(seg)
            if length >= left:
                return point - seg / max(length, 1e-12) * left
            left -= length
            point = path[index]
            if index == 0:
                return point
            index -= 1
        return point

    # ------------------------------------------------------------------ candidates
    def _places(self, qs, vs, obstacles):
        """Waiting places reachable from the splice state: (family, place, pieces to reach it, new segments)."""
        stop = brake(qs, vs, self.limits.acceleration)
        q_b = np.asarray(stop[0][-1, -1], float) if len(stop[1]) else np.asarray(qs, float)
        places = [("wait", q_b, Pieces(qs).add(stop), [], None)]
        if "no_refuge" not in self.ablate and len(obstacles):
            near = self.roadmap.nearest(q_b, 80)
            nodes = self.roadmap.nodes[near]
            centres = self.arm.spheres(nodes)
            clear = np.full(len(nodes), np.inf)
            for h in ("cv", "hold"):
                for j in range(len(obstacles)):
                    predicted, _ = obstacles.predict(j, self.tau[::3], h)
                    d = obstacles.surface_distance(j, centres[:, None], predicted)
                    clear = np.minimum(clear, d.min(axis=(1, 2)))
            score = clear - 0.05 * np.linalg.norm(nodes - q_b[None], axis=1)
            for k in np.argsort(-score)[: self.refuges]:
                r = nodes[k]
                if np.max(np.abs(r - q_b)) > 1e-6:
                    places.append(("refuge", r, Pieces(qs).add(stop).to(r, self.limits), [(q_b, r)], int(near[k])))
        if "no_escape" not in self.ablate:
            places += self._escapes(qs, stop, q_b, obstacles)
        if "no_retreat" not in self.ablate and remaining_route(self.path, q_b):
            for back in self.retreats:
                point = self._back_along_path(q_b, back)
                if np.linalg.norm(point - q_b) > 0.05:
                    places.append(("retreat", point, Pieces(qs).add(stop).to(point, self.limits), [(q_b, point)], None))
        return places, q_b

    def _refuge_route(self, node):
        """Static route from a roadmap node back to the goal (shortcut once, cached): (points ahead, new segments)."""
        if node not in self.refuge_routes:
            nodes = self.roadmap.path_to_goal(node)
            if nodes is None or len(nodes) < 2:
                self.refuge_routes[node] = None
            else:
                points, new = self._shortcut([self.roadmap.nodes[n] for n in nodes], None, 0.0, window=8)
                self.refuge_routes[node] = (points[1:], new[1:])
        return self.refuge_routes[node]

    def _sweep_clearance(self, Q, sweeps, obstacles):
        """Clearance (N,) of configurations Q (N,7) to the predicted sweeps of the moving obstacles."""
        centres = self.arm.spheres(Q)
        best = np.full(len(Q), np.inf)
        for j, points in sweeps:
            d = obstacles.surface_distance(j, centres[:, None], points) - self.arm.sphere_radius - VOXEL_RADIUS_M
            best = np.minimum(best, d.min(axis=(1, 2)))
        return best

    def _escapes(self, qs, stop, q_b, obstacles, steps=(0.25, 0.25, 0.3), h=0.02):
        """Places that move the arm out of the moving obstacles' predicted sweep (clearance-gradient ascent)."""
        sweeps = [(j, obstacles.predict(j, self.tau[::2], "cv")[0]) for j in range(len(obstacles))
                  if obstacles.speed(j) >= self.rest_speed]
        if not sweeps:
            return []
        out, q = [], np.asarray(q_b, float)
        for step in steps:
            probe = np.vstack([q] + [q + h * e for e in np.eye(7)])
            d = self._sweep_clearance(probe, sweeps, obstacles)
            if d[0] > 0.3:
                break
            grad = (d[1:] - d[0]) / h
            norm = float(np.linalg.norm(grad))
            if norm < 1e-6:
                break
            nxt = self.limits.clip(q + step * grad / norm)
            if self._segment_static(q_b, nxt)[0] < self.static_margin:
                break
            q = nxt
            out.append(("escape", q.copy(), Pieces(qs).add(stop).to(q, self.limits), [(q_b, q.copy())], None))
        return out

    def _monitored_clearance(self, Q, V, pieces, obstacles, centres=None, monitor=None):
        """Clearance (N,) of trajectories sampled at ``tau`` as the external monitor would let them run: each one is
        followed until its first protective-distance violation, then (after the reaction time) braked to rest and
        held there. ``pieces`` gives each trajectory's (controls, durations), or None for a Plan given as samples.
        ``centres`` and ``monitor`` (cv protective margins without buffer) may be passed in when already known."""
        if centres is None:
            centres = self.arm.spheres(Q.reshape(-1, 7)).reshape(Q.shape[:-1] + (-1, 3))
        if monitor is None:
            monitor = monitor_margins(self.arm, obstacles, Q, V, self.tau, self.supervisor, self.limits.acceleration,
                                      ("cv",), 0.0, centres=centres, protective=True)[0]
        clear = monitor_margins(self.arm, obstacles, Q, V, self.tau, self.supervisor, self.limits.acceleration,
                                self.hypotheses, 0.0, centres=centres, protective=False).min(axis=0)
        moving = np.max(np.abs(V), axis=-1) > self.supervisor.get("moving_threshold_rad_s", 0.01)
        violated = (monitor < 0.0) & moving
        first = np.where(violated.any(axis=1), violated.argmax(axis=1), len(self.tau))
        before = np.where(np.arange(len(self.tau))[None] <= first[:, None], clear, np.inf).min(axis=1)
        rows = np.flatnonzero(first < len(self.tau))
        if not len(rows):
            return clear.min(axis=1)
        reaction = float(self.supervisor["reaction_time_s"])
        stops = []
        for i in rows:
            t_b = self.tau[first[i]] + reaction
            if pieces[i] is not None:
                q_b, v_b = evaluate(*pieces[i], np.array([t_b]))
                q_b, v_b = q_b[0], v_b[0]
            else:
                j = min(int(np.searchsorted(self.tau, t_b)), len(self.tau) - 1)
                q_b, v_b = Q[i, j], V[i, j]
            stops.append(q_b + 0.5 * float(np.max(np.abs(v_b) / self.limits.acceleration)) * v_b)
        held = self.arm.spheres(np.asarray(stops))[:, None]  # (M, 1, S, 3), held at rest for the horizon
        c = obstacles.clearance(held, self.arm.sphere_radius, self.tau, self.hypotheses).min(axis=(0, 3))  # (M, T)
        c = np.where(np.arange(len(self.tau))[None] >= first[rows][:, None], c, np.inf).min(axis=1)
        out = clear.min(axis=1)
        out[rows] = np.minimum(before[rows], c)
        return out

    def _collision_margins(self, candidates, obstacles):
        """Monitor-consistent clearance ``c_full`` of candidates over the horizon (see ``_monitored_clearance``)."""
        todo = [c for c in candidates if "c_full" not in c]
        if not todo:
            return
        pieces = [c["pieces"].arrays() for c in todo]
        if all("_samples" in c for c in todo) and "clearance_only" not in self.ablate:
            # Reuse the samples, sphere centres and cv monitor margins computed while scoring.
            Q = np.stack([c["_samples"][0] for c in todo])
            V = np.stack([c["_samples"][1] for c in todo])
            centres = np.stack([c["_samples"][2] for c in todo])
            monitor = np.stack([c["_samples"][3] for c in todo]) + self.buffer
            values = self._monitored_clearance(Q, V, pieces, obstacles, centres, monitor)
        else:
            Q = np.empty((len(todo), len(self.tau), 7))
            V = np.empty_like(Q)
            for i, (controls, durations) in enumerate(pieces):
                Q[i], V[i] = evaluate(controls, durations, self.tau)
            values = self._monitored_clearance(Q, V, pieces, obstacles)
        for c, value in zip(todo, values):
            c["c_full"] = float(value)

    def _plan_collision_margin(self, plan, t_s, obstacles):
        Q, V = plan_state(plan, t_s + self.tau)
        return float(self._monitored_clearance(Q[None], V[None], [None], obstacles)[0])

    def _candidates(self, places):
        """Stay candidates (go there and stop) and resume candidates (each place, waits, speed scales)."""
        stays, resumes = [], []
        for family, place, base, new, node in places:
            back = self._refuge_route(node) if node is not None else None
            if back is not None:
                ahead, ahead_new = back
            else:
                ahead = remaining_route(self.path, place) or [self.goal]
                ahead_new = [(place, ahead[0])]
            reach = base.duration
            stays.append(dict(family=family, place=place, base=base, pieces=base.copy().wait(self.hold),
                              resume=np.inf, wait=self.hold, new=list(new), route=[place] + ahead, reach=reach,
                              kind="stay", scale=1.0))
            for scale in self.scales:
                if family != "wait" and scale != 1.0:
                    continue
                lim = self.limits if scale == 1.0 else _Scaled(self.limits, scale)
                for w in (self.waits if family == "wait" else self.place_waits):
                    pieces = base.copy().wait(w)
                    resume = pieces.duration
                    pieces.route(ahead, lim)
                    resumes.append(dict(family=family, place=place, pieces=pieces, resume=resume, wait=w, scale=scale,
                                        new=list(new) + list(ahead_new), route=[place] + ahead, reach=reach,
                                        kind="resume"))
        return stays, resumes

    def _reroutes(self, qs, vs, q_b, obstacles, at_rest):
        """Scored reroutes from the braking point around the obstacles: shortcut roadmap paths (cached per
        blocker pose), and, while waiting at rest with no safe reroute yet, OMPL plans with varied margins."""
        if "no_reroute" in self.ablate or not len(obstacles):
            return []
        stop = brake(qs, vs, self.limits.acceleration)
        positions = {}
        for j in range(len(obstacles)):
            now = obstacles.predict(j, np.array([0.0]), "hold")[0]
            positions[j] = now if obstacles.speed(j) < self.rest_speed else \
                np.vstack([now, obstacles.predict(j, np.array([0.5, 1.0]), "cv")[0]])
        key = (tuple(np.round(q_b, 3)),
               tuple(tuple(np.round(np.asarray(v).reshape(-1), 2)) for v in positions.values()))
        if key not in self.reroute_cache:
            paths = []
            clearance = np.full(len(self.roadmap.edges), np.inf)
            for j, pos in positions.items():
                # Edge clearances depend only on the obstacle's positions; a resting obstacle hits the cache.
                cache_key = (obstacles.items[j]["id"], tuple(np.round(np.asarray(pos, float).reshape(-1), 2)))
                edge = self.edge_cache.get(cache_key)
                if edge is None:
                    edge = self.roadmap.edge_clearance(self.arm, obstacles, {j: pos}, max(self.reroute_margins))
                    if len(self.edge_cache) > 64:
                        self.edge_cache.clear()
                    self.edge_cache[cache_key] = edge
                clearance = np.minimum(clearance, edge)
            for margin in self.reroute_margins:
                blocked = clearance < margin
                seen = set()
                for start in self.roadmap.nearest(q_b, 6):
                    nodes = self.roadmap.search(int(start), self.goal_node, blocked)
                    if nodes is None or tuple(nodes) in seen:
                        continue
                    seen.add(tuple(nodes))
                    points, new = self._shortcut([q_b] + [self.roadmap.nodes[n] for n in nodes], obstacles, margin)
                    paths.append(("reroute", points[1:], new))
                    if len(seen) >= 2:
                        break
                if paths:
                    break
            if len(self.reroute_cache) > 32:
                self.reroute_cache.clear()
            self.reroute_cache[key] = paths
        # Paths are cached; trajectories are rebuilt from the current splice state so that they splice exactly.
        out = []
        for family, waypoints, new in self.reroute_cache[key] + self.ompl_found.get(key, []):
            out += self._route_candidates(qs, stop, q_b, waypoints, family, new)
        self._score(out, obstacles)
        safe = any(c["static_ok"] and c["m_full"] >= 0.0 for c in out)
        if (not safe and at_rest and self.ompl_calls < 8 and "no_ompl" not in self.ablate
                and self._t_now >= self.next_ompl):
            # While waiting at rest, a sampling-based plan (simplified by OMPL) usually beats the roadmap detour;
            # attempts vary the moving-obstacle margin (a wider berth keeps the monitor's protective distance).
            from .world import ompl_path
            margin = (0.05, 0.1, 0.15, 0.03)[self.ompl_calls % 4]
            seconds = self.ompl_seconds * (1 + self.ompl_calls // 2)
            self.ompl_calls += 1
            self.next_ompl = self._t_now + 0.5
            moving = np.vstack([np.asarray(o["voxels_xyz"], float) for o in self._observed])
            path = ompl_path(self.arm.checker, q_b, self.goal, self.voxels, moving, 0.005, margin,
                             seconds, self.seed * 1000 + self.ompl_calls, f"ompl_{self.ompl_calls:03d}")
            if path is not None and len(path) > 1:
                points = [q_b] + [np.asarray(w, float) for w in path[1:]]
                entry = ("reroute_ompl", points[1:], list(zip(points[:-1], points[1:])))
                self.ompl_found.setdefault(key, []).append(entry)
                found = self._route_candidates(qs, stop, q_b, entry[1], entry[0], entry[2])
                self._score(found, obstacles)
                out += found
        return out

    def _segment_clear(self, a, b, obstacles, margin):
        """Straight segment a -> b clears the static scene (fast model) and, unless ``obstacles`` is None, every
        obstacle at its current position by ``margin``."""
        fast, samples = self._segment_static(a, b)
        if fast < self.static_margin:
            return False
        if obstacles is None or not len(obstacles):
            return True
        centres = self.arm.spheres(samples)
        for j in range(len(obstacles)):
            d = obstacles.surface_distance(j, centres[None], obstacles.items[j]["p"][None])[0]
            if (d - VOXEL_RADIUS_M - self.arm.sphere_radius).min() < margin:
                return False
        return True

    def _shortcut(self, points, obstacles, margin, window=None):
        """Greedy shortcut of a roadmap path; returns the points and its new (non-roadmap) segments (the first
        segment, from the start to the first node, is always new)."""
        out, new, i = [points[0]], [(points[0], points[1])] if len(points) > 1 else [], 0
        while i < len(points) - 1:
            j = len(points) - 1 if window is None else min(len(points) - 1, i + window)
            while j > i + 1 and not self._segment_clear(points[i], points[j], obstacles, margin):
                j -= 1
            if j > i + 1:
                new.append((points[i], points[j]))
            out.append(points[j])
            i = j
        return out, new

    def _route_candidates(self, qs, stop, q_b, waypoints, family, new):
        out = []
        for scale in self.reroute_scales:
            lim = self.limits if scale == 1.0 else _Scaled(self.limits, scale)
            pieces = Pieces(qs).add(stop).route(waypoints, lim)
            out.append(dict(family=family, place=q_b, pieces=pieces, resume=np.inf, wait=0.0, scale=scale,
                            new=list(new), route=[q_b] + list(waypoints), reach=0.0, kind="reroute"))
        return out

    def _improvement(self, qs, vs, plan, t_s, obstacles):
        """While a safe detour or wait is executing, a direct continuation that arrives clearly earlier."""
        remaining = plan.segments[-1].end - t_s
        stop = brake(qs, vs, self.limits.acceleration)
        q_b = np.asarray(stop[0][-1, -1], float) if len(stop[1]) else np.asarray(qs, float)
        candidates = []
        for route in (self.nominal, self.path):
            ahead = remaining_route(route, q_b)
            if not ahead:
                continue
            pieces = Pieces(qs).add(stop).route(ahead, self.limits)
            if pieces.duration < remaining - self.improve:
                candidates.append(dict(family="direct", place=q_b, pieces=pieces, resume=0.0, wait=0.0, scale=1.0,
                                       new=[(q_b, ahead[0])], route=[q_b] + ahead, reach=0.0, kind="resume"))
        if not candidates:
            return None
        self._score(candidates, obstacles)
        ok = [c for c in candidates if c["static_ok"] and c["m_full"] >= 0.0]
        return [min(ok, key=lambda c: c["duration"])] if ok else None

    # ------------------------------------------------------------------ scoring
    def _margins(self, Q, V, obstacles):
        """Per-sample monitor margins: cv over the horizon (N, T); every hypothesis over the window (N, W)."""
        centres = self.arm.spheres(Q.reshape(-1, 7)).reshape(Q.shape[:-1] + (-1, 3))
        protective = "clearance_only" not in self.ablate
        cv = monitor_margins(self.arm, obstacles, Q, V, self.tau, self.supervisor, self.limits.acceleration, ("cv",),
                             self.buffer, centres=centres, protective=protective)[0]
        w = int(np.searchsorted(self.tau, self.window, side="right"))
        others = tuple(h for h in self.hypotheses if h != "cv")
        window = cv[..., :w]
        if others:
            extra = monitor_margins(self.arm, obstacles, Q[..., :w, :], V[..., :w, :], self.tau[:w], self.supervisor,
                                    self.limits.acceleration, others, self.buffer, centres=centres[..., :w, :, :],
                                    protective=protective).min(axis=0)
            window = np.minimum(window, extra)
        self._last_centres = centres
        return cv, window

    def _score(self, candidates, obstacles):
        if not candidates:
            return
        Q = np.empty((len(candidates), len(self.tau), 7))
        V = np.empty_like(Q)
        for i, c in enumerate(candidates):
            controls, durations = c["pieces"].arrays()
            Q[i], V[i] = evaluate(controls, durations, self.tau)
            c["duration"] = float(np.sum(durations))
        cv, window = self._margins(Q, V, obstacles)
        centres = self._last_centres
        tail = self._tail_margins([c["pieces"].arrays() for c in candidates], None)
        for i, c in enumerate(candidates):
            c["_samples"] = (Q[i], V[i], centres[i], cv[i])
            c["m_window"] = float(window[i].min())
            c["m_full"] = min(float(cv[i].min()), c["m_window"], float(tail[i]))
            fast = min((self._segment_static(a, b)[0] for a, b in c["new"]), default=np.inf)
            c["static"] = fast
            c["static_ok"] = fast >= -self.relax

    def _tail_margins(self, pieces, plan, t_s=0.0):
        """Margins beyond the horizon against obstacles at rest (held in place), so that a resumption that
        reaches a resting blocker after the horizon is not mistaken for a safe one. ``pieces`` is a list of
        (controls, durations); ``plan`` (with start ``t_s``) is used instead when given."""
        rest = self._resting_obstacles
        count = 1 if plan is not None else len(pieces)
        if rest is None:
            return np.full(count, np.inf)
        if plan is not None:
            end = plan.segments[-1].end - t_s
        else:
            end = max(float(np.sum(d)) for _, d in pieces)
        tau = np.arange(self.horizon + self.dt, min(end, self.horizon + 15.0) + self.dt, self.dt)
        if not len(tau):
            return np.full(count, np.inf)
        if plan is not None:
            Q, V = plan_state(plan, t_s + tau)
            Q, V = Q[None], V[None]
        else:
            Q = np.empty((count, len(tau), 7))
            V = np.empty_like(Q)
            for i, (controls, durations) in enumerate(pieces):
                Q[i], V[i] = evaluate(controls, durations, tau)
        margins = monitor_margins(self.arm, rest, Q, V, tau, self.supervisor, self.limits.acceleration, ("hold",),
                                  self.buffer, protective="clearance_only" not in self.ablate)[0]
        return margins.min(axis=-1)

    def _plan_margins(self, plan, t_s, obstacles):
        """Worst margin of the current plan over the horizon (and beyond it, against resting obstacles), and
        over its commitment window."""
        Q, V = plan_state(plan, t_s + self.tau)
        cv, window = self._margins(Q[None], V[None], obstacles)
        tail = float(self._tail_margins(None, plan, t_s)[0])
        return min(float(cv.min()), float(window.min()), tail), float(window.min())

    # ------------------------------------------------------------------ decision
    def step(self, observation):
        self._trace = dict(t=observation["time_s"])
        self._t0 = time.perf_counter()
        out = self._step(observation)
        if self.debug:
            self._trace.update(submitted=out is not None, ms=round(1000 * (time.perf_counter() - self._t0), 1))
            with open("r3_debug.jsonl", "a") as f:
                f.write(json.dumps(self._trace, default=float) + "\n")
        return out

    def _step(self, observation):
        start = self._t0
        t = observation["time_s"]
        self._t_now = t
        if observation["supervisor"]["protective_stop"]:
            self.waiting = None
            return None
        plan = Plan.from_list(observation["plan"])
        budget = 0.0 if "no_latency" in self.ablate else self.budget
        t_s = t + budget
        qs, vs, _ = plan.at(t_s)
        q_now = np.asarray(observation["robot"]["q"], float)
        if np.max(np.abs(q_now - self.goal)) <= GOAL_TOLERANCE_RAD and np.max(np.abs(vs)) < 1e-3:
            return None
        accelerations = {}
        for o in observation["obstacles"]:
            v = np.asarray(o["velocity_m_s"], float)
            previous = self.prev_velocity.get(o["id"])
            if previous is not None and 0.05 <= t - previous[0] <= 0.3:
                accelerations[o["id"]] = (v - previous[1]) / (t - previous[0])
            self.prev_velocity[o["id"]] = (t, v)
        if "no_accel" in self.ablate:
            accelerations = None
        obstacles = Obstacles(observation["obstacles"], accelerations=accelerations)
        self._observed = observation["obstacles"]
        resting = self._rest_update(obstacles, t)
        self._resting_obstacles = obstacles.subset(resting) if resting else None
        to_goal = np.max(np.abs(plan.final_configuration() - self.goal)) <= 1e-6
        keep, keep_commit = self._plan_margins(plan, t_s, obstacles)
        self._trace.update(keep=round(keep, 3), keep_commit=round(keep_commit, 3), resting=len(resting))
        at_rest = np.max(np.abs(vs)) < 1e-6 and np.max(np.abs(observation["robot"]["v"])) < 0.02
        if to_goal and keep >= -self.keep_tolerance and keep_commit >= -self.keep_tolerance:
            # Hysteresis: a plan is adopted at margin >= 0 and kept down to -keep_tolerance, so that tiny
            # re-evaluation differences do not replace it with an equivalent plan at every observation.
            self.blocked_since, self.threshold = None, None
            if self.detour and "no_improve" not in self.ablate:
                better = self._improvement(qs, vs, plan, t_s, obstacles)
                if better:
                    return self._submit(better, plan, t, t_s, start, budget, at_rest)
            return None
        places, q_b = self._places(qs, vs, obstacles)
        stays, resumes = self._candidates(places)
        self._score(stays + resumes, obstacles)
        direct = next(c for c in resumes if c["family"] == "wait" and c["wait"] == 0.0 and c["scale"] == 1.0)
        blocked_rest = bool(resting) and direct["m_full"] < 0.0
        full = [c for c in resumes if c["static_ok"] and c["m_full"] >= 0.0]
        safe_stays = sorted((c for c in stays if c["static_ok"] and c["m_full"] >= 0.0),
                            key=lambda c: (c["family"] != "wait", c["reach"], -c["m_full"]))
        reroutes = []
        stuck = at_rest and self.waiting is not None and not full and not moving_conflict(obstacles, self.rest_speed)
        if (blocked_rest and (at_rest or keep_commit >= 0.0)) or stuck:
            # Rerouting is for obstacles at rest; it may take longer than the budget only while the committed
            # part of the current plan stays safe (or the robot waits at rest).
            reroutes = self._reroutes(qs, vs, q_b, obstacles, at_rest)
            if self.debug:
                self._trace["reroute_info"] = [(c["family"], c["scale"], round(c["m_full"], 3), round(c["m_window"], 3),
                                                round(c["static"], 3), round(c["duration"], 2)) for c in reroutes]
            full += [c for c in reroutes if c["static_ok"] and c["m_full"] >= 0.0]
        holding = self.waiting is not None and not to_goal and keep >= -self.keep_tolerance
        choice = None
        if blocked_rest:
            self.blocked_since = t if self.blocked_since is None else self.blocked_since
            rent = t - self.blocked_since
            buys = sorted((c for c in full if c["kind"] == "reroute"), key=lambda c: c["duration"])
            price = buys[0]["duration"] - direct["duration"] if buys else np.inf
            if self.threshold is None:
                # Randomized rent-or-buy: buy at z * price, z drawn from density e^z / (e - 1) on [0, 1].
                self.threshold = float(np.log1p(self.rng.uniform() * (np.e - 1.0))) if "randomized" in self.ablate else 1.0
            limit = self.threshold * price
            if "always_wait" in self.ablate:
                limit = np.inf
            elif "reroute_now" in self.ablate:
                limit = 0.0
            elif "timeout" in self.ablate:
                limit = 2.0 if buys else np.inf
            self._trace.update(rent=round(rent, 2), price=round(price, 2) if np.isfinite(price) else None)
            resumes_ok = [c for c in full if c["kind"] == "resume"]
            if resumes_ok:
                choice = sorted(resumes_ok, key=lambda c: (c["duration"], -c["m_full"]))
            elif buys and rent >= limit:
                choice = buys
            elif not holding and safe_stays:
                choice = safe_stays
        else:
            self.blocked_since, self.threshold = None, None
            if full:
                choice = sorted(full, key=lambda c: (c["duration"], -c["m_full"]))
                if to_goal and keep >= 0.0 and plan.segments[-1].end <= t_s + choice[0]["duration"] + self.hysteresis:
                    choice = None
            elif not holding and safe_stays:
                choice = safe_stays
        self._trace.update(full=len(full), stays=len(safe_stays), blocked_rest=bool(blocked_rest),
                           reroutes=len(reroutes), holding=bool(holding))
        if choice is None and not full and not safe_stays and not holding and "no_fallback" not in self.ablate:
            # Nothing keeps the monitor's protective distance over the horizon. Avoiding a collision comes first:
            # prefer candidates that keep clear of every predicted obstacle (a protective stop may still occur),
            # stays before resumptions, then the larger monitor margin.
            pool = [c for c in stays + reroutes if c["static_ok"]]
            pool += sorted((c for c in resumes if c["static_ok"]), key=lambda c: -c["m_window"])[:12]
            pool = pool or stays
            self._collision_margins(pool, obstacles)
            keep_clear = self._plan_collision_margin(plan, t_s, obstacles)
            rank = lambda c: (c["c_full"] >= 0.0, c["m_window"] >= 0.0, c["kind"] == "stay", c["m_full"], c["c_full"])
            best = max(pool, key=rank)
            self._trace.update(keep_clear=round(keep_clear, 3), best_clear=round(best["c_full"], 3))
            current = (keep_clear >= 0.0, keep_commit >= 0.0)
            if (best["c_full"] >= 0.0, best["m_window"] >= 0.0) > current or (
                    keep_clear < 0.0 and best["c_full"] > keep_clear + 0.01):
                choice = [best]
        if not choice:
            return None
        return self._submit(choice, plan, t, t_s, start, budget, at_rest)

    def _submit(self, choice, plan, t, t_s, start, budget, at_rest):
        rejected = []
        for tried, c in enumerate(choice):
            if tried >= self.exact_tries:
                break
            if time.perf_counter() - start > budget - 0.015 and not at_rest and "no_latency" not in self.ablate:
                rejected.append("late")
                break
            if not self._exact_ok(c["new"]):
                rejected.append(("exact", c["family"]))
                continue
            pieces = c["pieces"]  # a stay ends at rest at its place; resumption is a later, separate decision
            candidate = pieces.candidate()
            elapsed = time.perf_counter() - start
            start_time = t_s
            if elapsed > budget - 0.005:
                if not at_rest:
                    return None
                # Waiting at rest: the commanded state is constant, so a later start still splices exactly.
                start_time = t + elapsed + 0.03
            reason, _ = validate(plan, start_time, candidate, self.limits, t + elapsed)
            if reason != "accepted" and "no_latency" not in self.ablate:
                rejected.append((reason, c["family"]))
                continue
            self.path = [np.asarray(w, float) for w in c["route"]]
            self.waiting = dict(place=c["place"]) if c["kind"] == "stay" else None
            self.detour = not (c["family"] in ("wait", "direct") and c["wait"] == 0.0 and c["scale"] == 1.0)
            self._trace.update(choice=c["family"], kind=c["kind"], wait=c["wait"], m_full=round(c["m_full"], 3),
                               duration=round(c["duration"], 2))
            return dict(start_time_s=start_time, trajectory=candidate)
        self._trace["rejected"] = rejected
        return None


ABLATIONS = ("no_refuge", "no_escape", "no_retreat", "no_reroute", "no_ompl", "no_improve", "no_fallback", "no_accel",
             "always_wait", "reroute_now", "timeout", "randomized", "cv_only", "clearance_only", "no_latency")


def _ablation(name):
    def __init__(self, **kwargs):
        R3.__init__(self, ablate=(name,), **kwargs)
    return type(f"R3_{name}", (R3,), {"__init__": __init__, "__doc__": f"R3 with ``{name}`` (ablation)."})


for _name in ABLATIONS:
    globals()[f"R3_{_name}"] = _ablation(_name)
