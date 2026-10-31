"""Native EI-ZO inflation, counterexample recovery, and independent validation."""

from __future__ import annotations
import time
import numpy as np
import csdecomp as csd
from pydrake.geometry.optimization import HPolyhedron
from pydrake.math import BsplineBasis
from pydrake.trajectories import BsplineTrajectory
from .optimization import lscs, lgcs, scsto, bezier_values
from .common import atomic


def native_region(region):
    return csd.HPolyhedron(
        np.asarray(region.A(), np.float32), np.asarray(region.b(), np.float32)
    )


class Pipeline:
    def __init__(self, forest, seed):
        self.forest = forest
        self.cfg = forest.cfg
        options = csd.EizoOptions()
        for key, value in self.cfg["eizo"].items():
            setattr(options, key, value)
        options.seed = seed
        options.verbose = False
        options.track_iteration_information = False
        self.inflator = csd.CudaEdgeInflator(
            forest.minimal, forest.geometry_ids, options, forest.domain
        )
        self.inflation_calls = 0
        self.roundoff = []

    def inflate(self, u, v):
        result = self.inflator.inflateEdge(
            np.asarray(u, np.float32),
            np.asarray(v, np.float32),
            self.forest.empty_voxels,
            1e-6,
            False,
        )
        a = np.asarray(result.A(), float)
        b = np.asarray(result.b(), float)
        correction = np.maximum(np.maximum(a @ u - b, a @ v - b), 0)
        if correction.max() > 1e-5:
            raise ValueError(
                "EI-ZO lost a seed endpoint beyond floating-point tolerance"
            )
        self.roundoff.append(float(correction.max()))
        self.inflation_calls += 1
        return HPolyhedron(a, b + correction + 1e-8)

    def corridor(self, path, pool=None):
        sequence = []
        all_regions = list(pool or [])
        all_seeds = []
        for u, v in zip(path[:-1], path[1:]):
            found = next(
                (
                    i
                    for i, r in enumerate(all_regions)
                    if r.PointInSet(u, 1e-7) and r.PointInSet(v, 1e-7)
                ),
                None,
            )
            if found is None:
                found = len(all_regions)
                all_regions.append(self.inflate(u, v))
                all_seeds.append((u, v))
            if not sequence or sequence[-1] != found:
                sequence.append(found)
        return [all_regions[i] for i in sequence], sequence, all_regions, all_seeds

    def initial(self, path):
        pool = []
        pool_seeds = []
        sequence = []
        for u, v in zip(path[:-1], path[1:]):
            found = next(
                (
                    i
                    for i, r in enumerate(pool)
                    if r.PointInSet(u, 1e-7) and r.PointInSet(v, 1e-7)
                ),
                None,
            )
            if found is None:
                found = len(pool)
                pool.append(self.inflate(u, v))
                pool_seeds.append((np.array(u), np.array(v)))
            if not sequence or sequence[-1] != found:
                sequence.append(found)
        return [pool[i] for i in sequence], [pool_seeds[i] for i in sequence]

    def solve(self, method, path, drm, out):
        started = time.perf_counter()
        construction = 0.0
        optimization = 0.0
        validation_time = 0.0
        repairs = 0
        history = []
        tic = time.perf_counter()
        regions, seeds = self.initial(path)
        second_path = None
        if method == "LGCS":
            eligible = [
                i
                for i, r in enumerate(regions)
                if not r.PointInSet(path[0], 1e-6) and not r.PointInSet(path[-1], 1e-6)
            ]
            if eligible:
                index = np.random.default_rng(0).choice(eligible)
                second_path = self.forest.alternate_path(drm, regions[index])
                if second_path is not None:
                    for u, v in zip(second_path[:-1], second_path[1:]):
                        if not any(
                            r.PointInSet(u, 1e-7) and r.PointInSet(v, 1e-7)
                            for r in regions
                        ):
                            regions.append(self.inflate(u, v))
                            seeds.append((u, v))
        construction += time.perf_counter() - tic
        for recovery in range(self.cfg["maximum_recovery_iterations"] + 1):
            tic = time.perf_counter()
            if method == "LSCS":
                candidate = lscs(regions, path[0], path[-1])
            elif method == "LGCS":
                candidate = lgcs(regions, path[0], path[-1])
            elif method == "SCSTO":
                candidate = scsto(regions, path[0], path[-1], self.cfg)
            else:
                raise ValueError(method)
            optimization += time.perf_counter() - tic
            candidate["method"] = method
            save_candidate(out, candidate, regions, recovery)
            if not candidate["solver_success"]:
                return dict(
                    status="optimization_failed",
                    candidate=candidate,
                    regions=regions,
                    recovery_iterations=repairs,
                    construction_seconds=construction,
                    optimization_seconds=optimization,
                    total_seconds=construction + optimization,
                    wall_seconds=time.perf_counter() - started,
                    history=history,
                )
            tic = time.perf_counter()
            validation = validate(candidate, self.forest, regions)
            validation_time += time.perf_counter() - tic
            collision_count = len(validation["collision_points"])
            history.append(
                dict(
                    iteration=recovery,
                    collisions=collision_count,
                    passed=validation["passed"],
                )
            )
            atomic(out / f"validation_{recovery:02d}.json", validation)
            if (
                validation["passed"]
                or not collision_count
                or recovery == self.cfg["maximum_recovery_iterations"]
            ):
                break
            tic = time.perf_counter()
            options = csd.EditRegionsOptions()
            options.configuration_margin = self.cfg["eizo"]["configuration_margin"]
            options.bisection_steps = 9
            options.verbose = False
            points = validation["collision_points"]
            indices = validation["collision_regions"]
            selected = np.arange(len(points), dtype=int)
            fixed, _ = csd.EditRegionsCuda(
                np.asarray(points[selected].T, np.float32),
                np.asarray(indices[selected], dtype=np.uint32).tolist(),
                np.asarray([u for u, v in seeds], np.float32).T,
                np.asarray([v for u, v in seeds], np.float32).T,
                [native_region(r) for r in regions],
                self.forest.minimal,
                self.forest.geometry_ids,
                self.forest.empty_voxels,
                1e-6,
                options,
            )
            regions = [
                HPolyhedron(np.asarray(r.A(), float), np.asarray(r.b(), float))
                for r in fixed
            ]
            # Restore the full original path coverage as required by paper VI-B.
            for u, v in zip(path[:-1], path[1:]):
                if not any(
                    r.PointInSet(u, 1e-6) and r.PointInSet(v, 1e-6) for r in regions
                ):
                    regions.append(self.inflate(u, v))
                    seeds.append((u, v))
            if method != "LGCS":
                order = []
                for u, v in zip(path[:-1], path[1:]):
                    found = next(
                        i
                        for i, r in enumerate(regions)
                        if r.PointInSet(u, 1e-6) and r.PointInSet(v, 1e-6)
                    )
                    if not order or order[-1] != found:
                        order.append(found)
                regions = [regions[i] for i in order]
                seeds = [seeds[i] for i in order]
            construction += time.perf_counter() - tic
            repairs += 1
        return dict(
            status="success_numerical" if validation["passed"] else "validation_failed",
            candidate=candidate,
            regions=regions,
            validation=validation,
            recovery_iterations=repairs,
            inflation_calls=self.inflation_calls,
            roundoff_corrections=self.roundoff,
            construction_seconds=construction,
            optimization_seconds=optimization,
            validation_seconds=validation_time,
            total_seconds=construction + optimization,
            wall_seconds=time.perf_counter() - started,
            history=history,
            second_path=second_path,
        )


def save_candidate(out, candidate, regions=None, index=0):
    from pathlib import Path

    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    arrays = {
        key: value for key, value in candidate.items() if isinstance(value, np.ndarray)
    }
    if arrays:
        np.savez_compressed(out / f"candidate_{index:02d}.npz", **arrays)
    if regions:
        for i, region in enumerate(regions):
            np.savez_compressed(
                out / f"region_{index:02d}_{i:02d}.npz", A=region.A(), b=region.b()
            )
    atomic(
        out / f"optimization_{index:02d}.json",
        {
            key: value
            for key, value in candidate.items()
            if not isinstance(value, np.ndarray)
        },
    )


def validate(candidate, forest, regions=None):
    cfg = forest.cfg
    kind = candidate["kind"]
    endpoint = cfg["endpoint_tolerance"]
    tol = cfg["feasibility_tolerance"]
    if kind == "polygon":
        points = np.asarray(candidate["points"])
        record = forest.validate_polygon(points)
        margin = (
            np.array(
                [
                    np.max(
                        regions[i].A() @ np.array([points[j], points[j + 1]]).T
                        - regions[i].b()[:, None]
                    )
                    for j, i in enumerate(candidate["segment_regions"])
                ]
            )
            if regions
            else np.array([0.0])
        )
        which = record.pop("collision_segments")
        record["collision_regions"] = (
            np.array(candidate["segment_regions"])[which]
            if regions
            else np.array([], int)
        )
        record.update(
            cost=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
            duration=None,
            region_residual=float(margin.max()),
            region_pass=bool(margin.max() <= tol),
        )
        record["passed"] = bool(
            record["collision_free"]
            and record["endpoint_pass"]
            and record["limits_pass"]
            and record["region_pass"]
        )
        return record
    if kind == "bezier":
        controls = candidate["controls"]
        durations = candidate["durations"]
        duration = float(durations.sum())
        velocity = (
            (controls.shape[1] - 1)
            * np.diff(controls, axis=1)
            / durations[:, None, None]
        )
        acceleration = (
            (controls.shape[1] - 2)
            * np.diff(velocity, axis=1)
            / durations[:, None, None]
        )
        # Conservative number of samples from a continuous speed bound.
        count = max(
            cfg["validation_minimum_samples"],
            int(
                np.ceil(
                    duration
                    * np.linalg.norm(velocity, axis=2).max()
                    / cfg["validation_step"]
                )
            )
            + 1,
        )
        times = np.linspace(0, duration, count)
        q, segments = bezier_values(controls, durations, times)
        dq, _ = bezier_values(controls, durations, times, 1)
        ddq, _ = bezier_values(controls, durations, times, 2)
        residual = max(
            np.max(region.A() @ cp.T - region.b()[:, None])
            for region, cp in zip(regions, controls)
        )
        continuity = max(
            [0.0]
            + [
                float(np.max(np.abs(controls[i, -1] - controls[i + 1, 0])))
                for i in range(len(controls) - 1)
            ]
            + [
                float(np.max(np.abs(velocity[i, -1] - velocity[i + 1, 0])))
                for i in range(len(controls) - 1)
            ]
        )
        derivative_bounds = dict(
            velocity=float(np.abs(velocity).max()),
            acceleration=float(np.abs(acceleration).max()),
            type="continuous_Bezier_control_point_bounds",
            continuity_residual=continuity,
        )
    elif kind == "bspline":
        trajectory = BsplineTrajectory(
            BsplineBasis(candidate["spline_order"], candidate["knots"].tolist()),
            [point.reshape(2, 1) for point in candidate["controls"]],
        )
        duration = candidate["duration"]
        first = trajectory.MakeDerivative(1)
        second = trajectory.MakeDerivative(2)
        speed_bound = max(np.linalg.norm(cp) for cp in first.control_points())
        count = max(
            cfg["validation_minimum_samples"],
            int(np.ceil(duration * speed_bound / cfg["validation_step"])) + 1,
        )
        count = min(count, 200000)
        times = np.linspace(trajectory.start_time(), trajectory.end_time(), count)
        q = trajectory.vector_values(times).T
        dq = first.vector_values(times).T
        ddq = second.vector_values(times).T
        segments = np.zeros(len(times), int)
        residual = 0.0
        derivative_bounds = dict(
            velocity=float(np.max(np.abs(np.array(first.control_points())))),
            acceleration=float(np.max(np.abs(np.array(second.control_points())))),
            type="continuous_Bspline_derivative_control_point_bounds",
        )
    else:
        raise ValueError(kind)
    margins = forest.clearances(q)
    hits = np.flatnonzero(margins < 0)
    endpoint_error = max(
        np.linalg.norm(q[0] - cfg["start"]), np.linalg.norm(q[-1] - cfg["goal"])
    )
    endpoint_velocity = float(max(np.abs(dq[0]).max(), np.abs(dq[-1]).max()))
    derivatives = bool(
        derivative_bounds["velocity"] <= cfg["velocity_limit"] + tol
        and derivative_bounds["acceleration"] <= cfg["acceleration_limit"] + tol
        and derivative_bounds.get("continuity_residual", 0.0) <= tol
    )
    if candidate.get("method") == "NTO_distance":
        derivatives = True
    bounds = bool(
        np.all(q >= np.array(cfg["domain_lower"]) - tol)
        and np.all(q <= np.array(cfg["domain_upper"]) + tol)
    )
    record = dict(
        passed=bool(
            not len(hits)
            and endpoint_error <= endpoint
            and bounds
            and residual <= tol
            and derivatives
            and (candidate.get("method") == "NTO_distance" or endpoint_velocity <= tol)
        ),
        collision_free=bool(not len(hits)),
        minimum_clearance=float(margins.min()),
        endpoint_error=float(endpoint_error),
        endpoint_velocity=endpoint_velocity,
        limits_pass=bounds,
        region_residual=float(residual),
        derivative_pass=derivatives,
        derivative_bounds=derivative_bounds,
        cost=(
            float(duration)
            if candidate.get("method") != "NTO_distance"
            else float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum())
        ),
        duration=duration,
        samples=len(q),
        maximum_sample_joint_step=float(
            np.linalg.norm(np.diff(q, axis=0), axis=1).max()
        ),
        collision_points=q[hits],
        collision_regions=segments[hits],
        check="independent_analytic_circle_distances_on_derivative_controlled_samples",
    )
    record["_samples"] = dict(time=times, q=q, dq=dq, ddq=ddq)
    return record
