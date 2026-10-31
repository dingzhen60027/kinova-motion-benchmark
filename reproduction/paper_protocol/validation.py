"""Independent geometry, continuous derivative bounds and saved dense replay."""

import numpy as np
from pydrake.all import BsplineTrajectory, BsplineBasis
from reproduction.optimization import bezier_values


def polygon_samples(path, step):
    return np.vstack(
        [
            np.linspace(u, v, max(2, int(np.ceil(np.linalg.norm(v - u) / step)) + 1))[
                :-1
            ]
            for u, v in zip(path[:-1], path[1:])
        ]
        + [path[-1:]]
    )


def validate(candidate, environment, regions=None):
    cfg = environment.cfg
    kind = candidate["kind"]
    tol = cfg["feasibility_tolerance"]
    derivative_bounds = {}
    residual = 0.0
    continuity = 0.0
    endpoint_velocity = 0.0
    if kind == "polygon":
        controls = np.asarray(candidate["points"])
        q = polygon_samples(controls, cfg["validation_step"])
        times = np.linspace(0, 1, len(q))
        segments = []
        for j, (u, v) in enumerate(zip(controls[:-1], controls[1:])):
            segments.extend(
                [candidate["segment_regions"][j]]
                * (
                    max(
                        2,
                        int(np.ceil(np.linalg.norm(v - u) / cfg["validation_step"]))
                        + 1,
                    )
                    - 1
                )
            )
        segments = np.r_[segments, candidate["segment_regions"][-1]].astype(int)
        if regions:
            residual = max(
                float(
                    np.max(
                        regions[r].A() @ np.array([controls[j], controls[j + 1]]).T
                        - regions[r].b()[:, None]
                    )
                )
                for j, r in enumerate(candidate["segment_regions"])
            )
        cost = float(np.linalg.norm(np.diff(controls, axis=0), axis=1).sum())
        duration = None
    else:
        if kind == "bezier":
            controls = candidate["controls"]
            durations = candidate["durations"]
            duration = float(durations.sum())
            v = (
                (controls.shape[1] - 1)
                * np.diff(controls, axis=1)
                / durations[:, None, None]
            )
            a = (controls.shape[1] - 2) * np.diff(v, axis=1) / durations[:, None, None]
            speed = float(np.linalg.norm(v, axis=2).max())
            count = max(
                cfg["validation_minimum_samples"],
                int(np.ceil(duration * speed / cfg["validation_step"])) + 1,
            )
            times = np.linspace(0, duration, count)
            q, segments = bezier_values(controls, durations, times)
            dq, _ = bezier_values(controls, durations, times, 1)
            continuity = max(
                [0.0]
                + [
                    float(abs(controls[i, -1] - controls[i + 1, 0]).max())
                    for i in range(len(controls) - 1)
                ]
                + [
                    float(abs(v[i, -1] - v[i + 1, 0]).max())
                    for i in range(len(controls) - 1)
                ]
            )
            if regions:
                residual = max(
                    float(np.max(r.A() @ c.T - r.b()[:, None]))
                    for r, c in zip(regions, controls)
                )
        else:
            trajectory = BsplineTrajectory(
                BsplineBasis(candidate["spline_order"], candidate["knots"].tolist()),
                [p[:, None] for p in candidate["controls"]],
            )
            duration = candidate["duration"]
            dv = trajectory.MakeDerivative(1)
            da = trajectory.MakeDerivative(2)
            v = np.array(dv.control_points())
            a = np.array(da.control_points())
            speed = max(np.linalg.norm(p) for p in dv.control_points())
            count = max(
                cfg["validation_minimum_samples"],
                int(np.ceil(duration * speed / cfg["validation_step"])) + 1,
            )
            if count > 200000:
                raise ValueError(
                    "Candidate needs over 200000 validation points; retained as unresolved rather than coarsening the check"
                )
            times = np.linspace(trajectory.start_time(), trajectory.end_time(), count)
            q = trajectory.vector_values(times).T
            dq = dv.vector_values(times).T
            segments = np.zeros(len(q), int)
        derivative_bounds = dict(
            velocity=float(abs(v).max()),
            acceleration=float(abs(a).max()),
            continuity_residual=continuity,
        )
        endpoint_velocity = float(max(abs(dq[0]).max(), abs(dq[-1]).max()))
        cost = (
            float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum())
            if candidate["method"] == "NTO_distance"
            else duration
        )
    cuda = environment.cuda(q)
    independent = environment.independent(q)
    exact = None
    if environment.kind == "forest" and (
        kind == "polygon" or (kind == "bspline" and candidate["spline_order"] == 2)
    ):
        exact = environment.base.validate_polygon(
            candidate["points"] if kind == "polygon" else candidate["controls"]
        )
        # Exact closest points drive recovery for missed circle crossings.
        if not exact["collision_free"] and independent.all():
            q = np.vstack([q, exact["collision_points"]])
            cuda = environment.cuda(q)
            independent = environment.independent(q)
            extra = (
                np.asarray(candidate["segment_regions"])[exact["collision_segments"]]
                if kind == "polygon"
                else np.zeros(len(exact["collision_points"]), int)
            )
            segments = np.r_[segments, extra]
            times = np.r_[times, np.full(len(extra), np.nan)]
    collision_free = bool(
        independent.all() and cuda.all() and (exact is None or exact["collision_free"])
    )
    endpoint_error = (
        float(
            max(
                np.linalg.norm(q[0] - environment.start),
                np.linalg.norm(q[len(times) - 1] - environment.goal),
            )
        )
        if exact is None or not np.isnan(times[-1])
        else float(exact["endpoint_error"])
    )
    bounds = bool(
        np.all(q >= environment.lower - tol) and np.all(q <= environment.upper + tol)
    )
    derivative_pass = (
        kind == "polygon"
        or candidate["method"] == "NTO_distance"
        or (
            derivative_bounds["velocity"] <= cfg["velocity_limit"] + tol
            and derivative_bounds["acceleration"] <= cfg["acceleration_limit"] + tol
            and continuity <= tol
            and endpoint_velocity <= tol
        )
    )
    hits = np.flatnonzero(~cuda | ~independent)
    return dict(
        passed=bool(
            collision_free
            and endpoint_error <= cfg["endpoint_tolerance"]
            and bounds
            and residual <= tol
            and derivative_pass
        ),
        collision_free=collision_free,
        endpoint_error=endpoint_error,
        endpoint_velocity=endpoint_velocity,
        limits_pass=bounds,
        region_residual=residual,
        derivative_pass=derivative_pass,
        derivative_bounds=derivative_bounds,
        cost=cost,
        duration=duration,
        samples=len(q),
        cuda_collisions=int((~cuda).sum()),
        independent_collisions=int((~independent).sum()),
        model_disagreements=int((cuda != independent).sum()),
        maximum_sample_joint_step=float(
            np.linalg.norm(
                np.diff(q[: np.count_nonzero(np.isfinite(times))], axis=0), axis=1
            ).max()
        ),
        collision_points=q[hits],
        collision_regions=segments[hits],
        minimum_clearance=exact["minimum_clearance"] if exact else None,
        check="independent_Drake_or_exact_circle_numerical_check_not_continuous_certificate",
        _samples=dict(time=times, q=q, cuda=cuda, independent=independent),
    )
