"""Continuous-parameter Forest checks using floating-point polynomial roots.

Every polynomial piece is checked at its endpoints and at the stationary
points of squared distance to every circle.  This avoids a validation grid's
between-sample blind spots, but is *not* an interval or exact-arithmetic safety
certificate.  A failed conversion/root diagnostic is retained as unresolved.
No native collision library, Drake model, or GPU is used by this module.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import comb

import numpy as np
from numpy.polynomial import polynomial as poly
from scipy.interpolate import BSpline, PPoly


METHOD = "piecewise_polynomial_stationary_points_float64"
_MAX_DEGREE = 12
_ROOT_IMAG_TOL = 1e-9
_ROOT_AMBIGUITY_TOL = 1e-5
_ROOT_RESIDUAL_TOL = 1e-7
_CONVERSION_REL_TOL = 1e-9


@dataclass
class _Piece:
    start: float
    end: float
    coefficients: np.ndarray  # Increasing powers of local u in [0, 1].
    evaluate: object


def _finite(value, name, ndim=None):
    result = np.asarray(value, dtype=float)
    if not np.isfinite(result).all() or (ndim is not None and result.ndim != ndim):
        raise ValueError(f"{name} must have the expected shape and finite values")
    return result


def _de_casteljau(controls, u):
    work = controls.copy()
    for remaining in range(len(work)-1, 0, -1):
        work[:remaining] = (1-u)*work[:remaining] + u*work[1:remaining+1]
    return work[0]


def _bezier_power(controls):
    degree = len(controls)-1
    # C(n,k) Delta^k P_0 is the power coefficient of u^k.  Taking
    # differences first avoids expanding large common coordinate offsets.
    return np.asarray([comb(degree, k)*np.diff(controls, n=k, axis=0)[0]
                       for k in range(degree+1)])


def _pieces(candidate):
    kind = candidate.get("kind")
    if kind == "polygon":
        points = _finite(candidate.get("points"), "polygon points", 2)
        if points.shape[1:] != (2,) or len(points) < 2:
            raise ValueError("polygon points must be N x 2 with N >= 2")
        pieces = []
        for index, (first, second) in enumerate(zip(points[:-1], points[1:])):
            coefficients = np.array([first, second-first])
            pieces.append(_Piece(index/(len(points)-1), (index+1)/(len(points)-1),
                coefficients, lambda u, a=first, b=second: (1-u)*a+u*b))
        return pieces, 0., 1., "equal_polygon_segment_parameter"
    if kind == "bezier":
        controls = _finite(candidate.get("controls"), "Bezier controls", 3)
        durations = _finite(candidate.get("durations"), "Bezier durations", 1)
        if (controls.shape[0] < 1 or controls.shape[1] < 1 or controls.shape[2] != 2
                or durations.shape != (len(controls),) or np.any(durations <= 0)):
            raise ValueError("Bezier controls/durations must be shape-consistent and durations positive")
        if controls.shape[1]-1 > _MAX_DEGREE:
            raise ValueError(f"curve degree exceeds numerical check limit {_MAX_DEGREE}")
        edges = np.r_[0., np.cumsum(durations)]
        if not np.isfinite(edges).all() or np.any(np.diff(edges) <= 0):
            raise ValueError("Bezier cumulative time is not finite and strictly increasing")
        pieces = [_Piece(float(edges[i]), float(edges[i+1]), _bezier_power(c),
                         lambda u, c=c: _de_casteljau(c, u)) for i, c in enumerate(controls)]
        return pieces, 0., float(edges[-1]), "cumulative_bezier_duration"
    if kind == "bspline":
        controls = _finite(candidate.get("controls"), "B-spline controls", 2)
        knots = _finite(candidate.get("knots"), "B-spline knots", 1)
        order = candidate.get("spline_order")
        duration = float(candidate.get("duration", np.nan))
        if (not isinstance(order, (int, np.integer)) or isinstance(order, (bool, np.bool_))
                or order < 1 or order-1 > _MAX_DEGREE or controls.shape[1:] != (2,)
                or len(controls) < order or len(knots) != len(controls)+order
                or np.any(np.diff(knots) < 0) or not np.isfinite(duration) or duration <= 0):
            raise ValueError("B-spline controls, knots, order, or duration are invalid")
        start, end = float(knots[order-1]), float(knots[-order])
        if end <= start or not np.isclose(end-start, duration, rtol=1e-9, atol=1e-10):
            raise ValueError("B-spline duration disagrees with its active knot interval")
        splines = [BSpline(knots, controls[:, axis], order-1, extrapolate=False) for axis in range(2)]
        powers = [PPoly.from_spline(spline) for spline in splines]
        pieces = []
        for i, (left, right) in enumerate(zip(powers[0].x[:-1], powers[0].x[1:])):
            if right <= left or left < start or right > end:
                continue
            coefficients = np.column_stack([p.c[:, i][::-1] for p in powers])
            coefficients *= (right-left)**np.arange(order)[:, None]

            def evaluate(u, left=left, right=right):
                t = left+(right-left)*u
                # Include the left limit of an interior knot.  Discontinuous
                # pieces are separately rejected by the continuity diagnostic.
                if u == 1. and right < end:
                    t = np.nextafter(right, left)
                return np.array([spline(t) for spline in splines])

            pieces.append(_Piece(float(left), float(right), coefficients, evaluate))
        if not pieces:
            raise ValueError("B-spline has no positive-length active knot spans")
        return pieces, start, end, "active_bspline_knot_interval"
    raise ValueError(f"Unsupported Forest candidate representation: {kind!r}")


def _stationary_parameters(coefficients):
    """Return candidate real roots and explicit numerical concerns."""
    coefficients = poly.polytrim(np.asarray(coefficients, dtype=float), tol=0.)
    if not np.isfinite(coefficients).all():
        return [], ["nonfinite_stationary_polynomial"], 0., 0
    if len(coefficients) <= 1:
        return [], [], 0., 0
    scale = float(np.max(np.abs(coefficients)))
    normalized = coefficients/scale
    roots = poly.polyroots(normalized)
    if len(roots) != len(coefficients)-1 or not np.isfinite(roots).all():
        return [], ["nonfinite_or_missing_stationary_roots"], 0., len(roots)
    accepted, issues, max_residual = [], [], 0.
    for root in roots:
        real, imag = float(np.real(root)), abs(float(np.imag(root)))
        if not -_ROOT_AMBIGUITY_TOL <= real <= 1+_ROOT_AMBIGUITY_TOL:
            continue
        if imag > _ROOT_AMBIGUITY_TOL*(1+abs(real)):
            continue
        denominator = float(poly.polyval(abs(root), np.abs(normalized)))
        residual = float(abs(poly.polyval(root, normalized))/max(denominator, np.finfo(float).tiny))
        max_residual = max(max_residual, residual)
        if not np.isfinite(residual) or residual > _ROOT_RESIDUAL_TOL:
            issues.append("stationary_root_residual_exceeds_tolerance")
        if imag > _ROOT_IMAG_TOL*(1+abs(real)):
            # A nearly real conjugate pair may be an ill-conditioned multiple
            # real root.  Evaluate its projection but do not certify a pass.
            issues.append("ambiguous_near_real_stationary_root")
        if -_ROOT_IMAG_TOL <= real <= 1+_ROOT_IMAG_TOL:
            accepted.append(float(np.clip(real, 0., 1.)))
    return accepted, sorted(set(issues)), max_residual, len(roots)


def forest_curve_check(candidate, environment):
    """Numerically minimize circle clearance over every complete curve piece.

    ``collision_free`` requires a resolved check and minimum clearance >= 0.
    Exact tangency is allowed, matching Forest's existing circle predicate;
    negative clearance is never waived using a solver feasibility tolerance.
    ``feasibility_tolerance`` is recorded for near-contact and continuity
    diagnostics only.  ``clearance_witnesses`` contains a minimum for each
    piece/obstacle pair so a solver can also enforce a *positive* margin.
    Witness parameters normalize the full candidate's physical time domain;
    polygon candidates use equal segment-index intervals.

    Invalid inputs and numerical root/conversion failures return unresolved
    evidence instead of raising or silently accepting the trajectory.
    """
    result = dict(method=METHOD, guarantee="float64_numerical_check_not_interval_certificate",
        resolved=False, status="unresolved", collision_free=False, minimum_clearance=None,
        minimum_squared_center_distance=None, normalized_collision_parameters=[],
        normalized_minimum_parameters=[], collision_points=[], collision_segments=[],
        collision_obstacles=[], clearance_witnesses=[],
        diagnostics=dict(issues=[], pieces=0, obstacle_count=0, stationary_roots=0,
                         max_root_backward_residual=0., max_conversion_error=0.,
                         continuity_residual=0., maximum_supported_degree=_MAX_DEGREE))
    diagnostics = result["diagnostics"]
    try:
        if getattr(environment, "kind", "forest") != "forest":
            raise ValueError("forest_curve_check requires a Forest environment")
        cfg = environment.cfg
        tolerance = float(cfg.get("feasibility_tolerance", 1e-5))
        if not np.isfinite(tolerance) or tolerance < 0:
            raise ValueError("feasibility_tolerance must be finite and nonnegative")
        centers = _finite(environment.base.centers, "Forest circle centers", 2)
        if centers.shape[1:] != (2,) or len(centers) == 0:
            raise ValueError("Forest circle centers must be a nonempty N x 2 array")
        obstacle_radius, point_radius = float(cfg["obstacle_radius"]), float(cfg["point_radius"])
        radius = obstacle_radius+point_radius
        if (not np.isfinite([obstacle_radius, point_radius, radius]).all()
                or obstacle_radius <= 0 or point_radius < 0):
            raise ValueError("Forest radii must be finite; obstacle positive and robot nonnegative")
        if hasattr(environment.base, "radius") and not np.isclose(
                environment.base.radius, radius, rtol=1e-12, atol=1e-15):
            raise ValueError("Forest effective radius disagrees with the model")
        pieces, start, end, parameterization = _pieces(candidate)
        result.update(effective_radius=radius, obstacle_radius=obstacle_radius,
                      robot_point_radius=point_radius, feasibility_tolerance=tolerance,
                      contact_policy="computed_clearance_greater_than_or_equal_to_zero",
                      parameter_domain=[start, end], parameterization=parameterization)
        diagnostics.update(pieces=len(pieces), obstacle_count=len(centers))
        prior_endpoint = None
        for index, piece in enumerate(pieces):
            c = piece.coefficients
            if not np.isfinite(c).all():
                raise FloatingPointError(f"nonfinite coefficients in piece {index}")
            grid = np.linspace(0., 1., max(9, len(c)+1))
            source = np.asarray([piece.evaluate(u) for u in grid])
            converted = poly.polyval(grid, c).T
            if not np.isfinite(source).all() or not np.isfinite(converted).all():
                raise FloatingPointError(f"nonfinite conversion values in piece {index}")
            error = float(np.max(np.abs(source-converted)))
            diagnostics["max_conversion_error"] = max(diagnostics["max_conversion_error"], error)
            if error > _CONVERSION_REL_TOL*max(1., float(np.max(np.abs(source)))):
                diagnostics["issues"].append(dict(piece=index, reason="power_conversion_disagreement"))
            if prior_endpoint is not None:
                residual = float(np.max(np.abs(source[0]-prior_endpoint)))
                diagnostics["continuity_residual"] = max(diagnostics["continuity_residual"], residual)
                if residual > tolerance:
                    diagnostics["issues"].append(dict(piece=index, reason="discontinuous_curve"))
            prior_endpoint = source[-1]
            for obstacle, center in enumerate(centers):
                delta = c.copy()
                delta[0] -= center
                squared = poly.polyadd(poly.polymul(delta[:, 0], delta[:, 0]),
                                       poly.polymul(delta[:, 1], delta[:, 1]))
                roots, issues, residual, root_count = _stationary_parameters(poly.polyder(squared))
                diagnostics["stationary_roots"] += root_count
                diagnostics["max_root_backward_residual"] = max(diagnostics["max_root_backward_residual"], residual)
                diagnostics["issues"].extend(dict(piece=index, obstacle=obstacle, reason=reason) for reason in issues)
                parameters = np.unique(np.r_[0., roots, 1.])
                # Evaluate the original curve representation at each candidate
                # root to reduce cancellation in the squared-distance polynomial.
                points = np.asarray([piece.evaluate(u) for u in parameters])
                distances = np.linalg.norm(points-center, axis=1)
                if not np.isfinite(points).all() or not np.isfinite(distances).all():
                    raise FloatingPointError(f"nonfinite distance in piece {index}")
                best = int(np.argmin(distances))
                parameter = float(np.clip((piece.start+(piece.end-piece.start)*parameters[best]-start)/(end-start), 0., 1.))
                clearance = float(distances[best]-radius)
                witness = dict(piece=index, obstacle=obstacle, normalized_parameter=parameter,
                               point=points[best].tolist(), clearance=clearance,
                               squared_center_distance=float(distances[best]**2))
                result["clearance_witnesses"].append(witness)
                if clearance < 0:
                    result["normalized_collision_parameters"].append(parameter)
                    result["collision_points"].append(witness["point"])
                    result["collision_segments"].append(index)
                    result["collision_obstacles"].append(obstacle)
        witnesses = result["clearance_witnesses"]
        minimum = min(witnesses, key=lambda item: item["clearance"])
        result["minimum_clearance"] = minimum["clearance"]
        result["minimum_squared_center_distance"] = min(w["squared_center_distance"] for w in witnesses)
        result["normalized_minimum_parameters"] = [w["normalized_parameter"] for w in witnesses
            if abs(w["clearance"]-minimum["clearance"]) <= 1e-12]
        result["near_contact"] = bool(abs(minimum["clearance"]) <= tolerance)
        result["resolved"] = not diagnostics["issues"]
        result["collision_free"] = bool(result["resolved"] and minimum["clearance"] >= 0.)
        result["status"] = ("clear" if result["collision_free"] else "collision"
                            if minimum["clearance"] < 0 else "unresolved")
    except (ValueError, TypeError, AttributeError, KeyError, FloatingPointError, np.linalg.LinAlgError, OverflowError) as exc:
        diagnostics["issues"].append(dict(reason="check_exception", exception=f"{type(exc).__name__}: {exc}"))
    return result
