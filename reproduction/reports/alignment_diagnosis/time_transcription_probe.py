"""CPU-only diagnostic of the reconstructed minimum-time transcription.

Does not import or modify the benchmark implementation, run any roadmap, or
call GPU geometry. Reads frozen NTO_time candidates and solves an obstacle-free
one-dimensional control-count study. Every result is written to a new JSON.
"""
from __future__ import annotations

import hashlib
import json
import platform
import time
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import scipy
from scipy.interpolate import BSpline, PPoly
from scipy.optimize import linprog
from pydrake.all import (
    BsplineBasis, BsplineTrajectory, KnotVectorType,
    KinematicTrajectoryOptimization, SnoptSolver, IpoptSolver,
)

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).with_suffix('.json')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def exact_component_extrema(controls, knots, order):
    """Numerical polynomial root enumeration, not a temporal sampling grid."""
    controls = np.asarray(controls)
    start, end = float(knots[order - 1]), float(knots[-order])
    results = []
    for derivative in (1, 2):
        maxima = []
        for axis in range(controls.shape[1]):
            curve = BSpline(knots, controls[:, axis], order - 1)
            pp = PPoly.from_spline(curve.derivative(derivative))
            roots = pp.derivative().roots(extrapolate=False)
            candidates = np.r_[start, end, pp.x, roots]
            candidates = candidates[np.isfinite(candidates)]
            candidates = candidates[(candidates >= start) & (candidates <= end)]
            maxima.append(float(np.max(np.abs(pp(candidates)))))
        results.append(maxima)
    return dict(velocity_maxima=results[0], acceleration_maxima=results[1],
                velocity_max=max(results[0]), acceleration_max=max(results[1]))


def describe_curve(curve):
    controls = np.array(curve.control_points())[:, :, 0]
    knots = np.array(curve.basis().knots())
    exact = exact_component_extrema(controls, knots, curve.basis().order())
    hull_v = float(np.abs(np.array(curve.MakeDerivative(1).control_points())).max())
    hull_a = float(np.abs(np.array(curve.MakeDerivative(2).control_points())).max())
    return dict(**exact, velocity_control_bound=hull_v,
                acceleration_control_bound=hull_a,
                endpoint_velocity=float(max(np.abs(curve.EvalDerivative(curve.start_time())).max(),
                                            np.abs(curve.EvalDerivative(curve.end_time())).max())))


def independent_fixed_time_lp(count, duration, distance):
    """Linear feasibility oracle for fixed T; coefficients from SciPy basis."""
    breaks = np.linspace(0, 1, count - 2)
    knots = np.r_[np.zeros(3), breaks, np.ones(3)]
    first, second = [], []
    for j in range(count):
        c = np.zeros(count)
        c[j] = 1
        basis = BSpline(knots, c, 3)
        first.append(basis.derivative(1).c[:count - 1])
        second.append(basis.derivative(2).c[:count - 2])
    first, second = np.array(first).T, np.array(second).T
    equalities = np.zeros((4, count))
    equalities[0, 0], equalities[1, -1] = 1, 1
    equalities[2, :2] = [-1, 1]
    equalities[3, -2:] = [-1, 1]
    a = np.vstack([first, -first, second, -second])
    b = np.r_[np.full(2 * (count - 1), duration),
              np.full(2 * (count - 2), duration ** 2)]
    result = linprog(np.zeros(count), A_ub=a, b_ub=b,
                     A_eq=equalities, b_eq=[0, distance, 0, 0],
                     bounds=[(-1, distance + 1)] * count, method='highs',
                     options={'primal_feasibility_tolerance': 1e-9,
                              'dual_feasibility_tolerance': 1e-9})
    return result


def lp_optimum(count, distance):
    low, high = 0., 40.
    statuses = Counter()
    for _ in range(40):
        middle = (low + high) / 2
        result = independent_fixed_time_lp(count, middle, distance)
        statuses[str(result.status)] += 1
        if result.success:
            high = middle
        elif result.status == 2:
            low = middle
        else:
            raise RuntimeError(f'LP oracle unresolved: {result.message}')
    return dict(lower=low, upper=high, statuses=dict(statuses),
                numerical_feasibility_tolerance=1e-9)


def optimize_empty(count, solver_name, distance=8.):
    initial_controls = np.r_[0., np.linspace(0., distance, count - 2), distance]
    basis = BsplineBasis(4, count, KnotVectorType.kClampedUniform, 0., distance + 2)
    initial = BsplineTrajectory(basis, [np.array([[x]]) for x in initial_controls])
    kto = KinematicTrajectoryOptimization(initial)
    kto.AddPositionBounds(np.array([-1.]), np.array([distance + 1]))
    kto.AddDurationCost(1.)
    kto.AddDurationConstraint(.05, 100.)
    for s, x in ((0., 0.), (1., distance)):
        kto.AddPathPositionConstraint(np.array([x]), np.array([x]), s)
        kto.AddPathVelocityConstraint(np.zeros(1), np.zeros(1), s)
    kto.AddVelocityBounds(-np.ones(1), np.ones(1))
    kto.AddAccelerationBounds(-np.ones(1), np.ones(1))
    prog = kto.get_mutable_prog()
    solver = SnoptSolver() if solver_name == 'SNOPT' else IpoptSolver()
    options = {'Major iterations limit': 2000, 'Major feasibility tolerance': 1e-8,
               'Major optimality tolerance': 1e-8, 'Time limit': 60.}
    if solver_name == 'IPOPT':
        options = {'max_iter': 2000, 'tol': 1e-8, 'max_cpu_time': 60., 'print_level': 0}
    for name, value in options.items():
        prog.SetSolverOption(solver.solver_id(), name, value)
    begin = time.perf_counter()
    result = solver.Solve(prog)
    elapsed = time.perf_counter() - begin
    curve = kto.ReconstructTrajectory(result)
    return dict(count=count, solver=solver_name, solver_success=result.is_success(),
                solver_status=str(result.get_solution_result()),
                duration=float(curve.end_time() - curve.start_time()),
                seconds=elapsed, controls=np.array(curve.control_points())[:, :, 0].tolist(),
                knots=list(curve.basis().knots()), **describe_curve(curve))


def frozen_candidate_audit():
    runs = {
        'forest': '20261026T025110.685774Z_paper_constructed_forest',
        'fr3': '20261026T025846.407684Z_paper_constructed_fr3',
    }
    rows, inputs = [], []
    for environment, name in runs.items():
        for case in sorted((ROOT / 'reproduction/runs' / name / 'cases').iterdir()):
            meta_path = case / 'NTO_time/optimization_00.json'
            candidate_path = case / 'NTO_time/candidate_00.npz'
            result_path = case / 'NTO_time/result.json'
            if not all(p.exists() for p in (meta_path, candidate_path, result_path)):
                continue
            for p in (meta_path, candidate_path, result_path):
                inputs.append(dict(path=str(p.relative_to(ROOT)), sha256=digest(p)))
            meta, result = json.loads(meta_path.read_text()), json.loads(result_path.read_text())
            z = np.load(candidate_path)
            controls, knots = z['controls'], z['knots']
            curve = BsplineTrajectory(BsplineBasis(meta['spline_order'], knots.tolist()),
                                      [q[:, None] for q in controls])
            d = describe_curve(curve)
            duration = float(meta['duration'])
            # This re-times the identical geometric curve for a diagnosis;
            # it does not re-optimize controls or claim collision validity.
            continuous_scale = max(d['velocity_max'], np.sqrt(d['acceleration_max']))
            hull_scale = max(d['velocity_control_bound'],
                             np.sqrt(d['acceleration_control_bound']))
            rows.append(dict(environment=environment, case=case.name,
                             solver_success=meta['solver_success'], status=result['status'],
                             controls=len(controls), duration=duration, **d,
                             same_shape_exact_derivative_duration=duration * continuous_scale,
                             same_shape_hull_derivative_duration=duration * hull_scale,
                             acceleration_bound_minus_exact=d['acceleration_control_bound'] - d['acceleration_max']))
    summary = {}
    for env in runs:
        subset = [r for r in rows if r['environment'] == env]
        strict = [r for r in subset if r['status'] == 'success_numerical']
        solved = [r for r in subset if r['solver_success']]
        by_count = {}
        for n in sorted(set(r['controls'] for r in subset)):
            group = [r for r in strict if r['controls'] == n]
            by_count[str(n)] = dict(candidates=sum(r['controls'] == n for r in subset),
                                   strictly_valid=len(group),
                                   mean_duration=float(np.mean([r['duration'] for r in group])) if group else None,
                                   mean_exact_retimed=float(np.mean([r['same_shape_exact_derivative_duration'] for r in group])) if group else None)
        summary[env] = dict(candidates=len(subset), solved=len(solved), strict=len(strict),
                            maximum_acceleration_envelope_gap=max(abs(r['acceleration_bound_minus_exact']) for r in subset),
                            maximum_velocity_envelope_gap=max(r['velocity_control_bound'] - r['velocity_max'] for r in subset),
                            control_counts=by_count,
                            strict_mean_duration=float(np.mean([r['duration'] for r in strict])),
                            strict_mean_exact_retimed=float(np.mean([r['same_shape_exact_derivative_duration'] for r in strict])),
                            strict_mean_hull_retimed=float(np.mean([r['same_shape_hull_derivative_duration'] for r in strict])))
    return dict(summary=summary, rows=rows, input_hashes=inputs)


def main():
    counts = [4, 5, 6, 8, 12, 20]
    rows, failures = [], []
    for n in counts:
        oracle = lp_optimum(n, 8.)
        for solver in ('SNOPT', 'IPOPT'):
            try:
                row = optimize_empty(n, solver)
                row['fixed_time_lp_duration_bracket'] = oracle
                row['duration_minus_lp_upper'] = row['duration'] - oracle['upper']
                rows.append(row)
                print(n, solver, row['solver_success'], row['duration'], row['velocity_max'], flush=True)
            except Exception as exc:
                failures.append(dict(count=n, solver=solver, error=repr(exc)))
    formula = dict(distance=8., velocity_limit=1., acceleration_limit=1.,
                   unrestricted_bang_bang_trapezoid_duration_lower_bound=9.,
                   four_control_cubic_exact_duration=12.,
                   four_control_cubic_control_hull_duration=24.,
                   explanation='Rest cubic q(s)=D(3s^2-2s^3): peak speed=1.5D/T; velocity control bound=3D/T; peak and control acceleration both=6D/T^2.')
    output = dict(purpose='isolated CPU diagnostic; not a paper benchmark replacement',
                  python=platform.python_version(), scipy=scipy.__version__, numpy=np.__version__,
                  script_sha256=digest(Path(__file__)), analytic_controls=formula,
                  obstacle_free_solves=rows, failures=failures, frozen_candidates=frozen_candidate_audit())
    OUT.write_text(json.dumps(output, indent=2, allow_nan=False) + '\n')
    print(json.dumps(output['frozen_candidates']['summary'], indent=2))


if __name__ == '__main__':
    main()
