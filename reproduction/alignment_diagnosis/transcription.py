"""Controlled NTO variants of v5; unchanged objective and validation policy.

These variants expose unpublished clearance and spline choices. They are
diagnostics, not recovered author settings. The v5 implementation is retained.
"""
import time
import numpy as np
from pydrake.all import (BsplineBasis, BsplineTrajectory, KnotVectorType,
                        KinematicTrajectoryOptimization, SnoptSolver, IpoptSolver,
                        MinimumDistanceLowerBoundConstraint)


def solve(path, environment, spec):
    cfg, d = environment.protocol, environment.dimension
    original = np.asarray(path, float)
    timed = spec['problem'] == 'time'
    order = 4 if timed else 2
    controls = np.vstack([original[0], original, original[-1]]) if timed else original.copy()
    duration = float(np.linalg.norm(np.diff(controls, axis=0), axis=1).sum()+2) if timed else 1.
    initial = BsplineTrajectory(BsplineBasis(order, len(controls), KnotVectorType.kClampedUniform,
                                            0., duration), [q[:, None] for q in controls])
    probe = np.linspace(0., duration, 1001)
    original_values = initial.vector_values(probe)
    added = []
    while len(initial.control_points()) < spec.get('minimum_controls', 0):
        unique = np.unique(initial.basis().knots())
        k = int(np.argmax(np.diff(unique)))
        knot = float((unique[k] + unique[k+1])/2)
        initial.InsertKnots([knot])
        added.append(knot)
    initial_controls = np.array([p.ravel() for p in initial.control_points()])
    shape_error = float(np.max(np.abs(initial.vector_values(probe)-original_values)))
    if shape_error > 1e-10:
        raise ValueError('Knot insertion changed the initial curve')
    trajectory = KinematicTrajectoryOptimization(initial)
    prog = trajectory.get_mutable_prog()
    trajectory.AddPositionBounds(environment.lower, environment.upper)
    trajectory.AddPathPositionConstraint(environment.start, environment.start, 0.)
    trajectory.AddPathPositionConstraint(environment.goal, environment.goal, 1.)
    if timed:
        trajectory.AddDurationCost(1.)
        trajectory.AddDurationConstraint(.05, 100.)
        trajectory.AddVelocityBounds(-np.ones(d)*cfg['velocity_limit'], np.ones(d)*cfg['velocity_limit'])
        trajectory.AddAccelerationBounds(-np.ones(d)*cfg['acceleration_limit'], np.ones(d)*cfg['acceleration_limit'])
        trajectory.AddPathVelocityConstraint(np.zeros(d), np.zeros(d), 0.)
        trajectory.AddPathVelocityConstraint(np.zeros(d), np.zeros(d), 1.)
    else:
        trajectory.AddPathLengthCost()
        trajectory.AddDurationConstraint(1., 1.)
    nc = int(cfg['nto_collocation'][environment.kind][spec['problem']]*spec.get('nc_multiplier', 1))
    locations = np.arange(1, nc+1)/(nc+1)
    constraint = MinimumDistanceLowerBoundConstraint(environment.drake, spec.get('clearance', 0.),
                         environment.pc, influence_distance_offset=cfg['nto_influence_distance'])
    bindings = [trajectory.AddPathPositionConstraint(constraint, float(s)) for s in locations]
    name = cfg['nto_solvers'][environment.kind][spec['problem']]
    solver = SnoptSolver() if name == 'SNOPT' else IpoptSolver()
    options = ({'Major iterations limit': cfg['nto_major_iterations'],
                'Major feasibility tolerance': 1e-6, 'Major optimality tolerance': 1e-6,
                'Time limit': cfg['optimizer_time_limit_seconds']}
               if name == 'SNOPT' else
               {'max_iter': cfg['nto_major_iterations'], 'max_cpu_time': cfg['optimizer_time_limit_seconds'],
                'tol': 1e-6, 'print_level': 0})
    for key, value in options.items():
        prog.SetSolverOption(solver.solver_id(), key, value)
    started = time.perf_counter()
    result = solver.Solve(prog)
    solver_seconds = time.perf_counter()-started
    candidate = trajectory.ReconstructTrajectory(result)
    residuals = np.array([float(result.EvalBinding(b)[0]-b.evaluator().upper_bound()[0]) for b in bindings])
    objective = float(result.get_optimal_cost())
    return dict(kind='bspline', method='NTO_time' if timed else 'NTO_distance',
                solver=name, solver_success=bool(result.is_success()), solver_status=str(result.get_solution_result()),
                objective=objective if np.isfinite(objective) else None,
                controls=np.array([q.ravel() for q in candidate.control_points()]),
                knots=np.array(candidate.basis().knots()), duration=float(candidate.end_time()-candidate.start_time()),
                spline_order=order, control_points=len(candidate.control_points()),
                collocation_parameter_Nc=nc, collocation_points=nc,
                collocation_locations=locations.tolist(), collocation_residuals=residuals,
                max_collocation_residual=float(residuals.max()),
                warm_start_original_nodes=original, warm_start_controls=initial_controls,
                warm_start_knots=np.array(initial.basis().knots()), warm_start_duration=duration,
                warm_start_inserted_knots=added, warm_start_shape_max_error=shape_error,
                warm_start_node_count=len(original), clearance=spec.get('clearance', 0.),
                variant=spec, solver_seconds=solver_seconds,
                provenance='posthoc same-input diagnostic, not author settings')
