"""Dimension-independent paper objectives and the actual Drake NTO constraint."""

import numpy as np
from pydrake.all import (
    MathematicalProgram,
    ClarabelSolver,
    SnoptSolver,
    IpoptSolver,
    KinematicTrajectoryOptimization,
    BsplineTrajectory,
    BsplineBasis,
    KnotVectorType,
    HPolyhedron,
)


def lscs(regions, start, goal):
    d = len(start)
    p = MathematicalProgram()
    q = p.NewContinuousVariables(len(regions) + 1, d, "q")
    t = p.NewContinuousVariables(len(regions), "length")
    p.AddLinearEqualityConstraint(np.eye(d), start, q[0])
    p.AddLinearEqualityConstraint(np.eye(d), goal, q[-1])
    p.AddLinearCost(t.sum())
    for i, r in enumerate(regions):
        r.AddPointInSetConstraints(p, q[i])
        r.AddPointInSetConstraints(p, q[i + 1])
        a = np.zeros((d + 1, 2 * d + 1))
        a[0, 0] = 1
        a[1:, 1 : d + 1] = np.eye(d)
        a[1:, d + 1 :] = -np.eye(d)
        p.AddLorentzConeConstraint(a, np.zeros(d + 1), np.r_[t[i], q[i], q[i + 1]])
    result = ClarabelSolver().Solve(p)
    return dict(
        kind="polygon",
        solver_success=result.is_success(),
        solver_status=str(result.get_solution_result()),
        objective=float(result.get_optimal_cost()) if result.is_success() else None,
        points=result.GetSolution(q) if result.is_success() else None,
        segment_regions=list(range(len(regions))),
        solver="Clarabel_substituted_for_MOSEK",
    )


def lgcs(regions, start, goal):
    import gurobipy as gp
    from gurobipy import GRB

    d = len(start)
    n = len(regions)
    source = n
    target = n + 1
    # Each EI-ZO region retains the original box domain. LP bounding computes
    # valid per-axis big-M values; it does not assume a 2D [-5,5] domain.
    from scipy.optimize import linprog

    lower = []
    upper = []
    for k in range(d):
        unit = np.eye(d)[k]
        lows = []
        highs = []
        for r in regions:
            low = linprog(
                unit, A_ub=r.A(), b_ub=r.b(), bounds=[(None, None)] * d, method="highs"
            )
            high = linprog(
                -unit, A_ub=r.A(), b_ub=r.b(), bounds=[(None, None)] * d, method="highs"
            )
            if not low.success or not high.success:
                raise ValueError("Unbounded/empty convex set")
            lows.append(low.fun)
            highs.append(-high.fun)
        lower.append(min(min(lows), start[k], goal[k]) - 1e-7)
        upper.append(max(max(highs), start[k], goal[k]) + 1e-7)
    edges = []
    for i, r in enumerate(regions):
        for j in range(i + 1, n):
            if r.IntersectsWith(regions[j]):
                edges.extend([(i, j), (j, i)])
        if r.PointInSet(start, 1e-6):
            edges.append((source, i))
        if r.PointInSet(goal, 1e-6):
            edges.append((i, target))
    model = gp.Model("LGCS_paper")
    model.Params.OutputFlag = 0
    model.Params.Threads = 1
    model.Params.MIPGap = 1e-6
    model.Params.TimeLimit = 60
    model.Params.NonConvex = 0
    entry = model.addVars(
        n,
        d,
        lb={(i, k): lower[k] for i in range(n) for k in range(d)},
        ub={(i, k): upper[k] for i in range(n) for k in range(d)},
        name="entry",
    )
    exit = model.addVars(
        n,
        d,
        lb={(i, k): lower[k] for i in range(n) for k in range(d)},
        ub={(i, k): upper[k] for i in range(n) for k in range(d)},
        name="exit",
    )
    length = model.addVars(n, lb=0, name="length")
    flow = model.addVars(edges, vtype=GRB.BINARY, name="arc")
    delta = model.addVars(
        n,
        d,
        lb={(i, k): lower[k] - upper[k] for i in range(n) for k in range(d)},
        ub={(i, k): upper[k] - lower[k] for i in range(n) for k in range(d)},
        name="delta",
    )
    order = model.addVars(n, lb=1, ub=n, name="order")
    for i in range(n + 2):
        incoming = gp.quicksum(flow[u, v] for u, v in edges if v == i)
        outgoing = gp.quicksum(flow[u, v] for u, v in edges if u == i)
        model.addConstr(
            outgoing - incoming == (1 if i == source else -1 if i == target else 0)
        )
        model.addConstr(incoming <= 1)
        model.addConstr(outgoing <= 1)
        if i < n:
            for a, b in zip(regions[i].A(), regions[i].b()):
                model.addConstr(
                    gp.quicksum(float(a[k]) * entry[i, k] for k in range(d)) <= float(b)
                )
                model.addConstr(
                    gp.quicksum(float(a[k]) * exit[i, k] for k in range(d)) <= float(b)
                )
            for k in range(d):
                model.addConstr(delta[i, k] == exit[i, k] - entry[i, k])
            model.addQConstr(
                gp.quicksum(delta[i, k] * delta[i, k] for k in range(d))
                <= length[i] * length[i]
            )
    for u, v in edges:
        for k in range(d):
            first = float(start[k]) if u == source else exit[u, k]
            second = float(goal[k]) if v == target else entry[v, k]
            big_m = upper[k] - lower[k]
            model.addConstr(first - second <= big_m * (1 - flow[u, v]))
            model.addConstr(first - second >= -big_m * (1 - flow[u, v]))
        ou = 0 if u == source else order[u]
        ov = n + 1 if v == target else order[v]
        model.addConstr(ov >= ou + 1 - (n + 2) * (1 - flow[u, v]))
    model.setObjective(gp.quicksum(length[i] for i in range(n)), GRB.MINIMIZE)
    model.optimize()
    ok = model.SolCount > 0
    answer = dict(
        kind="polygon",
        solver_success=bool(ok),
        solver_status=int(model.Status),
        solver="Gurobi",
        objective=float(model.ObjVal) if ok else None,
        gap=float(model.MIPGap) if ok else None,
        graph_vertices=n,
        graph_edges=len(edges),
        solver_variables=model.NumVars,
    )
    if ok:
        sequence = []
        u = source
        while u != target:
            successors = [v for a, v in edges if a == u and flow[a, v].X > 0.5]
            if len(successors) != 1:
                raise ValueError("Invalid integer flow")
            u = successors[0]
            if u < n:
                if u in sequence:
                    raise ValueError("Cyclic integer flow")
                sequence.append(u)
        points = [np.asarray(start)]
        for i in sequence:
            arrival = np.array([entry[i, k].X for k in range(d)])
            if np.linalg.norm(arrival - points[-1]) > 1e-5:
                raise ValueError("Arc residual")
            points.append(np.array([exit[i, k].X for k in range(d)]))
        points[-1] = np.asarray(goal)
        answer.update(
            points=np.array(points), segment_regions=sequence, selected_regions=sequence
        )
    return answer


def scsto(regions, start, goal, cfg):
    from scsplanning import biconvex

    d = len(start)
    curve = biconvex(
        np.array(start),
        np.array(goal),
        regions,
        HPolyhedron.MakeBox(
            -np.ones(d) * cfg["velocity_limit"], np.ones(d) * cfg["velocity_limit"]
        ),
        HPolyhedron.MakeBox(
            -np.ones(d) * cfg["acceleration_limit"],
            np.ones(d) * cfg["acceleration_limit"],
        ),
        cfg["bezier_degree"],
        cfg["scsto_relative_tolerance"],
    )
    return dict(
        kind="bezier",
        solver_success=True,
        solver_status="author_biconvex_returned",
        solver="Clarabel",
        controls=np.array([c.points for c in curve.curves], dtype=float),
        durations=np.array([c.duration for c in curve.curves], dtype=float),
        objective=float(curve.duration),
        segment_regions=list(range(len(regions))),
    )


def nto(path, environment, minimum_time=False):
    cfg = environment.protocol
    d = environment.dimension
    path = np.asarray(path, float)
    order = 4 if minimum_time else 2
    count = max(len(path), cfg["nto_minimum_time_controls"] if minimum_time else order)
    if len(path) < count:
        distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        path = np.column_stack(
            [
                np.interp(np.linspace(0, distance[-1], count), distance, path[:, k])
                for k in range(d)
            ]
        )
    duration = (
        float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum() + 2)
        if minimum_time
        else 1.0
    )
    basis = BsplineBasis(order, count, KnotVectorType.kClampedUniform, 0.0, duration)
    initial = BsplineTrajectory(basis, [q.reshape(d, 1) for q in path])
    traj = KinematicTrajectoryOptimization(initial)
    prog = traj.get_mutable_prog()
    traj.AddPositionBounds(environment.lower, environment.upper)
    traj.AddPathPositionConstraint(environment.start, environment.start, 0.0)
    traj.AddPathPositionConstraint(environment.goal, environment.goal, 1.0)
    if minimum_time:
        traj.AddDurationCost(1.0)
        traj.AddDurationConstraint(0.05, 100.0)
        traj.AddVelocityBounds(
            -np.ones(d) * cfg["velocity_limit"], np.ones(d) * cfg["velocity_limit"]
        )
        traj.AddAccelerationBounds(
            -np.ones(d) * cfg["acceleration_limit"],
            np.ones(d) * cfg["acceleration_limit"],
        )
        traj.AddPathVelocityConstraint(np.zeros(d), np.zeros(d), 0.0)
        traj.AddPathVelocityConstraint(np.zeros(d), np.zeros(d), 1.0)
    else:
        traj.AddPathLengthCost()
        traj.AddDurationConstraint(1.0, 1.0)
    problem = "time" if minimum_time else "distance"
    nc = cfg["nto_collocation"][environment.kind][problem]
    constraint = environment.constraint()
    for s in np.linspace(0, 1, nc):
        traj.AddPathPositionConstraint(constraint, float(s))
    name = cfg["nto_solvers"][environment.kind][problem]
    solver = SnoptSolver() if name == "SNOPT" else IpoptSolver()
    if name == "SNOPT":
        for k, v in [
            ("Major iterations limit", cfg["nto_major_iterations"]),
            ("Major feasibility tolerance", 1e-6),
            ("Major optimality tolerance", 1e-6),
            ("Time limit", cfg["optimizer_time_limit_seconds"]),
        ]:
            prog.SetSolverOption(solver.solver_id(), k, v)
    else:
        for k, v in [
            ("max_iter", cfg["nto_major_iterations"]),
            ("max_cpu_time", cfg["optimizer_time_limit_seconds"]),
            ("tol", 1e-6),
            ("print_level", 0),
        ]:
            prog.SetSolverOption(solver.solver_id(), k, v)
    result = solver.Solve(prog)
    candidate = traj.ReconstructTrajectory(result)
    return dict(
        kind="bspline",
        method="NTO_time" if minimum_time else "NTO_distance",
        solver=name,
        solver_success=result.is_success(),
        solver_status=str(result.get_solution_result()),
        objective=(
            float(result.get_optimal_cost())
            if np.isfinite(result.get_optimal_cost())
            else None
        ),
        controls=np.array([q.ravel() for q in candidate.control_points()]),
        knots=np.array(candidate.basis().knots()),
        duration=float(candidate.end_time() - candidate.start_time()),
        spline_order=order,
        control_points=count,
        collocation_points=nc,
        collocation_locations=np.linspace(0, 1, nc).tolist(),
        collision_evaluator="Drake MinimumDistanceLowerBoundConstraint",
    )
