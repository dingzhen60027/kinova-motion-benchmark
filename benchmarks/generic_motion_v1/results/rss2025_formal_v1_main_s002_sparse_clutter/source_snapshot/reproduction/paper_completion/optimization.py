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
    """Perspective GCS MISOCP (Marcucci et al., Eq. 5.5), after elimination.

    Split each region into entry/exit vertices, joined by a Euclidean-cost
    edge. Inter-region edges have zero cost and identical endpoint positions.
    Eliminate the split vertices' coordinates with the vector-flow equalities.
    A transition point z_e therefore satisfies A_i z_e <= b_i phi_e for
    both incident regions; within-region displacement is sum(z_out-z_in).
    This is an algebraic projection, including of the continuous relaxation,
    of Eq. 5.5 on the split graph. No geometric big-M or order variables.
    """
    import gurobipy as gp
    from gurobipy import GRB

    start, goal = np.asarray(start, float), np.asarray(goal, float)
    d, n = len(start), len(regions)
    source, target = n, n + 1
    if not n:
        raise ValueError("LGCS requires at least one bounded convex region")
    if any(not r.IsBounded() for r in regions):
        raise ValueError("Perspective projection requires bounded convex regions")
    edges = []
    for i, r in enumerate(regions):
        for j in range(i + 1, n):
            if r.IntersectsWith(regions[j]):
                edges.extend([(i, j), (j, i)])
        if r.PointInSet(start, 1e-6):
            edges.append((source, i))
        if r.PointInSet(goal, 1e-6):
            edges.append((i, target))
    interior = [(u, v) for u, v in edges if u < n and v < n]
    model = gp.Model("LGCS_GCS_Eq5_5_projected")
    model.Params.OutputFlag = 0
    model.Params.Threads = 1
    model.Params.MIPGap = 1e-6
    model.Params.TimeLimit = 60
    model.Params.NonConvex = 0
    # Gurobi 13 presolve destroys SOC recognition on the deterministic
    # two-corridor test even with finite delta bounds. Preserve the submitted
    # convex cones, rather than permitting spatial/nonconvex optimization.
    model.Params.Presolve = 0
    flow = model.addVars(edges, vtype=GRB.BINARY, name="phi")
    transition = model.addVars(
        [(u, v, k) for u, v in interior for k in range(d)],
        lb=-GRB.INFINITY, name="z",
    )
    length = model.addVars(n, lb=0, name="length")
    delta = model.addVars(n, d, lb=-GRB.INFINITY, name="delta")

    def edge_point(u, v, k):
        if u == source:
            return float(start[k]) * flow[u, v]
        if v == target:
            return float(goal[k]) * flow[u, v]
        return transition[u, v, k]

    incoming = {i: [(u, v) for u, v in edges if v == i] for i in range(n + 2)}
    outgoing = {i: [(u, v) for u, v in edges if u == i] for i in range(n + 2)}
    for i in range(n + 2):
        phi_in = gp.quicksum(flow[e] for e in incoming[i])
        phi_out = gp.quicksum(flow[e] for e in outgoing[i])
        model.addConstr(phi_out - phi_in == (1 if i == source else -1 if i == target else 0))
        if i < n:
            model.addConstr(phi_out <= 1)
            for k in range(d):
                model.addConstr(
                    delta[i, k]
                    == gp.quicksum(edge_point(u, v, k) for u, v in outgoing[i])
                    - gp.quicksum(edge_point(u, v, k) for u, v in incoming[i])
                )
            model.addQConstr(gp.quicksum(delta[i, k] ** 2 for k in range(d)) <= length[i] ** 2)
    # Homogeneous membership on every incident set is essential: dropping it
    # would weaken Eq. 5.5 and would not be this projected formulation.
    perspective_rows = 0
    for u, v in edges:
        rows = {}
        for i in (u, v):
            if i < n:
                for a, b in zip(regions[i].A(), regions[i].b()):
                    key = tuple(float(x) for x in a)
                    rows[key] = min(rows.get(key, float("inf")), float(b))
        for a, b in rows.items():
            model.addConstr(
                gp.quicksum(a[k] * edge_point(u, v, k) for k in range(d)) <= b * flow[u, v]
            )
            perspective_rows += 1
    model.setObjective(gp.quicksum(length.values()), GRB.MINIMIZE)
    model.update()
    metadata = dict(
        kind="polygon", solver="Gurobi",
        formulation="GCS_Eq5.5_split_vertex_exact_projection",
        graph_vertices=n, graph_edges=len(edges),
        split_graph_vertices=2 * n + 2, split_graph_edges=n + len(edges),
        solver_variables=model.NumVars, solver_constraints=model.NumConstrs,
        solver_quadratic_constraints=model.NumQConstrs,
        perspective_membership_rows=perspective_rows,
        solver_parameters=dict(Threads=1, MIPGap=1e-6, TimeLimit=60,
                               NonConvex=0, Presolve=0),
    )
    # Preserve the solver exception (including license code) for the runner;
    # attach the actual program dimensions to aid diagnosis without retries.
    try:
        model.optimize()
    except gp.GurobiError as error:
        error.optimization_metadata = metadata
        raise
    ok = model.SolCount > 0
    answer = dict(
        **metadata, solver_success=bool(ok), solver_status=int(model.Status),
        objective=float(model.ObjVal) if ok else None,
        gap=float(model.MIPGap) if ok else None,
        globally_optimal=bool(model.Status == GRB.OPTIMAL),
    )
    if ok:
        sequence, selected_edges, points = [], [], [start.copy()]
        u = source
        while u != target:
            successors = [v for a, v in outgoing[u] if flow[a, v].X > 0.5]
            if len(successors) != 1:
                raise ValueError("Invalid integer flow in GCS solution")
            v = successors[0]
            selected_edges.append((u, v))
            if u < n:
                end = goal.copy() if v == target else np.array([transition[u, v, k].X for k in range(d)])
                points.append(end)
            u = v
            if u < n:
                if u in sequence:
                    raise ValueError("Cyclic source-to-target integer flow")
                sequence.append(u)
        # Eq. 5.5 permits disjoint zero-cost cycles; removing them cannot
        # increase the recovered source-to-target path cost (Theorem 5.7).
        discarded = [e for e in edges if flow[e].X > 0.5 and e not in selected_edges]
        points = np.array(points)
        answer.update(
            points=points, segment_regions=sequence, selected_regions=sequence,
            discarded_disjoint_cycle_edges=[list(e) for e in discarded],
            recovered_path_length=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
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
    if path.ndim != 2 or path.shape[1] != d or len(path) < 2:
        raise ValueError("NTO requires the ordered DRM nodes, including both endpoints")
    if not np.isfinite(path).all():
        raise ValueError("Non-finite DRM warm start")
    original_path = path.copy()
    order = 4 if minimum_time else 2
    repeated = bool(minimum_time and cfg.get("nto_repeat_initial_endpoints", True))
    if repeated:
        # With a clamped cubic, c0=c1 and c[-2]=c[-1] enforce zero endpoint
        # velocities. Retain every DRM node and append only those two copies.
        # This handling of rest endpoints is an explicitly declared inference;
        # the paper does not release its minimum-time transcription.
        path = np.vstack([path[0], path, path[-1]])
    count = len(path)
    if count < order:
        raise ValueError("Too few DRM control points for the requested spline order")
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
    locations_mode = cfg.get("nto_collocation_locations", "author_tex_interior_k_over_Nc")
    if locations_mode != "author_tex_interior_k_over_Nc":
        raise ValueError(f"Unsupported completion-protocol collocation: {locations_mode}")
    # The archived author's TeX (commented derivation) explicitly gives k/Nc,
    # k=1,...,Nc-1. Nc is thus the partition count, not the number of bindings.
    locations = np.arange(1, nc, dtype=float) / nc
    for s in locations:
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
        collocation_parameter_Nc=nc,
        collocation_points=len(locations),
        collocation_locations=locations.tolist(),
        collocation_source="author_arxiv_TeX_commented_appendix_derivation",
        warm_start_original_nodes=original_path,
        warm_start_controls=path,
        warm_start_node_count=len(original_path),
        warm_start_endpoint_copies=2 if repeated else 0,
        warm_start_resampled=False,
        warm_start_duration=duration,
        collision_evaluator="Drake MinimumDistanceLowerBoundConstraint",
    )
