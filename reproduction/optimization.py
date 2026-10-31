"""Actual convex, mixed-integer and nonlinear solvers matching paper objectives."""

from __future__ import annotations
import math
import time
import numpy as np
from pydrake.geometry.optimization import HPolyhedron
from pydrake.solvers import MathematicalProgram, ClarabelSolver, SnoptSolver
from pydrake.planning import KinematicTrajectoryOptimization
from pydrake.trajectories import BsplineTrajectory
from pydrake.math import BsplineBasis, KnotVectorType


def lscs(regions, start, goal):
    """Paper Eq.(19), using Clarabel in place of the unavailable MOSEK license."""
    prog = MathematicalProgram()
    q = prog.NewContinuousVariables(len(regions) + 1, 2, "q")
    t = prog.NewContinuousVariables(len(regions), "length")
    prog.AddLinearEqualityConstraint(np.eye(2), start, q[0])
    prog.AddLinearEqualityConstraint(np.eye(2), goal, q[-1])
    prog.AddLinearCost(np.ones(len(t)), 0, t)
    for i, region in enumerate(regions):
        region.AddPointInSetConstraints(prog, q[i])
        region.AddPointInSetConstraints(prog, q[i + 1])
        matrix = np.zeros((3, 5))
        matrix[0, 0] = 1
        matrix[1:, 1:3] = np.eye(2)
        matrix[1:, 3:] = -np.eye(2)
        prog.AddLorentzConeConstraint(matrix, np.zeros(3), np.r_[t[i], q[i], q[i + 1]])
    result = ClarabelSolver().Solve(prog)
    return dict(
        solver_success=result.is_success(),
        solver_status=str(result.get_solution_result()),
        points=result.GetSolution(q) if result.is_success() else None,
        kind="polygon",
        objective=float(result.get_optimal_cost()) if result.is_success() else None,
        segment_regions=list(range(len(regions))),
    )


def clipped_polygon(region):
    polygon = [np.array(p, float) for p in [(-5, -5), (5, -5), (5, 5), (-5, 5)]]
    for normal, bound in zip(region.A(), region.b()):
        if not polygon:
            break
        clipped = []
        for p, q in zip(polygon, polygon[1:] + polygon[:1]):
            vp, vq = normal @ p - bound, normal @ q - bound
            if vp <= 1e-9:
                clipped.append(p)
            if (vp < 0 and vq > 0) or (vp > 0 and vq < 0):
                clipped.append(p + (q - p) * (-vp / (vq - vp)))
        polygon = clipped
    return np.array(polygon)


def intersects(first, second):
    polygon = clipped_polygon(first)
    if len(polygon) == 0:
        return False
    a = np.vstack([first.A(), second.A()])
    b = np.r_[first.b(), second.b()]
    return len(clipped_polygon(HPolyhedron(a, b))) > 0


def lgcs(regions, start, goal):
    """Exact bounded MISOCP for a path through the convex-set intersection graph.

    All region entry/exit points satisfy the region inequalities. Selected arcs
    equate exit/entry points with domain-derived M=10; SOC costs measure each
    in-region segment. Order variables remove zero-cost cycles. Gurobi solves
    the integer model to a recorded gap, rather than a convex relaxation.
    """
    import gurobipy as gp
    from gurobipy import GRB

    model = gp.Model("LGCS")
    model.Params.OutputFlag = 0
    model.Params.Threads = 1
    model.Params.MIPGap = 1e-6
    model.Params.TimeLimit = 60
    model.Params.NonConvex = 0
    count = len(regions)
    source = count
    target = count + 1
    total = count + 2
    sets = list(regions) + [None, None]
    edges = []
    for i in range(count):
        for j in range(i + 1, count):
            if intersects(regions[i], regions[j]):
                edges.extend([(i, j), (j, i)])
        if regions[i].PointInSet(start, 1e-6):
            edges.append((source, i))
        if regions[i].PointInSet(goal, 1e-6):
            edges.append((i, target))
    inside = model.addVars(total, 2, lb=-5, ub=5, name="entry")
    outside = model.addVars(total, 2, lb=-5, ub=5, name="exit")
    length = model.addVars(total, lb=0, name="length")
    delta = model.addVars(total, 2, lb=-10, ub=10, name="delta")
    flow = model.addVars(edges, vtype=GRB.BINARY, name="arc")
    order = model.addVars(total, lb=0, ub=total, name="order")
    model.addConstr(order[source] == 0)
    for i in range(total):
        incoming = gp.quicksum(flow[u, v] for u, v in edges if v == i)
        outgoing = gp.quicksum(flow[u, v] for u, v in edges if u == i)
        model.addConstr(
            outgoing - incoming == (1 if i == source else -1 if i == target else 0)
        )
        model.addConstr(incoming <= 1)
        model.addConstr(outgoing <= 1)
        for k in range(2):
            model.addConstr(delta[i, k] == outside[i, k] - inside[i, k])
        model.addQConstr(
            delta[i, 0] * delta[i, 0] + delta[i, 1] * delta[i, 1]
            <= length[i] * length[i]
        )
        if i < count:
            for a, b in zip(regions[i].A(), regions[i].b()):
                model.addConstr(
                    gp.quicksum(float(a[k]) * inside[i, k] for k in range(2))
                    <= float(b)
                )
                model.addConstr(
                    gp.quicksum(float(a[k]) * outside[i, k] for k in range(2))
                    <= float(b)
                )
        else:
            point = start if i == source else goal
            for k in range(2):
                model.addConstr(inside[i, k] == float(point[k]))
                model.addConstr(outside[i, k] == float(point[k]))
    for u, v in edges:
        for k in range(2):
            model.addConstr(outside[u, k] - inside[v, k] <= 10 * (1 - flow[u, v]))
            model.addConstr(outside[u, k] - inside[v, k] >= -10 * (1 - flow[u, v]))
        model.addConstr(order[v] >= order[u] + 1 - (total + 1) * (1 - flow[u, v]))
    model.setObjective(gp.quicksum(length[i] for i in range(total)), GRB.MINIMIZE)
    model.optimize()
    ok = model.SolCount > 0
    record = dict(
        solver_success=bool(ok),
        solver_status=int(model.Status),
        kind="polygon",
        objective=float(model.ObjVal) if ok else None,
        gap=float(model.MIPGap) if ok else None,
        graph_vertices=count,
        graph_edges=len(edges),
        solver_variables=model.NumVars,
    )
    if not ok:
        return record
    sequence = []
    u = source
    visited = set()
    while u != target:
        if u in visited:
            raise ValueError("Gurobi returned a cyclic main flow")
        visited.add(u)
        successors = [v for a, v in edges if a == u and flow[a, v].X > 0.5]
        if len(successors) != 1:
            raise ValueError("Gurobi main flow is not a single path")
        u = successors[0]
        if u < count:
            sequence.append(u)
    points = [np.array(start)]
    segment_regions = []
    for i in sequence:
        entry = np.array([inside[i, k].X for k in range(2)])
        exit = np.array([outside[i, k].X for k in range(2)])
        # Entry equality has a finite solver residual; every saved segment is
        # independently checked for its assigned region and true collisions.
        if np.linalg.norm(entry - points[-1]) > 1e-5:
            raise ValueError("LGCS arc equality residual exceeds tolerance")
        points.append(exit)
        segment_regions.append(i)
    points[-1] = np.array(goal)
    record.update(
        points=np.array(points),
        segment_regions=segment_regions,
        selected_regions=sequence,
    )
    return record


def scsto(regions, start, goal, cfg):
    from scsplanning import biconvex

    bound = HPolyhedron.MakeBox(
        -np.ones(2) * cfg["velocity_limit"], np.ones(2) * cfg["velocity_limit"]
    )
    acceleration = HPolyhedron.MakeBox(
        -np.ones(2) * cfg["acceleration_limit"], np.ones(2) * cfg["acceleration_limit"]
    )
    curve = biconvex(
        np.array(start),
        np.array(goal),
        regions,
        bound,
        acceleration,
        cfg["bezier_degree"],
        cfg["scsto_relative_tolerance"],
    )
    controls = np.array([segment.points for segment in curve.curves], dtype=float)
    durations = np.array([segment.duration for segment in curve.curves], dtype=float)
    return dict(
        solver_success=True,
        solver_status="author_biconvex_returned",
        kind="bezier",
        controls=controls,
        durations=durations,
        objective=float(curve.duration),
        segment_regions=list(range(len(regions))),
    )


def nto(path, forest, minimum_time=False):
    cfg = forest.cfg
    path = np.asarray(path, float)
    order = 4 if minimum_time else 2
    count = max(
        cfg.get("nto_time_minimum_controls", 4) if minimum_time else order, len(path)
    )
    if len(path) < count:
        d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        path = np.column_stack(
            [np.interp(np.linspace(0, d[-1], count), d, path[:, i]) for i in range(2)]
        )
    if minimum_time:
        path[1] = path[0]
        path[-2] = path[-1]
    duration = (
        float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum() + 2)
        if minimum_time
        else 1.0
    )
    basis = BsplineBasis(order, count, KnotVectorType.kClampedUniform, 0.0, duration)
    initial = BsplineTrajectory(basis, [point.reshape(2, 1) for point in path])
    trajectory = KinematicTrajectoryOptimization(initial)
    prog = trajectory.get_mutable_prog()
    trajectory.AddPositionBounds(
        np.array(cfg["domain_lower"]), np.array(cfg["domain_upper"])
    )
    trajectory.AddPathPositionConstraint(
        np.array(cfg["start"]), np.array(cfg["start"]), 0.0
    )
    trajectory.AddPathPositionConstraint(
        np.array(cfg["goal"]), np.array(cfg["goal"]), 1.0
    )
    if minimum_time:
        trajectory.AddDurationCost(1.0)
        trajectory.AddDurationConstraint(0.05, 100.0)
        trajectory.AddVelocityBounds(
            -np.ones(2) * cfg["velocity_limit"], np.ones(2) * cfg["velocity_limit"]
        )
        trajectory.AddAccelerationBounds(
            -np.ones(2) * cfg["acceleration_limit"],
            np.ones(2) * cfg["acceleration_limit"],
        )
        trajectory.AddPathVelocityConstraint(np.zeros(2), np.zeros(2), 0.0)
        trajectory.AddPathVelocityConstraint(np.zeros(2), np.zeros(2), 1.0)
    else:
        trajectory.AddPathLengthCost()
        trajectory.AddDurationConstraint(1.0, 1.0)
    template = MathematicalProgram()
    dummy = template.NewContinuousVariables(2)

    def clearance(q):
        return np.array(
            [
                (q[0] - center[0]) ** 2
                + (q[1] - center[1]) ** 2
                - (forest.radius + cfg.get("nto_collision_margin", 0.0)) ** 2
                for center in forest.centers
            ]
        )

    evaluator = template.AddConstraint(
        clearance,
        np.zeros(len(forest.centers)),
        np.full(len(forest.centers), np.inf),
        dummy,
    ).evaluator()
    collocation = cfg[
        "nto_time_collocation" if minimum_time else "nto_distance_collocation"
    ]
    locations = np.linspace(0, 1, collocation)
    normalized_knots = np.unique(np.asarray(basis.knots()) / duration)
    claimed = set()
    for knot in normalized_knots[(normalized_knots > 0) & (normalized_knots < 1)]:
        candidates = [i for i in range(1, collocation - 1) if i not in claimed]
        index = min(candidates, key=lambda i: abs(locations[i] - knot))
        locations[index] = knot
        claimed.add(index)
    locations.sort()
    for s in locations:
        trajectory.AddPathPositionConstraint(evaluator, float(s))
    solver = SnoptSolver()
    prog.SetSolverOption(
        solver.solver_id(), "Major iterations limit", cfg["nto_major_iterations"]
    )
    prog.SetSolverOption(solver.solver_id(), "Major feasibility tolerance", 1e-6)
    prog.SetSolverOption(solver.solver_id(), "Major optimality tolerance", 1e-6)
    result = solver.Solve(prog)
    record = dict(
        solver_success=result.is_success(),
        solver_status=str(result.get_solution_result()),
        solver="SNOPT",
        kind="bspline",
        collocation_points=collocation,
        control_points=count,
        spline_order=order,
        objective=(
            float(result.get_optimal_cost())
            if np.isfinite(result.get_optimal_cost())
            else None
        ),
    )
    # Preserve candidate coefficients even when the solver reports failure.
    candidate = trajectory.ReconstructTrajectory(result)
    record["collocation_locations"] = locations.tolist()
    record.update(
        controls=np.array([point.ravel() for point in candidate.control_points()]),
        knots=np.array(candidate.basis().knots()),
        duration=float(candidate.end_time() - candidate.start_time()),
    )
    return record


def bezier_values(controls, durations, times, derivative=0):
    from scipy.special import comb

    points = np.asarray(controls, float)
    durations = np.asarray(durations, float)
    for _ in range(derivative):
        points = (
            np.diff(points, axis=1) * (points.shape[1] - 1) / durations[:, None, None]
        )
    breaks = np.r_[0, np.cumsum(durations)]
    segment = np.clip(
        np.searchsorted(breaks, times, side="right") - 1, 0, len(durations) - 1
    )
    q = np.empty((len(times), points.shape[-1]))
    degree = points.shape[1] - 1
    for i in range(len(durations)):
        mask = segment == i
        u = np.clip((times[mask] - breaks[i]) / durations[i], 0, 1)
        weights = np.array(
            [
                comb(degree, k) * u**k * (1 - u) ** (degree - k)
                for k in range(degree + 1)
            ]
        ).T
        q[mask] = weights @ points[i]
    return q, segment
