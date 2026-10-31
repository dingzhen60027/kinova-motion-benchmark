"""Opt-in fixed-input repairs; historical v5 and installed modules stay unchanged.

LGCS reuses the original Python code object with a function-local import
factory; SCSTO reuses the author's biconvex code with a function-local
polygonal initializer. Neither adapter changes sys.modules or module globals.
These settings are numerical reconstruction choices, not recovered paper settings.

The knot-preserving initializer follows scsplanning polygonal.py from
https://github.com/TobiaMarcucci/scsplanning/tree/d3f885aa2b74a10418de96e61ec2df0bf6ef16b8

MIT License
Copyright (c) 2024 Tobia Marcucci
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import builtins
import importlib
from types import FunctionType, SimpleNamespace

import numpy as np
from pydrake.all import HPolyhedron
from reproduction.paper_constructed import optimization


STRICT_GUROBI_PARAMETERS = dict(
    FeasibilityTol=1e-9, OptimalityTol=1e-9,
    BarQCPConvTol=1e-9, IntFeasTol=1e-9,
)


def _local_function(function, replacements):
    """Inject dependencies into this call's globals without changing the module."""
    namespace = dict(function.__globals__)
    namespace.update(replacements)
    return FunctionType(function.__code__, namespace, function.__name__,
                        function.__defaults__, function.__closure__)


def lgcs_strict(regions, start, goal):
    """Original v5 GCS model with four explicit 1e-9 numerical tolerances.

    Gurobi's restricted license still applies. No path polishing, geometric
    movement, constraint removal, or acceptance-threshold relaxation is used.
    """
    import gurobipy as gp
    models = []

    def strict_model(*args, **kwargs):
        model = gp.Model(*args, **kwargs)
        model.Params.OutputFlag = 0
        for key, value in STRICT_GUROBI_PARAMETERS.items():
            model.setParam(key, value)
        models.append(model)
        return model

    proxy = SimpleNamespace(**vars(gp))
    proxy.Model = strict_model
    real_import = builtins.__import__

    def local_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name == 'gurobipy' and level == 0:
            return proxy
        return real_import(name, globals, locals, fromlist, level)

    local_builtins = dict(vars(builtins))
    local_builtins['__import__'] = local_import
    solve = _local_function(optimization.lgcs, {'__builtins__': local_builtins})
    answer = solve(regions, start, goal)
    answer['solver_parameters'].update(STRICT_GUROBI_PARAMETERS)
    answer['method'] = 'LGCS'
    answer['adapter'] = 'original_v5_model_with_explicit_strict_gurobi_tolerances'
    answer['source_method'] = 'reproduction.paper_constructed.optimization.lgcs'
    if models and models[-1].SolCount:
        answer['solver_quality'] = {key: float(getattr(models[-1], key))
                                    for key in ['MaxVio', 'ConstrVio', 'IntVio', 'BoundVio']}
    return answer


def scsto_preserve_knots(regions, start, goal, cfg):
    """Author biconvex algorithm with one initial Bezier per original region.

    The original polygonal SOCP, scalar min-time solve, convex subproblems,
    alternating order, and stopping criterion are reused unchanged. Only
    near-collinear-knot deletion/projection in initialization is bypassed.
    A zero-length initial piece is reported explicitly, not silently removed.
    """
    author_polygonal = importlib.import_module('scsplanning.polygonal')
    author_biconvex = importlib.import_module('scsplanning.biconvex').biconvex
    from pybezier import CompositeBezierCurve
    initialization = {}

    def preserve_knots(q_init, q_term, input_regions, vel_set, acc_set, deg):
        points = author_polygonal.get_points(q_init, q_term, input_regions)
        solve_piece = author_polygonal.get_min_time_solver(vel_set, acc_set, deg)
        curves, initial_time = [], 0.0
        for index, (left, right) in enumerate(zip(points[:-1], points[1:])):
            distance = float(np.linalg.norm(right - left))
            if not np.isfinite(distance) or distance == 0:
                raise ValueError(f'Initial region {index} has a zero/nonfinite-length piece')
            curve = solve_piece(left, right, initial_time)
            curves.append(curve)
            initial_time = curve.final_time
        initial = CompositeBezierCurve(curves)
        initialization.update(
            initial_polygonal_points=points,
            retained_knot_indices=list(range(len(points))),
            removed_knot_count=0,
            initial_duration=float(initial.duration),
            initial_region_residual=max(float(np.max(r.A() @ c.points.T-r.b()[:, None]))
                                        for r, c in zip(input_regions, curves)),
            initial_transition_error=float(np.max(np.abs(initial.transition_points-points))),
        )
        residual = initialization['initial_region_residual']
        if not np.isfinite(residual) or residual > cfg['feasibility_tolerance']:
            error = ValueError(
                f'Knot-preserving initialization violates original region tolerance: '
                f'{residual} > {cfg["feasibility_tolerance"]}')
            error.initialization = dict(initialization,
                initial_controls=np.asarray([c.points for c in curves], float),
                initial_durations=np.asarray(initial.durations, float))
            raise error
        return initial

    solve = _local_function(author_biconvex, {'polygonal': preserve_knots})
    d = len(start)
    curve = solve(
        np.asarray(start), np.asarray(goal), regions,
        HPolyhedron.MakeBox(-np.ones(d)*cfg['velocity_limit'], np.ones(d)*cfg['velocity_limit']),
        HPolyhedron.MakeBox(-np.ones(d)*cfg['acceleration_limit'], np.ones(d)*cfg['acceleration_limit']),
        cfg['bezier_degree'], cfg['scsto_relative_tolerance'],
    )
    answer = dict(
        kind='bezier', method='SCSTO', solver_success=True, solver_status='author_biconvex_returned',
        solver='Clarabel', controls=np.asarray([c.points for c in curve.curves], float),
        durations=np.asarray([c.duration for c in curve.curves], float),
        objective=float(curve.duration), segment_regions=list(range(len(regions))),
        adapter='author_biconvex_with_all_polygonal_knots_retained',
        source_commit='d3f885aa2b74a10418de96e61ec2df0bf6ef16b8',
    )
    answer.update(initialization)
    return answer
