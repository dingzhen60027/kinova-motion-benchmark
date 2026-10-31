"""Fixed-input diagnostics of all four LGCS residual failures and one SCSTO assertion.

No historical source, input, result, region, or acceptance tolerance is modified.
Solver instrumentation is scoped to this process and restored after each call.
"""
import argparse
from contextlib import contextmanager, nullcontext
import hashlib
import importlib
import inspect
import json
from pathlib import Path
import time
import traceback

import numpy as np
from pydrake.all import HPolyhedron, ClarabelSolver, SolverOptions
from reproduction.common import ROOT, atomic, read
from reproduction.paper_constructed.model import ForestEnvironment, FR3
from reproduction.paper_constructed import optimization
from reproduction.paper_constructed.validation import validate
from reproduction.pipeline import save_candidate


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def clean(value):
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, np.generic):
        return clean(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


@contextmanager
def gurobi_trace(out, parameters):
    import gurobipy as gp
    original = gp.Model
    records = []

    class TracedModel(original):
        def optimize(self, *args, **kwargs):
            for key, val in parameters.items():
                self.setParam(key, val)
            self.setParam('LogFile', str(out / 'gurobi.log'))
            self.update()
            self.write(str(out / 'model.mps.gz'))
            settings = {k: getattr(self.Params, k) for k in [
                'Threads', 'MIPGap', 'TimeLimit', 'NonConvex', 'Presolve',
                'FeasibilityTol', 'OptimalityTol', 'BarQCPConvTol', 'IntFeasTol',
                'NumericFocus', 'Seed']}
            record = dict(parameters=settings, variables=self.NumVars,
                          linear_constraints=self.NumConstrs,
                          quadratic_constraints=self.NumQConstrs)
            tic = time.perf_counter()
            try:
                value = super().optimize(*args, **kwargs)
                for key in ['Status', 'SolCount', 'ObjVal', 'ObjBound', 'MIPGap',
                            'Runtime', 'Work', 'NodeCount', 'IterCount', 'BarIterCount',
                            'MaxVio', 'ConstrVio', 'BoundVio', 'IntVio',
                            'ConstrSVio', 'BoundSVio', 'DualVio', 'ComplVio']:
                    try:
                        record[key] = getattr(self, key)
                    except (gp.GurobiError, AttributeError):
                        pass
                if self.SolCount:
                    variables = self.getVars()
                    np.savez_compressed(out / 'gurobi_solution.npz',
                                        x=np.array([v.X for v in variables]),
                                        names=np.array([v.VarName for v in variables]))
                    record['flows'] = {v.VarName: v.X for v in variables
                                       if v.VarName.startswith('phi[')}
                return value
            except Exception as error:
                record['exception'] = repr(error)
                raise
            finally:
                record['instrumented_wall_seconds'] = time.perf_counter() - tic
                records.append(clean(record))
                atomic(out / 'solver_trace.json', records)
    gp.Model = TracedModel
    try:
        yield records
    finally:
        gp.Model = original


def constraint_record(prog, result, out):
    rows, arrays = [], {'decision_values': result.GetSolution(prog.decision_variables())}
    for i, binding in enumerate(prog.GetAllConstraints()):
        evaluator = binding.evaluator()
        x = result.GetSolution(binding.variables())
        y = np.asarray(evaluator.Eval(x)).ravel()
        lb, ub = evaluator.lower_bound(), evaluator.upper_bound()
        violation = np.maximum(np.maximum(lb - y, y - ub), 0)
        arrays[f'c{i:04d}_x'] = x
        arrays[f'c{i:04d}_y'] = y
        arrays[f'c{i:04d}_lb'] = lb
        arrays[f'c{i:04d}_ub'] = ub
        if hasattr(evaluator, 'GetDenseA'):
            arrays[f'c{i:04d}_A'] = evaluator.GetDenseA()
        rows.append(dict(index=i, evaluator=type(evaluator).__name__,
                         description=evaluator.get_description(),
                         max_violation=float(np.max(violation, initial=0)),
                         finite=bool(np.isfinite(y).all()),
                         variables=[str(v) for v in binding.variables()]))
    np.savez_compressed(out / 'program_solution_and_constraints.npz', **arrays)
    atomic(out / 'constraints.json', clean(rows))
    finite = [x['max_violation'] for x in rows if np.isfinite(x['max_violation'])]
    return dict(max_constraint_violation=max(finite, default=None),
                nonfinite_constraint_count=sum(not x['finite'] for x in rows),
                largest_residuals=sorted(rows, key=lambda x: x['max_violation'], reverse=True)[:8])


@contextmanager
def clarabel_trace(out, parameters):
    modules = [importlib.import_module('scsplanning.base_program'),
               importlib.import_module('scsplanning.polygonal')]
    originals = [m.ClarabelSolver for m in modules]
    polygonal_module = modules[1]
    original_vertex_indices = polygonal_module.get_vertex_indices
    options = dict(parameters)
    vertex_tol = options.pop('polygonal_vertex_tolerance', None)

    def vertex_indices(points, tol=1e-4):
        used = tol if vertex_tol is None else vertex_tol
        chosen = original_vertex_indices(points, tol=used)
        residuals = (np.linalg.norm(points[1:-1]-points[:-2],axis=1)
                     +np.linalg.norm(points[2:]-points[1:-1],axis=1)
                     -np.linalg.norm(points[2:]-points[:-2],axis=1))
        atomic(out / 'polygonal_vertex_selection.json', dict(
            points=points, triangle_inequality_residuals=residuals,
            tolerance=used, selected_vertex_indices=chosen,
            original_default_tolerance=1e-4))
        return chosen

    polygonal_module.get_vertex_indices = vertex_indices
    records = []
    actual_solver = ClarabelSolver

    class TracedSolver:
        def Solve(self, prog, *args, **kwargs):
            index = len(records)
            dest = out / f'solve_{index:03d}'
            dest.mkdir()
            caller = inspect.stack()[1]
            incoming = caller.frame.f_locals.get('curve')
            if incoming is not None:
                np.savez_compressed(dest / 'incoming_curve.npz',
                    controls=np.asarray([c.points for c in incoming.curves], float),
                    durations=np.asarray(incoming.durations, float),
                    transition_points=np.asarray(incoming.transition_points, float),
                    transition_velocities=np.asarray(incoming.derivative().transition_points, float))
            opts = SolverOptions()
            for key, value in options.items():
                opts.SetOption(actual_solver.id(), key, value)
            record = dict(index=index, program_class=type(prog).__name__,
                          caller_file=caller.filename, caller_line=caller.lineno,
                          caller_function=caller.function, parameters=parameters,
                          num_vars=prog.num_vars(), num_bindings=len(prog.GetAllConstraints()))
            tic = time.perf_counter()
            try:
                result = actual_solver().Solve(prog, solver_options=opts)
                detail = result.get_solver_details()
                record.update(success=result.is_success(),
                              solution_result=str(result.get_solution_result()),
                              objective=result.get_optimal_cost(),
                              details={k: getattr(detail, k) for k in
                                       ['status', 'iterations', 'solve_time']})
                record.update(constraint_record(prog, result, dest))
                if hasattr(prog, 'T'):
                    record['segment_times'] = result.GetSolution(prog.T)
                return result
            except Exception as error:
                record['exception'] = repr(error)
                raise
            finally:
                record['instrumented_wall_seconds'] = time.perf_counter() - tic
                records.append(clean(record))
                atomic(dest / 'result.json', clean(record))
                atomic(out / 'solver_trace.json', records)
    for m in modules:
        m.ClarabelSolver = TracedSolver
    try:
        yield records
    finally:
        for m, original in zip(modules, originals):
            m.ClarabelSolver = original
        polygonal_module.get_vertex_indices = original_vertex_indices


def load_case(row, indices, identities):
    run = Path(indices[row['environment']]['run_dir'])
    case = run / 'cases' / row['case_id']
    folder = case / row['method']
    manifest = read(run / 'manifest.json')
    cfg = manifest['config']
    data = read(case / 'input.json')
    old = read(folder / 'result.json')
    iteration = int(old.get('recovery_iterations', 0))
    prefix = f'{iteration:02d}'
    paths = [run / 'manifest.json', case / 'input.json', folder / 'result.json',
             folder / f'pre_solve_{prefix}.json', folder / f'pre_solve_seeds_{prefix}.npz']
    paths += sorted(folder.glob(f'pre_solve_region_{prefix}_*.npz'))
    for path in paths:
        identities[str(path)] = sha(path)
    seeds = np.load(folder / f'pre_solve_seeds_{prefix}.npz')
    path = seeds['path']
    regions = []
    for f in sorted(folder.glob(f'pre_solve_region_{prefix}_*.npz')):
        a = np.load(f)
        regions.append(HPolyhedron(a['A'], a['b']))
    if row['environment'] == 'forest':
        env = ForestEnvironment(cfg, data['environment_seed'])
        assert np.array_equal(env.base.centers, np.array(data['centers']))
    else:
        env = FR3(cfg, data['obstacles'])
    env.start, env.goal = path[0].copy(), path[-1].copy()
    env.cfg.update(start=env.start, goal=env.goal)
    assert np.array_equal(env.start, np.asarray(data['start']))
    assert np.array_equal(env.goal, np.asarray(data['goal']))
    return env, regions, path, old, iteration


def replay(row, name, parameters, root, indices, identities, adapter=False):
    folder = root / row['environment'] / row['case_id'] / name
    folder.mkdir(parents=True)
    env, regions, path, old, iteration = load_case(row, indices, identities)
    atomic(folder / 'historical_result.json', old)
    record = dict(environment=row['environment'], case=row['case_id'],
                  method=row['method'], variant=name, parameters=parameters,
                  region_count=len(regions), pre_solve_iteration=iteration, tolerance=env.cfg['feasibility_tolerance'])
    tracer = gurobi_trace if row['method'] == 'LGCS' else clarabel_trace
    tic = time.perf_counter()
    try:
        with (nullcontext() if adapter else tracer(folder, parameters)) as trace:
            if adapter:
                from reproduction.alignment_diagnosis.geometric_solvers import lgcs_strict, scsto_preserve_knots
                import gurobipy as gp
                author = importlib.import_module('scsplanning.biconvex')
                poly = importlib.import_module('scsplanning.polygonal')
                before = (gp.Model, author.biconvex, poly.get_vertex_indices, optimization.lgcs)
                candidate = (lgcs_strict(regions, path[0], path[-1]) if row['method'] == 'LGCS'
                             else scsto_preserve_knots(regions, path[0], path[-1], env.protocol))
                record['module_objects_unchanged'] = before == (gp.Model, author.biconvex, poly.get_vertex_indices, optimization.lgcs)
                assert record['module_objects_unchanged']
            elif row['method'] == 'LGCS':
                candidate = optimization.lgcs(regions, path[0], path[-1])
            else:
                candidate = optimization.scsto(regions, path[0], path[-1], env.protocol)
        candidate['method'] = row['method']
        save_candidate(folder, candidate, regions)
        check = validate(candidate, env, regions)
        np.savez_compressed(folder / 'trajectory_samples.npz', **check.pop('_samples'))
        atomic(folder / 'validation.json', clean(check))
        record.update(status='validated' if check['passed'] else 'validation_failed',
                      solver_success=candidate['solver_success'], objective=candidate['objective'],
                      validation={k: check[k] for k in [
                          'passed', 'region_residual', 'collision_free', 'native_cuda_collision_free',
                          'independent_collision_free', 'endpoint_error', 'limits_pass', 'derivative_pass']})
    except Exception as error:
        record.update(status='error', exception=repr(error), traceback=traceback.format_exc())
        if hasattr(error, 'initialization'):
            atomic(folder/'failed_initialization.json', clean(error.initialization))
    record['elapsed_seconds'] = time.perf_counter() - tic
    atomic(folder / 'result.json', clean(record))
    print(clean(record), flush=True)
    return record


def normal_controls(output):
    """Two fixed interior-point cases with real Forest/FR3 geometry checks."""
    import gurobipy as gp
    from reproduction.alignment_diagnosis.geometric_solvers import lgcs_strict, scsto_preserve_knots
    author = importlib.import_module('scsplanning.biconvex')
    poly = importlib.import_module('scsplanning.polygonal')
    before = (gp.Model, author.biconvex, poly.get_vertex_indices, optimization.lgcs)
    cfg_path = ROOT / 'reproduction/paper_constructed/config.json'
    cfg = read(cfg_path)
    adapter_path = ROOT / 'reproduction/alignment_diagnosis/geometric_solvers.py'
    (output / 'adapter_source_snapshot.py').write_bytes(adapter_path.read_bytes())
    identities = {str(cfg_path): sha(cfg_path), str(adapter_path): sha(adapter_path)}
    forest = ForestEnvironment(cfg, 0)
    forest.start, forest.goal = np.array([-4.8,-4.8]), np.array([-4.0,-4.0])
    fr3 = FR3(cfg)
    fr3.start = np.array([0.,-np.pi/4,0.,-3*np.pi/4,0.,np.pi/2,0.])
    fr3.goal = fr3.start + 0.01
    records = []
    for env in [forest, fr3]:
        env.cfg.update(start=env.start, goal=env.goal)
        lower, upper = np.minimum(env.start,env.goal)-0.02, np.maximum(env.start,env.goal)+0.02
        regions = [HPolyhedron.MakeBox(lower, upper)]
        initial_points = np.vstack([env.start, env.goal])
        endpoint_geometry_free = bool(env.cuda(initial_points).all() and env.independent(initial_points).all())
        for method, function in [('LGCS', lgcs_strict), ('SCSTO', scsto_preserve_knots)]:
            folder = output / f'{env.dimension}d' / method
            folder.mkdir(parents=True)
            record = dict(dimension=env.dimension, method=method, endpoint_geometry_free=endpoint_geometry_free,
                          start=env.start, goal=env.goal, region_lower=lower, region_upper=upper,
                          strict_endpoint_interior_margin=0.02)
            try:
                assert endpoint_geometry_free
                candidate = (function(regions, env.start, env.goal) if method=='LGCS'
                             else function(regions, env.start, env.goal, cfg))
                assert candidate['method'] == method
                save_candidate(folder, candidate, regions)
                result = validate(candidate, env, regions)
                np.savez_compressed(folder/'trajectory_samples.npz', **result.pop('_samples'))
                atomic(folder/'validation.json', clean(result))
                distance = float(np.linalg.norm(env.goal-env.start))
                D = float(np.max(np.abs(env.goal-env.start)))
                lower_time = 2*np.sqrt(D) if D<=1 else D+1
                check = abs(candidate['objective']-distance)<=1e-5 if method=='LGCS' else candidate['objective']>=lower_time-1e-5
                assert check
                assert result['passed']
                record.update(status='validated', objective=candidate['objective'],
                              analytic_length=distance, analytic_time_lower_bound=float(lower_time),
                              analytic_check_passed=bool(check), validation_passed=result['passed'],
                              region_residual=result['region_residual'], derivative_bounds=result['derivative_bounds'])
            except Exception as error:
                record.update(status='error', exception=repr(error), traceback=traceback.format_exc())
            record['module_objects_unchanged'] = before == (gp.Model, author.biconvex, poly.get_vertex_indices, optimization.lgcs)
            atomic(folder/'result.json', clean(record))
            records.append(clean(record))
    verified = all(sha(path)==h for path,h in identities.items())
    atomic(output/'input_hashes.json',identities)
    atomic(output/'summary.json',dict(records=records,input_reverification_passed=verified,
                                      validated=sum(x['status']=='validated' for x in records)))
    print(clean(records),flush=True)
    if not verified or any(x['status']!='validated' or not x['module_objects_unchanged'] for x in records):
        raise RuntimeError('A fixed normal-behavior control failed; all evidence retained')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--methods', nargs='+', default=['LGCS', 'SCSTO'])
    p.add_argument('--use-adapters', action='store_true')
    p.add_argument('--normal-controls', action='store_true')
    p.add_argument('--lgcs-variants', nargs='+', default=['baseline', 'tight_continuous'],
                   choices=['baseline', 'tight_continuous', 'tight_integer', 'tight_all'])
    p.add_argument('--scsto-variant', default='baseline')
    p.add_argument('--clarabel-options', default='{}')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'source_snapshot.py').write_bytes(Path(__file__).read_bytes())
    if args.normal_controls:
        normal_controls(args.output)
        return
    selection = ROOT / 'reproduction/reports/alignment_diagnosis/task_distribution/geometric_failures.json'
    indices_path = ROOT / 'reproduction/reports/paper_constructed/run_index.json'
    rows = [r for r in read(selection) if
            (r['method'] == 'LGCS' and r['status'] == 'validation_failed') or
            (r['method'] == 'SCSTO' and r['status'] == 'error')]
    assert len(rows) == 5
    identities = {str(selection): sha(selection), str(indices_path): sha(indices_path)}
    atomic(args.output / 'scope.json', dict(
        selection=rows, selection_rule='all four LGCS validation failures and the one SCSTO assertion',
        input_v5_unchanged=True, no_acceptance_threshold_relaxation=True,
        no_new_planning_or_regions=True, diagnostic_not_speed_benchmark=True,
        source_sha256=sha(__file__), command_arguments=vars(args)))
    records = []
    for row in rows:
        if row['method'] not in args.methods:
            continue
        if row['method'] == 'LGCS':
            choices = dict(baseline={}, tight_continuous=dict(
                FeasibilityTol=1e-9, OptimalityTol=1e-9, BarQCPConvTol=1e-9),
                tight_integer=dict(IntFeasTol=1e-9),
                tight_all=dict(FeasibilityTol=1e-9, OptimalityTol=1e-9,
                               BarQCPConvTol=1e-9, IntFeasTol=1e-9))
            variants = [(name, choices[name]) for name in args.lgcs_variants]
        else:
            variants = [(args.scsto_variant, json.loads(args.clarabel_options))]
        if args.use_adapters:
            from reproduction.alignment_diagnosis.geometric_solvers import STRICT_GUROBI_PARAMETERS
            variants = [('opt_in_adapter', STRICT_GUROBI_PARAMETERS if row['method']=='LGCS' else {'preserve_all_knots':True})]
            adapter_path = ROOT / 'reproduction/alignment_diagnosis/geometric_solvers.py'
            identities[str(adapter_path)] = sha(adapter_path)
            (args.output / 'adapter_source_snapshot.py').write_bytes(adapter_path.read_bytes())
        for name, params in variants:
            records.append(replay(row, name, params, args.output, read(indices_path), identities, args.use_adapters))
    verified = all(sha(path) == expected for path, expected in identities.items())
    atomic(args.output / 'input_hashes.json', identities)
    atomic(args.output / 'summary.json', dict(records=records, input_reverification_passed=verified,
                                             input_files=len(identities)))
    if not verified:
        raise RuntimeError('Historical input changed during diagnostic')


if __name__ == '__main__':
    main()
