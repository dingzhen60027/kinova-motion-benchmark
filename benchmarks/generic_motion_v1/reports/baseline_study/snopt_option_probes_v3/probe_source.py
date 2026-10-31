"""CPU-only SNOPT option diagnostics on a two-variable Rosenbrock problem."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

CASES = {
    'paper_options': {},
    'minor_print_1': {'Minor print level': 1},
    'print_level_3': {'Print level': 3},
    'summary_stdout': {'Summary file': 6},
    'total_limit_1': {'Iterations limit': 1},
    'major_limit_1': {'Major iterations limit': 1, 'Iterations limit': 1000000},
    'explicit_generous': {'Major iterations limit': 2000, 'Iterations limit': 1000000},
    'common_console': {},
    'common_file': {},
    'common_file_total_limit_1': {'Iterations limit': 1},
    'common_file_major_limit_1': {'Major iterations limit': 1, 'Iterations limit': 1000000},
    'common_file_explicit_generous': {'Iterations limit': 1000000},
    'common_file_rows164': {},
    'common_file_chain100_default': {},
    'common_file_chain100_generous': {'Iterations limit': 1000000},
    'direct_with_summary_rows164': {'Summary file': 6},
    'common_file_rows164_with_summary': {'Summary file': 6},
}


def worker(name, output):
    import numpy as np
    from pydrake.solvers import CommonSolverOption, MathematicalProgram, SnoptSolver, SolverOptions
    directory = output / name
    directory.mkdir(parents=True, exist_ok=True)
    prog = MathematicalProgram()
    dimension = 100 if 'chain100' in name else 2
    x = prog.NewContinuousVariables(dimension)
    prog.AddBoundingBoxConstraint(np.full(dimension, -2.), np.full(dimension, 2.), x)
    prog.AddCost(sum(100 * (x[i + 1] - x[i] ** 2) ** 2 + (1 - x[i]) ** 2 for i in range(dimension - 1)))
    initial_guess = np.tile([-1.2, 1.], dimension // 2)
    prog.SetInitialGuess(x, initial_guess)
    if 'rows164' in name:
        for _ in range(149):
            prog.AddConstraint(x[0] ** 2 + x[1] ** 2, 0., 10.)
        fixed = prog.NewContinuousVariables(14)
        prog.AddLinearEqualityConstraint(np.eye(14), np.zeros(14), fixed)
        prog.SetInitialGuess(fixed, np.zeros(14))
    solver = SnoptSolver()
    params = {'Major iterations limit': 2000, 'Major feasibility tolerance': 1e-6,
              'Major optimality tolerance': 1e-6, 'Major print level': 1,
              'Minor print level': 0, 'Time limit': 10.,
              'Print file': str((directory / 'snopt.log').resolve())}
    params.update(CASES[name])
    options = SolverOptions()
    if name.startswith('common_file'):
        options.SetOption(CommonSolverOption.kPrintFileName, params.pop('Print file'))
    if name == 'common_console':
        options.SetOption(CommonSolverOption.kPrintToConsole, True)
    for key, value in params.items():
        options.SetOption(solver.solver_id(), key, value)
    started = time.perf_counter()
    record = dict(name=name, solver_options=params, problem=f'bounded {dimension}-D Rosenbrock',
                  initial_guess=initial_guess.tolist(), known_global_minimum=[1.] * dimension,
                  common_console=name == 'common_console', common_file=name.startswith('common_file'))
    try:
        result = solver.Solve(prog, None, options)
        details = result.get_solver_details()
        record.update(success=bool(result.is_success()), status=str(result.get_solution_result()),
                      info=int(details.info), native_solve_seconds=float(details.solve_time),
                      objective=float(result.get_optimal_cost()), solution=result.GetSolution(x).tolist())
        np.savez(directory / 'raw.npz', x=result.GetSolution(x), F=details.F,
                 Fmul=details.Fmul, xmul=details.xmul)
    except Exception as exc:
        record.update(exception=repr(exc))
    record['wall_seconds'] = time.perf_counter() - started
    log = directory / 'snopt.log'
    record['print_log_bytes'] = log.stat().st_size if log.exists() else 0
    (directory / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--worker', choices=sorted(CASES))
    args = parser.parse_args()
    if args.worker:
        worker(args.worker, args.output)
        return
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'probe_source.py').write_text(Path(__file__).read_text())
    records = []
    for name in CASES:
        directory = args.output / name
        directory.mkdir(exist_ok=True)
        with (directory / 'stdout.log').open('w') as stdout, (directory / 'stderr.log').open('w') as stderr:
            try:
                run = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--worker', name,
                                      '--output', str(args.output.resolve())], stdout=stdout, stderr=stderr,
                                     timeout=15., check=False)
                record = json.loads((directory / 'result.json').read_text())
                record['worker_exit_code'] = run.returncode
            except Exception as exc:
                record = dict(name=name, process_error=repr(exc))
        record['stdout_bytes'] = (directory / 'stdout.log').stat().st_size
        records.append(record)
    report = dict(script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  probe_scope='Synthetic CPU optimization only; no benchmark query or GPU call', records=records)
    (args.output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
