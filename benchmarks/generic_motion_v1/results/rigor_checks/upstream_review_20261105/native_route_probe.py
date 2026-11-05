"""CPU-only routing diagnosis on the sealed real NTO first-solve transcription.

This is not a benchmark score. Both routes retain the same model, task,
initial spline, all mathematical constraints and objective. A 20-iteration
limit bounds this diagnosis. Only SPRAL ordering/scaling differs between the
two routes. GDB counts the located mo_split entry without pretending to have
the historical call stack. Nothing is installed or changed in the environment.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[5]
SOURCE = ROOT / ('benchmarks/generic_motion_v1/results/rigor_checks/dev_nto_incumbent/dev_seed0/jobs/'
    'dev-dense_clutter-001-p01-NTO_REFINED-minimum_time-4e4af77e/attempts/0001')
MODES = {'default_matching': {'spral_order': 'matching', 'spral_scaling': 'matching'}, 'metis_mc64': {'spral_order': 'metis', 'spral_scaling': 'mc64'}}
FAULT_FUNCTION_OFFSET = 0x279b86b


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')


def worker(args):
    sys.path.insert(0, str(ROOT))
    import numpy as np
    from pydrake.all import IpoptSolver
    from benchmarks.generic_motion_v1.model import Model
    from benchmarks.generic_motion_v1.runners import paper_nto as module
    out = Path(args.output)
    receipt = json.loads((SOURCE/'receipt.json').read_text())
    checks = {name: sha(SOURCE/name) == digest for name, digest in receipt['files'].items()}
    if not all(checks.values()):
        raise ValueError('Original attempt receipt mismatch')
    case = json.loads((SOURCE/'input.json').read_text())
    case['_objective'] = 'minimum_time'
    cfg = json.loads((SOURCE/'evaluation.json').read_text())
    profile = json.loads((SOURCE/'refinement/round_00/nto_parameters.json').read_text())['profile']
    profile = dict(profile, maximum_iterations=20, maximum_optimizer_seconds=30.)
    path = np.load(SOURCE/'path.npy', allow_pickle=False)
    initial, _, controls, order, duration = module.warm_start(path, 'minimum_time',
        minimum_time_control_points=profile['minimum_time_control_points'])
    with np.load(SOURCE/'refinement/round_00/nto_initial.npz', allow_pickle=False) as saved:
        np.testing.assert_array_equal(controls, saved['controls'])
        np.testing.assert_array_equal(initial.basis().knots(), saved['knots'])
        np.testing.assert_array_equal(path, saved['original_path'])
        np.testing.assert_equal(duration, saved['duration'])
    model = Model(case['voxels_xyz'], case['voxel_radius_m'])
    if any(getattr(model, k) != case[k] for k in ('robot_sha256', 'collision_geometry_sha256')):
        raise ValueError('Canonical model mismatch')
    def forbidden(*a, **kw):
        raise AssertionError('CUDA forbidden in this CPU-only routing probe')
    model.cuda = forbidden
    native_solver = IpoptSolver()
    class ProbeSolver:
        def available(self):
            return native_solver.available()
        def enabled(self):
            return native_solver.enabled()
        def solver_id(self):
            return native_solver.solver_id()
        def Solve(self, prog, initial_guess=None, solver_options=None):
            options = dict(MODES[args.mode], max_iter=20, print_user_options='yes', file_print_level=7)
            for key, value in options.items():
                prog.SetSolverOption(self.solver_id(), key, value)
            variables = prog.decision_variables()
            positions = {int(v.get_id()): i for i, v in enumerate(variables)}
            constraints = []
            for binding in prog.GetAllConstraints():
                ev = binding.evaluator()
                constraints.append(dict(type=type(ev).__name__, description=ev.get_description(),
                    lower=np.asarray(ev.lower_bound()).tolist(), upper=np.asarray(ev.upper_bound()).tolist(),
                    variables=[positions[int(v.get_id())] for v in binding.variables()]))
            fingerprint = dict(initial=np.asarray(prog.GetInitialGuess(variables)).tolist(),
                constraints=constraints,
                cost_types=[type(b.evaluator()).__name__ for b in prog.GetAllCosts()],
                model_sha256=model.robot_sha256, collision_sha256=model.collision_geometry_sha256,
                sphere_count=len(model.voxels_xyz), sphere_radius=model.voxel_radius)
            # Mathematical bounds contain infinity; canonical JSON permits its explicit representation here.
            encoded = json.dumps(fingerprint, sort_keys=True, separators=(',', ':')).encode()
            dump(out/'transcription.json', dict(sha256=hashlib.sha256(encoded).hexdigest(),
                decision_variables=len(variables), constraints=len(constraints),
                initial_sha256=hashlib.sha256(np.asarray(prog.GetInitialGuess(variables)).tobytes()).hexdigest(),
                explicitly_set_solver_options=options, mathematical_model_unchanged=True))
            return native_solver.Solve(prog, initial_guess, solver_options)
    module._solver = lambda objective: ProbeSolver()
    dump(out/'ready.json', dict(pid=os.getpid(), source_receipt_sha256=sha(SOURCE/'receipt.json'),
        source_files_checked=len(checks), exact_original_initial=True, gpu_queries_forbidden=True,
        mode=args.mode, maximum_iterations=20, cpu_time_ceiling_seconds=30,
        scope='Linear solver routing diagnostic; not benchmark result or timing comparison'))
    os.kill(os.getpid(), signal.SIGSTOP)
    started = time.perf_counter()
    try:
        candidate = module.paper_nto(path, model, case, cfg, out, started+30., _profile=profile)
        dump(out/'completion.json', dict(status='returned', solver_success=bool(candidate['solver_success']),
            solver_status=candidate['solver_status'], wall_seconds=time.perf_counter()-started))
    except BaseException as exc:
        dump(out/'completion.json', dict(status='python_exception', exception=repr(exc),
            traceback=traceback.format_exc(), wall_seconds=time.perf_counter()-started))
        raise


def driver(args):
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    results = []
    for mode in MODES:
        target = out/mode
        target.mkdir()
        counter_path = str(target/'gdb_route.json')
        gdb_commands = '''set pagination off
handle SIGSTOP stop print nopass
run
python
import gdb,json
inferior=gdb.selected_inferior()
lines=open('/proc/%%d/maps' %% inferior.pid).read().splitlines()
entries=[line.split() for line in lines if line.split()[-1].endswith('/pydrake/lib/libdrake.so')]
assert entries, 'libdrake map absent'
base=min(int(x[0].split('-')[0],16)-int(x[2],16) for x in entries)
class EntryCounter(gdb.Breakpoint):
    def __init__(self, address):
        super(EntryCounter,self).__init__('*0x%%x' %% address, internal=True)
        self.count=0
    def stop(self):
        self.count+=1
        return False
counter=EntryCounter(base+%d)
record={'loaded_library_base':hex(base),'entry_offset':hex(%d),'breakpoint':hex(base+%d),'library_maps':[' '.join(x) for x in entries]}
gdb.execute('continue')
record['entry_count']=counter.count
record['inferior_exited']=not gdb.selected_inferior().is_valid() or gdb.selected_inferior().pid==0
open(%r,'w').write(json.dumps(record,indent=2)+'\\n')
end
thread apply all bt
''' % (FAULT_FUNCTION_OFFSET, FAULT_FUNCTION_OFFSET, FAULT_FUNCTION_OFFSET, counter_path)
        (target/'gdb_commands.txt').write_text(gdb_commands)
        command = ['gdb', '--batch', '--quiet', '-x', str(target/'gdb_commands.txt'), '--args',
            str(ROOT/'.conda/envs/csdecomp/bin/python'), str(Path(__file__).resolve()),
            '--worker', '--mode', mode, '--output', str(target)]
        environment=os.environ.copy()
        for key in ('PYTHONPATH','PYTHONHOME','LD_LIBRARY_PATH'):
            environment.pop(key,None)
        environment.update(PYTHONNOUSERSITE='1', CUDA_VISIBLE_DEVICES='', OPENBLAS_NUM_THREADS='1',
            MKL_NUM_THREADS='1', OMP_NUM_THREADS=environment.get('OMP_NUM_THREADS','4'))
        start=time.perf_counter()
        with (target/'gdb.log').open('w') as log:
            try:
                run=subprocess.run(command,cwd=ROOT,env=environment,stdout=log,stderr=subprocess.STDOUT,timeout=70)
                exit_code=run.returncode
            except subprocess.TimeoutExpired:
                exit_code='diagnostic_outer_timeout'
        item=dict(mode=mode, command=command, exit_code=exit_code, seconds=time.perf_counter()-start,
            runtime_source_sha256={name:sha(ROOT/name) for name in ('benchmarks/generic_motion_v1/model.py',
                'benchmarks/generic_motion_v1/runners/paper_nto.py','benchmarks/generic_motion_v1/runners/nto_collision_constraint.py')},
            route=json.loads((target/'gdb_route.json').read_text()) if (target/'gdb_route.json').exists() else None,
            transcription=json.loads((target/'transcription.json').read_text()) if (target/'transcription.json').exists() else None,
            completion=json.loads((target/'completion.json').read_text()) if (target/'completion.json').exists() else None,
            native_sigsegv_observed='received signal SIGSEGV' in (target/'gdb.log').read_text(errors='replace'))
        dump(target/'driver.json',item)
        results.append(item)
    passed=all(x['exit_code']==0 and x['route'] is not None and x['completion'] is not None and not x['native_sigsegv_observed'] for x in results)
    passed=passed and results[0]['route']['entry_count']>0 and results[1]['route']['entry_count']==0
    passed=passed and results[0]['transcription']['sha256']==results[1]['transcription']['sha256']
    same = all(x['transcription'] is not None for x in results) and results[0]['transcription']['sha256']==results[1]['transcription']['sha256']
    summary=dict(scope='CPU routing diagnosis, not benchmark calls or scores',
        same_transcription=bool(same),
        expected_route_verified=bool(passed), script_sha256=sha(__file__), modes=results)
    dump(out/'summary.json',summary)
    print(json.dumps(dict(expected_route_verified=bool(passed), counts=[x['route']['entry_count'] if x['route'] else None for x in results]),indent=2))
    return 0 if passed else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--worker',action='store_true')
    parser.add_argument('--mode',choices=tuple(MODES),default='default_matching')
    args=parser.parse_args()
    if args.worker:
        worker(args)
    else:
        raise SystemExit(driver(args))
