#!/usr/bin/env python3
"""One unscored, CPU-only saved-case diagnostic. DO NOT RUN DURING MAIN TEST.

The only constraint wrapper returns native.Eval(x) unchanged, including its
AutoDiff derivatives. It records x only if that native call raises, then raises
the same exception. No finite differences, changed geometry, or fallback solve.
Prepared by source/API inspection only; runtime behavior is not yet verified.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys
import time
import traceback

ATTEMPT = ('results/rss2025_paper_comparators_v1_dev_s000/jobs/'
           'dev-route_choice-002-p00-NTO_PAPER-minimum_distance-48b608ce/attempts/0001')
TERMINAL_STATUSES = {'success_validated_numerical', 'timeout', 'invalid_endpoint',
    'invalid_scene_geometry', 'model_consistency_failed', 'collision_failed', 'limit_failed',
    'solver_failed', 'frontend_failed', 'validation_undetermined', 'infrastructure_error',
    'unsupported', 'planner_timeout', 'planner_failed'}
EXPECTED_SOURCES = {
    'benchmarks/generic_motion_v1/runners/paper_nto.py':
        'f6697df7115cf258b5cd7773de14aeb98e67a7e3d0952654715f765748c0337f',
    'benchmarks/generic_motion_v1/model.py':
        '5b152e3e5ca484e5378856de11747c45826653df8a1183582fe90e0958fc5116',
}


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def require_completed_main_run(run):
    """Cheap lifecycle gate before any Drake/CSDecomp import or model creation."""
    manifest = read(run / 'run_manifest.json')
    jobs = manifest.get('jobs')
    if not isinstance(jobs, list) or not jobs:
        raise RuntimeError('Main run has no readable nonempty job list')
    unfinished = []
    for job in jobs:
        attempts = sorted((run / 'jobs' / job['job_id'] / 'attempts').glob('[0-9][0-9][0-9][0-9]'))
        if not attempts:
            unfinished.append(job['job_id']); continue
        # The latest attempt must be sealed and terminal even if an older
        # terminal already exists; an actively running later attempt blocks us.
        receipt = attempts[-1] / 'receipt.json'
        if not receipt.is_file():
            unfinished.append(job['job_id']); continue
        value = read(receipt)
        if (not isinstance(value, dict) or value.get('retry_pending') is True
                or value.get('status') not in TERMINAL_STATUSES):
            unfinished.append(job['job_id'])
    if unfinished:
        raise RuntimeError(f'Main test still has {len(unfinished)} unfinished jobs; no diagnostic started')
    return {'run': str(run), 'registered_jobs': len(jobs),
            'latest_attempts_terminal': True,
            'run_manifest_sha256': sha(run / 'run_manifest.json'),
            'scope': 'Lifecycle gate only; not a numerical success assessment'}


def execute(args):
    root = args.repo.resolve()
    main_run = args.main_run.resolve() if args.main_run else root / 'results/test'
    gate = require_completed_main_run(main_run)
    expected_python = (root / '.conda/envs/csdecomp').resolve()
    if Path(sys.prefix).resolve() != expected_python:
        raise RuntimeError('Use the existing local/python.sh Conda interpreter')
    source = root / ATTEMPT
    output = args.output.resolve()
    if output.exists() or output.is_symlink():
        raise RuntimeError('Diagnostic output must be new; no existing evidence will be replaced')
    if output.is_relative_to((root / 'results').resolve()):
        raise RuntimeError('Use an external /tmp diagnostic path, outside the benchmark result index')
    for file, digest in EXPECTED_SOURCES.items():
        if sha(root / file) != digest:
            raise RuntimeError(f'Frozen source changed: {file}; inspect before this one-shot diagnostic')
    receipt = read(source / 'receipt.json')
    for file, digest in receipt['files'].items():
        target = (source / file).resolve()
        if not target.is_relative_to(source.resolve()) or sha(target) != digest:
            raise RuntimeError(f'Original attempt evidence changed: {file}')
    request = read(source / 'request.json')
    if (request['method'] != 'NTO_PAPER' or request['objective'] != 'minimum_distance'
            or request['method_seed'] != 1018884758 or request['case_id'] != 'dev-route_choice-002-p00'):
        raise RuntimeError('Saved paired-seed/method identity changed')
    old = read(source / 'nto_parameters.json')
    case = read(source / 'input.json')
    if (old['solver'] != 'SNOPT' or old['collocation_points'] != 149
            or old['profile']['clearance_m'] != 0.0
            or case['problem_id'] != 'dev-route_choice-002-p00'
            or case['_objective'] != 'minimum_distance'):
        raise RuntimeError('Original case/solver profile differs from the inspected record')
    if importlib.metadata.version('drake') != '1.51.1':
        raise RuntimeError('Drake version changed; this diagnostic targets the existing 1.51.1 backend')
    output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(root))
    import numpy as np
    import csdecomp

    gpu_calls = []
    def forbidden_gpu(*unused_args, **unused_kwargs):
        gpu_calls.append('attempted')
        raise AssertionError('GPU entry point forbidden in this CPU diagnostic')
    for name in dir(csdecomp):
        if 'cuda' in name.lower() and callable(getattr(csdecomp, name)):
            setattr(csdecomp, name, forbidden_gpu)
    from benchmarks.generic_motion_v1.model import Model
    from benchmarks.generic_motion_v1.roadmap import load_config
    from pydrake.autodiffutils import ExtractValue, ExtractGradient, InitializeAutoDiff
    import pydrake.all as drake_all
    from pydrake.solvers import PyFunctionConstraint
    from pydrake.geometry import Box

    Model.cuda = forbidden_gpu
    nto = importlib.import_module('benchmarks.generic_motion_v1.runners.paper_nto')
    path = np.load(source / 'path.npy', allow_pickle=False)
    cfg = load_config(read(source / 'evaluation.json'))
    with np.load(source / 'nto_initial.npz', allow_pickle=False) as initial:
        if (not np.array_equal(path, initial['original_path'])
                or not np.array_equal(path, initial['controls'])):
            raise RuntimeError('Original saved path/initial controls disagree')
    if nto.PROFILE != old['profile']:
        raise RuntimeError('Current paper profile is not the exact saved original profile')
    model = Model(case['voxels_xyz'], case['voxel_radius_m'])
    if (model.robot_sha256 != case['robot_sha256']
            or model.collision_geometry_sha256 != case['collision_geometry_sha256']):
        raise RuntimeError('Canonical model/geometry hash changed')
    native_model = model.nto_plant()
    if native_model.collision_pair_count != old['collision_pairs']:
        raise RuntimeError('Collision pair count changed')
    # Reuse the exact instance and context; no plant is created by the solver.
    model.nto_plant = lambda: native_model
    native_constraint_factory = drake_all.MinimumDistanceLowerBoundConstraint
    native_solver_factory = nto._solver
    native_constraint = None
    captured = None
    active_solve = False
    solve_calls = 0
    events = {'gpu_queries': gpu_calls, 'forwarding_equivalence_checked': False}
    summary = dict(purpose='One CPU diagnostic; never a scored benchmark attempt',
        original_attempt=str(source), original_receipt_sha256=sha(source/'receipt.json'),
        source_sha256=EXPECTED_SOURCES, main_test_gate=gate,
        original_method_seed=request['method_seed'], original_solver_seed=request['solver_seed'],
        original_solver_seed_used=request['solver_seed_used'],
        current_git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
        drake_version=importlib.metadata.version('drake'),
        solver_time_limit_seconds=old['parameters']['Time limit'],
        original_parameters=old, gpu_used=False, finite_differences=False,
        geometry_or_constraint_policy_changed=False, numerical_solve_count=0,
        note='Python forwarding adds callback overhead; elapsed time is diagnostic only')
    write(output/'diagnostic.json', summary)

    def capture_factory(*factory_args, **factory_kwargs):
        nonlocal native_constraint, captured
        if native_constraint is not None:
            raise RuntimeError('Unexpected second minimum-distance evaluator')
        native_constraint = native_constraint_factory(*factory_args, **factory_kwargs)
        def forward(x):
            nonlocal captured
            try:
                # Return the exact native object/AutoDiff values. No conversion,
                # clipping, alternate gradient, epsilon or exception recovery.
                return native_constraint.Eval(x)
            except Exception as exc:
                if active_solve and captured is None:
                    try:
                        q = np.asarray(ExtractValue(x), dtype=float).reshape(-1).copy()
                        scalar_kind = 'AutoDiffXd' if np.asarray(x).dtype.kind=='O' else 'double'
                        derivatives = (np.asarray(ExtractGradient(x), dtype=float).copy()
                            if scalar_kind == 'AutoDiffXd' else np.zeros((len(q), 0)))
                        captured = {'q': q, 'input_gradient': derivatives, 'scalar_kind': scalar_kind,
                                    'exception': repr(exc), 'traceback': traceback.format_exc()}
                        np.savez_compressed(output/'fault_input.npz', q=q, input_gradient=derivatives)
                        write(output/'fault_exception.json', {
                            'exception': captured['exception'], 'traceback': captured['traceback'],
                            'input_source': 'Exact x supplied to the native evaluator that raised',
                            'q_hex': [float(v).hex() for v in q],
                            'scalar_kind': scalar_kind})
                    except Exception as capture_error:
                        # Preserve the original native exception even if logging fails.
                        events['capture_error'] = repr(capture_error)
                raise
        wrapper = PyFunctionConstraint(native_constraint.num_vars(), forward,
            native_constraint.lower_bound(), native_constraint.upper_bound(),
            native_constraint.get_description())
        sparsity = native_constraint.gradient_sparsity_pattern()
        if sparsity is not None:
            wrapper.SetGradientSparsityPattern(sparsity)
        q0 = np.asarray(case['q_start'], dtype=float)
        # This preflight happens only after the main run is complete. It proves
        # exact forwarding at one actual input before allowing any Solve call.
        for x in (q0, InitializeAutoDiff(q0)):
            raw, wrapped = native_constraint.Eval(x), wrapper.Eval(x)
            if not np.array_equal(ExtractValue(raw), ExtractValue(wrapped)):
                raise RuntimeError('Wrapper changed native constraint values')
            if np.asarray(x).dtype.kind == 'O' and not np.array_equal(ExtractGradient(raw), ExtractGradient(wrapped)):
                raise RuntimeError('Wrapper changed native AutoDiff gradients')
        events['forwarding_equivalence_checked'] = True
        return wrapper

    class SolverProxy:
        def __init__(self):
            self.native = native_solver_factory('minimum_distance')
        def available(self): return self.native.available()
        def enabled(self): return self.native.enabled()
        def solver_id(self): return self.native.solver_id()
        def Solve(self, prog, initial_guess=None, solver_options=None):
            nonlocal solve_calls, active_solve
            if solve_calls:
                raise RuntimeError('A second solve/retry is forbidden')
            current = prog.solver_options().options[self.solver_id().name()]
            for name, value in old['parameters'].items():
                if name != 'Time limit' and current.get(name) != value:
                    raise RuntimeError(f'Native solver option changed: {name}')
            if set(current) != set(old['parameters']):
                raise RuntimeError('Native solver option keys changed')
            # The saved original remaining SNOPT budget is exact. Frontend
            # reconstruction is intentionally omitted, not recounted as a new score.
            prog.SetSolverOption(self.solver_id(), 'Time limit', old['parameters']['Time limit'])
            actual = prog.solver_options().options[self.solver_id().name()]
            if actual != old['parameters']:
                raise RuntimeError('Failed to preserve saved numerical solver options')
            write(output/'actual_solver_options.json', actual)
            solve_calls += 1
            active_solve = True
            try:
                return self.native.Solve(prog, initial_guess, solver_options)
            finally:
                active_solve = False
    solver = SolverProxy()
    drake_all.MinimumDistanceLowerBoundConstraint = capture_factory
    nto._solver = lambda objective: solver if objective=='minimum_distance' else (_ for _ in ()).throw(RuntimeError('Wrong objective'))
    start = time.perf_counter()
    try:
        candidate = nto.paper_nto(path, model, case, cfg, output, time.perf_counter()+60.0)
        summary['solver_returned'] = {'success': candidate['solver_success'], 'status': candidate['solver_status']}
        np.savez_compressed(output/'unscored_returned_candidate.npz',
            controls=candidate['controls'], knots=candidate['knots'], duration=candidate['duration'])
    except Exception as exc:
        summary.update(exception=repr(exc), traceback=traceback.format_exc())
    finally:
        nto._solver = native_solver_factory
        drake_all.MinimumDistanceLowerBoundConstraint = native_constraint_factory
        summary.update(numerical_solve_count=solve_calls,
            diagnostic_solve_call_wall_seconds=time.perf_counter()-start, events=events)
        write(output/'diagnostic.json', summary)

    if captured is not None:
        q = captured['q']
        summary['exact_fault_input_saved'] = True
        probes = {}
        probe_inputs = [('double', q)]
        if captured['scalar_kind'] == 'AutoDiffXd':
            probe_inputs.append(('same_input_autodiff', InitializeAutoDiff(q, captured['input_gradient'])))
        for label, x in probe_inputs:
            try:
                result = native_constraint.Eval(x)
                probes[label] = {'raised':False, 'values':ExtractValue(result).reshape(-1).tolist()}
            except Exception as exc:
                probes[label] = {'raised':True, 'exception':repr(exc),
                    'same_native_exception':repr(exc)==captured['exception']}
        summary['direct_original_evaluator_probes'] = probes
        try:
            native_model.plant.SetPositions(native_model.plant_context, q)
            query = native_model.scene_graph.get_query_output_port().Eval(native_model.scene_context)
            inspector = query.inspector()
            pairs = sorted(inspector.GetCollisionCandidates(), key=lambda pair:tuple(sorted(g.get_value() for g in pair)))
            diagnostic_pairs = []
            checked = 0
            for pair in pairs:
                a,b = sorted(pair,key=lambda g:g.get_value())
                if not (isinstance(inspector.GetShape(a),Box) and isinstance(inspector.GetShape(b),Box)):
                    continue
                checked += 1
                row = {'geometry_ids':[a.get_value(),b.get_value()],
                       'names':[inspector.GetName(a),inspector.GetName(b)]}
                try:
                    distance = query.ComputeSignedDistancePairClosestPoints(a,b)
                    if abs(distance.distance) >= 1e-8:
                        continue
                    row.update(distance=distance.distance, p_ACa=distance.p_ACa.tolist(),
                               p_BCb=distance.p_BCb.tolist(), nhat_BA_W=distance.nhat_BA_W.tolist())
                except Exception as exc:
                    row.update(exception=repr(exc), witness_points_available=False)
                row.update(box_sizes_m=[inspector.GetShape(g).size().tolist() for g in (a,b)],
                           X_W=[query.GetPoseInWorld(g).GetAsMatrix4().tolist() for g in (a,b)])
                diagnostic_pairs.append(row)
            write(output/'box_pairs_at_fault.json', {'box_pairs_checked':checked,
                'exception_or_near_touch_pairs':diagnostic_pairs,
                'note':'Explicit same-q pair queries after the one solve; exceptions identify candidate faulty pairs, no geometry altered'})
            summary['box_pair_exceptions'] = sum('exception' in row for row in diagnostic_pairs)
        except Exception as post_error:
            summary['pair_diagnostic_error'] = repr(post_error)
            summary['pair_diagnostic_traceback'] = traceback.format_exc()

    else:
        summary['exact_fault_input_saved'] = False
        summary['capture_limit'] = 'No exception intercepted inside the native distance evaluator; no fault q inferred from a last context or rounded solver log.'
    summary.update(reproduced_boxbox_failure=bool(captured and 'BoxBoxGradient' in captured['exception']),
                   files_sha256={p.name:sha(p) for p in output.iterdir() if p.is_file() and p.name!='diagnostic.json'})
    write(output/'diagnostic.json', summary)
    print(json.dumps({'output':str(output),'numerical_solve_count':solve_calls,
        'reproduced_boxbox_failure':summary['reproduced_boxbox_failure'],
        'exact_fault_input_saved':summary['exact_fault_input_saved']}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,required=True)
    parser.add_argument('--main-run',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--execute-after-main-test',action='store_true')
    args=parser.parse_args()
    if not args.execute_after_main_test:
        parser.error('Prepared diagnostic only: execution requires completed main run and --execute-after-main-test')
    execute(args)


if __name__=='__main__':
    main()
