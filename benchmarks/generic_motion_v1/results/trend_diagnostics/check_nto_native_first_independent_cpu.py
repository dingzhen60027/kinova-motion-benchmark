"""Independent CPU binding/transaction probe; never invokes a real optimizer."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import traceback
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from pydrake.all import (BsplineBasis, BsplineTrajectory, KinematicTrajectoryOptimization,
                        KnotVectorType, LinearConstraint, MathematicalProgram, SolverId)
from pydrake.autodiffutils import InitializeAutoDiff, ExtractValue, ExtractGradient

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from benchmarks.generic_motion_v1.common import sha
from benchmarks.generic_motion_v1.runners import nto_collision_constraint as module


class Backend:
    def __init__(self, directory):
        self.out = directory / 'backend.json'
        self.original = LinearConstraint(np.array([[1., 2.], [-2., 1.]]), [-1., -2.], [1., 2.])
        self.guarded_evaluations = 0
        self.owner = MathematicalProgram()
        variables = self.owner.NewContinuousVariables(2)
        self.constraint = self.owner.AddConstraint(self.evaluate, [-1., -2.], [1., 2.], variables).evaluator()

    def evaluate(self, q):
        self.guarded_evaluations += 1
        return self.original.Eval(q)

    def save(self):
        return deepcopy(getattr(self, 'execution', {}))


def trajectory():
    initial = BsplineTrajectory(BsplineBasis(3, 4, KnotVectorType.kClampedUniform, 0., 2.),
                                [np.array([[.2 + .1*i], [-.3 + .2*i]]) for i in range(4)])
    result = KinematicTrajectoryOptimization(initial)
    result.AddPositionBounds([-3., -3.], [3., 3.])
    result.AddDurationConstraint(2., 2.)
    result.AddPathLengthCost()
    return result


def binding_signature(program, bindings):
    records = []
    for binding in bindings:
        q = program.GetInitialGuess(binding.variables())
        y = binding.evaluator().Eval(InitializeAutoDiff(q))
        records.append(dict(variables=[int(v.get_id()) for v in binding.variables()],
                            lower=binding.evaluator().lower_bound().tolist(),
                            upper=binding.evaluator().upper_bound().tolist(),
                            value=ExtractValue(y).tolist(), gradient=ExtractGradient(y).tolist()))
    return records


class FakeResult:
    def __init__(self, x): self.x = x.copy()
    def GetSolution(self, variables): return self.x.copy()
    def is_success(self): return False
    def get_solution_result(self): return 'intentional_unsuccessful_result'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    report = dict(passed=False, scope='Real Drake bindings with injected solver behavior; no GPU and no optimizer Solve',
                  source_sha256=sha(Path(module.__file__)), checks=[])
    try:
        clock = [100.]
        state = dict(guarded=False, restarts=0, solver_invocations=0)
        backend = Backend(out); traj = trajectory(); prog = traj.get_mutable_prog()
        adapter = module.NativeFirstCollisionSolve(traj, backend, [.2, .4, .85], state)
        variables = prog.decision_variables(); initial = prog.GetInitialGuess(variables).copy()
        signature = binding_signature(prog, adapter.bindings)
        assert backend.guarded_evaluations == 0
        count, cost_count = len(prog.GetAllConstraints()), len(prog.GetAllCosts())
        log = out / 'ipopt.log'; calls = []

        class RecoveringSolver:
            def solver_id(self): return SolverId('independent_cpu_probe')
            def Solve(self, supplied, guess, options):
                assert supplied is prog
                calls.append(clock[0])
                np.testing.assert_array_equal(prog.GetInitialGuess(variables), initial)
                assert binding_signature(prog, adapter.bindings) == signature
                assert len(prog.GetAllConstraints()) == count and len(prog.GetAllCosts()) == cost_count
                if len(calls) == 1:
                    log.write_text('first native exception evidence\n')
                    prog.SetInitialGuess(variables, initial + 99.)
                    clock[0] += 3.
                    raise RuntimeError('BoxBoxGradient failed: X_WA * p_ACa - X_WB * p_BCb')
                log.write_text('second guarded invocation evidence\n')
                clock[0] += 2.
                return FakeResult(initial)

        with patch.object(module.time, 'perf_counter', lambda: clock[0]):
            returned = adapter.solve(RecoveringSolver(), 110., timeout_option='max_cpu_time', log_path=log)
        assert not returned.is_success() and len(calls) == 2
        assert state == dict(guarded=True, restarts=1, solver_invocations=2)
        assert [c['remaining_seconds'] for c in adapter.execution['solver_calls']] == [10., 7.]
        assert adapter.execution['removed_binding_counts'] == [1, 1, 1]
        assert log.read_text() == 'second guarded invocation evidence\n'
        assert (out/'ipopt.native_exception.log').read_text() == 'first native exception evidence\n'
        assert backend.guarded_evaluations > 0
        report['checks'].extend(['exact public-binding variables/bounds/values/AutoDiff preserved',
                                'all original bindings removed exactly once; other constraints/costs unchanged',
                                'complete decision-variable initial guess restored',
                                'first exception log retained separately',
                                'shared remaining budget 10 to 7 seconds',
                                'ordinary unsuccessful result returned unchanged'])

        next_backend = Backend(out); next_traj = trajectory()
        next_adapter = module.NativeFirstCollisionSolve(next_traj, next_backend, [.2, .4, .85], state)
        next_prog = next_traj.get_mutable_prog(); next_initial = next_prog.GetInitialGuess(next_prog.decision_variables())
        binding_signature(next_prog, next_adapter.bindings)
        assert next_backend.guarded_evaluations == 3
        class LaterRoundSolver:
            def solver_id(self): return SolverId('independent_cpu_probe')
            def Solve(self, supplied, guess, options): return FakeResult(next_initial)
        with patch.object(module.time, 'perf_counter', lambda: clock[0]):
            next_adapter.solve(LaterRoundSolver(), 110., timeout_option='max_cpu_time')
        assert state == dict(guarded=True, restarts=1, solver_invocations=3)
        assert len(next_adapter.execution['solver_calls']) == 1
        assert next_adapter.execution['guarded_mode_inherited']
        report['checks'].append('next refinement round inherits guarded bindings without another retry')
        report.update(passed=True, final_query_state=state,
                      first_round_execution=adapter.execution,
                      second_round_execution=next_adapter.execution,
                      native_callbacks_before_recovery=0, actual_optimizer_invocations=0, gpu_calls=0)
    except BaseException as error:
        report.update(error=repr(error), traceback=traceback.format_exc())
        raise
    finally:
        def convert(value):
            if isinstance(value, np.ndarray): return value.tolist()
            if isinstance(value, np.generic): return value.item()
            raise TypeError(type(value).__name__)
        (out/'review.json').write_text(json.dumps(report, indent=2, default=convert)+'\n')
        print(json.dumps({'passed':report['passed'], 'checks':len(report['checks']),
                          'output':str(out/'review.json')}))


if __name__ == '__main__':
    main()
