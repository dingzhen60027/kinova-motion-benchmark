"""Budget, binding and error-identity regressions; no GPU calls."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from benchmarks.generic_motion_v1.runners import nto_collision_constraint as module

KNOWN = "Failure in BoxBoxGradient(): condition '(X_WA * p_ACa - X_WB * p_BCb).norm() < kEps' failed."


class Program:
    def __init__(self):
        self.initial = np.array([1., 2., 3.])
        self.constraints = [object(), object()]  # Existing endpoint constraints.
        self.options = []
        self.on_remove = lambda: None

    def decision_variables(self): return np.arange(3)
    def GetInitialGuess(self, variables): return self.initial.copy()
    def SetInitialGuess(self, variables, value): self.initial = value.copy()
    def GetAllConstraints(self): return list(self.constraints)
    def SetSolverOption(self, solver, key, value): self.options.append((key, value))
    def RemoveConstraint(self, binding):
        self.on_remove()
        if binding not in self.constraints: return 0
        self.constraints.remove(binding)
        return 1


class Trajectory:
    def __init__(self): self.program, self.added = Program(), []
    def get_mutable_prog(self): return self.program
    def AddPathPositionConstraint(self, evaluator, location):
        binding = SimpleNamespace(evaluator=evaluator, location=location)
        self.program.constraints.append(binding)
        self.added.append(binding)
        return binding


class Backend:
    def __init__(self, out):
        self.original, self.constraint = object(), object()
        self.out = out / "nto_collision_backend.json"
    def save(self):
        module.write(self.out, {"execution": self.execution})


class Result:
    def __init__(self, initial, success=True): self.initial, self.success = initial, success
    def GetSolution(self, variables): return self.initial.copy()
    def is_success(self): return self.success
    def get_solution_result(self): return "success" if self.success else "failure"


class Solver:
    def __init__(self, functions): self.functions, self.calls = functions, []
    def solver_id(self): return "test"
    def Solve(self, program, initial, options):
        self.calls.append((program, initial, options))
        return self.functions[len(self.calls)-1](program)


def fixture(tmp_path, monkeypatch, state=None):
    clock = [0.]
    monkeypatch.setattr(module, "time", SimpleNamespace(perf_counter=lambda: clock[0]))
    trajectory, backend = Trajectory(), Backend(tmp_path)
    helper = module.NativeFirstCollisionSolve(trajectory, backend, [.25, .75], state)
    return clock, trajectory, backend, helper


@pytest.mark.parametrize("success", [True, False])
def test_normal_result_uses_only_native_bindings_and_never_retries(tmp_path, monkeypatch, success):
    clock, trajectory, backend, helper = fixture(tmp_path, monkeypatch)
    def run(program):
        assert all(b.evaluator is backend.original for b in trajectory.added)
        return Result(program.initial, success)
    solver = Solver([run])
    result = helper.solve(solver, 10., timeout_option="Time limit")
    assert result.success is success and len(solver.calls) == 1
    assert helper.execution["solver_calls"][0]["mode"] == "native"
    assert not helper.execution["guarded_retry_started"]
    assert len(trajectory.program.constraints) == 4


def test_nonfinite_solver_failure_is_preserved_without_retry(tmp_path, monkeypatch):
    clock, trajectory, backend, helper = fixture(tmp_path, monkeypatch)
    result = Result(np.full(3, np.nan), False)
    solver = Solver([lambda program: result])
    assert helper.solve(solver, 10., timeout_option="Time limit") is result
    assert len(solver.calls) == 1
    assert not helper.execution["solver_calls"][0]["decision_variables_finite"]
    assert np.isnan(np.load(tmp_path / "nto_nonfinite_solver_result.npz")["decision_variables"]).all()


@pytest.mark.parametrize("error", [RuntimeError(KNOWN), SystemExit(KNOWN)])
def test_known_assertion_restores_all_bindings_initial_and_remaining_budget(tmp_path, monkeypatch, error):
    clock, trajectory, backend, helper = fixture(tmp_path, monkeypatch)
    original = trajectory.program.initial.copy()
    log = tmp_path / "native.log"
    trajectory.program.on_remove = lambda: clock.__setitem__(0, clock[0] + .5)
    def first(program):
        log.write_text("original native exception evidence")
        program.initial[:] = 999.  # Recovery must restore the entire original guess.
        clock[0] = 3.
        raise error
    def second(program):
        assert len(program.constraints) == 4
        assert [b.location for b in trajectory.added[2:]] == [.25, .75]
        assert all(b.evaluator is backend.constraint for b in trajectory.added[2:])
        np.testing.assert_array_equal(program.initial, original)
        log.write_text("guarded solver output")
        return Result(program.initial)
    solver = Solver([first, second])
    helper.solve(solver, 10., timeout_option="Time limit", log_path=log)
    assert trajectory.program.options == [("Time limit", 10.), ("Time limit", 6.)]
    assert (tmp_path / "native.native_exception.log").read_text() == "original native exception evidence"
    assert log.read_text() == "guarded solver output"
    assert helper.execution["removed_binding_counts"] == [1, 1]
    assert helper.recovery_state == dict(guarded=True, restarts=1, solver_invocations=2)


@pytest.mark.parametrize("error", [RuntimeError("different geometry failure"), SystemExit("stop"),
                                   ValueError(KNOWN), KeyboardInterrupt()])
def test_unknown_errors_are_not_retried_or_swallowed(tmp_path, monkeypatch, error):
    clock, trajectory, backend, helper = fixture(tmp_path, monkeypatch)
    def fail(program): raise error
    solver = Solver([fail])
    with pytest.raises(type(error)):
        helper.solve(solver, 10., timeout_option="Time limit")
    assert len(solver.calls) == 1 and len(trajectory.added) == 2
    assert not helper.execution["solver_calls"][0]["known_boxbox_assertion"]


def test_guarded_failure_never_triggers_a_third_solve(tmp_path, monkeypatch):
    clock, trajectory, backend, helper = fixture(tmp_path, monkeypatch)
    def fail(program): raise RuntimeError(KNOWN)
    solver = Solver([fail, fail])
    with pytest.raises(RuntimeError, match="BoxBoxGradient"):
        helper.solve(solver, 10., timeout_option="Time limit")
    assert len(solver.calls) == 2
    assert [x["status"] for x in helper.execution["solver_calls"]] == ["exception", "exception"]


def test_initial_native_error_consumes_deadline_without_replacement(tmp_path, monkeypatch):
    clock, trajectory, backend, helper = fixture(tmp_path, monkeypatch)
    def fail(program):
        clock[0] = 11.
        raise RuntimeError(KNOWN)
    solver = Solver([fail])
    with pytest.raises(TimeoutError): helper.solve(solver, 10., timeout_option="Time limit")
    assert len(solver.calls) == 1 and len(trajectory.added) == 2


def test_binding_replacement_cost_is_also_inside_deadline(tmp_path, monkeypatch):
    clock, trajectory, backend, helper = fixture(tmp_path, monkeypatch)
    trajectory.program.on_remove = lambda: clock.__setitem__(0, clock[0] + 6.)
    def fail(program): raise RuntimeError(KNOWN)
    solver = Solver([fail])
    with pytest.raises(TimeoutError): helper.solve(solver, 10., timeout_option="Time limit")
    assert len(solver.calls) == 1 and helper.recovery_state["solver_invocations"] == 1
    assert helper.recovery_state["restarts"] == 0


def test_missing_binding_fails_closed_without_second_solver(tmp_path, monkeypatch):
    clock, trajectory, backend, helper = fixture(tmp_path, monkeypatch)
    trajectory.program.constraints.remove(helper.bindings[0])
    def fail(program): raise RuntimeError(KNOWN)
    solver = Solver([fail])
    with pytest.raises(RuntimeError, match="every original binding"):
        helper.solve(solver, 10., timeout_option="Time limit")
    assert len(solver.calls) == 1


def test_query_shared_recovery_is_not_repeated_in_later_round(tmp_path, monkeypatch):
    state = dict(guarded=False, restarts=0, solver_invocations=0)
    clock, trajectory, backend, helper = fixture(tmp_path, monkeypatch, state)
    def fail(program): raise RuntimeError(KNOWN)
    helper.solve(Solver([fail, lambda p: Result(p.initial)]), 10., timeout_option="Time limit")
    later = tmp_path / "later"; later.mkdir()
    clock, trajectory, backend, helper = fixture(later, monkeypatch, state)
    assert all(b.evaluator is backend.constraint for b in trajectory.added)
    solver = Solver([lambda p: Result(p.initial)])
    helper.solve(solver, 10., timeout_option="Time limit")
    assert len(solver.calls) == 1 and helper.execution["guarded_mode_inherited"]
    assert state == dict(guarded=True, restarts=1, solver_invocations=3)


def test_nonfinite_initial_guess_rejected_before_any_solver(tmp_path, monkeypatch):
    clock, trajectory, backend, helper = fixture(tmp_path, monkeypatch)
    trajectory.program.initial[0] = np.nan
    solver = Solver([])
    with pytest.raises(ValueError, match="must be finite"):
        helper.solve(solver, 10., timeout_option="Time limit")
    assert not solver.calls
