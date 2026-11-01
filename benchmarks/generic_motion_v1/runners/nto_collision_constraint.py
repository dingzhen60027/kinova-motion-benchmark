"""Narrow workaround for Drake issue 21675, preserving canonical constraints.

Ordinary evaluations use the unmodified native minimum-distance constraint,
including its AutoDiff gradient. Only the known BoxBoxGradient witness-point
assertion is recovered: all canonical pairs are retained, the failing boxes
are evaluated with complete OBB feature distances / separating axes, and the
same Drake MinimumValueLowerBoundConstraint aggregates the distances. No point
is assumed free and no constraint is dropped. This is a numerical adapter,
not an upstream Drake patch or a continuous-collision certificate.
"""
from __future__ import annotations

import itertools
from pathlib import Path
import time
import traceback

import numpy as np

from ..roadmap import write

ISSUE = "https://github.com/RobotLocomotion/drake/issues/21675"
UPSTREAM_PR = "https://github.com/RobotLocomotion/drake/pull/25144"
CONTACT_RECOVERY_LIMIT_M = 1e-9
DIFFERENCE_STEP_RAD = 1e-7


def is_known_boxbox_error(error):
    message = str(error)
    return ("BoxBoxGradient" in message and "failed" in message
            and "X_WA * p_ACa - X_WB * p_BCb" in message)


def _segment_distance(a, b, c, d):
    """Euclidean distance between two closed segments, including parallel ones."""
    u, v, w = b-a, d-c, a-c
    aa, bb, cc = float(u@u), float(u@v), float(v@v)
    dd, ee = float(u@w), float(v@w)
    denominator = aa*cc-bb*bb
    # Parallel edge pairs attain their minimum at an endpoint of at least one
    # edge. The endpoint-to-segment candidates are included in every case.
    candidates = []
    for point, start, direction, norm in ((a,c,v,cc), (b,c,v,cc),
                                        (c,a,u,aa), (d,a,u,aa)):
        t = np.clip((point-start)@direction/norm, 0., 1.)
        candidates.append(float(np.linalg.norm(point-start-t*direction)))
    if denominator > np.finfo(float).eps * aa*cc:
        s, t = (bb*ee-cc*dd)/denominator, (aa*ee-bb*dd)/denominator
        if 0. <= s <= 1. and 0. <= t <= 1.:
            candidates.append(float(np.linalg.norm(w+s*u-t*v)))
    return min(candidates)


def box_box_signed_distance(size_a, transform_a, size_b, transform_b):
    """Signed translational separation of two unchanged oriented boxes.

    Overlap uses all 15 OBB separating axes. Separation uses the complete
    closest-feature set: vertices-to-box and every edge-edge pair. SAT alone
    would underestimate the Euclidean distance of diagonally separated boxes.
    """
    size_a, size_b = np.asarray(size_a, float), np.asarray(size_b, float)
    xa, xb = np.asarray(transform_a, float), np.asarray(transform_b, float)
    if (size_a.shape != (3,) or size_b.shape != (3,) or xa.shape != (4,4)
            or xb.shape != (4,4) or not all(np.isfinite(x).all() for x in (size_a,size_b,xa,xb))
            or np.any(size_a <= 0) or np.any(size_b <= 0)):
        raise ValueError("OBB distance requires positive finite boxes and transforms")
    ra, rb, ca, cb = xa[:3,:3], xb[:3,:3], xa[:3,3], xb[:3,3]
    if (not np.allclose(ra.T@ra, np.eye(3), atol=1e-12, rtol=0)
            or not np.allclose(rb.T@rb, np.eye(3), atol=1e-12, rtol=0)):
        raise ValueError("OBB transforms must contain rotation matrices")
    ha, hb = size_a/2, size_b/2
    axes = list(ra.T)+list(rb.T)+[np.cross(a,b) for a in ra.T for b in rb.T]
    gaps = []
    for axis in axes:
        norm = np.linalg.norm(axis)
        if norm <= 1e-14:
            continue
        axis = axis/norm
        gaps.append(abs((cb-ca)@axis)-ha@np.abs(ra.T@axis)-hb@np.abs(rb.T@axis))
    maximum_gap = float(max(gaps))
    if maximum_gap <= 0:
        return maximum_gap
    signs = np.array(list(itertools.product((-1.,1.), repeat=3)))
    vertices_a, vertices_b = ca+(signs*ha)@ra.T, cb+(signs*hb)@rb.T
    a_in_b, b_in_a = (vertices_a-cb)@rb, (vertices_b-ca)@ra
    distance = min(np.linalg.norm(a_in_b-np.clip(a_in_b,-hb,hb),axis=1).min(),
                   np.linalg.norm(b_in_a-np.clip(b_in_a,-ha,ha),axis=1).min())
    edges = [(i,j) for i in range(8) for j in range(i+1,8)
             if np.count_nonzero(signs[i] != signs[j]) == 1]
    for i,j in edges:
        for k,l in edges:
            distance = min(distance, _segment_distance(vertices_a[i],vertices_a[j],vertices_b[k],vertices_b[l]))
    return float(distance)


class RobustMinimumDistanceConstraint:
    """Native fast path with an auditable, narrow same-geometry recovery."""

    def __init__(self, native, clearance, influence_distance, out=None):
        from pydrake.all import MathematicalProgram, MinimumDistanceLowerBoundConstraint
        self.native = native
        self.num_positions = native.plant.num_positions()
        self.clearance, self.influence_distance = float(clearance), float(influence_distance)
        self.out = None if out is None else Path(out)/"nto_collision_backend.json"
        self.original = MinimumDistanceLowerBoundConstraint(
            native.plant, self.clearance, native.plant_context,
            influence_distance_offset=self.influence_distance)
        self.pairs = tuple(native.inspector.GetCollisionCandidates())
        self.events = []
        self.native_evaluations = 0
        # KTO accepts the concrete PyFunctionConstraint returned by this public
        # MathematicalProgram API. The callback retains this adapter's owner.
        self._program = MathematicalProgram()
        x = self._program.NewContinuousVariables(self.num_positions)
        self.constraint = self._program.AddConstraint(
            self.evaluate, self.original.lower_bound(), self.original.upper_bound(), x).evaluator()
        self.save()

    def save(self):
        record = dict(backend="native_Drake_with_issue21675_near_contact_recovery",
                      upstream_issue=ISSUE, upstream_candidate_fix=UPSTREAM_PR,
                      official_patch_installed=False, model_changed=False,
                      collision_pairs=len(self.pairs), collision_pairs_dropped=0,
                      native_evaluations=self.native_evaluations,
                      recovery_events=self.events,
                      maximum_recoverable_box_distance_m=CONTACT_RECOVERY_LIMIT_M,
                      gradient_difference_step_rad=DIFFERENCE_STEP_RAD,
                      aggregation="unchanged native MinimumValueLowerBoundConstraint",
                      ordinary_values_and_gradients="unmodified native MinimumDistanceLowerBoundConstraint")
        if hasattr(self, "execution"):
            record["execution"] = self.execution
            record["native_evaluations_scope"] = (
                "Guarded Python callback only; direct native solver evaluations are not instrumented")
        if self.out is not None:
            write(self.out, record)
        return record

    def _distances(self, q):
        from pydrake.all import Box
        self.native.plant.SetPositions(self.native.plant_context,q)
        query = self.native.scene_graph.get_query_output_port().Eval(self.native.scene_context)
        values, recovered = [], []
        for a,b in self.pairs:
            try:
                distance = float(query.ComputeSignedDistancePairClosestPoints(a,b).distance)
            except (RuntimeError, SystemExit) as exc:
                if not is_known_boxbox_error(exc):
                    raise
                sa,sb = self.native.inspector.GetShape(a),self.native.inspector.GetShape(b)
                if not isinstance(sa,Box) or not isinstance(sb,Box):
                    raise RuntimeError("BoxBoxGradient recovery encountered non-box geometry") from exc
                distance = box_box_signed_distance(sa.size(),query.GetPoseInWorld(a).GetAsMatrix4(),
                                                   sb.size(),query.GetPoseInWorld(b).GetAsMatrix4())
                if abs(distance) > CONTACT_RECOVERY_LIMIT_M:
                    raise RuntimeError("BoxBoxGradient recovery rejected a non-contact disagreement") from exc
                recovered.append(dict(pair=[self.native.inspector.GetName(a),self.native.inspector.GetName(b)],
                                      signed_distance_m=distance,exception=str(exc)))
            if not np.isfinite(distance):
                raise ValueError("Nonfinite canonical distance in BoxBoxGradient recovery")
            values.append(distance)
        return np.asarray(values), recovered

    def _fallback_scalar(self, q):
        from pydrake.all import MinimumValueLowerBoundConstraint
        values, pairs = self._distances(q)
        # Reuse Drake's exact padding, penalty and smooth maximum. Returning
        # every canonical pair retains the same max_num_values normalization.
        aggregate = MinimumValueLowerBoundConstraint(
            1,self.clearance,self.influence_distance,len(self.pairs),
            lambda x, influence: values, lambda x, influence: values)
        return float(aggregate.Eval(np.zeros(1))[0]), pairs

    def _scalar(self, q):
        try:
            return float(self.original.Eval(q)[0])
        except (RuntimeError, SystemExit) as exc:
            if not is_known_boxbox_error(exc):
                raise
            return self._fallback_scalar(q)[0]

    def evaluate(self, q):
        from pydrake.autodiffutils import AutoDiffXd, ExtractGradient, ExtractValue
        q = np.asarray(q)
        if q.shape not in ((self.num_positions,), (self.num_positions, 1)):
            raise ValueError("Collision constraint expects a joint vector or column vector")
        q = q.reshape(-1)
        autodiff = any(isinstance(x,AutoDiffXd) for x in q)
        values = np.asarray(ExtractValue(q) if autodiff else q,float).ravel()
        if not np.isfinite(values).all():
            raise ValueError("Collision constraint joint positions must be finite")
        if autodiff and not np.isfinite(ExtractGradient(q)).all():
            raise ValueError("Collision constraint AutoDiff derivatives must be finite")
        self.native_evaluations += 1
        try:
            return self.original.Eval(q)
        except (RuntimeError, SystemExit) as exc:
            if not is_known_boxbox_error(exc):
                raise
            error = str(exc)
        began = time.perf_counter()
        event = dict(q=values, exception=error, used_autodiff=autodiff)
        try:
            value, pairs = self._fallback_scalar(values)
            event.update(recovered_pairs=pairs, constraint_value=value)
            if not pairs:
                raise RuntimeError("Known BoxBoxGradient error could not be reproduced per canonical pair")
            if autodiff:
                gradients = []
                for step in (DIFFERENCE_STEP_RAD, DIFFERENCE_STEP_RAD/2):
                    gradient = np.empty(len(values))
                    for i in range(len(values)):
                        delta = np.zeros(len(values));delta[i]=step
                        gradient[i]=(self._scalar(values+delta)-self._scalar(values-delta))/(2*step)
                    gradients.append(gradient)
                if not np.isfinite(gradients).all():
                    raise ValueError("Nonfinite gradient in BoxBoxGradient recovery")
                difference=float(np.max(np.abs(gradients[0]-gradients[1])))
                event.update(gradient=gradients[1], gradient_step_consistency_max_error=difference)
                if difference > 1e-3*max(1.,np.max(np.abs(gradients[1]))):
                    raise RuntimeError("BoxBoxGradient recovery finite-difference gradient did not converge")
                return np.asarray([AutoDiffXd(value,gradients[1]@ExtractGradient(q))])
            return np.asarray([value])
        except (Exception, SystemExit) as exc:
            event["recovery_failure"] = repr(exc)
            raise
        finally:
            # Restore the exact requested context after derivative probes.
            self.native.plant.SetPositions(self.native.plant_context,values)
            event["seconds"] = time.perf_counter()-began
            self.events.append(event)
            self.save()


class NativeFirstCollisionSolve:
    """Use native bindings; retry only a known Drake assertion, once in-budget.

    The guarded evaluator remains unchanged. Switching evaluators preserves
    every collocation location and the complete original decision-variable
    initial guess. It does not retry ordinary unsuccessful solver returns.
    """

    def __init__(self, trajectory, backend, locations, recovery_state=None):
        self.trajectory, self.backend = trajectory, backend
        self.program = trajectory.get_mutable_prog()
        self.recovery_state = (dict(guarded=False, restarts=0, solver_invocations=0)
                               if recovery_state is None else recovery_state)
        self.inherited_guarded = bool(self.recovery_state["guarded"])
        self.locations = tuple(float(s) for s in locations)
        if (not self.locations or not np.isfinite(self.locations).all()
                or any(s <= 0 or s >= 1 for s in self.locations)
                or any(a >= b for a, b in zip(self.locations, self.locations[1:]))):
            raise ValueError("Native collision locations must be ordered distinct interior parameters")
        evaluator = backend.constraint if self.inherited_guarded else backend.original
        self.bindings = [trajectory.AddPathPositionConstraint(evaluator, s)
                         for s in self.locations]
        self.execution = dict(policy="native_first_exact_BoxBoxGradient_one_guarded_retry",
                              solver_calls=[], guarded_retry_started=False,
                              collision_bindings=len(self.bindings),
                              collocation_locations=self.locations,
                              guarded_mode_inherited=self.inherited_guarded,
                              original_initial_guess_restored=False)
        backend.execution = self.execution
        backend.save()

    def solve(self, solver, deadline, *, timeout_option, log_path=None, solver_options=None):
        """Both native calls and recovery share this absolute deadline."""
        if not np.isfinite(deadline):
            raise ValueError("Native collision solve requires a finite deadline")
        variables = self.program.decision_variables()
        initial = np.asarray(self.program.GetInitialGuess(variables), float).copy()
        if not np.isfinite(initial).all():
            raise ValueError("Native collision solve initial decision variables must be finite")
        original_constraint_count = len(self.program.GetAllConstraints())
        self.execution.update(deadline_monotonic_seconds=float(deadline),
                              initial_decision_variables=initial,
                              initial_constraint_count=original_constraint_count)
        self.backend.save()
        for guarded in ((True,) if self.inherited_guarded else (False, True)):
            remaining = float(deadline) - time.perf_counter()
            if remaining <= 0:
                self.execution["budget_exhausted_before"] = "guarded_retry" if guarded else "native_solve"
                self.backend.save()
                raise TimeoutError("Native collision solve exhausted its shared deadline")
            self.program.SetSolverOption(solver.solver_id(), timeout_option, remaining)
            entry = dict(mode="guarded" if guarded else "native",
                         remaining_seconds=remaining, time_limit_option=timeout_option,
                         status="running", log_path=None if log_path is None else str(log_path))
            self.execution["solver_calls"].append(entry)
            self.recovery_state["solver_invocations"] += 1
            self.execution["query_solver_invocations"] = self.recovery_state["solver_invocations"]
            if guarded and not self.inherited_guarded:
                self.execution["guarded_retry_started"] = True
                self.recovery_state["restarts"] += 1
                self.execution["query_recovery_restarts"] = self.recovery_state["restarts"]
            self.backend.save()
            began = time.perf_counter()
            try:
                result = solver.Solve(self.program, None, solver_options)
                values = np.asarray(result.GetSolution(variables), float)
                finite = bool(np.isfinite(values).all())
                if not finite:
                    if self.backend.out is not None:
                        np.savez_compressed(self.backend.out.parent / "nto_nonfinite_solver_result.npz",
                                            decision_variables=values)
                entry.update(status="returned", solver_success=bool(result.is_success()),
                             solver_status=str(result.get_solution_result()),
                             decision_variables_finite=finite)
                # Preserve ordinary solver-failure returns, including their
                # raw variables/details. The existing candidate serializer and
                # strict common validator reject nonfinite trajectories.
                return result
            except BaseException as exc:
                # Python Drake can map the upstream assertion to SystemExit;
                # solver callbacks may instead wrap it as RuntimeError.
                known = isinstance(exc, (RuntimeError, SystemExit)) and is_known_boxbox_error(exc)
                entry.update(status="exception", exception=repr(exc),
                             traceback=traceback.format_exc(), known_boxbox_assertion=known)
                if guarded or not known:
                    raise
                if log_path is not None:
                    source = Path(log_path)
                    if source.exists():
                        retained = source.with_name(source.stem + ".native_exception" + source.suffix)
                        if retained.exists():
                            raise FileExistsError("Refusing to overwrite the first native solver failure log")
                        source.rename(retained)
                        entry["retained_log_path"] = str(retained)
                        entry["log_path"] = str(retained)
            finally:
                entry["wall_seconds"] = time.perf_counter() - began
                self.backend.save()
            # Only the first precisely matched exception reaches this block.
            # If work consumed the budget, preserve the error without changing
            # any bindings or starting a further solver invocation.
            if time.perf_counter() >= deadline:
                self.execution["budget_exhausted_before"] = "guarded_binding_replacement"
                self.backend.save()
                raise TimeoutError("Native assertion recovery exhausted its shared deadline")
            removed = [self.program.RemoveConstraint(binding) for binding in self.bindings]
            self.execution["removed_binding_counts"] = removed
            if any(count != 1 for count in removed):
                self.backend.save()
                raise RuntimeError("Native collision recovery could not remove every original binding exactly once")
            self.bindings = [self.trajectory.AddPathPositionConstraint(self.backend.constraint, s)
                             for s in self.locations]
            if len(self.program.GetAllConstraints()) != original_constraint_count:
                raise RuntimeError("Native collision recovery changed the number of constraints")
            self.program.SetInitialGuess(variables, initial)
            restored = np.asarray(self.program.GetInitialGuess(variables), float)
            if not np.array_equal(restored, initial):
                raise RuntimeError("Native collision recovery did not preserve the original initial guess")
            self.execution["original_initial_guess_restored"] = True
            self.recovery_state["guarded"] = True
            self.backend.save()
        raise AssertionError("Native collision recovery exceeded one retry")
