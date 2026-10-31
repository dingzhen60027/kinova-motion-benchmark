"""Explicit SCIP backend for the unchanged projected GCS MISOCP.

The binary flow, homogeneous incident-set rows, vector-flow equality and
Euclidean norm epigraph are identical to optimization.lgcs. SCIP solves the
unsquared norm expression with its nonlinear/SOC handlers and global bounds;
there is no relaxation-only or heuristic fallback. Optimality concerns this
supplied local region graph, not the full collision-free configuration space.
"""
from __future__ import annotations

import hashlib
import importlib
from pathlib import Path
import sys
import time

import numpy as np

from ..roadmap import write


SCIP_PARAMETERS = {
    'numerics/feastol': 1e-9,
    'numerics/dualfeastol': 1e-9,
    'limits/gap': 1e-6,
    'parallel/maxnthreads': 1,
    'lp/threads': 1,
    'randomization/randomseedshift': 0,
}
RESIDUAL_TOLERANCE = 1e-7
SCIP_TARGET = Path(__file__).resolve().parents[3] / '.local_setup/solver_probes/pyscipopt'
EXPECTED_PYSCIPOPT_VERSION = '6.3.0'


def dependency_files():
    """Exact provisioned package files to bind in each run's dependency seal."""
    if not SCIP_TARGET.is_dir():
        raise FileNotFoundError(f'Missing provisioned SCIP target: {SCIP_TARGET}')
    return sorted(path.resolve() for path in SCIP_TARGET.rglob('*')
                  if path.is_file() and '__pycache__' not in path.parts
                  and path.suffix != '.pyc')


def dependency_receipt():
    return {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in dependency_files()}


def _load_scip(out):
    """Use only the explicitly provisioned, version-checked local target.

    Nothing is installed here. The isolated target avoids modifying the user's
    Conda environment; missing/broken dependencies are reported as unsupported.
    Solver selection itself is explicit and never falls back to another solver.
    """
    from .paper_lgcs import LGCSUnsupported

    target = SCIP_TARGET
    try:
        if not target.is_dir():
            raise ImportError(f'Missing provisioned SCIP target: {target}')
        if str(target) not in sys.path:
            sys.path.insert(0, str(target))
        scip = importlib.import_module('pyscipopt')
        if not Path(scip.__file__).resolve().is_relative_to(target.resolve()):
            raise ImportError(f'SCIP package is not from the explicit provisioned target: {scip.__file__}')
        if scip.__version__ != EXPECTED_PYSCIPOPT_VERSION:
            raise ImportError(f'Unexpected PySCIPOpt version: {scip.__version__}')
        return scip
    except (ImportError, OSError) as error:
        record = dict(status='unsupported', solver='SCIP', solver_success=False,
                      backend='scip', error=repr(error), isolated_target=str(target))
        if out is not None:
            write(out / 'lgcs_failure.json', record)
        raise LGCSUnsupported(str(error), record) from error


def _metadata(scip, model):
    return dict(solver='SCIP', backend='scip', pyscipopt_version=scip.__version__,
                solver_version=[model.getMajorVersion(), model.getMinorVersion(), model.getTechVersion()],
                pyscipopt_module=str(Path(scip.__file__).resolve()),
                solver_license='SCIP >=8.0.3 Apache-2.0; PySCIPOpt MIT; bundled libraries retain their licenses',
                solver_license_source='https://www.scipopt.org/')


def dependency_probe(out=None):
    """Actually solve a tiny binary SOC program using this installed backend."""
    out = None if out is None else Path(out)
    if out is not None:
        out.mkdir(parents=True, exist_ok=True)
    scip = _load_scip(out)
    started = time.perf_counter()
    model = scip.Model('LGCS_SCIP_binary_SOC_dependency_probe')
    metadata = _metadata(scip, model)
    try:
        model.hideOutput()
        model.setParams(dict(SCIP_PARAMETERS, **{'limits/time': 5., 'limits/gap': 0.}))
        x = model.addVar('x', vtype='B')
        y = model.addVar('y', lb=None)
        t = model.addVar('t', lb=0.)
        model.addCons(x == 1)
        model.addCons(y == 2*x)
        model.addCons(scip.sqrt(y*y) <= t)
        model.setObjective(t, 'minimize')
        if out is not None:
            model.setLogfile(str(out / 'scip_dependency_probe.log'))
            model.writeProblem(str(out / 'scip_dependency_probe.cip'), verbose=False)
        model.optimize()
        sol = model.getBestSol()
        objective = float(model.getSolObjVal(sol)) if sol is not None else None
        ok = (str(model.getStatus()) == 'optimal' and objective is not None
              and abs(objective-2.) < RESIDUAL_TOLERANCE)
        metadata.update(status='available' if ok else 'solver_failed', solver_success=ok,
                        solver_status=str(model.getStatus()), objective=objective,
                        dual_bound=float(model.getDualbound()), gap=float(model.getGap()),
                        variables=3, constraints=3, probe='binary_SOC',
                        seconds=time.perf_counter()-started)
        if not ok:
            raise RuntimeError(f'SCIP dependency probe did not solve: {metadata}')
        return metadata
    except Exception as error:
        metadata.update(status='solver_failed', solver_success=False, error=repr(error))
        raise
    finally:
        metadata['seconds'] = time.perf_counter()-started
        if out is not None:
            write(out / 'lgcs_dependency_probe.json', metadata)
        model.freeProb()


def _positive_max(values):
    return max(0., max(values, default=0.))


def _residuals(edges, n, start, goal, flow, transitions, lengths, deltas, rows):
    """Independent NumPy check of the submitted, untransformed algebra."""
    d, source, target = len(start), n, n+1
    lookup = {edge: i for i, edge in enumerate(edges)}
    z = {e: transitions[i] for i, e in enumerate(e for e in edges if e[0] < n and e[1] < n)}
    def point(e):
        if e[0] == source:
            return start * flow[lookup[e]]
        if e[1] == target:
            return goal * flow[lookup[e]]
        return z[e]
    balance, degree, vector = [], [], []
    for i in range(n+2):
        incoming = [e for e in edges if e[1] == i]
        outgoing = [e for e in edges if e[0] == i]
        outflow = sum(flow[lookup[e]] for e in outgoing)
        inflow = sum(flow[lookup[e]] for e in incoming)
        balance.append(abs(outflow-inflow-(1 if i == source else -1 if i == target else 0)))
        if i < n:
            degree.append(outflow-1.)
            displacement = sum((point(e) for e in outgoing), np.zeros(d))-sum((point(e) for e in incoming), np.zeros(d))
            vector.append(float(np.max(np.abs(deltas[i]-displacement))))
    membership = [float(np.dot(a, point(e))-b*flow[lookup[e]]) for e, a, b in rows]
    result = dict(
        finite=bool(np.isfinite(flow).all() and np.isfinite(transitions).all()
                    and np.isfinite(lengths).all() and np.isfinite(deltas).all()),
        flow_integrality=float(np.max(np.abs(flow-np.round(flow)), initial=0)),
        flow_bounds=_positive_max(list(-flow)+list(flow-1)),
        flow_balance=_positive_max(balance), degree=_positive_max(degree),
        vector_flow=_positive_max(vector), homogeneous_membership=_positive_max(membership),
        length_nonnegative=_positive_max(list(-lengths)),
        soc_norm=_positive_max(list(np.linalg.norm(deltas, axis=1)-lengths)),
        tolerance=RESIDUAL_TOLERANCE)
    result['passed'] = result['finite'] and all(
        value <= RESIDUAL_TOLERANCE for key, value in result.items()
        if key not in ('finite', 'tolerance'))
    return result, z


def solve(regions, start, goal, *, time_limit, out=None):
    """Return a candidate and full solver evidence for explicit backend=scip."""
    # Public paper_lgcs.solve validates the endpoints and budget before dispatch.
    if isinstance(time_limit, bool) or not np.isfinite(time_limit) or time_limit <= 0:
        raise ValueError('LGCS needs a positive finite remaining solver budget')
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    if (start.ndim != 1 or not len(start) or goal.shape != start.shape
            or not np.isfinite([start, goal]).all()):
        raise ValueError('LGCS endpoints must be matching finite vectors')
    if not regions or any(r.ambient_dimension() != len(start) for r in regions):
        raise ValueError('LGCS regions must match the endpoint dimension')
    out = None if out is None else Path(out)
    if out is not None:
        out.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    scip = _load_scip(out)
    model = scip.Model('LGCS_GCS_Eq5_5_projected_SCIP')
    metadata = dict(_metadata(scip, model), method='LGCS', kind='polygon',
                    reproduction_status='adapted',
                    provenance='local_paper_GCS_MISOCP_reconstruction_with_SCIP',
                    formulation='GCS_Eq5.5_split_vertex_exact_projection',
                    numerical_variant='strict_scip_tolerances_v1',
                    optimality_scope='supplied local convex-region graph, not full collision-free C-space',
                    requested_time_limit_seconds=float(time_limit),
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        if any(not r.IsBounded() for r in regions):
            raise ValueError('Perspective projection requires bounded convex regions')
        model.hideOutput()
        if out is not None:
            model.setLogfile(str(out / 'scip.log'))
        model.setParams(SCIP_PARAMETERS)
        n, d = len(regions), len(start)
        source, target = n, n+1
        edges = []
        for i, region in enumerate(regions):
            for j in range(i+1, n):
                if region.IntersectsWith(regions[j]):
                    edges.extend([(i, j), (j, i)])
            if region.PointInSet(start, 1e-6):
                edges.append((source, i))
            if region.PointInSet(goal, 1e-6):
                edges.append((i, target))
        interior = [(u, v) for u, v in edges if u < n and v < n]
        flow = {e: model.addVar(f'phi_{e[0]}_{e[1]}', vtype='B') for e in edges}
        transition = {(u, v, k): model.addVar(f'z_{u}_{v}_{k}', lb=None)
                      for u, v in interior for k in range(d)}
        length = {i: model.addVar(f'length_{i}', lb=0.) for i in range(n)}
        delta = {(i, k): model.addVar(f'delta_{i}_{k}', lb=None) for i in range(n) for k in range(d)}
        def edge_point(u, v, k):
            if u == source:
                return float(start[k])*flow[u, v]
            if v == target:
                return float(goal[k])*flow[u, v]
            return transition[u, v, k]
        incoming = {i: [e for e in edges if e[1] == i] for i in range(n+2)}
        outgoing = {i: [e for e in edges if e[0] == i] for i in range(n+2)}
        for i in range(n+2):
            phi_out = scip.quicksum(flow[e] for e in outgoing[i])
            phi_in = scip.quicksum(flow[e] for e in incoming[i])
            model.addCons(phi_out-phi_in == (1 if i == source else -1 if i == target else 0), name=f'flow_{i}')
            if i < n:
                model.addCons(phi_out <= 1, name=f'degree_{i}')
                for k in range(d):
                    model.addCons(delta[i, k] == scip.quicksum(edge_point(u, v, k) for u, v in outgoing[i])
                                  - scip.quicksum(edge_point(u, v, k) for u, v in incoming[i]), name=f'vector_{i}_{k}')
                # Exact same SOC as sum(delta**2)<=length**2, length>=0.
                # Express the unsquared norm so SCIP checks residuals in
                # distance units even when a segment's length approaches zero.
                model.addCons(scip.sqrt(scip.quicksum(delta[i, k]**2 for k in range(d))) <= length[i], name=f'soc_{i}')
        rows = []
        for u, v in edges:
            unique = {}
            for i in (u, v):
                if i < n:
                    for a, b in zip(regions[i].A(), regions[i].b()):
                        key = tuple(float(x) for x in a)
                        unique[key] = min(unique.get(key, float('inf')), float(b))
            for a, b in unique.items():
                model.addCons(scip.quicksum(a[k]*edge_point(u, v, k) for k in range(d)) <= b*flow[u, v],
                              name=f'membership_{len(rows)}')
                rows.append(((u, v), a, b))
        model.setObjective(scip.quicksum(length.values()), 'minimize')
        metadata.update(graph_vertices=n, graph_edges=len(edges), split_graph_vertices=2*n+2,
                        split_graph_edges=n+len(edges), solver_variables=model.getNVars(False),
                        solver_constraints=model.getNConss(False)-n,
                        solver_quadratic_constraints=0, solver_nonlinear_constraints=n,
                        solver_soc_constraints=n,
                        constraint_representation='linear rows plus unsquared Euclidean norm epigraphs',
                        perspective_membership_rows=len(rows))
        if out is not None:
            model.writeProblem(str(out / 'model.cip'), verbose=False)
            np.savez_compressed(out / 'graph.npz', edges=np.asarray(edges, dtype=np.int64).reshape(-1, 2),
                                interior_edges=np.asarray(interior, dtype=np.int64).reshape(-1, 2),
                                start=start, goal=goal)
        remaining = float(time_limit)-(time.perf_counter()-started)
        metadata['model_construction_seconds'] = time.perf_counter()-started
        if remaining <= 0:
            raise TimeoutError('LGCS construction exhausted the remaining solver budget')
        model.setParam('limits/time', remaining)
        metadata['solver_parameters'] = dict(SCIP_PARAMETERS, **{'limits/time': remaining})
        if out is not None:
            model.writeParams(str(out / 'scip.set'), onlychanged=False, verbose=False)
        optimize_started = time.perf_counter()
        try:
            model.optimize()
        finally:
            metadata['solver_optimize_seconds'] = time.perf_counter()-optimize_started
        sol = model.getBestSol()
        status = str(model.getStatus())
        candidate = dict(metadata, objective='minimum_distance', solver_status=status,
                         solver_success=False, globally_optimal=False,
                         solver_has_incumbent=sol is not None, points=None,
                         solver_objective=float(model.getSolObjVal(sol)) if sol is not None else None,
                         solver_primal_bound=float(model.getPrimalbound()),
                         solver_dual_bound=float(model.getDualbound()),
                         gap=float(model.getGap()), solver_nodes=int(model.getNNodes()),
                         solver_reported_seconds=float(model.getSolvingTime()))
        if sol is not None:
            values = lambda variables: np.array([model.getSolVal(sol, v) for v in variables], float)
            flows = values(flow.values())
            transitions = values(transition.values()).reshape(len(interior), d)
            lengths = values(length.values())
            deltas = values(delta.values()).reshape(n, d)
            if out is not None:
                np.savez_compressed(out / 'solver_solution.npz', edges=np.asarray(edges, np.int64).reshape(-1, 2),
                                    interior_edges=np.asarray(interior, np.int64).reshape(-1, 2),
                                    flow=flows, transition=transitions, length=lengths, delta=deltas)
                model.writeSol(sol, str(out / 'solution.sol'), write_zeros=True)
            residual, z = _residuals(edges, n, start, goal, flows, transitions, lengths, deltas, rows)
            candidate['algebraic_validation'] = residual
            candidate['scip_original_solution_feasible'] = bool(model.checkSol(sol, printreason=True,
                completely=True, checkbounds=True, checkintegrality=True, checklprows=True, original=True))
            if residual['passed'] and candidate['scip_original_solution_feasible']:
                selected = {e for e, value in zip(edges, flows) if value > .5}
                selected_path, sequence, points, u = [], [], [start.copy()], source
                while u != target:
                    successors = [v for a, v in outgoing[u] if (a, v) in selected]
                    if len(successors) != 1:
                        raise ValueError('Invalid integer flow in SCIP GCS solution')
                    v = successors[0]
                    selected_path.append((u, v))
                    if u < n:
                        points.append(goal.copy() if v == target else z[u, v].copy())
                    u = v
                    if u < n:
                        if u in sequence:
                            raise ValueError('Cyclic source-to-target integer flow')
                        sequence.append(u)
                points = np.asarray(points)
                membership = _positive_max([float(np.max(regions[i].A()@point-regions[i].b(), initial=0))
                    for segment, i in enumerate(sequence) for point in points[segment:segment+2]])
                recovered_cost = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
                # Discarding disjoint cycles cannot increase path cost. A
                # positive cost discrepancy in the other direction is invalid.
                objective_excess = max(0., recovered_cost-candidate['solver_objective'])
                decoded_ok = membership <= RESIDUAL_TOLERANCE and objective_excess <= RESIDUAL_TOLERANCE
                candidate.update(points=points, segment_regions=sequence, selected_regions=sequence,
                                 discarded_disjoint_cycle_edges=[list(e) for e in sorted(selected-set(selected_path))],
                                 recovered_path_length=recovered_cost, decoded_membership_violation=membership,
                                 decoded_objective_excess=objective_excess,
                                 solver_success=bool(decoded_ok), globally_optimal=bool(decoded_ok and status == 'optimal'))
            if not candidate['solver_success']:
                candidate['candidate_rejection'] = 'Original algebra or decoded region path fails the explicit numerical gate'
        candidate['total_lgcs_seconds'] = time.perf_counter()-started
        if out is not None:
            write(out / 'lgcs_solver.json', {k: v for k, v in candidate.items() if not isinstance(v, np.ndarray)})
        return candidate
    except Exception as error:
        record = dict(metadata, status='solver_failed', solver_success=False, error=repr(error),
                      total_lgcs_seconds=time.perf_counter()-started)
        if out is not None:
            write(out / 'lgcs_failure.json', record)
        error.optimization_metadata = record
        raise
    finally:
        model.freeProb()
