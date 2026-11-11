"""LGCS adapter using the existing, inspectable perspective GCS formulation.

This is a local reconstruction of the paper's LGCS optimization, not released
author LGCS source. Native DRM and EI-ZO remain the shared front end. The Gurobi
license is used as installed: a size or license failure is never replaced by
LSCS or a continuous relaxation and reported as LGCS success.
"""
from __future__ import annotations

import builtins
import hashlib
from pathlib import Path
import time
from types import FunctionType, SimpleNamespace

import numpy as np

from ..roadmap import write


STRICT_PARAMETERS = dict(FeasibilityTol=1e-9, OptimalityTol=1e-9,
                         BarQCPConvTol=1e-9, IntFeasTol=1e-9)


class LGCSUnsupported(RuntimeError):
    """The installed solver/dependency cannot execute this exact program."""

    def __init__(self, message, metadata):
        super().__init__(message)
        self.optimization_metadata = metadata
        self.error_category = 'dependency_unsupported'
        self.dependency_unsupported = True


def _directory(out):
    if out is None:
        return None
    directory = Path(out)
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def _save(out, name, value):
    if out is not None:
        write(out / name, value)


def _gurobi_failure(error, metadata, out):
    number = int(getattr(error, 'errno', -1))
    record = dict(metadata, solver_success=False, error=repr(error),
                  solver_error_number=number)
    # Gurobi 10009: no license; 10010: restricted-license model size. Other
    # errors (e.g. a nonconvex model) remain genuine solver/program failures.
    unsupported = number in (10009, 10010)
    record['status'] = 'unsupported' if unsupported else 'solver_failed'
    _save(out, 'lgcs_failure.json', record)
    if unsupported:
        raise LGCSUnsupported(str(error), record) from error
    error.optimization_metadata = record
    raise error


def dependency_probe(out=None, *, backend='gurobi'):
    """Run a real one-variable Gurobi solve; availability alone is insufficient."""
    if backend == 'scip':
        from .paper_lgcs_scip import dependency_probe as probe
        return probe(out)
    if backend != 'gurobi':
        raise ValueError(f'Unknown explicit LGCS solver backend: {backend!r}')
    out = _directory(out)
    try:
        import gurobipy as gp
    except ImportError as error:
        record = dict(status='unsupported', error=repr(error),
                      solver='gurobipy', solver_success=False)
        _save(out, 'lgcs_dependency_probe.json', record)
        raise LGCSUnsupported(str(error), record) from error
    model = None
    started = time.perf_counter()
    metadata = dict(solver='gurobipy', version=list(gp.gurobi.version()),
                    license_scope='This small solve does not establish permission for a larger model')
    try:
        model = gp.Model('generic_motion_lgcs_dependency_probe')
        model.Params.OutputFlag = 0
        model.Params.Threads = 1
        model.Params.TimeLimit = 5
        x = model.addVar(lb=0)
        model.addConstr(x >= 1)
        model.setObjective(x)
        model.optimize()
        metadata.update(status='available' if model.Status == gp.GRB.OPTIMAL else 'solver_failed',
                        solver_success=bool(model.Status == gp.GRB.OPTIMAL),
                        solver_status=int(model.Status),
                        objective=float(model.ObjVal) if model.SolCount else None,
                        variables=int(model.NumVars), constraints=int(model.NumConstrs))
        if not metadata['solver_success']:
            raise RuntimeError(f'Gurobi dependency probe did not solve: {metadata}')
        return metadata
    except gp.GurobiError as error:
        metadata.update(status='unsupported' if int(error.errno) in (10009, 10010) else 'solver_failed',
                        solver_success=False, error=repr(error), solver_error_number=int(error.errno))
        _gurobi_failure(error, metadata, out)
    finally:
        metadata['seconds'] = time.perf_counter() - started
        _save(out, 'lgcs_dependency_probe.json', metadata)
        if model is not None:
            model.dispose()


def solve(regions, start, goal, *, time_limit, out=None, backend='gurobi'):
    """Solve LGCS and return a generic polygon candidate plus full pool indices.

    Pass ``[regions[i] for i in candidate['segment_regions']]`` to the common
    validator. Its collision segment numbers must be mapped through that same
    index list before editing the full region pool. The submitted MPS model and
    native solver log are retained when ``out`` is provided, including failures.
    """
    if (isinstance(time_limit, bool) or not np.isfinite(time_limit)
            or time_limit <= 0):
        raise ValueError('LGCS needs a positive finite remaining solver budget')
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    if (start.ndim != 1 or goal.shape != start.shape or not len(start)
            or not np.isfinite([start, goal]).all()):
        raise ValueError('LGCS endpoints must be matching finite vectors')
    if not regions or any(r.ambient_dimension() != len(start) for r in regions):
        raise ValueError('LGCS regions must match the endpoint dimension')
    if backend == 'scip':
        from .paper_lgcs_scip import solve as scip_solve
        return scip_solve(regions, start, goal, time_limit=time_limit, out=out)
    if backend != 'gurobi':
        raise ValueError(f'Unknown explicit LGCS solver backend: {backend!r}')
    out = _directory(out)
    try:
        import gurobipy as gp
    except ImportError as error:
        record = dict(status='unsupported', error=repr(error), solver='gurobipy')
        _save(out, 'lgcs_failure.json', record)
        raise LGCSUnsupported(str(error), record) from error
    from reproduction.paper_completion.optimization import lgcs

    parameters = dict(STRICT_PARAMETERS, TimeLimit=float(time_limit))
    models = []
    timings = {}

    class ModelProxy:
        def __init__(self, *args, **kwargs):
            self.model = gp.Model(*args, **kwargs)
            models.append(self.model)

        def __getattr__(self, name):
            return getattr(self.model, name)

        def optimize(self):
            # The historical implementation specifies 60 s during construction;
            # overwrite it at the actual solve with the worker's remaining time.
            for key, value in parameters.items():
                self.model.setParam(key, value)
            if out is not None:
                self.model.setParam('LogFile', str(out / 'gurobi.log'))
                self.model.setParam('OutputFlag', 1)
                self.model.setParam('LogToConsole', 0)
                self.model.write(str(out / 'model.mps.gz'))
            remaining = float(time_limit) - (time.perf_counter() - started)
            if remaining <= 0:
                raise TimeoutError('LGCS construction exhausted the remaining solver budget')
            parameters['TimeLimit'] = remaining
            self.model.setParam('TimeLimit', remaining)
            optimize_started = time.perf_counter()
            try:
                return self.model.optimize()
            finally:
                timings['solver_optimize_seconds'] = time.perf_counter() - optimize_started

    proxy = SimpleNamespace(**vars(gp))
    proxy.Model = ModelProxy
    real_import = builtins.__import__

    def local_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name == 'gurobipy' and level == 0:
            return proxy
        return real_import(name, globals, locals, fromlist, level)

    # Function-local injection preserves other running Python code and the
    # historical source module; no monkey patch of sys.modules is performed.
    namespace = dict(lgcs.__globals__)
    namespace['__builtins__'] = dict(vars(builtins), __import__=local_import)
    function = FunctionType(lgcs.__code__, namespace, lgcs.__name__,
                            lgcs.__defaults__, lgcs.__closure__)
    started = time.perf_counter()
    metadata = dict(method='LGCS', reproduction_status='adapted',
                    provenance='local_paper_GCS_MISOCP_reconstruction_with_Gurobi',
                    source_method='reproduction.paper_completion.optimization.lgcs',
                    source_sha256=hashlib.sha256(Path(lgcs.__code__.co_filename).read_bytes()).hexdigest(),
                    numerical_variant='strict_gurobi_tolerances_v1',
                    requested_time_limit_seconds=float(time_limit),
                    solver_version=list(gp.gurobi.version()))
    try:
        candidate = function(regions, start, goal)
        candidate['solver_parameters'].update(parameters)
        candidate.update(metadata, solver_objective=candidate.pop('objective'),
                         objective='minimum_distance')
        if models and models[-1].SolCount:
            candidate['solver_quality'] = {
                key: float(getattr(models[-1], key))
                for key in ('MaxVio', 'ConstrVio', 'IntVio', 'BoundVio')}
        candidate.update(timings, total_lgcs_seconds=time.perf_counter()-started)
        _save(out, 'lgcs_solver.json', {k: v for k, v in candidate.items()
                                      if not isinstance(v, np.ndarray)})
        return candidate
    except gp.GurobiError as error:
        metadata.update(getattr(error, 'optimization_metadata', {}))
        metadata.setdefault('solver_parameters', {}).update(parameters)
        metadata.update(timings, total_lgcs_seconds=time.perf_counter()-started)
        _gurobi_failure(error, metadata, out)
    finally:
        for model in models:
            model.dispose()


def second_route(model, roadmap_path, endpoints, regions, drm_options, *, seed,
                 out, validate_path, deadline=None, query_config=None):
    """One paper VI-C alternate-route attempt, with unchanged query geometry.

    ``validate_path(path, output_directory)`` must perform the same endpoint,
    collision and numerical checks as the first DRM path and return a dict
    containing ``passed``. Unavailable/invalid alternatives retain the first
    route; unexpected native errors propagate to the isolated worker.
    """
    import csdecomp as csd

    out = _directory(out)
    if out is None:
        raise ValueError('Alternate DRM search requires a persistent output directory')
    endpoints = np.asarray(endpoints, float)
    if endpoints.shape != (2, 7) or not np.isfinite(endpoints).all():
        raise ValueError('Alternate DRM requires the original two finite 7D endpoints')
    if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)) or seed < 0:
        raise ValueError('Alternate route selection seed must be a nonnegative integer')
    if deadline is not None and time.perf_counter() >= deadline:
        raise TimeoutError('Online budget exhausted before alternate DRM search')
    started = time.perf_counter()
    eligible = [i for i, region in enumerate(regions)
                if not region.PointInSet(endpoints[0], 1e-6)
                and not region.PointInSet(endpoints[1], 1e-6)]
    metadata = dict(status='starting', first_path_retained=True,
                    algorithm='paper_VI-C_one_additional_path_attempt',
                    selection_rng='numpy.PCG64', selection_seed=int(seed),
                    eligible_region_indices=eligible, endpoints_changed=False,
                    original_roadmap_sha256=hashlib.sha256(Path(roadmap_path).read_bytes()).hexdigest())
    _save(out, 'alternate_path_search.json', metadata)
    if not eligible:
        metadata.update(status='no_eligible_interior_region', seconds=time.perf_counter()-started)
        _save(out, 'alternate_path_search.json', metadata)
        return None, metadata
    selected = int(np.random.Generator(np.random.PCG64(int(seed))).choice(eligible))
    region = regions[selected]
    np.savez_compressed(out / 'excluded_region.npz', A=region.A(), b=region.b())
    copied = csd.DRM()
    copied.Read(str(roadmap_path))
    nodes = copied.id_to_node_map
    kept = {i: q for i, q in nodes.items() if not region.PointInSet(q, 1e-7)}
    copied.id_to_node_map = kept
    copied.node_adjacency_map = {
        i: [j for j in adjacent if j in kept]
        for i, adjacent in copied.node_adjacency_map.items() if i in kept}
    copied.id_to_pose_map = {i: pose for i, pose in copied.id_to_pose_map.items() if i in kept}
    copied.collision_map = {
        cell: [i for i in ids if i in kept]
        for cell, ids in copied.collision_map.items()}
    # Native edge_collision_map is read-only in Python. Removed-node entries
    # only populate forbidden_edge_set; adjacency no longer visits those IDs.
    # No coordinates are dereferenced through that map in BuildCollisionSet.
    metadata.update(selected_region_index=selected, original_nodes=len(nodes),
                    kept_nodes=len(kept), blocked_nodes=len(nodes)-len(kept),
                    edge_collision_map='unchanged read-only native map; removed-node entries are harmless forbidden-edge keys')
    np.savez_compressed(out / 'node_selection.npz',
                        original_node_ids=np.asarray(sorted(nodes), np.int64),
                        kept_node_ids=np.asarray(sorted(kept), np.int64))
    if len(kept) < 2:
        metadata.update(status='no_remaining_roadmap', seconds=time.perf_counter()-started)
        _save(out, 'alternate_path_search.json', metadata)
        return None, metadata
    target = out / 'alternate_roadmap.map'
    copied.Write(str(target))
    metadata.update(alternate_roadmap_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                    status='native_planner_starting')
    _save(out, 'alternate_path_search.json', metadata)
    options = csd.DrmPlannerOptions()
    for key, value in drm_options.items():
        setattr(options, key, value)
    planner = csd.DrmPlanner(model.csd_plant, options, True)
    planner.LoadRoadmap(str(target))
    planner.BuildCollisionSet(model.voxels)
    if deadline is not None and time.perf_counter() >= deadline:
        raise TimeoutError('Online budget exhausted before alternate native Plan')
    try:
        if query_config is None:
            ok, path = planner.Plan(endpoints[0], endpoints[1], model.voxels, model.voxel_radius)
        else:
            from .csdecomp_baseline import plan_query
            ok, path = plan_query(planner, model, endpoints, query_config, out)
    except RuntimeError as error:
        from .failure_classification import KNOWN_DRM_NO_ROUTE, LEGACY_DRM_NO_ROUTE
        # Both native backends report this ordinary optional-search outcome.
        # Do not turn CUDA, map or other backend faults into a retained route.
        if str(error) not in (KNOWN_DRM_NO_ROUTE, LEGACY_DRM_NO_ROUTE):
            raise
        metadata.update(status='native_no_alternate_path', exception=repr(error),
                        seconds=time.perf_counter()-started)
        _save(out, 'alternate_path_search.json', metadata)
        return None, metadata
    path = np.asarray(path, float)
    np.save(out / 'path.npy', path)
    metadata['native_success'] = bool(ok)
    if not ok:
        metadata.update(status='native_no_alternate_path', seconds=time.perf_counter()-started)
        _save(out, 'alternate_path_search.json', metadata)
        return None, metadata
    validation = validate_path(path, out / 'validation')
    passed = bool(validation['passed'])
    metadata.update(status='found' if passed else 'alternate_path_validation_failed',
                    validation_status=validation.get('status'), independently_valid=passed,
                    seconds=time.perf_counter()-started)
    _save(out, 'alternate_path_search.json', metadata)
    return (path if passed else None), metadata
