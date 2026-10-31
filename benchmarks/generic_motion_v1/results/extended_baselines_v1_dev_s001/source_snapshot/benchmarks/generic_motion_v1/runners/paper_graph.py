"""Two-route EI-ZO/LGCS pipeline using the existing perspective MISOCP.

The region pool is a graph, not an ordered corridor. Recovery assignments are
explicitly translated from the selected path back to this pool. No native
algorithm, canonical geometry or fixed task endpoint is changed.
"""
from pathlib import Path
import time
import numpy as np
from ..roadmap import write
from .validate import validate
from .native_handoff import native_region_with_anchors


def selected_pool_indices(candidate, regions):
    indices = candidate.get('segment_regions')
    if not isinstance(indices, list) or len(indices) != len(candidate['points']) - 1:
        raise ValueError('LGCS requires one pool index for every PWL segment')
    if any(isinstance(i, bool) or not isinstance(i, (int, np.integer)) or
           not 0 <= i < len(regions) for i in indices):
        raise ValueError('LGCS selected region index outside saved pool')
    return np.asarray(indices, int)


def save_pool(out, regions, seeds, paths):
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    arrays = {}
    for i, region in enumerate(regions):
        arrays[f'A_{i:04d}'] = region.A(); arrays[f'b_{i:04d}'] = region.b()
    np.savez_compressed(out / 'regions.npz', **arrays)
    np.savez_compressed(out / 'pool_paths.npz', seeds=np.asarray(seeds),
                        **{f'path_{i}': p for i, p in enumerate(paths)})
    write(out / 'regions.json', dict(count=len(regions), facets=[len(r.b()) for r in regions],
        order='Unordered LGCS graph pool; candidate.segment_regions specifies the selected order'))


def protected_edge_anchors(seed, paths, endpoints=None):
    anchors = list(np.asarray(seed, float))
    if endpoints is not None:
        for path in paths:
            if any(np.array_equal(q, path[0]) for q in seed):
                anchors.append(endpoints[0])
            if any(np.array_equal(q, path[-1]) for q in seed):
                anchors.append(endpoints[1])
    return np.asarray(anchors, float)


def complete_pool_cover(regions, seeds, paths, inflate, tolerance, out, on_edge, endpoints=None):
    """Preserve every edge of both accepted DRM paths, including after cuts."""
    from .csdecomp_baseline import cover_region_anchors
    events = []
    for path_index, path in enumerate(paths):
        for edge_index, (a, b) in enumerate(zip(path[:-1], path[1:])):
            anchors = protected_edge_anchors([a, b], paths, endpoints)
            found = next((i for i, r in enumerate(regions)
                          if r.PointInSet(a, 1e-7) and r.PointInSet(b, 1e-7)), None)
            if found is None:
                on_edge()
                found = len(regions)
                region = cover_region_anchors(inflate(a, b), anchors,
                    tolerance, Path(out) / f'edge_{path_index}_{edge_index}')
                regions.append(region); seeds.append(np.asarray([a, b]))
                action = 'inflate_uncovered_original_edge'
            else:
                action = 'reuse_covering_region'
                regions[found] = cover_region_anchors(regions[found], anchors, tolerance,
                    Path(out) / f'edge_{path_index}_{edge_index}')
            events.append(dict(path=path_index, edge=edge_index, region=found, action=action))
    complete = all(any(all(r.PointInSet(q, 1e-7) for q in
                       protected_edge_anchors([a, b], paths, endpoints)) for r in regions)
                   for p in paths for a, b in zip(p[:-1], p[1:]))
    write(Path(out) / 'path_coverage.json', dict(complete=complete, paths=len(paths), events=events))
    if not complete:
        raise ValueError('LGCS graph lost a fixed DRM seed edge')
    return regions, seeds


def run_graph(model, case, cfg, out, path, endpoints, road, seed, inflate, stage, result, deadline):
    import csdecomp as csd
    from pydrake.all import HPolyhedron
    from .paper_lgcs import solve, second_route
    from .csdecomp_baseline import (build_active_corridor, cover_region_anchors,
        _save_candidate, _validate_frontend_path, _native_option_values)
    out = Path(out)
    stage('eizo_inflation'); started = time.perf_counter()
    regions, mapping = build_active_corridor(path, endpoints, inflate,
        cfg['region_coverage_tolerance_rad'], out / 'first_corridor',
        on_edge=lambda: stage('eizo_inflation'))
    seeds = [path[[entry['original_seed_edge_index'], entry['original_seed_edge_index'] + 1]]
             for entry in mapping]
    paths = [path]
    result['timing']['eizo_inflation_seconds'] = time.perf_counter() - started
    stage('alternate_drm_path'); started = time.perf_counter()
    alternative, metadata = second_route(model, road / 'native.map', endpoints, regions, cfg['drm'],
        seed=seed, out=out / 'alternate', deadline=deadline,
        validate_path=lambda p, dest: _validate_frontend_path(p, model, case, cfg, dest),
        query_config=cfg)
    result['timing']['alternate_drm_path_seconds'] = time.perf_counter() - started
    result['alternate_route'] = metadata
    if alternative is not None:
        paths.append(np.asarray(alternative, float))
    stage('eizo_inflation'); started = time.perf_counter()
    regions, seeds = complete_pool_cover(regions, seeds, paths, inflate,
        cfg['region_coverage_tolerance_rad'], out / 'initial_pool_cover',
        lambda: stage('eizo_inflation'), endpoints)
    result['timing']['eizo_inflation_seconds'] += time.perf_counter() - started
    result['timing'].update(trajectory_optimization_seconds=0., validation_seconds=0.,
                           collision_recovery_seconds=0.)
    candidate = validation = None
    for iteration in range(cfg['max_recovery_attempts'] + 1):
        round_out = out / 'rounds' / f'{iteration:03d}'
        save_pool(round_out, regions, seeds, paths); save_pool(out, regions, seeds, paths)
        stage('trajectory_optimization'); started = time.perf_counter()
        remaining = min(cfg['optimizer_budget_seconds'], deadline - started)
        if remaining <= 0:
            raise TimeoutError('LGCS construction exhausted online budget')
        candidate = solve(regions, endpoints[0], endpoints[1], time_limit=remaining, out=round_out,
                          backend=cfg.get('lgcs', {}).get('solver', 'gurobi'))
        result['timing']['trajectory_optimization_seconds'] += time.perf_counter() - started
        _save_candidate(round_out, candidate); _save_candidate(out, candidate)
        result['metrics']['recovery_attempts'] = iteration
        if candidate.get('points') is None:
            result['status'] = 'solver_failed'; break
        selected = selected_pool_indices(candidate, regions)
        stage('validation'); started = time.perf_counter()
        validation = validate(candidate, model, case, cfg,
            regions=[regions[i] for i in selected], out=round_out)
        result['timing']['validation_seconds'] += time.perf_counter() - started
        write(out / 'validation.json', {k: v for k, v in validation.items() if k != '_samples'})
        np.savez_compressed(out / 'trajectory_samples.npz', **validation['_samples'])
        result['status'] = validation['status'] if candidate['solver_success'] else 'solver_failed'
        if (not candidate['solver_success'] or validation['passed'] or
                validation['status'] != 'collision_failed' or iteration == cfg['max_recovery_attempts']):
            break
        stage('collision_recovery'); started = time.perf_counter()
        recovery_out = out / 'recovery' / f'{iteration:03d}'
        recovery_out.mkdir(parents=True, exist_ok=True)
        points = validation['collision_points']
        assignment = selected[np.asarray(validation['collision_regions'], int)]
        witnesses = np.unique(np.linspace(0, len(points) - 1,
            min(len(points), cfg['maximum_recovery_witnesses']), dtype=int))
        edit = csd.EditRegionsOptions(); edit.configuration_margin = .01
        write(recovery_out / 'edit_parameters.json', dict(native_effective_options=_native_option_values(edit)))
        protected = [protected_edge_anchors(s, paths, endpoints) for s in seeds]
        native = [native_region_with_anchors(r, anchors, cfg['region_coverage_tolerance_rad'],
                  recovery_out / 'native_handoff' / f'{i:04d}')
                  for i, (r, anchors) in enumerate(zip(regions, protected))]
        seeds_array = np.asarray(seeds)
        np.savez_compressed(recovery_out / 'native_edit_inputs.npz', seeds=seeds_array,
            collision_points=points[witnesses], pool_assignments=assignment[witnesses], selected_pool=selected)
        edited = csd.EditRegionsCuda(points[witnesses].T, assignment[witnesses].tolist(),
            seeds_array[:, 0].T, seeds_array[:, 1].T, native, model.mplant,
            model.robot_geometry_ids, model.voxels, model.voxel_radius, edit)
        np.savez_compressed(recovery_out / 'native_edit_debug.npz',
            projections=np.asarray(edited[1][0]), optimized_collisions=np.asarray(edited[1][1]))
        save_pool(recovery_out / 'native_edited',
                  [HPolyhedron(r.A(), r.b()) for r in edited[0]], seeds, paths)
        regions = [cover_region_anchors(HPolyhedron(r.A(), r.b()), anchors,
            cfg['region_coverage_tolerance_rad'], recovery_out / 'seed_coverage' / f'{i:04d}')
            for i, (r, anchors) in enumerate(zip(edited[0], protected))]
        regions, seeds = complete_pool_cover(regions, seeds, paths, inflate,
            cfg['region_coverage_tolerance_rad'], recovery_out / 'restored_cover',
            lambda: stage('collision_recovery'), endpoints)
        result['timing']['collision_recovery_seconds'] += time.perf_counter() - started
    return candidate, validation, regions
