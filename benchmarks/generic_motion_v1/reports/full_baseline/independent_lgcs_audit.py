"""Read-only, NumPy/standard-library audit of every sealed LGCS attempt.

No planner, GPU call, Gurobi model, or runtime validator is imported. This
checks saved algebra and provenance, not physical collision freedom. Usage:
  bash local/python.sh -m benchmarks.generic_motion_v1.reports.full_baseline.independent_lgcs_audit \
    --run benchmarks/generic_motion_v1/results/rss2025_full_smoke_v1 --output /new/audit/directory
"""
from __future__ import annotations

import argparse
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import traceback

import numpy as np


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    with Path(path).open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else hashlib.sha256(handle.read()).hexdigest()


def arrays(path):
    with np.load(path, allow_pickle=False) as saved:
        return {key: saved[key] for key in saved.files}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def close(left, right, tolerance, message):
    left, right = np.asarray(left), np.asarray(right)
    require(left.shape == right.shape, message + ' (shape)')
    require(np.isfinite(left).all() and np.isfinite(right).all(), message + ' (nonfinite)')
    error = float(np.max(np.abs(left-right), initial=0.))
    require(error <= tolerance, f'{message}: {error} > {tolerance}')
    return error


def load_pool(directory):
    data = arrays(directory/'regions.npz')
    indices = sorted(int(key[2:]) for key in data if key.startswith('A_'))
    require(indices == list(range(len(indices))), 'Noncontiguous region pool indices')
    regions = [(data[f'A_{i:04d}'], data[f'b_{i:04d}']) for i in indices]
    for A, b in regions:
        require(A.ndim == 2 and A.shape[1] == 7 and b.shape == (len(A),), 'Malformed region array')
        require(np.isfinite(A).all() and np.isfinite(b).all(), 'Nonfinite region')
    pool = arrays(directory/'pool_paths.npz')
    paths = [pool[key] for key in sorted((key for key in pool if key.startswith('path_')), key=lambda key: int(key[5:]))]
    seeds = pool['seeds']
    require(seeds.shape == (len(regions), 2, 7), 'Seed/pool dimensions disagree')
    require(bool(paths), 'Missing original DRM paths')
    return regions, seeds, paths


def residual(region, points):
    A, b = region
    return float(np.max(A@np.asarray(points).T-b[:, None], initial=-np.inf))


def protected(points, paths, endpoints):
    result = list(points)
    for path in paths:
        if any(np.array_equal(q, path[0]) for q in points):
            result.append(endpoints[0])
        if any(np.array_equal(q, path[-1]) for q in points):
            result.append(endpoints[1])
    return np.asarray(result)


def pool_cover(regions, seeds, paths, endpoints, raw_original, alternate, endpoint_tolerance):
    close(paths[0], raw_original, 0., 'First DRM path changed in pool')
    require(len(paths) in (1, 2), 'Unexpected number of LGCS input paths')
    if len(paths) == 2:
        require(alternate is not None, 'Second pool path has no accepted alternate')
        close(paths[1], alternate, 0., 'Second DRM path changed in pool')
    count, maximum = 0, -np.inf
    all_edges = [(a, b) for path in paths for a, b in zip(path[:-1], path[1:])]
    for path in paths:
        close(path[[0, -1]], endpoints, endpoint_tolerance, 'DRM path differs from canonical endpoints')
        for a, b in zip(path[:-1], path[1:]):
            value = min(residual(region, protected([a, b], paths, endpoints)) for region in regions)
            require(value <= 1e-7+1e-12, f'Accepted pool lost an original DRM edge: residual {value}')
            maximum = max(maximum, value)
            count += 1
    for index, seed in enumerate(seeds):
        require(any(np.array_equal(seed, edge) for edge in all_edges), 'Region seed is not an original DRM edge')
        require(residual(regions[index], protected(seed, paths, endpoints)) <= 1e-7+1e-12,
                'Region lost its own protected seed')
    return dict(original_edges_checked=count, maximum_best_cover_residual=maximum)


def mps_dimensions(path):
    """Parse submitted MPS text without invoking the commercial optimizer."""
    rows, variables, quadratic_rows, binary = set(), set(), set(), set()
    section = None
    with gzip.open(path, 'rt') as handle:
        for line in handle:
            if not line.strip() or line.startswith('*'):
                continue
            fields = line.split()
            if not line[0].isspace():
                section = fields[0]
                if section == 'QCMATRIX':
                    quadratic_rows.add(fields[1])
                continue
            if section == 'ROWS' and fields[0] != 'N':
                rows.add(fields[1])
            elif section == 'COLUMNS' and 'MARKER' not in line:
                variables.add(fields[0])
            elif section == 'BOUNDS':
                variables.add(fields[2])
                if fields[0] == 'BV':
                    binary.add(fields[2])
    return dict(solver_variables=len(variables), solver_constraints=len(rows-quadratic_rows),
                solver_quadratic_constraints=len(quadratic_rows), binary_flow_variables=len(binary))


def check_alternate(attempt, job, endpoints, hash_cache):
    path = attempt/'alternate/alternate_path_search.json'
    if not path.exists():
        return None, None
    data = read(path)
    require(data['endpoints_changed'] is False, 'Alternate attempt changed endpoints')
    require(data['selection_seed'] == job['method_seed'], 'Alternate selection seed changed')
    original = Path(job['roadmap_dir'])/'native.map'
    if original not in hash_cache:
        hash_cache[original] = digest(original)
    require(data['original_roadmap_sha256'] == hash_cache[original], 'Alternate used another original roadmap')
    record = dict(status=data['status'], selected_region=data.get('selected_region_index'),
                  eligible_regions=data['eligible_region_indices'], accepted=False)
    if 'selected_region_index' in data:
        selection = arrays(attempt/'alternate/node_selection.npz')
        original_ids, kept_ids = selection['original_node_ids'], selection['kept_node_ids']
        roadmap_arrays = arrays(Path(job['roadmap_dir'])/'roadmap.npz')
        excluded = arrays(attempt/'alternate/excluded_region.npz')
        require(set(original_ids) == set(roadmap_arrays['node_ids']), 'Alternate original node list changed')
        require(set(kept_ids) <= set(original_ids), 'Alternate introduced roadmap nodes')
        expected = sorted(int(i) for i, q in zip(roadmap_arrays['node_ids'], roadmap_arrays['nodes'])
                          if np.max(excluded['A']@q-excluded['b']) > 1e-7)
        require(expected == sorted(kept_ids.tolist()), 'Excluded-region node pruning differs from saved geometry')
        require(data['blocked_nodes'] == len(original_ids)-len(kept_ids), 'Alternate blocked-node count wrong')
        chosen = int(np.random.Generator(np.random.PCG64(data['selection_seed'])).choice(data['eligible_region_indices']))
        require(chosen == data['selected_region_index'], 'Alternate PCG64 choice does not replay')
        A = arrays(attempt/'first_corridor/regions.npz')
        close(excluded['A'], A[f'A_{chosen:04d}'], 0., 'Excluded normals differ from initial region')
        close(excluded['b'], A[f'b_{chosen:04d}'], 0., 'Excluded offsets differ from initial region')
        if 'alternate_roadmap_sha256' in data:
            require(digest(attempt/'alternate/alternate_roadmap.map') == data['alternate_roadmap_sha256'], 'Alternate map hash mismatch')
        record.update(blocked_nodes=data['blocked_nodes'], original_nodes=len(original_ids), kept_nodes=len(kept_ids))
    if data['status'] == 'found':
        candidate = np.load(attempt/'alternate/path.npy', allow_pickle=False)
        close(candidate[[0, -1]], endpoints, job['evaluation_config']['endpoint_tolerance_rad'], 'Alternate endpoints changed')
        validation = read(attempt/'alternate/validation/validation.json')
        require(validation['passed'] and data['independently_valid'], 'Accepted alternate had failed validation')
        record['accepted'] = True
        return candidate, record
    return None, record


def candidate_audit(directory, regions, endpoints, cfg):
    candidate = read(directory/'candidate.json')
    require(candidate['kind'] == 'polygon' and candidate['objective'] == 'minimum_distance',
            'LGCS candidate changed the fixed representation or objective')
    values = arrays(directory/'candidate.npz')
    if 'points' not in values:
        return dict(solver_success=candidate['solver_success'], no_points=True)
    points = values['points']
    selected = candidate['segment_regions']
    require(points.ndim == 2 and points.shape[1] == 7 and len(points) >= 2, 'Malformed PWL candidate')
    require(len(selected) == len(points)-1 and all(type(i) is int and 0 <= i < len(regions) for i in selected),
            'Bad selected pool membership indices')
    endpoint_error = close(points[[0, -1]], endpoints, cfg['endpoint_tolerance_rad'], 'Optimizer changed fixed endpoints')
    maximum = max(residual(regions[index], points[i:i+2]) for i, index in enumerate(selected))
    length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
    close(length, candidate['recovered_path_length'], 1e-10, 'Recovered PWL cost mismatch')
    # Removing disjoint zero-cost integer-flow cycles cannot increase cost.
    require(length <= candidate['solver_objective']+1e-5, 'Recovered path exceeds MISOCP objective')
    record = dict(solver_success=candidate['solver_success'], points=len(points), selected_regions=selected,
                  recovered_length_rad=length, maximum_region_residual=maximum,
                  selected_membership_passed=maximum <= cfg['feasibility_tolerance'], endpoint_error=endpoint_error)
    validation_path = directory/'validation.json'
    if validation_path.exists():
        validation = read(validation_path)
        samples = arrays(directory/'trajectory_samples.npz')
        require(validation['passed'] == (validation['status'] == 'success_validated_numerical'),
                'Validation passed flag and status disagree')
        counts = np.maximum(2, np.ceil(np.linalg.norm(np.diff(points, axis=0), axis=1)/cfg['validation_step_rad'])+1).astype(int)
        reconstructed = np.vstack([np.linspace(a, b, n) for a, b, n in zip(points[:-1], points[1:], counts)])
        segment = np.concatenate([np.full(n, i) for i, n in enumerate(counts)])
        times = np.concatenate([np.linspace(i, i+1, n)/(len(points)-1) for i, n in enumerate(counts)])
        error = close(samples['q'], reconstructed, 1e-12, 'Saved PWL samples differ from independent interpolation')
        close(samples['segment'], segment, 0., 'Saved sample segment assignments wrong')
        close(samples['time'], times, 1e-12, 'Saved normalized PWL parameter wrong')
        close(length, validation['cost'], 1e-10, 'Validated PWL cost mismatch')
        close(maximum, validation['region_residual'], 1e-10, 'Saved membership residual wrong')
        require(validation['samples'] == len(samples['q']), 'Saved validation sample count wrong')
        native, independent, clearance = samples['native'], samples['independent'], samples['clearance']
        require(native.dtype == independent.dtype == np.bool_
                and native.shape == independent.shape == clearance.shape == (len(samples['q']),),
                'Saved collision labels must be one boolean per finite trajectory sample')
        require(np.isfinite(clearance).all(), 'Nonfinite saved collision clearance')
        require(np.array_equal(independent, clearance > 0.),
                'Saved independent collision labels disagree with signed clearance')
        # The frozen canonical Model.contact_band_m is 2e-6. This checks
        # consistency of saved numerical evidence, without calling its model.
        ambiguous = np.abs(clearance) <= 2e-6
        collision_counts = dict(native_collisions=int((~native).sum()),
            independent_collisions=int((~independent).sum()),
            model_disagreements=int((native != independent).sum()),
            contact_band_samples=int(ambiguous.sum()))
        for key, value in collision_counts.items():
            require(validation[key] == value, f'Saved {key} count disagrees with collision labels')
        require(validation['collision_free'] == bool(native.all() and independent.all()),
                'Saved collision-free flag disagrees with collision labels')
        close(float(clearance.min()), validation['minimum_sampled_clearance_m'], 1e-12,
              'Saved minimum clearance disagrees with samples')
        if validation['passed']:
            require(record['selected_membership_passed'], 'Validator accepted out-of-region segment')
            require(bool(native.all() and independent.all() and not ambiguous.any()),
                    'Validation accepted colliding or numerically ambiguous samples')
            require(validation['joint_limits_passed'] and validation['derivative_limits_passed'],
                    'Validation accepted a reported motion-limit violation')
        record.update(samples=len(samples['q']), maximum_interpolation_error=error,
                      validation_status=validation['status'], passed=validation['passed'],
                      saved_collision_labels_consistent=True, collision_counts=collision_counts)
    return record


def audit_recovery(directory, previous, cfg):
    source = arrays(directory/'native_edit_inputs.npz')
    validation = read(previous/'validation.json')
    candidate = read(previous/'candidate.json')
    points = np.asarray(validation['collision_points'])
    segments = np.asarray(validation['collision_regions'], int)
    selected = np.asarray(candidate['segment_regions'], int)
    witnesses = np.unique(np.linspace(0, len(points)-1, min(len(points), cfg['maximum_recovery_witnesses']), dtype=int))
    close(source['collision_points'], points[witnesses], 0., 'Native edit collision witnesses changed')
    close(source['selected_pool'], selected, 0., 'Native edit selected pool changed')
    close(source['pool_assignments'], selected[segments[witnesses]], 0., 'Native edit mapped witnesses to wrong pool')
    original_pool = arrays(previous/'pool_paths.npz')
    close(source['seeds'], original_pool['seeds'], 0., 'Native edit original seeds changed')
    handoffs = []
    for handoff in sorted((directory/'native_handoff').glob('*/handoff.json')):
        record = read(handoff)
        original = arrays(handoff.parent/'original.npz')
        corrected_path = handoff.parent/'corrected.npz'
        if corrected_path.exists():
            corrected = arrays(corrected_path)
            close(corrected['A'], original['A'].astype(np.float32), 0., 'Native handoff changed quantized normals')
            close(corrected['anchors'], original['anchors'].astype(np.float32), 0., 'Native handoff changed quantized anchors')
            norms = np.linalg.norm(corrected['A'].astype(float), axis=1)
            delta = np.divide(np.abs(corrected['b'].astype(float)-original['b']), norms,
                              out=np.zeros_like(norms), where=norms > 0)
            if record['status'] == 'passed':
                require(delta.max(initial=0.) <= cfg['region_coverage_tolerance_rad'], 'Native offset correction exceeded frozen tolerance')
                require(all(record['native_anchor_containment_after']), 'Native handoff claimed passing without anchor containment')
            handoffs.append(dict(index=handoff.parent.name, status=record['status'], maximum_offset_change=float(delta.max(initial=0.))))
    restored = directory/'restored_cover/path_coverage.json'
    return dict(witnesses=len(witnesses), mapping_verified=True, handoffs=handoffs,
                restored_cover_claim=read(restored)['complete'] if restored.exists() else None,
                note='Raw edited pools may lose unprotected spans; only subsequent accepted optimizer pools are required to cover all original edges.')


def audit_attempt(attempt, job, latest, hash_cache):
    receipt, result = read(attempt/'receipt.json'), read(attempt/'result.json')
    require(receipt['status'] == result['status'], 'Receipt status differs from result')
    for name, expected in receipt['files'].items():
        require(digest(attempt/name) == expected, f'Receipt hash mismatch: {name}')
    saved_case = read(attempt/'input.json')
    for key, expected in job['case'].items():
        require(saved_case.get(key) == expected, f'Frozen case changed: {key}')
    require(saved_case['_objective'] == 'minimum_distance', 'LGCS objective changed')
    cfg = read(attempt/'evaluation.json')
    require(cfg == job['evaluation_config'], 'Frozen evaluation configuration changed')
    record = dict(job_id=job['job_id'], attempt=attempt.name, latest=latest, status=result['status'],
                  error_category=result.get('error_category'), files_checked=len(receipt['files']),
                  family=job['family'], rounds=[], recoveries=[], alternate=None,
                  final_stage=result.get('failed_stage', result.get('final_stage')))
    if job['case'].get('q_start') is None or job['case'].get('q_goal') is None:
        require(result['status'] == 'invalid_endpoint', 'Frozen invalid input was used for planning')
        return record
    endpoints = np.asarray([job['case']['q_start'], job['case']['q_goal']])
    roadmap_meta = read(Path(job['roadmap_dir'])/'metadata.json')
    for key in ('robot_sha256', 'collision_geometry_sha256'):
        require(roadmap_meta[key] == saved_case[key], 'Roadmap and online frozen robot geometry differ')
    alternate, alternate_record = check_alternate(attempt, job, endpoints, hash_cache)
    record['alternate'] = alternate_record
    raw_original = np.load(attempt/'path.npy', allow_pickle=False) if (attempt/'path.npy').exists() else None
    rounds = sorted((attempt/'rounds').glob('[0-9][0-9][0-9]'))
    for directory in rounds:
        regions, seeds, paths = load_pool(directory)
        checks = pool_cover(regions, seeds, paths, endpoints, raw_original, alternate, cfg['endpoint_tolerance_rad'])
        item = dict(round=directory.name, pool_regions=len(regions), original_paths=len(paths), **checks)
        solver_file = directory/'lgcs_solver.json'
        failure_file = directory/'lgcs_failure.json'
        metadata_file = solver_file if solver_file.exists() else failure_file
        if metadata_file.exists():
            metadata = read(metadata_file)
            mps = mps_dimensions(directory/'model.mps.gz')
            for key in ('solver_variables', 'solver_constraints', 'solver_quadratic_constraints'):
                require(mps[key] == metadata[key], f'MPS {key} differs from solver metadata')
            require(mps['solver_quadratic_constraints'] == len(regions), 'MPS missing per-region cone')
            require(mps['binary_flow_variables'] == metadata['graph_edges'], 'MPS flow variables are not binary')
            item.update(mps=mps, solver_error_number=metadata.get('solver_error_number'),
                        solver_status=metadata.get('solver_status'), solver_success=metadata['solver_success'])
            if metadata.get('solver_error_number') == 10010:
                require(metadata['status'] == 'unsupported' and not metadata['solver_success'], 'License failure misreported as solver success')
        if (directory/'candidate.json').exists():
            item['candidate'] = candidate_audit(directory, regions, endpoints, cfg)
        record['rounds'].append(item)
    for recovery in sorted((attempt/'recovery').glob('[0-9][0-9][0-9]')):
        if (recovery/'native_edit_inputs.npz').exists():
            record['recoveries'].append(dict(round=recovery.name, **audit_recovery(recovery, attempt/'rounds'/recovery.name, cfg)))
    if rounds and (attempt/'validation.json').exists():
        final = rounds[-1]
        if (final/'validation.json').exists():
            require(read(attempt/'validation.json') == read(final/'validation.json'), 'Top-level validation differs from latest validated round')
            record['root_validation_scope'] = 'final_round'
        else:
            record['root_validation_scope'] = 'prior_round_only; last solve produced no validated candidate'
            require(result['status'] != 'success_validated_numerical', 'Stale validation accepted as final success')
    if result['status'] == 'success_validated_numerical':
        require(bool(record['rounds']) and record['rounds'][-1].get('candidate', {}).get('passed'), 'Successful LGCS missing final validated polygon')
        require(record['rounds'][-1]['candidate']['solver_success'] is True,
                'Successful LGCS has an unsuccessful final solver candidate')
        require(result.get('frontend_native_success') is True and result.get('frontend_success') is True,
                'Successful LGCS did not pass both native and numerical frontend gates')
        frontend = read(attempt/'frontend_validation/validation.json')
        require(frontend['passed'] is True and frontend['status'] == 'success_validated_numerical',
                'Successful LGCS has no passing numerical seed validation')
        require(result['frontend_validation_status'] == frontend['status'],
                'Successful LGCS frontend validation status changed')
        record['success_solver_and_frontend_gates_verified'] = True
    return record


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    run, out = args.run.resolve(), args.output.resolve()
    require(run not in out.parents and run != out, 'Audit output must be outside the sealed run')
    require(not (out/'audit.json').exists() and not (out/'audit.md').exists(), 'Refusing to overwrite previous audit results')
    manifest = read(run/'run_manifest.json')
    errors, records, hash_cache, source_count = [], [], {}, 0
    for name, expected in manifest['source_hashes'].items():
        path = run/'source_snapshot'/name
        if not path.exists() or digest(path) != expected:
            errors.append(dict(kind='source_snapshot', file=name))
        source_count += 1
    for job in manifest['jobs']:
        attempts = sorted((run/'jobs'/job['job_id']/'attempts').glob('[0-9][0-9][0-9][0-9]'))
        if not attempts or any(not (attempt/'receipt.json').exists() for attempt in attempts):
            errors.append(dict(kind='unsealed_job', job_id=job['job_id']))
            continue
        if job['method'] != 'LGCS':
            continue
        for attempt in attempts:
            try:
                records.append(audit_attempt(attempt, job, attempt == attempts[-1], hash_cache))
            except Exception as error:
                errors.append(dict(kind='attempt_audit', job_id=job['job_id'], attempt=attempt.name,
                                   error=repr(error), traceback=traceback.format_exc()))
    latest = [item for item in records if item['latest']]
    counts = dict(latest_status=dict(Counter(item['status'] for item in latest)),
                  all_attempt_status=dict(Counter(item['status'] for item in records)),
                  latest_alternate_status=dict(Counter(item['alternate']['status'] for item in latest if item['alternate'])),
                  accepted_second_routes=sum(bool(item['alternate'] and item['alternate']['accepted']) for item in latest),
                  recovery_rounds=sum(len(item['recoveries']) for item in latest),
                  recovered_successes=sum(bool(item['recoveries']) and item['status'] == 'success_validated_numerical' for item in latest),
                  license_limit_failures=sum(item.get('solver_error_number') == 10010 for row in records for item in row['rounds']),
                  unsupported_attempts=sum(item['error_category'] == 'dependency_unsupported' for item in records),
                  native_process_aborts=sum(item['error_category'] == 'native_process_signal' for item in records),
                  selected_segments_checked=sum(len(item.get('candidate', {}).get('selected_regions', [])) for row in records for item in row['rounds']),
                  interpolated_samples_checked=sum(item.get('candidate', {}).get('samples', 0) for row in records for item in row['rounds']),
                  accepted_pool_original_edges_checked=sum(item['original_edges_checked'] for row in records for item in row['rounds']))
    report = dict(passed=not errors, run=str(run), run_manifest_sha256=digest(run/'run_manifest.json'),
                  source_commit=manifest['source_code_commit'], planned_lgcs_jobs=sum(job['method'] == 'LGCS' for job in manifest['jobs']),
                  audited_attempts=len(records), source_snapshots_checked=source_count,
                  counts=counts, records=records, errors=errors, audit_source_sha256=digest(__file__),
                  limitations=['No GPU, planner, solver or runtime validator was called.',
                      'Membership and interpolation are independently recomputed from saved arrays; this is not a continuous collision proof.',
                  'Actual collision labels are preserved evidence from the original run and are not recomputed in this CPU audit.',
                      'Saved native/independent labels, signed clearances, ambiguity counts and passed flags are checked for consistency; every success also requires solver success and both frontend gates.',
                      'Same-robot/scene checks use immutable case/model hashes and frozen call sites. The native alternate-roadmap binary is hash-checked; its serializer is not independently decoded.',
                      'Raw cut pools may temporarily lose original path coverage; complete original-edge coverage is independently checked for every subsequent optimizer input pool.',
                      'A final failed solve may retain prior-round top-level validation; this audit labels its scope and never counts it as final success.'])
    out.mkdir(parents=True, exist_ok=True)
    (out/'audit.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    lines = ['# Independent LGCS evidence audit', '', f"Audit passed: **{report['passed']}**. Planned LGCS jobs: {report['planned_lgcs_jobs']}; audited attempts: {len(records)}.",
             '', f"Run: `{run}`", f"Runtime commit: `{report['source_commit']}`", '', '## Recorded outcomes', '',
             '```json', json.dumps(counts, indent=2), '```', '', '## Scope and limits', '']
    lines.extend('- '+item for item in report['limitations'])
    if errors:
        lines.extend(['', '## Audit failures', '', '```json', json.dumps(errors, indent=2), '```'])
    (out/'audit.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({key: report[key] for key in ('passed', 'planned_lgcs_jobs', 'audited_attempts', 'counts', 'errors')}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
