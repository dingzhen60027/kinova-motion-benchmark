"""Read-only CPU audit of frozen benchmark generation and distribution.

No model/solver/GPU is constructed. Stored collision classifications are
checked for internal consistency, not recomputed; this is explicitly not a
fresh collision audit. All artifacts are written beside this script only.
"""
from __future__ import annotations

import ast
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np

from benchmarks.generic_motion_v1.generator.scene import _proposal, domain_seed, digest, rng
from benchmarks.generic_motion_v1.generator.voxelize import voxelize

HERE = Path(__file__).resolve().parent
BASE = HERE.parents[2]
ROOT = BASE.parents[1]
ARCHIVE = BASE / 'reports/qualification/frozen_v1_source'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def desc(values):
    x = np.asarray(values, dtype=float)
    if len(x) == 0:
        return {'n': 0}
    return dict(n=len(x), minimum=float(x.min()), median=float(np.median(x)),
                mean=float(x.mean()), maximum=float(x.max()))


def duplicates(groups):
    return [v for v in groups.values() if len(v) > 1]


def connected_components(objects):
    centres = np.asarray([o['center_m'] for o in objects])
    radii = np.asarray([o['radius_m'] for o in objects])
    adjacent = np.linalg.norm(centres[:, None] - centres[None, :], axis=-1) <= radii[:, None] + radii[None, :]
    unseen = set(range(len(objects)))
    count = 0
    while unseen:
        queue = [unseen.pop()]
        count += 1
        while queue:
            node = queue.pop()
            new = unseen.intersection(np.where(adjacent[node])[0])
            unseen.difference_update(new)
            queue.extend(new)
    return count


def main(output_dir=HERE):
    started = time.monotonic()
    assert BASE.name == 'generic_motion_v1'
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / 'audit.json'
    if output.exists():
        raise FileExistsError('Preserve this audit; use a new output directory for another execution')
    checks, sources = [], {}

    def check(name, passed, detail=None):
        checks.append(dict(name=name, passed=bool(passed), detail=detail))

    def read_json(path):
        sources[str(path.relative_to(ROOT))] = sha(path)
        return json.loads(path.read_text())

    original_model = read_json(BASE / 'reports/model_audit/audit.json')['model']
    lower, upper = np.array(original_model['lower']), np.array(original_model['upper'])
    rows, scenes, receipts = [], {}, {}
    source_comparison = []
    for split in ('smoke', 'dev', 'test'):
        receipt = read_json(BASE / 'receipts' / f'{split}_generation.json')
        receipts[split] = receipt
        config = read_json(BASE / 'configs' / f'{split}_generator.json')
        check(f'{split}:config_hash', sha(BASE / 'configs' / f'{split}_generator.json') == receipt['protocol']['config_sha256'])
        manifest = BASE / 'manifests' / f'{split}.jsonl'
        sources[str(manifest.relative_to(ROOT))] = sha(manifest)
        check(f'{split}:manifest_hash', sha(manifest) == receipt['manifest_sha256'])
        for rel, expected in receipt['files_sha256'].items():
            check(f'{split}:frozen_file:{rel}', sha(BASE / rel) == expected)
        current_sources = dict(receipt['protocol']['generator_sources_sha256'])
        current_sources['model.py'] = receipt['protocol']['model_source_sha256']
        for rel, expected in current_sources.items():
            current = BASE / rel
            archived = ARCHIVE / current.relative_to(ROOT)
            current_ok = sha(current) == expected
            archived_ok = archived.exists() and sha(archived) == expected
            check(f'{split}:source_provenance:{rel}', current_ok or archived_ok)
            source_comparison.append(dict(split=split, path=rel, frozen_sha256=expected,
                current_sha256=sha(current), current_matches=current_ok,
                archived_original_matches=archived_ok))
        these = [json.loads(line) for line in manifest.read_text().splitlines()]
        check(f'{split}:count', len(these) == receipt['problem_count'])
        for row in these:
            pid, sid = row['problem_id'], row['scene_id']
            index, pair = int(sid.rsplit('-', 1)[1]), int(pid.rsplit('-p', 1)[1])
            ep = row['endpoint_sampling']
            check(f'{pid}:row_hash', digest({k: v for k, v in row.items() if k != 'problem_sha256'}) == row['problem_sha256'])
            check(f'{pid}:domain_seeds', row['scene_seed'] == domain_seed(receipt['seed'], 'scene', split, row['family'], index) and row['endpoint_seed'] == domain_seed(receipt['seed'], 'endpoint', split, row['family'], index, pair))
            check(f'{pid}:distance_bin_cycle', ep['distance_bin'] == config['distance_bin_cycle'][pair % len(config['distance_bin_cycle'])])
            for r, seed in enumerate(row['paired_seeds']):
                check(f'{pid}:replica_seed:{r}', seed['drm_seed'] == r and seed['method_seed'] == domain_seed(receipt['seed'], 'method', split, row['family'], index, pair, r) % 2**32 and seed['solver_seed'] == domain_seed(receipt['seed'], 'solver', split, row['family'], index, pair, r) % 2**32)
            if sid not in scenes:
                scene = read_json(BASE / row['scene_file'])
                scenes[sid] = scene
                random = rng(scene['scene_seed'])
                for _ in range(scene['generation_rejected_layouts'] + 1):
                    proposed = _proposal(scene['family'], random, config)
                check(f'{sid}:geometry_rng_replay', proposed == scene['obstacle_geometry'])
                xyz, cover = voxelize(proposed, config['voxel_radius_m'], config['maximum_voxel_cell_side_m'])
                check(f'{sid}:voxel_reconstruction', np.array_equal(xyz, np.asarray(scene['voxels_xyz']).reshape(-1, 3)) and cover == scene['voxelization'])
            scene = scenes[sid]
            check(f'{pid}:scene_identity', row['obstacle_geometry'] == scene['obstacle_geometry'] and row['voxels_xyz'] == scene['voxels_xyz'] and row['scene_sha256'] == sha(BASE / row['scene_file']))
            with np.load(BASE / row['input_arrays_file'], allow_pickle=False) as data:
                q = data['endpoint_proposal_q']
                free = data['endpoint_proposal_cuda_free']
                clearance = data['endpoint_proposal_independent_clearance_m']
                proposals = rng(row['endpoint_seed']).uniform(lower + ep['joint_margin_rad'], upper - ep['joint_margin_rad'], q.shape)
                check(f'{pid}:all_joint_rng_replay', np.array_equal(proposals, q))
                check(f'{pid}:proposal_array_shape', q.shape == (ep['drawn_configurations'], 7) and free.shape == clearance.shape == (len(q),) and np.isfinite(q).all() and np.isfinite(clearance).all())
                first = goal = None
                considered = collision_rejected = distance_rejected = 0
                for state, legal, distance in zip(q, free, clearance):
                    considered += 1
                    if not legal or distance <= ep['minimum_clearance_m']:
                        collision_rejected += 1
                        continue
                    if first is None:
                        first = state
                        continue
                    norm = np.sqrt(np.mean(((state - first) / (upper - lower))**2))
                    if ep['distance_bin_bounds'][0] <= norm < ep['distance_bin_bounds'][1]:
                        goal = state
                        break
                    distance_rejected += 1
                same_start = row['q_start'] is None if first is None else np.array_equal(first, row['q_start'])
                same_goal = row['q_goal'] is None if goal is None else np.array_equal(goal, row['q_goal'])
                check(f'{pid}:first_acceptable_pair', same_start and same_goal)
                check(f'{pid}:rejection_accounting', considered == ep['considered_configurations'] and collision_rejected == ep['collision_or_clearance_rejections'] and distance_rejected == ep['distance_bin_rejections'] and len(q) - considered == ep['unused_batch_tail_configurations'])
                check(f'{pid}:recorded_model_agreement', np.count_nonzero(free != (clearance > 0)) == ep['checker_disagreements'])
                if row['validation_status'] == 'valid_input':
                    line = data['straight_line_q']
                    length = np.linalg.norm(goal - first)
                    norm = np.sqrt(np.mean(((goal - first) / (upper - lower))**2))
                    count = max(2, int(np.ceil(length / row['straight_line_step_rad'])) + 1)
                    check(f'{pid}:distances', abs(length - row['start_goal_joint_distance']) < 1e-12 and abs(norm - row['normalized_start_goal_joint_distance']) < 1e-12)
                    check(f'{pid}:line_samples', count == row['straight_line_samples'] and np.array_equal(line, np.linspace(first, goal, count)))
                    native, clear = data['straight_line_cuda_free'], data['straight_line_independent_clearance_m']
                    blocked = not (native.all() and (clear > 0).all())
                    check(f'{pid}:line_label', blocked == row['straight_line_collision'] and int((~native).sum()) == row['straight_line_native_colliding_samples'] and int((clear <= 0).sum()) == row['straight_line_independent_colliding_samples'] and float(clear.min()) == row['straight_line_minimum_clearance_m'])
                    check(f'{pid}:endpoint_limits', np.all(first >= lower + ep['joint_margin_rad']) and np.all(first <= upper - ep['joint_margin_rad']) and np.all(goal >= lower + ep['joint_margin_rad']) and np.all(goal <= upper - ep['joint_margin_rad']))
                else:
                    check(f'{pid}:exhausted_invalid', goal is None and len(q) == ep['maximum_attempts'])
            rows.append(row)

    world_groups, endpoints, pairs, scene_seeds, endpoint_seeds = (defaultdict(list) for _ in range(5))
    for sid, scene in scenes.items():
        ordered = sorted(scene['voxels_xyz'])
        key = digest(dict(xyz=ordered, radius=scene['voxel_radius_m']))
        world_groups[key].append(dict(scene_id=sid, split=scene['split'], family=scene['family']))
        scene_seeds[scene['scene_seed']].append(sid)
    for row in rows:
        endpoint_seeds[row['endpoint_seed']].append(row['problem_id'])
        if row['validation_status'] != 'valid_input':
            continue
        for side in ('q_start', 'q_goal'):
            endpoints[digest(row[side])].append(dict(problem_id=row['problem_id'], endpoint=side))
        pairs[digest(sorted([row['q_start'], row['q_goal']]))].append(row['problem_id'])
    repeated_worlds = duplicates(world_groups)
    check('no_unintended_exact_world_duplicates', not any(any(s['family'] != 'open_control' for s in group) for group in repeated_worlds))
    check('unique_scene_seeds', not duplicates(scene_seeds))
    check('unique_endpoint_seeds', not duplicates(endpoint_seeds))
    check('no_exact_individual_endpoint_duplicates', not duplicates(endpoints))
    check('no_exact_unordered_endpoint_pair_duplicates', not duplicates(pairs))
    distributions = []
    for split in ('smoke', 'dev', 'test'):
        for family in ['all', 'sparse_clutter', 'dense_clutter', 'bottleneck', 'route_choice', 'open_control']:
            group = [r for r in rows if r['split'] == split and (family == 'all' or r['family'] == family)]
            valid = [r for r in group if r['validation_status'] == 'valid_input']
            subset_scenes = [scenes[k] for k in sorted({r['scene_id'] for r in group})]
            distributions.append(dict(split=split, family=family, registered=len(group), valid=len(valid),
                statuses=dict(Counter(r['validation_status'] for r in group)),
                scene_ids=len(subset_scenes), valid_scene_ids=len({r['scene_id'] for r in valid}),
                unique_obstacle_geometries=len({s['obstacle_geometry_sha256'] for s in subset_scenes}),
                valid_unique_obstacle_geometries=len({r['obstacle_geometry_sha256'] for r in valid}),
                requested_distance_bins=dict(Counter(r['endpoint_sampling']['distance_bin'] for r in group)),
                valid_distance_bins=dict(Counter(r['endpoint_sampling']['distance_bin'] for r in valid)),
                straight_line_blocked=sum(r['straight_line_collision'] for r in valid),
                straight_line_free=len(valid) - sum(r['straight_line_collision'] for r in valid),
                normalized_distance=desc([r['normalized_start_goal_joint_distance'] for r in valid]),
                distance_rad=desc([r['start_goal_joint_distance'] for r in valid]),
                minimum_endpoint_clearance_m=desc([min(r['endpoint_clearances_m']) for r in valid]),
                drawn_proposals=sum(r['endpoint_sampling']['drawn_configurations'] for r in group),
                considered_proposals=sum(r['endpoint_sampling']['considered_configurations'] for r in group),
                collision_or_clearance_rejections=sum(r['endpoint_sampling']['collision_or_clearance_rejections'] for r in group),
                distance_bin_rejections=sum(r['endpoint_sampling']['distance_bin_rejections'] for r in group),
                rejected_scene_proposals=sum(s['generation_rejected_layouts'] for s in subset_scenes),
                obstacle_count=desc([len(s['obstacle_geometry']) for s in subset_scenes]),
                voxel_count=desc([len(s['voxels_xyz']) for s in subset_scenes])))

    geometry_details = []
    for sid, scene in scenes.items():
        objects = scene['obstacle_geometry']
        row = dict(scene_id=sid, split=scene['split'], family=scene['family'],
                   obstacle_count=len(objects), voxel_count=len(scene['voxels_xyz']),
                   sphere_radius_m=desc([o['radius_m'] for o in objects if o['shape'] == 'sphere']),
                   box_side_m=desc([d for o in objects if o['shape'] == 'box' for d in o['size_m']]))
        if objects:
            row.update(center_min_m=np.min([o['center_m'] for o in objects], axis=0).tolist(),
                       center_max_m=np.max([o['center_m'] for o in objects], axis=0).tolist())
        if scene['family'] == 'bottleneck':
            a, b = objects
            yaw, center = a['yaw_rad'], np.mean([a['center_m'], b['center_m']], axis=0)
            normal = np.array([-np.sin(yaw), np.cos(yaw), 0.])
            direction = np.array([np.cos(yaw), np.sin(yaw), 0.])
            projections = []
            xyz = np.array(scene['voxels_xyz'])
            for coverage in scene['voxelization']['obstacles']:
                begin = coverage['first_voxel']
                end = begin + coverage['voxel_count']
                projections.append(xyz[begin:end] @ direction)
            cover_gap = projections[1].min() - projections[0].max() - 2 * scene['voxel_radius_m']
            valid = [r for r in rows if r['scene_id'] == sid and r['validation_status'] == 'valid_input']
            opposite = 0
            for r in valid:
                projected = (np.asarray(r['endpoint_tcp_world_m']) - center) @ normal
                opposite += int(np.min(projected) < -a['size_m'][1] / 2 and np.max(projected) > a['size_m'][1] / 2)
            row.update(workspace_gap_m=a['workspace_gap_m'], yaw_rad=yaw,
                voxel_cover_projected_gap_m=float(cover_gap), valid_pairs=len(valid),
                tcp_endpoints_strictly_on_opposite_wall_sides=opposite,
                diagnostic_only='TCP plane-side count does not establish required passage or C-space bottleneck; finite walls can be bypassed.')
        if scene['family'] == 'route_choice':
            clusters = [[o for o in objects if o['cluster'] == i] for i in (0, 1)]
            pair_clearance = [np.linalg.norm(np.array(a['center_m']) - b['center_m']) - a['radius_m'] - b['radius_m'] for a in clusters[0] for b in clusters[1]]
            row.update(actual_cluster_centroid_distance_m=float(np.linalg.norm(np.mean([o['center_m'] for o in clusters[0]],axis=0) - np.mean([o['center_m'] for o in clusters[1]],axis=0))),
                minimum_cross_cluster_source_sphere_gap_m=float(min(pair_clearance)),
                source_sphere_connected_components_by_cluster=[connected_components(c) for c in clusters])
        geometry_details.append(row)

    exposure = []
    exposed_cases = set()
    for manifest in sorted((BASE / 'results').glob('*/run_manifest.json')):
        run = json.loads(manifest.read_text())
        if run.get('split') != 'test':
            continue
        sources[str(manifest.relative_to(ROOT))] = sha(manifest)
        results = list(manifest.parent.glob('jobs/*/attempts/*/result.json'))
        actual_cases, statuses = set(), Counter()
        result_hashes = {}
        for path in results:
            result = json.loads(path.read_text())
            case = result.get('case_id') or result.get('problem_id') or result.get('identity', {}).get('case_id')
            if not case:
                # Resolve the registered immutable job rather than parsing a name.
                job_id = path.parents[2].name
                job = next((j for j in run['jobs'] if j['job_id'] == job_id), None)
                case = job['case_id'] if job else None
            if case:
                actual_cases.add(case)
            statuses[str(result.get('status'))] += 1
            result_hashes[str(path.relative_to(ROOT))] = sha(path)
        exposed_cases.update(actual_cases)
        exposure.append(dict(run=manifest.parent.name, created_utc=run.get('created_utc'),
            source_code_commit=run.get('source_code_commit'), formal_test_run=run.get('formal_test_run'),
            planned_slots=len(run['jobs']), saved_result_count=len(results),
            actual_case_count=len(actual_cases), actual_case_ids=sorted(actual_cases),
            saved_statuses=dict(statuses), result_hashes_sha256=digest(result_hashes)))

    ast_imports = {}
    for filename in ('scene.py', 'endpoints.py', 'manifest.py'):
        path = BASE / 'generator' / filename
        tree = ast.parse(path.read_text())
        imports = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.extend(x.name for x in node.names)
            elif isinstance(node, ast.ImportFrom):
                imports.append('.' * node.level + str(node.module))
        ast_imports[filename] = imports
    invalid = [r for r in rows if r['validation_status'] != 'valid_input']
    report = dict(schema='benchmark_design_audit_v2_local',
        command='bash local/python.sh -m benchmarks.generic_motion_v1.reports.benchmark_design_audit_v2.local_design.audit',
        source_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        cpu_only=True, new_collision_checks=0, new_planning_calls=0, frozen_inputs_modified=False,
        limitations=['Stored collision labels were replayed for accounting; no new collision oracle was called.',
            'Exact duplicate checks do not establish statistical independence or exclude near duplicates.',
            'Workspace geometry labels do not establish required C-space passages or multiple homotopy classes.',
            'Historical test exposure is measured from saved runs; causal tuning decisions cannot be inferred from timestamps alone.'],
        check_count=len(checks), checks_passed=sum(x['passed'] for x in checks),
        all_integrity_and_replay_checks_passed=all(x['passed'] for x in checks),
        failed_checks=[x for x in checks if not x['passed']], checks=checks,
        distributions=distributions, geometry_details=geometry_details,
        repeated_geometry_groups=repeated_worlds, duplicate_scene_seeds=duplicates(scene_seeds),
        duplicate_endpoint_seeds=duplicates(endpoint_seeds), duplicate_endpoints=duplicates(endpoints),
        duplicate_unordered_pairs=duplicates(pairs),
        invalid_inputs=[dict(problem_id=r['problem_id'], scene_id=r['scene_id'], split=r['split'],
            family=r['family'], q_start_is_none=r['q_start'] is None, q_goal_is_none=r['q_goal'] is None,
            sampling=r['endpoint_sampling']) for r in invalid],
        frozen_vs_current_sources=source_comparison, generator_imports=ast_imports,
        existing_test_exposure=dict(run_count=len(exposure), saved_result_count=sum(r['saved_result_count'] for r in exposure),
            unique_case_count=len(exposed_cases), case_ids=sorted(exposed_cases), runs=exposure),
        source_files_sha256=sources, elapsed_seconds=time.monotonic()-started)
    output.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n')
    (output_dir / 'checks.json').write_text(json.dumps(checks, indent=2) + '\n')
    print(json.dumps({k: report[k] for k in ['check_count', 'checks_passed', 'all_integrity_and_replay_checks_passed', 'failed_checks', 'elapsed_seconds']}, indent=2))
    print(json.dumps({'existing_test_exposure': {k: report['existing_test_exposure'][k] for k in ('run_count', 'saved_result_count', 'unique_case_count')}}, indent=2))
    return 0 if report['all_integrity_and_replay_checks_passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=HERE)
    args = parser.parse_args()
    raise SystemExit(main(args.output))
