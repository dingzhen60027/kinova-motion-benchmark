"""Read-only, CPU-only qualification of the already-frozen formal test inputs.

No planner or benchmark Model/CUDA method is imported. Diagnostic states are
the first 128 stored proposals of each wholly invalid scene, not new samples.
"""
from __future__ import annotations

from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.distance import cdist

from benchmarks.generic_motion_v1.generator.integrity import preflight
from benchmarks.generic_motion_v1.generator.scene import digest, domain_seed
from benchmarks.generic_motion_v1.generator.voxelize import voxelize

BASE = Path(__file__).resolve().parents[2]
ROOT = BASE.parents[1]
OUT = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def quantiles(values):
    return np.quantile(values, [0, .25, .5, .75, 1]).tolist() if len(values) else None


def fixed_cover_distance(scene, inventory):
    xyz = np.asarray(scene['voxels_xyz']).reshape(-1, 3)
    values = np.full(len(xyz), np.inf)
    for entry, pose in zip(inventory['geometries'], inventory['zero_configuration_geometry_world_poses']):
        if entry['moving_robot']:
            continue
        pose = np.asarray(pose)
        local = (xyz - pose[:3, 3]) @ pose[:3, :3]
        assert entry['shape'] == 'box'
        delta = np.abs(local) - np.asarray(entry['dimensions_m']) / 2
        distances = np.linalg.norm(np.maximum(delta, 0), axis=1) + np.minimum(delta.max(axis=1), 0) - .04
        values = np.minimum(values, distances)
    return values


def physical_diagnosis(scene, q, inventory):
    from pydrake.all import (RobotDiagramBuilder, LoadModelDirectives, ProcessModelDirectives,
        RigidTransform, RotationMatrix, Sphere, Box, CoulombFriction, Role)
    assets = ROOT / 'csdecomp/tests/test_assets'
    builder = RobotDiagramBuilder()
    parser, plant, sg = builder.parser(), builder.plant(), builder.scene_graph()
    for name, path in [('test_assets', assets), ('kortex_description', assets / 'kinova/kortex_description'),
                       ('robotiq_arg85_description', assets / 'robotiq')]:
        parser.package_map().Add(name, str(path))
    ProcessModelDirectives(LoadModelDirectives(str(assets / 'directives/kinova_sens_on_table.yml')), plant, parser)
    source_ids = set()
    for obstacle in scene['obstacle_geometry']:
        if obstacle['shape'] == 'sphere':
            shape = Sphere(obstacle['radius_m'])
            rotation = RotationMatrix()
        else:
            shape = Box(*obstacle['size_m'])
            rotation = RotationMatrix.MakeZRotation(obstacle['yaw_rad'])
        gid = plant.RegisterCollisionGeometry(plant.world_body(),
            RigidTransform(rotation, obstacle['center_m']), shape,
            f"readiness_source_{obstacle['obstacle_id']}", CoulombFriction(.5, .5))
        source_ids.add(gid)
    plant.Finalize()
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    pc, sc = plant.GetMyContextFromRoot(context), sg.GetMyContextFromRoot(context)
    query = lambda: sg.get_query_output_port().Eval(sc)
    inspector = query().inspector()
    ids = {inspector.GetName(g): g for frame in inspector.GetAllFrameIds()
           for g in inspector.GetGeometries(frame, Role.kProximity)}
    assert plant.GetPositionNames() == inventory['joint_names']
    original_names = [g['name'] for g in inventory['geometries']]
    expected_pairs = {frozenset(original_names[i] for i in pair) for pair in inventory['checked_pairs']}
    actual_pairs = {frozenset(inspector.GetName(g) for g in pair) for pair in inspector.GetCollisionCandidates()
                    if not any(g in source_ids for g in pair)}
    assert actual_pairs == expected_pairs
    assert len(inspector.GetCollisionCandidates()) == len(expected_pairs) + 15 * len(source_ids)
    moving = [entry for entry in inventory['geometries'] if entry['moving_robot']]
    xyz = np.asarray(scene['voxels_xyz'])
    raw_static, raw_physical, raw_voxel = [], [], []
    physical_counts, voxel_counts = Counter(), Counter()
    for state in q:
        plant.SetPositions(pc, state)
        actual = query()
        static, physical = np.inf, np.inf
        physical_colliding = set()
        for result in actual.ComputeSignedDistancePairwiseClosestPoints():
            if result.id_A in source_ids or result.id_B in source_ids:
                physical = min(physical, result.distance)
                if result.distance <= 0:
                    robot = result.id_B if result.id_A in source_ids else result.id_A
                    physical_colliding.add(inspector.GetName(robot))
            else:
                static = min(static, result.distance)
        voxel = np.inf
        for entry in moving:
            X = actual.GetPoseInWorld(ids[entry['name']])
            local = (xyz - X.translation()) @ X.rotation().matrix()
            assert entry['shape'] == 'box'
            delta = np.abs(local) - np.asarray(entry['dimensions_m']) / 2
            value = float(np.min(np.linalg.norm(np.maximum(delta, 0), axis=1) + np.minimum(delta.max(axis=1), 0) - .04))
            voxel = min(voxel, value)
            if value <= 0:
                voxel_counts[entry['name']] += 1
        physical_counts.update(physical_colliding)
        raw_static.append(static)
        raw_physical.append(physical)
        raw_voxel.append(voxel)
    static, physical, cover = map(np.asarray, (raw_static, raw_physical, raw_voxel))
    summary = dict(sample_count=len(q), sampling='first 128 persisted proposals of p00; no new sampling',
        static_base_only_free_count=int((static > .0001).sum()),
        physical_obstacles_only_free_count=int((physical > .0001).sum()),
        full_original_geometry_free_count=int(((static > .0001) & (physical > .0001)).sum()),
        conservative_cover_only_free_count=int((cover > .0001).sum()),
        full_frozen_geometry_free_count=int(((static > .0001) & (cover > .0001)).sum()),
        original_geometry_free_but_cover_rejected=int(((static > .0001) & (physical > .0001) & (cover <= .0001)).sum()),
        physical_collision_states_by_robot_geometry=dict(physical_counts),
        cover_collision_states_by_robot_geometry=dict(voxel_counts),
        static_clearance_quantiles_m=quantiles(static), original_obstacle_clearance_quantiles_m=quantiles(physical),
        frozen_cover_clearance_quantiles_m=quantiles(cover),
        global_infeasibility_proven=False, collision_geometry_changed_for_acceptance=False)
    return summary, dict(q=q, static_clearance_m=static, original_geometry_clearance_m=physical, frozen_cover_clearance_m=cover)


def near_scenes(scenes, left_ids, right_ids=None):
    clouds = {sid: np.asarray(scenes[sid]['voxels_xyz']) for sid in scenes if scenes[sid]['voxels_xyz']}
    trees = {sid: cKDTree(cloud) for sid, cloud in clouds.items()}
    best, pairs = None, []
    for i, left in enumerate(left_ids):
        if left not in trees:
            continue
        for right in (right_ids if right_ids is not None else left_ids[i+1:]):
            if right not in trees:
                continue
            distance = max(float(trees[left].query(clouds[right])[0].max()),
                           float(trees[right].query(clouds[left])[0].max()))
            entry = dict(left=left, right=right, symmetric_voxel_centre_hausdorff_m=distance)
            if best is None or distance < best['symmetric_voxel_centre_hausdorff_m']:
                best = entry
            if distance <= .01:
                pairs.append(entry)
    return dict(metric='symmetric Hausdorff distance of frozen equal-radius sphere centres; not a topology/feasibility metric',
        diagnostic_threshold_m=.01, closest_pair=best, pairs_at_or_below_threshold=pairs,
        open_control_excluded=True, criterion_used_to_select_or_regenerate_data=False)


def main():
    start = time.perf_counter()
    before = {split: sha(BASE / 'manifests' / f'{split}.jsonl') for split in ('smoke', 'dev', 'test')}
    all_rows = {split: [json.loads(line) for line in (BASE / 'manifests' / f'{split}.jsonl').read_text().splitlines()]
                for split in ('smoke', 'dev', 'test')}
    rows = all_rows['test']
    config = read(BASE / 'configs/test_generator.json')
    receipt = read(BASE / 'receipts/test_generation.json')
    prior = read(BASE / 'reports/qualification/generator_test_audit.json')
    prior_receipt = read(BASE / 'reports/qualification/generator_reaudit_receipt.json')
    inventory = read(BASE / 'reports/model_audit/audit.json')['model']
    errors = []
    preflight(BASE / 'manifests/test.jsonl', rows, config, receipt, sha)
    hash_failures = [path for path, expected in receipt['files_sha256'].items() if sha(BASE / path) != expected]
    errors.extend(f'input_hash:{path}' for path in hash_failures)
    assert prior_receipt['splits']['test']['report_sha256'] == sha(BASE / 'reports/qualification/generator_test_audit.json')
    assert prior['passed'] and prior['manifest_sha256'] == before['test']
    source_asset_failures = [p for p, expected in inventory['source_files_sha256'].items() if sha(ROOT / p) != expected]
    errors.extend(f'canonical_source_asset_hash:{p}' for p in source_asset_failures)
    low, high = np.asarray(inventory['lower']), np.asarray(inventory['upper'])
    scenes = {r['scene_id']: read(BASE / r['scene_file']) for group in all_rows.values() for r in group}
    scene_results = {}
    for sid in sorted({r['scene_id'] for r in rows}):
        scene = scenes[sid]
        xyz, certificate = voxelize(scene['obstacle_geometry'])
        if not np.array_equal(xyz, np.asarray(scene['voxels_xyz']).reshape(-1, 3)) or certificate != scene['voxelization']:
            errors.append(f'voxel_regeneration:{sid}')
        fixed = fixed_cover_distance(scene, inventory)
        if (fixed <= 0).any():
            errors.append(f'fixed_robot_or_equipment_penetration:{sid}')
        scene_results[sid] = dict(family=scene['family'], obstacle_count=len(scene['obstacle_geometry']),
            voxel_count=len(xyz), geometry_sha256=scene['obstacle_geometry_sha256'],
            layout_rejections=scene['generation_rejected_layouts'],
            fixed_geometry_overlapping_voxels=int((fixed <= 0).sum()),
            fixed_geometry_minimum_clearance_m=float(fixed.min()) if len(fixed) else None,
            minimum_cover_margin_m=min((x['coverage_margin_m'] for x in certificate['obstacles']), default=None),
            maximum_outward_overapproximation_bound_m=max((x['maximum_outward_overapproximation_m'] for x in certificate['obstacles']), default=0))
    cases, invalid_by_scene = [], {}
    for row in rows:
        if digest({k:v for k,v in row.items() if k != 'problem_sha256'}) != row['problem_sha256']:
            errors.append(f'problem_hash:{row["problem_id"]}')
        with np.load(BASE / row['input_arrays_file'], allow_pickle=False) as data:
            q = data['endpoint_proposal_q']; free = data['endpoint_proposal_cuda_free']; clear = data['endpoint_proposal_independent_clearance_m']
            actual = np.random.Generator(np.random.PCG64(row['endpoint_seed'])).uniform(low+.02, high-.02, q.shape)
            assert np.array_equal(q, actual)
            assert np.all(q >= low+.02) and np.all(q <= high-.02)
            start_q = goal_q = None
            considered = collision_rejections = distance_rejections = 0
            for state, native, clearance in zip(q, free, clear):
                considered += 1
                if not native or clearance <= .0001:
                    collision_rejections += 1; continue
                if start_q is None:
                    start_q = state; continue
                normalized = float(np.sqrt(np.mean(((state-start_q)/(high-low))**2)))
                a,b = row['endpoint_sampling']['distance_bin_bounds']
                if a <= normalized < b:
                    goal_q = state; break
                distance_rejections += 1
            for field, expected in [('q_start', start_q), ('q_goal', goal_q)]:
                assert (expected is None and row[field] is None) or np.array_equal(expected, row[field])
            for field, actual in [('considered_configurations',considered),('collision_or_clearance_rejections',collision_rejections),('distance_bin_rejections',distance_rejections)]:
                assert row['endpoint_sampling'][field] == actual
            result = dict(problem_id=row['problem_id'], scene_id=row['scene_id'], validation_status=row['validation_status'],
                proposal_count=len(q), native_free_proposals=int(free.sum()), independent_free_proposals=int((clear>0).sum()),
                minimum_clearance_rule_pass_proposals=int((free & (clear>.0001)).sum()),
                raw_clearance_quantiles_m=quantiles(clear), collision_rejections=collision_rejections,
                distance_rejections=distance_rejections, considered=considered,
                endpoint_seed=row['endpoint_seed'], raw_arrays_sha256=sha(BASE/row['input_arrays_file']))
            cases.append(result)
            if row['validation_status'] != 'valid_input':
                invalid_by_scene.setdefault(row['scene_id'], []).append(result)
    invalid_diagnostics, diagnostic_arrays = {}, {}
    for sid, group in invalid_by_scene.items():
        row = next(r for r in rows if r['scene_id']==sid)
        with np.load(BASE / row['input_arrays_file']) as data:
            q = data['endpoint_proposal_q'][:128].copy()
        print(f'CPU-only original-geometry diagnosis: {sid}', flush=True)
        result, arrays = physical_diagnosis(scenes[sid], q, inventory)
        result.update(problem_count=len(group), raw_proposal_count=sum(x['proposal_count'] for x in group),
            all_recorded_native_reject=all(x['native_free_proposals']==0 for x in group),
            all_recorded_independent_reject=all(x['independent_free_proposals']==0 for x in group),
            no_start_found=all(r['q_start'] is None for r in rows if r['scene_id']==sid),
            scene_details=scene_results[sid])
        invalid_diagnostics[sid] = result
        diagnostic_arrays.update({f'{sid}__{key}':value for key,value in arrays.items()})
    families = {}
    for family in sorted({r['family'] for r in rows}):
        selected = [r for r in rows if r['family']==family]
        valid = [r for r in selected if r['validation_status']=='valid_input']
        layouts = [scene_results[sid] for sid in sorted({r['scene_id'] for r in selected})]
        families[family] = dict(total=len(selected), valid=len(valid), invalid=len(selected)-len(valid),
            scene_ids=len(layouts), eligible_scene_ids=len({r['scene_id'] for r in valid}),
            actual_geometry_count=len({s['geometry_sha256'] for s in layouts}),
            independent_random_geometries=0 if family=='open_control' else len(layouts),
            obstacle_count_quantiles=quantiles([s['obstacle_count'] for s in layouts]),
            voxel_count_quantiles=quantiles([s['voxel_count'] for s in layouts]),
            distance_bins_all=dict(Counter(r['endpoint_sampling']['distance_bin'] for r in selected)),
            distance_bins_valid=dict(Counter(r['endpoint_sampling']['distance_bin'] for r in valid)),
            normalized_distance_quantiles=quantiles([r['normalized_start_goal_joint_distance'] for r in valid]),
            distance_rad_quantiles=quantiles([r['start_goal_joint_distance'] for r in valid]),
            straight_line_blocked=sum(r['straight_line_collision'] is True for r in valid),
            straight_line_free=sum(r['straight_line_collision'] is False for r in valid))
    test_scenes = sorted({r['scene_id'] for r in rows})
    other_scenes = sorted({r['scene_id'] for split,group in all_rows.items() if split!='test' for r in group})
    near = dict(within_test=near_scenes(scenes,test_scenes), cross_split=near_scenes(scenes,test_scenes,other_scenes))
    valid = [r for r in rows if r['validation_status']=='valid_input']
    other_valid = [r for split,group in all_rows.items() if split!='test' for r in group if r['validation_status']=='valid_input']
    qa = [(r['problem_id'], name, r[name]) for r in valid for name in ('q_start','q_goal')]
    qb = [(r['problem_id'], name, r[name]) for r in other_valid for name in ('q_start','q_goal')]
    d = cdist(np.array([v[2] for v in qa])/(high-low),np.array([v[2] for v in qb])/(high-low))/np.sqrt(7)
    a,b = np.unravel_index(d.argmin(), d.shape)
    near['cross_split_individual_endpoints'] = dict(metric='RMS joint range-normalized difference',
        diagnostic_threshold=.01, pairs_at_or_below_threshold=int((d<=.01).sum()),
        minimum_distance=float(d[a,b]), closest_test=list(qa[a][:2]), closest_other=list(qb[b][:2]),
        criterion_used_to_select_or_regenerate_data=False)
    cross = {}
    for field in ('scene_seed','endpoint_seed','obstacle_geometry_sha256'):
        test_values = {r[field] for r in rows if field!='obstacle_geometry_sha256' or r['family']!='open_control'}
        others = {r[field] for split,group in all_rows.items() if split!='test' for r in group if field!='obstacle_geometry_sha256' or r['family']!='open_control'}
        cross[field] = len(test_values & others)
        if cross[field]: errors.append(f'cross_split_overlap:{field}')
    after = {split:sha(BASE/'manifests'/f'{split}.jsonl') for split in before}
    if after!=before: errors.append('manifest_changed_during_readiness')
    evidence = {str(path.relative_to(BASE)):sha(path) for path in [BASE/'manifests/test.jsonl',BASE/'configs/test_generator.json',
        BASE/'receipts/test_generation.json',BASE/'reports/qualification/generator_test_audit.json',
        BASE/'reports/qualification/generator_reaudit_receipt.json',BASE/'task_spec/BENCHMARK_SPEC.md',Path(__file__)]}
    result = dict(read_only=True, gpu_calls=0, planner_calls=0, formal_test_planning_executed=False,
        source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        elapsed_seconds=time.perf_counter()-start, command='bash local/python.sh -m benchmarks.generic_motion_v1.reports.baseline_study.test_input_readiness',
        passed=not errors, errors=errors, all_420_valid=False, valid_problem_count=len(valid), invalid_problem_count=len(rows)-len(valid),
        total_problem_count=len(rows), scene_id_count=len(test_scenes), input_files_checked=len(receipt['files_sha256']),
        input_hash_failures=hash_failures, canonical_asset_hash_failures=source_asset_failures,
        prior_full_gpu_input_audit_verified=True, new_raw_rng_and_acceptance_replay_proposals=sum(c['proposal_count'] for c in cases),
        new_cpu_physical_diagnostic_states=sum(v['sample_count'] for v in invalid_diagnostics.values()),
        invalid_scene_diagnoses=invalid_diagnostics, family_distribution=families, cases=cases,
        exact_cross_split_overlap_counts=cross, near_duplicate_diagnostics=near, scene_details=scene_results,
        evidence_sha256=evidence, manifest_sha256_before=before, manifest_sha256_after=after,
        decision=dict(v1_usable_as_fixed_test_with_declared_invalid_inputs=not errors,
            recommended_primary_denominator=395, retained_invalid_inputs=25,
            v2_required_to_claim_420_valid_and_full_family_balance=True,
            v2_required_to_use_current_395_valid=False,
            formal_evaluation_authorized_by_this_audit=False,
            global_path_feasibility_unknown=True))
    with (OUT/'test_input_readiness.json').open('x') as f: json.dump(result,f,indent=2,sort_keys=True); f.write('\n')
    np.savez_compressed(OUT/'test_input_readiness_cpu_diagnostics.npz', **diagnostic_arrays)
    print(json.dumps({k:v for k,v in result.items() if k not in ('cases','scene_details','evidence_sha256')},indent=2),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
