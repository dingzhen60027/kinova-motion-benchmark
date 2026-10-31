"""Read-only geometry diagnostics; no planner calls or dataset mutations.

Compare the frozen cover to source primitives, and attribute straight-line
collisions to the fixed world versus added obstacles. Diagnostic classifications
never replace frozen labels. All samples are finite numerical checks.
"""
from pathlib import Path
from collections import Counter, defaultdict
from datetime import datetime, timezone
import argparse
import hashlib
import json
import time

import numpy as np

from benchmarks.generic_motion_v1.model import Model, ASSETS, DIRECTIVE
from benchmarks.generic_motion_v1.generator.voxelize import voxelize

ROOT = Path(__file__).resolve().parents[4]
BENCH = ROOT / 'benchmarks/generic_motion_v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_model(obstacles, canonical):
    from pydrake.all import (RobotDiagramBuilder, ProcessModelDirectives,
                            LoadModelDirectives, RigidTransform, RotationMatrix,
                            Box, Sphere, CoulombFriction)
    builder = RobotDiagramBuilder()
    parser, plant = builder.parser(), builder.plant()
    parser.package_map().Add('test_assets', str(ASSETS))
    parser.package_map().Add('kortex_description', str(ASSETS/'kinova/kortex_description'))
    parser.package_map().Add('robotiq_arg85_description', str(ASSETS/'robotiq'))
    ProcessModelDirectives(LoadModelDirectives(str(DIRECTIVE)), plant, parser)
    added_ids = []
    for i, ob in enumerate(obstacles):
        shape = Sphere(ob['radius_m']) if ob['shape'] == 'sphere' else Box(*ob['size_m'])
        pose = RigidTransform(RotationMatrix.MakeZRotation(ob.get('yaw_rad', 0)), ob['center_m'])
        added_ids.append(plant.RegisterCollisionGeometry(plant.world_body(), pose, shape,
                                        f'audit_source_obstacle_{i}', CoulombFriction(.5, .5)))
    graph = builder.scene_graph()
    plant.Finalize()
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    pc = plant.GetMyContextFromRoot(context)
    sc = graph.GetMyContextFromRoot(context)
    query = graph.get_query_output_port().Eval(sc)
    assert plant.GetPositionNames() == canonical.joint_names
    assert np.allclose(plant.GetPositionLowerLimits(), canonical.lower, rtol=0, atol=2e-7)
    assert np.allclose(plant.GetPositionUpperLimits(), canonical.upper, rtol=0, atol=2e-7)
    assert len(query.inspector().GetCollisionCandidates()) == len(canonical.pairs) + len(obstacles)*len(canonical.robot_geometry_ids)
    inspector = query.inspector()
    actual_pairs = {frozenset(inspector.GetName(g) for g in pair)
                    for pair in inspector.GetCollisionCandidates()}
    expected_pairs = {frozenset(canonical.inspector.GetName(canonical._drake_ids[i]) for i in pair)
                      for pair in canonical.pairs}
    robot_names = [canonical.inspector.GetName(canonical._drake_ids[i]) for i in canonical.robot_geometry_ids]
    expected_pairs.update(frozenset((name, inspector.GetName(g))) for name in robot_names for g in added_ids)
    assert actual_pairs == expected_pairs, 'Source geometry collision pair identities differ'

    def distances(qs):
        values = []
        for q in qs:
            plant.SetPositions(pc, q)
            query = graph.get_query_output_port().Eval(sc)
            values.append(min((d.distance for d in query.ComputeSignedDistancePairwiseClosestPoints()), default=float('inf')))
        return np.array(values)
    # Closure keeps the diagram/context alive.
    return distances, (diagram, context)


def main(output):
    out = Path(output)
    out.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    inputs = [BENCH/'manifests'/f'{s}.jsonl' for s in ('smoke', 'dev', 'test')]
    rows = [json.loads(line) for p in inputs for line in p.read_text().splitlines()]
    for name in ('robot', 'generator', 'evaluation'):
        inputs.append(BENCH/'configs'/f'{name}.json')
    inputs += [BENCH/'model.py', BENCH/'generator/voxelize.py', Path(__file__)]
    scenes = {}
    by_scene = defaultdict(list)
    for row in rows:
        by_scene[row['scene_id']].append(row)
        scenes[row['scene_id']] = BENCH/row['scene_file']
    inputs += sorted(set(scenes.values()))
    initial = {str(p.relative_to(ROOT)): sha(p) for p in inputs}
    model = Model()
    robot = json.loads((BENCH/'configs/robot.json').read_text())
    assert model.robot_sha256 == robot['robot_sha256']
    assert model.collision_geometry_sha256 == robot['collision_geometry_sha256']
    random = np.random.Generator(np.random.PCG64(2026103001))
    q_all = random.uniform(model.lower+.02, model.upper-.02, (2048, 7))
    base_native = model.cuda(q_all)
    base_distance = model.clearance(q_all)
    base_disagree = np.flatnonzero((base_native != (base_distance > 0)) & (abs(base_distance)>model.contact_band_m))
    q_pool = q_all[base_native & (base_distance>1e-4)][:128]
    assert len(q_pool) == 128
    np.savez_compressed(out/'configuration_pool.npz', all_q=q_all, base_cuda=base_native,
                        base_clearance=base_distance, selected_q=q_pool)
    scene_records, line_records, errors = [], [], []
    if len(base_disagree):
        errors.append('base_checker_disagreement')
    for number, (sid, path) in enumerate(sorted(scenes.items())):
        scene = json.loads(path.read_text())
        cover, certificate = voxelize(scene['obstacle_geometry'])
        assert np.array_equal(cover, np.array(scene['voxels_xyz']).reshape(-1, 3))
        assert certificate == scene['voxelization']
        source, keepalive = source_model(scene['obstacle_geometry'], model)
        model.set_voxels(cover)
        native = model.cuda(q_pool)
        cover_d = model.clearance(q_pool)
        source_d = source(q_pool)
        band = model.contact_band_m
        mismatch = (native != (cover_d>0)) & (abs(cover_d)>band)
        # A free cover state must also be free under the contained source geometry.
        missed = native & (cover_d>band) & (source_d < -band)
        extra = (source_d>band) & (cover_d < -band)
        valid_rows = [r for r in by_scene[sid] if r['validation_status']=='valid_input']
        endpoint_q = np.array([r[k] for r in valid_rows for k in ('q_start','q_goal')]).reshape(-1,7)
        end_native = model.cuda(endpoint_q)
        end_cover = model.clearance(endpoint_q)
        end_source = source(endpoint_q)
        if np.any(mismatch): errors.append(f'checker_disagreement:{sid}')
        if np.any(missed): errors.append(f'cover_misses_source:{sid}')
        if not np.all(end_native & (end_cover>1e-4) & (end_source>band)):
            errors.append(f'invalid_accepted_endpoint:{sid}')
        records = certificate['obstacles']
        record = dict(scene_id=sid, split=scene['split'], family=scene['family'],
                      geometry_sha256=scene['obstacle_geometry_sha256'],
                      generation_status=scene['generation_status'],
                      source_collision_pair_identities_verified=True,
                      q_count=len(q_pool), source_free=int((source_d>band).sum()),
                      cover_free=int((native & (cover_d>band)).sum()),
                      source_free_cover_colliding=int(extra.sum()), cover_misses_source=int(missed.sum()),
                      cuda_drake_disagreement_outside_band=int(mismatch.sum()),
                      accepted_endpoint_count=len(endpoint_q),
                      min_endpoint_source_clearance_m=float(end_source.min()) if len(end_source) else None,
                      min_endpoint_cover_clearance_m=float(end_cover.min()) if len(end_cover) else None,
                      max_cover_outward_bound_m=max((r['maximum_outward_overapproximation_m'] for r in records), default=0),
                      min_cell_coverage_margin_m=min((r['coverage_margin_m'] for r in records), default=None))
        np.savez_compressed(out/f'{sid}.npz', q=q_pool, source_clearance=source_d,
                            cover_clearance=cover_d, cuda_free=native, endpoints=endpoint_q,
                            endpoint_source_clearance=end_source, endpoint_cover_clearance=end_cover,
                            endpoint_cuda_free=end_native)
        scene_records.append(record)
        for row in valid_rows:
            start, goal = np.array(row['q_start']), np.array(row['q_goal'])
            distance = np.linalg.norm(goal-start)
            coarse = np.linspace(start,goal,int(np.ceil(distance/.04))+1)
            fine = np.linspace(start,goal,int(np.ceil(distance/.005))+1)
            model.set_voxels()
            base_coarse = model.cuda(coarse)
            base_fine = model.cuda(fine)
            model.set_voxels(cover)
            full_coarse = model.cuda(coarse)
            full_fine = model.cuda(fine)
            # Recheck every newly detected collision in the independent checker.
            bad = np.flatnonzero(~full_fine)
            new_label = bool(full_fine.all() != full_coarse.all())
            witnesses = fine[bad] if new_label else np.empty((0,7))
            witness_d = model.clearance(witnesses)
            np.savez_compressed(out/f"straight_{row['problem_id']}.npz", q_coarse=coarse, q_fine=fine,
                                base_coarse_cuda_free=base_coarse,base_fine_cuda_free=base_fine,
                                cover_coarse_cuda_free=full_coarse,cover_fine_cuda_free=full_fine,
                                independent_new_collision_q=witnesses,independent_new_collision_clearance=witness_d)
            if len(witnesses):
                np.savez_compressed(out/f"line_{row['problem_id']}.npz", q=witnesses,
                                    indices=bad, independent_clearance=witness_d, fine_sample_count=len(fine))
            line_records.append(dict(problem_id=row['problem_id'], split=row['split'],family=row['family'],
                stored_straight_collision=row['straight_line_collision'],
                coarse_cuda_collision=bool(not full_coarse.all()),fine_cuda_collision=bool(not full_fine.all()),
                coarse_base_collision=bool(not base_coarse.all()),fine_base_collision=bool(not base_fine.all()),
                fine_samples=len(fine), classification_changed_at_finer_resolution=new_label,
                independent_new_collision_count=int((witness_d < -band).sum()),
                independent_new_ambiguous_count=int((abs(witness_d)<=band).sum())))
        print(json.dumps(dict(scene=number+1,total=len(scenes),id=sid,source_free=record['source_free'],
                              cover_free=record['cover_free'],errors=len(errors))),flush=True)
    aggregates=[]
    for split in ('smoke','dev','test'):
        for family in sorted({r['family'] for r in line_records if r['split']==split}):
            group=[r for r in line_records if r['split']==split and r['family']==family]
            aggregates.append(dict(split=split,family=family,cases=len(group),
                stored_blocked=sum(r['stored_straight_collision'] for r in group),
                fine_blocked=sum(r['fine_cuda_collision'] for r in group),
                fine_base_blocked=sum(r['fine_base_collision'] for r in group),
                fine_added_obstacle_only_blocked=sum(r['fine_cuda_collision'] and not r['fine_base_collision'] for r in group),
                changed_labels=sum(r['classification_changed_at_finer_resolution'] for r in group)))
    assert all(sha(ROOT/p)==h for p,h in initial.items())
    report=dict(complete=True, passed=not errors, errors=errors, created_utc=datetime.now(timezone.utc).isoformat(),
        new_planner_calls=0, frozen_inputs_changed=False, seed=2026103001,
        diagnostic_scope='Finite model/input checks, not planning. Test cases inspected only for input validity and geometry; no score or parameter tuning.',
        q_pool_rule='First 128 of 2048 uniform full-joint samples free in fixed base world, selected before examining added obstacles.',
        pooling_warning='The same diagnostic q pool is reused across scenes; counts are paired diagnostics, not independent random task observations.',
        source_geometry_warning='Source primitives are an audit comparison only. All benchmark methods still use the unchanged frozen cover.',
        sample_resolution_note='Straight-line labels compare CUDA 0.04 and 0.005 rad; newly changed collision labels independently checked in Drake.',
        continuous_safety_proof=False, elapsed_s=time.perf_counter()-started, sources_sha256=initial,
        base_random_samples=len(q_all), base_checker_disagreement_count=len(base_disagree),
        scene_count=len(scene_records),accepted_endpoint_count=sum(r['accepted_endpoint_count'] for r in scene_records),
        scene_comparisons=scene_records,straight_line_summary=aggregates,straight_line_cases=line_records)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    files=sorted(p for p in out.iterdir() if p.is_file())
    (out/'SHA256SUMS').write_text(''.join(f'{sha(p)}  {p.name}\n' for p in files))
    print(json.dumps(dict(complete=True,passed=not errors,errors=errors,elapsed_s=report['elapsed_s'])),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    main(args.output)
