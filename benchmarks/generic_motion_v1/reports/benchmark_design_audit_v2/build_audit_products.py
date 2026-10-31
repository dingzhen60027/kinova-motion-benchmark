"""Join read-only input audits into supplementary case annotations.

These are descriptive annotations, never a replacement manifest or a selector
based on any planner's success. Run from the existing project Conda environment.
"""
from pathlib import Path
from collections import Counter
import argparse
import hashlib
import json

HERE=Path(__file__).resolve().parent
BENCH=HERE.parents[1]

def load(path):
    return json.loads(path.read_text())

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main(output):
    output=Path(output)
    names=('case_audit_annotations.jsonl','audit_summary.json')
    if any((output/name).exists() for name in names):
        raise FileExistsError('Choose a fresh output directory; preserve previous outputs')
    local=load(HERE/'local_design/audit.json')
    geometry=load(HERE/'geometry_run_v3/report.json')
    assert local['all_integrity_and_replay_checks_passed'] and geometry['passed']
    scene_map={r['scene_id']:r for r in geometry['scene_comparisons']}
    line_map={r['problem_id']:r for r in geometry['straight_line_cases']}
    manifest_paths=[BENCH/'manifests'/f'{s}.jsonl' for s in ('smoke','dev','test')]
    rows=[json.loads(line) for p in manifest_paths for line in p.read_text().splitlines()]
    annotations=[]
    for row in rows:
        scene=scene_map[row['scene_id']]
        line=line_map.get(row['problem_id'])
        annotations.append(dict(schema='benchmark_design_audit_annotation_v1',
            problem_id=row['problem_id'],original_problem_sha256=row['problem_sha256'],
            split=row['split'],family=row['family'],scene_id=row['scene_id'],
            physical_environment_key=row['obstacle_geometry_sha256'],
            statistical_cluster_key='open_control_shared_base' if row['family']=='open_control' else row['scene_id'],
            input_status=row['validation_status'],distance_bin=row['endpoint_sampling']['distance_bin'],
            family_scope='workspace geometry description; no certified C-space passage or topology',
            feasibility='unknown_from_input_audit; do not infer infeasibility from sampling or planner failure',
            frozen_geometry='canonical robot plus radius 0.04 m cover spheres; original primitives are audit-only',
            historical_test_exposure=row['split']=='test' and row['problem_id'] in local['existing_test_exposure']['case_ids'],
            straight_line_diagnostic=line,
            source_vs_cover_diagnostic={k:scene[k] for k in ('q_count','source_free','cover_free','source_free_cover_colliding')},
            selection_policy='retain all registered records; this file does not select problems'))
    summary=dict(schema='benchmark_design_audit_summary_v1',new_planner_calls=0,
        original_manifests_unchanged=True,registered_cases=len(rows),
        valid_cases=sum(r['validation_status']=='valid_input' for r in rows),
        split_counts={split:dict(registered=sum(r['split']==split for r in rows),
              statuses=dict(Counter(r['validation_status'] for r in rows if r['split']==split)),
              valid_distance_bins=dict(Counter(r['endpoint_sampling']['distance_bin'] for r in rows
                    if r['split']==split and r['validation_status']=='valid_input')))
            for split in ('smoke','dev','test')},
        integrity_checks=local['check_count'],geometry_scene_count=geometry['scene_count'],
        checked_endpoint_count=geometry['accepted_endpoint_count'],
        straight_line_valid_case_count=len(line_map),
        straight_line_fine_sample_count=sum(r['fine_samples'] for r in line_map.values()),
        straight_line_changed_labels=sum(r['classification_changed_at_finer_resolution'] for r in line_map.values()),
        scene_configuration_checks=sum(r['q_count'] for r in scene_map.values()),
        cover_misses_source_samples=sum(r['cover_misses_source'] for r in scene_map.values()),
        cuda_drake_disagreement_outside_band=sum(r['cuda_drake_disagreement_outside_band'] for r in scene_map.values()),
        representation_effects=[dict(split=split,family=family,**{k:sum(r[k] for r in scene_map.values()
                   if r['split']==split and r['family']==family)
               for k in ('q_count','source_free','cover_free','source_free_cover_colliding')})
            for split in ('smoke','dev','test') for family in sorted({r['family'] for r in scene_map.values() if r['split']==split})],
        source_sha256={str(p.relative_to(BENCH)):sha(p) for p in manifest_paths+
            [HERE/'local_design/audit.json',HERE/'geometry_run_v3/report.json',Path(__file__)]},
        readiness=dict(controlled_frozen_protocol_use='supported_by_current_numerical_audits',
            universal_manipulator_benchmark_claim='not_established',
            mandatory_narrow_passage_claim='not_established',
            untouched_test_claim='false_test_previously_evaluated',
            continuous_collision_certificate='not_provided',
            external_public_suite='not_yet_integrated_in_this_turn'))
    assert len(annotations)==490 and summary['valid_cases']==463 and len(line_map)==463
    output.mkdir(parents=True,exist_ok=True)
    (output/names[0]).write_text(''.join(json.dumps(r,sort_keys=True,ensure_ascii=False)+'\n' for r in annotations))
    (output/names[1]).write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({k:summary[k] for k in ('registered_cases','valid_cases','checked_endpoint_count','straight_line_fine_sample_count','straight_line_changed_labels')},ensure_ascii=False))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=HERE)
    main(parser.parse_args().output)
