"""CPU-only summary and checksums of the already frozen test inputs.

Run with ``bash local/python.sh benchmarks/generic_motion_v1/reports/test_freeze_analysis.py``.
This script never constructs a robot, changes a dataset, or calls a planner.
"""
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

BASE = Path(__file__).resolve().parents[1]
ROOT = BASE.parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def load_split(name):
    return [json.loads(line) for line in (BASE / 'manifests' / f'{name}.jsonl').read_text().splitlines()]


def quantiles(values):
    return np.quantile(values, [0, .5, 1]).tolist() if values else None


def run():
    rows = load_split('test')
    receipt = read(BASE / 'receipts/test_generation.json')
    audit = read(BASE / 'reports/test_input_audit.json')
    acceptance = read(BASE / 'reports/acceptance.json')
    scenes = {r['scene_id']: read(BASE / r['scene_file']) for r in rows}
    valid = [r for r in rows if r['validation_status'] == 'valid_input']
    family_distribution = {}
    for family in sorted({r['family'] for r in rows}):
        fr = [r for r in rows if r['family'] == family]
        fv = [r for r in fr if r['validation_status'] == 'valid_input']
        fs = [s for s in scenes.values() if s['family'] == family]
        family_distribution[family] = {
            'cases': len(fr), 'layout_ids': len(fs), 'valid_input': len(fv),
            'invalid_input': len(fr) - len(fv),
            'actual_distinct_geometries': len({s['obstacle_geometry_sha256'] for s in fs}),
            'independent_random_geometries': 0 if family == 'open_control' else len(fs),
            'straight_line_colliding': sum(r['straight_line_collision'] is True for r in fv),
            'straight_line_colliding_fraction_valid': sum(r['straight_line_collision'] is True for r in fv) / len(fv) if fv else None,
            'obstacles_per_layout_min_median_max': quantiles([len(s['obstacle_geometry']) for s in fs]),
            'voxels_per_layout_min_median_max': quantiles([len(s['voxels_xyz']) for s in fs]),
            'voxel_xyz_float32_bytes_per_layout_min_median_max': quantiles([12 * len(s['voxels_xyz']) for s in fs]),
            'joint_distance_rad_min_median_max': quantiles([r['start_goal_joint_distance'] for r in fv]),
            'normalized_joint_distance_min_median_max': quantiles([r['normalized_start_goal_joint_distance'] for r in fv]),
            'requested_distance_bin_counts': dict(Counter(r['endpoint_sampling']['distance_bin'] for r in fr)),
            'legal_distance_bin_counts': dict(Counter(r['endpoint_sampling']['distance_bin'] for r in fv)),
            'layout_rejections': sum(s['generation_rejected_layouts'] for s in fs),
            'fixed_geometry_overlapping_voxels': sum(s['fixed_geometry_overlapping_voxels'] for s in fs),
            'endpoint_collision_or_clearance_rejections': sum(r['endpoint_sampling']['collision_or_clearance_rejections'] for r in fr),
            'endpoint_distance_bin_rejections': sum(r['endpoint_sampling']['distance_bin_rejections'] for r in fr),
        }
    comparisons = {}
    for split in ('smoke', 'dev'):
        other = load_split(split)
        current_geometries = {r['obstacle_geometry_sha256'] for r in rows if r['family'] != 'open_control'}
        other_geometries = {r['obstacle_geometry_sha256'] for r in other if r['family'] != 'open_control'}
        current_q = {tuple(r[key]) for r in rows for key in ('q_start', 'q_goal') if r[key] is not None}
        other_q = {tuple(r[key]) for r in other for key in ('q_start', 'q_goal') if r[key] is not None}
        comparisons[split] = dict(
            shared_non_open_geometry_hashes=sorted(current_geometries & other_geometries),
            shared_scene_ids=sorted({r['scene_id'] for r in rows} & {r['scene_id'] for r in other}),
            shared_scene_seeds=sorted({r['scene_seed'] for r in rows} & {r['scene_seed'] for r in other}),
            shared_endpoint_seeds=sorted({r['endpoint_seed'] for r in rows} & {r['endpoint_seed'] for r in other}),
            shared_individual_endpoint_configurations=[list(q) for q in sorted(current_q & other_q)],
        )
    checks = {relative: sha(BASE / relative) == expected for relative, expected in receipt['files_sha256'].items()}
    core_paths = set(receipt['files_sha256']) | {
        'receipts/test_generation.json', 'reports/test_input_audit.json',
        'reports/acceptance.json', 'configs/generator.json', 'model.py',
        'reports/test_freeze_analysis.py',
    } | set(receipt['protocol']['generator_sources_sha256'])
    hashes = {str((BASE / relative).relative_to(ROOT)): sha(BASE / relative) for relative in sorted(core_paths)}
    checksum_path = BASE / 'manifests/test.SHA256SUMS'
    checksum_path.write_text(''.join(f'{value}  {relative}\n' for relative, value in hashes.items()))
    commands = []
    for line in (BASE / 'reports/cli_commands.jsonl').read_text().splitlines():
        item = json.loads(line)
        argv = item.get('argv', [])
        if item.get('event') == 'finish' and (
                ('generate' in argv and '--split' in argv and 'test' in argv) or
                ('audit' in argv and any(str(arg).endswith('manifests/test.jsonl') for arg in argv))):
            commands.append(item)
    no_shared = all(not values for comparison in comparisons.values() for values in comparison.values())
    no_test_jobs = True
    for run_manifest in (BASE / 'results').glob('*/run_manifest.json'):
        if any(job.get('case', {}).get('split') == 'test' for job in read(run_manifest).get('jobs', [])):
            no_test_jobs = False
    report = dict(
        passed=bool(audit['passed'] and all(checks.values()) and no_shared and
                    len(rows) == 420 and len(scenes) == 84 and no_test_jobs),
        test_planner_execution_permitted=False, test_planner_jobs_present=not no_test_jobs,
        dataset_status='frozen inputs only; no test planner run or test-score tuning',
        source_code_commit=receipt['source_code_commit'], source_seed=receipt['seed'],
        problem_count=len(rows), layout_id_count=len(scenes),
        actual_distinct_geometry_count=len({s['obstacle_geometry_sha256'] for s in scenes.values()}),
        independent_random_geometry_count=sum(s['family'] != 'open_control' for s in scenes.values()),
        valid_input_count=len(valid), invalid_input_count=len(rows) - len(valid),
        status_counts=dict(Counter(r['validation_status'] for r in rows)),
        straight_line_colliding_count=sum(r['straight_line_collision'] is True for r in valid),
        straight_line_colliding_fraction_valid=sum(r['straight_line_collision'] is True for r in valid) / len(valid),
        unknown_global_feasibility_count=len(rows), numerical_check_not_continuous_certificate=True,
        endpoint_proposal_count=sum(r['endpoint_sampling']['drawn_configurations'] for r in rows),
        endpoint_checker_disagreements=sum(r['endpoint_sampling']['checker_disagreements'] for r in rows),
        edge_sample_count=sum(r.get('straight_line_samples', 0) for r in rows),
        endpoint_minimum_clearance_m=min(min(r['endpoint_clearances_m']) for r in valid),
        family_distribution=family_distribution, cross_split_checks=comparisons,
        open_control_note='Four layout IDs reuse one fixed base-only geometry, not four independent random environments; this geometry is also shared with smoke open control by design.',
        acceptance_gates={g: acceptance[g] for g in ('G0', 'G1', 'G2')},
        acceptance_evidence_sha256=acceptance['evidence_sha256'],
        manifest_sha256=sha(BASE / 'manifests/test.jsonl'),
        generator_config_sha256=sha(BASE / 'configs/generator.json'),
        frozen_test_config_sha256=sha(BASE / 'configs/test_generator.json'),
        geometry_protocol_sha256=receipt['geometry_protocol_sha256'],
        checksum_file=str(checksum_path.relative_to(ROOT)), checksum_file_sha256=sha(checksum_path),
        checksum_file_count=len(hashes), receipt_file_hash_checks_passed=all(checks.values()),
        files_sha256=hashes, actual_cli_commands=commands,
        stdout_logs=['reports/test_freeze_generate.stdout.log', 'reports/test_freeze_input_audit.stdout.log'],
    )
    (BASE / 'reports/test_freeze_audit.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    lines = ['# Test split freeze audit', '',
        'This is the RSS-inspired Kinova-7D adaptation. It is not an exact reproduction of the FR3 paper benchmark.', '',
        f"Frozen **{len(rows)} cases / {len(scenes)} layout IDs** after recorded G0/G1/G2 acceptance. "
        f"Input audit passed: **{report['passed']}**. No test planning jobs were created or run.", '',
        f"Legal inputs: **{len(valid)}**; invalid inputs retained: **{len(rows) - len(valid)}**. "
        f"Numerically colliding straight lines: **{report['straight_line_colliding_count']}/{len(valid)}** legal inputs. "
        'Global problem feasibility remains unknown; finite sampling is not a continuous collision certificate.', '',
        '| Family | Cases | Layout IDs | Legal / invalid | Straight blocked | Obstacles min/median/max | Voxels min/median/max |',
        '|---|---:|---:|---:|---:|---|---|']
    for family, item in family_distribution.items():
        fmt = lambda key: '/'.join(f'{x:g}' for x in item[key])
        lines.append(f"| {family} | {item['cases']} | {item['layout_ids']} | {item['valid_input']} / {item['invalid_input']} | "
                     f"{item['straight_line_colliding']}/{item['valid_input']} | {fmt('obstacles_per_layout_min_median_max')} | {fmt('voxels_per_layout_min_median_max')} |")
    lines += ['', 'The four open-control layout IDs represent one actual geometry. The other 80 layouts have 80 distinct random geometries. '
        'No non-open geometry, scene seed, endpoint seed, or individual endpoint configuration is shared with smoke or dev.', '',
        'All invalid cases and rejection counts remain in the frozen manifest. Scene rejection uses fixed-equipment overlap only; '
        'endpoint rejection uses collision, clearance, limits and predeclared distance bins. No planner outcome is used for selection.', '',
        'The JSON companion includes per-family distance distributions, raw-voxel memory counts, collision disagreements, '
        'all SHA256 values and actual CLI timing records. Both original geometry and its conservative 0.04 m sphere cover are preserved.', '',
        f"- Manifest SHA256: `{report['manifest_sha256']}`",
        f"- Generator / copied test config SHA256: `{report['generator_config_sha256']}`",
        f"- Geometry protocol SHA256: `{report['geometry_protocol_sha256']}`",
        f"- SHA256SUMS entries: **{len(hashes)}**; checksum file: `manifests/test.SHA256SUMS`.", '',
        'Commands executed from the repository root:', '', '```bash',
        'bash local/python.sh -m benchmarks.generic_motion_v1 generate --split test --seed 20261027 --output benchmarks/generic_motion_v1 --acceptance benchmarks/generic_motion_v1/reports/acceptance.json',
        'bash local/python.sh -m benchmarks.generic_motion_v1 audit --manifest benchmarks/generic_motion_v1/manifests/test.jsonl --output benchmarks/generic_motion_v1/reports/test_input_audit.json',
        'bash local/python.sh benchmarks/generic_motion_v1/reports/test_freeze_analysis.py',
        'sha256sum -c benchmarks/generic_motion_v1/manifests/test.SHA256SUMS', '```', '',
        'Full generation and audit stdout are saved in `reports/test_freeze_generate.stdout.log` and `reports/test_freeze_input_audit.stdout.log`.', '']
    (BASE / 'reports/test_freeze_audit.md').write_text('\n'.join(lines))
    print(json.dumps({k: report[k] for k in ('passed', 'problem_count', 'layout_id_count', 'valid_input_count',
          'invalid_input_count', 'straight_line_colliding_count', 'manifest_sha256', 'checksum_file_count')}, indent=2))


if __name__ == '__main__':
    run()
