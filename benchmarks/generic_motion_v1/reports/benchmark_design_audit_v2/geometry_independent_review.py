"""Independent read-only arrays/provenance review of geometry probe v3."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
BASE = HERE.parents[1]
ROOT = BASE.parents[1]
RUN = HERE / 'geometry_run_v3'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    target = HERE / 'geometry_independent_review.json'
    if target.exists():
        raise FileExistsError(target)
    report = json.loads((RUN / 'report.json').read_text())
    model = json.loads((BASE / 'reports/model_audit/audit.json').read_text())['model']
    band = model['contact_band_m']
    checks = []

    def check(name, value):
        checks.append(dict(name=name, passed=bool(value)))

    for line in (RUN / 'SHA256SUMS').read_text().splitlines():
        expected, name = line.split(None, 1)
        check('artifact_sha:' + name, sha(RUN / name) == expected)
    for rel, expected in report['sources_sha256'].items():
        check('source_sha:' + rel, sha(ROOT / rel) == expected)
    for rel, expected in model['source_files_sha256'].items():
        check('canonical_robot_asset:' + rel, sha(ROOT / rel) == expected)
    rows = [json.loads(line) for split in ('smoke', 'dev', 'test') for line in (BASE / 'manifests' / f'{split}.jsonl').read_text().splitlines()]
    by_id = {r['problem_id']: r for r in rows}
    by_scene = defaultdict(list)
    for row in rows:
        by_scene[row['scene_id']].append(row)
    with np.load(RUN / 'configuration_pool.npz', allow_pickle=False) as raw:
        q_all, base, clearance, pool = [raw[k] for k in ('all_q', 'base_cuda', 'base_clearance', 'selected_q')]
    random = np.random.Generator(np.random.PCG64(report['seed']))
    expected_q = random.uniform(np.array(model['lower']) + .02, np.array(model['upper']) - .02, (2048, 7))
    check('base_rng_exact', np.array_equal(q_all, expected_q))
    check('pool_selected_first_base_free_without_added_obstacles', np.array_equal(pool, q_all[base & (clearance > 1e-4)][:128]))
    check('base_mismatch_count', int(((base != (clearance > 0)) & (abs(clearance) > band)).sum()) == report['base_checker_disagreement_count'])
    for entry in report['scene_comparisons']:
        sid = entry['scene_id']
        scene = json.loads((BASE / 'scenes' / f'{sid}.json').read_text())
        with np.load(RUN / f'{sid}.npz', allow_pickle=False) as raw:
            q, src, cover, native, ends, esrc, ecover, enative = [raw[k] for k in ('q', 'source_clearance', 'cover_clearance', 'cuda_free', 'endpoints', 'endpoint_source_clearance', 'endpoint_cover_clearance', 'endpoint_cuda_free')]
        check(sid + ':same_common_pool', np.array_equal(q, pool))
        check(sid + ':finite_arrays', all(np.isfinite(x).all() for x in (q, src, cover, ends, esrc, ecover)))
        expected_ends = np.array([r[k] for r in by_scene[sid] if r['validation_status'] == 'valid_input' for k in ('q_start', 'q_goal')]).reshape(-1, 7)
        check(sid + ':frozen_endpoint_identity', np.array_equal(ends, expected_ends))
        recomputed = dict(q_count=len(q), source_free=int((src > band).sum()),
            cover_free=int((native & (cover > band)).sum()),
            source_free_cover_colliding=int(((src > band) & (cover < -band)).sum()),
            cover_misses_source=int((native & (cover > band) & (src < -band)).sum()),
            cuda_drake_disagreement_outside_band=int(((native != (cover > 0)) & (abs(cover) > band)).sum()),
            accepted_endpoint_count=len(ends),
            min_endpoint_source_clearance_m=float(esrc.min()) if len(esrc) else None,
            min_endpoint_cover_clearance_m=float(ecover.min()) if len(ecover) else None,
            max_cover_outward_bound_m=max((r['maximum_outward_overapproximation_m'] for r in scene['voxelization']['obstacles']), default=0),
            min_cell_coverage_margin_m=min((r['coverage_margin_m'] for r in scene['voxelization']['obstacles']), default=None))
        for key, value in recomputed.items():
            check(sid + ':summary:' + key, entry[key] == value)
        check(sid + ':accepted_endpoints_free', np.all(enative & (ecover > 1e-4) & (esrc > band)))

        check(sid + ':named_collision_pair_identity_receipt', entry['source_collision_pair_identities_verified'] is True)

    check('scene_count', report['scene_count'] == len(report['scene_comparisons']) == 109)
    check('endpoint_count', report['accepted_endpoint_count'] == sum(r['accepted_endpoint_count'] for r in report['scene_comparisons']) == 926)
    check('all_valid_lines_present', {r['problem_id'] for r in report['straight_line_cases']} == {r['problem_id'] for r in rows if r['validation_status'] == 'valid_input'})
    for line in report['straight_line_cases']:
        row = by_id[line['problem_id']]
        with np.load(RUN / f"straight_{row['problem_id']}.npz", allow_pickle=False) as raw:
            coarse = raw['q_coarse']; fine = raw['q_fine']
            bc, bf, cc, cf = [raw[k] for k in ('base_coarse_cuda_free','base_fine_cuda_free','cover_coarse_cuda_free','cover_fine_cuda_free')]
            witness = raw['independent_new_collision_q']; wd = raw['independent_new_collision_clearance']
        length = np.linalg.norm(np.array(row['q_goal']) - row['q_start'])
        check(line['problem_id'] + ':raw_interpolation', np.array_equal(coarse, np.linspace(row['q_start'], row['q_goal'], int(np.ceil(length/.04))+1)) and np.array_equal(fine, np.linspace(row['q_start'], row['q_goal'], int(np.ceil(length/.005))+1)))
        check(line['problem_id'] + ':raw_boolean_shapes', bc.shape == cc.shape == (len(coarse),) and bf.shape == cf.shape == (len(fine),) and all(mask.dtype.kind == 'b' for mask in (bc,bf,cc,cf)))
        raw_flags = dict(coarse_cuda_collision=bool(not cc.all()), fine_cuda_collision=bool(not cf.all()), coarse_base_collision=bool(not bc.all()), fine_base_collision=bool(not bf.all()))
        check(line['problem_id'] + ':raw_classifications', all(line[k] == v for k,v in raw_flags.items()))
        changed = bool(cc.all() != cf.all())
        expected_witness = fine[~cf] if changed else np.empty((0,7))
        check(line['problem_id'] + ':new_witnesses', np.array_equal(witness, expected_witness) and wd.shape == (len(witness),) and int((wd < -band).sum()) == line['independent_new_collision_count'] and int((abs(wd) <= band).sum()) == line['independent_new_ambiguous_count'])
        check(line['problem_id'] + ':raw_cover_superset_collision', not np.any(cc & ~bc) and not np.any(cf & ~bf))
        check(line['problem_id'] + ':line_sample_count', line['fine_samples'] == int(np.ceil(np.linalg.norm(np.array(row['q_goal']) - row['q_start']) / .005)) + 1)
        check(line['problem_id'] + ':stored_label', line['stored_straight_collision'] == row['straight_line_collision'])
        check(line['problem_id'] + ':new_label_arithmetic', line['classification_changed_at_finer_resolution'] == (line['coarse_cuda_collision'] != line['fine_cuda_collision']))
        check(line['problem_id'] + ':old_coarse_recheck', line['coarse_cuda_collision'] == row['straight_line_collision'])
        check(line['problem_id'] + ':base_subset', not line['fine_base_collision'] or line['fine_cuda_collision'])
    for group in report['straight_line_summary']:
        lines = [r for r in report['straight_line_cases'] if r['split'] == group['split'] and r['family'] == group['family']]
        counts = dict(cases=len(lines), stored_blocked=sum(r['stored_straight_collision'] for r in lines),
            fine_blocked=sum(r['fine_cuda_collision'] for r in lines),
            fine_base_blocked=sum(r['fine_base_collision'] for r in lines),
            fine_added_obstacle_only_blocked=sum(r['fine_cuda_collision'] and not r['fine_base_collision'] for r in lines),
            changed_labels=sum(r['classification_changed_at_finer_resolution'] for r in lines))
        for key, value in counts.items():
            check(f"line_aggregate:{group['split']}:{group['family']}:{key}", group[key] == value)
    assert all(r['passed'] for r in checks), [r for r in checks if not r['passed']]
    result = dict(passed=True, check_count=len(checks), checks=checks,
        scope='Independent raw-array arithmetic, input/provenance hashes, frozen endpoint matching; code review of same-directive canonical source model and exact named collision-pair assertions. Full saved coarse/fine q and collision masks independently reduced for all 463 lines.',
        new_planner_calls=0, new_cuda_calls=0, new_cpu_collision_calls=0,
        important_limitations=[
            'The v3 probe fixes the v2 evidence gap by saving every per-sample straight-line mask; this review recomputes their reductions but does not rerun collision engines.',
            'The same 128 base-free configurations are reused across scenes; these are paired diagnostics, not 13952 independent environment samples.',
            'A cover-free count of zero in 128 configurations is not proof of empty free space or planning infeasibility.',
            'Source geometry is only a counterfactual diagnostic; historical methods used unchanged frozen cover geometry.',
            'No new held-out planning results were produced; existing v1 test exposure remains documented separately.'],
        source_files_sha256={str((RUN/'report.json').relative_to(ROOT)):sha(RUN/'report.json'),str((HERE/'geometry_probe.py').relative_to(ROOT)):sha(HERE/'geometry_probe.py'),str(Path(__file__).relative_to(ROOT)):sha(Path(__file__))})
    target.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=True,checks=len(checks),new_collision_calls=0,new_cuda_calls=0,new_planner_calls=0),indent=2))


if __name__ == '__main__':
    main()
