"""Post-run integrity and paired DRM audit; never runs a planner or solver.

Requires all four coordinator blocks to be sealed before hashing result trees.
Historical verification is limited to 50 smoke + 900 dev slots. Formal-v1 raw
results are protected by Git diff, frozen key hashes and input/dependency seals;
their trajectories are not reloaded. Failed audits are saved, never overwritten.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import traceback
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
BASE = ROOT / 'benchmarks/generic_motion_v1'
OUT = Path(__file__).resolve().parent
SOURCE = 'e6b7e372e091ce5b01417073e0880e7c3068910c'
OLD_SOURCE = '72984027030175b7bca78e2dcf592b38f0c1864b'
ARCHIVE_COMMIT = '73356f1a0677a0a7e6fecaf4c25969055fb8d36b'
SUCCESS = 'success_validated_numerical'
METHODS = {('EI_ZO', 'minimum_distance'): 'LSCS', ('LGCS', 'minimum_distance'): 'LGCS',
           ('NTO_REFINED', 'minimum_distance'): 'NTO-refined-distance',
           ('EI_ZO', 'minimum_time'): 'SCSTO', ('NTO_REFINED', 'minimum_time'): 'NTO-refined-time'}
BLOCKS = {'smoke_s000': ('smoke', 0, 10, 8), 'dev_s000': ('dev', 0, 60, 60),
          'dev_s001': ('dev', 1, 60, 60), 'dev_s002': ('dev', 2, 60, 60)}
PROTECTED = {
    'manifests/test.jsonl': '9fcbe5ccae28d8681586b1ac6c9986f51f7cc7b5cdc19352c101910e3b6ed03b',
    'reports/baseline_study/formal_protocol_v1.json': '8d152ac237cf39fe9602bcf54d47fbe8e30471dd5e3096ff05375ff4b3fcd7d7',
    'reports/baseline_study/formal_main_metrics/metrics.json': 'dde3098ac6efa4fb7d7913a22cdd0c9093d6d6de74da4e304e93a1088075d1db',
    'reports/baseline_study/formal_size_metrics/metrics.json': 'a0a34410ef1e8f4f1bada972af1fed5167c4663a8048b2c8957b567ce3ba89c7',
    'results/rss2025_full_smoke_v1/run_manifest.json': 'a2faa273ecc64dcecf0444030f9d75c143c67824023fab39a82f5bfcd82a233b',
    'results/rss2025_full_dev_v1/run_manifest.json': '44452e26ea4e0eacee15e4df12e9cf183b48c561c6fd1ddf73ab86631a75919c',
}


def read(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def check_files(table, root=ROOT):
    require(bool(table), 'Empty hash inventory')
    changed = []
    for name, digest in table.items():
        path = Path(name)
        if not path.is_absolute():
            path = root / path
        if not path.is_file() or sha(path) != digest:
            changed.append(str(path))
    require(not changed, 'Changed protected files: ' + repr(changed))
    return dict(passed=True, files=len(table))


def unique_index(rows, key, description):
    output = {}
    for row in rows:
        identity = key(row)
        require(identity not in output, f'Duplicate {description}: {identity}')
        output[identity] = row
    return output


def preflight(frozen, index, completed):
    require(frozen['source_commit'] == SOURCE, 'Wrong frozen source commit')
    require(frozen['runtime_source_hashes'], 'Missing frozen source inventory')
    require(set(map(tuple, frozen['declaration']['methods'])) == set(METHODS), 'Wrong methods')
    require(frozen['planned_registered_slots'] == index['slots'] == 950, 'Wrong slot total')
    require(frozen['test_planner_calls'] == index['test_planner_calls'] == 0, 'Test jobs declared')
    require(frozen['timeout_seconds'] == 60 and frozen['gpu_workers'] == 1, 'Wrong budget/concurrency')
    fi = unique_index(frozen['blocks'], lambda b: b['name'], 'freeze block')
    ix = unique_index(index['blocks'], lambda b: b['name'], 'index block')
    require(set(fi) == set(ix) == set(BLOCKS), 'Missing or extra development block')
    require(completed.get('complete') is True and completed.get('completed_jobs') == 950
            and completed.get('total_jobs') == 950, 'Coordinator not complete; do not audit partial runs')
    co = unique_index(completed['blocks'], lambda b: b['name'], 'completed block')
    require(set(co) == set(BLOCKS), 'Missing completed blocks')
    for name, (split, seed, registered, legal) in BLOCKS.items():
        b = fi[name]
        require((b['split'], b['registered_inputs'], b['legal_inputs'], b['planned_jobs']) ==
                (split, registered, legal, registered * 5), f'Wrong block accounting: {name}')
        require({k: v for k, v in ix[name].items() if k != 'run_manifest_sha256'} == b,
                f'Index differs from freeze: {name}')
        require({k: v for k, v in co[name].items() if k != 'verification'} == ix[name],
                f'Completed block differs from index: {name}')
        check = co[name]['verification']
        require(check.get('complete') is True and check.get('integrity_passed') is True
                and check.get('completed_jobs') == registered * 5, f'Unsealed block: {name}')
    require(len({b['run'] for b in ix.values()}) == 4, 'Duplicate run directories')
    return ix


def audit_matrix(frozen, block, manifest, data, roadmap):
    """Pure matrix check, with full case equality (including all endpoint data)."""
    name = block['name']
    split, seed, registered, legal = BLOCKS[name]
    require(manifest['source_code_commit'] == SOURCE and manifest['source_hashes'] ==
            frozen['runtime_source_hashes'], f'Source differs: {name}')
    require(manifest['split'] == split and manifest['formal_test_run'] is False
            and manifest.get('study_protocol') is None and manifest['selection'] == 'all',
            f'Test/selection mismatch: {name}')
    require(manifest['manifest_path'] == block['manifest'] and
            manifest['manifest_sha256'] == frozen['data_files'][block['manifest']], 'Dataset mismatch')
    require(manifest['timeout_seconds'] == frozen['timeout_seconds'], 'Budget mismatch')
    expected_maps = {p: h for p, h in frozen['roadmap_files'].items()
                     if Path(p).parent == Path(block['roadmap'])}
    require(expected_maps and manifest['roadmap_files'] == expected_maps, 'Roadmap seal mismatch')
    require(int(roadmap['seed']) == seed and roadmap['status'] == 'ready', 'Wrong graph seed/state')
    require(roadmap['requested_nodes'] == roadmap['actual_nodes'] == 6000, 'Wrong graph size')
    require(len(data) == registered and sum(c['validation_status'] == 'valid_input' for c in data) == legal,
            'Wrong input accounting')
    cases = unique_index(data, lambda c: c['problem_id'], 'case')
    jobs = unique_index(manifest['jobs'], lambda j: (j['case_id'], j['method'], j['objective']), 'job')
    require(set(jobs) == {(pid, m, o) for pid in cases for m, o in METHODS}, 'Incomplete method/case matrix')
    require(len({j['job_id'] for j in jobs.values()}) == len(jobs), 'Duplicate job IDs')
    for (pid, method, objective), job in jobs.items():
        case = cases[pid]
        require(case['split'] == split and job['case'] == case, 'Case/endpoints changed')
        require(job['scene_id'] == case['scene_id'] and job['family'] == case['family'], 'Case identity changed')
        require(job['evaluation_config'] == frozen['evaluation_config'], 'Evaluation config changed')
        require(job['roadmap_dir'] == block['roadmap'] and job['roadmap_id'] == roadmap['roadmap_id'],
                'Roadmap identity changed')
        pairs = [p for p in case['paired_seeds'] if p['drm_seed'] == seed]
        require(len(pairs) == 1 and job['method_seed'] == pairs[0]['method_seed'] and
                job['solver_seed'] == pairs[0]['solver_seed'], 'Method/graph seed mismatch')
        require(all(case[k] == roadmap[k] for k in ('robot_sha256', 'collision_geometry_sha256')),
                'Canonical model differs between input and graph')
    return dict(passed=True, registered=len(jobs), legal=legal * 5, invalid=(registered - legal) * 5,
                source=manifest['source_code_commit'], roadmap_seed=seed, roadmap_id=roadmap['roadmap_id'])


def job_key(job):
    method = 'NTO_PAPER' if job['method'] == 'NTO_REFINED' else job['method']
    return (job['case_id'], job['roadmap_id'], job['method_seed'], method, job['objective'])


def load_outcome(run, job, source):
    attempts = sorted((Path(run) / 'jobs' / job['job_id'] / 'attempts').glob('[0-9][0-9][0-9][0-9]'))
    require(bool(attempts), 'Missing attempts: ' + job['job_id'])
    attempt = attempts[-1]
    receipt = read(attempt / 'receipt.json')
    require(not receipt.get('retry_pending'), 'Final attempt still pending')
    result = read(attempt / 'result.json')
    require(result.get('source_code_commit') == source, 'Worker source commit differs')
    saved = read(attempt / 'input.json')
    saved.pop('_objective', None)
    require(saved == job['case'], 'Saved input changed')
    require(read(attempt / 'evaluation.json') == job['evaluation_config'], 'Saved config changed')
    require(read(attempt / 'request.json') == job, 'Saved request changed')
    require(read(attempt / 'runtime_sources.json').get('unchanged') is True, 'Worker source changed')
    legal = job['case']['validation_status'] == 'valid_input'
    if not legal:
        require(result['status'] == 'invalid_endpoint' and not result.get('frontend_native_success')
                and not (attempt / 'query.json').exists() and not (attempt / 'path.npy').exists(),
                'Invalid input reached planning or was promoted')
    path = np.load(attempt / 'path.npy', allow_pickle=False) if (attempt / 'path.npy').exists() else None
    if result.get('frontend_native_success'):
        require(path is not None and path.ndim == 2 and path.shape[1] == 7 and len(path) >= 2
                and np.isfinite(path).all(), 'Native success lacks finite joint path')
    return dict(job=job, result=result, receipt=receipt, attempt=attempt, path=path,
                process=read(attempt / 'process.json'), query=read(attempt / 'query.json')
                if (attempt / 'query.json').exists() else None, legal=legal,
                all_attempts=[dict(path=str(p), receipt_sha256=sha(p / 'receipt.json'),
                                  retry_pending=read(p / 'receipt.json').get('retry_pending', False),
                                  status=read(p / 'receipt.json')['status']) for p in attempts])


def check_query(row, build, roadmap=None):
    q, job, result = row['query'], row['job'], row['result']
    if q is None:
        require(row['path'] is None and not result.get('frontend_native_success'), 'Path without native query evidence')
        return 'not_reached'
    for key in ('binary_sha256', 'native_library_sha256', 'patched_source_sha256', 'upstream_source_sha256'):
        require(q[key] == build[key], 'Query dependency/source hash differs: ' + key)
    require(read(row['attempt'] / 'build_receipt.json') == build, 'Per-query build receipt differs')
    require(all(q.get(k) is False for k in ('endpoint_change', 'cuda_changed', 'search_budget_changed')),
            'Query altered invariants')
    require(q['q_start'] == job['case']['q_start'] and q['q_goal'] == job['case']['q_goal'] and
            q['online_voxel_radius_m'] == job['case']['voxel_radius_m'], 'Query endpoints/radius differ')
    layout = q['layout_before']
    require(layout['num_joints'] == 7 and layout['loaded'] is True and layout['initialized'] is True,
            'Query ABI/layout precondition differs')
    if roadmap is not None:
        require(layout['nodes'] == roadmap['actual_nodes'] and
                result.get('roadmap_sha256') == roadmap['native_map_sha256'], 'Query loaded another graph')
    for key in ('max_number_planning_attempts', 'max_iterations_steering_to_node', 'max_nodes_to_expand',
                'online_edge_step_size'):
        require(layout[key] == float(np.float32(job['evaluation_config']['drm'][key])), 'Query budget/step differs')
    status = q['status']
    require(status in ('returned', 'native_exception', 'running'), 'Unknown native query state')
    if status == 'returned':
        require(q['query_success'] == bool(result.get('frontend_native_success')), 'Native query success conflicts')
        require(row['path'] is not None, 'Returned native query lacks saved path')
    elif status == 'native_exception':
        require(q['query_success'] is False and bool(q.get('exception')) and not result.get('frontend_native_success'),
                'Native exception promoted to success')
    else:
        require(result['status'] in ('timeout', 'infrastructure_error') and not result.get('frontend_native_success'),
                'Interrupted native query promoted to success')
    return status


def compare_frontend(old, new):
    """Method optimization may change; fixed native query outcomes may not."""
    a, b = old['job'], new['job']
    require(job_key(a) == job_key(b) and a['case'] == b['case'], 'Historical pairing/input differs')
    require(all(a[k] == b[k] for k in ('roadmap_dir', 'solver_seed', 'solver_seed_used')), 'Historical seed differs')
    for key in ('drm', 'roadmap', 'endpoint_tolerance_rad', 'validation_step_rad',
                'validation_minimum_samples', 'validation_maximum_samples', 'feasibility_tolerance'):
        require(a['evaluation_config'][key] == b['evaluation_config'][key], 'Historical frontend/validator config differs')
    result = dict(case_id=b['case_id'], roadmap_id=b['roadmap_id'], method=b['method'], objective=b['objective'],
                  old_status=old['result']['status'], new_status=new['result']['status'],
                  old_attempt=str(old['attempt']), new_attempt=str(new['attempt']), passed=False)
    x, y = old['path'], new['path']
    r0, r1 = old['result'], new['result']
    recognized_abort = False
    if x is None and old['process'].get('exit_code') == -6:
        stderr = (old['attempt'] / 'stderr.log').read_text(errors='replace')
        stdout = (old['attempt'] / 'stdout.log').read_text(errors='replace')
        recognized_abort = ('closest_node >= 0' in stderr or
                            'Failed to find path in roadmap after verification step.' in stdout)
    if recognized_abort:
        q = new['query'] or {}
        result.update(kind='reviewed_abort_to_query_failure', passed=(r1['status'] == 'frontend_failed'
            and (y is None or y.size == 0) and not r1.get('frontend_native_success') and not r1.get('frontend_success')
            and q.get('query_success') is False and q.get('status') in ('native_exception', 'returned')))
        return result
    if x is not None or y is not None:
        equal = x is not None and y is not None and x.shape == y.shape and np.array_equal(x, y)
        result.update(kind='returned_path', path_equal=equal,
                      old_path_shape=list(x.shape) if x is not None else None,
                      new_path_shape=list(y.shape) if y is not None else None)
        result['passed'] = bool(equal and bool(old['result'].get('frontend_native_success')) ==
                                bool(new['result'].get('frontend_native_success')) and
                                bool(old['result'].get('frontend_success')) == bool(new['result'].get('frontend_success')))
        return result
    if not old['legal'] and not new['legal']:
        result.update(kind='same_invalid_input', passed=r0['status'] == r1['status'] == 'invalid_endpoint')
    else:
        result.update(kind='no_returned_path', passed=(r0['status'] == r1['status'] == 'frontend_failed'
            and not r0.get('frontend_native_success') and not r1.get('frontend_native_success')))
    return result


def aggregate(rows):
    groups = []
    for split, label in sorted({(r['job']['case']['split'], METHODS[(r['job']['method'], r['job']['objective'])]) for r in rows}):
        selected = [r for r in rows if r['job']['case']['split'] == split
                    and METHODS[(r['job']['method'], r['job']['objective'])] == label]
        for family in ['all', *sorted({r['job']['family'] for r in selected})]:
            subset = [r for r in selected if family == 'all' or r['job']['family'] == family]
            legal = [r for r in subset if r['legal']]
            successes = sum(r['result']['status'] == SUCCESS for r in legal)
            groups.append(dict(split=split, method=label, family=family, registered=len(subset), legal=len(legal),
                invalid=len(subset)-len(legal), success=successes, legal_failures=len(legal)-successes,
                status_counts=dict(Counter(r['result']['status'] for r in subset)),
                native_frontend=sum(bool(r['result'].get('frontend_native_success')) for r in legal),
                accepted_frontend=sum(bool(r['result'].get('frontend_success')) for r in legal)))
    return groups


def preserve_archive():
    check_files(PROTECTED, BASE)
    protocol = read(BASE / 'reports/baseline_study/formal_protocol_v1.json')
    checks = {key: check_files(protocol[key]) for key in ('data_files', 'external_dependency_files')}
    checks['model'] = check_files(protocol['model']['files'])
    prefix = 'benchmarks/generic_motion_v1/'
    pathspecs = [':(glob)' + prefix + pattern for pattern in
        ('results/rss2025_formal_v1_*/**', 'results/rss2025_full_dev_v1/**', 'results/rss2025_full_smoke_v1/**',
         'inputs/test-*', 'scenes/test-*', 'manifests/test*')]
    command = ['git', 'diff', '--name-only', ARCHIVE_COMMIT, '--', *pathspecs]
    changed = subprocess.check_output(command, cwd=ROOT, text=True).splitlines()
    additions = subprocess.check_output(['git', 'ls-files', '--others', '--exclude-standard', '--', *pathspecs],
                                        cwd=ROOT, text=True).splitlines()
    require(not changed and not additions, 'Archived raw data/test changed: ' + repr(changed + additions))
    return dict(passed=True, against_commit=ARCHIVE_COMMIT, key_sha256=PROTECTED, checks=checks,
                git_diff_command=command, changed_paths=changed, untracked_additions=additions,
                scope='No old formal trajectories loaded; Git-tracked bytes + frozen key/input/model/dependency hashes')


def audit(output, *, freeze_path=OUT/'execution_freeze.json', index_path=OUT/'run_index.json',
          completed_path=OUT/'completed_blocks.json'):
    report_path = Path(output)
    require(report_path.suffix == '.json', 'Audit output must be a JSON file')
    output = report_path.with_suffix('')
    markdown = report_path.with_suffix('.md')
    require(not report_path.exists() and not output.exists() and not markdown.exists(),
            'Preserve prior audit; choose a new output path')
    output.mkdir(parents=True)
    report = dict(schema_version=1, kind='development_repair_integrity_audit', passed=False, complete=False,
        source_code_commit=SOURCE, created_utc=datetime.now(timezone.utc).isoformat(), errors=[],
        numerical_scope='Receipt/invariant/path-array audit only. Independent trajectory audits remain separate.',
        planner_calls=0, solver_calls=0, old_formal_trajectories_loaded=0)
    try:
        frozen, index, completed = read(freeze_path), read(index_path), read(completed_path)
        blocks = preflight(frozen, index, completed)
        report['evidence'] = {k: dict(path=str(Path(p).resolve()), sha256=sha(p)) for k, p in
                              [('freeze',freeze_path),('run_index',index_path),('completed',completed_path)]}
        report['source_checks'] = check_files(frozen['runtime_source_hashes'])
        from ...common import code_inventory
        require(code_inventory() == frozen['runtime_source_hashes'], 'Runtime inventory gained/lost files after freeze')
        for key in ('data_files', 'roadmap_files', 'external_dependency_files'):
            report[key] = check_files(frozen[key])
        require(sha(OUT/'development_protocol.json') == frozen['declaration_sha256'] and
                sha(OUT/'run_development_study.py') == frozen['execution_script_sha256'] and
                sha(BASE/'configs/rss2025_reliable.json') == frozen['config_file_sha256'], 'Freeze declaration/config changed')
        report['archive_protection'] = preserve_archive()
        from ...batch import verify
        old_rows, new_rows, checks, matrices = [], [], {}, {}
        old_manifests = {}
        for split in ('smoke', 'dev'):
            run = BASE / 'results' / ('rss2025_full_' + split + '_v1')
            check = verify(run, current_sources=False)
            require(check['complete'] and check['integrity_passed'], 'Historical reference unsealed: ' + repr(check))
            m = read(run/'run_manifest.json'); old_manifests[split] = m
            require(m['source_code_commit'] == OLD_SOURCE, 'Wrong historical source')
            checks[str(run)] = check
            old_rows.extend(load_outcome(run, j, OLD_SOURCE) for j in m['jobs'])
        build = read(ROOT/frozen['evaluation_config']['drm_query']['build_dir']/'build.json')
        for name, b in blocks.items():
            run = Path(b['run']); require(sha(run/'run_manifest.json') == b['run_manifest_sha256'], 'Run index hash differs')
            m = read(run/'run_manifest.json')
            data = [json.loads(line) for line in Path(b['manifest']).read_text().splitlines() if line.strip()]
            roadmap = read(Path(b['roadmap'])/'metadata.json')
            matrices[name] = audit_matrix(frozen,b,m,data,roadmap)
            check = verify(run, current_sources=True)
            require(check['complete'] and check['integrity_passed'], 'New run unsealed: ' + repr(check))
            checks[str(run)] = check
            for job in m['jobs']:
                row = load_outcome(run,job,SOURCE); row['query_status'] = check_query(row,build,roadmap); new_rows.append(row)
        old = unique_index(old_rows,lambda r:job_key(r['job']),'historical pairing')
        new = unique_index(new_rows,lambda r:job_key(r['job']),'new pairing')
        require(set(old) == set(new) and len(new) == 950, 'Historical/new paired slots differ')
        require(sum(r['legal'] for r in new_rows) == 940, 'Expected 940 legal and 10 invalid slots')
        comparisons = [compare_frontend(old[k], new[k]) for k in sorted(new)]
        shared = defaultdict(list)
        for row in new_rows:
            j=row['job'];shared[(j['case_id'],j['roadmap_id'],j['method_seed'])].append(row)
        common = []
        for key, rows in sorted(shared.items()):
            require(len(rows) == 5, 'Missing method in common-DRM group')
            paths=[r['path'] for r in rows]
            equal=all((paths[0] is None and p is None) or (paths[0] is not None and p is not None
                and paths[0].shape == p.shape and np.array_equal(paths[0],p)) for p in paths)
            outcomes={(bool(r['result'].get('frontend_native_success')),bool(r['result'].get('frontend_success'))) for r in rows}
            common.append(dict(case_id=key[0],roadmap_id=key[1],method_seed=key[2],methods=5,
                               path_equal=equal,frontend_outcome_equal=len(outcomes)==1,passed=equal and len(outcomes)==1))
        write(output/'paired_drm.json',dict(comparisons=comparisons,within_new_methods=common))
        write(output/'attempts.json',dict(attempts=[dict(key=list(job_key(r['job'])),attempts=r['all_attempts']) for r in new_rows]))
        report.update(complete=True,verifications=checks,matrices=matrices,registered_slots=950,legal_slots=940,
            invalid_slots=10,groups=aggregate(new_rows),paired_drm_groups=len(common),paired_drm_slots=len(comparisons),
            path_comparison_counts=dict(Counter(c['kind'] for c in comparisons)),
            query_status_counts=dict(Counter(r['query_status'] for r in new_rows)),
            paired_mismatches=[c for c in comparisons if not c['passed']],
            within_new_mismatches=[c for c in common if not c['passed']],
            all_attempts=sum(len(r['all_attempts']) for r in new_rows),
            retry_pending_attempts=sum(a['retry_pending'] for r in new_rows for a in r['all_attempts']),
            run_manifest_sha256={b['run']:b['run_manifest_sha256'] for b in blocks.values()})
        if report['paired_mismatches'] or report['within_new_mismatches']:
            report['errors'].append('Common DRM results differ; do not release an equivalence claim')
        report['passed'] = not report['errors']
    except Exception as exc:
        report['errors'].append(repr(exc)); (output/'exception.txt').write_text(traceback.format_exc())
    finally:
        report['audit_script_sha256'] = sha(__file__)
        write(report_path,report)
        lines=['# 开发集基线修复独立完整性审计', '',
            f"审计通过：{report['passed']}；完整运行：{report['complete']}。", '',
            '本审计复核收据、来源、输入、依赖和公共 DRM 数组，不重新执行规划、求解器或碰撞验证。独立轨迹审计须另行绑定。', '',
            f"源码：`{SOURCE}`。历史参考：`{OLD_SOURCE}`。", '']
        if report.get('complete'):
            lines += [f"登记 {report['registered_slots']} slots，其中合法 {report['legal_slots']}，无效 {report['invalid_slots']}。",
                f"核对 {report['paired_drm_slots']} 个历史/新运行配对和 {report['paired_drm_groups']} 个五方法公共 DRM 组。", '',
                '| split | 方法 | 登记 | 合法 | 数值验证成功 | 合法失败 | 无效输入 |',
                '|---|---|---:|---:|---:|---:|---:|']
            for group in report['groups']:
                if group['family']=='all':
                    lines.append('| {split} | {method} | {registered} | {legal} | {success} | {legal_failures} | {invalid} |'.format(**group))
            lines += ['', '无效输入不计入规划成功率分母。失败类别、各族计数、被中断的尝试和逐对路径差异保留在 JSON 与同名目录。', '']
        lines += ['错误：' + ('无。' if not report['errors'] else '\n\n'+'\n'.join('- '+e for e in report['errors'])), '',
            '原正式结果未重算；保护范围为 Git 对照、归档关键 SHA、冻结 test 输入/模型和依赖指纹。当前四个登记块不含 test 规划任务。',
            '通过有限路径对照不构成所有可能输入的算法等价证明；任何 DRM 差异均阻止本审计通过。', '']
        markdown.write_text('\n'.join(lines))
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planning-finished',action='store_true',required=True)
    parser.add_argument('--output',type=Path,default=OUT/'integrity_audit.json')
    args=parser.parse_args()
    result=audit(args.output)
    print(json.dumps({k:result.get(k) for k in ('passed','complete','registered_slots','legal_slots','errors')},indent=2))
    if not result['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
