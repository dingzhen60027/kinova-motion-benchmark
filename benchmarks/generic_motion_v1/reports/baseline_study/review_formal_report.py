"""Independent CPU-only arithmetic/readout review; not a trajectory audit.

Does not import the renderer or runtime metrics implementation. Bootstrap bounds
are checked for faithful reporting, not independently recomputed here.
"""
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

import numpy as np

HERE = Path(__file__).resolve().parent
BACKEND = {'eizo_inflation_seconds', 'alternate_drm_path_seconds',
           'trajectory_optimization_seconds', 'validation_seconds', 'collision_recovery_seconds'}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and np.isfinite(value)


def stats(values):
    a = np.array([v for v in values if finite(v)], dtype=float)
    return dict(count=len(a), mean=None if not len(a) else float(a.mean()),
                p50=None if not len(a) else float(np.quantile(a, .5)),
                p90=None if not len(a) else float(np.quantile(a, .9)),
                p95=None if not len(a) else float(np.quantile(a, .95)))


def key(row):
    return row['case_id'], row['roadmap_nodes'], row['roadmap_seed'], row['label']


def group_rows(rows, group):
    pool = [r for r in rows if r['roadmap_nodes'] == group['roadmap_nodes'] and r['label'] == group['label']]
    scope = group['scope']
    if scope == 'open_control':
        return [r for r in pool if r['family'] == 'open_control']
    pool = [r for r in pool if r['family'] != 'open_control']
    if scope == 'obstacle_main':
        return pool
    name, value = scope.split(':')
    field = {'family': 'family', 'straight_line': 'straight_line_collision', 'distance_bin': 'distance_bin'}[name]
    value = {'True': True, 'False': False}.get(value, value)
    return [r for r in pool if r[field] == value]


def main():
    paths = {name: HERE/f'formal_{name}_metrics/metrics.json' for name in ('main', 'size')}
    datasets = {name: json.loads(path.read_text()) for name, path in paths.items()}
    bundle_path = HERE/'formal_audit/audit_bundle.json'
    bundle = json.loads(bundle_path.read_text())
    report_path = HERE.parent/'benchmark_v1_formal_baselines.md'
    report = report_path.read_text()
    errors, checks, group_count, pair_count = [], 0, 0, 0
    def check(condition, message):
        nonlocal checks
        checks += 1
        if not condition:
            errors.append(message)
    check(bundle['complete'] and bundle['passed'] and not bundle['errors'], 'Audit bundle did not pass')
    for name, path in paths.items():
        digest = sha(path)
        check(digest == bundle['metrics_sha256']['scaling' if name == 'size' else name], f'{name} metrics binding')
        check(digest in report, f'{name} report provenance')
    main_map = {key(r): r for r in datasets['main']['rows']}
    size_map = {key(r): r for r in datasets['size']['rows']}
    shared = set(main_map) & set(size_map)
    distinct = dict(main_map); distinct.update(size_map)
    check(len(main_map) == len(datasets['main']['rows']) == 6300, 'Main 6300 unique slots')
    check(len(size_map) == len(datasets['size']['rows']) == 4800, 'Scaling 4800 unique observations')
    check(len(shared) == 1200 and all(k[1] == 6000 for k in shared), 'Reused 6000-node slots')
    check(all(main_map[k] == size_map[k] for k in shared), 'Main/scaling reused observations differ')
    check(len(distinct) == 9900 and sum(r['input_legal'] for r in distinct.values()) == 9300, 'Distinct legal/registered accounting')
    for name, data in datasets.items():
        rows = data['rows']
        check(data['complete'] and data['comparable'] and not data['pairing_differences'], f'{name} comparison gate')
        check(len({r['case_id'] for r in rows}) == (420 if name == 'main' else 80), f'{name} fixed IDs')
        check(len({r['case_id'] for r in rows if r['input_legal']}) == (395 if name == 'main' else 75), f'{name} legal fixed IDs')
        for r in rows:
            check(not r['input_legal'] or finite(r['cold_e2e_seconds']), f'{name}/{r["job_id"]} missing legal latency')
            if r['success']:
                check(r['input_legal'] and r['native_drm_success'] is True and r['drm_success'] is True
                      and r['solver_reported_success'] is True and r['final_validation_passed'] is True,
                      f'{name}/{r["job_id"]} invalid success gate')
            if not r['backend_timing_complete']:
                check(not BACKEND.intersection(r['completed_stage_seconds']), f'{name}/{r["job_id"]} incomplete backend counted')
            for stage, value in r['completed_stage_seconds'].items():
                check(value == r['timing'][stage], f'{name}/{r["job_id"]} selected timer changed')
        for group in data['groups']:
            group_count += 1
            pool = group_rows(rows, group)
            legal = [r for r in pool if r['input_legal']]
            success = [r for r in legal if r['success']]
            failed = [r for r in legal if not r['success']]
            name_key = f'{name}/{group["roadmap_nodes"]}/{group["scope"]}/{group["label"]}'
            check(group['requested_jobs'] == len(pool) and group['legal_jobs'] == len(legal), name_key+' counts')
            check(group['legal_status_counts'] == dict(Counter(r['status'] for r in legal)), name_key+' statuses')
            check(group['valid_success']['numerator'] == len(success) and group['valid_success']['denominator'] == len(legal), name_key+' rate')
            check(group['conditional_native_drm']['denominator'] == sum(r['native_drm_success'] is True for r in legal), name_key+' native denominator')
            check(group['conditional_validated_drm']['denominator'] == sum(r['drm_success'] is True for r in legal), name_key+' seed denominator')
            check(group['legal_scene_clusters'] == len({r['cluster_id'] for r in legal}), name_key+' scene clusters')
            if group['scope'] == 'open_control':
                check(group['legal_scene_clusters'] == 1 and group['valid_success']['ci95'] is None, name_key+' control CI')
            for field, summaries in group['statistics'].items():
                for scope, sample in [('all_legal', legal), ('validated_success', success), ('failed_legal', failed)]:
                    actual = stats([r[field] for r in sample]); saved = summaries[scope]
                    for metric in actual:
                        a, b = actual[metric], saved[metric]
                        check((a is None and b is None) or (a is not None and b is not None and np.isclose(a,b,atol=1e-12,rtol=1e-12)), name_key+f'/{field}/{scope}/{metric}')
            for field, summaries in group['stage_seconds'].items():
                for scope, sample in [('all_legal', legal), ('validated_success', success), ('failed_legal', failed)]:
                    actual = stats([r['completed_stage_seconds'].get(field) for r in sample]); saved=summaries[scope]
                    for metric in actual:
                        a,b=actual[metric],saved[metric]
                        check((a is None and b is None) or (a is not None and b is not None and np.isclose(a,b,atol=1e-12,rtol=1e-12)),name_key+f'/stage/{field}/{scope}/{metric}')
        for pair in data['paired_comparisons']:
            pair_count += 1
            selected = {}
            for label in (pair['method_a'], pair['method_b']):
                g = dict(roadmap_nodes=pair['roadmap_nodes'],scope=pair['scope'],label=label)
                selected[label] = {(r['case_id'],r['roadmap_id'],r['method_seed']):r for r in group_rows(rows,g) if r['input_legal']}
            a,b=selected[pair['method_a']],selected[pair['method_b']]
            check(set(a)==set(b) and len(a)==pair['paired_legal_jobs'],'Paired keys/denominator')
            keys=sorted(a); both=[k for k in keys if a[k]['success'] and b[k]['success']]
            values={'valid_success_difference':[int(a[k]['success'])-int(b[k]['success']) for k in keys],
                'observed_cold_latency_difference':[a[k]['cold_e2e_seconds']-b[k]['cold_e2e_seconds'] for k in keys],
                'common_success_cost_difference':[a[k]['cost']-b[k]['cost'] for k in both],
                'common_success_cold_latency_difference':[a[k]['cold_e2e_seconds']-b[k]['cold_e2e_seconds'] for k in both]}
            for field,v in values.items():
                estimate=None if not v else float(np.mean(v)); saved=pair[field]
                check(saved['denominator']==len(v),'Paired '+field+' denominator')
                check((estimate is None and saved['estimate'] is None) or
                    (estimate is not None and saved['estimate'] is not None and np.isclose(estimate,saved['estimate'],atol=1e-12,rtol=1e-12)), 'Paired '+field+' estimate')
    # Independently inspect the report's four primary tables and failed-latency
    # readouts. Full JSON arithmetic above covers every main/scaling stratum.
    context, table_headers, table_rows = {}, None, []
    def finish_table():
        if not table_headers or not table_rows:
            return
        if '6000 节点' not in context.get(2,''):
            return
        scope='obstacle_main' if '障碍主套件' in context[2] else 'open_control'
        for cells in table_rows:
            label=cells[0]
            match=[g for g in datasets['main']['groups'] if g['scope']==scope and g['label']==label]
            if not match:
                continue
            g=match[0]
            if table_headers[1]=='有效成功/合法查询 [95% CI]':
                for cell,field in zip(cells[1:4],('valid_success','conditional_native_drm','conditional_validated_drm')):
                    saved=g[field]; n,d=map(int,re.match(r'(\d+)/(\d+)',cell).groups())
                    check((n,d)==(saved['numerator'],saved['denominator']),f'Report {scope}/{label}/{field} counts')
                    check(f"{100*saved['estimate']:.2f}" in cell,f'Report {scope}/{label}/{field} estimate')
                    if saved['ci95'] is not None:
                        check(f"[{saved['ci95'][0]*100:.2f}, {saved['ci95'][1]*100:.2f}]" in cell,f'Report {scope}/{label}/{field} CI transcription')
            if table_headers[1]=='冷耗时 n':
                cold=g['statistics']['cold_e2e_seconds']['all_legal']
                check(int(cells[1])==cold['count']==g['legal_jobs'], f'Report {scope}/{label} excludes failed cold timings')
                check(cells[2]==' / '.join(f'{cold[k]:.3f}' for k in ('p50','p90','p95')), f'Report {scope}/{label} cold quantiles')
    for line in report.splitlines()+['']:
        if not line.startswith('|'):
            finish_table(); table_headers=None; table_rows=[]
        if line.startswith('## '):
            context[2]=line[3:]
        elif line.startswith('### '):
            context[3]=line[4:]
        elif line.startswith('|'):
            cells=[x.strip() for x in line.strip('|').split('|')]
            if table_headers is None: table_headers=cells
            elif not all(set(x)<=set(':- ') for x in cells): table_rows.append(cells)
    launch=json.loads((HERE/'formal_launch_session_observation.json').read_text())
    check(launch['launch_session_exit_code']==143 and launch['resume_finish_records']==24
          and not launch['nonzero_resume_finishes'] and launch['coordinator_complete_ledger']['complete'], 'Launch observation classification')
    result=dict(passed=not errors,errors=errors,checks=checks,metric_groups_recomputed=group_count,
        paired_point_estimates_recomputed=pair_count,main_registered_slots=len(main_map),
        size_observations=len(size_map),shared_6000_slots=len(shared),distinct_slots=len(distinct),
        legal_planner_calls=sum(r['input_legal'] for r in distinct.values()),
        input_invalid_slots=sum(not r['input_legal'] for r in distinct.values()),
        bootstrap_intervals_independently_recomputed=False,
        confidence_interval_scope='Checked faithful transcription of frozen metric CIs; no new bootstrap or inference claims.',
        numerical_trajectory_audit_repeated=False,report_sha256=sha(report_path),
        input_sha256={str(p):sha(p) for p in [*paths.values(),bundle_path]},
        visual_review={'figures_inspected':6,'layout_blockers':[],
            'notes':'All six PNGs were visually inspected: visible axes, method labels, legends, captions and numeric outputs. Control figures omit CI; scaling successful-cost curves are marked descriptive.'},
        launch_observation='Exit 143 is preserved; completed ledger and 24 zero resume exits are separate evidence. Cause not inferred.')
    (HERE/'formal_report_independent_review.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('input_sha256','visual_review')},indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__':
    main()
