"""Read-only comparison coverage audit; no planner, solver or GPU calls."""
from collections import Counter
from datetime import datetime, timezone
from hashlib import sha256
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[4]
BASE = ROOT / 'benchmarks/generic_motion_v1'
OUT = Path(__file__).resolve().parent


def load(path):
    return json.loads(Path(path).read_text())


def digest(path):
    h = sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def summary(rows):
    values = {}
    for label in sorted({r['label'] for r in rows}):
        selected = [r for r in rows if r['label'] == label]
        legal = [r for r in selected if r['input_legal']]
        values[label] = dict(registered=len(selected), legal=len(legal),
            success=sum(r['success'] for r in legal),
            accepted_drm_seeds=sum(r['seed_cost_validated'] for r in legal),
            statuses=dict(Counter(r['status'] for r in selected)),
            distinct_scenes=len({r['scene_id'] for r in selected}),
            blocked_queries=sum(r['straight_line_collision'] is True for r in legal),
            blocked_success=sum(r['straight_line_collision'] is True and r['success'] for r in legal))
    return values


def identity(row):
    return row['case_id'], row['roadmap_id'], row['method_seed']


def main():
    historical = BASE / 'reports/baseline_study/dev_metrics_stage_qualified/metrics.json'
    current = BASE / 'reports/baseline_repair_v2/metrics/metrics.json'
    old, new = load(historical), load(current)
    errors, inputs, file_count, receipt_count = [], {}, 0, 0
    inputs[str(historical.relative_to(ROOT))] = digest(historical)
    inputs[str(current.relative_to(ROOT))] = digest(current)
    # Check the full old dev and completed reliable smoke/dev evidence. No
    # reclassification, regeneration or editing of existing attempts occurs.
    for metrics in (old, new):
        for run in metrics['runs']:
            path = Path(run['run']) / 'run_manifest.json'
            observed = digest(path)
            inputs[str(path.relative_to(ROOT))] = observed
            if observed != run['manifest_sha256']:
                errors.append(f'Run manifest mismatch: {path}')
        for row in metrics['rows']:
            attempt = Path(row['attempt'])
            receipt = load(attempt / 'receipt.json')
            receipt_count += 1
            if receipt['status'] != row['status']:
                errors.append(f'Status mismatch: {attempt}')
            for name, expected in receipt['files'].items():
                target = attempt / name
                file_count += 1
                if not target.is_file() or digest(target) != expected:
                    errors.append(f'Raw file mismatch: {target}')
            if load(attempt / 'result.json')['status'] != row['status']:
                errors.append(f'Raw result mismatch: {attempt}')
    current_dev = [r for r in new['rows'] if r['split'] == 'dev']
    seed_reference = {identity(r): r for r in current_dev if r['label'] == 'LSCS'}
    pairs = {}
    fields = ('q_start', 'q_goal', 'input_sha256', 'robot_sha256',
              'geometry_sha256', 'budget_seconds', 'roadmap_nodes')
    for label in ('NTO-distance', 'NTO-time'):
        matched, accepted, identical = 0, 0, 0
        for row in old['rows']:
            if row['label'] != label:
                continue
            reference = seed_reference[identity(row)]
            matched += 1
            if any(row[k] != reference[k] for k in fields):
                errors.append(f'Input/reference mismatch: {row["job_id"]}')
            if row['seed_cost_validated'] != reference['seed_cost_validated']:
                errors.append(f'DRM acceptance mismatch: {row["job_id"]}')
            if row['seed_cost_validated']:
                accepted += 1
                if row['path_sha256'] == reference['path_sha256']:
                    identical += 1
                else:
                    errors.append(f'DRM path differs: {row["job_id"]}')
        pairs[label] = dict(matched_queries=matched, accepted_seed_pairs=accepted,
                           bitwise_identical_saved_path_files=identical)
    formal = BASE / 'reports/baseline_study/formal_protocol_v1.json'
    inputs[str(formal.relative_to(ROOT))] = digest(formal)
    formal_metrics_path = BASE / 'reports/baseline_study/formal_main_metrics/metrics.json'
    formal_metrics = load(formal_metrics_path)
    inputs[str(formal_metrics_path.relative_to(ROOT))] = digest(formal_metrics_path)
    source_paths = ['runners/paper_nto.py', 'runners/paper_nto_refined.py',
                    'runners/paper_lgcs.py', 'runners/paper_lgcs_scip.py',
                    'runners/validate.py', 'model.py', 'runners/study_metrics.py',
                    'configs/rss2025_reliable.json', 'configs/rss2025_eizo.json']
    for rel in source_paths:
        path = BASE / rel
        inputs[str(path.relative_to(ROOT))] = digest(path)
    report = dict(audit_utc=datetime.now(timezone.utc).isoformat(),
        source_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        scope='Read-only evidence audit; no planning, solver, collision or GPU calls. Historical formal data not retuned or rerun.',
        audit_passed=not errors, errors=errors, checked_receipts=receipt_count,
        checked_receipt_files=file_count, inputs_sha256=inputs,
        historical_dev=summary(old['rows']), reliable_dev=summary(current_dev),
        reliable_smoke=summary([r for r in new['rows'] if r['split'] == 'smoke']),
        archived_formal=dict(
            scope='Read existing sealed summary and protocol only; formal raw receipts were not rechecked by this audit.',
            source_commit=load(formal)['source_code_commit'],
            execution=load(formal)['execution'],
            main_obstacles=summary([r for r in formal_metrics['rows'] if r['family'] != 'open_control']),
            open_control=summary([r for r in formal_metrics['rows'] if r['family'] == 'open_control'])),
        historical_original_nto_vs_current_frontend=pairs,
        coverage_gap='Original unrefined NTO_PAPER has not been run as a complete current-safe-native development matrix; NTO_REFINED is a distinct explicit protocol.',
        recommended_new_executions=dict(method='NTO_PAPER',
            objectives=['minimum_distance', 'minimum_time'],
            smoke=dict(cases=10, roadmap_nodes=6000, roadmap_seeds=[0], registered_slots=20, legal_slots=16),
            dev=dict(cases=60, roadmap_nodes=6000, roadmap_seeds=[0,1,2], registered_slots=360, legal_slots=360),
            total_registered_slots=380, total_legal_slots=376,
            config='configs/rss2025_reliable.json', cold_deadline_seconds=60,
            concurrency=1, formal_test_planner_calls=0,
            selection='All registered smoke/dev cases, no success-conditioned selection'),
        reuse_policy=dict(
            preserve='Retain all sealed historical and reliable results, including failures.',
            reuse='Use reliable LSCS/LGCS/SCSTO and NTO_REFINED as distinct current evidence; cite their source commit and execution freeze.',
            no_pooling='Current study_metrics requires identical full runtime inventories and resolved configurations. Do not bypass it to pool old and newly edited runtime scores.',
            same_inventory='If no runtime/config changes are required, the existing reliable sources can support separately frozen NTO_PAPER blocks on the identical inventory.',
            changed_inventory='If runtime edits alter the source inventory, report a separately audited cross-version reference, or rerun all five paper combinations under one freeze; never relabel historical results as fresh.',
            no_cherry_pick='Never select per-query best result between original and refined NTO.'))
    (OUT / 'existing_evidence_audit.json').write_text(json.dumps(report, indent=2) + '\n')
    if errors:
        raise SystemExit(f'Audit failed with {len(errors)} errors')
    print(json.dumps(dict(audit_passed=True, checked_receipts=receipt_count,
                          checked_receipt_files=file_count, pairs=pairs)))


if __name__ == '__main__':
    main()
