"""Bind completed evidence without changing any experiment result."""
from pathlib import Path
from datetime import datetime, timezone
import json
import subprocess
from ...common import ROOT, BASE, read, write, sha, code_inventory
from ...batch import verify
from .run_extended_study import OUT, check_frozen
from .build_report import validate_inputs


def validate_derived(report, audit, replays):
    """Reject stale derived files before making a delivery claim."""
    metrics_path=OUT/'metrics/metrics.json'
    validate_inputs(OUT,metrics_path,OUT/'final_audit.json')
    receipt=read(OUT/'report_receipt.json')
    bindings={
        'report_sha256':report,
        'builder_sha256':OUT/'build_report.py',
        'metrics_sha256':metrics_path,
        'execution_freeze_sha256':OUT/'execution_freeze.json',
        'final_audit_sha256':OUT/'final_audit.json',
        'commands_sha256':OUT/'commands.jsonl',
        'completed_blocks_sha256':OUT/'completed_blocks.json',
    }
    for key,path in bindings.items():
        if receipt.get(key)!=sha(path):raise ValueError('Stale report binding: '+key)
    if receipt.get('source_commit')!=read(OUT/'execution_freeze.json')['source_commit']:
        raise ValueError('Report implementation commit differs from freeze')
    if replays.get('metrics_sha256')!=sha(metrics_path) or replays.get('audit_sha256')!=sha(OUT/'final_audit.json'):
        raise ValueError('Stale replay inputs')
    metrics=read(metrics_path)
    rows={str(Path(row['attempt']).resolve()):row for row in metrics['rows']}
    entries=replays['entries']
    expected=set(metrics['expected_method_labels'])
    available_success={row['label'] for row in metrics['rows'] if row['split']=='dev' and row['success']}
    successful={item['label'] for item in entries if item['role']=='validated_final_trajectory'}
    if (successful!=available_success or {item['label'] for item in entries}!=expected
            or len({item['name'] for item in entries})!=len(entries)):
        raise ValueError('Incomplete or duplicate method replay views')
    for item in entries:
        attempt=Path(item['source_attempt']).resolve()
        row=rows.get(str(attempt))
        if row is None or any(item[key]!=row[key] for key in ('label','case_id','status')):
            raise ValueError('Replay does not match a registered observation')
        if item['role']=='validated_final_trajectory' and not row['success']:
            raise ValueError('Failed observation labeled as a valid replay')
        if item['role']=='failed_attempt_saved_evidence' and row['success']:
            raise ValueError('Successful observation labeled as a failed replay')
        if not item['rendered'] or item['source_receipt_sha256']!=sha(attempt/'receipt.json'):
            raise ValueError('Replay source receipt changed')
        directory=(OUT/'replays'/item['name']).resolve()
        if directory.parent!=(OUT/'replays').resolve():raise ValueError('Replay path outside gallery')
        current={str(p.relative_to(OUT/'replays')):sha(p) for p in directory.rglob('*') if p.is_file()}
        if current!=item['files']:raise ValueError('Replay files changed: '+item['name'])
        if not all((directory/name).is_file() for name in ('replay.html','replay.json','scene.png','joints.png')):
            raise ValueError('Missing replay artifact: '+item['name'])
        data=read(directory/'replay.json')
        if data['case_id']!=row['case_id'] or data['status']!=row['status']:
            raise ValueError('Rendered replay identity differs from observation')
        sample=data.get('sample_source')
        if sample:
            sample_path=(attempt/sample).resolve()
            if not sample_path.is_relative_to(attempt) or data['sample_source_sha256']!=sha(sample_path):
                raise ValueError('Replay numerical samples changed')
    qa=read(OUT/'replays/browser_qa.json')
    if not qa.get('complete') or qa.get('gallery_sha256')!=sha(OUT/'replays/index.html'):
        raise ValueError('Missing or stale replay browser verification')
    if qa.get('observations_sha256')!=sha(OUT/'replays/browser_observations.json'):
        raise ValueError('Browser observation evidence changed')
    views={item['name']:item for item in qa['views']}
    if set(views)!={item['name'] for item in entries}:raise ValueError('Incomplete browser verification')
    for name,item in views.items():
        if not item['passed'] or item['page_sha256']!=sha(OUT/'replays'/name/'replay.html'):
            raise ValueError('Failed or stale browser verification: '+name)
        screenshot=(OUT/'replays'/item['screenshot']).resolve()
        if screenshot.parent!=(OUT/'replays').resolve() or item['screenshot_sha256']!=sha(screenshot):
            raise ValueError('Browser screenshot evidence changed: '+name)


def main():
    target=OUT/'delivery.json'
    if target.exists():raise FileExistsError('Preserve prior delivery seal')
    frozen=read(OUT/'execution_freeze.json');check_frozen(frozen)
    done=read(OUT/'completed_blocks.json');audit=read(OUT/'final_audit.json')
    report=BASE/'reports/benchmark_v1_extended_baselines.md'
    replays=read(OUT/'replays/index.json')
    if not done['complete'] or done['completed_jobs']!=1140 or not audit['passed'] or not replays['complete'] or not report.is_file():
        raise ValueError('Complete study, audit, report and replay evidence required')
    validate_derived(report,audit,replays)
    if code_inventory()!=frozen['runtime_source_hashes']:raise ValueError('Runtime changed')
    run_paths=[Path(b['run']) for b in frozen['blocks']]+[BASE/'results/extended_baselines_v1_cli_nomap']
    sources={}
    receipts={}
    raw_files=0
    for run in run_paths:
        check=verify(run)
        if not check['complete'] or not check['integrity_passed']:raise ValueError(str(run))
        sources[str(run.relative_to(ROOT))]=dict(manifest_sha256=sha(run/'run_manifest.json'),verification=check)
        for p in sorted(run.glob('jobs/*/attempts/*/receipt.json')):
            receipts[str(p.relative_to(ROOT))]=sha(p)
            raw_files+=len(read(p)['files'])
    files={}
    for root in (OUT,):
        for path in sorted(root.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix not in ('.pyc','.pyo') and path.name not in ('SHA256SUMS','delivery.json'):
                files[str(path.relative_to(ROOT))]=sha(path)
    for path in (report,ROOT/'README.md',BASE/'README.md',BASE/'EXTENDED_BASELINES.md'):
        files[str(path.relative_to(ROOT))]=sha(path)
    for run in run_paths:
        for path in sorted(run.iterdir()):
            if path.is_file() and path.name!='.resume.lock':files[str(path.relative_to(ROOT))]=sha(path)
        for path in (run/'source_snapshot').rglob('*'):
            if path.is_file():files[str(path.relative_to(ROOT))]=sha(path)
    files.update(receipts)
    checksum=OUT/'SHA256SUMS'
    checksum.write_text(''.join(digest+'  '+name+'\n' for name,digest in sorted(files.items())))
    write(target,dict(complete=True,sealed_utc=datetime.now(timezone.utc).isoformat(),
        implementation_commit=frozen['source_commit'],runtime_sources_unchanged=True,
        current_head_before_artifact_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        study_registered_slots=1140,study_legal_slots=1128,study_invalid_slots=12,
        auxiliary_cli_jobs=1,test_planner_calls=0,runs=sources,receipts=len(receipts),raw_files=raw_files,
        direct_files=len(files),checksum_file=str(checksum.relative_to(ROOT)),checksum_sha256=sha(checksum),
        final_audit_sha256=sha(OUT/'final_audit.json'),report_sha256=sha(report),
        note='Each sealed attempt receipt binds its complete raw files. Auxiliary standalone OMPL CLI probe is outside the comparison denominator. Historical raw evidence is preserved; old README bytes are archived under prior_readmes.'))
    print(json.dumps(read(target),indent=2))

if __name__=='__main__':main()
