"""Bind finished development results, derived reports and UI inspection."""
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
import json
import subprocess
import sys

from benchmarks.generic_motion_v1.common import BASE, ROOT, code_inventory, read, sha

OUT=Path(__file__).resolve().parent


def require(value, message):
    if not value:
        raise ValueError(message)


def main():
    destination=OUT/'delivery.json'
    sums=OUT/'SHA256SUMS'
    require(not destination.exists() and not sums.exists(), 'Preserve previous delivery seals')
    frozen=read(OUT/'execution_freeze.json')
    completed=read(OUT/'completed_blocks.json')
    index=read(OUT/'run_index.json')
    metrics=read(OUT/'metrics/metrics.json')
    require(completed['complete'] is True and completed['completed_jobs']==950, 'Incomplete study')
    require(metrics['complete'] is True and metrics['comparable'] is True and
            len(metrics['rows'])==metrics['requested_jobs']==950, 'Unaccepted metrics')
    require(code_inventory()==frozen['runtime_source_hashes'], 'Measured runtime changed')
    audits={name:read(OUT/(name+'.json')) for name in
            ('integrity_audit','lgcs_audit','nto_audit','final_report_review_v2')}
    require(all(a['passed'] is True and a['errors']==[] for a in audits.values()), 'Unaccepted audit')
    browser=read(OUT/'browser_checks_v1/inspection.json')
    replays=read(OUT/'development_replays_v1/index.json')
    require(browser['passed'] is True and browser['errors']==[] and len(browser['views'])==6,
            'Incomplete browser inspection')
    require(replays['complete'] is True and not replays['failures'] and
            len(replays['illustrations'])==6 and all(r['source_unchanged'] for r in replays['illustrations']),
            'Unaccepted replay sources')
    receipt=read(OUT/'report_receipt.json')
    require(sha(receipt['report'])==receipt['report_sha256'], 'Main report changed')
    review=audits['final_report_review_v2']
    require(sha(review['addendum_path'])==review['addendum_sha256'], 'Reviewed addendum changed')
    require(sha(OUT/'browser_checks_v1/inspection.json')==review['browser_inspection_sha256'],
            'Browser evidence changed after final review')
    for name,digest in receipt['audit_hashes'].items():
        require(sha(OUT/f'{name}_audit.json')==digest, 'Independent audit changed')

    files={}
    def capture(path):
        path=Path(path).resolve()
        require(path.is_relative_to(ROOT), 'Delivery artifact outside project')
        files[str(path.relative_to(ROOT))]=sha(path)

    # Raw artifacts are transitively bound by the already verified receipts.
    # Record every receipt and source snapshot without altering any attempt.
    attempt_count=0
    for block in index['blocks']:
        run=Path(block['run'])
        require(sha(run/'run_manifest.json')==block['run_manifest_sha256'], 'Run manifest changed')
        capture(run/'run_manifest.json');capture(run/'identity.json')
        receipts=sorted((run/'jobs').glob('*/attempts/*/receipt.json'))
        require(len(receipts)==block['planned_jobs'], 'Wrong run receipt coverage')
        attempt_count+=len(receipts)
        for path in receipts:capture(path)
        for path in (run/'source_snapshot').rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts:capture(path)
    require(attempt_count==950, 'Wrong total receipt coverage')
    for path in OUT.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts:
            capture(path)
    for path in (ROOT/'README.md',BASE/'README.md',BASE/'reports/benchmark_v1_reliable_baselines.md',
                 BASE/'reports/benchmark_v1_reliable_completion.md'):
        capture(path)
    statuses=dict(Counter(r['status'] for r in metrics['rows']))
    legal=sum(r['input_legal'] for r in metrics['rows'])
    require(legal==940 and sum(statuses.values())==950,'Wrong outcome population')
    result=dict(schema_version=1,complete=True,created_utc=datetime.now(timezone.utc).isoformat(),
        command=sys.orig_argv,source_commit=frozen['source_commit'],
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        registered_slots=950,legal_calls=legal,invalid_slots=950-legal,statuses=statuses,
        raw_receipts=attempt_count,files=files,
        integrity_scope='Direct hashes of reports, snapshots and every completed receipt; each receipt '
            'transitively binds its raw arrays, solver logs and request. Three independent audits verified '
            'the raw files. No planner or solver was rerun during sealing.',
        measured_driver_exit_code=0,render_report_exit_code=0,browser_check_exit_code=0,
        preserved_historical_result_commit='73356f1a0',new_test_planner_calls=0,
        main_report=receipt['report'],completion_report=str(BASE/'reports/benchmark_v1_reliable_completion.md'))
    with destination.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    files[str(destination.relative_to(ROOT))]=sha(destination)
    with sums.open('x') as stream:
        for name,digest in sorted(files.items()):stream.write(f'{digest}  {name}\n')
    print(json.dumps(dict(complete=True,raw_receipts=attempt_count,direct_files=len(files),
                         statuses=statuses,delivery_sha256=sha(destination)),indent=2))


if __name__=='__main__':
    main()
