"""Seal the finished comparator study without rewriting historical evidence."""
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
import json
import subprocess
import sys

from benchmarks.generic_motion_v1.common import BASE, ROOT, code_inventory, read, sha

OUT = Path(__file__).resolve().parent


def require(ok, message):
    if not ok:
        raise ValueError(message)


def main():
    destination, sums = OUT/'delivery.json', OUT/'SHA256SUMS'
    require(not destination.exists() and not sums.exists(), 'Preserve previous delivery seals')
    frozen = read(OUT/'execution_freeze.json')
    completed = read(OUT/'completed_blocks.json')
    metrics = read(OUT/'comparison_metrics/metrics.json')
    require(completed['complete'] is True and completed['completed_jobs'] == 380, 'Incomplete new runs')
    require(metrics['complete'] is True and metrics['comparable'] is True and
            len(metrics['rows']) == metrics['requested_jobs'] == 1330, 'Incomplete comparison')
    require(code_inventory() == frozen['runtime_source_hashes'], 'Runtime changed')
    for name in ('comparison_audit', 'final_review'):
        result = read(OUT/(name+'.json'))
        require(result['passed'] is True and result['errors'] == [], 'Failed '+name)
    for path, digest in read(OUT/'final_review.json')['files'].items():
        require(sha(path) == digest, 'Evidence changed after final review: '+path)
    browser = read(OUT/'browser_checks_v1/inspection.json')
    replay = read(OUT/'replays/index.json')
    require(browser['passed'] is True and browser['errors'] == [], 'Failed browser inspection')
    require(replay['complete'] is True and not replay['failures'] and
            all(r['source_unchanged'] for r in replay['illustrations']), 'Replay source changed')
    report = OUT.parent/'benchmark_v1_paper_comparators.md'
    require(report.is_file(), 'Missing comparison report')
    report_receipt = read(OUT/'report_receipt.json')
    require(sha(report) == report_receipt['report_sha256'], 'Report changed after generation')
    for path, digest in {**report_receipt['inputs'], **report_receipt['source_files']}.items():
        require(sha(path) == digest, 'Report source changed after generation: '+path)
    original = [r for r in metrics['rows'] if r['method'] == 'NTO_PAPER']
    legal_original = sum(r['input_legal'] for r in original)
    require((len(original), legal_original) == (380, 376), 'New legal-call count differs')
    files = {}
    def capture(path):
        path = Path(path).resolve()
        require(path.is_relative_to(ROOT), 'Artifact outside project')
        files[str(path.relative_to(ROOT))] = sha(path)
    receipts = new_receipts = 0
    for entry in metrics['runs']:
        run = Path(entry['run'])
        require(sha(run/'run_manifest.json') == entry['manifest_sha256'], 'Run identity changed')
        capture(run/'run_manifest.json'); capture(run/'identity.json')
        attempts = sorted((run/'jobs').glob('*/attempts/*/receipt.json'))
        require(len(attempts) == len(read(run/'run_manifest.json')['jobs']), 'Incorrect receipt coverage')
        receipts += len(attempts)
        if run.name.startswith('rss2025_paper_comparators_v1_'):
            new_receipts += len(attempts)
        for path in attempts:
            capture(path)
        for path in (run/'source_snapshot').rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts:
                capture(path)
    require((receipts, new_receipts) == (1330, 380), 'New and reused records conflated')
    for path in OUT.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts and path.name != 'seal.log':
            capture(path)
    for path in (report, ROOT/'README.md', BASE/'README.md'):
        capture(path)
    result = dict(schema_version=1, complete=True, created_utc=datetime.now(timezone.utc).isoformat(),
        command=sys.orig_argv, source_commit=frozen['source_commit'],
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        new_registered_slots=len(original), new_legal_calls=legal_original,
        new_invalid_slots=len(original)-legal_original,
        reused_registered_slots=950, combined_registered_slots=1330,
        combined_legal_calls=sum(r['input_legal'] for r in metrics['rows']),
        combined_status_counts=dict(Counter(r['status'] for r in metrics['rows'])),
        raw_receipts=receipts, new_test_planner_calls=0, files=files,
        integrity_scope='Reports and all1330 raw receipts are directly bound. Each verified receipt binds its raw arrays, controls, logs, requests and validation.950 observations are reused exactly once;380 are newly registered. No planner is run during sealing.',
        main_report=str(report))
    with destination.open('x') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    files[str(destination.relative_to(ROOT))] = sha(destination)
    with sums.open('x') as stream:
        for name,digest in sorted(files.items()):
            stream.write(f'{digest}  {name}\n')
    print(json.dumps(dict(complete=True,new_receipts=new_receipts,raw_receipts=receipts,
        direct_files=len(files),delivery_sha256=sha(destination)),indent=2))


if __name__ == '__main__':
    main()
