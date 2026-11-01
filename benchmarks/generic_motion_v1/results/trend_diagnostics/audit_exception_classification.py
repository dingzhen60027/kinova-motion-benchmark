"""Read small sealed metadata to audit historical exception compatibility."""
from pathlib import Path
import argparse
import collections
import hashlib
import json
import sys

ROOT = next(path for path in Path(__file__).resolve().parents if (path/'scripts/evaluate.py').is_file())
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.runners.failure_classification import (
    LEGACY_DRM_NO_ROUTE, exception_needs_attention, legacy_drm_failure_evidence)


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--selection', default='results/selection.json')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    selection = ROOT/args.selection
    report = dict(command=[sys.executable, *sys.argv], selection=str(selection),
        selection_sha256=digest(selection), scope='current_dev/current_test first sealed terminal attempts; only small JSON and exception traces; no arrays, planners, GPU or raw mutations',
        comparisons={}, source_manifests_sha256={}, failures=[])
    for comp in read(selection)['comparisons']:
        if comp['id'] not in ('current_dev','current_test'):
            continue
        total = 0
        groups = collections.Counter()
        examples = {}
        for spec in comp['runs']:
            root = ROOT/spec['path']; manifest = read(root/'run_manifest.json')
            report['source_manifests_sha256'][str(root/'run_manifest.json')] = digest(root/'run_manifest.json')
            for job in manifest['jobs']:
                if (('methods' in spec and job['method'] not in spec['methods']) or
                        ('objectives' in spec and job['objective'] not in spec['objectives'])):
                    continue
                for attempt in sorted((root/'jobs'/job['job_id']/'attempts').glob('[0-9][0-9][0-9][0-9]')):
                    if not (attempt/'receipt.json').exists():
                        continue
                    receipt = read(attempt/'receipt.json')
                    if receipt.get('retry_pending'):
                        continue
                    result = read(attempt/'result.json'); total += 1
                    if result.get('exception'):
                        legacy = legacy_drm_failure_evidence(attempt,result,
                            source_hashes=manifest['source_hashes'],receipt=receipt)
                        attention = exception_needs_attention(result,legacy_drm_evidence=legacy)
                        key = (result['status'],result.get('failed_stage'),result['exception'],attention,bool(legacy))
                        groups[key] += 1; examples.setdefault(key,str(attempt))
                        if result['exception'] == repr(RuntimeError(LEGACY_DRM_NO_ROUTE)) and (not legacy or attention):
                            report['failures'].append(dict(attempt=str(attempt),reason='Historical no-route lost its receipt-bound interpretation'))
                    break
        report['comparisons'][comp['id']] = dict(attempts=total, exceptions=[dict(
            count=count,raw_status=key[0],phase=key[1],exception=key[2],requires_attention=key[3],
            legacy_source_trace_and_receipt_verified=key[4],example=examples[key]) for key,count in groups.items()])
    report['passed'] = not report['failures']
    output = ROOT/args.output
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(output),passed=report['passed'],
        attempts={key:value['attempts'] for key,value in report['comparisons'].items()},
        failures=len(report['failures']))))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
