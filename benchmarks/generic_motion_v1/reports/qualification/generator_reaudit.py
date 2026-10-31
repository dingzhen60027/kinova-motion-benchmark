"""Input-only requalification of frozen splits; never constructs a planner."""
from __future__ import annotations

import json
import time
from pathlib import Path

from benchmarks.generic_motion_v1.generator.audit import audit
from benchmarks.generic_motion_v1.generator.manifest import file_sha256, json_text
from benchmarks.generic_motion_v1.model import Model


def main():
    base = Path(__file__).resolve().parents[2]
    output = Path(__file__).resolve().parent
    command = 'bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.generator_reaudit'
    model = Model()
    results = {}
    for split in ('smoke', 'dev', 'test'):
        manifest = base / 'manifests' / f'{split}.jsonl'
        destination = output / f'generator_{split}_audit.json'
        if destination.exists():
            raise FileExistsError(f'Refusing to overwrite qualification evidence: {destination}')
        before = file_sha256(manifest)
        start = time.perf_counter()
        print(f'START {split}; frozen manifest {before}', flush=True)
        report = audit(manifest, model=model, out=destination)
        elapsed = time.perf_counter() - start
        after = file_sha256(manifest)
        results[split] = dict(passed=report['passed'], errors=report['errors'],
            problem_count=report.get('problem_count'), scene_count=report.get('scene_count'),
            valid_input_count=report.get('valid_input_count'), invalid_input_count=report.get('invalid_input_count'),
            endpoint_proposals_replayed=report.get('endpoint_proposals_replayed'),
            proposal_collision_checks_repeated=report.get('proposal_collision_checks_repeated'),
            elapsed_seconds=elapsed, manifest_sha256_before=before, manifest_sha256_after=after,
            input_manifest_unchanged=before == after, report_sha256=file_sha256(destination),
            report_file=str(destination.relative_to(base)))
        print(json.dumps({'split': split, **results[split]}, sort_keys=True), flush=True)
    receipt = dict(command=command, input_only=True, test_planning_executed=False,
        passed=all(r['passed'] and r['input_manifest_unchanged'] for r in results.values()), splits=results)
    with (output / 'generator_reaudit_receipt.json').open('x') as stream:
        stream.write(json_text(receipt))
    print(json.dumps(receipt, indent=2), flush=True)
    return 0 if receipt['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
