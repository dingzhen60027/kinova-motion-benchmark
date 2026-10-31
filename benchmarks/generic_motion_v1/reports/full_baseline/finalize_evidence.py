"""Read-only final input/dependency checks and an updated command inventory."""
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess

from ...common import ROOT, BASE, read, sha, write

OUT = Path(__file__).resolve().parent


def main():
    completion = read(OUT / 'completion.json')
    if not completion['complete'] or completion['total_jobs'] != 950:
        raise ValueError('All preregistered runs must be sealed first')
    dependencies = read(OUT / 'dependency_files.json')['files']
    dependency_checks = {name: sha(Path(name)) == item['sha256']
                         for name, item in dependencies.items()}
    archive = BASE / 'reports/qualification/frozen_v1_source'
    freeze = []
    for line in (BASE / 'manifests/test.SHA256SUMS').read_text().splitlines():
        expected, name = line.split(None, 1)
        current = sha(ROOT / name)
        archived = sha(archive / name) if (archive / name).is_file() else None
        passed = current == expected or (name.endswith('.py') and archived == expected)
        freeze.append(dict(path=name, expected_sha256=expected, current_sha256=current,
                           frozen_source_archive_sha256=archived,
                           current_matches_freeze=current == expected, passed=passed))
    freeze_result = dict(passed=all(e['passed'] for e in freeze), entries=freeze,
        original_freeze_entry_count=len(freeze),
        unchanged_current_files=sum(e['current_matches_freeze'] for e in freeze),
        hardened_source_files=[e['path'] for e in freeze if not e['current_matches_freeze']],
        rule='All non-Python frozen assets must still match; differing Python files require exact original source in the explicit archive. Original SHA list is never rewritten.')
    write(OUT / 'test_freeze_integrity.json', freeze_result)
    cuda_diff = subprocess.check_output(
        ['git', 'diff', 'origin/main', '--', 'csdecomp/src'], cwd=ROOT, text=True)
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    analyses = [read(OUT / f'rss2025_full_{split}_v1_analysis.json') for split in ('smoke', 'dev')]
    native_errors = []
    for analysis in analyses:
        for row in analysis['records']:
            if row['status'] != 'infrastructure_error':
                continue
            attempt = BASE / row['attempt']
            item = dict(job=row['job_id'], method=row['method'], objective=row['objective'],
                        error_category=row['error_category'])
            for field, filename in [('phase', 'phase.json'), ('endpoint', 'endpoint_audit.json')]:
                if (attempt / filename).is_file():
                    item[field] = read(attempt / filename)
            for filename in ('stderr.log', 'stdout.log'):
                if (attempt / filename).is_file():
                    item[filename + '_tail'] = (attempt / filename).read_text(errors='replace').splitlines()[-20:]
            native_errors.append(item)
    # Preserve the earlier diagnosis, whose endpoint/phase filenames may differ.
    write(OUT / 'final_infrastructure_failures.json', dict(records=native_errors,
        scope='All sealed smoke/dev infrastructure errors; original complete logs remain in each attempt'))
    commands = [json.loads(line) for line in (BASE / 'reports/cli_commands.jsonl').read_text().splitlines()
                if 'rss2025_full_' in line or 'reports/full_baseline/replays/' in line]
    write(OUT / 'actual_cli_commands.json', dict(scope='Actual CLI start/finish events, including worker and replay invocations',
                                                events=commands))
    scope = read(OUT / 'paper_reproduction_scope.json')
    scope['recorded_utc'] = datetime.now(timezone.utc).isoformat()
    for track in scope['tracks']:
        if track['track'] == 'kinova7_generic_v1':
            track.update(state='five_method_smoke_and_dev_completed', completed_jobs=950,
                         smoke_jobs=50, dev_jobs=900,
                         development_status_counts=[dict(method=g['method'], objective=g['objective'],
                             status_counts=g['status_counts']) for g in analyses[1]['groups']])
    write(OUT / 'paper_reproduction_scope.json', scope)
    checks = dict(recorded_utc=datetime.now(timezone.utc).isoformat(),
        runtime_head_before_evidence_commit=head, frozen_runtime_commit='72984027030175b7bca78e2dcf592b38f0c1864b',
        dependency_files_unchanged=dependency_checks, test_freeze_passed=freeze_result['passed'],
        upstream_cuda_source_diff_empty=not cuda_diff, completed_jobs=completion['total_jobs'],
        paired_path_mismatches=completion['paired_path_mismatches'],
        paired_frontend_mismatches=completion['paired_frontend_mismatches'],
        formal_test_executed=False)
    checks['passed'] = (all(dependency_checks.values()) and freeze_result['passed'] and not cuda_diff
                        and head == checks['frozen_runtime_commit']
                        and completion['paired_path_mismatches'] == completion['paired_frontend_mismatches'] == 0)
    write(OUT / 'final_checks.json', checks)
    if not checks['passed']:
        raise ValueError('Final evidence checks failed; see final_checks.json')
    print(json.dumps(dict(passed=True, jobs=950, dependency_files=len(dependencies),
                          frozen_entries=len(freeze), infrastructure_failures=len(native_errors))))


if __name__ == '__main__':
    main()
