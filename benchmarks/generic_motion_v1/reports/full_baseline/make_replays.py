"""Render saved completed attempts; selection affects illustrations only."""
from collections import defaultdict
from pathlib import Path
import subprocess
from ...common import read, write, ROOT, BASE

OUT = Path(__file__).resolve().parent
LABELS = {('EI_ZO', 'minimum_distance'): 'LSCS', ('LGCS', 'minimum_distance'): 'LGCS',
          ('NTO_PAPER', 'minimum_distance'): 'NTO_distance',
          ('EI_ZO', 'minimum_time'): 'SCSTO', ('NTO_PAPER', 'minimum_time'): 'NTO_time'}


def main():
    data = read(OUT / 'rss2025_full_dev_v1_analysis.json')
    if not data['integrity']['complete'] or not data['integrity']['integrity_passed']:
        raise ValueError('A complete sealed development run is required')
    successes = [r for r in data['records'] if r['status'] == 'success_validated_numerical']
    pairs = defaultdict(list)
    for row in successes:
        pairs[(row['case_id'], row['roadmap_id'])].append(row)
    common = [rows for _, rows in sorted(pairs.items()) if len(rows) == 5]
    blocked = [rows for rows in common if all(r['straight_line_collision'] for r in rows)]
    selected = (blocked or common)[0] if common else [
        sorted([r for r in successes if (r['method'], r['objective']) == key],
               key=lambda r: (not r['straight_line_collision'], r['job_id']))[0] for key in LABELS]
    choices = [(LABELS[(r['method'], r['objective'])], r) for r in selected]
    failures = sorted([r for r in data['records'] if r['method'] == 'NTO_PAPER'
                       and r['status'] == 'collision_failed'], key=lambda r: r['job_id'])
    if failures:
        choices.append(('NTO_collision_failure', failures[0]))
    licenses = sorted([r for r in data['records'] if r['method'] == 'LGCS'
                       and r['status'] == 'unsupported'], key=lambda r: r['job_id'])
    if licenses:
        choices.append(('LGCS_license_failure', licenses[0]))
    index = []
    for name, row in choices:
        target = OUT / 'replays' / name
        command = ['bash', 'local/python.sh', '-m', 'benchmarks.generic_motion_v1', 'replay',
                   '--attempt', str(BASE / row['attempt']), '--output', str(target)]
        completed = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        (OUT / f'replay_{name}.log').write_text(completed.stdout + completed.stderr)
        index.append(dict(label=name, job_id=row['job_id'], case_id=row['case_id'],
            status=row['status'], blocked_straight_line=row['straight_line_collision'],
            input_attempt=row['attempt'], output=str(target.relative_to(BASE)),
            command=command, returncode=completed.returncode))
        write(OUT / 'replay_index.json', dict(selection='First lexicographic common-success case, prefer blocked straight line; failed illustrations separately selected. No evaluation record is filtered.',
            shared_case_for_five_methods=bool(common), replays=index))
        if completed.returncode:
            raise RuntimeError(f'Replay failed for {row["job_id"]}; saved actual log')


if __name__ == '__main__':
    main()
