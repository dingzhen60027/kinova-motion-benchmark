"""Check every saved CUDA/Drake disagreement against the fixed contact band."""
from pathlib import Path
import numpy as np
from ...common import BASE, read, write, sha
from ...batch import verify

OUT = Path(__file__).resolve().parent


def main():
    records, issues, count, points = [], [], 0, 0
    for name in ('rss2025_full_smoke_v1', 'rss2025_full_dev_v1'):
        run = BASE / 'results' / name
        check = verify(run)
        if not check['complete'] or not check['integrity_passed']:
            raise ValueError(f'Incomplete evidence: {name}')
        for path in sorted((run / 'jobs').rglob('trajectory_samples.npz')):
            count += 1
            with np.load(path, allow_pickle=False) as arrays:
                if not {'native', 'independent', 'clearance'} <= set(arrays.files):
                    issues.append(dict(file=str(path), error='Missing collision flags/clearance'))
                    continue
                native = arrays['native']; independent = arrays['independent']
                clearance = arrays['clearance']; mismatch = native != independent
                points += len(clearance)
                if mismatch.any():
                    indices = np.flatnonzero(mismatch)
                    max_distance = float(np.max(np.abs(clearance[mismatch])))
                    row = dict(file=str(path.relative_to(BASE)), sha256=sha(path),
                        indices=indices.tolist(), clearance_m=clearance[mismatch].tolist(),
                        maximum_absolute_clearance_m=max_distance,
                        inside_fixed_contact_band=bool(max_distance <= 2e-6))
                    records.append(row)
                    if not row['inside_fixed_contact_band']:
                        issues.append(row)
    result = dict(passed=not issues, checked_files=count, checked_saved_samples=points,
        sample_count_scope='All stored files including repeated root/round/seed copies; not independent observations',
        contact_band_m=2e-6, disagreement_files=records, issues=issues,
        note='Reads original runtime classifications; does not recompute collision geometry or certify continuous safety')
    write(OUT / 'contact_disagreement_audit.json', result)
    if issues:
        raise ValueError('Collision classifications differ outside the fixed contact band')
    print({k: v for k, v in result.items() if k not in ('disagreement_files', 'issues')})


if __name__ == '__main__':
    main()
