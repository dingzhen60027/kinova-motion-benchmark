"""Verify immutable test assets and retain the original frozen source provenance.

Run from repository root with the existing Conda launcher. Current hardened
source is allowed to differ only when an exact frozen source archive exists.
This is not permission to replace any scene, configuration or case.
"""
import argparse
import hashlib
import json
from pathlib import Path
from benchmarks.generic_motion_v1.common import ROOT, BASE, write


def check():
    archive = BASE / 'reports/qualification/frozen_v1_source'
    entries = []
    for line in (BASE / 'manifests/test.SHA256SUMS').read_text().splitlines():
        expected, relative = line.split(None, 1)
        current = ROOT / relative
        actual = hashlib.sha256(current.read_bytes()).hexdigest() if current.is_file() else None
        old = archive / relative
        archived = hashlib.sha256(old.read_bytes()).hexdigest() if old.is_file() else None
        matched = actual == expected
        source_preserved = relative.endswith('.py') and archived == expected
        entries.append(dict(path=relative, expected_sha256=expected, current_sha256=actual,
            frozen_source_archive_sha256=archived, current_matches_freeze=matched,
            passed=matched or source_preserved))
    return dict(passed=all(r['passed'] for r in entries), entries=entries,
        original_freeze_entry_count=len(entries),
        unchanged_current_files=sum(r['current_matches_freeze'] for r in entries),
        hardened_source_files=[r['path'] for r in entries if not r['current_matches_freeze']],
        rule='All non-Python frozen assets must still match; differing Python files require exact original source in the explicit archive. Original SHA list is never rewritten.')


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=check();write(args.output,result)
    print(json.dumps({k:v for k,v in result.items() if k!='entries'},indent=2))
    raise SystemExit(0 if result['passed'] else 1)
