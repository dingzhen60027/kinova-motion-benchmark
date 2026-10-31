"""Archive this platform acceptance's evidence and verify every file by reading it back."""
import hashlib
import json
from pathlib import Path
import tarfile

from research_platform.storage import ROOT, atomic, sha, verify_run


def main():
    out = ROOT / 'research_platform/reports'
    archive = out / 'evidence.tar.gz'
    reserved = {'evidence.tar.gz', 'evidence_manifest.json', 'evidence_receipt.json', 'bundle.log'}
    runs = sorted((ROOT / 'runs').glob('platform_*'))
    sources = list(out.rglob('*')) + [p for run in runs for p in run.rglob('*')]
    files, excluded = {}, []
    for p in sorted(set(sources)):
        if p.name in reserved or any(x in p.parts for x in ['__pycache__', '.pytest_cache']):
            continue
        relative = str(p.relative_to(ROOT))
        if p.is_symlink():
            excluded.append(dict(path=relative, reason='test fixture symlink; never dereference', target=str(p.readlink())))
        elif p.is_file():
            files[relative] = dict(sha256=sha(p), bytes=p.stat().st_size)
    verification = {str(run.relative_to(ROOT)):verify_run(run, require_complete=False) for run in runs}
    atomic(out / 'evidence_manifest.json', dict(files=files, excluded=excluded, runs=verification))
    with tarfile.open(archive, 'w:gz') as tar:
        for relative in files:
            tar.add(ROOT / relative, arcname=relative, recursive=False)
    seen = {}
    with tarfile.open(archive, 'r:gz') as tar:
        for member in tar:
            if not member.isfile():
                raise ValueError('Unexpected archive member')
            stream = tar.extractfile(member)
            h = hashlib.sha256()
            while block := stream.read(1024*1024): h.update(block)
            seen[member.name] = dict(sha256=h.hexdigest(), bytes=member.size)
    if seen != files:
        raise ValueError('Archive readback differs from source evidence')
    receipt = dict(archive_sha256=sha(archive), archive_bytes=archive.stat().st_size,
                   file_count=len(files), all_members_readback_verified=True,
                   manifest_sha256=sha(out/'evidence_manifest.json'),
                   scope='Final v4 acceptance plus earlier platform runs, raw tests and failure replays; excludes caches and test symlinks')
    atomic(out / 'evidence_receipt.json', receipt)
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__': main()
