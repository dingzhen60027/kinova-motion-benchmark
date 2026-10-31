"""Archive all new diagnostic evidence and verify every member by SHA-256."""
import argparse
import hashlib
from pathlib import Path
import tarfile
from reproduction.common import ROOT, atomic


def sha(path):
    with Path(path).open('rb') as file:
        return hashlib.file_digest(file, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else hashlib.sha256(file.read()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    output=args.output.resolve()
    if output.exists():
        raise FileExistsError('Refuse to replace a prior evidence archive')
    reports=ROOT/'reproduction/reports/alignment_diagnosis'
    manifest=reports/'evidence_manifest.json'
    receipt=reports/'evidence_receipt.json'
    exclusions={output,manifest,receipt}
    roots=[reports,ROOT/'reproduction/alignment_diagnosis']
    paths=[p for folder in roots for p in folder.rglob('*')
           if p.is_file() and not p.is_symlink() and '__pycache__' not in p.parts
           and p.resolve() not in exclusions]
    paths += [ROOT/'reproduction/diagnose_task_distribution.py',ROOT/'README.md',ROOT/'reproduction/README.md']
    entries=[dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(set(paths))]
    atomic(manifest,dict(scope='all new diagnostic evidence and execution source; prior v5 inputs remain in the prior archive',
                         files=entries,excluded_generated_metadata=[str(p.relative_to(ROOT)) for p in exclusions]))
    output.parent.mkdir(parents=True,exist_ok=True)
    with tarfile.open(output,'w:gz') as archive:
        for row in entries:
            archive.add(ROOT/row['path'],arcname=row['path'],recursive=False)
    expected={r['path']:r for r in entries}
    verified=0
    with tarfile.open(output,'r:gz') as archive:
        names=[]
        for member in archive:
            names.append(member.name)
            if not member.isfile() or member.name not in expected:
                raise ValueError('Unexpected archive entry: '+member.name)
            with archive.extractfile(member) as file:
                actual=hashlib.sha256(file.read()).hexdigest()
            if actual!=expected[member.name]['sha256'] or member.size!=expected[member.name]['bytes']:
                raise ValueError('Archive payload mismatch: '+member.name)
            verified+=1
        if len(names)!=len(set(names)) or set(names)!=set(expected):
            raise ValueError('Missing or duplicate archive member')
    unchanged=all(sha(ROOT/r['path'])==r['sha256'] for r in entries)
    record=dict(archive=str(output.relative_to(ROOT)),archive_bytes=output.stat().st_size,
                archive_sha256=sha(output),files=verified,uncompressed_bytes=sum(r['bytes'] for r in entries),
                every_member_rehashed=True,source_files_unchanged_during_packaging=unchanged,
                manifest_sha256=sha(manifest),
                prior_inputs='reproduction/artifacts/csdecomp_paper_constructed_20261026.tar.gz; not duplicated here')
    atomic(receipt,record)
    print(record)
    if not unchanged:raise RuntimeError('Evidence changed during packaging')


if __name__=='__main__':main()
