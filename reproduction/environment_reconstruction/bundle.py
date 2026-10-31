"""Archive raw environment runs and verify every member by streaming readback."""
import hashlib
from pathlib import Path
import tarfile

from reproduction.common import ROOT, atomic


def hash_stream(stream):
    digest = hashlib.sha256()
    while chunk := stream.read(1024 * 1024):
        digest.update(chunk)
    return digest.hexdigest()


def main():
    base = ROOT / "reproduction/reports/environment_reconstruction"
    folders = ["run", "run_final", "preview_initial", "preview_initial_fixed"]
    entries = {}
    for folder in folders:
        for path in sorted((base / folder).rglob("*")):
            if path.is_symlink():
                raise ValueError("Unexpected symlink: " + str(path))
            if path.is_file():
                with path.open("rb") as stream:
                    entries[str(path.relative_to(base))] = dict(
                        bytes=path.stat().st_size, sha256=hash_stream(stream))
    # Repository-wide rules ignore logs. Preserve top-level and evidence-audit
    # logs too, but not this command's actively written stdout file.
    for path in sorted(base.rglob("*.log")):
        if path == base / "bundle.log":
            continue
        with path.open("rb") as stream:
            entries[str(path.relative_to(base))] = dict(
                bytes=path.stat().st_size, sha256=hash_stream(stream))
    archive = base / "evidence.tar.gz"
    with tarfile.open(archive, "w:gz") as tf:
        for name in entries:
            tf.add(base / name, arcname=name, recursive=False)
    failures = []
    with tarfile.open(archive, "r:gz") as tf:
        names = tf.getnames()
        if len(names) != len(entries) or set(names) != set(entries):
            raise ValueError("Archive member set mismatch")
        for name in names:
            member = tf.getmember(name)
            with tf.extractfile(member) as stream:
                actual = hash_stream(stream)
            if actual != entries[name]["sha256"] or member.size != entries[name]["bytes"]:
                failures.append(name)
    atomic(base / "evidence_manifest.json", dict(
        archive=archive.name, raw_folders=folders, files=entries))
    with archive.open("rb") as stream:
        archive_hash = hash_stream(stream)
    record = dict(archive=archive.name, bytes=archive.stat().st_size,
                  sha256=archive_hash, files=len(entries),
                  uncompressed_bytes=sum(x["bytes"] for x in entries.values()),
                  full_member_readback=True, mismatches=failures, passed=not failures)
    atomic(base / "evidence_receipt.json", record)
    print(record)
    if failures:
        raise SystemExit("Archive readback failed")


if __name__ == "__main__":
    main()
