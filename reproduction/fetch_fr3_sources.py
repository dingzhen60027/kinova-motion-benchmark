"""Fetch the pinned public FR3 geometry with the recorded source checksums."""

import argparse
import hashlib
from pathlib import Path
import urllib.request
from reproduction.common import ROOT, atomic, read


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--destination",
        type=Path,
        default=ROOT / "reproduction/vendor/franka_description",
    )
    args = parser.parse_args()
    assets = ROOT / "reproduction/paper_protocol/assets"
    model = read(assets / "fr3_33.json")
    sources = list(model["sources"])
    for name in ["LICENSE", "NOTICE"]:
        sources.append(
            dict(
                path=name,
                sha256=hashlib.sha256(
                    (assets / ("franka_" + name)).read_bytes()
                ).hexdigest(),
            )
        )
    rows = []
    base = (
        "https://raw.githubusercontent.com/frankarobotics/franka_description/"
        + model["source_revision"]
        + "/"
    )
    for source in sources:
        relative = Path(source["path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("Invalid source path")
        target = args.destination / relative
        if target.exists():
            content = target.read_bytes()
            action = "verified_existing"
        else:
            with urllib.request.urlopen(base + source["path"], timeout=60) as response:
                content = response.read()
            action = "downloaded"
        actual = hashlib.sha256(content).hexdigest()
        if actual != source["sha256"]:
            raise ValueError("Checksum mismatch: " + source["path"])
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(content)
        rows.append(dict(**source, bytes=len(content), action=action))
        print(action, source["path"], flush=True)
    atomic(
        args.destination / "fetch_manifest.json",
        dict(
            repository=model["source_repository"],
            revision=model["source_revision"],
            files=rows,
        ),
    )


if __name__ == "__main__":
    main()
