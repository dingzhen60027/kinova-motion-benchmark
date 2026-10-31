"""Regenerate the FR3 fit separately and compare exact numerical asset bytes."""

import argparse
import hashlib
from pathlib import Path
from reproduction.common import ROOT, atomic
from reproduction.paper_protocol import build_assets


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    build_assets.SOURCE = args.source_dir.resolve()
    build_assets.ASSETS = args.output.resolve()
    build_assets.main()
    rows = []
    for name in ["fr3_33.json", "franka_LICENSE", "franka_NOTICE"]:
        original = ROOT / "reproduction/paper_protocol/assets" / name
        rebuilt = args.output / name
        rows.append(
            dict(
                name=name,
                original_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
                rebuilt_sha256=hashlib.sha256(rebuilt.read_bytes()).hexdigest(),
                identical=original.read_bytes() == rebuilt.read_bytes(),
            )
        )
    atomic(
        args.output / "verification.json",
        dict(
            source_dir=str(args.source_dir),
            files=rows,
            passed=all(row["identical"] for row in rows),
        ),
    )
    if not all(row["identical"] for row in rows):
        raise ValueError("Rebuilt asset differs; original asset was not modified")
    print("All three regenerated asset/license files are byte-identical", flush=True)


if __name__ == "__main__":
    main()
