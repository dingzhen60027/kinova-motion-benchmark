"""Package source, evidence, licenses and measured inputs without large native maps."""

import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
from reproduction.common import ROOT, atomic, read


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "reproduction/artifacts/csdecomp_paper_protocol_20261025.tar.gz",
    )
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    files = {}
    tracked = (
        subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT)
        .decode()
        .split("\0")
    )
    for name in tracked:
        if name and (name.startswith("reproduction/") or name == "README.md"):
            files[name] = (ROOT / name).read_bytes()
    for folder in [
        "reproduction/vendor/scsplanning",
        "reproduction/vendor/franka_description",
        "reproduction/logs",
    ]:
        for path in (ROOT / folder).rglob("*"):
            if path.is_file() and not any(
                x in path.parts for x in [".git", "__pycache__", ".venv"]
            ):
                if (
                    folder.endswith("logs")
                    or path.suffix
                    in [
                        ".py",
                        ".toml",
                        ".txt",
                        ".md",
                        ".yaml",
                        ".xacro",
                        ".stl",
                        ".json",
                    ]
                    or path.name in ["LICENSE", "NOTICE"]
                ):
                    files[str(path.relative_to(ROOT))] = path.read_bytes()
    files["local/python.sh"] = (ROOT / "local/python.sh").read_bytes()
    index = read(ROOT / "reproduction/reports/paper_protocol/run_index.json")
    for family in ["forest", "fr3"]:
        run = Path(index[family])
        paths = [run / "manifest.json"] + list(run.glob("gpu_*.txt"))
        if (run / "cache_provenance.json").exists():
            paths.append(run / "cache_provenance.json")
        paths += (
            list((run / "scenes").glob("*/scene.json"))
            + list((run / "roadmaps").glob("*/roadmap.json"))
            + list((run / "roadmaps").glob("*/samples.npy"))
        )
        paths += list((run / "cases").glob("*/input.json")) + list(
            (run / "cases").glob("*/seed_path.npz")
        )
        for path in paths:
            files["measured_inputs/" + family + "/" + str(path.relative_to(run))] = (
                path.read_bytes()
            )
    for path in (ROOT / "reproduction/diagnostics/paper_protocol").iterdir():
        if path.is_file() and path.suffix in [".log", ".json"]:
            files["preserved_diagnostics/" + path.name] = path.read_bytes()
    metadata = dict(
        packaged_at_commit=subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        upstream_base="7661a97bb7c4931be73d2bc4f8c45c3094ff476e",
        measured_implementation_hash=read(Path(index["forest"]) / "manifest.json")[
            "implementation_hash"
        ],
        scope="Apply over upstream checkout. Contains source, locks, reports, figures, logs, third-party source/licenses, actual scenes, roadmap sample nodes and seed paths. Large native .map files and full per-method trajectory arrays remain in local raw runs and are not included.",
    )
    files["BUNDLE_METADATA.json"] = (json.dumps(metadata, indent=2) + "\n").encode()
    inventory = {
        name: dict(bytes=len(content), sha256=hashlib.sha256(content).hexdigest())
        for name, content in sorted(files.items())
    }
    files["BUNDLE_FILES.json"] = (json.dumps(inventory, indent=2) + "\n").encode()
    with args.output.open("wb") as output, gzip.GzipFile(
        fileobj=output, mode="wb", mtime=0
    ) as compressed, tarfile.open(fileobj=compressed, mode="w|") as archive:
        for name, content in sorted(files.items()):
            entry = tarfile.TarInfo(name)
            entry.size = len(content)
            entry.mode = 0o644
            archive.addfile(entry, io.BytesIO(content))
    with tarfile.open(args.output, "r:gz") as archive:
        for name, expected in inventory.items():
            actual = archive.extractfile(name).read()
            if (
                len(actual) != expected["bytes"]
                or hashlib.sha256(actual).hexdigest() != expected["sha256"]
            ):
                raise ValueError("Bundle verification failed: " + name)
    result = dict(
        path=str(args.output.resolve()),
        bytes=args.output.stat().st_size,
        sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
        files=len(files),
        all_contents_verified=True,
        **metadata
    )
    atomic(args.output.with_suffix(".manifest.json"), result)
    print(result, flush=True)


if __name__ == "__main__":
    main()
