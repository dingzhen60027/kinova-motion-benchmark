"""Persist the exact command, exit status and log for a verification step."""

import argparse
import datetime
import hashlib
from pathlib import Path
import subprocess
import time
from reproduction.common import ROOT, atomic, read


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    cmd = args.command[1:] if args.command and args.command[0] == "--" else args.command
    if not cmd or not args.label.replace("_", "").replace("-", "").isalnum():
        raise ValueError("Supply a command and a simple unique label")
    path = ROOT / "reproduction/reports/paper_completion/verification_commands.json"
    rows = read(path) if path.exists() else []
    if any(row["label"] == args.label for row in rows):
        raise ValueError("Use a new label to preserve previous attempts")
    log = ROOT / "reproduction/logs" / ("completion_verification_" + args.label + ".log")
    log.parent.mkdir(parents=True, exist_ok=True)
    row = dict(
        label=args.label,
        command=cmd,
        status="running",
        started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        log=str(log.relative_to(ROOT)),
        recorder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    )
    rows.append(row)
    atomic(path, rows)
    tic = time.perf_counter()
    with log.open("w") as stream:
        result = subprocess.run(cmd, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
    row.update(
        status="completed" if result.returncode == 0 else "failed",
        exit_code=result.returncode,
        elapsed_seconds=time.perf_counter() - tic,
        completed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    )
    for line in log.read_text().splitlines():
        if line.startswith("THROUGHPUT_DIR="):
            row["run_dir"] = line.split("=", 1)[1]
    atomic(path, rows)
    print(row, flush=True)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
