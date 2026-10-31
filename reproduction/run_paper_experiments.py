"""Record and run the two frozen paper-parameter simulation families serially."""

import argparse
import datetime
import json
from pathlib import Path
import subprocess
import time
from reproduction.common import ROOT, atomic


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache-from", help="Optional compatible FR3 scene/map cache")
    parser.add_argument("--ledger", type=Path, help="New command ledger path")
    args = parser.parse_args()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    ledger = args.ledger or ROOT / "reproduction" / (stamp + "_paper_commands.json")
    if ledger.exists():
        raise FileExistsError("Preserve the previous command ledger: " + str(ledger))
    rows = []
    for family in ["forest", "fr3"]:
        cmd = [
            "bash",
            "local/python.sh",
            "-u",
            "-m",
            "reproduction.paper_protocol.run",
            "run",
            "--environment",
            family,
        ]
        if family == "fr3" and args.cache_from:
            cmd += ["--cache-from", args.cache_from]
        log = ROOT / f"reproduction/logs/{stamp}_paper_{family}_full.log"
        log.parent.mkdir(parents=True, exist_ok=True)
        row = dict(
            label=f"paper_v3_{family}_full",
            command=cmd,
            started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            status="running",
            log=str(log.relative_to(ROOT)),
        )
        rows.append(row)
        atomic(ledger, rows)
        started = time.perf_counter()
        with log.open("w") as stream:
            result = subprocess.run(
                cmd, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT
            )
        row.update(
            status="completed" if result.returncode == 0 else "failed",
            exit_code=result.returncode,
            elapsed_seconds=time.perf_counter() - started,
            completed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        )
        for line in log.read_text().splitlines():
            if line.startswith("RUN_DIR="):
                row["run_dir"] = line.split("=", 1)[1]
        atomic(ledger, rows)
        print(row["label"], row["exit_code"], row["elapsed_seconds"], flush=True)
        if result.returncode:
            return result.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
