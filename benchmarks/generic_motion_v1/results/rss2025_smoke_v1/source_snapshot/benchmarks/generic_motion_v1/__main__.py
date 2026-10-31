"""Unified command line; use the repository's existing Conda Python launcher."""
import argparse
import json
from pathlib import Path
from .common import BASE, ROOT, write


def _guard_audit_output(output):
    if output is None:
        return
    freeze = BASE / 'manifests/test.SHA256SUMS'
    if freeze.exists():
        protected = {(ROOT / line.split(None, 1)[1]).resolve()
                     for line in freeze.read_text().splitlines() if line.strip()}
        if Path(output).resolve() in protected or Path(output).resolve() == freeze.resolve():
            raise ValueError('Audit output must not overwrite frozen evidence; choose a new report path')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    p = commands.add_parser("generate")
    p.add_argument("--split", choices=["smoke", "dev", "test"], required=True)
    p.add_argument("--seed", type=int, default=20261027)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--config", type=Path)
    p.add_argument("--acceptance", type=Path, help="Passing G0/G1/G2 receipt required to freeze test")
    p = commands.add_parser("audit")
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--output", type=Path)
    p = commands.add_parser("build-roadmap")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--nodes", type=int, default=6000)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--config", type=Path)
    for cmd in ("run", "batch"):
        p = commands.add_parser(cmd)
        p.add_argument("--manifest", type=Path, required=True)
        p.add_argument("--roadmaps", type=Path, nargs="+", required=True)
        p.add_argument("--output", type=Path, required=True)
        p.add_argument("--objectives", nargs="+", choices=["minimum_distance", "minimum_time"], default=["minimum_distance", "minimum_time"])
        p.add_argument("--methods", nargs="+", choices=["EI_ZO", "NTO"], default=["EI_ZO"])
        p.add_argument("--selection", choices=["all", "first-per-family"], default="all")
        p.add_argument("--problem", required=cmd == "run")
        p.add_argument("--timeout", type=float, default=60.)
        p.add_argument("--config", type=Path)
        p.add_argument("--prepare-only", action="store_true")
    for cmd in ("resume", "summarize", "verify"):
        p = commands.add_parser(cmd)
        p.add_argument("--run", type=Path, required=True)
        if cmd == "resume":
            p.add_argument("--max-jobs", type=int)
    p = commands.add_parser("replay")
    p.add_argument("--attempt", type=Path, required=True)
    p.add_argument("--output", type=Path)
    p = commands.add_parser("_worker", help=argparse.SUPPRESS)
    p.add_argument("--attempt", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "generate":
        from .generator import generate
        unlock = False
        if args.split == "test":
            if not args.acceptance:
                parser.error("Test freeze requires --acceptance with verified G0/G1/G2")
            receipt = json.loads(args.acceptance.read_text())
            unlock = all(receipt.get(g) is True for g in ("G0", "G1", "G2"))
            if not unlock:
                parser.error("G0/G1/G2 have not passed")
        result = generate(args.split, args.seed, args.output, config_path=args.config, unlock_test=unlock)
    elif args.command == "audit":
        from .generator import audit
        _guard_audit_output(args.output)
        result = audit(args.manifest, out=args.output)
    elif args.command == "build-roadmap":
        from .roadmap import build_roadmap
        result = build_roadmap(args.output, nodes=args.nodes, seed=args.seed, config=args.config)
    elif args.command in ("run", "batch"):
        from .batch import prepare, resume
        run = prepare(args.manifest, args.roadmaps, args.output, objectives=args.objectives,
            methods=args.methods, selection=args.selection, problem=args.problem, timeout=args.timeout, config=args.config)
        result = {"run": str(run)} if args.prepare_only else resume(run)
    elif args.command == "resume":
        from .batch import resume
        result = resume(args.run, max_jobs=args.max_jobs)
    elif args.command == "verify":
        from .batch import verify
        result = verify(args.run)
    elif args.command == "summarize":
        from .runners.summarize import summarize
        result = summarize(args.run)
    elif args.command == "replay":
        from .runners.replay import replay
        result = replay(args.attempt, output=args.output)
    else:
        from .batch import worker
        worker(args.attempt)
        return
    print(json.dumps(result, indent=2, default=str, allow_nan=False))
    if isinstance(result, dict) and (result.get("integrity_passed") is False or result.get("passed") is False):
        raise SystemExit(1)


if __name__ == "__main__":
    import datetime
    import os
    import sys
    import time
    ledger = BASE / "reports" / "cli_commands.jsonl"
    ledger.parent.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    record = dict(argv=[sys.executable, *sys.argv], cwd=os.getcwd(), pid=os.getpid(),
                  started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    with ledger.open("a") as stream:
        stream.write(json.dumps(dict(record, event="start")) + "\n")
    exit_code = 0
    try:
        main()
    except BaseException as exc:
        exit_code = exc.code if isinstance(exc, SystemExit) else 1
        raise
    finally:
        with ledger.open("a") as stream:
            stream.write(json.dumps(dict(record, event="finish", exit_code=exit_code,
                wall_seconds=time.perf_counter() - started)) + "\n")
