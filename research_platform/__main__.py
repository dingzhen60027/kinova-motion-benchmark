"""Single public entry point: python -m research_platform --help."""
import argparse
import json
from pathlib import Path
import sys

from .config import METHODS, case_grid, load_config, resolve_config
from .storage import atomic, read, verify_inputs, verify_run


def main(argv=None):
    parser = argparse.ArgumentParser(description="CSDecomp research workbench")
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("doctor", help="Check research dependencies and GPU/solver functionality")
    p.add_argument("--probe", action="store_true")
    p.add_argument("--dependencies-only", action="store_true")
    p.add_argument("--output", type=Path)
    p = sub.add_parser("config", help="Validate and show a fully resolved configuration")
    p.add_argument("path", type=Path)
    p.add_argument("--output", type=Path)
    p = sub.add_parser("run", help="Create a new run and execute its frozen job plan")
    p.add_argument("--config", type=Path, required=True)
    p.add_argument("--output", type=Path)
    p.add_argument("--prepare-only", action="store_true")
    p.add_argument("--case")
    p = sub.add_parser("resume", help="Continue pending/interrupted jobs without replacing failures")
    p.add_argument("--run", type=Path, required=True)
    p.add_argument("--case")
    p.add_argument("--retry-failed", action="store_true")
    p.add_argument("--retry-job", action="append", default=[])
    for name, help_text in [("status", "Show all planned jobs and outcomes"),
                            ("verify", "Verify integrity AND planned-job completeness"),
                            ("summarize", "Write full-denominator CSV/JSON comparisons")]:
        p = sub.add_parser(name, help=help_text)
        p.add_argument("--run", type=Path, required=True)
    p = sub.add_parser("replay", help="Render a case from saved inputs and trajectories")
    p.add_argument("--run", type=Path, required=True)
    p.add_argument("--case", required=True)
    p.add_argument("--output", type=Path)
    sub.add_parser("methods", help="List supported real method implementations")
    args = parser.parse_args(argv)
    try:
        if args.command == "methods":
            result = dict(methods=list(METHODS), variants=dict(
                LGCS="strict Gurobi tolerances", SCSTO="preserved initial knots",
                NTO="preserved pointwise collision baseline",
                NTO_strong="exact Forest segments / adaptive collocation, enriched time spline, cumulative refinement budget"))
        elif args.command == "doctor":
            from .diagnostics import dependencies, doctor
            result = dependencies() if args.dependencies_only else doctor(args.probe)
            if args.output:
                atomic(args.output, result)
            print(json.dumps(result, indent=2))
            return 0 if result.get("ready", result.get("passed")) else 1
        elif args.command == "config":
            result = resolve_config(load_config(args.path))
            if args.output:
                atomic(args.output, result)
        elif args.command == "run":
            from .runner import create_run, schedule
            run = create_run(args.config, args.output)
            print("Run: " + str(run), flush=True)
            result = dict(run=str(run), prepared=True)
            if not args.prepare_only:
                result = schedule(run, args.case)
                from .reporting import summarize_run
                summarize_run(run)
        elif args.command == "resume":
            from .runner import schedule
            result = schedule(args.run, args.case, args.retry_failed, args.retry_job)
            from .reporting import summarize_run
            summarize_run(args.run)
        elif args.command in ["status", "verify"]:
            result = verify_run(args.run, require_complete=args.command == "verify")
            print(json.dumps(result, indent=2))
            return 0 if result["verified"] else 2
        elif args.command == "summarize":
            from .reporting import summarize_run
            result = summarize_run(args.run)
        elif args.command == "replay":
            from .reporting import replay_case
            result = replay_case(args.run, args.case, args.output)
        print(json.dumps(result, indent=2, default=str))
        return 0
    except (ValueError, RuntimeError, OSError, KeyError) as exc:
        print(f"research-platform: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
