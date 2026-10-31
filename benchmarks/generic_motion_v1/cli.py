"""Small public facade over the existing dataset, batch and replay modules."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

from .common import BASE, ROOT, read, resolve_case
from .runners.methods import METHODS

ACTIVE_METHODS = tuple(METHODS)


def emit(value):
    print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False, default=str))


def require_environment():
    expected = Path(os.environ.get('CSDECOMP_CONDA_PREFIX', ROOT / '.conda/envs/csdecomp')).expanduser().resolve()
    if Path(sys.prefix).resolve() != expected:
        raise RuntimeError('Use the existing Conda environment: bash local/python.sh scripts/evaluate.py ...')


def project_path(value):
    path = Path(value).expanduser()
    return path if path.is_absolute() else ROOT / path


def benchmark_main(argv=None):
    p = argparse.ArgumentParser(description='One fixed Kinova dataset: inspect, check and reproduce inputs.')
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('index', help='Rebuild reference index and problem feature table, without changing inputs')
    sub.add_parser('check', help='Check all input hashes, counts and reference identities')
    s = sub.add_parser('show'); s.add_argument('problem')
    s = sub.add_parser('generate', help='Reproduce a split into an explicitly separate destination')
    s.add_argument('--split', required=True, choices=['smoke', 'dev', 'test'])
    s.add_argument('--seed', type=int, help='Defaults to the frozen split generation seed')
    s.add_argument('--output', required=True)
    s.add_argument('--config', help='Defaults to the frozen split generator configuration')
    s = sub.add_parser('build-roadmap')
    s.add_argument('--nodes', type=int, default=6000)
    s.add_argument('--seed', type=int, default=0)
    s.add_argument('--output', required=True)
    s.add_argument('--config', default='configs/evaluation.json')
    args = p.parse_args(argv)
    require_environment()
    if args.command in ('index', 'check', 'show'):
        from . import dataset
        translated = {'index': 'build', 'check': 'check', 'show': 'show'}[args.command]
        parameters = [translated]
        if args.command == 'show': parameters += ['--problem', args.problem]
        # The dataset module owns index validation and feature semantics.
        dataset.main(parameters)
        return
    if args.command == 'generate':
        from .generator import generate
        output = project_path(args.output)
        resolved = output.resolve()
        if resolved == BASE.resolve() or resolved.is_relative_to(BASE.resolve()):
            p.error('Generate into a separate reproduction directory; canonical inputs remain unchanged')
        if output.exists() or output.is_symlink():
            p.error('Generation output must be a new path; existing data is never overwritten')
        seed = (args.seed if args.seed is not None else
                read(BASE / f'receipts/{args.split}_generation.json')['seed'])
        config = (project_path(args.config) if args.config else
                  BASE / f'configs/{args.split}_generator.json')
        emit(generate(args.split, seed, output, config_path=config,
                      unlock_test=args.split == 'test'))
    else:
        from .roadmap import build_roadmap
        emit(build_roadmap(project_path(args.output), nodes=args.nodes, seed=args.seed,
                           config=project_path(args.config)))


def evaluate_main(argv=None):
    p = argparse.ArgumentParser(description='Run the existing native/adapted baselines with one validator.')
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('methods')
    sub.add_parser('check', help='Real environment, collision, solver and negative-case checks')
    for cmd in ('run', 'batch'):
        s = sub.add_parser(cmd)
        s.add_argument('--split', choices=['smoke', 'dev', 'test'], default='smoke' if cmd == 'run' else 'dev')
        s.add_argument('--problem', required=cmd == 'run')
        s.add_argument('--methods', nargs='+', choices=ACTIVE_METHODS, default=list(ACTIVE_METHODS))
        s.add_argument('--track', choices=['path', 'time', 'both'], default='both')
        s.add_argument('--seeds', nargs='+', type=int)
        s.add_argument('--config', default='configs/evaluation.json')
        s.add_argument('--output', required=True)
        s.add_argument('--mode', choices=['end_to_end','fixed_initial_regions'], default='end_to_end')
        s.add_argument('--initial-source-run')
        s.add_argument('--prepare-only', action='store_true')
        s.add_argument('--max-jobs', type=int)
    for cmd in ('resume', 'verify', 'status'):
        s = sub.add_parser(cmd); s.add_argument('--run', required=True)
        if cmd == 'resume': s.add_argument('--max-jobs', type=int)
    s = sub.add_parser('summarize')
    s.add_argument('--selection', default='results/selection.json')
    s.add_argument('--output', default='results')
    s = sub.add_parser('replay')
    s.add_argument('--attempt', required=True); s.add_argument('--output', required=True)
    args = p.parse_args(argv)
    require_environment()
    if args.command == 'methods':
        from .runners.methods import catalog
        emit([r for r in catalog() if r['method'] in ACTIVE_METHODS]); return
    if args.command == 'check':
        from .health import check
        result = check(ROOT / 'results/checks.json'); emit(result)
        if not result['passed']: raise SystemExit(1)
        return
    if args.command in ('run', 'batch'):
        from .batch import prepare, resume
        from .roadmap import load_config
        cfg = load_config(project_path(args.config))
        seeds = args.seeds if args.seeds is not None else ([0, 1, 2] if args.split == 'dev' else [0])
        objectives = {'path': ['minimum_distance'], 'time': ['minimum_time'],
                      'both': ['minimum_distance', 'minimum_time']}[args.track]
        from .runners.methods import INDEPENDENT
        independent = set(args.methods) <= INDEPENDENT
        roadmaps = None if independent else [BASE / f'roadmaps/rss2025_n6000_s{seed:03d}' for seed in seeds]
        run = prepare(BASE / f'manifests/{args.split}.jsonl', roadmaps, project_path(args.output),
            methods=args.methods, objectives=objectives, problem=args.problem,
            seed_indices=seeds if independent else None, timeout=cfg['online_budget_seconds'], config=cfg,
            allow_test=True, snapshot_sources=False,mode=args.mode,
            initial_source_run=project_path(args.initial_source_run) if args.initial_source_run else None)
        emit({'run': str(run), 'prepared': True} if args.prepare_only else resume(run, max_jobs=args.max_jobs))
    elif args.command in ('resume', 'verify', 'status'):
        from .batch import resume, verify
        run = project_path(args.run)
        if args.command == 'resume': result = resume(run, max_jobs=args.max_jobs)
        elif args.command == 'verify': result = verify(run)
        else:
            from .runners.results import run_status
            result = run_status(run)
        emit(result)
        if result.get('integrity_passed') is False: raise SystemExit(1)
    elif args.command == 'summarize':
        from .runners import results
        results.main(['--output', str(project_path(args.output)), '--selection', str(project_path(args.selection))])
    else:
        from .runners.replay import replay
        emit(replay(project_path(args.attempt), output=project_path(args.output)))
