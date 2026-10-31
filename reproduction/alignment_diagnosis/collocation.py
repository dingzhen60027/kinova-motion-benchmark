"""Locate saved Forest NTO failures relative to the actual bound constraints.

This is a post-hoc audit of every saved v5 Forest NTO candidate, not a new
benchmark. The analytic circle oracle is independent of Drake's smooth
minimum-distance constraint. Failed solver candidates are audited separately.
"""
import argparse
from collections import Counter
import hashlib
from pathlib import Path
import time

import numpy as np
from pydrake.all import BsplineBasis, BsplineTrajectory
from reproduction.common import ROOT, atomic, read
from reproduction.paper_constructed.model import ForestEnvironment


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    index = read(ROOT / 'reproduction/reports/paper_constructed/run_index.json')
    run = Path(index['forest']['run_dir'])
    manifest = read(run / 'manifest.json')
    records, identities = [], {}
    started = time.perf_counter()
    atomic(args.output / 'scope.json', dict(
        input_run=run, input_manifest_sha256=sha(run / 'manifest.json'),
        selection='all 400 saved cases, both NTO objectives, including solver failures',
        kind='posthoc_diagnostic_no_optimization_no_reselection',
        hypotheses=['between-collocation collision', 'collocation violation beyond solver tolerance',
                    'smooth-constraint tolerance versus physical clearance'],
        source_sha256=sha(__file__),
    ))
    environments = {}
    constraints = {}
    for case in manifest['cases']:
        seed = case['environment_seed']
        if seed not in environments:
            environments[seed] = ForestEnvironment(manifest['config'], seed)
            constraints[seed] = environments[seed].constraint()
        env, constraint = environments[seed], constraints[seed]
        for method in ['NTO_distance', 'NTO_time']:
            folder = run / 'cases' / case['id'] / method
            paths = [folder / n for n in ['optimization_00.json', 'candidate_00.npz',
                                         'validation.json', 'result.json']]
            for path in paths:
                identities[str(path.relative_to(run))] = sha(path)
            meta, saved, valid, result = read(paths[0]), np.load(paths[1]), read(paths[2]), read(paths[3])
            curve = BsplineTrajectory(BsplineBasis(meta['spline_order'], saved['knots'].tolist()),
                                     [q[:, None] for q in saved['controls']])
            s = np.array(meta['collocation_locations'])
            q = curve.vector_values(curve.start_time() + s * meta['duration']).T
            clearances = env.base.clearances(q)
            evaluator = np.array([constraint.Eval(x)[0] for x in q])
            ub = float(constraint.upper_bound()[0])
            residual = evaluator - ub
            exact = env.base.validate_polygon(saved['controls']) if meta['spline_order'] == 2 else None
            solver_ok = meta['solver_success']
            # Solver tolerance is in normalized smooth-constraint units, not metres.
            feasible = bool(residual.max() <= 1e-6)
            collision = not valid['collision_free']
            if not solver_ok:
                category = 'solver_failed'
            elif not feasible:
                category = 'reported_success_but_collocation_residual_exceeds_1e-6'
            elif collision and clearances.min() >= 0:
                category = 'between_collocation_collision'
            elif collision:
                category = 'collocation_penetration_within_smooth_constraint_tolerance'
            else:
                category = 'validated_collision_free'
            record = dict(case=case['id'], environment_seed=seed, method=method,
                          solver_success=solver_ok, strict_success=result['status']=='success_numerical',
                          saved_collision_free=valid['collision_free'],
                          collocation_min_clearance=float(clearances.min()),
                          max_smooth_constraint_residual=float(residual.max()),
                          collocation_negative_points=int((clearances < 0).sum()),
                          collocation_parameter_Nc=len(s), category=category,
                          exact_path_min_clearance=exact['minimum_clearance'] if exact else None,
                          control_count=meta['control_points'])
            records.append(record)
            dest = args.output / 'samples' / case['id']
            dest.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(dest / (method + '.npz'), s=s, q=q, clearances=clearances,
                                evaluator=evaluator, residual=residual)
    verified = all(sha(run / name) == expected for name, expected in identities.items())
    atomic(args.output / 'input_hashes.json', identities)
    atomic(args.output / 'records.json', records)
    summary = dict(input_files=len(identities), input_reverification_passed=verified,
                   candidates=len(records), elapsed_seconds=time.perf_counter()-started,
                   methods={m: dict(categories=dict(Counter(r['category'] for r in records if r['method']==m)),
                                    solver_success=sum(r['solver_success'] for r in records if r['method']==m))
                            for m in ['NTO_distance', 'NTO_time']})
    atomic(args.output / 'summary.json', summary)
    print(summary, flush=True)
    if not verified:
        raise RuntimeError('Input changed during audit')


if __name__ == '__main__':
    main()
