"""CPU-only explicit SCIP probes of preserved development LGCS region pools.

This does not regenerate a front end or assert physical collision validity.
The original query, all region coefficients, and both source hashes are saved.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time
import traceback

import numpy as np
from pydrake.all import HPolyhedron

from benchmarks.generic_motion_v1.roadmap import write
from benchmarks.generic_motion_v1.runners import paper_lgcs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--round', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--time-limit', type=float, default=60.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    write(args.output/'request.json', dict(rounds=[str(p.resolve()) for p in args.round],
        time_limit=args.time_limit, backend='scip', scope='saved-dev-corridor CPU algebra only'))
    for i, round_path in enumerate(args.round):
        out = args.output/f'{i:03d}'
        out.mkdir()
        attempt = round_path.parent.parent
        inp = json.loads((attempt/'input.json').read_text())
        if inp.get('split') != 'dev':
            raise ValueError('Development solver probe only accepts the dev split')
        regions_file = round_path/'regions.npz'
        with np.load(regions_file, allow_pickle=False) as saved:
            indices = sorted(int(key[2:]) for key in saved.files if key.startswith('A_'))
            regions = [HPolyhedron(saved[f'A_{k:04d}'], saved[f'b_{k:04d}']) for k in indices]
        original = next((json.loads(p.read_text()) for p in [round_path/'lgcs_failure.json', round_path/'lgcs_solver.json'] if p.exists()), None)
        record = dict(round=str(round_path.resolve()), case=inp['problem_id'],
            source_sha256={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in [attempt/'input.json', regions_file]}, original=original)
        write(out/'provenance.json', record)
        shutil.copyfile(attempt/'input.json', out/'input.json')
        shutil.copyfile(regions_file, out/'regions.npz')
        started = time.perf_counter()
        try:
            answer = paper_lgcs.solve(regions, inp['q_start'], inp['q_goal'],
                                     time_limit=args.time_limit, out=out/'solver', backend='scip')
            record['candidate'] = {k:v for k,v in answer.items() if not isinstance(v,np.ndarray)}
            if answer.get('points') is not None:
                points = np.asarray(answer['points'])
                np.save(out/'points.npy', points)
                memberships = [float(np.max(regions[index].A() @ points[seg:seg+2].T
                                - regions[index].b()[:,None])) for seg,index in enumerate(answer['segment_regions'])]
                cost = float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
                record['independent_numpy_check'] = dict(
                    start_exact=bool(np.array_equal(points[0],inp['q_start'])),
                    goal_exact=bool(np.array_equal(points[-1],inp['q_goal'])),
                    segment_membership_max=max(memberships), recomputed_pwl_length=cost,
                    physical_collision_checked=False, continuous_collision_proof=False)
            for key in ['graph_vertices','graph_edges','solver_variables','solver_constraints',
                        'perspective_membership_rows']:
                if original is not None and original.get(key) != answer.get(key):
                    raise ValueError(f'Changed program dimension {key}: {original.get(key)} != {answer.get(key)}')
            if original is not None and original.get('solver_quadratic_constraints') != answer.get('solver_soc_constraints'):
                raise ValueError('Changed number of equivalent second-order cones')
            record['program_dimensions_match_original'] = original is not None
        except Exception as error:
            record.update(error=repr(error), exception_type=type(error).__name__)
            (out/'exception.txt').write_text(traceback.format_exc())
        record['observed_cpu_probe_seconds'] = time.perf_counter()-started
        write(out/'result.json',record)
        results.append(record)
        write(args.output/'summary.json',dict(complete=False, results=results))
        candidate=record.get('candidate',{})
        print(json.dumps(dict(case=record['case'], original_variables=(original or {}).get('solver_variables'),
            solver_success=candidate.get('solver_success'), status=candidate.get('solver_status'),
            cost=candidate.get('solver_objective'), gap=candidate.get('gap'),
            seconds=record['observed_cpu_probe_seconds'],error=record.get('error')),allow_nan=False),flush=True)
    write(args.output/'summary.json',dict(complete=True, results=results))


if __name__ == '__main__':
    main()
