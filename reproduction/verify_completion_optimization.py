"""Small, predetermined mathematical checks; not paper benchmark observations."""
from pathlib import Path
import contextlib
import hashlib
import io
import json
import os
import time
import traceback
import numpy as np
from pydrake.all import HPolyhedron, MosekSolver, GurobiSolver
from reproduction.common import ROOT, read, atomic
from reproduction.pipeline import save_candidate
from reproduction.paper_completion.optimization import lscs, lgcs, nto
from reproduction.paper_protocol.optimization import lgcs as legacy_lgcs

OUT = ROOT / 'reproduction/reports/paper_completion/optimization_checks'


def run():
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    source = ROOT / 'reproduction/paper_completion/optimization.py'
    atomic(OUT / 'identity.json', dict(command='bash local/python.sh -m reproduction.verify_completion_optimization',
        optimization_sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
    license_info = {
        'environment_variable_presence': {k: k in os.environ for k in (
            'MOSEKLM_LICENSE_FILE', 'GRB_LICENSE_FILE', 'GRB_WLSACCESSID', 'GRB_WLSSECRET', 'GRB_LICENSEID')},
        'drake': {s.solver_id().name(): {'available': s.available(), 'enabled': s.enabled()}
                  for s in (MosekSolver(), GurobiSolver())},
        'license_secrets_read': False,
    }
    atomic(OUT / 'solver_availability.json', license_info)
    old_run = ROOT / 'reproduction/runs/20261025T082752.022456Z_paper_fr3'
    selected = sorted(p for p in (old_run / 'cases').glob('*/LGCS') if list(p.glob('region_00_*.npz')))[:3]
    for directory in selected:
        region_paths = sorted(directory.glob('region_00_*.npz'))
        regions = [HPolyhedron(np.load(p)['A'], np.load(p)['b']) for p in region_paths]
        with np.load(directory / 'candidate_00.npz') as archive:
            start, goal = archive['points'][[0, -1]]
        case = directory.parent.name
        atomic(OUT / case / 'inputs.json', dict(
            selection='first three lexicographically ordered frozen FR3 cases with saved LGCS region_00 artifacts',
            frozen_run=str(old_run.relative_to(ROOT)), start=start, goal=goal,
            regions=[dict(path=str(p.relative_to(ROOT)), sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in region_paths]))
        for name, solver in [('legacy_big_m', legacy_lgcs), ('eq5_5_projection', lgcs)]:
            tic = time.perf_counter()
            output = OUT / case / name
            try:
                answer = solver(regions, start, goal)
                elapsed = time.perf_counter() - tic
                save_candidate(output, answer, regions)
                residual = None
                if answer['solver_success']:
                    residual = max(float(np.max(regions[i].A() @ answer['points'][j:j+2].T - regions[i].b()[:, None]))
                                   for j, i in enumerate(answer['segment_regions']))
                row = dict(case=case, planner=name, status='solved' if answer['solver_success'] else 'optimization_failed',
                    elapsed_seconds=elapsed, objective=answer['objective'], region_residual=residual,
                    solver_status=answer['solver_status'], gap=answer.get('gap'),
                    solver_variables=answer.get('solver_variables'), graph_vertices=len(regions))
            except Exception as error:
                row = dict(case=case, planner=name, status='exception', elapsed_seconds=time.perf_counter() - tic,
                           error=repr(error), optimization_metadata=getattr(error, 'optimization_metadata', None),
                           traceback=traceback.format_exc())
            atomic(output / 'result.json', row)
            rows.append(row)
    atomic(OUT / 'results.json', rows)
    print(json.dumps(rows, indent=2))


if __name__ == '__main__':
    run()
