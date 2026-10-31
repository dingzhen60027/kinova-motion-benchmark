"""Independent, CPU-only sealed LGCS audit for the frozen repair study.

No runtime, planner, collision checker or optimizer is imported. Run only after
all four blocks are complete. Statistics keep old/new runtimes separate and
pair the same fixed query/map/seed; a pair is not an isolated solver ablation.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import re
import traceback

import numpy as np

from benchmarks.generic_motion_v1.reports.full_baseline import independent_lgcs_audit as old

read, digest, arrays, require, close = old.read, old.digest, old.arrays, old.require, old.close
SUCCESS = 'success_validated_numerical'
TOLERANCE = 1e-7


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def scip_gap(primal, dual):
    """SCIP uses min(abs(primal), abs(dual)), unlike Gurobi's primal divisor."""
    if primal == dual:
        return 0.
    if not finite(primal) or not finite(dual) or primal*dual <= 0:
        return None
    return abs(primal-dual)/min(abs(primal), abs(dual))


def scip_log(path):
    text = Path(path).read_text(errors='replace')
    status = re.findall(r'^SCIP Status\s*:\s*(.*)$', text, re.M)
    if not status:
        return dict(complete_footer=False)
    source = status[-1]
    mapping = {'optimal solution found': 'optimal', 'gap limit reached': 'gaplimit',
               'time limit reached': 'timelimit', 'infeasible': 'infeasible',
               'unbounded': 'unbounded', 'infeasible or unbounded': 'inforunbd',
               'node limit reached': 'nodelimit', 'memory limit reached': 'memlimit',
               'user interrupt': 'userinterrupt'}
    bracket = source[source.rfind('[')+1:source.rfind(']')]
    require(bracket in mapping, 'Unrecognized actual SCIP footer: '+source)
    def number(label):
        matches = re.findall(r'^'+label+r'\s*:\s*([^\s]+)', text, re.M)
        require(bool(matches), 'Missing SCIP log field '+label)
        try:
            value=float(matches[-1])
            return value if math.isfinite(value) else None
        except ValueError:
            return None  # "infinite" is not an observed finite gap.
    return dict(complete_footer=True, status=mapping[bracket], status_text=source,
                primal=number('Primal Bound'), dual=number('Dual Bound'),
                rounded_gap_percent=number('Gap'), seconds=number(r'Solving Time \(sec\)'))


def cip_dimensions(path, n, d, edges):
    text = Path(path).read_text()
    variables = re.findall(r'^\s*\[(binary|continuous)\] <([^>]+)>:', text, re.M)
    constraints = re.findall(r'^\s*\[(linear|nonlinear)\] <([^>]+)>:', text, re.M)
    names = [name for _, name in variables]
    require(len(names) == len(set(names)), 'Duplicate CIP variable names')
    interior = [(u, v) for u, v in edges if u < n and v < n]
    expected = {f'phi_{u}_{v}' for u, v in edges}
    expected.update(f'z_{u}_{v}_{k}' for u, v in interior for k in range(d))
    expected.update(f'delta_{i}_{k}' for i in range(n) for k in range(d))
    expected.update(f'length_{i}' for i in range(n))
    require(set(names) == expected, 'CIP variables differ from declared GCS graph')
    require({name for kind, name in variables if kind == 'binary'} == {f'phi_{u}_{v}' for u, v in edges},
            'CIP does not give exactly binary edge flows')
    cones = {name for kind, name in constraints if kind == 'nonlinear'}
    require(cones == {f'soc_{i}' for i in range(n)}, 'CIP missing per-region norm cones')
    for line in text.splitlines():
        if '[nonlinear]' in line:
            require('^0.5' in line, 'CIP does not use the frozen unsquared norm expression')
    return dict(solver_variables=len(names), solver_constraints=sum(kind == 'linear' for kind, _ in constraints),
                solver_soc_constraints=len(cones), solver_nonlinear_constraints=len(cones),
                solver_quadratic_constraints=0, binary_flow_variables=len(edges))


def check_text_solution(path, expected, objective):
    text = Path(path).read_text()
    lines = text.splitlines()
    require(lines and lines[0].startswith('objective value:'), 'Missing textual SCIP objective')
    close(float(lines[0].split(':', 1)[1]), objective, 1e-10, 'solution.sol objective mismatch')
    values = {}
    for line in lines[1:]:
        fields = line.split()
        if len(fields) >= 2:
            require(fields[0] not in values, 'Duplicate solution.sol variable')
            values[fields[0]] = float(fields[1])
    require(set(values) == set(expected), 'solution.sol omitted or invented raw variables')
    for name, value in expected.items():
        close(values[name], value, 1e-10, 'solution.sol/raw mismatch '+name)


def raw_solution_check(directory, regions, endpoints, metadata):
    """Recompute original flow/norm algebra and selected path without a solver."""
    graph, raw = arrays(directory/'graph.npz'), arrays(directory/'solver_solution.npz')
    n, d = len(regions), len(endpoints[0])
    close(graph['start'], endpoints[0], 0., 'SCIP start changed')
    close(graph['goal'], endpoints[1], 0., 'SCIP goal changed')
    close(raw['edges'], graph['edges'], 0., 'Raw solution graph changed')
    close(raw['interior_edges'], graph['interior_edges'], 0., 'Raw interior graph changed')
    edges = [tuple(map(int, e)) for e in graph['edges']]
    require(len(edges) == len(set(edges)), 'Duplicate GCS graph edge')
    require(all(0 <= u <= n and 0 <= v <= n+1 and u != v and v != n and u != n+1 for u,v in edges),
            'Invalid GCS graph edge index/direction')
    interior = [(u,v) for u,v in edges if u < n and v < n]
    require(interior == [tuple(map(int,e)) for e in graph['interior_edges']], 'Interior edge subset/order wrong')
    require(all((v,u) in edges for u,v in interior), 'Region adjacency is not bidirectional')
    # This checks source/goal adjacency directly; all region-region intersections
    # are intentionally not re-solved with another optimizer in this audit.
    for i, (A,b) in enumerate(regions):
        require(((n,i) in edges) == bool(np.all(A@endpoints[0] <= b+1e-6)), 'Start membership graph mismatch')
        require(((i,n+1) in edges) == bool(np.all(A@endpoints[1] <= b+1e-6)), 'Goal membership graph mismatch')
    phi, zvalues, lengths, deltas = raw['flow'], raw['transition'], raw['length'], raw['delta']
    for value, shape in [(phi,(len(edges),)),(zvalues,(len(interior),d)),(lengths,(n,)),(deltas,(n,d))]:
        require(value.shape == shape and np.isfinite(value).all(), 'Malformed/nonfinite raw SCIP array')
    flow = dict(zip(edges,phi)); z = dict(zip(interior,zvalues))
    def point(e):
        return endpoints[0]*flow[e] if e[0] == n else endpoints[1]*flow[e] if e[1] == n+1 else z[e]
    maximum = lambda values: float(max([0., *values]))
    residuals = dict(finite=True, flow_integrality=float(np.max(np.abs(phi-np.round(phi)),initial=0)),
        flow_bounds=maximum([*-phi, *(phi-1)]), length_nonnegative=maximum(-lengths),
        soc_norm=maximum(np.linalg.norm(deltas,axis=1)-lengths))
    balances, degrees, vectors, membership, rows = [], [], [], [], 0
    for i in range(n+2):
        incoming, outgoing = [e for e in edges if e[1]==i], [e for e in edges if e[0]==i]
        outflow, inflow = sum(flow[e] for e in outgoing), sum(flow[e] for e in incoming)
        balances.append(abs(outflow-inflow-(1 if i == n else -1 if i == n+1 else 0)))
        if i<n:
            degrees.append(outflow-1)
            displacement = sum((point(e) for e in outgoing),np.zeros(d))-sum((point(e) for e in incoming),np.zeros(d))
            vectors.append(float(np.max(np.abs(displacement-deltas[i]))))
    for e in edges:
        unique={}
        for i in e:
            if i<n:
                for a,b in zip(*regions[i]):
                    key=tuple(float(x) for x in a)
                    unique[key]=min(unique.get(key,float('inf')),float(b))
        for a,b in unique.items():
            membership.append(float(np.dot(a,point(e))-b*flow[e])); rows+=1
    residuals.update(flow_balance=maximum(balances),degree=maximum(degrees),vector_flow=maximum(vectors),
                     homogeneous_membership=maximum(membership),tolerance=TOLERANCE)
    residuals['passed']=all(v<=TOLERANCE for k,v in residuals.items() if k not in ('finite','tolerance'))
    for key,value in residuals.items():
        saved=metadata['algebraic_validation'][key]
        if isinstance(value,bool):
            require(saved is value,'Saved algebraic gate disagrees: '+key)
        else:
            close(value,saved,1e-11,'Saved algebraic residual disagrees: '+key)
    require(rows == metadata['perspective_membership_rows'],'Homogeneous row count mismatch')
    linear_count=(n+2)+n+n*d+rows
    require(linear_count == metadata['solver_constraints'],'Linear program constraint count mismatch')
    dimensions=cip_dimensions(directory/'model.cip',n,d,edges)
    for key,value in dimensions.items():
        if key != 'binary_flow_variables':
            require(metadata[key] == value,'CIP/metadata dimension mismatch '+key)
    expected={f'phi_{u}_{v}':flow[u,v] for u,v in edges}
    expected.update({f'z_{u}_{v}_{k}':z[u,v][k] for u,v in interior for k in range(d)})
    expected.update({f'delta_{i}_{k}':deltas[i,k] for i in range(n) for k in range(d)})
    expected.update({f'length_{i}':lengths[i] for i in range(n)})
    close(float(lengths.sum()),metadata['solver_objective'],1e-9,'Raw lengths do not sum to solver objective')
    check_text_solution(directory/'solution.sol',expected,metadata['solver_objective'])
    answer=dict(algebra=residuals,dimensions=dimensions,decoded=False)
    if not residuals['passed'] or not metadata['scip_original_solution_feasible']:
        require(metadata['solver_success'] is False,'Rejected original algebra counted successful')
        return answer
    selected={e for e,v in flow.items() if v>.5}
    u,sequence,path,used=n,[],[endpoints[0]],[]
    while u != n+1:
        successors=[v for a,v in selected if a==u]
        require(len(successors)==1,'Invalid selected integer flow')
        v=successors[0];used.append((u,v))
        if u<n:
            path.append(endpoints[1] if v==n+1 else z[u,v])
        u=v
        if u<n:
            require(u not in sequence,'Selected source-to-goal cycle')
            sequence.append(u)
    path=np.asarray(path)
    require(sequence == metadata['segment_regions'] == metadata['selected_regions'],'Raw flow and region sequence differ')
    discarded=sorted(selected-set(used))
    require([list(e) for e in discarded] == metadata['discarded_disjoint_cycle_edges'],'Discarded cycle record differs')
    residual=max(old.residual(regions[index],path[i:i+2]) for i,index in enumerate(sequence))
    residual=max(0.,residual)
    cost=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum())
    excess=max(0.,cost-metadata['solver_objective'])
    close(residual,metadata['decoded_membership_violation'],1e-11,'Decoded membership mismatch')
    close(excess,metadata['decoded_objective_excess'],1e-11,'Decoded objective excess mismatch')
    close(cost,metadata['recovered_path_length'],1e-10,'Decoded length mismatch')
    require(metadata['solver_success'] == bool(residual<=TOLERANCE and excess<=TOLERANCE),'Decoded acceptance gate differs')
    candidate_file=directory/'candidate.npz'
    if candidate_file.exists():
        close(arrays(candidate_file)['points'],path,1e-12,'Saved candidate differs from decoded raw flow')
    answer.update(decoded=True,regions=sequence,points=path.tolist(),cost=cost,membership=residual)
    return answer


def scip_round(directory, regions, endpoints):
    metadata=read(directory/'lgcs_solver.json')
    require(metadata['solver']=='SCIP' and metadata['backend']=='scip','Frozen SCIP replaced by another backend')
    require(metadata['pyscipopt_version']=='6.3.0' and metadata['solver_version']==[10,0,2],'Solver version differs from frozen dependency')
    graph=arrays(directory/'graph.npz')
    close(graph['start'],endpoints[0],0.,'Graph start changed')
    close(graph['goal'],endpoints[1],0.,'Graph goal changed')
    edges=[tuple(map(int,e)) for e in graph['edges']]
    require(metadata['graph_vertices']==len(regions) and metadata['graph_edges']==len(edges),'Graph size metadata wrong')
    dimensions=cip_dimensions(directory/'model.cip',len(regions),len(endpoints[0]),edges)
    for key,value in dimensions.items():
        if key!='binary_flow_variables':require(metadata[key]==value,'CIP/metadata mismatch: '+key)
    log=scip_log(directory/'scip.log')
    require(log['complete_footer'],'Completed solver metadata lacks a complete SCIP footer')
    require(log['status']==metadata['solver_status'],'SCIP log status differs from JSON')
    for logkey,metakey in [('primal','solver_primal_bound'),('dual','solver_dual_bound')]:
        if log[logkey] is not None and finite(metadata[metakey]):
            close(log[logkey],metadata[metakey],1e-9*max(1.,abs(metadata[metakey])), 'SCIP log/JSON bound differs')
    status=metadata['solver_status']; has=metadata['solver_has_incumbent']
    require(metadata['globally_optimal'] == bool(status=='optimal' and metadata['solver_success']),
            'gaplimit/timelimit mislabeled global optimal or optimal flag missing')
    record=dict(solver_status=status,solver_success=metadata['solver_success'],has_incumbent=has,
                primal=metadata['solver_primal_bound'],dual=metadata['solver_dual_bound'],gap=metadata['gap'],
                globally_optimal=metadata['globally_optimal'],log=log)
    if has:
        primal,dual=metadata['solver_primal_bound'],metadata['solver_dual_bound']
        require(finite(primal) and finite(dual),'Incumbent has nonfinite saved bounds')
        require(dual<=primal+1e-8,'SCIP lower bound exceeds incumbent objective')
        close(metadata['solver_objective'],primal,1e-10,'SCIP best incumbent differs from primal bound')
        gap=scip_gap(primal,dual)
        if gap is not None:
            close(gap,metadata['gap'],1e-10,'SCIP fractional gap formula mismatch')
            if log['rounded_gap_percent'] is not None:
                require(abs(log['rounded_gap_percent']-100*gap)<=.0050001,'Rounded log percent and fractional gap differ')
        if status=='gaplimit':
            require(metadata['gap']<=metadata['solver_parameters']['limits/gap']+1e-12,'Gaplimit exceeds requested fractional limit')
        record['raw']=raw_solution_check(directory,regions,endpoints,metadata)
    else:
        require(metadata['solver_success'] is False and metadata['solver_objective'] is None,
                'No-incumbent solve reported accepted solution')
        if (directory/'candidate.npz').exists():
            require('points' not in arrays(directory/'candidate.npz'),'No-incumbent solve saved trajectory points')
    return record


def receipt_check(attempt,job,manifest,cache):
    receipt,result=read(attempt/'receipt.json'),read(attempt/'result.json')
    files=receipt['files']
    require(isinstance(files,dict) and 'result.json' in files,'Missing result in sealed receipt')
    for name,expected in files.items():
        path=(attempt/name).resolve()
        require(path.is_relative_to(attempt.resolve()) and name!='receipt.json','Unsafe receipt path')
        if path not in cache:
            cache[path]=digest(path)
        require(cache[path]==expected,'Receipt SHA mismatch: '+str(path))
    actual={str(p.relative_to(attempt)) for p in attempt.rglob('*') if p.is_file()
            and p.name!='receipt.json' and 'replay' not in p.relative_to(attempt).parts}
    require(actual==set(files),'Sealed receipt file inventory incomplete')
    require(receipt['status']==result['status'],'Receipt/result status differs')
    if receipt.get('retry_pending'):
        require(result['status']=='infrastructure_error','Interrupted attempt not preserved as infrastructure failure')
        return receipt,result
    require(read(attempt/'request.json')==job,'Request differs from manifest job')
    for field,expected in {'problem_id':job['case_id'],'scene_id':job['scene_id'],'family':job['family'],
                           'objective':job['objective'],'method':job['method'],'method_seed':job['method_seed'],
                           'roadmap_id':job['roadmap_id']}.items():
        require(result.get(field)==expected,'Result identity differs: '+field)
    runtime=read(attempt/'runtime_sources.json')
    require(runtime['before']==runtime['after']==manifest['source_hashes'] and runtime['unchanged'] is True,
            'Attempt mixed or changed runtime sources')
    process=read(attempt/'process.json')
    require(finite(process['cold_e2e_seconds']) and process['cold_e2e_seconds']>=0,'Invalid observed process time')
    close(result['cold_e2e_seconds'],process['cold_e2e_seconds'],0.,'Result/process cold time differs')
    if result['status']==SUCCESS:
        require(process['exit_code']==0 and process['timed_out'] is False,'Success conflicts with process termination')
    return receipt,result


def audit_attempt(attempt,job,manifest,latest,cache,backend):
    receipt,result=receipt_check(attempt,job,manifest,cache)
    if receipt.get('retry_pending'):
        return dict(latest=latest,attempt=str(attempt),retry_pending=True,status=result['status'])
    case=read(attempt/'input.json');cfg=read(attempt/'evaluation.json')
    require(all(case.get(k)==v for k,v in job['case'].items()),'Fixed case changed')
    require(case['_objective']=='minimum_distance' and cfg==job['evaluation_config'],'Objective/config changed')
    process=read(attempt/'process.json')
    map_path=Path(job['roadmap_dir'])/'native.map'
    map_sha=manifest['roadmap_files'][str(map_path)]
    record=dict(attempt=str(attempt),latest=latest,status=result['status'],backend=backend,
        source_commit=manifest['source_code_commit'],case_id=job['case_id'],case_sha=case['problem_sha256'],
        family=job['family'],split=case['split'],roadmap_id=job['roadmap_id'],map_sha256=map_sha,
        method_seed=job['method_seed'],solver_seed=job['solver_seed'],legal=case.get('q_start') is not None,
        cold_seconds=process['cold_e2e_seconds'],raw_timing=result.get('timing',{}),
        error_category=result.get('error_category'),rounds=[],recoveries=[],alternate=None,
        successful=result['status']==SUCCESS,cost=None)
    if not record['legal']:
        require(result['status']=='invalid_endpoint','Invalid fixed input used for planning')
        return record
    roadmap=read(Path(job['roadmap_dir'])/'metadata.json')
    for key in ('robot_sha256','collision_geometry_sha256'):
        require(roadmap[key]==case[key],'Roadmap/case canonical model differs: '+key)
    endpoints=np.asarray([case['q_start'],case['q_goal']],float)
    alternate,alternate_record=old.check_alternate(attempt,job,endpoints,cache)
    record['alternate']=alternate_record
    raw_path=np.load(attempt/'path.npy',allow_pickle=False) if (attempt/'path.npy').exists() else None
    rounds=sorted((attempt/'rounds').glob('[0-9][0-9][0-9]'))
    for directory in rounds:
        # A process can stop between pool save files. Preserve the partial
        # attempt as failure, never treat its stale root validation as success.
        if not all((directory/name).exists() for name in ['regions.npz','pool_paths.npz']):
            require(result['status']!=SUCCESS,'Success has incomplete final region pool')
            record['rounds'].append(dict(round=directory.name,partial_pool=True));continue
        regions,seeds,paths=old.load_pool(directory)
        item=dict(round=directory.name,pool_regions=len(regions),
                  cover=old.pool_cover(regions,seeds,paths,endpoints,raw_path,alternate,cfg['endpoint_tolerance_rad']))
        if (directory/'lgcs_solver.json').exists():
            if backend=='scip':
                expected=manifest['source_hashes']['benchmarks/generic_motion_v1/runners/paper_lgcs_scip.py']
                require(read(directory/'lgcs_solver.json')['source_sha256']==expected,'Optimizer source differs from run snapshot')
                item['solver']=scip_round(directory,regions,endpoints)
            else:
                m=read(directory/'lgcs_solver.json')
                require(m['solver']=='Gurobi','Old LGCS backend changed')
                dimensions=old.mps_dimensions(directory/'model.mps.gz')
                require(all(m[k]==v for k,v in dimensions.items() if k!='binary_flow_variables'),'Old Gurobi MPS dimensions changed')
                item['solver']=dict(solver_status=m['solver_status'],solver_success=m['solver_success'],gap=m.get('gap'))
        elif (directory/'lgcs_failure.json').exists():
            m=read(directory/'lgcs_failure.json')
            require(m['solver_success'] is False,'Failure metadata claims success')
            item['failure']=m
            if backend=='scip':
                require(m.get('solver')=='SCIP','SCIP failure silently substituted another solver')
            elif m.get('solver_error_number')==10010:
                require(m['status']=='unsupported','Gurobi license failure labeled as algorithm failure')
                dimensions=old.mps_dimensions(directory/'model.mps.gz')
                require(all(m[k]==v for k,v in dimensions.items() if k!='binary_flow_variables'),'Failed Gurobi model dimensions changed')
        else:
            require(result['status']!=SUCCESS or directory!=rounds[-1],'Success missing solver receipt')
            item['solver_interrupted_before_metadata']=True
            if (directory/'scip.log').exists():
                item['partial_solver_log']=scip_log(directory/'scip.log')
        if (directory/'candidate.json').exists():
            item['candidate']=old.candidate_audit(directory,regions,endpoints,cfg)
            m=read(directory/'candidate.json')
            if (directory/'lgcs_solver.json').exists():
                solver=read(directory/'lgcs_solver.json')
                require(all(m.get(k)==v for k,v in solver.items()),'Candidate metadata differs from saved optimizer')
        record['rounds'].append(item)
    for recovery in sorted((attempt/'recovery').glob('[0-9][0-9][0-9]')):
        if (recovery/'native_edit_inputs.npz').exists():
            record['recoveries'].append(old.audit_recovery(recovery,attempt/'rounds'/recovery.name,cfg))
    if rounds and (attempt/'validation.json').exists():
        if (rounds[-1]/'validation.json').exists():
            require(read(attempt/'validation.json')==read(rounds[-1]/'validation.json'),'Root/latest validation differ')
            record['root_validation_scope']='final_round'
        else:
            record['root_validation_scope']='prior_round_only'
            require(not record['successful'],'Stale root validation counted as success')
    if record['successful']:
        require(record['rounds'] and record['rounds'][-1].get('candidate',{}).get('passed') is True,
                'Success missing passing final candidate validation')
        require(record['rounds'][-1]['candidate']['solver_success'] is True,'Success failed solver gate')
        require(result['frontend_native_success'] is True and result['frontend_success'] is True,'Success failed frontend gate')
        front=read(attempt/'frontend_validation/validation.json')
        require(front['passed'] is True and front['status']==SUCCESS,'Success lacks validated seed')
        require(result['frontend_validation_status']==front['status'],'Frontend status changed')
        final=read(rounds[-1]/'validation.json')
        require(final['cost_unit']=='rad','Wrong cost unit')
        record['cost']=final['cost']
        close(arrays(attempt/'candidate.npz')['points'],arrays(rounds[-1]/'candidate.npz')['points'],0.,'Root/final candidate differs')
    return record


def run_preflight(run,expected_sha=None,expected_sources=None):
    manifest=read(run/'run_manifest.json')
    if expected_sha is not None:
        require(digest(run/'run_manifest.json')==expected_sha,'Run manifest differs from frozen index')
    require(read(run/'identity.json')['run_manifest_sha256']==digest(run/'run_manifest.json'),'Run identity changed')
    if expected_sources is not None:
        require(manifest['source_hashes']==expected_sources,'Run uses unexpected source inventory')
    require(len({j['job_id'] for j in manifest['jobs']})==len(manifest['jobs']),'Duplicate jobs')
    for job in manifest['jobs']:
        require(Path(job['job_id']).name==job['job_id'] and job['job_id'] not in ('.','..'),'Unsafe job path')
        attempts=sorted((run/'jobs'/job['job_id']/'attempts').glob('[0-9][0-9][0-9][0-9]'))
        require(attempts and all((a/'receipt.json').exists() for a in attempts),'Study has incomplete/unsealed jobs')
        require(not read(attempts[-1]/'receipt.json').get('retry_pending'),'Study has pending retry')
    return manifest


def audit_run(run,manifest,backend,cache):
    errors,records=[],[]
    for name,sha in manifest['source_hashes'].items():
        require(digest(run/'source_snapshot'/name)==sha,'Frozen source snapshot changed: '+name)
    require(digest(manifest['manifest_path'])==manifest['manifest_sha256'],'Input manifest changed')
    for name,sha in manifest['roadmap_files'].items():
        path=Path(name)
        if path not in cache:cache[path]=digest(path)
        require(cache[path]==sha,'Frozen roadmap changed: '+name)
    for job in manifest['jobs']:
        if job['method']!='LGCS':continue
        require(job['objective']=='minimum_distance','LGCS objective changed')
        actual=job['evaluation_config'].get('lgcs',{}).get('solver','gurobi')
        require(actual==backend,'Run mixed solver backends')
        attempts=sorted((run/'jobs'/job['job_id']/'attempts').glob('[0-9][0-9][0-9][0-9]'))
        for attempt in attempts:
            try:
                records.append(audit_attempt(attempt,job,manifest,attempt==attempts[-1],cache,backend))
            except Exception as error:
                errors.append(dict(run=str(run),job_id=job['job_id'],attempt=str(attempt),error=repr(error),traceback=traceback.format_exc()))
    return records,errors


def paired_comparison(new,previous):
    def key(row):return (row['case_id'],row['map_sha256'],row['method_seed'],row['solver_seed'])
    def index(rows):
        chosen=[row for row in rows if row.get('latest') and row.get('split')=='dev' and not row.get('retry_pending')]
        result={key(row):row for row in chosen}
        require(len(result)==len(chosen),'Duplicate same-runtime LGCS case/map/seed')
        require(len({row['source_commit'] for row in chosen})==1,'Do not pool different runtimes')
        require(len({row['backend'] for row in chosen})==1,'Do not pool different backends')
        return result
    a,b=index(new),index(previous)
    require(set(a)==set(b),'Old/new LGCS dev case/map/seed sets differ')
    pairs=[]
    for key_ in sorted(a):
        x,y=a[key_],b[key_]
        require(x['case_sha']==y['case_sha'] and x['legal']==y['legal'],'Paired fixed query/legality differs')
        common=x['successful'] and y['successful']
        pairs.append(dict(case_id=x['case_id'],family=x['family'],map_sha256=x['map_sha256'],
            method_seed=x['method_seed'],solver_seed=x['solver_seed'],legal=x['legal'],
            new_runtime=x['source_commit'],old_runtime=y['source_commit'],new_backend=x['backend'],old_backend=y['backend'],
            new_status=x['status'],old_status=y['status'],new_success=x['successful'],old_success=y['successful'],
            new_cost=x['cost'],old_cost=y['cost'],cost_difference_new_minus_old=x['cost']-y['cost'] if common else None,
            new_cold_seconds=x['cold_seconds'],old_cold_seconds=y['cold_seconds'],
            cold_difference_new_minus_old=x['cold_seconds']-y['cold_seconds']))
    return pairs


def statistics(values):
    x=np.asarray([v for v in values if finite(v)],float)
    if not len(x):return dict(n=0)
    return dict(n=len(x),mean=float(x.mean()),median=float(np.median(x)),p90=float(np.quantile(x,.9)),p95=float(np.quantile(x,.95)))


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--index',type=Path,required=True);p.add_argument('--freeze',type=Path,required=True)
    p.add_argument('--old-dev',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True,help='New output prefix, e.g. reports/baseline_repair_v2/lgcs_audit')
    p.add_argument('--planning-finished',action='store_true',required=True)
    args=p.parse_args(argv)
    output_files={suffix:Path(str(args.output)+suffix) for suffix in ('.json','.md','_paired_dev.csv')}
    require(not any(path.exists() for path in output_files.values()),'Refusing to overwrite prior audit')
    index,freeze=read(args.index),read(args.freeze)
    require(index['slots']==freeze['planned_registered_slots']==950 and len(index['blocks'])==4,'Wrong development study inventory')
    require(index['test_planner_calls']==freeze['test_planner_calls']==0,'Unexpected test-phase work in development audit')
    manifests=[]
    for block in index['blocks']:
        run=Path(block['run']).resolve()
        require(not args.output.resolve().is_relative_to(run),'Audit output must not be inside a sealed run')
        m=run_preflight(run,block['run_manifest_sha256'],freeze['runtime_source_hashes'])
        require(m['source_code_commit']==freeze['source_commit'],'New study runtime differs from freeze')
        require(len(m['jobs'])==block['planned_jobs'],'Block count differs from index')
        manifests.append((run,m))
    old_run=args.old_dev.resolve();old_manifest=run_preflight(old_run)
    require(not args.output.resolve().is_relative_to(old_run),'Output inside historical run')
    require(old_manifest['source_code_commit']!=freeze['source_commit'],'Old/new runtime unexpectedly identical')
    # Full hashes happen only after every block is sealed; do not call while
    # the GPU study is running because hashing can perturb its observed time.
    args.output.parent.mkdir(parents=True,exist_ok=True)
    cache,errors,records,previous,pairs={},[],[],[],[]
    try:
        for group in ('data_files','roadmap_files','external_dependency_files'):
            for name,sha in freeze[group].items():
                path=Path(name)
                if path not in cache:cache[path]=digest(path)
                require(cache[path]==sha,'Execution freeze dependency changed: '+name)
        for run,m in manifests:
            rec,err=audit_run(run,m,'scip',cache);records.extend(rec);errors.extend(err)
        previous,old_errors=audit_run(old_run,old_manifest,'gurobi',cache);errors.extend(old_errors)
        pairs=paired_comparison(records,previous) if not errors else []
    except Exception as error:
        errors.append(dict(kind='audit_or_pairing',error=repr(error),traceback=traceback.format_exc()))
        pairs=[]
    legal=[r for r in pairs if r['legal']]
    transitions=Counter(f"{r['old_status']} -> {r['new_status']}" for r in legal)
    latest_new=[row for row in records if row.get('latest') and not row.get('retry_pending')]
    new_counts={split:dict(jobs=sum(row['split']==split for row in latest_new),
        legal_jobs=sum(row['split']==split and row['legal'] for row in latest_new),
        statuses=dict(Counter(row['status'] for row in latest_new if row['split']==split)),
        solver_statuses=dict(Counter(round_['solver']['solver_status'] for row in latest_new if row['split']==split
                             for round_ in row['rounds'] if 'solver' in round_))) for split in ('smoke','dev')}
    expected_lgcs=sum(j['method']=='LGCS' for _,m in manifests for j in m['jobs'])
    if len(latest_new)!=expected_lgcs:
        errors.append(dict(kind='missing_latest_lgcs_audit',expected=expected_lgcs,observed=len(latest_new)))
    result=dict(passed=not errors,complete=True,index=str(args.index.resolve()),index_sha256=digest(args.index),
        freeze_sha256=digest(args.freeze),new_source_commit=freeze['source_commit'],
        old_source_commit=old_manifest['source_code_commit'],new_attempts=records,old_attempts=previous,
        expected_lgcs_jobs=expected_lgcs,new_counts=new_counts,
        paired_dev= dict(n=len(pairs),legal_n=len(legal),transitions=dict(transitions),
            old_success=sum(r['old_success'] for r in legal),new_success=sum(r['new_success'] for r in legal),
            gained=sum(r['new_success'] and not r['old_success'] for r in legal),
            lost=sum(r['old_success'] and not r['new_success'] for r in legal),
            cost_difference_common_success=statistics(r['cost_difference_new_minus_old'] for r in legal),
            cold_difference_all_legal=statistics(r['cold_difference_new_minus_old'] for r in legal),
            old_cold_all_legal=statistics(r['old_cold_seconds'] for r in legal),
            new_cold_all_legal=statistics(r['new_cold_seconds'] for r in legal)),
        audit_source_sha256=digest(__file__),helper_source_sha256=digest(old.__file__),errors=errors,
        limitations=['No GPU, solver, planner or collision checker was invoked.',
            'Saved collision labels and clearances are checked for consistency, not recomputed; no mathematical continuous collision proof.',
            'All source/goal memberships and selected edges are independently checked; completeness of region-region intersection graph is not re-solved.',
            'SCIP optimal/gap are numerical solver claims for the supplied local region graph, not full configuration-space global optimality.',
            'SCIP fractional gap uses min absolute primal/dual denominator; textual log gap is rounded percent.',
            'Old/new runtimes and backends are kept separate. Paired pipeline changes cannot identify a solver-only causal effect.',
            'Costs use only common validated successes; observed process latency includes all legal failure outcomes. No timeout censoring or offline construction time is hidden.',
            'Three map seeds repeat each task; differences are descriptive, no IID or confidence-interval claim.'])
    output_files['.json'].write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    if pairs:
        with output_files['_paired_dev.csv'].open('w',newline='') as h:
            writer=csv.DictWriter(h,fieldnames=list(pairs[0]));writer.writeheader();writer.writerows(pairs)
    lines=['# Independent LGCS repair audit','',f"Passed: **{result['passed']}**.",
        f"New runtime `{result['new_source_commit']}`; old runtime `{result['old_source_commit']}`.",'',
        '## Separate new-runtime outcomes','', '```json',json.dumps(new_counts,indent=2),'```','',
        '## Paired development outcomes','', '```json',json.dumps(result['paired_dev'],indent=2),'```','',
        '## Scope','']+['- '+x for x in result['limitations']]
    if errors:lines+=['','## Failures','','```json',json.dumps(errors,indent=2),'```']
    output_files['.md'].write_text('\n'.join(lines)+'\n')
    print(json.dumps(dict(passed=result['passed'],paired_dev=result['paired_dev'],errors=errors),indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
