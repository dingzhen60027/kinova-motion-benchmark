"""Small fixed historical regressions; never modifies or pools old measurements."""
from collections import Counter, defaultdict
from pathlib import Path
import traceback
import numpy as np
import csdecomp as csd
from reproduction.common import ROOT, read, atomic
from reproduction.paper_completion import model as model_adapter
model_adapter.ASSETS = ROOT / "reproduction/paper_protocol/assets"
from reproduction.paper_completion.model import FR3
from reproduction.paper_completion.data import attach_alternate, endpoint_audit, online_pose_ik, planner
from reproduction.paper_completion.pipeline import Pipeline

OLD=ROOT/'reproduction/runs/20261025T082752.022456Z_paper_fr3'
OUT=ROOT/'reproduction/reports/paper_completion/diagnostics/pipeline/final_regressions'

def run():
    cfg=read(OLD/'manifest.json')['config']
    summary={'historical_run':str(OLD),'regressions':[]}
    counts=Counter(); categories=Counter(); environments=defaultdict(Counter)
    for p in sorted((OLD/'cases').glob('*/case.json')):
        r=read(p); counts[r['status']]+=1
        if not r.get('roadmap',{}).get('success'):
            log=(p.parent/'stdout.log').read_text()
            category=('graph_disconnected' if 'Path does not exist.' in log else
                      'target_collision' if 'Target configuration is in collision' in log else
                      'start_collision' if 'starting in collision' in log else
                      'steering_or_lazy_budget_exhausted')
            categories[category]+=1; environments[p.parent.name[:3]][category]+=1
    summary['historical_status_counts']=dict(counts)
    summary['historical_native_failure_categories']=dict(categories)
    summary['historical_native_failures_by_environment']={k:dict(v) for k,v in environments.items()}
    atomic(OUT/'historical_failure_classification.json',summary)
    for case,method in [('e00_r09_n12000','LGCS'),('e09_r07_n06000','SCSTO')]:
        source=OLD/'cases'/case; data=read(source/'input.json')
        env=FR3(cfg,data['obstacles']); env.start=np.asarray(data['start']); env.goal=np.asarray(data['goal'])
        env.cfg.update(start=env.start,goal=env.goal)
        path=np.load(source/'seed_path.npz')['path']
        destination=OUT/case/method; destination.mkdir(parents=True,exist_ok=True)
        map_path=OLD/'roadmaps'/case[4:]/'native.map'
        drm=csd.DRM(); drm.Read(str(map_path))
        attach_alternate(env,map_path,destination)
        result={'case':case,'method':method,'original_result':read(source/method/'result.json')}
        try:
            answer=Pipeline(env,1337).solve(method,path,drm,destination)
            candidate=answer.pop('candidate'); regions=answer.pop('regions'); validation=answer.pop('validation',None)
            if validation is not None:
                samples=validation.pop('_samples'); np.savez_compressed(destination/'replay.npz',**samples)
            result.update(answer,validation=validation,candidate_solver_success=candidate['solver_success'],regions=len(regions))
        except Exception as exc:
            result.update(status='error',exception=repr(exc),traceback=traceback.format_exc())
        atomic(destination/'regression.json',result)
        summary['regressions'].append(result)
    scene=read(OLD/'scenes/e08/scene.json'); env=FR3(cfg,scene['obstacles'])
    summary['old_boundary_goal_audit']=endpoint_audit(env,scene['goal'])
    atomic(OUT/'regression_summary.json',summary)
    print({k:v for k,v in summary.items() if k!='regressions'})
    print([(x['case'],x['method'],x['status'],x.get('validation',{}).get('passed')) for x in summary['regressions']])

if __name__=='__main__': run()
