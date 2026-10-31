"""Isolate official double geometry queries; Drake may terminate the process."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import HERE, ROOT, write


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--api',choices=['pair','all','penetration'])
    args=parser.parse_args()
    out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=True)
    if args.api:
        old=HERE/'results/rss2025_full_dev_v1/jobs/dev-route_choice-002-p00-NTO_PAPER-minimum_distance-48b608ce/attempts/0001'
        case=json.loads((old/'input.json').read_text())
        q=np.load(Path(__file__).with_name('nto_backend_last_q_probe')/'last_evaluated_q.npy')
        model=Model(case['voxels_xyz'],case['voxel_radius_m'])
        native=model.nto_plant()
        native.plant.SetPositions(native.plant_context,q)
        query=native.scene_graph.get_query_output_port().Eval(native.scene_context)
        names={'robotiq::left_inner_finger_collision','table::Box'}
        pair=next(p for p in native.inspector.GetCollisionCandidates() if {native.inspector.GetName(g) for g in p}==names)
        write(out/f'{args.api}_request.json',dict(api=args.api,q=q,pair=sorted(names),AutoDiff=False))
        if args.api=='pair':
            value=query.ComputeSignedDistancePairClosestPoints(*pair)
            result=dict(distance=value.distance)
        elif args.api=='all':
            values=query.ComputeSignedDistancePairwiseClosestPoints()
            result=dict(count=len(values),pair_values=[v.distance for v in values if {v.id_A,v.id_B}==set(pair)])
        else:
            values=query.ComputePointPairPenetration()
            result=dict(count=len(values),pair_depths=[v.depth for v in values if {v.id_A,v.id_B}==set(pair)])
        write(out/f'{args.api}_result.json',result)
        return
    result=[]
    for api in ('pair','all','penetration'):
        command=[sys.executable,'-m','benchmarks.generic_motion_v1.reports.baseline_study.nto_boxbox_api_probe','--output',str(out),'--api',api]
        with (out/f'{api}.log').open('w') as stream:
            proc=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,timeout=15)
        log=(out/f'{api}.log').read_text()
        row=dict(api=api,command=command,returncode=proc.returncode,
                 returned=(out/f'{api}_result.json').exists(),boxbox_assertion='BoxBoxGradient' in log and 'failed' in log)
        if row['returned']:row['result']=json.loads((out/f'{api}_result.json').read_text())
        result.append(row)
    write(out/'comparison.json',dict(results=result,AutoDiff=False,model_changed=False,pairs_changed=False))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
