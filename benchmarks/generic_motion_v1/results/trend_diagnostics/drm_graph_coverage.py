"""Fully inspect a frozen DRM graph under the native sampled edge predicate."""
import argparse
import collections
import fcntl
import json
from pathlib import Path
import sys
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
from benchmarks.generic_motion_v1.common import BASE,cases,sha
from benchmarks.generic_motion_v1.roadmap import write,load_config
from benchmarks.generic_motion_v1.runners.native_drm_query import load


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    tasks=[c for c in cases(BASE/'manifests/dev.jsonl') if c['scene_id']==a.scene]
    assert tasks and all(c['split']=='dev' for c in tasks)
    import csdecomp as csd
    from benchmarks.generic_motion_v1.model import Model
    from benchmarks.generic_motion_v1.runners.csdecomp_baseline import _validate_frontend_path
    from drm_attachment_audit import components
    with (ROOT/'results/.gpu.lock').open('a+') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        began=time.perf_counter();model=Model(tasks[0]['voxels_xyz'],tasks[0]['voxel_radius_m'])
        module,receipt=load(BASE/'native/build/drm_consistent_query')
        opts=csd.DrmPlannerOptions();opts.online_edge_step_size=.01;opts.voxel_padding=.01
        planner=csd.DrmPlanner(model.csd_plant,opts,True)
        roadmap=BASE/'roadmaps/rss2025_n6000_s000/native.map';planner.LoadRoadmap(str(roadmap))
        nodes=planner.drm.id_to_node_map;ids=sorted(nodes)
        xyz=np.asarray(model.voxels_xyz,np.float32).T
        q=np.asarray([nodes[i] for i in ids],np.float32)
        free=model.cuda(q)
        padded=np.asarray(csd.CheckCollisionFreeVoxelsCuda(q.T,model.voxels,.05,model.mplant,model.robot_geometry_ids),bool)
        free_ids={i for i,v in zip(ids,free) if v};padded_ids={i for i,v in zip(ids,padded) if v}
        edges=sorted({(min(i,j),max(i,j)) for i,ns in planner.drm.node_adjacency_map.items() for j in ns})
        safe=np.zeros(len(edges),bool);checked=0;adj={i:[] for i in free_ids}
        for index,(i,j) in enumerate(edges):
            if i not in padded_ids or j not in padded_ids:continue
            checked+=1;safe[index]=bool(module.steer(planner,nodes[i],nodes[j],xyz,.05))
            if safe[index]:adj[i].append(j);adj[j].append(i)
            if checked%2000==0:print(json.dumps(dict(edges_checked=checked,elapsed=time.perf_counter()-began)),flush=True)
        labels=components(nodes,adj,set(ids)-free_ids,set())
        sizes=collections.Counter(labels.values())
        np.savez_compressed(a.output/'graph.npz',node_ids=ids,node_configurations=q,
            online_free=free,padded_free=padded,edges=edges,native_edge_safe=safe,
            components=np.asarray([labels.get(i,-1) for i in ids]))
        result=dict(scope='Diagnostic finite graph only; every retained edge checked by native inclusive 0.01-rad sampling with existing 0.01-m internal padding. This is not a continuous proof or a benchmark score.',
            scene_id=a.scene,roadmap_sha256=sha(roadmap),build_sha256=receipt['binary_sha256'],
            graph_nodes=len(ids),online_free_nodes=len(free_ids),padded_free_nodes=len(padded_ids),
            graph_edges=len(edges),edges_sampled=checked,safe_edges=int(safe.sum()),
            components=len(sizes),largest_components=sorted(sizes.values(),reverse=True)[:20],
            graph_check_seconds=time.perf_counter()-began,endpoint_radius_m=.04,internal_radius_m=.05,edge_step_rad=.01,tasks=[])
        cfg=load_config(ROOT/'configs/evaluation.json')
        for case in tasks:
            row=dict(problem_id=case['problem_id'])
            visible={}
            for key in ('q_start','q_goal'):
                visible[key]=[i for i in sorted(free_ids) if module.steer(planner,np.asarray(case[key]),nodes[i],xyz,.04)]
            common={labels[i] for i in visible['q_start']} & {labels[i] for i in visible['q_goal']}
            row.update(visible_start_nodes=len(visible['q_start']),visible_goal_nodes=len(visible['q_goal']),
                       common_components=len(common),native_sampled_graph_route_exists=bool(common))
            folder=a.output/case['problem_id'];folder.mkdir()
            np.savez_compressed(folder/'attachments.npz',start_ids=visible['q_start'],goal_ids=visible['q_goal'])
            if common:
                # Numerical capacity witness, never fed into the benchmark runner.
                label=min(common);start=next(i for i in visible['q_start'] if labels[i]==label)
                goal=next(i for i in visible['q_goal'] if labels[i]==label)
                parents={start:None};queue=collections.deque([start])
                while goal not in parents:
                    current=queue.popleft()
                    for nxt in adj[current]:
                        if nxt not in parents:parents[nxt]=current;queue.append(nxt)
                reverse=[];current=goal
                while current is not None:reverse.append(current);current=parents[current]
                path=np.asarray([case['q_start']]+[nodes[i] for i in reversed(reverse)]+[case['q_goal']])
                np.save(folder/'capacity_witness.npy',path)
                validation=_validate_frontend_path(path,model,case,cfg,folder/'validation')
                row['independent_witness_validation_passed']=validation['passed']
            result['tasks'].append(row)
            write(a.output/'summary.json',result)
        result['total_diagnostic_seconds']=time.perf_counter()-began
        write(a.output/'summary.json',result)
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
