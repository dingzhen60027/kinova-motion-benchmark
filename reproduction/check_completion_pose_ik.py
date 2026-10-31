"""One preregistered historical graph failure, retaining its original pose task."""
from pathlib import Path
import numpy as np
import networkx as nx
from reproduction.common import ROOT,read,atomic
from reproduction.paper_completion import model as model_adapter
model_adapter.ASSETS = ROOT / "reproduction/paper_protocol/assets"
from reproduction.paper_completion.model import FR3
from reproduction.paper_completion.data import planner,online_pose_ik,endpoint_audit
from reproduction.paper_completion.validation import polygon_samples
OLD=ROOT/'reproduction/runs/20261025T082752.022456Z_paper_fr3'
OUT=ROOT/'reproduction/reports/paper_completion/diagnostics/pipeline/pose_ik_e01_r01_n03000'
OUT.mkdir(parents=True,exist_ok=True)
cfg=read(OLD/'manifest.json')['config']; cfg['endpoint_collision_margin']=1e-5
scene=read(OLD/'scenes/e01/scene.json'); f=FR3(cfg,scene['obstacles'])
f.start=np.asarray(scene['start'],np.float32).astype(float)
p=planner(f);p.LoadRoadmap(str(OLD/'roadmaps/r01_n03000/native.map'));p.BuildCollisionSet(f.voxels)
drm=p.drm;blocked=set(p.collision_set);graph=nx.Graph();nodes=drm.id_to_node_map
for i in nodes:
    if i not in blocked:graph.add_node(i)
for i,js in drm.node_adjacency_map.items():
    for j in js:
        if i in graph and j in graph:graph.add_edge(i,j)
components=sorted(nx.connected_components(graph),key=len,reverse=True)
lookup={i:k for k,c in enumerate(components) for i in c}
def nearest(q):return min(graph,key=lambda i:float(np.linalg.norm(np.asarray(nodes[i])-q)))
s=nearest(f.start);g=nearest(scene['goal'])
result=dict(case='e01_r01_n03000',old_status='graph_disconnected',component_sizes=[len(c) for c in components],
            old_start_nearest_node=s,old_goal_nearest_node=g,
            old_start_component=lookup[s],old_goal_component=lookup[g],
            old_goal=scene['goal'],target_pose=scene['target_pose'])
ik=online_pose_ik(f,drm,scene['target_pose'],OUT,collision_set=p.collision_set)
result['ik']=ik
if ik['status']=='success':
    f.goal=np.asarray(ik['goal']);f.cfg.update(start=f.start,goal=f.goal)
    g=nearest(f.goal);result['new_goal_nearest_node']=g;result['new_goal_component']=lookup[g]
    try:
        ok,path=p.Plan(f.start,f.goal,f.voxels,f.voxel_radius)
        result['native_success']=bool(ok)
        if ok:
            path=np.asarray(path);np.savez_compressed(OUT/'seed_path.npz',path=path)
            dense=polygon_samples(path,cfg['validation_step'])
            cuda=f.cuda(dense);ind=f.independent(dense)
            result['dense_validation']=dict(samples=len(dense),cuda_collisions=int((~cuda).sum()),independent_collisions=int((~ind).sum()),passed=bool(cuda.all() and ind.all()))
    except RuntimeError as e:result.update(native_success=False,native_exception=repr(e))
atomic(OUT/'result.json',result)
print({k:v for k,v in result.items() if k not in ['ik','target_pose','old_goal']})
print('IK',ik['status'],'attempts',len(ik['attempts']))
