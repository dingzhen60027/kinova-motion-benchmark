"""Construct fixed anchors from a declared finite geometric scan, before planning.

No roadmap, inverse kinematics, path search, inflation or trajectory solver is used.
The first passing mirrored pair in the frozen product ordering is selected. All
candidate CUDA predicates are retained, and independent distance queries are
performed in order until the first pair passes every fixed scene.
"""
import argparse
import hashlib
import itertools
from pathlib import Path
import time
import traceback
import numpy as np
from reproduction.common import ROOT, atomic, read
from .model import IIWA, drake_model
from .scenes import LAYOUTS, layout, surface_cloud, voxelize


def construction_contract():
    return dict(version=1,method='first passing pair in declared finite product ordering; geometric predicates only',
        mirrored_joint_1_magnitudes=[0.8,1.0],joint_2=[0.6,0.4,0.8,0.2,1.0],
        joint_4=[-0.8,-1.2,-0.4,0.4,0.8,1.2],joint_6=[0.0,0.4,-0.4,0.8,-0.8,1.2,-1.2],
        joints_3_5_7=0.0,side_order=['right','left'],canonicalization='float32 then float64',
        tcp_x_interval=[0.2,0.8],tcp_abs_y_interval=[0.25,0.8],tcp_z_interval=[0.25,0.9],
        minimum_independent_clearance_m=0.001,layouts=list(range(len(LAYOUTS))),
        selection='All generated candidates receive FK, limit and native static/all-layout tests. In product order, eligible pairs receive independent static/all-layout voxel and exact-solid signed distances. Stop at the first passing pair; later independent predicates explicitly remain untested.',
        forbidden_selection_information=['IK success','roadmap connectivity','planner results','trajectory optimizer results'],
        immutable_geometry='Pinned public boxes, table, all 15 layouts, native and Drake filters; no geometry or pair exclusion changes',
        original_author_anchors_recovered=False,physical_experiment=False)


def distance_record(parts,q):
    _,context,plant,pc,sg,inspector,_=parts
    plant.SetPositions(pc,q)
    ds=sg.get_query_output_port().Eval(sg.GetMyContextFromRoot(context)).ComputeSignedDistancePairwiseClosestPoints()
    names=[];distances=[]
    for d in ds:
        names.append([inspector.GetName(d.id_A),inspector.GetName(d.id_B)])
        distances.append(float(d.distance))
    values=np.asarray(distances)
    j=int(values.argmin()) if len(values) else None
    return values,names,dict(minimum_signed_distance=float(values.min()) if len(values) else None,
        nearest_pair=names[j] if j is not None else None,
        penetrations=[dict(pair=names[i],distance=float(values[i])) for i in np.flatnonzero(values<0)])


def execute(args):
    out=Path(args.output).resolve()
    if out.exists():raise ValueError('Refuse overwrite of anchor construction evidence')
    out.mkdir(parents=True)
    cfg=read(Path(__file__).parent/'config.json');contract=construction_contract()
    for name in ['config.json','construct_anchors.py','model.py','scenes.py']:
        p=Path(__file__).parent/name;target=out/'source_snapshot'/name;target.parent.mkdir(exist_ok=True)
        target.write_bytes(p.read_bytes())
    record=dict(status='running',physical_experiment=False,planning_executed=False,
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in (out/'source_snapshot').iterdir()},
        selection_contract=contract,selected_index=None)
    atomic(out/'preregistration.json',record);atomic(out/'config_before.json',cfg)
    started=time.perf_counter();rows=[];measurements=[]
    try:
        candidates=[]
        for angle,q2,q4,q6 in itertools.product(contract['mirrored_joint_1_magnitudes'],contract['joint_2'],contract['joint_4'],contract['joint_6']):
            candidates.append([[-angle,q2,0,q4,0,q6,0],[angle,q2,0,q4,0,q6,0]])
        q=np.asarray(candidates,np.float32).astype(float);np.save(out/'candidate_configurations.npy',q)
        f=IIWA(cfg);flat=q.reshape(-1,7);poses=np.asarray([f.pose(x)for x in flat]);np.save(out/'candidate_poses.npy',poses)
        xyz=poses[:,:3,3];limits=((flat>=f.lower)&(flat<=f.upper)).all(axis=1)
        workspace=np.ones(len(flat),bool)
        for axis,key in [(0,'tcp_x_interval'),(1,'tcp_abs_y_interval'),(2,'tcp_z_interval')]:
            v=np.abs(xyz[:,axis])if axis==1 else xyz[:,axis];lo,hi=contract[key];workspace&=(v>=lo)&(v<=hi)
        workspace[::2]&=xyz[::2,1]<0;workspace[1::2]&=xyz[1::2,1]>0
        static=f.cuda(flat);all_cuda=[static];models=[]
        for index in contract['layouts']:
            scene=layout(index,cfg);cloud,labels=surface_cloud(scene,cfg['cloud_surface_grid']);centers,keys,mask=voxelize(cloud,labels,scene,cfg['online_voxel_side'])
            g=IIWA(cfg,centers);all_cuda.append(g.cuda(flat));solid,_=drake_model(cfg,solids=scene['solids'])
            models.append((index,g.parts,solid))
            atomic(out/'scenes'/f'{index:02d}.json',scene)
            np.savez_compressed(out/f'layout_{index:02d}_voxels.npz',centers=centers,keys=keys)
        cuda=np.asarray(all_cuda);np.savez_compressed(out/'all_native_predicates.npz',q=flat,static_and_layout_cuda=cuda,workspace=workspace,limits=limits)
        native_eligible=(cuda.all(axis=0)&workspace&limits).reshape(-1,2).all(axis=1)
        for i in range(len(q)):
            rows.append(dict(index=i,q=q[i].tolist(),limits=limits.reshape(-1,2)[i].tolist(),workspace=workspace.reshape(-1,2)[i].tolist(),
                native_static_free=static.reshape(-1,2)[i].tolist(),native_all_layouts_free=cuda[1:].all(axis=0).reshape(-1,2)[i].tolist(),
                independently_examined=False,status='native_or_workspace_rejected'if not native_eligible[i]else'eligible_not_independently_examined'))
        atomic(out/'candidates.json',rows)
        for i in np.flatnonzero(native_eligible):
            rows[i]['independently_examined']=True;passed=True;query_models=[('static',f.parts)]
            for index,vox,solid in models:query_models +=[(f'layout_{index:02d}_voxel',vox),(f'layout_{index:02d}_solid',solid)]
            candidate_dir=out/'independent'/f'{i:04d}';candidate_dir.mkdir(parents=True)
            for label,parts in query_models:
                pair_distances=[];pair_names=None
                for side,x in zip(contract['side_order'],q[i]):
                    distances,names,summary=distance_record(parts,x);pair_distances.append(distances);pair_names=names
                    good=summary['minimum_signed_distance'] is not None and summary['minimum_signed_distance']>=contract['minimum_independent_clearance_m']
                    passed &= good;measurements.append(dict(candidate_index=int(i),model=label,side=side,passed=bool(good),**summary))
                np.savez_compressed(candidate_dir/f'{label}.npz',q=q[i],distance=np.asarray(pair_distances))
                atomic(candidate_dir/f'{label}_pairs.json',pair_names)
            rows[i]['status']='selected'if passed else'independent_clearance_rejected'
            atomic(out/'independent_measurements.json',measurements);atomic(out/'candidates.json',rows)
            if passed:
                record['selected_index']=int(i);record['selected_pair']=q[i].tolist();break
        if record['selected_index'] is None:raise RuntimeError('Declared finite candidate scan exhausted without a valid fixed pair; no parameters or filters changed')
        chosen=record['selected_pair'];cfg['initial_right_configuration']=chosen[0];cfg['right_target_generating_configuration']=chosen[0];cfg['left_target_generating_configuration']=chosen[1]
        cfg['anchor_construction']={'evidence':str(out.relative_to(ROOT)),'candidate_index':record['selected_index'],'minimum_clearance_m':contract['minimum_independent_clearance_m'],'criterion':'First passing pair in preregistered finite geometric ordering; all 15 voxel and exact-solid layouts checked; no planning results used'}
        cfg['assumptions']['targets']='Fixed FK targets selected before any roadmap or planning using the retained finite geometric scan. Virtual state advances only after strict candidate validation.'
        atomic(out/'config_proposed.json',cfg)
        record.update(status='completed',candidate_pairs=len(q),native_eligible_pairs=int(native_eligible.sum()),independently_examined_pairs=sum(x['independently_examined']for x in rows),config_not_applied=True)
    except BaseException as error:
        record.update(status='failed',error=repr(error),traceback=traceback.format_exc());raise
    finally:
        record['wall_seconds']=time.perf_counter()-started;atomic(out/'manifest.json',record)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);execute(p.parse_args())
if __name__=='__main__':main()
