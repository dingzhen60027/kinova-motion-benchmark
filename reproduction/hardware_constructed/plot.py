"""Render every requested trial from saved records; never select by success."""
import argparse
import hashlib
import itertools
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pydrake.math import RigidTransform, RollPitchYaw
from reproduction.common import atomic, read
from .model import IIWA, robot_urdf

CORNERS=np.array(list(itertools.product([-.5,.5],repeat=3)))
FACES=np.array([[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]])


def box(ax,size,transform,color,alpha=.6):
    points=CORNERS*np.asarray(size)
    world=points@transform[:3,:3].T+transform[:3,3]
    ax.add_collection3d(Poly3DCollection(world[FACES],facecolors=color,edgecolors='#333333',linewidths=.25,alpha=alpha))


def robot(ax,model,xml,q,alpha=.7):
    model.drake.SetPositions(model.pc,q)
    for link in xml.findall('link'):
        if link.attrib['name']=='probe_table':continue
        body=model.drake.GetBodyByName(link.attrib['name'],model.instance)
        frame=model.drake.EvalBodyPoseInWorld(model.pc,body).GetAsMatrix4()
        for collision in link.findall('collision'):
            shape=collision.find('geometry/box')
            if shape is None:continue
            origin=collision.find('origin')
            xyz=np.fromstring(origin.attrib.get('xyz','0 0 0'),sep=' ')if origin is not None else np.zeros(3)
            rpy=np.fromstring(origin.attrib.get('rpy','0 0 0'),sep=' ')if origin is not None else np.zeros(3)
            transform=frame@RigidTransform(RollPitchYaw(rpy),xyz).GetAsMatrix4()
            box(ax,np.fromstring(shape.attrib['size'],sep=' '),transform,'#337da8',alpha)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    root=args.run_dir.resolve();manifest=read(root/'manifest.json');cfg=manifest['config'];indices=manifest['trials']
    if manifest['status']in {'running','pending'}:raise ValueError('Wait for the run to stop before plotting')
    args.output.mkdir(parents=True,exist_ok=True)
    f=IIWA(cfg);xml=ET.fromstring(robot_urdf(cfg))
    rows=int(np.ceil(len(indices)/3));fig=plt.figure(figsize=(14,4.4*rows));records=[];inputs={}
    def note(path):
        if path.exists():inputs[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    note(root/'manifest.json')
    for panel,index in enumerate(indices,1):
        ax=fig.add_subplot(rows,3,panel,projection='3d');out=root/'trials'/f'{index:02d}'
        result_path=out/'result.json';result=read(result_path)if result_path.exists()else dict(status='missing_result')
        input_path=out/'input.json';request=read(input_path)if input_path.exists()else None
        scene_path=root/'scenes'/f'{index:02d}.json'
        if scene_path.exists():
            scene=read(scene_path)
            for solid in scene['solids']:
                transform=np.eye(4);transform[:3,3]=solid['center']
                color='#b6b7b8'if solid['name']=='table'else('#d38b3f'if solid['name']=='box'else'#719957')
                box(ax,solid['size'],transform,color,.22 if solid['name']=='table'else .5)
        q0=np.asarray(request['start']if request else cfg['initial_right_configuration'])
        robot(ax,f,xml,q0)
        trajectory=out/'SCSTO/trajectory_samples.npz';sample_count=0
        if trajectory.exists():
            with np.load(trajectory)as values:q=values['q'];sample_count=len(q)
            selected=np.linspace(0,len(q)-1,min(301,len(q))).astype(int)
            xyz=np.array([f.pose(point)[:3,3]for point in q[selected]])
            success=bool(result.get('executed_in_virtual_state'))
            ax.plot(*xyz.T,color='#087443'if success else'#bb3046',linewidth=2)
            robot(ax,f,xml,q[-1],alpha=.16)
        if request:
            target=np.asarray(request['target_pose'])[:3,3];ax.scatter(*target,color='black',marker='x',s=35)
        status=result.get('status','unknown');detail='left'if index%2==0 else'right'
        if result.get('exact_scene_collision_samples',0):detail+=f"; solid collisions={result['exact_scene_collision_samples']}"
        if request is None:detail+='; declared initial pose (no recorded start)'
        ax.set_title(f'{index+1:02d}  {status}\n{detail}',fontsize=9)
        ax.set(xlim=(-.35,1.1),ylim=(-.85,.85),zlim=(-.1,1.35),xlabel='x [m]',ylabel='y [m]',zlabel='z [m]')
        ax.set_box_aspect((1.45,1.7,1.45));ax.view_init(elev=24,azim=-57);ax.tick_params(labelsize=7)
        for path in [result_path,input_path,scene_path,trajectory]:note(path)
        records.append(dict(index=index,status=status,all_saved_trajectory_samples=sample_count,
            displayed_trajectory_samples=min(301,sample_count),success=bool(result.get('executed_in_virtual_state')),
            exact_solid_collision_samples=result.get('exact_scene_collision_samples'),missing_recorded_start=request is None))
    fig.suptitle('Constructed hardware-condition simulation — all requested trials\nPublic 11-box iiwa/WSG, declared table/rack/box; no physical execution or camera measurements',fontsize=14,y=.997)
    fig.legend(handles=[Patch(color='#337da8',label='Robot at recorded start'),Patch(color='#337da8',alpha=.16,label='Saved candidate endpoint'),Patch(color='#719957',label='Rack solids'),Patch(color='#d38b3f',label='Box solid'),Patch(color='#087443',label='Validated TCP candidate'),Patch(color='#bb3046',label='Rejected saved TCP candidate')],loc='lower center',ncol=3,fontsize=9)
    fig.tight_layout(rect=(0,.025,1,.97));path=args.output/'hardware_simulation_all_trials.png';fig.savefig(path,dpi=160);plt.close(fig)
    atomic(args.output/'plot_manifest.json',dict(status='completed',figure=str(path),figure_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        plot_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),run_dir=str(root),selection='Every manifest trial, including failures and missing records; no best-case selection',
        physical_execution=False,panels=records,input_sha256=inputs,notes='Ghost robot is the saved candidate endpoint even if rejected. TCP path is subsampled only for display; original validation/replay arrays are unchanged.'))
    print(path,flush=True)


if __name__=='__main__':main()
