"""Public 11-box iiwa/WSG assembly with matched native and Drake predicates."""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET
import numpy as np
import csdecomp as csd
from pydrake.all import RobotDiagramBuilder, Role, CollisionFilterDeclaration, GeometrySet
from reproduction.common import read
from reproduction.probe_public_iiwa import SOURCES, assemble
from reproduction.paper_constructed.model import FR3

ASSETS=Path(__file__).parent/'assets'


def robot_urdf(cfg):
    for name,_,expected in SOURCES:
        if hashlib.sha256((ASSETS/name).read_bytes()).hexdigest()!=expected:
            raise ValueError('Pinned public URDF mismatch: '+name)
    root=ET.fromstring(assemble(ASSETS,table=True))
    root.find("link[@name='probe_table']/collision/geometry/box").set('size',' '.join(map(str,cfg['table']['size'])))
    root.find("joint[@name='table_weld']/origin").set('xyz',' '.join(map(str,cfg['table']['center'])))
    root.append(ET.fromstring('<link name="hardware_tcp"/>'))
    offset=' '.join(map(str,cfg['tcp_from_gripper']))
    root.append(ET.fromstring(f'<joint name="hardware_tcp_weld" type="fixed"><parent link="wsg50_gripper_link"/><child link="hardware_tcp"/><origin xyz="{offset}"/></joint>'))
    return ET.tostring(root,encoding='unicode')


def drake_model(cfg,centers=None,solids=None):
    root=ET.fromstring(robot_urdf(cfg))
    dynamic=[]
    for i,center in enumerate([]if centers is None else centers):
        name=f'virtual_voxel_{i}'
        shape=f'<sphere radius="{cfg["online_voxel_radius"]}"/>'
        dynamic.append((name,center,shape))
    for i,solid in enumerate([]if solids is None else solids):
        if solid['online']:
            dynamic.append((f'ground_truth_box_{i}',solid['center'],f'<box size="{" ".join(map(str,solid["size"]))}"/>'))
    for name,center,shape in dynamic:
        root.append(ET.fromstring(f'<link name="{name}"><collision name="{name}"><geometry>{shape}</geometry></collision></link>'))
        root.append(ET.fromstring(f'<joint name="{name}_weld" type="fixed"><parent link="world"/><child link="{name}"/><origin xyz="{" ".join(map(str,center))}"/></joint>'))
    builder=RobotDiagramBuilder();plant,sg=builder.plant(),builder.scene_graph()
    instance=builder.parser().AddModelsFromString(ET.tostring(root,encoding='unicode'),'urdf')[0]
    plant.Finalize()
    robot_ids=[]
    for link in root.findall('link'):
        name=link.attrib['name']
        if name!='probe_table' and not name.startswith(('virtual_voxel_','ground_truth_box_')):
            robot_ids.extend(plant.GetCollisionGeometriesForBody(plant.GetBodyByName(name,instance)))
    if len(robot_ids)!=11:raise ValueError('Expected the pinned 8+3 robot collision boxes')
    for name,_,_ in dynamic:
        ids=plant.GetCollisionGeometriesForBody(plant.GetBodyByName(name,instance))
        sg.collision_filter_manager().Apply(CollisionFilterDeclaration().AllowBetween(GeometrySet(robot_ids),GeometrySet(ids)))
    diagram=builder.Build();context=diagram.CreateDefaultContext();pc=plant.GetMyMutableContextFromRoot(context)
    inspector=sg.get_query_output_port().Eval(sg.GetMyContextFromRoot(context)).inspector()
    return (diagram,context,plant,pc,sg,inspector,instance),robot_ids


def native_model(parts,robot_ids):
    _,_,plant,pc,_,inspector,_=parts
    converted=csd.convert_drake_plant_to_csd_plant(plant,pc,inspector)
    ids=[g for frame in inspector.GetAllFrameIds()for g in inspector.GetGeometries(frame,Role.kProximity)]
    lookup={g:i for i,g in enumerate(ids)}
    assert len(ids)==len(converted.getSceneCollisionGeometries())
    pairs=sorted((min(lookup[a],lookup[b]),max(lookup[a],lookup[b]))for a,b in inspector.GetCollisionCandidates())
    si=converted.getSceneInspector();si.robot_to_world_collision_pairs=pairs;si.robot_geometry_ids=sorted(lookup[g]for g in robot_ids)
    native=csd.Plant(converted.getKinematicTree(),si,converted.getSceneCollisionGeometries())
    assert native.getMinimalPlant().num_collision_pairs==len(pairs)
    return native


class IIWA:
    kind='hardware_simulation'
    dimension=7
    pose=FR3.pose
    ik=FR3.ik

    def __init__(self,cfg,centers=None):
        self.protocol=cfg
        self.parts,robot_ids=drake_model(cfg,centers)
        self.diagram,self.context,self.drake,self.pc,self.sg,self.inspector,self.instance=self.parts
        if centers is None:self.native=native_model(self.parts,robot_ids)
        else:
            static,ids=drake_model(cfg);self.native=native_model(static,ids)
        self.minimal=self.native.getMinimalPlant();self.geometry_ids=self.native.getRobotGeometryIds()
        self.lower=self.drake.GetPositionLowerLimits();self.upper=self.drake.GetPositionUpperLimits()
        if len(self.lower)!=7 or len(self.geometry_ids)!=11:raise ValueError('Expected seven joints and all 11 online robot boxes')
        self.domain=csd.HPolyhedron();self.domain.MakeBox(self.lower,self.upper)
        self.voxels=csd.Voxels(np.empty((3,0),np.float32)if centers is None else np.asarray(centers,np.float32).T)
        self.voxel_radius=cfg['online_voxel_radius']
        self.cfg=dict(cfg,domain_lower=self.lower.tolist(),domain_upper=self.upper.tolist())
        self.tcp=self.drake.GetFrameByName(cfg['tcp_link'],self.instance)

    def cuda(self,points):
        return np.asarray(csd.CheckCollisionFreeVoxelsCuda(np.asarray(points,np.float32).T,self.voxels,self.voxel_radius,self.minimal,self.geometry_ids),bool)

    independent=FR3.independent

    def metadata(self):
        pairs=sorted(sorted([self.inspector.GetName(a),self.inspector.GetName(b)])for a,b in self.inspector.GetCollisionCandidates())
        static=[x for x in pairs if not any('virtual_voxel_'in y for y in x)]
        return dict(robot='public KUKA LBR iiwa7 R800 + WSG',physical_experiment=False,robot_boxes=11,
            dynamic_robot_boxes=len(self.geometry_ids),positions=len(self.lower),lower=self.lower,upper=self.upper,
            static_pairs_including_table=len(static),robot_table_pairs=sum(any('probe_table'in y for y in x)for x in static),
            all_drake_candidate_pairs=len(pairs),native_static_pairs=int(self.minimal.num_collision_pairs),pair_names=pairs,
            filter='Default Drake robot/weld filters; explicitly restore every robot box vs synthetic obstacle; no added self exclusions',
            original_105_pair_model_recovered=False)


def exact_box_flags(cfg,scene,points):
    parts,_=drake_model(cfg,solids=scene['solids'])
    diagram,context,plant,pc,sg,_,_=parts;flags=[];minimum=[]
    for q in points:
        plant.SetPositions(pc,q)
        distances=sg.get_query_output_port().Eval(sg.GetMyContextFromRoot(context)).ComputeSignedDistancePairwiseClosestPoints()
        distance=min((x.distance for x in distances),default=1e6)
        flags.append(distance>=0);minimum.append(distance)
    return np.asarray(flags,bool),np.asarray(minimum)
