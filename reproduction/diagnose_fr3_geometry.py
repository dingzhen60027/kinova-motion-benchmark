"""Inspect frozen q7 collisions versus official mesh convex hulls; no benchmark edits."""
from pathlib import Path
import csv
import xml.etree.ElementTree as ET
import numpy as np
import csdecomp as csd
from pydrake.all import RobotDiagramBuilder, RigidTransform, CoulombFriction, Convex
from reproduction.common import ROOT, read, atomic
from reproduction.paper_completion.model import FR3, fr3_urdf
from reproduction.paper_completion.build_assets import stl
OUT=ROOT/'reproduction/reports/paper_completion/diagnostics/pipeline/mesh_comparison'
OUT.mkdir(parents=True,exist_ok=True)
cfg=read(ROOT/'reproduction/runs/20261025T082752.022456Z_paper_fr3/manifest.json')['config']
f=FR3(cfg)
# Use exactly the same chain; remove every sphere and register only the two
# official bodies implicated by the independent signed-distance query.
xml=ET.fromstring(fr3_urdf())
for link in xml.findall('link'):
 for collision in link.findall('collision'):link.remove(collision)
b=RobotDiagramBuilder();p=b.plant();sg=b.scene_graph()
instance=b.parser().AddModelsFromString(ET.tostring(xml,encoding='unicode'),'urdf')[0]
geometries={}
for body,relative in [('fr3_link5','meshes/robots/fr3/collision/link5.stl'),('fr3_hand','meshes/robot_ee/franka_hand_white/collision/hand.stl')]:
 source=ROOT/'reproduction/vendor/franka_description'/relative
 triangles=stl(source); vertices,inverse=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
 obj=OUT/(body+'.obj')
 obj.write_text(''.join('v %.17g %.17g %.17g\n'%tuple(v) for v in vertices)+''.join('f %d %d %d\n'%tuple(face+1) for face in inverse.reshape(-1,3)))
 geometries[body]=p.RegisterCollisionGeometry(p.GetBodyByName(body,instance),RigidTransform(),Convex(str(obj)),body+'_official_mesh_convex_hull',CoulombFriction(.9,.8))
p.Finalize();diagram=b.Build();context=diagram.CreateDefaultContext();pc=p.GetMyMutableContextFromRoot(context)
drm=csd.DRM();drm.Read(str(ROOT/'reproduction/runs/20261025T082752.022456Z_paper_fr3/roadmaps/r01_n03000/native.map'))
base=np.asarray(drm.id_to_node_map[2922],float)
configurations=[('home',np.asarray([0,-np.pi/4,0,-3*np.pi/4,0,np.pi/2,0]))]
for q7 in np.linspace(-1.5,1.5,31):
 q=base.copy();q[-1]=q7;configurations.append(('node2922_q7_scan',q))
rows=[]
for label,q in configurations:
 f.drake.SetPositions(f.pc,q)
 query=f.sg.get_query_output_port().Eval(f.sg.GetMyContextFromRoot(f.context))
 collisions=sorted([x for x in query.ComputeSignedDistancePairwiseClosestPoints(0) if x.distance<0],key=lambda x:x.distance)
 pairs=[dict(first_geometry=f.inspector.GetName(x.id_A),second_geometry=f.inspector.GetName(x.id_B),distance=x.distance) for x in collisions]
 p.SetPositions(pc,q);mq=sg.get_query_output_port().Eval(sg.GetMyContextFromRoot(context))
 distance=mq.ComputeSignedDistancePairClosestPoints(geometries['fr3_link5'],geometries['fr3_hand']).distance
 rows.append(dict(label=label,q7=float(q[-1]),configuration=q,sphere_collision_count=len(pairs),sphere_pairs=pairs,official_link5_hand_convex_hull_distance=float(distance),hulls_separated=bool(distance>0)))
atomic(OUT/'results.json',dict(scope='Pairwise diagnostic only; not replacement model or performance benchmark',mesh_source_revision='7aeeddc449edf8d62b594f9e36a81da53e7796f9',mesh_interpretation='Convex hulls enclose official collision meshes. Positive separation proves this body pair does not collide for these meshes; penetration of hulls alone would not prove mesh collision.',rows=rows))
with (OUT/'joint_scan.csv').open('w') as stream:
 writer=csv.DictWriter(stream,fieldnames=['label','q7','sphere_collision_count','deepest_sphere_penetration','deepest_first_geometry','deepest_second_geometry','official_link5_hand_convex_hull_distance','hulls_separated']);writer.writeheader()
 for r in rows:
  pairs=r['sphere_pairs'];writer.writerow(dict(label=r['label'],q7=r['q7'],sphere_collision_count=len(pairs),deepest_sphere_penetration=pairs[0]['distance'] if pairs else 0,deepest_first_geometry=pairs[0]['first_geometry'] if pairs else '',deepest_second_geometry=pairs[0]['second_geometry'] if pairs else '',official_link5_hand_convex_hull_distance=r['official_link5_hand_convex_hull_distance'],hulls_separated=r['hulls_separated']))
print('home',rows[0]);print('scan_count',len(rows)-1,'sphere_collision_points',sum(r['sphere_collision_count']>0 for r in rows[1:]),'official_hulls_separated',sum(r['hulls_separated'] for r in rows))
