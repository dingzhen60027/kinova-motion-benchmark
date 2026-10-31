from pathlib import Path
import json,time,traceback
from reproduction.common import read,atomic
from reproduction.hardware_constructed.model import drake_model
r=Path('/home/eric/wjg/wd/csdecomp');out=r/'reproduction/reports/paper_constructed/hardware_simulation/check_01';cfg=read(r/'reproduction/hardware_constructed/config.json')
parts,_=drake_model(cfg);_,ctx,p,pc,sg,ins,_=parts
rows=[]
for key in ['initial_right_configuration','left_target_generating_configuration']:
 p.SetPositions(pc,cfg[key]);query=sg.get_query_output_port().Eval(sg.GetMyContextFromRoot(ctx));ds=query.ComputeSignedDistancePairwiseClosestPoints()
 distances=sorted([dict(a=ins.GetName(d.id_A),b=ins.GetName(d.id_B),distance=float(d.distance))for d in ds],key=lambda d:d['distance'])
 rows.append(dict(anchor=key,q=cfg[key],distances=distances,penetrating_pairs=[d for d in distances if d['distance']<0]))
atomic(out/'anchor_pair_diagnosis.json',dict(source_snapshot='source_snapshot/manifest.json',rows=rows))
print(json.dumps([dict(anchor=x['anchor'],penetrating_pairs=x['penetrating_pairs'],closest=x['distances'][:3])for x in rows],indent=2))
