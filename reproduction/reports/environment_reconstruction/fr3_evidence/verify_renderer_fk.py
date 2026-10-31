"""Compare render-only CPU FK against the actual Drake model, not visual guesses."""
from pathlib import Path
import json
import numpy as np
from reproduction.environment_reconstruction.render import fk, robot_mesh, ASSET
from reproduction.paper_constructed.model import drake_fr3
from reproduction.common import ROOT, read
asset=read(ASSET)
cfg=read(ROOT/'reproduction/paper_constructed/config.json')
diagram,context,plant,pc,sg,inspector,instance=drake_fr3(cfg)
rng=np.random.default_rng(250410783)
qs=rng.uniform(plant.GetPositionLowerLimits(),plant.GetPositionUpperLimits(),(32,7))
errors=[]
for q in qs:
 plant.SetPositions(pc,q)
 tf=fk(q,asset)
 for name,m in tf.items():
  actual=plant.EvalBodyPoseInWorld(pc,plant.GetBodyByName(name,instance)).GetAsMatrix4()
  errors.append(float(np.max(np.abs(actual-m))))
 _,_,balls=robot_mesh(q,asset)
 assert len(balls)==33
result=dict(status='passed' if max(errors)<1e-12 else 'failed',poses=32,
            body_transforms=len(errors),max_absolute_transform_error=max(errors),
            all_robot_sphere_counts=33,gpu_calls=0)
Path(__file__).with_name('renderer_fk_validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(result)
assert result['status']=='passed'
