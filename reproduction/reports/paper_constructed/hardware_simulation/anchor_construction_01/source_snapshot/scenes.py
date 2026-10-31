"""Fifteen predeclared tabletop modifications; deterministic virtual surfaces."""
import numpy as np

# (rack x/y or absent, box x/y or absent); coordinates in metres.
LAYOUTS = [
    ((.58, 0), (.62, .30)), ((.50, 0), (.62, .30)),
    ((.44, 0), (.64, -.30)), ((.52, .08), None),
    (None, (.50, 0)), ((.60, -.08), (.70, .28)),
    ((.48, -.08), (.68, .28)), ((.56, .06), (.62, -.28)),
    ((.64, 0), None), ((.46, .04), (.68, -.30)),
    (None, (.58, .12)), ((.54, -.06), (.66, -.28)),
    ((.62, .06), (.56, -.30)), ((.50, 0), None),
    ((.56, 0), (.66, .30)),
]


def layout(index, cfg):
    rack, box = LAYOUTS[index]
    solids = [dict(name='table', center=cfg['table']['center'], size=cfg['table']['size'], online=False)]
    if rack is not None:
        x,y = rack
        for height in (.12,.32):
            solids.append(dict(name=f'rack_shelf_{height}', center=[x,y,height], size=[.22,.36,.025], online=True))
        for a in (-.095,.095):
            for b in (-.165,.165):
                solids.append(dict(name=f'rack_leg_{a}_{b}',center=[x+a,y+b,.16],size=[.03,.03,.32],online=True))
    if box is not None:
        solids.append(dict(name='box',center=[box[0],box[1],.11],size=[.16,.16,.22],online=True))
    return dict(index=index,target_side='left'if index%2==0 else 'right',
        rack_translation_xy=rack,box_translation_xy=box,solids=solids,
        provenance='predeclared constructed dimensions/layout; not original camera/obstacle records')


def surface_cloud(scene, spacing):
    clouds,labels=[],[]
    for index,solid in enumerate(scene['solids']):
        size=np.asarray(solid['size']);center=np.asarray(solid['center'])
        points=[]
        for axis in range(3):
            other=[j for j in range(3)if j!=axis]
            grid=[np.linspace(-size[j]/2,size[j]/2,int(np.ceil(size[j]/spacing))+1)for j in other]
            a,b=np.meshgrid(*grid,indexing='ij')
            for sign in (-1,1):
                face=np.empty((a.size,3));face[:,axis]=sign*size[axis]/2
                face[:,other[0]]=a.ravel();face[:,other[1]]=b.ravel();points.append(face+center)
        points=np.unique(np.vstack(points),axis=0)
        clouds.append(points);labels.extend([index]*len(points))
    return np.vstack(clouds),np.asarray(labels,dtype=np.int32)


def voxelize(points, labels, scene, side):
    # Known table geometry is already a static collision object. Its synthetic
    # points are retained but explicitly removed as known static background.
    online=np.array([x['online']for x in scene['solids']],dtype=bool)[labels]
    keys=np.unique(np.floor(points[online]/side).astype(np.int64),axis=0)
    centers=((keys+.5)*side).astype(np.float32)
    return centers,keys,online
