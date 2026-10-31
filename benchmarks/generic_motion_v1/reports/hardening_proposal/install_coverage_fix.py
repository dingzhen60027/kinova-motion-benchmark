"""Authorized precision repair, limited to benchmark wrapper and config."""
from pathlib import Path
import json
p=Path('benchmarks/generic_motion_v1/runners/csdecomp_baseline.py')
s=p.read_text()
helper='''def cover_region_anchors(region,anchors,maximum_normalized_correction,out=None):
    """Normalize float32 region handoff without moving the fixed endpoints.

    A plane's correction is divided by its normal norm, so row scaling cannot
    evade the fixed tolerance. Larger errors fail closed. Every saved native
    halfspace and every anchor remains inspectable before and after the repair.
    """
    from pydrake.all import HPolyhedron
    A=np.asarray(region.A(),dtype=float)
    b=np.asarray(region.b(),dtype=float)
    anchors=np.asarray(anchors,dtype=float)
    directory=None if out is None else Path(out)
    if directory is not None:
        directory.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(directory/'before.npz',A=A,b=b,anchors=anchors)
    if (A.ndim!=2 or A.shape[1]!=7 or b.shape!=(len(A),) or anchors.ndim!=2
            or anchors.shape[1]!=7 or len(anchors)==0 or not np.isfinite(A).all()
            or not np.isfinite(b).all() or not np.isfinite(anchors).all()
            or not np.isfinite(maximum_normalized_correction) or maximum_normalized_correction<0):
        raise ValueError('Malformed region, anchors, or numerical coverage tolerance')
    residual=A@anchors.T-b[:,None]
    correction=np.maximum(0.,np.max(residual,axis=1))
    norms=np.linalg.norm(A,axis=1)
    normalized=np.divide(correction,norms,out=np.zeros_like(correction),where=norms>0)
    normalized[(norms==0)&(correction>0)]=np.inf
    largest=float(np.max(normalized,initial=0.))
    record=dict(status='accepted' if largest<=maximum_normalized_correction else 'rejected',
        maximum_normalized_correction=largest,allowed_normalized_correction=maximum_normalized_correction,
        corrected_planes=int(np.count_nonzero(correction)),normal_matrix_changed=False,
        fixed_endpoints_changed=False,maximum_raw_anchor_residual_before=float(np.max(residual,initial=0.)),
        note='Bounded numerical region handoff only; unchanged robot/obstacles; final collision validator still applies')
    if directory is not None:write(directory/'coverage.json',record)
    if largest>maximum_normalized_correction:
        raise ValueError(f'Region requires normalized endpoint correction {largest:.9g}, '
                         f'exceeding fixed numerical tolerance {maximum_normalized_correction:.9g}')
    corrected_b=b+correction
    result=HPolyhedron(A,corrected_b)
    after=A@anchors.T-corrected_b[:,None]
    record['maximum_raw_anchor_residual_after']=float(np.max(after,initial=0.))
    if directory is not None:
        np.savez_compressed(directory/'after.npz',A=A,b=corrected_b,anchors=anchors,
                            normalized_corrections=normalized,raw_corrections=correction)
        write(directory/'coverage.json',record)
    return result


def _region_anchors(path,index,endpoints):
    anchors=[path[index],path[index+1]]
    if index==0:anchors.append(endpoints[0])
    if index==len(path)-2:anchors.append(endpoints[1])
    return np.asarray(anchors,dtype=float)


'''
needle='def dependency_audit(out):'
assert s.count(needle)==1;s=s.replace(needle,helper+needle)
old='''                    if containing is not None:
                        regions.append(containing)
                    else:
                        native=inflator.inflateEdge(a,b,model.voxels,model.voxel_radius,False)
                        region=HPolyhedron(native.A(),native.b())
                        residual=max(float(np.max(region.A()@p-region.b())) for p in (a,b))
                        # Tiny float32 endpoint normalization only; the final
                        # independent checker still controls candidate acceptance.
                        if residual>1e-5:
                            raise RuntimeError(f'EI-ZO omitted its fixed seed edge by {residual} rad')
                        corrected=np.maximum(region.b(),np.maximum(region.A()@a,region.A()@b))
                        regions.append(HPolyhedron(region.A(),corrected))
                    _save_regions(out,regions)
'''
new='''                    if containing is not None:
                        region=containing
                    else:
                        native=inflator.inflateEdge(a,b,model.voxels,model.voxel_radius,False)
                        region=HPolyhedron(native.A(),native.b())
                    region=cover_region_anchors(region,_region_anchors(path,i,endpoints),
                        cfg['region_coverage_tolerance_rad'],out/'coverage'/'inflation'/f'{i:04d}')
                    regions.append(region)
                    _save_regions(out,regions)
'''
assert s.count(old)==1;s=s.replace(old,new)
old='''                    regions=[HPolyhedron(r.A(),r.b()) for r in corrected[0]]
                    for i,(a,b) in enumerate(zip(path[:-1],path[1:])):
                        if not regions[i].PointInSet(a,1e-6) or not regions[i].PointInSet(b,1e-6):
                            native=inflator.inflateEdge(a,b,model.voxels,model.voxel_radius,False)
                            regions[i]=HPolyhedron(native.A(),native.b())
                    _save_regions(out,regions)
'''
new='''                    regions=[HPolyhedron(r.A(),r.b()) for r in corrected[0]]
                    for i,(a,b) in enumerate(zip(path[:-1],path[1:])):
                        regions[i]=cover_region_anchors(regions[i],_region_anchors(path,i,endpoints),
                            cfg['region_coverage_tolerance_rad'],
                            out/'coverage'/f'recovery_{attempt:03d}'/'edited'/f'{i:04d}')
                        if not regions[i].PointInSet(a,1e-6) or not regions[i].PointInSet(b,1e-6):
                            native=inflator.inflateEdge(a,b,model.voxels,model.voxel_radius,False)
                            regions[i]=cover_region_anchors(HPolyhedron(native.A(),native.b()),
                                _region_anchors(path,i,endpoints),cfg['region_coverage_tolerance_rad'],
                                out/'coverage'/f'recovery_{attempt:03d}'/'reinflated'/f'{i:04d}')
                    _save_regions(out,regions)
'''
assert s.count(old)==1;s=s.replace(old,new)
p.write_text(s)
p=Path('benchmarks/generic_motion_v1/configs/evaluation.json')
cfg=json.loads(p.read_text());cfg['region_coverage_tolerance_rad']=1e-6
p.write_text(json.dumps(cfg,indent=2)+'\n')
