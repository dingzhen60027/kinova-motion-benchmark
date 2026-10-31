"""Construct a 33-sphere FR3 approximation from geometry, without planning scores.

Each subdivided official collision triangle is enclosed wholly in one fitted
sphere. Deterministic minimax cluster fits are compared under a fixed total
sphere-volume objective; dynamic programming allocates exactly 33 spheres.
The continuous surface certificate does not assert complete solid coverage.
"""
from pathlib import Path
import argparse
import copy
import hashlib
import itertools
import math
import time
import traceback
import numpy as np
from scipy.optimize import minimize
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from scipy.stats import qmc
from reproduction.common import ROOT, atomic, read

SOURCE = ROOT / 'reproduction/vendor/franka_description'
ASSETS = Path(__file__).parent / 'assets'
REPORTS = ROOT / 'reproduction/reports/paper_constructed/model_construction'
BASELINE = ROOT / 'reproduction/paper_completion/assets/fr3_33.json'
MAX_EDGE = .02
BUFFER = 1e-5
TOTAL = 33


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stl(path):
    raw = Path(path).read_bytes()
    count = int.from_bytes(raw[80:84], 'little')
    if len(raw) != 84 + count * 50:
        raise ValueError('Expected binary STL: ' + str(path))
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    return np.frombuffer(raw, dtype=dtype, offset=84)['vertices'].astype(float)


def finger_boxes(right=False):
    return [([0,.0185,.011],[.022,.015,.020],[0,0,0]),
            ([0,.0068,.0022],[.022,.0088,.0038],[0,0,0]),
            ([0,.0159,.02835],[.0175,.007,.0235],[-np.pi/6,0,np.pi] if right else [np.pi/6,0,0]),
            ([0,.00758,.04525],[.0175,.0152,.0185],[0,0,0])]


def finger_points(right=False):
    corners = np.array(list(itertools.product([-1.,1.], repeat=3))) / 2
    return np.vstack([Rotation.from_euler('xyz', angles).apply(corners * size) + center
                      for center,size,angles in finger_boxes(right)])


def finger_triangles(right=False):
    corners = np.array(list(itertools.product([-1.,1.], repeat=3))) / 2
    triangles=[]
    for center,size,angles in finger_boxes(right):
        points=Rotation.from_euler('xyz', angles).apply(corners * size)+center
        triangles.extend(points[ConvexHull(points).simplices])
    return np.array(triangles)


def subdivide(triangles):
    pending=list(triangles); answer=[]
    while pending:
        triangle=pending.pop()
        lengths=np.array([np.linalg.norm(triangle[(i+1)%3]-triangle[i]) for i in range(3)])
        i=int(np.argmax(lengths))
        if lengths[i] <= MAX_EDGE:
            answer.append(triangle)
        else:
            a,b,c=triangle[i],triangle[(i+1)%3],triangle[(i+2)%3]
            midpoint=(a+b)/2
            pending.extend([np.array([a,midpoint,c]),np.array([midpoint,b,c])])
    return np.array(answer)


def enclosing_ball(points):
    points=np.unique(points.reshape(-1,3),axis=0)
    origin=points.mean(0);scale=max(float(np.ptp(points,axis=0).max()),1e-8)
    x=(points-origin)/scale
    initial=np.r_[np.zeros(3),np.sum(x*x,axis=1).max()]
    def constraint(z):return z[3]-np.sum((x-z[:3])**2,axis=1)
    def jacobian(z):return np.c_[2*(x-z[:3]),np.ones(len(x))]
    result=minimize(lambda z:z[3],initial,jac=lambda z:np.array([0.,0.,0.,1.]),
        method='SLSQP',constraints=[dict(type='ineq',fun=constraint,jac=jacobian)],
        bounds=[(None,None)]*3+[(0,None)],options=dict(ftol=1e-11,maxiter=100,disp=False))
    center=origin+scale*result.x[:3]
    radius=float(np.linalg.norm(points-center,axis=1).max()+BUFFER)
    return center,radius,dict(success=bool(result.success),status=int(result.status),message=str(result.message),iterations=int(result.nit))


def fit_triangles(triangles,count,strategy):
    points=triangles.mean(axis=1)
    areas=np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1)
    if strategy=='farthest':
        centers=[points[np.argmax(np.linalg.norm(points-np.average(points,axis=0,weights=areas),axis=1))]]
        for _ in range(1,count):
            distance=np.linalg.norm(points[:,None]-np.array(centers)[None],axis=2).min(axis=1)
            centers.append(points[np.argmax(distance)])
        centers=np.array(centers)
    else:
        mean=np.average(points,axis=0,weights=areas)
        _,_,vectors=np.linalg.svd((points-mean)*np.sqrt(areas[:,None]),full_matrices=False)
        axis=vectors[0]
        if axis[np.argmax(abs(axis))]<0:axis=-axis
        projection=(points-mean)@axis
        order=np.argsort(projection,kind='stable'); cumulative=np.cumsum(areas[order]);cumulative/=cumulative[-1]
        centers=points[order[np.searchsorted(cumulative,(np.arange(count)+.5)/count)]]
    previous=None;best=None;iterations=[]
    for iteration in range(24):
        distances=np.linalg.norm(points[:,None]-centers[None],axis=2)
        labels=distances.argmin(axis=1)
        # Deterministic repair of an empty cluster, independent of configurations.
        for i in range(count):
            if not np.any(labels==i):
                eligible=np.array([np.count_nonzero(labels==label)>1 for label in labels])
                residual=distances[np.arange(len(points)),labels]
                residual[~eligible]=-1
                labels[np.argmax(residual)]=i
        if previous is not None and np.array_equal(labels,previous):break
        previous=labels.copy();balls=[];solver=[]
        for i in range(count):
            center,radius,info=enclosing_ball(triangles[labels==i]);balls.append(dict(center=center,radius=radius));solver.append(info)
        volume=sum(4*np.pi/3*b['radius']**3 for b in balls)
        iterations.append(dict(iteration=iteration,sphere_volume_sum=volume,enclosing_solvers=solver))
        if best is None or volume<best['sphere_volume_sum']:
            best=dict(spheres=balls,labels=labels.copy(),sphere_volume_sum=volume)
        centers=np.array([b['center']for b in balls])
    return dict(**best,strategy=strategy,iterations=iterations)



def recursive_candidates(triangles,maximum):
    """Geometry-only binary partitions; no configuration-space observations."""
    points=triangles.mean(axis=1)
    areas=np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1)
    def ball(indices):
        center,radius,info=enclosing_ball(triangles[indices])
        return dict(center=center,radius=radius),info
    def volume(item):return 4*np.pi/3*item['radius']**3
    groups=[np.arange(len(points))];first,info=ball(groups[0]);balls=[first]
    history=[dict(count=1,enclosing_solver=info)];results={};cache={}
    for count in range(1,maximum+1):
        labels=np.empty(len(points),dtype=int)
        for i,group in enumerate(groups):labels[group]=i
        results[count]=dict(spheres=copy.deepcopy(balls),labels=labels,sphere_volume_sum=sum(map(volume,balls)),
                            strategy='recursive_volume_splits',iterations=copy.deepcopy(history))
        if count==maximum:break
        options=[]
        for index,group in enumerate(groups):
            if len(group)<2:continue
            key=tuple(group)
            if key not in cache:
                mean=np.average(points[group],axis=0,weights=areas[group])
                _,_,axes=np.linalg.svd((points[group]-mean)*np.sqrt(areas[group,None]),full_matrices=False)
                alternatives=[]
                for axis_number,axis in enumerate(np.vstack([axes,np.eye(3)])):
                    if axis[np.argmax(abs(axis))]<0:axis=-axis
                    order=np.argsort((points[group]-mean)@axis,kind='stable')
                    cumulative=np.cumsum(areas[group[order]]);cumulative/=cumulative[-1]
                    for quantile in [.2,.35,.5,.65,.8]:
                        position=max(1,min(len(group)-1,int(np.searchsorted(cumulative,quantile))+1))
                        subsets=[group[order[:position]],group[order[position:]]]
                        if any(not len(x)for x in subsets):continue
                        children=[];solvers=[]
                        for subset in subsets:
                            child,solver=ball(subset);children.append(child);solvers.append(solver)
                        alternatives.append((sum(map(volume,children))-volume(balls[index]),subsets,children,
                            dict(axis=axis_number,quantile=quantile,enclosing_solvers=solvers)))
                cache[key]=min(alternatives,key=lambda item:item[0])
            option=cache[key];options.append((option[0],index,option))
        change,index,(_,subsets,children,record)=min(options,key=lambda item:item[0])
        history.append(dict(count=count+1,split_cluster=index,volume_change=change,**record))
        groups[index:index+1]=subsets;balls[index:index+1]=children
    return results


def point_surface_distance(points,triangles):
    values=[]
    a,b,c=triangles[:,0],triangles[:,1],triangles[:,2]
    ab,ac=b-a,c-a
    uu=(ab*ab).sum(axis=1);uv=(ab*ac).sum(axis=1);vv=(ac*ac).sum(axis=1)
    determinant=uu*vv-uv*uv
    normal=np.cross(ab,ac);normal2=(normal*normal).sum(axis=1)
    for start in range(0,len(points),128):
        p=points[start:start+128,None,:];ap=p-a
        d1=(ap*ab).sum(axis=2);d2=(ap*ac).sum(axis=2)
        s=np.divide(vv*d1-uv*d2,determinant,out=np.zeros_like(d1),where=abs(determinant)>1e-30)
        t=np.divide(uu*d2-uv*d1,determinant,out=np.zeros_like(d1),where=abs(determinant)>1e-30)
        inside=(s>=0)&(t>=0)&(s+t<=1)&(normal2>1e-30)
        plane=np.divide((ap*normal).sum(axis=2)**2,normal2,out=np.full_like(d1,np.inf),where=normal2>1e-30)
        d=np.where(inside,plane,np.inf)
        for first,last in [(a,b),(b,c),(c,a)]:
            delta=last-first;length2=(delta*delta).sum(axis=1)
            parameter=np.divide(((p-first)*delta).sum(axis=2),length2,out=np.zeros_like(d),where=length2>1e-30)
            nearest=first+np.clip(parameter,0,1)[...,None]*delta
            d=np.minimum(d,((p-nearest)**2).sum(axis=2))
        values.extend(np.sqrt(np.maximum(0,d.min(axis=1))))
    return np.array(values)


def point_inside(points,triangles):
    answers=[]
    for direction in [np.array([1.,.314159,.57721]),np.array([.173205,1.,.471405]),np.array([.367879,.223607,1.])]:
        a=triangles[:,0];e1=triangles[:,1]-a;e2=triangles[:,2]-a
        h=np.cross(direction,e2);det=(e1*h).sum(axis=1)
        inv=np.divide(1.,det,out=np.zeros_like(det),where=abs(det)>1e-15)
        counts=[]
        for start in range(0,len(points),128):
            delta=points[start:start+128,None,:]-a
            u=(delta*h).sum(axis=2)*inv
            cross=np.cross(delta,e1)
            v=(cross*direction).sum(axis=2)*inv
            along=(cross*e2).sum(axis=2)*inv
            hits=(abs(det)>1e-15)&(u>=0)&(v>=0)&(u+v<=1)&(along>1e-10)
            counts.extend(hits.sum(axis=1)%2==1)
        answers.append(counts)
    answers=np.asarray(answers)
    return answers.sum(axis=0)>=2,~np.all(answers==answers[0],axis=0)


def inside_finger_boxes(points,right=False):
    inside=np.zeros(len(points),dtype=bool)
    for center,size,angles in finger_boxes(right):
        local=Rotation.from_euler('xyz',angles).inv().apply(points-center)
        inside|=np.all(abs(local)<=np.array(size)/2+1e-12,axis=1)
    return inside,np.zeros(len(points),dtype=bool)


def audit_geometry(triangles,subtriangles,balls,finger=False,right=False):
    centers=np.array([b['center']for b in balls]);radii=np.array([b['radius']for b in balls])
    corner_gaps=np.linalg.norm(subtriangles[:,:,None,:]-centers[None,None],axis=3)-radii
    # A triangle lies in a convex sphere if all three vertices lie in it.
    face_gaps=corner_gaps.max(axis=1).min(axis=1)
    probes=np.vstack([triangles.reshape(-1,3),triangles.mean(axis=1)])
    gaps=(np.linalg.norm(probes[:,None]-centers[None],axis=2)-radii).min(axis=1)
    indices=np.arange(512);z=1-2*(indices+.5)/len(indices);angle=np.pi*(3-np.sqrt(5))*indices
    directions=np.c_[np.sqrt(1-z*z)*np.cos(angle),np.sqrt(1-z*z)*np.sin(angle),z]
    surface=(centers[:,None]+radii[:,None,None]*directions).reshape(-1,3)
    distance=point_surface_distance(surface,triangles)
    inside,ambiguous=inside_finger_boxes(surface,right)if finger else point_inside(surface,triangles)
    outward=np.where(inside,0,distance)
    vertices=np.unique(triangles.reshape(-1,3),axis=0);hull=ConvexHull(vertices)
    hull_overshoot=max(float(np.max(hull.equations[:,:3]@b['center']+hull.equations[:,3]+b['radius']))for b in balls)
    mesh_triangles=len(triangles)
    internal=qmc.Sobol(3,scramble=False).random_base2(12)*(vertices.max(0)-vertices.min(0))+vertices.min(0)
    internal_inside,internal_ambiguous=inside_finger_boxes(internal,right)if finger else point_inside(internal,triangles)
    hull_inside=np.all(internal@hull.equations[:,:3].T+hull.equations[:,3]<=1e-12,axis=1)
    _,vertex_indices=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
    faces=vertex_indices.reshape(-1,3)
    edges=np.sort(np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]),axis=1)
    _,edge_counts=np.unique(edges,axis=0,return_counts=True)
    internal_gaps=(np.linalg.norm(internal[:,None]-centers[None],axis=2)-radii).min(axis=1)
    return dict(spheres=len(balls),original_triangles=mesh_triangles,subtriangles=len(subtriangles),
        continuous_surface_coverage_certified=bool(np.all(face_gaps<=1e-10)),
        uncovered_subtriangles=int(np.count_nonzero(face_gaps>1e-10)),maximum_subtriangle_gap=float(face_gaps.max()),
        source_vertex_centroid_probes=len(probes),uncovered_probes=int(np.count_nonzero(gaps>1e-10)),maximum_probe_gap=float(gaps.max()),
        sphere_surface_probes=len(surface),sphere_surface_outward_max_m=float(outward.max()),
        sphere_surface_outward_p95_m=float(np.quantile(outward,.95)),sphere_surface_ray_ambiguous=int(ambiguous.sum()),
        exact_maximum_convex_hull_halfspace_violation_m=hull_overshoot,
        sphere_volume_sum=float(4*np.pi/3*np.sum(radii**3)),
        boundary_edge_incidence_two=bool(np.all(edge_counts==2)),boundary_non_two_incidence_edges=int(np.count_nonzero(edge_counts!=2)),
        solid_probe_scope='4096 deterministic bounding-box points; exact union-of-box inside test'if finger else '4096 deterministic bounding-box points; three-ray majority inside test, not continuous volume certificate',
        solid_inside_probes=int(internal_inside.sum()),
        solid_uncovered_inside_probes=int(np.count_nonzero(internal_inside&(internal_gaps>1e-10))),
        solid_maximum_inside_gap_m=None if not internal_inside.any()else float(internal_gaps[internal_inside].max()),
        solid_ray_ambiguous=int(internal_ambiguous.sum()),
        convex_hull_inside_probes=int(hull_inside.sum()),
        convex_hull_uncovered_inside_probes=int(np.count_nonzero(hull_inside&(internal_gaps>1e-10))),
        convex_hull_maximum_inside_gap_m=None if not hull_inside.any()else float(internal_gaps[hull_inside].max()))


def allocate(candidates,total):
    states={0:(0.,{})}
    for name,items in candidates.items():
        updated={}
        for used,(cost,allocation)in states.items():
            for k,item in items.items():
                if used+k>total:continue
                value=cost+item['sphere_volume_sum']
                if used+k not in updated or value<updated[used+k][0]:updated[used+k]=(value,dict(allocation,**{name:k}))
        states=updated
    return states[total]


def main():
    parser=argparse.ArgumentParser(__doc__);parser.add_argument('--report-dir',type=Path,default=REPORTS)
    parser.add_argument('--maximum-per-link',type=int,default=8);args=parser.parse_args()
    out=args.report_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    baseline=read(BASELINE);manifest=dict(status='running',started_unix=time.time(),builder_sha256=sha(__file__),
        original_asset_sha256=sha(BASELINE),selection='Minimum sum of sphere volumes among deterministic candidate library under exact33budget; no configuration or planning outcome used',
        max_subtriangle_edge_m=MAX_EDGE,coverage_buffer_m=BUFFER,maximum_spheres_per_link=args.maximum_per_link,
        global_continuous_solid_coverage_certified=False,attempted_links=[])
    atomic(out/'manifest.json',manifest);(out/'builder_snapshot.py').write_bytes(Path(__file__).read_bytes())
    atomic(out/'preflight.json',dict(trimesh_available=False,attempted_import='import trimesh',failure="ModuleNotFoundError: No module named 'trimesh'",installed_new_packages=False,geometry_libraries='existing NumPy/SciPy only; independent Drake audit belongs to separate script'))
    try:
        meshes={f'fr3_link{i}':stl(SOURCE/f'meshes/robots/fr3/collision/link{i}.stl')for i in range(8)}
        meshes['fr3_hand']=stl(SOURCE/'meshes/robot_ee/franka_hand_white/collision/hand.stl')
        meshes['fr3_leftfinger']=finger_triangles();meshes['fr3_rightfinger']=finger_triangles(True)
        candidates={};subdivided={}
        for name,triangles in meshes.items():
            dest=out/name;dest.mkdir();small=subdivide(triangles);subdivided[name]=small
            np.savez_compressed(dest/'geometry_inputs.npz',original_triangles=triangles,subtriangles=small)
            candidates[name]={}
            recursive=recursive_candidates(small,args.maximum_per_link)
            for count in range(1,args.maximum_per_link+1):
                attempts=[]
                for strategy in ['farthest','pca_quantile','recursive_volume_splits']:
                    tic=time.perf_counter();answer=recursive[count]if strategy=='recursive_volume_splits'else fit_triangles(small,count,strategy)
                    labels=answer.pop('labels');np.savez_compressed(dest/f'k{count:02d}_{strategy}_labels.npz',labels=labels)
                    answer['elapsed_seconds']=time.perf_counter()-tic
                    atomic(dest/f'k{count:02d}_{strategy}.json',answer);attempts.append(answer)
                winner=min(attempts,key=lambda item:item['sphere_volume_sum'])
                candidates[name][count]=winner
            manifest['attempted_links'].append(name);atomic(out/'manifest.json',manifest)
            print(name,'subtriangles',len(small),'candidate_volumes',[round(candidates[name][k]['sphere_volume_sum'],7)for k in candidates[name]],flush=True)
        cost,allocation=allocate(candidates,TOTAL)
        selected={name:candidates[name][count]['spheres']for name,count in allocation.items()}
        baseline_audits={};new_audits={}
        for name,triangles in meshes.items():
            baseline_audits[name]=audit_geometry(triangles,subdivided[name],baseline['spheres'][name],name.endswith('finger'),name=='fr3_rightfinger')
            new_audits[name]=audit_geometry(triangles,subdivided[name],selected[name],name.endswith('finger'),name=='fr3_rightfinger')
            atomic(out/'geometry_audit.json',dict(baseline=baseline_audits,selected=new_audits))
        assert sum(map(len,selected.values()))==TOTAL
        assert all(a['continuous_surface_coverage_certified']for a in new_audits.values())
        asset=copy.deepcopy(baseline)
        asset.update(fidelity='user_authorized_geometry_constructed_33_spheres_not_original_author_asset',spheres=selected,
            fitting=manifest['selection'],continuous_mesh_cover_certified=False,
            continuous_surface_coverage_certified=True,continuous_solid_coverage_certified=False,
            sphere_allocation=allocation,mesh_probe_audit=[dict(link=name,**audit)for name,audit in new_audits.items()],
            construction_provenance=dict(builder_sha256=manifest['builder_sha256'],baseline_asset_sha256=sha(BASELINE),
                report_directory=str(out.relative_to(ROOT)),max_subtriangle_edge_m=MAX_EDGE,coverage_buffer_m=BUFFER,
                counts_selected_by='exact discrete dynamic programming over saved per-link candidates',
                continuous_surface_certificate='Every original source triangle is partitioned into subtriangles; all three vertices of each subtriangle lie in one convex sphere. Checked at serialized float64 centers/radii.',
                sphere_volume_upper_bound_m3=cost,planning_outcomes_used=False,home_or_q7_used_for_selection=False))
        atomic(out/'selected_asset.json',asset);atomic(ASSETS/'fr3_33.json',asset)
        manifest.update(status='completed',allocation=allocation,sphere_volume_sum=cost,
            baseline_sphere_volume_sum=sum(x['sphere_volume_sum']for x in baseline_audits.values()),
            final_asset_sha256=sha(ASSETS/'fr3_33.json'),elapsed_seconds=time.time()-manifest['started_unix'])
        atomic(out/'manifest.json',manifest);print('SELECTED',allocation,'volume',cost,flush=True)
    except BaseException as error:
        manifest.update(status='failed',error=repr(error),traceback=traceback.format_exc())
        atomic(out/'manifest.json',manifest);raise


if __name__=='__main__':main()
