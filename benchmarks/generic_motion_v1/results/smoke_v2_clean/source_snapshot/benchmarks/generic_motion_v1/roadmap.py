"""Native DRM construction with recorded PCG64 nodes and canonical geometry."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import subprocess
import time
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def clean(value):
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, np.generic):
        return clean(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean(v) for v in value]
    return value


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(clean(value), indent=2, sort_keys=True, allow_nan=False) + '\n')
    temp.replace(path)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_config(config=None):
    answer = json.loads((HERE / 'configs/evaluation.json').read_text())
    if config:
        supplied = json.loads(Path(config).read_text()) if isinstance(config, (str, Path)) else config
        for key, value in supplied.items():
            if key not in answer:
                raise ValueError(f'Unknown evaluation setting {key}')
            if isinstance(value, dict):
                unknown = set(value) - set(answer[key])
                if unknown:
                    raise ValueError(f'Unknown {key} settings: {sorted(unknown)}')
                answer[key].update(value)
            else:
                answer[key] = value
    return answer


def build_roadmap(out, nodes=6000, seed=0, config=None):
    """Build and persist exactly ``nodes`` accepted offline configurations.

    Node random draws are reproducible. Native graph serialization may differ
    with C++ unordered-map ordering; its actual bytes are always hashed.
    """
    from .model import Model
    import csdecomp as csd
    out = Path(out)
    if (out / 'metadata.json').exists():
        raise FileExistsError(f'Will not overwrite existing roadmap {out}')
    out.mkdir(parents=True, exist_ok=True)
    if not isinstance(nodes, int) or nodes < 2:
        raise ValueError('Roadmap needs at least two accepted nodes')
    cfg = load_config(config)['roadmap']
    began = time.perf_counter()
    model = Model()
    meta = dict(status='building', roadmap_id=f'kinova7-full-n{nodes}-s{seed}',
                requested_nodes=nodes, seed=int(seed), rng='numpy.random.PCG64',
                source_code_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                builder_sha256=sha(__file__), model_sha256=sha(HERE/'model.py'),
                robot_sha256=model.robot_sha256, collision_geometry_sha256=model.collision_geometry_sha256,
                config=cfg, timing={}, online_obstacles=0,
                native_sampling='not used: add_nodes_manual receives saved PCG64 samples',
                collision_map_semantics='Native discretized node-pruning cache; finite-radius online obstacles are rechecked by native lazy edge tests. Cache is not a continuous collision certificate.')
    write(out/'metadata.json',meta)
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([1709301,int(seed)])))
    accepted, proposals = [], []
    count = 0
    tic = time.perf_counter()
    while count < nodes:
        q = rng.uniform(model.lower + .001,model.upper - .001,(min(4096,max(256,2*(nodes-count))),7)).astype(np.float32)
        free = model.cuda(q)
        proposals.append(q)
        batch = q[free][:nodes-count]
        accepted.append(batch)
        count += len(batch)
        if sum(map(len,proposals)) > nodes * 100:
            raise RuntimeError('Offline sampling exhausted the fixed draw cap')
    sample = np.vstack(accepted)
    np.savez_compressed(out/'node_sampling.npz',accepted=sample,proposals=np.vstack(proposals))
    meta['timing']['sampling_seconds']=time.perf_counter()-tic
    meta['proposal_count']=sum(map(len,proposals))
    options=csd.RoadmapOptions()
    options.map_center=np.asarray(cfg['map_center'])
    options.robot_map_size_x,options.robot_map_size_y,options.robot_map_size_z=cfg['map_extent']
    for name in ('offline_voxel_resolution','max_task_space_distance_between_nodes','max_configuration_distance_between_nodes','edge_step_size'):
        setattr(options,name,cfg[name])
    options.nodes_processed_before_debug_statement=10000
    builder=csd.RoadmapBuilder(model.csd_plant,model.tcp_name,options)
    for name,operation in [('add_nodes',lambda:builder.add_nodes_manual(sample.T)),
                           ('graph',lambda:builder.build_roadmap(cfg['max_neighbors'])),
                           ('pose_map',builder.build_pose_map),('collision_map',builder.build_collision_map)]:
        meta['current_stage']=name
        write(out/'metadata.json',meta)
        tic=time.perf_counter(); operation()
        meta['timing'][name+'_seconds']=time.perf_counter()-tic
        # A pre-cache graph remains inspectable if collision-map construction fails.
        if name=='pose_map':
            builder.write(str(out/'pre_collision_map.map'))
        write(out/'metadata.json',meta)
    builder.write(str(out/'native.map'))
    drm=builder.get_drm()
    ids=sorted(drm.id_to_node_map)
    edges=np.asarray([(i,j) for i,adj in drm.node_adjacency_map.items() for j in adj],dtype=int).reshape(-1,2)
    np.savez_compressed(out/'roadmap.npz',node_ids=ids,nodes=np.asarray([drm.id_to_node_map[i] for i in ids]),edges=edges)
    meta.update(status='ready',actual_nodes=len(ids),directed_edges=len(edges),
                offline_seconds=time.perf_counter()-began,native_map_sha256=sha(out/'native.map'),
                node_sampling_sha256=sha(out/'node_sampling.npz'),
                map_voxel_count=int(np.prod(np.ceil(np.asarray(cfg['map_extent'])/(2*cfg['offline_voxel_resolution'])))),
                native_map_bytes=(out/'native.map').stat().st_size)
    write(out/'metadata.json',meta)
    return meta
