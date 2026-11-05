"""Native DRM construction with recorded PCG64 nodes and canonical geometry."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import subprocess
import time
import traceback
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


def roadmap_identity(nodes, seed, config, robot_sha256, collision_geometry_sha256):
    """Different graph/cache/model protocols must not share a roadmap ID."""
    payload = dict(config=config, robot_sha256=robot_sha256,
                   collision_geometry_sha256=collision_geometry_sha256)
    digest = hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':'),
                                      allow_nan=False).encode()).hexdigest()
    return f'kinova7-full-n{nodes}-s{seed}-{digest[:12]}', digest


def load_config(config=None):
    answer = json.loads((HERE / 'configs/evaluation.json').read_text())
    # Preserve the installed native default for existing protocols, while
    # permitting an explicitly recorded author-source iteration cap.
    answer['eizo'].setdefault('max_iterations', 500)
    if config is not None:
        supplied = json.loads(Path(config).read_text()) if isinstance(config, (str, Path)) else config
        if not isinstance(supplied, dict):
            raise ValueError('Evaluation configuration must be an object')
        for key, value in supplied.items():
            if key in ('ompl', 'extended_baselines'):
                if not isinstance(value, dict):
                    raise ValueError(f'{key} must be an object')
                if key == 'extended_baselines' and set(value) - {'chomp', 'stomp', 'trajopt'}:
                    raise ValueError('Unknown extended baseline configuration')
                answer[key] = json.loads(json.dumps(value, allow_nan=False))
                continue
            # Optional backends are explicit opt-ins. Do not add defaults to
            # historical configurations, whose resolved values are frozen.
            if key == 'lgcs':
                if (not isinstance(value, dict) or set(value) != {'solver'} or
                        value['solver'] not in ('gurobi', 'scip')):
                    raise ValueError('lgcs must specify solver gurobi or scip')
                answer[key] = dict(value)
                continue
            if key == 'drm_query':
                if (not isinstance(value, dict) or set(value) != {'backend', 'build_dir'} or
                        value['backend'] != 'safe_native' or
                        not isinstance(value['build_dir'], str) or not value['build_dir'].strip()):
                    raise ValueError('drm_query requires safe_native and an explicit build_dir')
                answer[key] = dict(value)
                continue
            if key == 'nto_refined':
                if (not isinstance(value, dict)
                        or not value or set(value) - {'minimum_time_control_points', 'early_accept_validated'}):
                    raise ValueError('nto_refined must contain supported settings')
                if 'minimum_time_control_points' in value:
                    count = value['minimum_time_control_points']
                    if (isinstance(count, (bool, np.bool_))
                            or not isinstance(count, (int, np.integer)) or count < 4):
                        raise ValueError('nto_refined.minimum_time_control_points must be an integer >= 4')
                if 'early_accept_validated' in value and not isinstance(value['early_accept_validated'], bool):
                    raise ValueError('nto_refined.early_accept_validated must be boolean')
                answer[key] = dict(value)
                continue
            if key == 'ipopt':
                from .runners.ipopt_options import settings
                answer[key] = settings({'ipopt': value})
                continue
            if key not in answer:
                raise ValueError(f'Unknown evaluation setting {key}')
            if isinstance(answer[key], dict):
                if not isinstance(value, dict):
                    raise ValueError(f'Evaluation setting {key} must be an object')
                unknown = set(value) - set(answer[key])
                if unknown:
                    raise ValueError(f'Unknown {key} settings: {sorted(unknown)}')
                answer[key].update(value)
            else:
                answer[key] = value
    # Reject malformed values before native code can allocate, loop or silently
    # accept NaN comparisons. These are schema bounds, not tuned parameters.
    def numeric(value, name, integer=False, zero=False):
        if (isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.integer, np.floating))
                or not np.isfinite(value) or value < 0 or (not zero and value == 0)
                or (integer and not isinstance(value, (int, np.integer)))):
            raise ValueError(f'Invalid finite {"integer " if integer else ""}evaluation setting {name}')
    integer_keys = {'validation_minimum_samples', 'validation_maximum_samples', 'bezier_degree',
                    'max_recovery_attempts', 'maximum_recovery_witnesses'}
    for key, value in answer.items():
        if key == 'protocol':
            if value != 'kinova7_generic_v1_evaluation':
                raise ValueError('Unsupported evaluation protocol')
        elif not isinstance(value, dict):
            numeric(value, key, key in integer_keys, key == 'max_recovery_attempts')
    for section in ('drm', 'eizo', 'roadmap', 'nto'):
        for key, value in answer[section].items():
            name = section + '.' + key
            if key == 'try_shortcutting':
                if not isinstance(value, bool):
                    raise ValueError('try_shortcutting must be boolean')
            elif key == 'solver':
                if value != 'IPOPT':
                    raise ValueError('Unsupported NTO solver; this adapter implements IPOPT only')
            elif key in ('map_center', 'map_extent'):
                array = np.asarray(value, dtype=float)
                if array.shape != (3,) or not np.isfinite(array).all() or (key == 'map_extent' and np.any(array <= 0)):
                    raise ValueError(f'Invalid evaluation setting {name}')
            else:
                integer = key in {'max_iterations_steering_to_node', 'max_nodes_to_expand',
                    'max_number_planning_attempts', 'num_particles', 'bisection_steps', 'mixing_steps',
                    'max_hyperplanes_per_iteration', 'max_iterations', 'max_neighbors', 'collocation_points',
                    'minimum_control_points', 'maximum_iterations'}
                numeric(value, name, integer, key == 'voxel_padding')
    if (answer['validation_minimum_samples'] < 2 or
            answer['validation_minimum_samples'] > answer['validation_maximum_samples'] or
            answer['bezier_degree'] < 2 or answer['nto']['minimum_control_points'] < 4 or
            answer['optimizer_budget_seconds'] > answer['online_budget_seconds']):
        raise ValueError('Inconsistent sample, spline or total/optimizer budget configuration')
    for key in ('delta', 'epsilon', 'tau'):
        if not 0 < answer['eizo'][key] < 1:
            raise ValueError(f'eizo.{key} must lie strictly between zero and one')
    if 'ompl' in answer:
        from .runners.ompl_baselines import settings
        settings(answer)
    if 'extended_baselines' in answer:
        from .runners.chomp_stomp import _profile
        for method in ('CHOMP', 'STOMP'):
            if method.lower() in answer['extended_baselines']:
                _profile(method, answer)
        if 'trajopt' in answer['extended_baselines']:
            from .runners.trajopt_baseline import settings
            settings(answer)
    # Materialize the effective native linear-system policy in every new run.
    # Read-only historical summaries use their stored configuration directly.
    from .runners.ipopt_options import settings as ipopt_settings
    answer['ipopt'] = ipopt_settings(answer)
    return answer


def build_roadmap(out, nodes=6000, seed=0, config=None):
    """Append-only build with an inspectable failure receipt, including CPU errors."""
    out=Path(out)
    if out.exists() and (not out.is_dir() or any(out.iterdir())):
        raise FileExistsError(f'Will not overwrite existing roadmap {out}')
    out.mkdir(parents=True,exist_ok=True)
    began=time.perf_counter()
    try:
        if isinstance(nodes,bool) or not isinstance(nodes,(int,np.integer)) or nodes<2:
            raise ValueError('Roadmap needs an integer count of at least two nodes')
        if isinstance(seed,(bool,np.bool_)) or not isinstance(seed,(int,np.integer)) or not 0<=seed<2**32:
            raise ValueError('Roadmap seed must be an unsigned 32-bit integer')
        cfg=load_config(config)
        return _build_roadmap(out,int(nodes),int(seed),cfg)
    except Exception as exc:
        meta=json.loads((out/'metadata.json').read_text()) if (out/'metadata.json').is_file() else {}
        meta.update(status='failed',exception=repr(exc),requested_nodes=nodes,seed=seed,
                    offline_seconds=time.perf_counter()-began,
                    timing_scope='Builder call wall time; process startup is external',
                    failed_stage=meta.get('current_stage','input_or_model_setup'))
        (out/'exception.txt').write_text(traceback.format_exc())
        write(out/'metadata.json',meta)
        raise


def _build_roadmap(out, nodes=6000, seed=0, config=None):
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
    identity, protocol_hash = roadmap_identity(nodes, seed, cfg, model.robot_sha256,
                                               model.collision_geometry_sha256)
    meta = dict(status='building', roadmap_id=identity, roadmap_protocol_sha256=protocol_hash,
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
    meta['current_stage']='sampling';write(out/'metadata.json',meta)
    try:
        while count < nodes:
            q = rng.uniform(model.lower + .001,model.upper - .001,(min(4096,max(256,2*(nodes-count))),7)).astype(np.float32)
            proposals.append(q)
            free = np.asarray(model.cuda(q))
            if free.shape!=(len(q),) or free.dtype!=np.bool_:
                raise ValueError('Offline collision checker returned malformed sample classifications')
            batch = q[free][:nodes-count]
            accepted.append(batch)
            count += len(batch)
            if sum(map(len,proposals)) > nodes * 100:
                raise RuntimeError('Offline sampling exhausted the fixed draw cap')
    except Exception:
        np.savez_compressed(out/'node_sampling_partial.npz',
            proposals=np.vstack(proposals) if proposals else np.empty((0,7)),
            accepted=np.vstack(accepted) if accepted else np.empty((0,7)))
        raise
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
                timing_scope='Native builder including model/CUDA initialization and serialization; Python process startup is external',
                offline_seconds=time.perf_counter()-began,native_map_sha256=sha(out/'native.map'),
                node_sampling_sha256=sha(out/'node_sampling.npz'),
                map_voxel_count=int(np.prod(np.ceil(np.asarray(cfg['map_extent'])/(2*cfg['offline_voxel_resolution'])))),
                native_map_bytes=(out/'native.map').stat().st_size)
    write(out/'metadata.json',meta)
    return meta
