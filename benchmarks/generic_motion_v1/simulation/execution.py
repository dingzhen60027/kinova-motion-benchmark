"""Torque-controlled rigid-body execution, distinct from saved-state playback."""
from __future__ import annotations

import os
os.environ.setdefault('MUJOCO_GL', 'egl')

import hashlib
import json
from pathlib import Path
import time
import traceback

import numpy as np

from ..common import ROOT, read, write, sha, head


def configuration(path=None):
    cfg = read(ROOT / 'configs/mujoco.json')
    if path is not None:
        extra = dict(path) if isinstance(path, dict) else read(path)
        if set(extra) - set(cfg):
            raise ValueError('Unknown simulation configuration fields')
        cfg.update(extra)
    if cfg['controller'] != 'computed_torque_pd':
        raise ValueError('Unsupported controller')
    for key, value in cfg.items():
        if key in ('controller', 'gravity_m_s2'):
            continue
        if isinstance(value, bool) or not np.isfinite(value) or value < 0:
            raise ValueError('Invalid simulation setting: ' + key)
    if (not 0 < cfg['timestep_s'] <= .01 or cfg['kp_s2'] <= 0 or cfg['kd_s'] <= 0
            or int(cfg['maximum_steps']) != cfg['maximum_steps'] or cfg['maximum_steps'] < 1):
        raise ValueError('Invalid integration/controller settings')
    gravity = np.asarray(cfg['gravity_m_s2'], float)
    if gravity.shape != (3,) or not np.isfinite(gravity).all():
        raise ValueError('Gravity must be a finite three-vector')
    return cfg


def source_hashes():
    files = list(Path(__file__).parent.glob('*.py')) + [ROOT/'scripts/simulate.py']
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(files) if p.exists()}


def checked_attempt(attempt):
    """Check original immutable evidence without changing it or rerunning CUDA."""
    attempt = Path(attempt).resolve()
    receipt = read(attempt/'receipt.json')
    if not isinstance(receipt.get('files'),dict) or not {'input.json','result.json'} <= set(receipt['files']):
        raise ValueError('Source receipt must bind input.json and result.json')
    from ..batch import _attempt_receipt
    _attempt_receipt(attempt)
    if receipt.get('retry_pending'):
        raise ValueError('Source attempt is not a terminal outcome')
    for name, digest in receipt['files'].items():
        file = (attempt/name).resolve()
        if not file.is_relative_to(attempt) or not file.is_file() or sha(file) != digest:
            raise ValueError('Corrupted source attempt: ' + name)
    case = read(attempt/'input.json')
    result = read(attempt/'result.json')
    if receipt['status'] != result.get('status'):
        raise ValueError('Source receipt/result status differs')
    request = read(attempt/'request.json') if (attempt/'request.json').exists() else {}
    case_id = result.get('problem_id',result.get('case_id',request.get('case_id')))
    if case.get('problem_id') != case_id:
        raise ValueError('Source case/result identity differs')
    result = dict(result)
    for key in ('method','objective'):
        if key not in result and key in request:
            if 'request.json' not in receipt['files']:
                raise ValueError('Unsealed source request')
            result[key]=request[key]
    if result.get('status')=='success_validated_numerical' and result.get('objective')=='minimum_time':
        if not {'candidate.json','candidate.npz','validation.json'} <= set(receipt['files']):
            raise ValueError('Source receipt must bind the validated reference')
    return case, result, receipt


def _torque(mujoco, model, data, ref, ids, cfg, disabled):
    qi, vi = ids
    q, v, a = ref
    desired = np.zeros(model.nv)
    desired[vi] = a + cfg['kp_s2']*(q-data.qpos[qi]) + cfg['kd_s']*(v-data.qvel[vi])
    mass = np.empty((model.nv, model.nv))
    mujoco.mj_fullM(model, mass, data.qM)
    tau = mass@desired + data.qfrc_bias-data.qfrc_passive
    return np.zeros(7) if disabled else tau[vi]


def _rollout(metadata, reference, case, cfg, output, disabled=False):
    import mujoco
    model = mujoco.MjModel.from_xml_path(metadata['xml_path'])
    data = mujoco.MjData(model)
    qi = np.asarray(metadata['qpos_ids'], int)
    vi = np.asarray(metadata['dof_ids'], int)
    ui = np.asarray(metadata['actuator_ids'], int)
    effort = np.asarray(metadata['effort_limits_nm'], float)
    if model.nv != 7 or model.nq != 7 or model.nu != 7 or np.any(effort <= 0):
        raise ValueError('Execution requires seven independent torque actuators')
    dt = float(model.opt.timestep)
    steps = int(np.ceil((reference.duration+cfg['settle_seconds'])/dt))
    if steps > cfg['maximum_steps']:
        raise ValueError('Physical horizon exceeds declared step cap')
    q0, v0, _ = reference.at(0.)
    if not np.allclose(q0, case['q_start'], atol=1e-8, rtol=0):
        raise ValueError('Reference differs from frozen start')
    if not np.allclose(reference.at(reference.duration)[0], case['q_goal'], atol=1e-8, rtol=0):
        raise ValueError('Reference differs from frozen goal')
    # The only state assignment in the dynamics rollout is this initialization.
    data.qpos[qi] = q0
    data.qvel[vi] = v0
    mujoco.mj_forward(model, data)
    arrays = {name: [] for name in ('time','qpos','qvel','qacc','qref','qvel_ref','qacc_ref',
                                   'torque','torque_requested','actual_torque','actuator_force',
                                   'contacts_count','minimum_contact_distance_m','tcp')}
    contacts = []
    start = time.perf_counter()
    exception = None
    try:
        for step in range(steps+1):
            if not np.isclose(data.time,step*dt,rtol=0,atol=1e-8):
                raise FloatingPointError('Physical clock reset or drift')
            # Both contact geometry and state belong to the same data.time.
            mujoco.mj_step1(model, data)
            target = reference.at(float(data.time))
            requested = _torque(mujoco, model, data, target, (qi,vi), cfg, disabled)
            command = np.clip(requested, -effort, effort)
            data.ctrl[ui] = command
            # Refresh acceleration with this control without advancing state.
            mujoco.mj_forward(model, data)
            warnings = [int(w.number) for w in data.warning]
            if any(warnings):
                raise FloatingPointError('MuJoCo integration warnings: '+str(warnings))
            if not np.isfinite(np.r_[data.qpos,data.qvel,data.qacc,command]).all():
                raise FloatingPointError('Nonfinite physical state')
            distances = [float(data.contact[i].dist) for i in range(data.ncon)]
            penetrations = [i for i,d in enumerate(distances) if d < -cfg['penetration_tolerance_m']]
            for i in penetrations:
                c = data.contact[i]
                if len(contacts) < 100:
                    contacts.append(dict(time=float(data.time), geom1=int(c.geom1), geom2=int(c.geom2), distance_m=float(c.dist)))
            values = dict(time=float(data.time), qpos=data.qpos.copy(), qvel=data.qvel.copy(),
                          qacc=data.qacc.copy(), qref=target[0], qvel_ref=target[1], qacc_ref=target[2],
                          torque=command.copy(), torque_requested=requested.copy(), contacts_count=len(penetrations),
                          actual_torque=data.qfrc_actuator[vi].copy(), actuator_force=data.actuator_force[ui].copy(),
                          minimum_contact_distance_m=min(distances) if distances else np.nan,
                          tcp=data.xpos[metadata['tcp_body_id']].copy())
            for key, value in values.items():
                arrays[key].append(value)
            if step < steps:
                mujoco.mj_step2(model, data)
    except Exception:
        exception = traceback.format_exc()
        (output/'integration_failure.log').write_text(exception)
    arrays = {k: np.asarray(v) for k,v in arrays.items()}
    np.savez_compressed(output/'rollout.npz', **arrays)
    if not len(arrays['time']):
        raise RuntimeError('Integration produced no states: '+str(exception))
    integration_seconds = time.perf_counter()-start
    from ..model import Model
    independent = Model(case.get('voxels_xyz',[]),case.get('voxel_radius_m',.04))
    clearance = np.asarray(independent.clearance(arrays['qpos'][:,qi]),float)
    if clearance.shape != arrays['time'].shape or not np.isfinite(clearance).all():
        raise ValueError('Independent actual-state collision check failed')
    arrays['independent_clearance_m'] = clearance
    np.savez_compressed(output/'rollout.npz', **arrays)
    band = cfg['penetration_tolerance_m']
    disagreement = ((clearance < -band) != (arrays['contacts_count']>0)) & (np.abs(clearance)>band)
    error = arrays['qpos'][:,qi]-arrays['qref']
    nominal = int(np.argmin(np.abs(arrays['time']-reference.duration)))
    velocity = np.max(np.abs(arrays['qvel'][:,vi]), axis=0)
    acceleration = np.max(np.abs(arrays['qacc'][:,vi]), axis=0)
    lower, upper = np.asarray(metadata['lower']),np.asarray(metadata['upper'])
    position_excess = float(max(0., np.max(lower-arrays['qpos'][:,qi]),np.max(arrays['qpos'][:,qi]-upper)))
    velocity_excess = float(max(0.,np.max(velocity-np.asarray(case['velocity_limits_rad_s']))))
    acceleration_excess = float(max(0.,np.max(acceleration-np.asarray(case['acceleration_limits_rad_s2']))))
    endpoint_error = float(np.max(np.abs(arrays['qpos'][-1,qi]-case['q_goal'])))
    tracking_error = float(np.max(np.abs(error)))
    checks = dict(integration_complete=exception is None and len(arrays['time'])==steps+1,
                  contact_free=not bool(np.any(arrays['contacts_count'])),
                  independent_collision_free=bool(np.all(clearance>band)),
                  collision_checker_agreement=not bool(np.any(disagreement)),
                  tracking=tracking_error<=cfg['tracking_tolerance_rad'], endpoint=endpoint_error<=cfg['endpoint_tolerance_rad'],
                  joint_limits=position_excess<=cfg['position_tolerance_rad'],
                  actual_effort_limits=bool(np.all(np.abs(arrays['actual_torque'])<=effort+1e-9)),
                  velocity_limits=velocity_excess<=cfg['velocity_tolerance_rad_s'],
                  acceleration_limits=acceleration_excess<=cfg['acceleration_tolerance_rad_s2'])
    return dict(status='execution_success_numerical' if all(checks.values()) else 'execution_failed', checks=checks,
                controller_disabled=disabled, controller=cfg['controller'],
                steps=len(arrays['time'])-1, physical_duration_s=float(arrays['time'][-1]),
                reference_duration_s=reference.duration, wall_seconds=time.perf_counter()-start,
                integration_wall_seconds=integration_seconds,
                independent_validation_seconds=time.perf_counter()-start-integration_seconds,
                minimum_independent_clearance_m=float(np.min(clearance)),
                collision_checker_disagreement_samples=int(np.count_nonzero(disagreement)),
                tracking_rmse_rad=float(np.sqrt(np.mean(error**2))), maximum_tracking_error_rad=tracking_error,
                nominal_endpoint_error_rad=float(np.max(np.abs(arrays['qpos'][nominal,qi]-case['q_goal']))),
                settled_endpoint_error_rad=endpoint_error, joint_limit_excess_rad=position_excess,
                maximum_velocity_rad_s=velocity.tolist(), maximum_acceleration_rad_s2=acceleration.tolist(),
                velocity_excess_rad_s=velocity_excess, acceleration_excess_rad_s2=acceleration_excess,
                contact_steps=int(np.count_nonzero(arrays['contacts_count'])), first_penetrations=contacts,
                saturation_steps=int(np.count_nonzero(np.any(np.abs(arrays['torque_requested'])>effort+1e-10,axis=1))),
                peak_commanded_torque_nm=np.max(np.abs(arrays['torque']),axis=0).tolist(),
                peak_actual_torque_nm=np.max(np.abs(arrays['actual_torque']),axis=0).tolist(),
                integration_warning_counts=[int(w.number) for w in data.warning],
                validation_scope='Finite per-integration-step rigid-body states, canonical Drake clearance recheck; no hardware calibration or continuous collision certificate.',
                assumptions='Exact-model computed-torque feedback, ideal torque motors, original asset effort limits and inertia, fixed gripper, soft contact.')


def run(attempt, output, config=None, render=False, disable_control=False):
    from .reference import load_reference
    from .mujoco_model import build
    attempt, output = Path(attempt).resolve(),Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    result = dict(status='infrastructure_error', source_attempt=str(attempt), source_commit=head(),
                  simulation_source_hashes=source_hashes())
    try:
        cfg = configuration(config)
        write(output/'config.json', cfg)
        result['source_receipt_sha256'] = sha(attempt/'receipt.json')
        case, original, receipt = checked_attempt(attempt)
        result.update(problem_id=case['problem_id'], scene_id=case['scene_id'], family=case['family'],
                      method=original['method'], original_planning_status=original['status'],
                      source_input_sha256=sha(attempt/'input.json'), source_trajectory_is_unchanged=True)
        if case.get('validation_status') != 'valid_input':
            result['status'] = 'invalid_input'
        elif original['status'] != 'success_validated_numerical':
            result['status'] = 'no_validated_reference'
        elif original['objective'] != 'minimum_time':
            result['status'] = 'unsupported_untimed_path'
        else:
            validation = read(attempt/'validation.json')
            if not validation['passed']:
                raise ValueError('Original output lacks passing numerical validation')
            reference = load_reference(attempt)
            write(output/'reference.json', reference.metadata)
            metadata = build(case, output/'model', cfg)
            result.update(_rollout(metadata,reference,case,cfg,output,disable_control))
            result['model_xml_sha256'] = sha(output/'model/model.xml')
            if render:
                from .render import render_rollout
                result['render'] = render_rollout(output/'model/model.xml',output/'rollout.npz',output/'render')
    except Exception:
        result.update(status='infrastructure_error',error=traceback.format_exc())
        (output/'failure.log').write_text(result['error'])
    result['artifacts'] = {str(p.relative_to(output)):sha(p) for p in sorted(output.rglob('*'))
                           if p.is_file() and p.name!='result.json'}
    write(output/'result.json',result)
    write(output/'receipt.json',dict(result_sha256=sha(output/'result.json')))
    return result
