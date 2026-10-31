"""Audit and report an actual 35,000-node/15-trial constructed hardware run.

Only saved records and arrays are read. No kinematics, GPU work, planning or
optimization is invoked. Accounting completeness does not imply trial success.
"""
import argparse
from collections import Counter
import hashlib
import math
from pathlib import Path
import numpy as np
from reproduction.common import ROOT, atomic, read
from reproduction.package_completion import describe

DEFAULT_OUTPUT=ROOT/'reproduction/reports/paper_constructed/hardware_simulation'
METRICS=['synthetic_surface_generation_seconds','voxelization_seconds','collision_set_seconds','ik_seconds','drm_seconds',
         'construction_seconds','optimization_seconds','validation_seconds','exact_scene_validation_seconds',
         'planning_component_seconds','wall_seconds','duration','regions','voxels']
TIME_LABELS={
    'synthetic_surface_generation_seconds':'Synthetic surface generation (not perception)',
    'voxelization_seconds':'Virtual point-cloud voxelization','collision_set_seconds':'Native collision-set construction',
    'ik_seconds':'Online IK','drm_seconds':'DRM path query','construction_seconds':'EI-ZO / safe-set construction and recovery',
    'optimization_seconds':'SCSTO','validation_seconds':'Pipeline numerical validation',
    'exact_scene_validation_seconds':'Independent exact scene-box validation',
    'planning_component_seconds':'Sum of recorded planning components','wall_seconds':'Whole trial wall time',
}


class Evidence:
    def __init__(self):self.files={}
    def record(self,path):
        path=Path(path);self.files[str(path.resolve())]=describe(path)
    def json(self,path,required=True):
        path=Path(path)
        if not path.exists():
            if required:raise FileNotFoundError(path)
            return None
        self.record(path);return read(path)
    def arrays(self,path):
        path=Path(path)
        if not path.exists():return None
        self.record(path)
        with np.load(path,allow_pickle=False)as data:return {key:data[key]for key in data.files}


def distribution(rows,key):
    values=[float(row[key])for row in rows if row.get(key)is not None]
    if not all(math.isfinite(value)for value in values):raise ValueError('Nonfinite recorded metric: '+key)
    return dict(recorded=len(values),mean=float(np.mean(values))if values else None,
                sample_sd=float(np.std(values,ddof=1))if len(values)>1 else None)


def equal_number(a,b):
    if a is None or b is None:return a is b
    return bool(np.isclose(float(a),float(b),rtol=1e-10,atol=1e-12))


def audit_run(root,evidence,require_full=True):
    root=Path(root).resolve();m=evidence.json(root/'manifest.json');cfg=m['config'];indices=m['trials'];issues=[]
    if m.get('status')in {'running','pending','preparing'}:raise ValueError('Run must be terminal before reporting')
    full=cfg.get('roadmap_nodes')==35000 and indices==list(range(15))
    if require_full and not full:raise ValueError('The formal report requires the declared 35,000-node / all-15-trial run; use --smoke-run for a smaller supplement')
    if m.get('physical_experiment')is not False:issues.append('manifest must explicitly identify physical_experiment=false')
    preflight=evidence.json(root/'model_check/model_check.json',required=False)
    roadmap=evidence.json(root/'roadmap/roadmap.json',required=False)
    saved_summary=evidence.json(root/'summary.json',required=False)
    targets=evidence.json(root/'fixed_targets.json',required=False)
    source_manifest=evidence.json(root/'source_snapshot/reproduction/hardware_constructed/assets/source_manifest.json',required=False)
    if preflight and preflight.get('status')=='passed':
        if not(preflight.get('declared_initial_free')and preflight.get('declared_left_anchor_free')and preflight.get('declared_right_anchor_free')and preflight.get('declared_anchor_limits_valid',True)and preflight.get('all_layout_anchors_valid')):
            issues.append('preflight claims passed but fixed task anchors are not valid')
        if preflight.get('disagreements')!=0 or preflight.get('online_disagreements')!=0:
            issues.append('preflight claims passed with predicate disagreements')
    if preflight and preflight.get('status')=='passed':
        expected_anchors=np.asarray([cfg['initial_right_configuration'],cfg['left_target_generating_configuration'],cfg['right_target_generating_configuration']])
        checks=preflight.get('all_layout_anchor_checks',[])
        if [x.get('index')for x in checks]!=list(range(15)):issues.append('preflight does not record all 15 fixed layouts')
        for index in range(15):
            arrays=evidence.arrays(root/'model_check'/f'anchor_layout_{index:02d}.npz')
            if arrays is None:issues.append(f'preflight layout {index}: missing raw anchor predicates');continue
            if not np.array_equal(arrays.get('q'),expected_anchors):issues.append(f'preflight layout {index}: wrong anchor configurations')
            if not all(np.asarray(arrays.get(key)).all()for key in ['cuda','independent_voxel','exact_solid']):issues.append(f'preflight layout {index}: nonfree anchor')
            margin=preflight['required_anchor_clearance']
            if min(float(np.min(arrays[key]))for key in ['voxel_minimum_signed_distance','solid_minimum_signed_distance'])<margin:issues.append(f'preflight layout {index}: insufficient recorded anchor clearance')
    right_is_initial=np.array_equal(cfg['right_target_generating_configuration'],cfg['initial_right_configuration'])
    if not right_is_initial:issues.append('right target anchor differs from checked initial anchor; separate preflight evidence is required')
    previous=np.asarray(cfg['initial_right_configuration'],np.float32).astype(float)
    last_completed_side='right';rows=[];details=[];state_issues=[]
    for index in indices:
        out=root/'trials'/f'{index:02d}';result=evidence.json(out/'result.json',required=False)
        if result is None:
            issues.append(f'trial {index}: missing result record')
            result=dict(index=index,status='missing_result',executed_in_virtual_state=False)
        request=evidence.json(out/'input.json',required=False)
        scene=evidence.json(root/'scenes'/f'{index:02d}.json',required=False)
        process=evidence.json(out/'process/outcome.json',required=False)
        model=evidence.json(out/'model.json',required=False)
        validation=evidence.json(out/'SCSTO/validation.json',required=False)
        exact=evidence.arrays(out/'SCSTO/exact_scene_validation.npz')
        samples=evidence.arrays(out/'SCSTO/trajectory_samples.npz')
        side='left'if index%2==0 else'right'
        advanced=bool(result.get('executed_in_virtual_state'))
        local=[];state_delta=0.0;side_switched=False
        if request:
            recorded=np.asarray(request['start'],float)
            if recorded.shape!=(7,)or not np.array_equal(recorded,previous):local.append('recorded input start differs from prior validated endpoint / held state')
            if request.get('target_side')!=side:local.append('target-side schedule changed')
            if targets is not None and not np.array_equal(np.asarray(request['target_pose']),np.asarray(targets[side])):local.append('target pose differs from frozen target')
        elif advanced:local.append('state advanced without a saved input')
        if scene and scene.get('target_side')!=side:local.append('scene target-side label differs from fixed schedule')
        if model and result.get('voxels')is not None:
            expected=model['static_pairs_including_table']+model['dynamic_robot_boxes']*result['voxels']
            if model['all_drake_candidate_pairs']!=expected:local.append('Drake pair count does not equal static plus all robot/voxel pairs')
            if model['dynamic_robot_boxes']!=11 or model['native_static_pairs']!=model['static_pairs_including_table']:local.append('robot box count or native/static pairs differ from declared model')
        if advanced:
            required=['solver_success','validation_passed','native_cuda_collision_free','independent_voxel_collision_free','exact_scene_collision_free']
            if not all(result.get(key)is True for key in required):local.append('virtual state advanced without every solver/geometry gate')
            if result.get('status')!='success_numerical':local.append('state advance has a nonsuccess terminal status')
            if not validation or not all(validation.get(key)is True for key in ['passed','collision_free','native_cuda_collision_free','independent_collision_free','limits_pass','derivative_pass']):local.append('saved numerical validation does not support advance')
            if exact is None or samples is None:local.append('state advanced without saved trajectory and exact-solid checks')
            else:
                q=np.asarray(samples.get('q'));free=np.asarray(exact.get('free'));distance=np.asarray(exact.get('minimum_signed_distance'))
                if q.ndim!=2 or q.shape[1]!=7 or len(q)==0 or not np.isfinite(q).all():local.append('saved trajectory shape/nonfiniteness invalid')
                else:
                    if not np.array_equal(q,exact.get('q')):local.append('exact-box check did not use the saved trajectory samples')
                    if free.shape!=(len(q),)or distance.shape!=(len(q),)or not free.all()or not np.array_equal(free,distance>=0):local.append('exact-solid flags/distances do not support collision freedom')
                    if not(np.asarray(samples.get('cuda')).all()and np.asarray(samples.get('independent')).all()):local.append('saved voxel-model flags do not support collision freedom')
                    terminal=np.asarray(result.get('terminal_configuration'),float)
                    if terminal.shape!=(7,)or not np.isfinite(terminal).all():local.append('invalid terminal configuration')
                    else:
                        if np.linalg.norm(q[-1]-terminal)>cfg['endpoint_tolerance']:local.append('recorded terminal is not saved trajectory endpoint')
                        if request and np.linalg.norm(q[0]-np.asarray(request['start'],np.float32))>cfg['endpoint_tolerance']:local.append('saved trajectory start differs from canonical input')
            terminal=np.asarray(result.get('terminal_configuration',previous),float)
            if terminal.shape==(7,)and np.isfinite(terminal).all():
                state_delta=float(np.linalg.norm(terminal-previous));previous=terminal
                side_switched=side!=last_completed_side;last_completed_side=side
        elif result.get('status')=='success_numerical':local.append('success status did not advance virtual state')
        component_fields=result.get('planning_components_recorded')
        if component_fields is not None:
            expected=sum(result[key]for key in component_fields)if component_fields else None
            if not equal_number(expected,result.get('planning_component_seconds')):local.append('planning component sum differs from recorded measured fields')
        partial=result.get('partial_record')or{}
        details.append(dict(index=index,status=result['status'],scheduled_target_side=side,state_advanced=advanced,
            completed_target_side_switch=side_switched,virtual_joint_displacement_l2=state_delta,
            wall_seconds=result.get('wall_seconds'),wall_seconds_missing=result.get('wall_seconds')is None,
            configured_watchdog_seconds=cfg['trial_timeout_seconds'],process=process,
            partial_record_measured_timings={key:partial[key]for key in TIME_LABELS if partial.get(key)is not None},
            exact_scene_collision_samples=result.get('exact_scene_collision_samples'),
            seed_valid=result.get('drm_seed_valid'),issues=local))
        state_issues.extend(f'trial {index}: {issue}'for issue in local);rows.append(result)
    issues.extend(state_issues)
    statuses=dict(Counter(row['status']for row in rows));successes=sum(bool(row.get('executed_in_virtual_state'))for row in rows)
    metrics={key:distribution(rows,key)for key in METRICS}
    if saved_summary:
        if saved_summary.get('requested')!=len(indices):issues.append('summary requested count mismatch')
        if saved_summary.get('case_status_counts')!=statuses:issues.append('summary status counts mismatch')
        if saved_summary.get('strict_virtual_successes')!=successes:issues.append('summary strict virtual success count mismatch')
        for key,value in metrics.items():
            actual=saved_summary.get('metrics',{}).get(key)
            if actual is None or actual.get('recorded')!=value['recorded']or not equal_number(actual.get('mean'),value['mean'])or not equal_number(actual.get('sample_sd'),value['sample_sd']):
                issues.append('summary metric/denominator mismatch: '+key)
    elif m.get('status')=='completed_with_failures_retained':issues.append('completed run is missing summary')
    if 'last_virtual_state'in m and not np.array_equal(previous,np.asarray(m['last_virtual_state'])):issues.append('manifest last_virtual_state breaks the saved chain')
    return dict(run_dir=str(root),run_status=m.get('status'),full_declared_condition=full,config=cfg,preflight=preflight,roadmap=roadmap,
        source_manifest=source_manifest,targets=targets,saved_summary=saved_summary,requested=len(indices),rows=rows,details=details,
        case_status_counts=statuses,strict_virtual_successes=successes,completed_side_switches=sum(x['completed_target_side_switch']for x in details),
        nonzero_virtual_joint_motions=sum(x['virtual_joint_displacement_l2']>cfg['endpoint_tolerance']for x in details),
        metrics=metrics,metrics_given_strict_virtual_success={key:distribution([row for row in rows if row.get('executed_in_virtual_state')],key)for key in METRICS},
        right_target_equals_checked_initial=right_is_initial,issues=issues,audit_passed=not issues,
        physical_experiment=False,original_author_hardware_reproduction=False)


def number(value,precision=3):return 'not recorded'if value is None else f'{value:.{precision}f}'


def moment(value,scale=1.):
    if value['mean']is None:return 'not recorded'
    mean=number(value['mean']*scale)
    return mean+(' ± '+number(value['sample_sd']*scale)if value['sample_sd']is not None else' (SD unavailable; n=1)')


def markdown(report,smoke):
    cfg=report['config'];preflight=report['preflight']or{};meta=preflight.get('metadata',{});roadmap=report['roadmap']or{}
    statuses='; '.join(f"{key}: {value}"for key,value in sorted(report['case_status_counts'].items()))
    lines=['# Constructed hardware-condition simulation: measured report','',
        f"The formal run requested **{report['requested']} trials** with **{cfg['roadmap_nodes']:,} roadmap nodes**. It recorded **{report['strict_virtual_successes']} strictly validated virtual state updates** and **{report['completed_side_switches']} validated changes of the scheduled target-side label**. Terminal statuses: {statuses}.",'',
        '**This is a constructed kinematic planning simulation, not a physical robot experiment or a reproduction of camera measurements.** It does not reproduce the paper’s 15 physical trajectories or Table II latency. The Forest/FR3 grid statistics remain separate. A `completed` summary denotes accounting completion, not success of every trial.','',
        f"Raw run: `{report['run_dir']}`. Run state: `{report['run_status']}`. Saved-record accounting audit: **{'passed'if report['audit_passed']else'issues found'}**. The audit reads saved inputs, trajectory samples, collision flags and distances; it does not rerun collision geometry or certify continuous safety.",'',
        '## Published settings and actual constructed inputs','',
        '| Item | Paper hardware experiment | This run |','|---|---|---|',
        f"| Trials / node count | 15 trajectories / 35,000 nodes | {report['requested']} requested / {roadmap.get('actual_nodes','not completed or not recorded')} actual nodes |",
        '| Robot / gripper | KUKA LBR iiwa 7 R800; 11 collision boxes | Pinned public iiwa7 R800 (8 boxes) + WSG (3 boxes); author calibration unavailable |',
        f"| Collision pairs | Reported 105 self pairs | {meta.get('static_pairs_including_table','not recorded')} static including table; {meta.get('robot_table_pairs','not recorded')} robot/table; {meta.get('static_pairs_including_table',0)-meta.get('robot_table_pairs',0)if meta else'not recorded'} robot/robot |",
        f"| Online robot geometries | 11 boxes | {meta.get('dynamic_robot_boxes','not recorded')} boxes, including anchored base |",
        f"| DRM distance caps / neighbors | 0.45 m / 4.5 rad / 10 | {cfg['max_task_space_distance']} / {cfg['max_configuration_distance']} / {cfg['roadmap_neighbors']} |",
        f"| Offline voxel full side | 0.06 m | {cfg['offline_voxel_side']} m; native half-side {roadmap.get('native_voxel_half_side','not recorded')} |",
        f"| Online voxel side / sphere radius | 0.08 m / circumsphere | {cfg['online_voxel_side']} m / {cfg['online_voxel_radius']:.9f} m |",
        f"| EI-ZO / lazy step | FR3 parameters, max. 20 planes / 0.1 rad | `{cfg['eizo']}` / {cfg['lazy_edge_step']} rad |",
        f"| Joint velocity / acceleration | ±1 rad/s / ±1 rad/s² | ±{cfg['velocity_limit']} / ±{cfg['acceleration_limit']} |",
        '| Sensing and execution | Three Intel RealSense D415 cameras and physical robot execution | Neither present; no sensor/controller/device latency |','',
        'Parameter sources are Section VII-B, Appendix D (Hardware) and Table VI. The 30-iteration EI-ZO cap follows the draft TeX comment; it does not establish the δ/ε guarantee for every returned set. Degree-six Bezier curves, relative SCSTO tolerance 0.01, validation limits, attempt counts and watchdogs are explicit construction choices. No extra collision pairs or filters were introduced to force the reported 105-pair count.','',
        'The public WSG is welded at translation `(0, 0, 0.09)` and RPY `(-pi/2, 0, pi/2)`; the declared TCP is `(0, -0.12, 0)` in its frame. These are public-URDF frame adaptation choices, not author calibration. Source bytes and hashes are retained in the run’s `source_snapshot/reproduction/hardware_constructed/assets/source_manifest.json`.','',
        f"Frozen initial/right configuration: `{cfg['initial_right_configuration']}`. Left target-generating configuration: `{cfg['left_target_generating_configuration']}`. Right target-generating configuration: `{cfg['right_target_generating_configuration']}`. Right target anchor equals the separately checked initial anchor: `{report['right_target_equals_checked_initial']}`. Preflight status: `{preflight.get('status','not recorded')}`; initial free: `{preflight.get('declared_initial_free','not recorded')}`; left anchor free: `{preflight.get('declared_left_anchor_free','not recorded')}`.",'',
        'The anchors were chosen before any roadmap or planning by a preregistered finite geometric scan, not by planner outcomes. The first candidate pair passing the fixed workspace, static model and all 15 voxel/exact-solid scenes was selected. The original colliding anchors and source snapshot are retained in `reports/paper_constructed/hardware_simulation/check_01`; the complete scan and selection are in `anchor_construction_01`. The revised preflight checks all three anchors against all 15 layouts. The target poses are fixed FK transforms saved in `fixed_targets.json`. The 15 source-defined rack/box placements are frozen in `scenes/00.json`–`14.json`; no target or obstacle is regenerated after a planner/solver failure. The table dimensions/center and every rack/box component are declared construction data. A nominal alternating target schedule is not evidence of 15 completed side-to-side motions. Failed trials hold the actual virtual state, so a subsequent scheduled target may be on the same side as the last completed one.','',
        'The synthetic cloud samples all box surfaces without ray tracing, occlusion, camera noise or calibration. Known table points are saved but excluded from online voxels because the exact table is already static in the collision model. Surface voxels do not certify solid occupancy. Saved final trajectories receive a separate Drake check against the exact constructed rack/box solids; that check, native/independent voxel checks, solver success, endpoint, bounds and derivative checks must all pass before state advancement.','',
        '## Recorded trial outcomes and state chain','',
        '| Trial | Scheduled target | Terminal status | Virtual update | Joint-state change (L2 rad) | Exact solid collision samples | Wall time (s) |',
        '|---:|---|---|---|---:|---:|---:|']
    for row in report['details']:
        lines.append(f"| {row['index']+1} | {row['scheduled_target_side']} | {row['status']} | {row['state_advanced']} | {row['virtual_joint_displacement_l2']:.6f} | {row['exact_scene_collision_samples']if row['exact_scene_collision_samples']is not None else'not recorded'} | {number(row['wall_seconds'])} |")
    lines+=['',f"There were {report['nonzero_virtual_joint_motions']} validated nonzero virtual joint-state changes. A zero change after failure is a recorded hold-state rule, not a zero-duration timing measurement. Missing wall times remain missing. The configured per-trial watchdog is {cfg['trial_timeout_seconds']} s and offline watchdog is {cfg['offline_timeout_seconds']} s; neither value is substituted for measured runtime. Timeouts may preserve measured subcomponents under `partial_record`; those are separately listed in `report_audit.json` and are not silently pooled into completed timing fields.",'',
        '## Timing with explicit denominators','',
        'Means and sample standard deviations below use only rows containing that actual timing field. Failed trials remain in the status denominator. Missing/unattempted components are never replaced by zero. The strict-success column is an additional conditional view, not a replacement denominator.','',
        '| Component | Recorded n / requested | All recorded mean ± SD (ms) | Strict-success recorded n | Strict-success mean ± SD (ms) |',
        '|---|---:|---:|---:|---:|']
    for key,label in TIME_LABELS.items():
        all_rows=report['metrics'][key];good=report['metrics_given_strict_virtual_success'][key]
        lines.append(f"| {label} | {all_rows['recorded']} / {report['requested']} | {moment(all_rows,1000)} | {good['recorded']} | {moment(good,1000)} |")
    lines+=['',
        'Synthetic surface construction and voxelization are not camera perception. `planning_component_seconds` is the sum of the actually recorded collision-set, IK, DRM, safe-set and SCSTO components; it excludes model setup, map loading, warmup, synthetic-cloud generation and final independent checks. On a failed trial it can be a partial sum, with contributing field names retained per row. It is not end-to-end camera-to-robot latency. SCSTO duration is planned kinematic trajectory time, not measured execution time.','',
        f"Planned trajectory duration: {moment(report['metrics']['duration'])} s, n={report['metrics']['duration']['recorded']}. Region count: {moment(report['metrics']['regions'])}, n={report['metrics']['regions']['recorded']}. Online voxels: {moment(report['metrics']['voxels'])}, n={report['metrics']['voxels']['recorded']}. Offline-map status: `{roadmap.get('status','not recorded')}`; recorded offline total: {number(roadmap.get('offline_total_seconds'))} s. Failed offline partial records stay nested in the raw roadmap record; missing totals are not inferred.",'',
        '## Audit, scope and retained failures','',
        'The posthoc audit recomputes status counts, success counts and every summary mean/SD/denominator. It checks the saved start-state chain, unchanged targets, recorded exact-solid flags versus signed distances, trajectory identity between voxel and exact-solid checks, endpoint identity, and each required gate before a virtual update. It separately verifies actual robot/voxel pair counts when available. An accounting audit can pass even when every planning trial fails.','']
    if report['issues']:lines.extend('- '+issue for issue in report['issues'])
    else:lines.append('No inconsistencies were found in these saved-record checks.')
    if smoke:
        lines+=['','## Separate smoke run','',f"The optional smoke run `{smoke['run_dir']}` used {smoke['config']['roadmap_nodes']} nodes and requested {smoke['requested']} trials; statuses: `{smoke['case_status_counts']}`. Its {smoke['strict_virtual_successes']} virtual successes are not pooled with the formal run. Saved-record audit issues: `{smoke['issues']}`."]
    lines+=['','All source snapshots, IK attempts, seed paths, clouds, corridors, candidates, validation arrays, process exits and failures remain in the raw run. Static replay includes all requested trials rather than a successful subset. There is no claim of the paper’s original calibration, camera observations, obstacle arrangements, 105-pair model, physical controller behavior, 15 physical successes or published latency. The newly constructed supplement addresses executable planning under similar public parameters while preserving these missing experimental conditions.','']
    return '\n'.join(lines)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--smoke-run',type=Path);parser.add_argument('--output',type=Path,default=DEFAULT_OUTPUT);args=parser.parse_args()
    evidence=Evidence();formal=audit_run(args.run_dir,evidence,require_full=True)
    smoke=audit_run(args.smoke_run,evidence,require_full=False)if args.smoke_run else None
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'report.md').write_text(markdown(formal,smoke))
    result=dict(status='passed'if formal['audit_passed']and(smoke is None or smoke['audit_passed'])else'issues_found',
        physical_experiment=False,run_dir=formal['run_dir'],formal=formal,smoke=smoke,
        report_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),input_files=evidence.files,
        report_sha256=hashlib.sha256((args.output/'report.md').read_bytes()).hexdigest())
    atomic(args.output/'report_audit.json',result)
    print(args.output/'report.md',flush=True)
    return 0 if result['status']=='passed'else 2


if __name__=='__main__':raise SystemExit(main())
