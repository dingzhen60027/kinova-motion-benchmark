"""Read existing v5 tasks with NumPy only; no CUDA, Drake, IK or solver calls."""
from pathlib import Path
import collections
import hashlib
import json
import subprocess
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
RUN = ROOT / 'reproduction/runs/20261026T025846.407684Z_paper_constructed_fr3'
inputs = {}
def read(path):
    path = Path(path)
    data = path.read_bytes()
    inputs[str(path.relative_to(ROOT))] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
    return json.loads(data)
def touch(path):
    path=Path(path); data=path.read_bytes()
    inputs[str(path.relative_to(ROOT))] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
def digest(x):
    return hashlib.sha256(json.dumps(x, sort_keys=True).encode()).hexdigest()
def stats(x):
    x=np.asarray(x,dtype=float)
    return dict(n=len(x), min=float(x.min()), mean=float(x.mean()), median=float(np.median(x)), max=float(x.max())) if len(x) else dict(n=0)
def rot(axis, angle):
    c,s=np.cos(angle),np.sin(angle)
    if axis==0: return np.array([[1,0,0],[0,c,-s],[0,s,c]])
    if axis==1: return np.array([[c,0,s],[0,1,0],[-s,0,c]])
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def transform(xyz=(0,0,0),rpy=(0,0,0)):
    T=np.eye(4);T[:3,3]=xyz;T[:3,:3]=rot(2,rpy[2])@rot(1,rpy[1])@rot(0,rpy[0]);return T
asset=read(ROOT/'reproduction/paper_constructed/assets/fr3_33.json')
cfg=read(ROOT/'reproduction/paper_constructed/config.json')
for name in ['reproduction/paper/2504.10783v1.pdf','reproduction/paper/paper.txt','reproduction/paper_constructed/data.py','reproduction/paper_constructed/model.py','reproduction/paper_constructed/assets/fr3_limits_20250411.yaml','reproduction/reports/paper_completion/source_evidence/arxiv_text/sections/experiments.tex','reproduction/reports/paper_completion/source_evidence/arxiv_text/sections/drm.tex','reproduction/reports/paper_completion/source_evidence/arxiv_text/sections/appendix.tex']:
    touch(ROOT/name)
lower=np.array([asset['limits'][f'joint{i}']['limit']['lower'] for i in range(1,8)])
upper=np.array([asset['limits'][f'joint{i}']['limit']['upper'] for i in range(1,8)])
def fk(q):
    T=np.eye(4)
    for i in range(1,9):
        k=asset['kinematics'][f'joint{i}']['kinematic']
        T=T@transform([k[x] for x in ['x','y','z']],[k[x] for x in ['roll','pitch','yaw']])
        if i<8: T=T@transform(rpy=(0,0,q[i-1]))
    return T@transform(rpy=(0,0,-np.pi/4))@transform(xyz=(0,0,.1034))
scene_records=[]; scene_by_seed={}
for p in sorted((RUN/'scenes').glob('*/scene.json')):
    d=read(p);scene_by_seed[d['seed']]=d
    start=fk(d['start']);target=np.array(d['target_pose']);witness=fk(d['target_generating_configuration']);obs=np.array(d['obstacles']);radius=d['obstacle_radius']
    side_checks={}
    for key,pose in [('right',start),('left',target)]:
        xyz=pose[:3,3]; side_checks[key]=bool(np.all(xyz>=cfg['fr3_scene'][key+'_ee_lower']) and np.all(xyz<=cfg['fr3_scene'][key+'_ee_upper']))
    overlaps=sum(np.linalg.norm(obs[i]-obs[j])<2*radius for i in range(len(obs)) for j in range(i+1,len(obs)))
    scene_records.append(dict(seed=d['seed'],status=d['status'],start_xyz=start[:3,3].tolist(),target_xyz=target[:3,3].tolist(),start_tcp_z_world_z=float(start[2,2]),target_tcp_z_world_z=float(target[2,2]),target_angle_to_world_down_deg=float(np.degrees(np.arccos(np.clip(-target[2,2],-1,1)))),target_witness_fk_max_abs_error=float(np.max(np.abs(witness-target))),start_in_declared_box=side_checks['right'],target_in_declared_box=side_checks['left'],start_and_witness_in_joint_limits=bool(np.all((np.array([d['start'],d['target_generating_configuration']])>=lower)&(np.array([d['start'],d['target_generating_configuration']])<=upper))),obstacle_centers_lower=obs.min(axis=0).tolist(),obstacle_centers_upper=obs.max(axis=0).tolist(),obstacle_overlap_pairs=int(overlaps),target_above_this_scene_highest_obstacle=bool(target[2,3]>(obs[:,2]+radius).max()),start_above_this_scene_highest_obstacle=bool(start[2,3]>(obs[:,2]+radius).max()),tcp_endpoint_distance=float(np.linalg.norm(start[:3,3]-target[:3,3])),sampling_streams=dict(start=d['start_sampling']['seed_sequence'],target=d['target_sampling']['seed_sequence'],obstacles=d['obstacle_seed_sequence'])))
cases=[]; status=collections.Counter(); mismatches=[]
for folder in sorted((RUN/'cases').iterdir()):
    case=read(folder/'case.json');status[case['status']]+=1
    ik=read(folder/'online_pose_ik.json');seed=case['environment_seed'];scene=scene_by_seed[seed]
    rec=dict(id=folder.name,status=case['status'],environment_seed=seed,ik_status=ik['status'],ik_attempts=len(ik['attempts']),target_matches_scene=ik['target_pose']==scene['target_pose'],selected_node_id=ik.get('selected_node_id'))
    if ik.get('goal') is not None:
        goal=np.array(ik['goal']);target=np.array(scene['target_pose']);actual=fk(goal)
        rec.update(goal=goal.tolist(),goal_pose_position_error=float(np.linalg.norm(actual[:3,3]-target[:3,3])),goal_pose_rotation_matrix_error=float(np.linalg.norm(actual[:3,:3]-target[:3,:3])),goal_to_hidden_witness_distance=float(np.linalg.norm(goal-scene['target_generating_configuration'])),selected_init_equals_hidden_witness=any(a['accepted'] and np.array_equal(a['initial'],scene['target_generating_configuration']) for a in ik['attempts']))
    if (folder/'input.json').exists():
        inp=read(folder/'input.json');rec['start_matches_scene']=inp['start']==scene['start'];rec['obstacles_match_scene']=inp['obstacles']==scene['obstacles'];rec['input_target_matches_scene']=inp['target_pose']==scene['target_pose'];rec['input_goal_matches_online_ik']=inp['goal']==ik['goal']
    if (folder/'seed_path.npz').exists():
        touch(folder/'seed_path.npz');path=np.load(folder/'seed_path.npz')['path']
        rec.update(drm_segments=len(path)-1,drm_path_length=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()))
    for key in ['target_matches_scene','start_matches_scene','obstacles_match_scene','input_target_matches_scene','input_goal_matches_online_ik']:
        if key in rec and not rec[key]:mismatches.append([folder.name,key])
    cases.append(rec)
for file,record in inputs.items():
    assert hashlib.sha256((ROOT/file).read_bytes()).hexdigest()==record['sha256'],file
source_urls=dict(paper='https://arxiv.org/pdf/2504.10783v1',tex='https://arxiv.org/src/2504.10783v1',franka=f"https://github.com/frankarobotics/franka_description/tree/{asset['source_revision']}")
summary=dict(scene_count=len(scene_records),case_count=len(cases),case_statuses=dict(status),all_side_boxes_valid=all(x['start_in_declared_box'] and x['target_in_declared_box'] for x in scene_records),all_start_and_witness_limits_valid=all(x['start_and_witness_in_joint_limits'] for x in scene_records),target_fk_max_error=max(x['target_witness_fk_max_abs_error'] for x in scene_records),target_height=stats([x['target_xyz'][2] for x in scene_records]),start_height=stats([x['start_xyz'][2] for x in scene_records]),target_upward_z_count=sum(x['target_tcp_z_world_z']>0 for x in scene_records),target_within_30deg_down_count=sum(x['target_angle_to_world_down_deg']<=30 for x in scene_records),targets_above_own_scene_highest_obstacle=sum(x['target_above_this_scene_highest_obstacle'] for x in scene_records),start_above_own_scene_highest_obstacle=sum(x['start_above_this_scene_highest_obstacle'] for x in scene_records),total_obstacle_overlaps=sum(x['obstacle_overlap_pairs'] for x in scene_records),task_identity_mismatches=mismatches,ik_statuses=dict(collections.Counter(x['ik_status'] for x in cases)),ik_attempts=stats([x['ik_attempts'] for x in cases]),goal_position_error=stats([x['goal_pose_position_error'] for x in cases if 'goal_pose_position_error' in x]),goal_rotation_matrix_error=stats([x['goal_pose_rotation_matrix_error'] for x in cases if 'goal_pose_rotation_matrix_error' in x]),hidden_witness_used_as_accepted_init_count=sum(x.get('selected_init_equals_hidden_witness',False) for x in cases),drm_segments=stats([x['drm_segments'] for x in cases if 'drm_segments' in x]),drm_path_length=stats([x['drm_path_length'] for x in cases if 'drm_path_length' in x]),single_segment_drm_paths=sum(x.get('drm_segments')==1 for x in cases))
evidence=dict(schema_version=1,scope='Read-only audit of existing v5 task files, source files and independent NumPy FK; no GPU, IK, solver or planner execution.',git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),command='bash local/python.sh reproduction/reports/environment_reconstruction/task_semantics_audit.py',sources=source_urls,source_priority=['formal_published_pdf','public_fixed_commit_sources','unpublished_author_TeX_comments','declared_constructed_assumptions'],model=dict(dof=7,spheres_by_link={k:len(v) for k,v in asset['spheres'].items()},joint_lower=lower.tolist(),joint_upper=upper.tolist(),base_transform='identity; +x forward, +z up; paper does not define world axes',table=cfg['fr3_scene']['table_size'],table_center=cfg['fr3_scene']['table_center'],tcp_weld_translation=[0,0,.1034],hand_weld_yaw=-np.pi/4,finger_joint_displacement_each_m=.04,author_33sphere_asset_recovered=False,published_static_pairs=527,actual_static_pairs=268,published_dynamic_pairs=660,actual_dynamic_pairs=660),summary=summary,scenes=scene_records,cases=cases,input_files=inputs,inputs_unchanged=True)
evidence['semantic_findings'] = [
    dict(id='fr3_count', status='matched_public_counts', paper='VII-A1: 7DOF, 33 robot spheres, 20 obstacle spheres, box table', actual='Same counts and primitive types; geometry itself constructed'),
    dict(id='target_distribution', status='unpublished_not_proven_violation', paper='Random right-side configuration to random left-side pose; no distribution given', actual='Joint-uniform collision-free conditional FK target distribution; not Cartesian-uniform or Haar-uniform SO3'),
    dict(id='online_ik', status='matched_procedure_constructed_numeric_settings', paper='V-B: collision-free roadmap pose map warm-starts IK', actual='Same procedure; translation+0.2*rotation public code metric, max10 attempts is constructed'),
    dict(id='table_and_world_frame', status='unpublished_constructed', paper='Box table and obstacle cloud in front of robot; no world frame or dimensions', actual='Identity base, tabletop z0, box1.4x1.6x0.1, center(.5,0,-.05)'),
    dict(id='downward_grasp', status='not_a_published_requirement', paper='Figure6 is an example, not a target orientation distribution', actual='6/10 target TCP z axes upward; cannot infer a requirement violation'),
    dict(id='pair_count', status='confirmed_difference', paper_static=527, actual_static=268, paper_dynamic=660, actual_dynamic=660, rule='Do not invent pairs or alter filters merely to match total'),
    dict(id='task_difficulty', status='observed_distribution_difference_not_unique_cause', paper_segments=4.49, actual_segments=summary['drm_segments']['mean'], paper_path_length_rad=8.55, actual_path_length_rad=summary['drm_path_length']['mean'], actual_one_segment=summary['single_segment_drm_paths']),
    dict(id='simulation_derivative_limits', status='unpublished_constructed', paper='Simulation equations give symbolic bounds; unit bounds explicitly specified for hardware', actual='Unit per-joint velocity/acceleration bounds'),
]
evidence['proposed_sampling_protocol'] = dict(status='design_only_not_executed',family='reachable_pose_tabletop_traversal',primary_start='joint_uniform conditioned only on declared free endpoint and right TCP box',primary_target='independent joint_uniform witness conditioned only on declared free endpoint and left TCP box; save FK pose then freeze',independent_streams=['obstacle','start','target','roadmap'],proposal_cap_each_endpoint=200000,retain_exhaustion=True,hidden_witness_allowed_for_online_ik=False,selection_on_planning_success=False,selection_on_path_length_or_segments=False,target_fixed_across_roadmaps=True,case_endpoints_shared_across_methods=True,optional_pose_space_diagnostic=dict(position='uniform declared Cartesian left box',orientation='Haar-uniform SO3 via uniform quaternion sampling',reselect_on_ik_failure=False,claim_more_faithful_than_primary=False),validation='Report all scheduled tasks, attrition and conditional-on-DRM results; geometry values frozen from independent evidence before outcomes')
(OUT/'task_semantics_evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
print(json.dumps(summary,indent=2))
