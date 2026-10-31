"""Create reviewable candidate and unified diff, without editing live source."""
from pathlib import Path
import difflib
import hashlib
import json

root=Path(__file__).resolve().parents[4]
relative=Path('benchmarks/generic_motion_v1/runners/csdecomp_baseline.py')
source=root/relative
out=Path(__file__).resolve().parent
before=source.read_text()
helper='''def _input_endpoints(case):
    """Reject recorded or malformed input before any conversion or GPU call."""
    if case.get('validation_status') != 'valid_input':
        return None
    try:
        endpoints=np.asarray([case.get('q_start'),case.get('q_goal')],dtype=float)
    except (TypeError,ValueError):
        return None
    if endpoints.shape != (2,7) or not np.isfinite(endpoints).all():
        return None
    return endpoints


def _assert_motion_limits(case,cfg):
    """The fixed manifest defines the task; evaluation cannot silently change it.

    This v1 runner supports uniform per-joint limits. A different vector or
    endpoint velocity requires a separately named and frozen input protocol.
    """
    for record_key,config_key in (
            ('velocity_limits_rad_s','velocity_limit_rad_s'),
            ('acceleration_limits_rad_s2','acceleration_limit_rad_s2')):
        try:
            manifest_limits=np.asarray(case[record_key],dtype=float)
            configured=float(cfg[config_key])
        except (KeyError,TypeError,ValueError) as exc:
            raise ValueError(f'Missing or malformed frozen motion limits: {record_key}') from exc
        if (manifest_limits.shape != (7,) or not np.isfinite(manifest_limits).all()
                or np.any(manifest_limits<=0) or not np.isfinite(configured) or configured<=0
                or not np.array_equal(manifest_limits,np.full(7,configured))):
            raise ValueError(f'Evaluation {config_key} differs from frozen manifest {record_key}; '
                             'a different task requires a separately frozen experiment protocol')
    try:
        endpoint_velocity=np.asarray(case['endpoint_velocity_rad_s'],dtype=float)
    except (KeyError,TypeError,ValueError) as exc:
        raise ValueError('Missing or malformed frozen endpoint velocity') from exc
    if endpoint_velocity.shape != (7,) or not np.array_equal(endpoint_velocity,np.zeros(7)):
        raise ValueError('This frozen protocol requires seven zero endpoint velocities')


'''
needle='def run_case(case,roadmap_dir,out,objective='
assert before.count(needle)==1
candidate=before.replace(needle,helper+needle)
old='''            endpoints=np.asarray([case.get('q_start'),case.get('q_goal')],float)
            if (case.get('validation_status') != 'valid_input' or endpoints.shape != (2,7)
                    or not np.isfinite(endpoints).all()):'''
new='''            endpoints=_input_endpoints(case)
            if endpoints is None:'''
assert candidate.count(old)==1
candidate=candidate.replace(old,new)
old="result['generation_rejections']=case.get('generation_rejections',case.get('endpoint_generation'))"
new="result['generation_rejections']=case.get('generation_rejections',case.get('endpoint_sampling'))"
assert candidate.count(old)==1
candidate=candidate.replace(old,new)
needle="""                return result
            import csdecomp as csd
"""
replacement="""                return result
            stage('input_contract')
            _assert_motion_limits(case,cfg)
            import csdecomp as csd
"""
assert candidate.count(needle)==1
candidate=candidate.replace(needle,replacement)
needle="""            elif phase=='drm_path':result['status']='frontend_failed'
"""
replacement="""            elif phase=='input_contract':
                result['status']='infrastructure_error';result['error_category']='input_protocol_mismatch'
            elif phase=='drm_path':result['status']='frontend_failed'
"""
assert candidate.count(needle)==1
candidate=candidate.replace(needle,replacement)
(out/'csdecomp_baseline.py.candidate').write_text(candidate)
diff=''.join(difflib.unified_diff(before.splitlines(True),candidate.splitlines(True),fromfile='a/'+str(relative),tofile='b/'+str(relative)))
(out/'runner_hardening.patch').write_text(diff)
(out/'proposal.json').write_text(json.dumps(dict(source=str(relative),source_sha256=hashlib.sha256(before.encode()).hexdigest(),candidate_sha256=hashlib.sha256(candidate.encode()).hexdigest(),scope='CPU-only input guards; no algorithm or successful-input numerical changes'),indent=2)+'\n')
print('Prepared patch; live source unchanged.')
