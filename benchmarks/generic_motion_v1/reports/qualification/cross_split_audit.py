"""Read-only duplicate and seed audit over every frozen case, without planning."""
from collections import defaultdict
import hashlib
import json
import numpy as np
from benchmarks.generic_motion_v1.common import BASE, cases, sha, write


def groups(values):
    return [v for v in values.values() if len(v)>1]


def main():
    worlds=defaultdict(list);pair_ids=defaultdict(list);seeds=defaultdict(list)
    report={};seen_scenes={}
    for split in ('smoke','dev','test'):
        path=BASE/'manifests'/f'{split}.jsonl';rows=cases(path)
        report[split]=dict(rows=len(rows),manifest_sha256=sha(path),valid=sum(c['validation_status']=='valid_input' for c in rows))
        for c in rows:
            seeds[c['endpoint_seed']].append(c['problem_id'])
            a=np.asarray(c['voxels_xyz'],dtype=np.float32).reshape(-1,3)
            ordered=np.asarray(sorted(a.tolist()),dtype=np.float32).reshape(-1,3)
            key=hashlib.sha256(ordered.tobytes()+str(c['voxel_radius_m']).encode()).hexdigest()
            if c['scene_id'] not in seen_scenes:
                worlds[key].append(dict(scene_id=c['scene_id'],split=split,family=c['family']))
                seen_scenes[c['scene_id']]=key
            if c['validation_status']=='valid_input':
                ends=sorted([c['q_start'],c['q_goal']])
                pair=hashlib.sha256(json.dumps([key,ends],separators=(',',':')).encode()).hexdigest()
                pair_ids[pair].append(c['problem_id'])
    duplicates=groups(worlds)
    unintended=[g for g in duplicates if any(x['family']!='open_control' for x in g)]
    result=dict(passed=not unintended and not groups(pair_ids) and not groups(seeds),splits=report,
        exact_geometry_duplicate_groups=duplicates,unintended_geometry_duplicate_groups=unintended,
        duplicate_unordered_endpoint_pairs=groups(pair_ids),duplicate_endpoint_seed_groups=groups(seeds),
        meaning='Same numerical voxel world compared after sorting centres. All open_control IDs intentionally share the one fixed base world; they are not independent environments. Near-duplicate geometry is not ruled out by exact hashes.',
        methods_executed=False)
    write(BASE/'reports/qualification/cross_split_audit.json',result)
    print(json.dumps(result,indent=2));return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
