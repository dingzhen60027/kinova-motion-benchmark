# Native DRM abort audit: fixed sparse-clutter case, roadmap seed 2

Audit date: 2026-10-28. This is a read-only analysis of two sealed attempts in the ongoing `rss2025_dev_v1` run. No planner, CUDA operation, model construction, rerun, input change or runtime-code change was performed for this audit. The conclusions concern these two attempts, not the final aggregate of the ongoing run.

Both attempts terminate in the native DRM frontend with `SIGABRT` after exhausting selectable roadmap-node candidates while trying to connect the fixed start configuration. **They are native-process failures, not evidence that the planning problem is infeasible.** EI-ZO, LSCS and SCSTO were not reached. The saved `infrastructure_error` / `native_process_signal` classification is accurate; reporting these as optimizer failures, timeouts or proven no-solution cases would not be accurate.

## Sealed evidence

Paths below are relative to `benchmarks/generic_motion_v1/`.

| Objective | Attempt directory | External wall time | Exit code | Timeout |
|---|---|---:|---:|---|
| Minimum distance | `results/rss2025_dev_v1/jobs/dev-sparse_clutter-000-p01-EI_ZO-minimum_distance-e72d5254/attempts/0001/` | 0.8654767517000437 s | -6 | false |
| Minimum time | `results/rss2025_dev_v1/jobs/dev-sparse_clutter-000-p01-EI_ZO-minimum_time-176842c2/attempts/0001/` | 0.8657355168834329 s | -6 | false |

Each attempt has 12 files listed by its `receipt.json`. All 24 listed file hashes were independently recalculated and match. Each `request.json` equals the corresponding complete job object in `run_manifest.json`. After removing the runner's `_objective` label, each `input.json` equals that job's frozen `case`. Both `runtime_sources.json` records report unchanged before/after source hashes and are byte-identical.

The two inputs differ only in `_objective`. Their endpoint-audit, evaluation, stdout, stderr and runtime-source files are byte-identical. The requests differ only in `job_id` and `objective`. In particular, the robot, collision geometry, obstacles, sphere centres/radius, endpoints, roadmap and seeds were not changed between objectives.

| Shared identity | Value |
|---|---|
| Case / scene | `dev-sparse_clutter-000-p01` / `dev-sparse_clutter-000` |
| Runtime commit | `e1abf004b4347dff75b15a8868e14b732d3bf539` |
| Roadmap | `kinova7-full-n6000-s2-e258c5f9914c` |
| Roadmap construction seed | 2 |
| Method seed | 3273196092; EI-ZO was not reached |
| Solver seed field | 771059514; optimizer was not reached |
| Robot SHA-256 | `a25f447c8caf4aff6dc76196c3d57cc940f00e802367f48034f06a9973927c75` |
| Collision-geometry SHA-256 | `896ab9f55c5d8081b843358bd85cde2daae1f432564593f86218bf61733ce4fa` |
| Model-source SHA-256 | `5b152e3e5ca484e5378856de11747c45826653df8a1183582fe90e0958fc5116` |
| Problem SHA-256 | `7926f78175c7d47a7a1d8d1e3d1d5b14ae7aa4ae9c39c7ba1fe2e8149c4bb1e0` |
| Scene SHA-256 | `93cd0c49a8697aa305249ce3019335696a78e62e22ae8fe96ad39694b315c35a` |
| Voxel-geometry SHA-256 | `49bf886a6ef08927302f33edfe014359856272e4cca6eb1afab3bc34018fb2c5` |
| Online spheres | 207 centres; radius 0.04 m |
| Evaluation-file SHA-256 | `372ae73a7a1f41af9e91117391f81eadf156682bbfb30f4a1bbbcf53ca748a24` |
| Native-map SHA-256 | `5b735cd3605db7b8cef80595bcc3e1c300f31f6ff22fe511a36f9ca476e06b61` |

The map file's current SHA matches its metadata and both result records. The saved `roadmap.npz`, exported from the native map object by `roadmap.py`, contains 6000 finite float32 configurations of dimension 7, node IDs 1–6000 and 75522 directed edges. All stored node coordinates lie between -3.0984352 and 3.09861 rad. These checks exclude nonfinite endpoints/nodes and negative node IDs as explanations supported by the saved inputs; this audit did not instrument the aborted process's internal memory.

The native loading log reports all roadmap nodes valid against the current base geometry. Before calling the planner, the runner also verified model hashes, native-map hash, requested map configuration and scene cache bounds. Map construction occurred at commit `0c3028ee5b6d80e3901578e7abcb1385abcdf122`; use at the later runtime commit is explicit and does not relabel its construction provenance.

## What the logs establish

Both `endpoint_audit.json` files record unchanged endpoints, `valid: true`, and CUDA-free results for both ends. Independent minimum clearances are 0.017410790231424174 m at the start and 0.00810697539810306 m at the goal. The endpoint-validity check includes joint limits and the existing contact band. This is therefore not an invalid-endpoint rejection.

The complete meaningful stdout sequence is identical:

1. The map is loaded for `kinova::end_effector_link`.
2. All roadmap nodes pass the current robot-geometry validation.
3. `Built collision set: 5847 total (0 from geometry, 5847 from environment)`.
4. `Built forbidden edge set of size: 0`.
5. `Failed steer to start joint.` appears 15 times.

Both stderr files end with:

```text
python: csdecomp/src/planning/drm_planner.cpp:249: int32_t csdecomp::DrmPlanner::GetClosestNonCollidingConfigurationInRoadMap(const VectorXf&, Eigen::VectorXf*): Assertion `closest_node >= 0' failed.
```

The last saved phase is `drm_path`. No completed `drm_path_seconds`, `path.npy`, candidate, polytope, trajectory samples or validation result exists for either attempt. The prior Drake deprecation warning is not the abort cause. Neither process reached its 60-second timeout.

## Source meaning and bounded inference

Inspected source: `csdecomp/src/planning/drm_planner.cpp`, SHA-256 `159300e72e24c2ed398c2c1453a94c10f1f34024f20b014c06e89825380658d6`. `git diff v0.1.2 -- csdecomp/src/planning/drm_planner.cpp` is empty. The run uses the installed native binary; the matching native assertion text and line support this source interpretation, but reading source is not a substitute for a rebuilt/instrumented binary test.

At lines 234–250, `GetClosestNonCollidingConfigurationInRoadMap` initializes `closest_node = -1`, scans nodes outside `collision_set_`, and selects the smallest Euclidean joint distance. It asserts `closest_node >= 0` before returning. With finite query/node vectors and positive IDs, no selected node means the current exclusion set leaves no selectable candidate.

The initial environment cache excludes 5847 of 6000 nodes, leaving 153 candidates. The connection loop at lines 466–493 inserts each node into `collision_set_` when its straight-line steering check from the start fails. A planning retry does not clear that exclusion set. The frozen options allow 10 connection attempts per planning attempt and 20 planning attempts. Thus 15 logged exhausted connection loops account for 150 additional discarded candidates. The source-driven reconstruction is that the next planning attempt discarded the remaining three and then called the nearest-node lookup on an exhausted set, which asserted. The final three individual IDs/checks were not logged; this reconstruction is supported by the counts and control flow, not presented as an observed internal trace.

The caller contains an intended `if (start_id == -1)` failure branch, but the callee's assertion prevents that return in this build. Disabling assertions would not by itself implement a safe failure return, because the following map lookup still uses the invalid ID. A future upstream correction should explicitly handle an empty selectable set and return a structured frontend failure or catchable exception. **No correction or altered build was applied during this frozen run.**

`collision_set_` is a pruning/exclusion set, not a certificate that every member configuration is itself colliding. Here it combines offline environment-cache hits with nodes rejected because the particular connection from the start failed. A free roadmap node can have an obstructed straight-line start connection; another route could still reach it. Cache discretization also has the limitations already described in the paper-algorithm source audit. These observations do not establish whether another seed, denser map or planner would solve this fixed task.

The input remains `feasibility_unknown`. Its sampled direct joint-space line is marked colliding (minimum sampled clearance -0.06932029362206373 m), but a colliding direct line is not an infeasibility proof. This audit establishes neither a solution nor the absence of one.

## Reporting and reproducible checks

Retain both failures in the full planned-job denominator and in the native-process-failure category. They are paired objective executions of the same case and roadmap seed, not two independent random scenes. There is no candidate cost, no optimization success and no finished DRM timing to impute. The measured external abort times may be summarized as failure timings; they cannot be labelled successful planning latency. The full development run was still in progress when this audit was written.

The actual audit used `sed`, `rg`, `git diff` and CPU-only JSON/NumPy/SHA checks through the existing Conda wrapper. The following compact command reproduces the key artifact checks without importing CSDecomp or constructing a model:

```bash
cd /home/eric/wjg/wd/csdecomp
PYTHONDONTWRITEBYTECODE=1 bash local/python.sh - <<'PY'
from pathlib import Path
import hashlib, json
import numpy as np
b = Path('benchmarks/generic_motion_v1')
r = b / 'results/rss2025_dev_v1'
m = json.loads((r / 'run_manifest.json').read_text())
names = [
    'dev-sparse_clutter-000-p01-EI_ZO-minimum_distance-e72d5254',
    'dev-sparse_clutter-000-p01-EI_ZO-minimum_time-176842c2',
]
records = []
for name in names:
    p = r / 'jobs' / name / 'attempts/0001'
    receipt = json.loads((p / 'receipt.json').read_text())
    assert len(receipt['files']) == 12
    for filename, expected in receipt['files'].items():
        assert hashlib.sha256((p / filename).read_bytes()).hexdigest() == expected
    job = next(j for j in m['jobs'] if j['job_id'] == name)
    assert json.loads((p / 'request.json').read_text()) == job
    case = json.loads((p / 'input.json').read_text())
    case.pop('_objective')
    assert case == job['case']
    assert json.loads((p / 'runtime_sources.json').read_text())['unchanged']
    records.append(case)
    print(name, json.loads((p / 'process.json').read_text())['exit_code'])
assert records[0] == records[1]
road = b / 'roadmaps/rss2025_n6000_s002'
meta = json.loads((road / 'metadata.json').read_text())
assert hashlib.sha256((road / 'native.map').read_bytes()).hexdigest() == meta['native_map_sha256']
with np.load(road / 'roadmap.npz', allow_pickle=False) as data:
    assert data['nodes'].shape == (6000, 7)
    assert np.isfinite(data['nodes']).all()
    assert np.array_equal(data['node_ids'], np.arange(1, 6001))
    print('nodes:', data['nodes'].shape, 'directed edges:', data['edges'].shape)
print('receipt, pairing and map checks passed')
PY
```
