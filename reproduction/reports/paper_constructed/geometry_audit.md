# Independent constructed-model geometry audit

Status: the fixed 1,184-configuration audit completed for the frozen new
33-sphere model in both static-table and 20-obstacle conditions, with zero
query failures. Residual approximation disagreements remain and are reported
below; this is not equivalence to the authors' unreleased model.

## Reference and fixed inputs

The reference independently reads official Franka description revision
`39f8c0bcea9bc48ae5334edcb2aa2a504f2d2c85` (2025-04-11). Its kinematics, joint
limits, hand xacro and both SRDF files are saved with URL and SHA-256 under
`geometry_audit/official/`. The nine official arm/hand STL files are checked
against the prior byte-for-byte comparison with that same prepublication
revision. The reference uses each complete collision STL's convex hull and all
four exact collision boxes on each finger. Finger opening is fixed at 0.04 m,
and hand/TCP transforms follow the published description.

This is a geometry oracle for the stated approximation, not an exact
non-convex mesh contact oracle. Drake's [proximity documentation](https://drake.mit.edu/doxygen_cxx/group__geometry__file__formats.html)
describes the convex-hull substitution. Positive hull separation proves the
corresponding original meshes are separated. Hull penetration alone may be
caused by a filled concavity and does not prove non-convex mesh contact. The
validator therefore labels sphere-free/reference-colliding results as
`reference_false_negative_candidate`; exact primitive-only collision witnesses
are counted separately. Negative distances use the strict `< 0` convention.
Distances within `1e-8` of contact are flagged without changing that convention.

All SRDF exclusions are parsed from those independently downloaded official
files. No outcome-dependent collision exclusions are added. The online scene
explicitly enables each obstacle against every robot geometry, including the
fixed base, matching the constructed driver policy. Default welded base/table
filtering is retained. The same body-pair policy is asserted across the sphere
and reference representations.

Before reading the new sphere asset, the validator saved 1,184 configurations:

| Fixed group | Count | Selection |
|---|---:|---|
| Home | 1 | `[0,-pi/4,0,-3pi/4,0,pi/2,0]` |
| Historical q7 scan | 31 | Exact saved configurations from the prior link5/hand diagnostic |
| Uniform joints | 1,024 | Prepublication joint ranges, fixed SeedSequence `[250410783,20261026,601]` |
| Left/right task workspace | 64 each | First points in the fixed TCP boxes from a separate joint-uniform stream `[250410783,20261026,602]` |

Workspace selection required 797 candidates. It uses only TCP position, with
no collision check, IK solve, sphere-model read or planning-success selection.
The entire candidate pool and TCP positions are saved. The scene has the
unchanged declared table and 20 obstacles generated with SeedSequence
`[250410783,0]`. Both static-table and online-obstacle conditions use the same
configuration list. The historical scan remains exact even if a point lies
outside the newly pinned joint limits; domain membership is reported, never
silently clamped or filtered.

`geometry_audit/preregistration.json` records the sample design and every frozen
input hash. `samples.npz` and `sample_metadata.json` retain actual inputs.
Preparation is idempotent: an existing registration is checked, not replaced.
The completed validation used the new package's exact `fr3_urdf` chain, replaced
its collision geometry with the official reference, and checked its FK against
the independently constructed preparation chain at every configuration.

## Actual preparation checks

Commands executed in the existing Conda environment:

```bash
bash local/python.sh -m reproduction.validate_constructed_geometry prepare
```

A second execution verified the existing frozen inputs and source hashes. AST
parsing also passed. The actual execution logs are `geometry_prepare.log` and
`geometry_prepare_verify.log` beside this report.

Two CPU-only reference probes queried all candidate-pair distances. Their
records and logs are preserved as `geometry_audit/oracle_preflight.json`,
`geometry_audit/oracle_online_preflight.json`, `geometry_oracle_preflight.log`
and `geometry_oracle_online_preflight.log`.

- Static reference: 60 candidate pairs.
- Online reference: 400 candidate pairs, including all 20 base/obstacle pairs.
- Home link5/hand hull distance: **+0.037048101092356324 m**; this was also the
  minimum over all 60 static candidate pairs at home.
- All 31 historical q7 configurations have separated link5/hand hulls.

These probes read no constructed sphere model and do not establish the new
model's error rate.

## Actual comparison

The following command was executed after the model builder declared its asset
frozen, and before the new full benchmark started:

```bash
bash local/python.sh -m reproduction.validate_constructed_geometry validate
```

The command created `geometry_audit/validation_20261026T024911Z/` and saved every configuration's
full pairwise signed-distance arrays, pair identities, deepest contacts,
collision lists, body-pair disagreements, confusion counts by sample group,
near-contact flags, domain membership and exceptions. It recorded the asset,
sample and validator hashes and checked that the asset did not change during the run.
Reference/sphere FK, official filters, limits and the total of 33 spheres are
checked. CUDA and optimization solvers are not invoked. Both failures and
incomplete attempts are retained rather than removed or folded into successes.

This is a bounded geometry diagnostic, not a performance benchmark or proof
that a reconstructed model matches the authors' unreleased sphere model.


The frozen asset SHA-256 is
`77b602396d93004c0aef6cf3776ef5783ed6fff9b08c0734829d3b9d2c1bd120`.
Its filters, official kinematics, prepublication limits, 33-sphere count and
all 1,184 FK comparisons passed. The constructed configuration's `fr3_scene`
matched the preregistered geometry settings; subsequent NTO-collocation changes
do not alter these inputs. None of the points were outside the pinned joint
limits or within `1e-8 m` of contact in either representation.

| Condition | Both free | Both collide | Sphere false positive | Reference false-negative candidate | Query failures |
|---|---:|---:|---:|---:|---:|
| Static table | 1,053 | 110 | 18 | 3 | 0 |
| Table + 20 obstacles | 780 | 364 | 38 | 2 | 0 |

Each row uses all **1,184 fixed diagnostic configurations**. The confirmed
false-positive fractions among reference-free configurations are 18/1,071 and
38/818. The candidate false-negative fractions among reference-colliding
configurations are 3/113 and 2/366; these are **convex-reference disagreements**,
not established original-mesh false negatives. No missed collision involved
an exact-primitive-only reference pair. The sample mixture includes deliberate
regression cases and workspace subsets, so its aggregate rates are not
estimates of the paper's task distribution or planning success.

For the independent uniform-joint subset of 1,024 points, static disagreement
counts are 16 sphere false positives and 2 reference false-negative candidates;
online counts are 27 and 2. The 128 workspace points contribute 2/1 static
and 11/0 online, respectively. Raw group counts remain in `summary.json`.

The new home configuration is free in both static representations: minimum
sphere distance **+0.0139269116 m**, minimum reference distance
**+0.0370481011 m**. All 31 historical q7 points are also free in both
representations; the smallest new sphere clearance is **+0.0121787563 m**.
This directly resolves the prior home/q7 regression for these fixed points.
With the declared online obstacles, home is colliding in both representations
(sphere distance **−0.0215231600 m**, reference **−0.00413993295 m**), which is
retained as a collision rather than reselected into a free test.

All three static reference false-negative candidates are link5/hand hull
intersections:

| Configuration | Sphere minimum distance (m) | Reference hull distance (m) |
|---|---:|---:|
| `joint_0608` | +0.0178974782 | −0.0005010782 |
| `joint_0673` | +0.0094285572 | −0.0005418788 |
| `workspace_left_38` | +0.0093418781 | −0.0011313458 |

The first two also appear as candidates in the online condition. The later
[original-triangle diagnostic](mesh_candidate_diagnostic.md) evaluated all three
fixed configurations and found numerical surface separation of 41.469, 34.410
and 34.296 mm, respectively, with both containment tests outside. Topology and
self-intersection screens passed; no input changed. This supports convex-hull
filling as the explanation at these three points, without rewriting the
original confusion counts or claiming a global coverage certificate.

Residual overapproximation is visible elsewhere: static `joint_0172` has a
link0/hand sphere penetration of **−0.0318332353 m** while every official hull
pair is separated (minimum **+0.00329772061 m**). Online `joint_0961` has a
hand/obstacle-1 sphere penetration of **−0.0205173929 m** with reference
minimum **+0.00216884038 m**. Thus removing the old q7 obstruction does not
make the new model exact or remove all false obstacles.

The reference has 60 static and 400 online candidate pairs. The new spheres
have 268 static and 928 online pairs; all 33 × 20 robot/obstacle pairs are
present. This still differs from the paper's reported 1,187 pairs, and no
filters were changed to force that count. All distances, deepest pair names,
per-body disagreements and empty exception files are retained in the result
directory. `geometry_validate.log` records the actual execution.

## Actual supplementary trajectory replay

`reproduction/replay_constructed_geometry.py` inspected every saved FR3
`trajectory_samples.npz`, including failed solver candidates, after the timed
main benchmark and throughput runs ended. It excludes non-finite-time witness
rows only from sample selection, and chooses the nearest saved sample to 17
fixed uniform targets on the **saved parameter/time axis**, recording selected
values, offsets, original row indices and duplicate selections. Polygon and
distance-NTO axes are normalized parameters, not physical seconds; each method
records its axis semantics. Duplicate timestamps and midpoint ties select the
earliest original row. No curve interpolation is introduced.

The replay reads the run's frozen source/asset/config and each case's original
obstacle inputs; it applies the same official geometry, SRDF and fixed-base
online policy. Full reference pair distances, deepest pairs, collisions,
original selected flags, exceptions and missing-trajectory statuses are saved
separately. It never rewrites main-run flags. This bounded sampling is additional
numerical evidence, not continuous trajectory certification.

The actual smoke and full commands were:

```bash
bash local/python.sh -m reproduction.replay_constructed_geometry \
  --run-dir reproduction/runs/20261026T025000.439432Z_paper_constructed_fr3 \
  --output reproduction/reports/paper_constructed/official_replay_smoke \
  --trajectory-limit 1

bash local/python.sh -m reproduction.replay_constructed_geometry \
  --run-dir reproduction/runs/20261026T025846.407684Z_paper_constructed_fr3 \
  --output reproduction/reports/paper_constructed/official_replay
```

Both returned exit code 0. The independent
[`replay_commands.json`](replay_commands.json) preserves commands, source hashes,
start/end times, exit codes and separate log paths. Smoke checked one of five
pilot trajectories, with `incomplete_scope=true`; its 17 selected rows were
reference-free. Smoke is not substituted for full coverage. The full invocation
took 43.629 s wall time in the current environment; this is diagnostic runtime,
not a paper performance measurement.

The [full summary](official_replay/summary.json) reports 1,911 of 1,911 saved
trajectories checked across all 400 cases, zero query errors and
`incomplete_scope=false`. The 2,000 requested case/method combinations include
89 without a saved trajectory; their original statuses remain in
`trajectory_inventory.json`. They are not counted as free paths. All 32,487
selected rows were distinct within their trajectories, with no duplicate
selection or non-finite-time witness exclusion in this run.

| Method | Saved trajectories checked | All 17 selected rows reference-free | Trajectories with a reference collision | Exact-primitive collision points | Hull-only candidate points |
|---|---:|---:|---:|---:|---:|
| LSCS | 393 | 386 | 7 | 0 | 7 |
| LGCS | 339 | 333 | 6 | 0 | 6 |
| SCSTO | 393 | 385 | 8 | 0 | 8 |
| NTO distance | 393 | 344 | 49 | 23 | 137 |
| NTO time | 393 | 358 | 35 | 7 | 68 |

These denominators include every saved candidate, irrespective of solver or
main validation status. In particular, NTO distance's saved results comprise
188 `success_numerical`, 157 `validation_failed` and 48 `optimization_failed`;
NTO time has 344, 28 and 21, respectively. LGCS includes 338 successful and
one validation-failed candidate. All 393 saved LSCS and SCSTO candidates carry
`success_numerical`. Absence of a saved trajectory is separately retained.
Thus the table is not a solver success rate, main CUDA CFR, or paper-reproduction
success rate.

An exact-primitive collision point has a negative-distance pair containing no
mesh hull, so it is numerical collision evidence for the declared boxes/spheres.
A hull-only point has no such exact-primitive witness; its original non-convex
mesh contact remains unresolved. The three static candidate configurations
resolved above are a separate fixed diagnostic and cannot resolve these newly
observed trajectory points by analogy. No point was within the `1e-8 m`
near-contact band. Positive reference distances establish separation only at
the checked rows, not between them or for the physical robot.

All five saved-axis bookkeeping controls passed in both invocations: earliest
row for duplicate-time ties, exclusion of non-finite-time witnesses, recorded
short-trajectory duplicates, rejection of nonmonotonic axes, and rejection of
nonfinite configurations at finite axis values. The manifest source-map and
configuration digests, all frozen source files, official geometry inputs,
scenes, case inputs, original result records and absent metadata were verified.
[Final input reverification](official_replay/input_reverification.json) passed
with no changed or missing input and no change in the trajectory file set.

The [original-triangle diagnostic](mesh_candidate_diagnostic.md) also completed
after the timed work, with all 15 analytic controls passed, no query failures,
no unresolved fixed configuration and a successful final input identity check.
It requires one closed component per body, checks boundary witnesses outside
shared triangle simplices, and conservatively rejects ill-conditioned geometry.
Both supplementary analyses retain their numerical limitations and leave all
main experiment results unchanged.
