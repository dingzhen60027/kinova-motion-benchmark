# Task distribution and saved-path diagnosis

This is a read-only analysis of all 800 scheduled v5 cases. It runs no planner,
optimizer, CUDA kernel or model-selection procedure. It preserves every original
case and recomputes geometric quantities from the saved arrays. The new script
uses the existing Conda environment through the project launcher:

```bash
bash local/python.sh reproduction/diagnose_task_distribution.py
```

The successful execution verified 3,200 input files unchanged by SHA-256 and
passed three analytic controls (diameter intersection, tangent line and separated
line). The file count and full audit values are recorded in `summary.json`;
the identities are in `input_sha256.json`. All calculated values below come from
this execution. Empirical spread in the JSON uses population standard deviation.

## 1. Distance NTO only succeeds on straight-path cases in both environments

| Environment | Native DRM single-segment paths | Strict NTO-distance success on those paths | Multi-segment paths | Strict NTO-distance success on multi-segment paths |
|---|---:|---:|---:|---:|
| Forest | 120 | 120 | 280 | **0** |
| FR3 | 188 | 188 | 205 | **0** |

FR3 additionally has seven DRM failures for which no NTO candidate exists. These
are not included in the 393-path classification. The equality in the table is
an observed per-case relation, not merely matching totals: see `cases.csv` and
`summary.json` → `nto_distance_by_seed_segments`.

In Forest, all 120 successes come from environment seeds **1, 4 and 5**, each
repeated across 10 roadmap seeds and four sizes. Their fixed start-to-goal lines
are already collision-free. The minimum exact direct-line clearances are
141.078, 2.506 and 6.601 mm, respectively. The other seven scenes require a
detour. Across their 280 cases there are 265 solver-successful colliding
candidates and 15 solver failures. Consequently the reported 30% is completely
determined by the three direct-path scenes; it is not evidence of successful
nontrivial obstacle-avoidance optimization.

The 400 cases contain only ten independently generated Forest environments.
Forty roadmap variants of one scene must not be treated as 40 independent
obstacle draws when interpreting this concentration. No scene or endpoint has
been changed in response to these findings.

## 2. Forest distance-NTO failure is between collocation points

For all 400 saved distance-NTO candidates, the diagnostic reconstructs the
order-2 spline directly from its control points and knot vector. It evaluates
the **actual saved 20 constraint locations**, and analytically computes the
minimum distance of every continuous line segment to every obstacle circle.
It does not invoke Drake to obtain these distances.

For the **265 solver-successful but invalid** candidates:

- Every candidate satisfies all sampled collision constraints to within
  **1e-6 m**. The smallest pointwise clearance across all candidates is
  **−2.3814e−8 m**, substantially smaller in magnitude than the saved SNOPT
  feasibility tolerance of 1e−6.
- Exact continuous penetration ranges from **9.290 to 342.662 mm**;
  its median is **83.792 mm**. These new values are analytic PWL minima, so
  they can be slightly deeper than a previous dense-sample minimum.
- The largest arc-length gap between adjacent constraint locations within a
  candidate has median **0.936 m**, range **0.507–1.736 m**. The obstacle
  diameter is 0.7 m. Uniform spline-parameter spacing is not uniform physical
  spacing after control points move.

Thus the recorded large collisions are compatible with successful solution of
the submitted pointwise program: the constraints miss collision intervals.
This isolates a concrete transcription/validation gap; it does not show that
the native checker or SNOPT simply ignored the submitted constraints. Nor does
it establish that the authors used this particular knot vector, clearance or
collocation placement. The formal paper gives Nc but does not release all
these choices. Every per-case interval and constraint-point clearance is in
`forest_nto_distance_checks.json`.

The corresponding FR3 relation (zero strict distance-NTO successes for all
205 multi-segment seeds) is a strong reason to check the same mechanism there.
This report has not analytically certified FR3 swept motion or re-evaluated its
nonlinear distance constraints, so it does not assert the Forest geometric
proof applies unchanged to the robot.

## 3. All 12 invalid Forest DRM paths fit the 0.1 m lazy-check gap

The independent exact line-circle check reproduced the saved minimum clearance
for every Forest seed path to within 1e−12 m. The 12 invalid paths penetrate
by **0.147–2.606 mm**. Each has one colliding segment/obstacle interval;
the interval arc length is **20.267–85.269 mm**, shorter than the published
100 mm sampling step.

The current native `SteerFunctionIsSafe` samples in float32 from the segment
start at multiples of `online_edge_step_size`; it does not adapt to obstacle
clearance. Replaying those exact sampling positions in NumPy on the saved
final path yields positive analytic clearance for all 12 paths. The minimum
sample clearance ranges from **0.012–2.122 mm**. This is a CPU emulation of
sample locations, explicitly not another native collision-check invocation.

| Case | Exact penetration (mm) | Minimum lazy-sample clearance (mm) | Collision interval length (mm) |
|---|---:|---:|---:|
| e00_r00_n00800 | 1.081916 | 0.787512 | 54.997184 |
| e00_r06_n01600 | 2.606430 | 0.083577 | 85.269289 |
| e02_r01_n00200 | 0.431264 | 1.642664 | 34.739027 |
| e02_r07_n00200 | 0.254387 | 2.122401 | 26.683812 |
| e06_r03_n00400 | 0.488750 | 0.499589 | 36.980377 |
| e07_r00_n01600 | 1.356005 | 0.196858 | 61.558674 |
| e07_r02_n00800 | 1.518313 | 0.644609 | 65.131158 |
| e07_r02_n01600 | 1.518313 | 0.644609 | 65.131158 |
| e07_r06_n00800 | 2.158642 | 0.012332 | 77.624579 |
| e07_r08_n00400 | 1.946497 | 1.435974 | 73.722810 |
| e07_r09_n00800 | 0.146725 | 0.296333 | 20.266874 |
| e09_r01_n00800 | 2.535337 | 0.557081 | 84.102632 |

The 12 skips of each geometric method therefore preserve the EI-ZO precondition
that its seed is collision-free. They should not be relabeled EI-ZO failures.
The 0.1 m value matches the paper; denser checking, exact-circle checking or
edge rejection/replanning would be a declared robustness change, not a recovered
unpublished author parameter. A controlled comparison should retain the original
0.1 m arm as well as any repaired arm.

## 4. Path-segment counts: shortcutting is already in the paper

The formal paper, §V-B step 3, explicitly uses A*, lazy checks and **greedy short
cutting**. The current adapter sets `try_shortcutting=True`; the current native
code returns the shortcut path. The recorded statistic is exactly
`len(returned_path)-1`, verified against all 793 saved successful DRM paths.
Neither a pre-shortcut path nor its node IDs were saved. Therefore a no-shortcut
counterfactual cannot be reconstructed from these artifacts without replanning.

| Statistic | Paper Forest | Saved Forest | Paper FR3 | Saved FR3 |
|---|---:|---:|---:|---:|
| DRM line-segment mean | 3.45 | 2.140 | 4.49 | 1.850 |
| Saved vertex mean | not specified | 3.140 | not specified | 2.850 |
| DRM path length | 10.90 m | 10.853 m | 8.55 rad | 6.421 rad |
| Single-segment path fraction | not specified | 120/400 | not specified | 188/393 |

Table I's paper DRM line-segment means are exactly one larger than its LSCS
convex-set means (3.45 vs 2.45, 4.49 vs 3.49). This coincidence raises a possible
vertex/segment naming ambiguity, but is **not proof of an off-by-one error**:
one convex set can cover multiple adjacent segments. Even under the unverified
vertex-count interpretation, the FR3 gap remains 4.49 vs 2.85 and the path-length
gap remains unchanged. Do not relabel the current statistic to claim agreement.

FR3's current task/model family contains many direct paths and shorter
joint-space traversals. This is observed, not a demonstrated causal attribution
to any one sphere model, pose distribution, seed or planner version. Matching
the paper's average by selecting favorable scenes would not recover its task
distribution.

## 5. Other geometric-method failures

`geometric_failures.json` preserves all non-successful geometric-method records
and the relevant saved validation fields.

- **54 FR3 LGCS failures** are restricted Gurobi license failures, not measured
  infeasibility. Saved program sizes and exceptions remain available.
- Four LGCS candidates (three Forest, one FR3) pass native and independent
  collision checks but fail region membership: residuals are
  **1.1900e−5, 1.3251e−5, 1.8374e−5 and 2.8584e−5** against the unchanged
  1e−5 threshold. Their endpoints, limits and derivative checks pass. The
  current recovery loop exits when there are no collision points, so it cannot
  repair membership-only numerical violations.
- Forest `e03_r03_n00400` SCSTO fails at the released `scsplanning` fixed-velocity
  subproblem's `assert result.is_success()`. Its four input regions and seed
  path were saved. No solver result, iteration-state curve or detailed solver
  termination status was exposed before the assertion, so a more specific
  numerical cause cannot be recovered from this run. Instrumented replay is
  needed; a blanket claim of infeasibility would be unjustified.

## 6. Paper conditions and next controlled steps

The local primary paper is `reproduction/paper/2504.10783v1.pdf`; its text is
`reproduction/paper/paper.txt`. Relevant specifications are §V-B for shortcutting,
§VII-A1 for geometry/tasks, §VII-A3 for NTO transcription, Table I for reported
statistics, and Appendix D/Tables III–V for counts, solver choices and lazy step.

The current Forest shape, obstacle count/radius and central sampling square
match the disclosed conditions. Exact endpoints **[−4,−4] → [4,3]**, random seed
values and RNG family are construction choices; the paper only states bottom
left to top right. It does not require independently randomized endpoints.
The current FR3 7-DOF/33-sphere/20-obstacle structure and right-to-left pose task
match the disclosed high-level structure, but original sphere placements,
pair filters, obstacle radii/distribution and task-pose distribution remain
unrecovered. The detailed existing `experimental_contract.md` remains the
record of these differences, not this diagnosis's newly inferred settings.

The warranted order is: (1) hold saved tasks/DRM paths fixed and diagnose NTO
collocation/knot/control-point and clearance choices with explicit paired arms;
(2) retain 0.1-step evidence while validating a declared remedy for invalid
seed paths; (3) instrument the saved SCSTO numerical failure and tighten solver
feasibility for LGCS without loosening validation; (4) only after transcription
is sound, preregister new independent task distributions rather than selecting
scenes to fit published success rates. No new performance claim can be made
from this CPU-only audit.
