# Constructed paper-condition benchmark: experimental contract

This contract separates published requirements, implementation evidence, and choices needed to construct missing inputs. It does not identify the constructed model or scenes as the authors' original assets. The earlier results and source snapshots remain separate. This audit ran no GPU kernels, IK, planners, or trajectory solvers; its numerical task-distribution observations come from reading the ten already saved v4 scenes.

The primary specification is [arXiv:2504.10783v1](https://arxiv.org/pdf/2504.10783v1), submitted 2025-04-15. Page numbers below are one-based PDF pages: the environment and Fig. 6 are on p. 8, methods and experiment sizes on p. 9, Table I on p. 10, and supplementary parameters/Tables III–VI on p. 14. The saved [author TeX source](https://arxiv.org/src/2504.10783v1) supplies secondary evidence; comments are draft material and do not override the formal PDF. Local source identities, the read-only sampling audit, and source URLs are in `source_evidence.json`.

## 1. Published conditions to preserve

| Component | Required condition | Primary location |
|---|---|---|
| Forest | 2D square of side 10; 15 circles of radius 0.35; centers in a central square of side 7; bottom-left to top-right task | p. 8, VII-A1, Fig. 6 |
| FR3 | 7 joints; 33 robot spheres; one box table; 20 spherical obstacles in front; right-side starting configuration and left-side target pose | p. 8, VII-A1, Fig. 6 |
| Experiment grid | 10 environment seeds × 10 roadmap seeds × 4 sizes = 400 cases per environment and approach | p. 9, VII-A5 |
| Forest sizes | 200, 400, 800, 1600 roadmap nodes | p. 9, VII-A5 |
| FR3 sizes | 3000, 6000, 9000, 12000 roadmap nodes | p. 9, VII-A5 |
| Offline DRM | Uniform free configurations; node, adjacency, pose and voxel collision maps; k=10; symmetric adjacency; task/configuration distance thresholds 10; voxel side 0.06 m | V; Appendix B; p. 14, Table V |
| Voxel interpretation | Circumscribing radius √3·s/2 for side s. Native `offline_voxel_resolution` is a half-side, hence 0.03 for s=0.06 | Appendix B and public source API |
| Online DRM | Apply current obstacles to collision map; initialize goal-pose IK from nearby collision-free pose-map nodes; A*, lazy edge checks, greedy shortcutting | pp. 6–7, V |
| Lazy checking | Step 0.1 m for Forest, 0.1 rad for FR3 | p. 14, Appendix D |
| Forest EI-ZO | δ=.05, ε=.01, Np=1000, at most 10 faces/iteration, 30 mixing steps, stepback .01 | p. 14, Table III |
| FR3 EI-ZO | δ=.005, ε=.005, Np=10000, at most 10 faces/iteration, 60 mixing steps, stepback .01 | p. 14, Table IV |
| Minimum distance | Sum of Euclidean segment lengths, fixed start/end configurations; DRM PWL warm start for NTO | pp. 8–9, (14), VII-A3 |
| Minimum time | Minimize duration, bounded velocity and acceleration, zero start/end velocity; fourth-order B-spline warm start from DRM nodes for NTO | p. 9, (15), VII-A3 |
| NTO solvers/counts | Forest distance SNOPT/Nc=20, Forest time SNOPT/Nc=100; FR3 distance SNOPT/Nc=150, FR3 time IPOPT/Nc=150 | p. 14, Appendix D |
| LSCS / LGCS / SCSTO | LSCS SOCP; LGCS uses two inflated paths and a graph-of-convex-sets MICP; SCSTO alternates convex subproblems for minimum time | p. 9, VII-A4; VI-C |
| Collision recovery | Validate trajectory, refine sets with collision counterexamples, reoptimize; preserve/recover coverage of the initial PWL path | pp. 7–8, VI-B, Fig. 5 |

Table V prints a length unit for the configuration threshold even for FR3. Preserve the numerical value 10 and document the native joint-space Euclidean interpretation; do not silently convert a joint threshold into meters.

The EI-ZO limit of 30 iterations comes from commented author TeX, not Tables III/IV. The public implementation's nine bisection steps and the recommended τ=.5 are separate provenance items. A capped iteration count and configured δ/ε alone do not establish that every returned set passed the statistical stopping test.

## 2. Missing inputs and explicit construction choices

The paper does not publish sphere placements/radii, complete pair filters, obstacle radii/distribution, table dimensions, exact seed values, numerical task boxes, target orientation law, simulation velocity/acceleration limits, or all solver settings. The following are the present constructed choices, not recovered author settings:

| Item | Declared choice |
|---|---|
| Official kinematics/limits | Franka description revision `39f8c0bcea9bc48ae5334edcb2aa2a504f2d2c85` (2025-04-11), source SHA checks retained |
| Gripper | Fixed 0.04 m finger opening on each side, official hand/TCP transforms |
| Table | Size [1.4, 1.6, .1] m, center [.5, 0, −.05] m; surface z=0 |
| Obstacles | 20 spheres, radius .05 m; independent centers uniform in [.25, −.4, .15] to [.75, .4, .8] m; overlap allowed |
| Right-side TCP box | [.2, .15, .2] to [.8, .65, 1] m |
| Left-side TCP box | [.2, −.65, .2] to [.8, −.15, 1] m |
| Coordinate convention | Robot-base axes; the configured right side is +y and left side is −y. Figures must state the viewing direction, because the paper gives no signed coordinate definition. |
| FR3 collision-map workspace | Size [1.5, 1.8, 1.2] m, center [.5, 0, .55] m |
| Forest endpoints | [−4, −4] to [4, 3]; numeric coordinates were not published; point is represented by radius 10⁻⁶ m |
| Goal pose family | FK pushforward of joint-uniform samples conditioned on the left TCP box and declared collision-free model; target pose remains fixed during online IK |
| NTO collision locations | Exactly Nc interior points k/(Nc+1), k=1,…,Nc. Count follows the formal PDF; the uniform interior locations are constructed. |
| NTO warm start details | Order-2 distance spline and order-4 time spline; one additional copy of each endpoint for time standstill; no arclength resampling. Unpublished details remain labeled choices. |
| Dynamic limits | Per-joint absolute velocity/acceleration 1; the hardware section states these bounds, but the simulation section does not |
| Numerical/budget choices | 1e−5 m endpoint margin; 10 online IK attempts; 200000 endpoint-draw limit; configured 60 s solver limits for LGCS/NTO; 240 s case watchdog shared by all methods; 2000 NTO iterations; degree-6 SCSTO; relative tolerance .01; 20 recovery iterations |
| Validation | Configuration step .005 and at least 2001 samples for trajectory checks; endpoint/feasibility tolerances 1e−5; derivative checks as implemented; sampled checking is not continuous certification |
| Runtime deviations | Installed csdecomp 0.1.2/modern Drake and solvers; Clarabel substitutes for unavailable MOSEK; licensed Gurobi MISOCP attempted with license failures retained |

The formal PDF describes Nc collision-check points. Commented TeX used k/Nc with k=1,…,Nc−1, which gives one fewer constraint. The constructed package now deliberately uses the formal count; the old v4 implementation and results retain their original grid. Neither grid may be described as an independently recovered author runner.

## 3. The 33 spheres and 1187/527 pair counts

The published arithmetic is exact: 1187−527=660=33×20. The constructed policy therefore explicitly includes **all 33 robot spheres in dynamic obstacle checking**, including spheres on the fixed base. A geometry being active for collision checking does not make its kinematic frame movable. Native geometry IDs and Drake obstacle filters must implement the same 660 sphere/obstacle pairs. Base/table filtering remains governed by the static fixed-body policy.

The remaining 527 cannot be uniquely reconstructed from that arithmetic:

- There are C(33,2)=528 unique unordered robot-sphere pairs before any physical filtering. Treating 527 as pure robot self-collision would exclude only one pair and leave the table outside that count.
- Treating 527 as robot self-collision plus all 33 sphere/table pairs would imply 494 robot/robot pairs and 34 exclusions from C(34,2)=561. This is an interpretation, not evidence of the authors' implementation.
- The paper-era public parser iterates only pairs with `geom_B > geom_A`, excludes same-body/adjacent/fixed-chain pairs, and skips world/world pairs. Its source does not justify explaining the number by silently counting directed duplicates.
- With 33 spheres distributed over at most eight rigid components, even the most balanced allocation has at least 52 within-component pairs. Excluding only those leaves at most 476 robot/robot plus 33 robot/table = 509 unique pairs. With seven moving rigid components the corresponding upper bound is 499. These conditional bounds show why the actual author representation or count convention is needed; they do not establish a unique correction to the paper.

**Acceptance:** report the actual unique robot/robot, robot/table, robot/obstacle, and excluded-pair inventories with link names, rigid components, and reasons. Keep exactly 660 dynamic checks. Do not duplicate checks, remove legitimate geometry, invent moving joints, or alter SRDF exclusions to force 527. Record the static and total counts as an unresolved model difference when they differ.

Source: [paper-era parser](https://github.com/wernerpe/csdecomp/blob/ee94f8145bb32ef6c08b1e575218335b9daf533f/csdecomp/src/cpp/urdf_parser.cpp#L504), especially its unordered-pair loop and pair exclusions. The published counts are in p. 8, VII-A1; the table remains explicit geometry in that paragraph.

## 4. Geometry construction and acceptance independent of planning success

Use the prepublication official FR3 geometry and kinematics as a documented reference. Preserve all 33 positive-radius spheres and the chosen fixed opening. Freeze the fitting objective, sphere allocation, constraints, source mesh hashes, and calibration samples before observing benchmark planning outcomes. Keep all attempted fits and reasons for selecting the final geometry. A better success rate is not an acceptable geometry-selection criterion by itself.

The official [arm coarse geometry](https://github.com/frankarobotics/franka_description/blob/39f8c0bcea9bc48ae5334edcb2aa2a504f2d2c85/robots/common/franka_arm.xacro), [capsule macro](https://github.com/frankarobotics/franka_description/blob/39f8c0bcea9bc48ae5334edcb2aa2a504f2d2c85/robots/common/utils.xacro), and [hand geometry](https://github.com/frankarobotics/franka_description/blob/39f8c0bcea9bc48ae5334edcb2aa2a504f2d2c85/end_effectors/common/franka_hand.xacro) provide an alternative constructive reference. Capsule length is the cylinder length/endcap-center separation. For a capsule of radius r and length L, n≥2 equally spaced centers including both endpoints with radius sqrt(r²+[L/(2(n−1))]²) cover the entire capsule continuously. A chain with unchanged radius r generally leaves gaps. Transform the link7 coarse frame by its explicit Rz(π/4) before using local capsule coordinates. Covering coarse capsules does not prove coverage of the detailed collision meshes.

The official [Franka collision documentation](https://github.com/frankarobotics/franka_ros/blob/develop/franka_description/doc/index.rst) distinguishes detailed convex geometry from coarse capsules. The audit should report reference agreement, coverage, and excessive expansion separately. Native/Drake agreement on the same sphere model verifies implementation consistency, not physical fidelity. Vertex/face-center coverage alone is sampled evidence. A convex-hull reference can create collision candidates where a nonconvex surface would not collide, so label that scope explicitly.

Required checks before main measurement: FK agreement; source limits and fixed transforms; exact sphere count; matching static pair filters; all 33 dynamic IDs; home/reference-pose contact inspection; saved independently sampled joint/workspace configurations; full confusion counts against the geometric reference; and source/asset hashes. Do not discard samples because a sphere model or planner dislikes them. Report near-contact cases and strict collision sign conventions. Rechecking fixed audit configurations is permitted; selecting a model from final benchmark success is not.

A targeted public-asset check found [StanfordASL/frax's released 22-sphere Panda model](https://github.com/StanfordASL/frax/blob/cc0a066b5b539c729838308496ae1b8a1d6aeada/frax/assets/franka_panda/bubblify/panda_spherized.yml). It is not the missing FR3 33-sphere model. Its [modeling documentation](https://github.com/StanfordASL/frax/blob/cc0a066b5b539c729838308496ae1b8a1d6aeada/docs/modeling_collision.md) is useful engineering context; no unverified Panda→FR3 substitution is authorized by that resemblance. Modern [cuRobo fitting code](https://github.com/NVlabs/curobo/blob/main/curobo/examples/getting_started/build_robot_model.py) likewise provides a fitting approach, not the 2025 paper's asset.

## 5. Task-distribution audit and recommendation

The copied v4 generator samples joint configurations uniformly, tests collision freedom, and accepts TCP positions in the declared side boxes. It selects the first accepted right configuration and the FK pose of the first accepted left configuration. Therefore the target distribution is a reachable-pose distribution induced by joint space; it is neither uniform in Cartesian position nor uniform over SO(3). It guarantees a witness configuration for the target under the sampled model. The saved witness must remain provenance only; do not secretly use it as online IK initialization or replace the fixed pose after an IK/planning failure.

Reading the ten old scenes gave target heights 0.378118–0.986438 m. Eight of ten target TCP z axes had positive world-z components; all ten angles to world-down were greater than 60 degrees. This differs from a specifically down-facing grasp-task family, but the paper only describes a random pose. The unlabelled single example in Fig. 6 does not establish an orientation law. Right/left TCP classification also does not imply that every arm link lies on that side; the text does not impose that stronger condition.

**Primary comparison:** compare methods within each fixed v5 case, using the same declared task boxes and reachable-pose family. Label this a constructed tabletop traversal family. v5 simultaneously changes the sphere geometry, dynamic participation of the fixed base, NTO collision-point grid and endpoint random streams. A comparison with v4 is therefore not a controlled model-only ablation or a same-endpoint paired comparison. Do not attribute a change in overall success rate entirely to geometry repair. If a downward-looking manipulation family is desired, preregister a separate position/orientation distribution and seeds before running it; do not silently narrow the orientation distribution to obtain successful plans.

The copied v4 generator shared one RNG stream between obstacle centers and joint proposals, accumulated batches of 2048, and collected eleven left candidates although it used only the first. Its nominal 200000 proposal cap could reach 200704. The constructed generator has now been inspected after correction: obstacle centers retain `SeedSequence([250410783, seed])`; start and target use independent `SeedSequence([250410783, 5, seed, 1])` and `SeedSequence([250410783, 5, seed, 2])` streams. Each side has a strict 200000-draw cap, with each batch limited by the remaining allowance. The first free configuration satisfying the corresponding TCP box is used. Batch draw counts, free points inspected, stream identifiers, and exhaustion are saved. The pose is not resampled in response to method outcomes. These are explicit construction choices.

The current 0…9 seeds have already been observed in v4 and must not be described as held-out test data. Reusing the obstacle stream preserves the obstacle environment for a given index; reusing a roadmap RNG seed does not preserve accepted roadmap nodes when collision geometry changes. The new endpoint streams do not preserve v4 endpoints. Cross-version results are descriptive comparisons of different constructed protocols. Within v5, methods can be paired on their common case inputs and validated outcomes. An additional unobserved seed grid can be frozen for new evaluation, with no replacement based on results. The proposed example grid is environment seeds 100…109 and roadmap seeds 200…209; this is a recommendation, not a claim that the current configuration has adopted it.

## 6. Selection, validation, budgets and reporting

1. Freeze configuration, geometry, collision filters, runtime, seed list and method order before results. Use identical fixed scene/pose tasks and the same resolved joint endpoints across methods within a case. Different roadmap-seeded IK solutions are allowed across cases because the task is a fixed pose. Record that distinction.
2. Generate all declared environments, including disconnected ones. Endpoint rejection for the preregistered free-endpoint family is part of the task definition; rejection after planning/optimization success is observed is forbidden. Keep endpoint exhaustion, IK failure, no-path, invalid lazy path, inflation failure, license failure, timeout and validation failure.
3. The TeX contains an unused commented suggestion to choose seeds whose DRM succeeds. Do not import that suggestion into the new data generation. The formal paper conditions Table I on DRM success; reproduce that conditional denominator and also report all 400 scheduled cases and every attrition stage.
4. Distinguish native DRM success from independently validated DRM success. Never inflate a known-colliding seed merely to improve completion. If NTO receives such a seed for a diagnostic, keep this explicit and do not count that method under the same validated-seed denominator without qualification.
5. Save solver status and independent feasibility separately. A usable result needs finite data, endpoints, joint limits, collision checks, velocity/acceleration constraints where applicable, and the documented validation tolerances. Solver success alone is insufficient. Preserve failed candidates and per-stage times.
6. The frozen implementation uses a **240 s total-case watchdog covering all five methods**. LGCS and NTO configure 60 s solver limits. LSCS and SCSTO have no separate 60 s solver limit and rely on the shared watchdog; recovery can invoke further solves. There is no 60 s whole-method guarantee. Fixed order can censor later methods. Analysis records an interrupted attempt with `interrupted_no_completed_timing` and later methods as `not_run_due_to_timeout`, separately from solver failures. Report this limitation and its observed incidence. Future protocols may use isolated per-method budgets, but must not relabel the present run. Fixed method order and common EI-ZO seed 1337 are explicit choices, not recovered author settings.
7. Report offline map construction separately from online IK, collision-set creation, A*, inflation, optimization, recovery and independent validation. Keep process warm-up scope explicit. No parallel GPU/solver workload during timing. Modern hardware/runtime timings cannot substantiate the original paper's speedup directly.
8. Compare costs on explicitly identified common-success subsets within the same v5 case and expose the denominator. Use environment-level clustering for uncertainty if added: the 40 roadmap conditions per environment are repeated measurements, not 400 independent environments. Do not treat shared numeric seed labels as matched endpoints or matched roadmaps across v4/v5. A future controlled ablation must explicitly hold those inputs and every other changed setting fixed.
9. Gurobi license errors remain failures of the available execution environment. A complete enumerated-path SOCP diagnostic can be reported separately, with completion/optimality limits; it is not the paper's MICP timing or an automatic replacement for blocked LGCS.
10. The separate historical-core build and its three mechanism probes do not retroactively change the version used by the earlier 800 main cases. Pin the core selected for each new run. A historical API lacking recovery cannot silently stand in for the modern recovered pipeline.

The 15 physical KUKA executions with three D415 cameras, changing perceived obstacles and their hardware timings remain outside constructed FR3 simulation. A public URDF assembly or simulated execution cannot supply those physical observations. Consequently the defensible deliverable is a fully specified and actually executed **constructed reproduction of the public simulation conditions**, with deviations and failures retained; equality with all original experiments is not established by construction alone.

## 7. Read-only implementation findings at this snapshot

The constructed optimizer now implements exactly Nc constraints at the declared new locations. The native/Drake adapter explicitly restores robot/online-obstacle pairs including the fixed base, preserving kinematics. Those changes meet the corresponding contract items.

The separate endpoint streams and strict cap have also been implemented and inspected. The 0…9 list is explicitly a fixed constructed list, and interrupted/unstarted method states are already separated in analysis. The subsequently completed [construction record](model_construction.md) and [geometry audit](geometry_audit.md) identify final attempt `model_construction_04`, with 268 static and 660 dynamic candidate pairs. Their saved evidence establishes the stated surface coverage and fixed-pose findings, while retaining unresolved hull-only collision candidates. Observed benchmark timeout attrition still comes from the actual case summaries. `finger_opening` describes the **fixed 0.04 m gripper** implemented by the URDF; it is not a supported adjustable degree of freedom. A future configurable gripper would require corresponding source changes and a new frozen run. `source_evidence.json` records the original audit snapshot and subsequent report-review checks separately.
