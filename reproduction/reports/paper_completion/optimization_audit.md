# Optimization transcription audit

Date: 2026-10-25. Scope: the five trajectory-optimization methods in Section VII-A and Appendices C/D of arXiv:2504.10783v1. This audit changes only the new `paper_completion` implementation; the previously measured `paper_protocol` implementation and observations remain frozen.

Final numerical file: `reproduction/paper_completion/optimization.py`, SHA-256 `dbce28518dc6d94052391397fe42a84a75d4f647e4ec5fb893c0ac64c7a520ec`.

## Evidence and interpretation

The published PDF was text-extracted and its Appendix C/D page was visually inspected. The downloaded author's original TeX is archived under `source_evidence/arxiv_text/sections/`; `experiments.tex` contains the published method definitions, while the end of `appendix.tex` contains a **commented, unpublished NTO derivation**. That comment is stronger evidence than an invented implementation choice, but does not establish what the unpublished experimental runner actually executed.

| Method | Evidence | Implemented transcription | Remaining difference |
|---|---|---|---|
| LSCS | Appendix C; Section VII-A4 | Exact SOCP for minimum Euclidean polygonal length through the supplied ordered convex sets | Clarabel replaces unavailable MOSEK |
| LGCS | Section VII-A4 references Marcucci et al. Eq. 5.5 | Perspective-flow MISOCP on a split entry/exit graph, with exact elimination of redundant coordinates | Original authors' graph-construction code and Gurobi settings unavailable; local restricted license still blocks larger problems |
| SCSTO | Section VII-A4; released `scsplanning` source | Pinned author's `biconvex` implementation, unchanged | Simulation bounds, Bezier degree and tolerance were not fully specified in this paper |
| NTO distance | Published Section VII-A3 and commented Appendix derivation | Original DRM nodes as order-2 controls, fixed duration 1, summed adjacent control-point distances; actual Drake minimum-distance constraint | Original exact knot/solver options and clearance not released |
| NTO time | Section VII-A3 and Eq. 15 | Order-4 controls retain each original DRM node; repeat the first and last node once to initialize zero endpoint velocities; actual velocity/acceleration bounds and endpoint-rest constraints | Endpoint copies and initialization duration are declared mathematical reconstruction, not recovered author code |

The published Eq. (19d) visibly reads `v_i = v_(i+1)` for every segment. Applied literally together with distinct fixed endpoints this would make every benchmark infeasible. The implementation follows the surrounding definition of a polygonal shortest path: one shared knot at each adjacent-set intersection, both endpoints of each segment inside its convex set. The erroneous printed equality is not silently presented as a working algorithm. Appendix C also does not explicitly repeat goal membership in its final set; our segment-containment form enforces it.

## NTO correction

The old v3 minimum of eight controls and arclength resampling have been removed. Every input DRM node is retained exactly in the warm start. Distance optimization has exactly the original node count. Time optimization has the original node count plus two endpoint copies. For a clamped cubic spline the initial derivative vanishes when the first two controls and last two controls coincide. Adding these copies preserves every interior roadmap node and avoids forcing a three-node detour into a four-control straight-line family. The paper does not describe this endpoint treatment, so it remains explicitly inferred.

The commented author TeX gives collision sites `k/Nc` for `k = 1, ..., Nc-1`; the new profile implements this formula literally. Thus the paper parameter `Nc=20/100/150` creates **19/99/149 actual constraint bindings**, respectively, rather than including both endpoints. Candidate records now distinguish `collocation_parameter_Nc` from `collocation_points`, and save every actual location. Known endpoints and their numerical validation remain independent of these optimization bindings. The formal published body says Nc points, while the comment informally also calls Nc the number of constraints. This conflicts with the comment’s explicit Nc−1-site index range; choosing the archived formula is a declared interpretation, not proof that the original runner used 19/99/149 constraints.

Each saved candidate contains original nodes, actual warm-start controls, endpoint-copy count, resampling=false, initial duration, and collocation provenance. The original controls are serialized into the candidate NPZ by the existing array writer.

Unpublished choices retained and declared: clamped uniform knots; zero clearance bound and 0.01 influence offset; time initialization equal to original polyline length plus 2 seconds; duration range 0.05–100 seconds; unit joint velocity/acceleration limits; 2000 nonlinear iterations and 60-second solver limit; solver tolerances 1e-6. None was selected to match the published success percentages. The time warm start need not initially be collision-free or satisfy all derivative limits; it is an optimizer starting point, not a validated solution.

## LGCS construction and equivalence

[Marcucci et al., primary Eq. 5.5 source](https://groups.csail.mit.edu/robotics-center/public_papers/Marcucci21.pdf) uses binary edge flows, homogeneous convex-set membership, and simultaneous scalar/vector flow conservation. Our graph has entry and exit vertices for each region. The internal edge charges Euclidean length; transitions between intersecting regions enforce coincident endpoints at zero cost. Eliminating the internal-edge endpoint coordinates and flow leaves transition points `z_e`, binaries `phi_e`, and region displacement variables. This is an algebraic projection of that formulation on the stated split graph, including its continuous relaxation.

For transition `e=(i,j)`, both `A_i z_e <= b_i phi_e` and `A_j z_e <= b_j phi_e` are enforced. Bounded regions ensure `z_e=0` on inactive edges. Source/target edge points are the fixed endpoint times their flow. Region displacement is the sum of outgoing transition points minus the sum of incoming points; its norm is bounded by a nonnegative length epigraph. Scalar flow conservation and degree at most one choose one source-to-target path. Disconnected zero-cost cycles may occur, as allowed in the referenced formulation; any such edges are recorded and excluded from the recovered path. This does not change the optimal source-to-target cost.

The prior v3 integer formulation used geometric big-M continuity implications and order variables. It represented the same union-path objective at integral solutions but did not implement the referenced perspective relaxation. The new formulation therefore improves methodological fidelity; it is not merely a renamed big-M model. No claim is made that the unpublished author's exact graph representation was recovered.

Solver settings are recorded in each answer: Gurobi, one thread, relative MIP gap 1e-6, 60 seconds, `NonConvex=0`, `Presolve=0`. Gurobi 13's default presolve failed to recognize the SOC on the predetermined two-corridor test. `PreQLinearize=0`, `Aggregate=0`, and explicit finite displacement bounds were actually tried and failed; their logs remain. Disabling presolve preserved the explicit convex cones and passed the oracle. Gurobi documents that presolve can destroy recognizable convex structure and that disabling it can resolve this, with a possible performance penalty. [Gurobi convexity guidance](https://support.gurobi.com/hc/en-us/articles/4419622498065-Why-does-Gurobi-report-that-my-convex-model-is-non-convex)

A further exact elimination of displacement auxiliaries was also tested, both directly and with continuous copies of endpoint flows. Gurobi rejected the resulting generalized quadratic representation under `NonConvex=0`; neither draft is used. Their source snapshot and failure logs are retained. The production implementation keeps standard explicit SOC displacement variables. No nonconvex solver mode was enabled to conceal these failures.

## License inspection

Read-only probes found Drake MOSEK available but disabled, and Drake's Gurobi binding unavailable. No MOSEK/Gurobi license environment variables were present. No license file contents or secrets were read. The existing `gurobipy` license executed real small MISOCPs and reported restricted non-production use, expiration 2027-11-29.

For quadratic models this license allows **200 variables**, rather than the general linear limit of 2000 variables; 2000 linear constraints also apply. [Official license limits](https://support.gurobi.com/hc/en-us/articles/360051597492-How-do-I-resolve-a-Model-too-large-for-size-limited-Gurobi-license-error). The exact perspective formulation cannot in general remove this external restriction. License failures retain code 10010 and actual model dimensions; no decomposition is relabeled as the paper's Gurobi MISOCP.

## Actual verification

Commands, using the existing Conda interpreter through the project launcher:

```bash
bash local/python.sh -m py_compile reproduction/paper_completion/optimization.py
bash local/python.sh -m pytest reproduction/tests/test_paper_completion_optimization.py -q
bash local/python.sh -m reproduction.verify_completion_optimization
```

Final tests: **4 passed in 0.58 seconds**, with one third-party NumPy warning while enumerating an infeasible candidate corridor. The tests cover:

1. All simple corridor permutations of a fixed two-route box problem as an independent SOCP oracle; LGCS matches both the enumeration optimum and analytic path length 4.2.
2. Both conic planners attain the analytic 7-DOF single-region optimum `sqrt(7)`.
3. Real Forest SNOPT distance/time solves pass independent validation; warm starts preserve nodes and the author's commented collocation formula. Initial cubic endpoint derivatives are zero.
4. A disconnected pair of convex sets reports infeasible instead of returning a fabricated path.

The helper selected the **first three lexicographically ordered frozen FR3 cases with saved LGCS region artifacts**, before running either formulation. These are mathematical diagnostic checks, not additional paper benchmark observations. It loads exactly the archived regions/endpoints, records every source hash, and retains candidate arrays, solver metadata and failures.

| Frozen case | Old big-M objective | New Eq. 5.5 projection | New variables / linear constraints |
|---|---:|---|---:|
| e00_r00_n03000 | 7.5977871438 | License 10010 | 204 / 1473 |
| e00_r00_n06000 | 7.3131175483 | License 10010 | 236 / 2028 |
| e00_r00_n09000 | 6.8251144503 | Optimal, 6.8251133836 | 180 / 1377 |

The third new solve has gap 0 and maximum set-membership residual `8.88e-16`; its objective differs from the old solve by `1.07e-6`, consistent with the old relative optimality gap. The diagnostic's raw output is `optimization_checks/results.json`. These checks do **not** establish that all graph sizes fit the available license, that collision validation of the original seed model was repaired by the optimizer, or that whole-paper numbers have been reproduced.

All initial PSD-recognition failures, restricted-license failures, successful tests and failed elimination drafts are retained beside this report. Numerical code is frozen at the hash above before the parent experiment runner proceeds.
