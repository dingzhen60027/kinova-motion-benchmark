# Original-triangle diagnostic for the remaining link5/hand candidates

Status: **executed successfully after the timed benchmark and throughput runs**.
All 15 analytic controls passed. All three fixed configurations were numerically
separated in the original triangle meshes, with no containment and no unresolved
configuration. The input SHA inventory passed its final verification. These
float64 numerical findings do not constitute an exact-predicate proof.

The scope is the three unique configurations already saved in
`geometry_audit/validation_20261026T024911Z/`: `joint_0608`, `joint_0673` and
`workspace_left_38`. The first two recur in the online condition, producing the
original three static plus two online candidate records. The script checks
these identities and their exact configurations rather than searching for
additional examples or selecting favorable results.

`diagnose_constructed_mesh_candidates.py` uses the official prepublication STL
triangles and each configuration's actual link5/hand world transforms from the
pinned official kinematic chain. It verifies the saved asset and oracle
registration hashes. No sphere parameters, filters, task inputs or benchmark
flags are changed. The original binary STLs contain 266 link5 triangles and 200 hand triangles.
Both passed the executed single-component, closed/oriented-manifold and
numerical self-intersection screens, with no degenerate, ill-conditioned,
duplicate, boundary or nonmanifold faces/edges identified.

## Numerical method and decision boundaries

No new library was installed. Lightweight module availability checks found no
`trimesh`, `python-fcl`, `Open3D` or `libigl`; NumPy, SciPy and Drake are present.
The new script implements a small transparent numerical diagnostic:

1. Surface distance is the minimum over triangle pairs, using vertex/face and
   edge/edge distances plus explicit segment/face intersection checks. The
   edge-distance problem includes all four boundary optima and its SVD-based
   interior stationary point. AABB lower bounds prune pairs that cannot improve
   the current minimum. A proper transverse crossing provides a zero-distance
   witness; coplanar overlap and boundary contacts remain distinguished.
2. Each original mesh is checked for degenerate/duplicate faces, boundary and
   nonmanifold edges, consistent orientation, manifold vertex links, connected
   components and nonzero component volume. Welding uses exactly identical STL
   coordinates only; no tolerance weld, repair, reorientation or face removal is
   applied. A numerical self-intersection screen also examines transverse
   crossings, coplanar positive-area overlap and nonadjacent near contacts.
   It collects all edge/face witnesses, including endpoints and coplanar edge
   proximity. For triangles sharing vertices or an edge, every witness is
   checked against that shared simplex; contact outside it, or beyond a tight
   roundoff band around it, remains unresolved. It does not ignore all contact
   merely because a pair shares a vertex.
3. Positive surface distance alone does **not** establish disjoint solids.
   A containment or separation conclusion requires **one closed component in
   each mesh**. Multiple components remain `unresolved_component_solid_semantics`
   because independently oriented components can cancel total winding, and
   union/cavity semantics are not inferred. For a single component, one
   deterministic face-centroid representative is tested against the other
   entire mesh in both directions. Under the closed embedded-surface assumptions,
   a component cannot switch between inside and outside without crossing the
   other surface. The numerical
   topology/self-intersection checks and their limitations are saved rather
   than assumed away. The winding formulation follows the primary
   [generalized winding-number work](https://users.cs.utah.edu/~ladislav/jacobson13robust/jacobson13robust.html).

All predicates use floating-point arithmetic, not exact or interval arithmetic.
The `1e-8 m` contact band marks an unresolved result; it does not relax any main
collision threshold. Noninteger/unstable winding, malformed topology,
self-intersection-screen issues, near contact and exceptions remain explicitly
unresolved. Adjacent-face self-intersection screening is not formal embedding
certification. Deeply ill-conditioned or adversarial triangle arrangements may
need an independently validated exact-predicate library later.

Possible results are numerical transverse intersection, numerical closed-mesh
containment, numerical separation of the original triangle meshes, or a named
unresolved condition. A separation result would support convex-hull filling as
the explanation for these particular candidates; an intersection or containment
result would retain evidence of a sphere-model miss or a geometry inconsistency.
Neither outcome establishes global sphere coverage, exact equivalence to the
physical robot, or correctness of every trajectory.

## Controls and preserved evidence

Fifteen analytic synthetic controls are run before any real candidate calculation:
parallel triangle distance; skew-interior, parallel and zero-length edge
distances; proper transverse intersection; coplanar overlap; closed tetrahedron
topology/self-screen; positive-distance nested tetrahedron containment; outside
winding; rejection of an open mesh; unresolved opposite-orientation multiple
components; a shared-vertex boundary crossing away from that vertex; a valid
shared edge; a slender triangle interior crossing; and conservative rejection
of a numerically ill-conditioned triangle. Barycentric coordinates use
dominant-axis projected signed areas instead of a cancellation-prone Gram
determinant. Faces with area/squared-edge ratio at most `1e-12` prevent a
reliable-separation conclusion. The script stops and preserves results
if any control fails. **All 15 controls passed in the standalone execution
and again before the real candidate calculation.**

Every invocation creates a new output directory. It records commands, script
and source hashes, numeric parameters, all topology/self-screen findings,
synthetic controls, candidate provenance, signed convex-reference witnesses,
original configurations, actual world transforms, posed triangle arrays,
closest triangle/point witnesses, winding values/residuals and exceptions.
A unified input SHA inventory is captured before real geometry calculations
and rechecked at the end, including both scripts, original asset, row files,
STLs, official inputs and registration. Input drift prevents a valid summary;
partial per-case outputs then remain unverified. Cases are not dropped because
their result is inconclusive. No original
experiment file is overwritten.

## Actual commands and results

The independent ledger [`mesh_diagnostic_commands.json`](mesh_diagnostic_commands.json)
records exact commands, start/end times, elapsed wall time, exit codes, source
SHA-256 and log paths. Both actual invocations returned exit code 0; no failed
attempt was replaced or omitted. The standalone controls took 0.412 s and the
three-candidate invocation took 4.221 s in this environment. These are diagnostic
execution times, not paper performance measurements.

```bash
bash local/python.sh -m reproduction.diagnose_constructed_mesh_candidates \
  --self-test-only \
  --output reproduction/reports/paper_constructed/mesh_controls_20261026T034235.956727Z

bash local/python.sh -m reproduction.diagnose_constructed_mesh_candidates \
  --validation-dir reproduction/reports/paper_constructed/geometry_audit/validation_20261026T024911Z \
  --output reproduction/reports/paper_constructed/mesh_candidate_diagnostic_20261026T034309.558905Z
```

| Fixed configuration | Original-mesh surface distance (m) | Convex-reference signed distance (m) | Numerical original-mesh finding |
|---|---:|---:|---|
| `joint_0608` | 0.041468693684562345 | −0.0005010781674290929 | Separated; both containment tests outside |
| `joint_0673` | 0.03441026573427353 | −0.0005418788432862131 | Separated; both containment tests outside |
| `workspace_left_38` | 0.034295723322812076 | −0.0011313457785132686 | Separated; both containment tests outside |

The first two configurations each account for a static and an online occurrence,
so these three results cover the five pre-existing candidate records. Neither
mesh had a self-screen issue: 1,902 link5 triangle pairs and 1,394 hand pairs were
examined in the self screens. The six containment winding values had absolute
magnitude at most 8.84e-18. The closest-distance searches considered 53,200
possible cross-mesh triangle pairs per configuration, with 570, 530 and 485
pairs requiring full distance tests after AABB pruning, respectively.

The actual [summary](mesh_candidate_diagnostic_20261026T034309.558905Z/summary.json)
reports `completed_numerical_diagnostic`, 3 unique configurations,
5 original occurrences and 0 unresolved configurations. The
[input reverification](mesh_candidate_diagnostic_20261026T034309.558905Z/input_reverification.json)
reports no changed or missing input. Posed original triangles, closest point
witnesses, topology findings, winding values, original configurations and exact
transforms are preserved beside that summary.

These observations support convex-hull filling of concavities as the explanation
for these particular link5/hand disagreements. They provide no evidence of an
original-mesh overlap at these three fixed points. They do not prove global
coverage by the reconstructed spheres, establish exact physical-robot contact,
or validate all saved trajectories. The main experiment flags and the original
convex-reference confusion tables remain unchanged.
