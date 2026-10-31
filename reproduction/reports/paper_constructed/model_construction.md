# FR3 33-sphere construction and geometry audit

The final model is a newly constructed approximation of official FR3 collision geometry under the user's authorization. It is **not the authors' unpublished 33-sphere model**, and matching the sphere count does not recover that model. Selection used only geometry; the home configuration, the previous q7 regression, the 1184-pose independent audit and planning results were not used to select the final asset.

The selected asset is `reproduction/paper_constructed/assets/fr3_33.json`, SHA256 `77b602396d93004c0aef6cf3776ef5783ed6fff9b08c0734829d3b9d2c1bd120`. Its frozen builder SHA256 is `f0f0f10e29a91d7af65f80c8ebca733821a595840e28e22470dfd81a9ad80bc3`. All original v4 code, assets, reports and raw results are retained.

## Why the earlier fit created a false obstacle

The earlier fit used unweighted unique STL vertices, Lloyd means and a cluster-maximum radius plus 2.5 mm. This does not minimize sphere volume, limits neither local outward excess nor interlink false obstacles, and does not ensure continuous coverage between vertices. Its two link5 spheres implicated in the earlier home collision had radii about 90.45 and 93.07 mm. The old home spheres reported 35.16 mm penetration between link5 and hand, while the official collision-mesh convex hulls had 37.05 mm positive separation. The old hand also missed 27 source-vertex/face-centroid probes in the present geometric audit. These facts justify reconstructing the fit from geometry rather than reducing radii or suppressing the link pair.

Original evidence remains in `reproduction/reports/paper_completion/diagnostics/pipeline/mesh_comparison/results.json`. No new collision filter or joint-limit change was introduced by this builder. The final saved verification explicitly checks equality of source metadata, kinematics, limits, disabled collision links and limit provenance with the v4 asset.

## Fixed construction and selection rule

1. Read the nine official STL collision meshes and the eight official finger boxes. The mesh bytes are those already verified equivalent to the prepublication official description; the inherited source manifest and prepublication joint-limit provenance remain in the asset.
2. Bisect each triangle's longest edge until every edge is at most 20 mm. Fit a sphere enclosing every vertex of each assigned subtriangle with a minimum-enclosing-ball convex optimization, then recompute its actual maximum vertex distance and add 10 µm. The buffer protects numerical containment; it is not a scaled-down collision radius.
3. Generate one through eight spheres per link using two deterministic initializations and iterative nearest-center partitions, plus deterministic recursive geometric splits. Recursive splits examine principal/coordinate axes and fixed area quantiles (0.2, 0.35, 0.5, 0.65, 0.8); choose the split minimizing the sum of the two child-sphere volumes. No pose configuration enters this calculation.
4. Select exactly 33 spheres by dynamic programming minimizing the summed sphere volumes over the saved candidate library. This is exact selection within that finite library, **not a global optimum over all possible sphere unions**. The volume objective was unchanged after the first attempt; recursive geometric candidates were added before the independent pose audit was read.

The sum of sphere volumes is an upper bound on union volume because balls overlap; it is not the robot's occupied volume. It is not sufficient by itself to prevent a large local envelope. The audit therefore reports the sampled maximum and 95th-percentile outward distance for each link. The current selection does not impose an outward-distance cap, so residual false obstacles remain measurable limitations. The geometry source and collision filters were not adjusted to improve acceptance.

The selected allocation is links 0–7: **5, 4, 3, 2, 2, 6, 2, 2**, hand: **3**, fingers: **2 + 2**, totaling **33**. Root integration includes all 33 robot spheres in online checks despite five spheres belonging to the fixed base. The actual model now has 268 static candidates and 660 online sphere/obstacle pairs with 20 obstacles, totaling 928. These counts differ from the paper and the earlier constructed model.

## Actual geometric measurements

Summed sphere volume changed from **0.07190549 m³** to **0.03528970 m³**, a **50.92%** reduction. The final complete construction ran in **16.518 s** on the existing environment. This time is a model-construction diagnostic, not a paper algorithm timing result.

Outward distances use 512 deterministic Fibonacci sphere-surface points per ball. The maximum is a sampled lower bound on the true worst outward deviation; p95 is over the equal-count sphere samples, not an area-weighted quantile. Distances are to the source mesh surfaces with three-ray inside classification; finger inclusion uses the exact union of the four boxes. `geometry_audit.json` additionally records exact maximum supporting-halfspace violation of each mesh convex hull, ray disagreements and topology checks.

| Link | Balls | Old sum volume (cm³) | New sum volume (cm³) | Old sampled max outward (mm) | New sampled max outward (mm) | New sampled p95 outward (mm) | Uncovered / inside volume probes |
|---|---:|---:|---:|---:|---:|---:|---:|
| link0 | 5 | 12696.35 | 7717.96 | 90.79 | 56.00 | 36.40 | 0/1771 |
| link1 | 4 | 10423.76 | 4969.47 | 74.01 | 32.73 | 22.76 | 0/1831 |
| link2 | 3 | 11499.41 | 5012.89 | 65.52 | 36.97 | 26.45 | 0/1842 |
| link3 | 2 | 9504.17 | 3288.77 | 67.90 | 33.51 | 24.40 | 0/1215 |
| link4 | 2 | 8992.91 | 3388.71 | 64.01 | 33.50 | 25.30 | 0/1229 |
| link5 | 6 | 9943.46 | 4457.97 | 65.82 | 31.30 | 22.65 | 0/1227 |
| link6 | 2 | 4346.46 | 3075.03 | 51.66 | 44.51 | 34.11 | 0/2052 |
| link7 | 2 | 2762.25 | 1105.76 | 54.76 | 31.74 | 25.41 | 0/1849 |
| hand | 3 | 1581.89 | 2157.24 | 48.13 | 34.81 | 28.53 | 0/2445 |
| leftfinger | 2 | 77.42 | 57.95 | 16.58 | 16.40 | 10.43 | 0/1897 |
| rightfinger | 2 | 77.42 | 57.95 | 16.58 | 16.40 | 10.43 | 0/1897 |

The hand's summed volume increases because its new spheres cover the source surface that the old hand approximation missed, while its sampled maximum outward deviation decreases. It would be misleading to describe the old smaller volume as equally complete coverage. Link5's sampled maximum outward deviation decreases from 65.82 to 31.30 mm. Neither measurement guarantees that every pose is free of approximation-induced collision.

## What coverage is and is not established

All **17177** subdivided triangles pass containment after reloading the serialized centers and radii. Every triangle's three vertices lie inside one convex sphere with at least approximately 10 µm margin; therefore the entire triangle lies in that sphere. Subdivision preserves the original triangular surface. This establishes a continuous **source-surface** coverage statement under float64 geometry and the recorded numerical tolerance.

This is **not a continuous solid-volume enclosure certificate**. A union of balls covering a closed surface can leave an interior cavity. Furthermore, exact edge-incidence checks detect non-two-incidence edges in link0 and link6, so their raw STL parity tests should not be treated as rigorous solid membership proofs. The asset explicitly keeps `continuous_solid_coverage_certified=false` and `continuous_mesh_cover_certified=false`.

For each link, 4096 fixed Sobol bounding-box points were checked. The three-ray majority test (or exact finger box membership) classified **19255** points as inside; **zero** of these were uncovered by the selected spheres. The same bounding-box samples contain **25873** points inside the convex hulls, of which **788** were not covered. Those hull points can lie in filled mesh concavities or gaps between finger boxes; they do not establish 788 actual mesh-solid misses. The raw counts are retained per link rather than hidden. Continuous solid containment remains unproved.

The analytic unit-cube enclosing-ball check and independent reloading of every selected triangle bound passed in `model_construction_04/verification.json`. All 8,200 saved enclosing-solver status records in the final candidate files indicate success; histories repeat some records, so this is not a count of distinct optimizer invocations.

## Independent held-out geometry audit

After the asset hash was frozen, the independent audit executed its preregistered **1184 configurations**: home, 31 old q7 poses, 1024 uniform joint samples and 64 samples from each workspace side. It used the official mesh convex hulls and exact finger boxes, the official SRDF exclusions and strict distance < 0 collision decisions. No configurations or thresholds were changed in response to this result.

| Condition | Both free | Both collide | Sphere false positive | Hull-collision / sphere-free candidate | Query failures |
|---|---:|---:|---:|---:|---:|
| Static table | 1053 | 110 | 18 | 3 | 0 |
| Same poses with 20 online spheres | 780 | 364 | 38 | 2 | 0 |

Home and all 31 old q7 configurations are free in both models in the static condition. In the fixed online scene, home collides in both models and all 31 q7 poses remain free in both. The independent audit found zero misses involving only exact primitive reference pairs. The 3/2 reference-negative candidates involve mesh convex hulls: hull penetration alone is not proof that the original nonconvex mesh solids collide. They remain unresolved collision-approximation candidates and are retained with all pairwise distances. Positive hull separation, in contrast, certifies separation of the original enclosed meshes, so the sphere false positives are meaningful approximation artifacts.

Detailed immutable evidence: `geometry_audit/validation_20261026T024911Z/summary.json`, `input_identity.json`, both row tables, pair-distance NPZ files and failure lists. This model fixes the specific earlier home/q7 false obstacle and reduces geometric excess, while still exhibiting 18/38 held-out sphere false positives. It must not be described as an exact geometry replacement or complete author-asset recovery.

## Execution and retained unsuccessful attempts

All fitting used the existing Conda launcher `bash local/python.sh`; no package installation or environment change was made. The initial `trimesh` import failed with `ModuleNotFoundError`, which is retained in each preflight record; the actual builder uses existing NumPy and SciPy. Commands are also listed in `model_construction_commands.json`.

| Attempt | Actual outcome | Saved candidate files | Evidence |
|---|---|---:|---|
| 01 | Completed; initial two-strategy library; 0.03887203 m³; finger allocation 6+6 exposed an inefficient candidate library | 176 | `model_construction/` and `model_construction_execution.log` |
| 02 | Failed: a single-triangle cluster had no legal recursive split; all completed records and traceback retained | 168 | `model_construction_02/` and matching execution log |
| 03 | All geometric candidates/audits completed; failed at serialization because relative report path was used with absolute ROOT | 264 | `model_construction_03/` and matching execution log |
| 04 | Completed after the two implementation fixes; selected final model without reading held-out audit results | 264 | `model_construction_04/` and matching execution log |

The first successful asset is retained as `model_construction/selected_asset.json`. Each attempt retains the exact builder snapshot and manifest. The final attempt retains every candidate's centers/radii, sphere volumes, solver statuses, geometric triangle assignments, and the original/subdivided mesh arrays. The failed attempts are not relabeled as completed.

The actual final command was:

```bash
bash local/python.sh -m reproduction.paper_constructed.build_assets \
  --report-dir reproduction/reports/paper_constructed/model_construction_04 \
  > reproduction/reports/paper_constructed/model_construction_04_execution.log 2>&1
```

The report directory is intentionally required to be fresh to preserve old records. Model fitting and geometry verification were completed before the main constructed benchmark began. No fitting rerun or additional batch geometry workload was started during those timings.
