# FR3 environment reconstruction: source and image evidence

This audit uses the paper, its original figure, and public upstream geometry. It does not use Table I target statistics, planner success rates, path lengths, or optimizer costs to select the environment.

## Sources and executed checks

The primary benchmark source is [arXiv:2504.10783v1, Section VII-A1 and Figure 6](https://arxiv.org/abs/2504.10783v1). The stored 14-page PDF places Figure 6 on PDF page 8. `figure6_actual-000.png` is its original 1199 × 1158-pixel FR3 raster, extracted without image editing from PDF image object 553. `author_figure6.pdf` is extracted from the already archived author source member `figures/franka_benchmark_gradient.pdf`; it contains the combined Forest/FR3 figure. The public source, PDF, source figure, table files, and URLs are hashed in `sources.json` and `measurements.json`.

The existing `reproduction/paper/pages/figure6.png` is a page preview, not the original FR3 raster. PDF page 6 contains Figure 3, the DRM illustration; its magenta voxel wall is **not** an obstacle wall in the benchmark. Its extraction is retained separately as `figure3_drm_embedded-000.png` to prevent confusion.

Commands actually run:

```bash
pdfimages -list reproduction/paper/2504.10783v1.pdf
pdfimages -f 8 -l 8 -png reproduction/paper/2504.10783v1.pdf reproduction/reports/environment_reconstruction/fr3_evidence/figure6_actual
/usr/bin/pdftotext -f 8 -l 8 reproduction/paper/2504.10783v1.pdf -
bash local/python.sh reproduction/reports/environment_reconstruction/fr3_evidence/measure_evidence.py
```

The bundled `pdfinfo` and `pdftoppm` initially failed because they require GLIBC 2.38; `/usr/bin/pdfinfo` and `/usr/bin/pdftoppm` worked. This did not affect the original raster extraction. The author [project webpage](https://sites.google.com/view/GPUPolytopes) was read successfully. It identifies the linked [video](https://www.youtube.com/watch?v=BXf0h8ePw0M) as the hardware demonstration. Both the embed and watch fetch returned `Online fetch throttled`; no video frames were inspected or used as evidence.

## What the paper fixes

- Seven robot joints, FR3 model, 33 robot collision spheres.
- Twenty spherical online obstacles in front of the robot; a box represents the table.
- Start is a random configuration on the table's right side; target is a random pose on its left side.
- Figure 6 uses red for the initial robot and blue for the terminal robot. The red gripper is at image right, the blue gripper at image left; intermediate configurations are overlaid.
- The paper gives 1187 total and 527 self-collision pairs. It does not release the sphere placements or pair list. Those counts do not determine the missing geometry.

The paper does **not** specify obstacle radii, exact obstacle sampling support/distribution, table dimensions/transform, right/left workspace boxes, target orientation distribution, start configurations, random seeds, or this figure's camera calibration. One pictured trajectory is not the distribution of 400 tasks. Its end effector orientations do not justify making every target point downward.

## Geometric evidence from the original image

The background white arm, colored trajectory poses, black spherical obstacles, beveled tabletop and three visible table legs can be distinguished at original resolution. No metric ruler or coordinate axes appear.

At least three black sphere regions near image coordinates `(711, 496)`, `(815, 495)`, and `(884, 483)` appear clipped by the tabletop plane. This supports including obstacles with centers close to tabletop height. The previous construction's center height `z >= 0.15 m`, with `r = 0.05 m`, cannot produce spheres intersecting the `z = 0` table plane. It does not establish the author's exact lower bound.

A recorded threshold experiment (`max(R,G,B)<65`, channel spread `<10`, connected area `>150`, image y below 650) finds 15 dark connected regions. This is **not** an obstacle count: overlaps merge spheres, and transparent robot overlays change their colors. Twenty is taken from the paper, not inferred from these components. The stored pixel boxes are useful visual landmarks, not recovered 3D centers.

A manually annotated four-corner table homography places illustrative base-plane candidates near 40–51% of the table width and 15–22% of its depth from the rear edge. The rear corner and base are partly occluded, so these are sensitivity points, **not a statistical confidence interval**. The result supports a base near the middle of the rear side. It cannot alone fix a metric table size, camera, or sphere depth. Full annotation coordinates and the homography calculation are reproducible in `measure_evidence.py`.

## Public table asset with a concrete provenance

The upstream files [table.obj](https://github.com/wernerpe/csdecomp/blob/eb699f7c3070e4510d4eb17056e47878ea985c9d/csdecomp/tests/test_assets/table.obj) and [table.urdf](https://github.com/wernerpe/csdecomp/blob/eb699f7c3070e4510d4eb17056e47878ea985c9d/csdecomp/tests/test_assets/table.urdf) were present in first commit `eb699f7c3070e4510d4eb17056e47878ea985c9d`, dated 2025-02-17. Their current and paper-date checkout hashes are identical:

| File | SHA-256 |
|---|---|
| `table.obj` | `3e2f605cf4fbe58f0c4d3b23a5c9a7bba65781aad759e088d4d0d6fe9879107b` |
| `table.urdf` | `af6db04235e8d67f550c2f97e734a4282d15da7a0ca543d24ef0267e9f97ea7c` |

The OBJ has 84 vertices, bounds `[-0.75,-0.75,-0.996706]` to `[0.75,0.75,0.003294] m`, beveled corners and four legs. Its visual form is strongly consistent with the Figure 6 table, although image similarity does not prove identity. The public URDF intentionally uses a larger `2.5 × 2.5 × 0.2 m` collision box centered at local `[0,0,-0.1]`, rather than the 1.5 m visual square. Do not silently replace its collision shape with the visual mesh bounds. The related public Kinova directive translates this table by `[0.4,0,0]`. This supplies a reproducible public default, **not the missing FR3 directive**.

## Recommended independent reconstruction profile

| Parameter | Recommended value | Evidence class |
|---|---|---|
| Robot base | `[0,0,0]` | Explicit coordinate convention, retained |
| Table visual | Original public `table.obj`, translation `[0.4,0,0]` | Public asset; benchmark identity unconfirmed |
| Table collision | Size `[2.5,2.5,0.2]`, center `[0.4,0,-0.1]` | Public URDF plus related public placement |
| Robot collision spheres | Retain the existing independently audited 33-sphere asset | Constructed substitute; original unavailable |
| Obstacle count | 20 | Paper explicit |
| Obstacle radius | Retain `0.05 m` | Unpublished; construction assumption |
| Obstacle x/y center bounds | Retain x `[0.25,0.75]`, y `[-0.4,0.4]` | Unpublished; a single image does not identify a unique wider box |
| Obstacle z center bounds | `[0,0.8] m` | Include plane-intersecting spheres seen in Figure 6; upper bound retained assumption |
| Start side | Negative y | Explicit FR3 convention: x forward, z up, robot-right is -y |
| Target side | Positive y | Same explicit convention |
| Pose orientation | Do not constrain from a single illustrated pose | No published distribution |

The existing v5 side labels are the opposite (`right` means positive y). Swapping labels in a new profile makes the coordinate convention explicit and supports reproducing the figure's red-right/blue-left presentation. The paper does not supply its axis directions, so this is not recovery of its hidden coordinate frame.

The recommended changes should be frozen before any planner evaluation. Keep the original v5 unchanged, preserve every new generated task, and report endpoint sampling failures without resampling after a solver result. Render the same saved scene with true link transforms, all 33 collision spheres, the public table visual, and the actual collision-box extent so visual plausibility and collision semantics can be inspected independently.

## Remaining uncertainty

Exact author geometry, configuration sampling, target orientations, task seeds, camera and 3D sphere locations remain unavailable. Neither image matching nor matching mean performance establishes recovery of those inputs. The proposed profile improves specific source-based environmental choices; it does not claim exact recovery of the author's benchmark.

## Implemented renderer and actual verification

`reproduction/environment_reconstruction/render.py` now renders the frozen final run into `../preview/`. It rejects ASSET, table OBJ, table URDF, or protocol hashes that disagree with the run manifest. All 14 pinned official source files also match the hashes recorded in the sphere asset. The extra official mesh display is visual context only; it does not alter planning geometry. The render-only FK was compared against the actual Drake plant on 32 fixed random configurations, checking 416 body transforms with maximum absolute error 6.66e-16. Every rendered robot pose contains all 33 collision spheres.

The initial direct invocation of the FK check failed to import the repository package; running it as a module (`bash local/python.sh -m reproduction.reports.environment_reconstruction.fr3_evidence.verify_renderer_fk`) passed. The initial matplotlib render sorted table and arm as whole objects and produced incorrect occlusion; its images remain under `preview_initial/`. A single shared face collection fixed this, as checked in `preview_initial_fixed/` and final `preview/`. The final view uses a front-side camera and tighter visual bounds for clear robot display.

The final viewer was opened in the real in-app browser. Scene selection, all eight visibility controls, drag orbit, wheel zoom and reset were exercised; its accessibility state retained the e07 validation failure and console errors were empty. The browser screenshot API timed out twice during initial verification, so browser pixel screenshots are not claimed. Final PNGs were visually inspected directly. The failed e07 path is red and dashed in both payload and PNG; validated paths are gold. An absent case does not fabricate an IK goal or path. An intentional manifest hash mismatch was rejected without changing any source file. See `renderer_controls.json`, `renderer_fk_validation.json`, and `../preview/browser_verification.json`.
