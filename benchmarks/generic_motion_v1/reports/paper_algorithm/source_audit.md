# RSS 2025 EI-ZO algorithm source audit

Audit date: 2026-10-28. Baseline inspected: repository commit `2c7578a3b57b03eb0f16ab3407dc2847cb4c99ce`, before the concurrent paper-algorithm changes. This is a source and parameter audit, not an experimental-results report. No GPU experiment was run for this audit. Implementation and execution outcomes must be established by the separate run artifacts.

The intended scope is the paper's **single-path DRM → EI-ZO → LSCS / SCSTO → collision-recovery pipeline**, applied to the existing frozen Kinova benchmark. It is an algorithm reproduction on an adapted benchmark, not a recovery of the authors' FR3 benchmark or a reproduction of all paper experiments.

## 1. Authoritative sources and provenance

1. **Published paper:** Peter Werner, Richard Cheng, Tom Stewart, Russ Tedrake, Daniela Rus, *Superfast Configuration-Space Convex Set Computation on GPUs for Online Motion Planning*, RSS 2025, DOI [10.15607/RSS.2025.XXI.045](https://doi.org/10.15607/RSS.2025.XXI.045). [Proceedings record](https://www.roboticsproceedings.org/rss21/p045.html), [official PDF](https://www.roboticsproceedings.org/rss21/p045.pdf).
   - The official PDF was downloaded successfully on the audit date: **6,908,376 bytes**, **14 pages**, SHA-256 **`e61c094de19de72481ac480c37c1717bb9d145c675a9b3fc71066ea0fc726c57`**.
   - Temporary inspected copy: `/tmp/werner25-official-rss-audit.pdf`. The PDF contains raster page content; `pdftotext` does not recover the paper text. Pages 4–9 and 13–14 were rendered and visually inspected. Algorithm 1, the practical stopping/step-back discussion, single-path coverage and recovery, solver descriptions, and Tables IV–VI were checked against the published pages rather than inferred solely from arXiv HTML.
   - Temporary files are not permanent evidence storage. The URL, hash and byte count above identify the inspected official artifact. Earlier `/tmp/csdecomp_rss2025.pdf` and `/tmp/csdecomp_rss2025_alt.pdf` are HTML responses, **not** this PDF.
2. **Searchable preprint:** [arXiv 2504.10783v1 HTML](https://arxiv.org/html/2504.10783v1), [PDF](https://arxiv.org/pdf/2504.10783v1), [source archive](https://arxiv.org/src/2504.10783v1). The checked algorithm and parameter passages agree with the official pages above; this audit does not claim a complete byte-level or whole-paper identity.
   - Existing local PDF: `reproduction/paper/2504.10783v1.pdf`, SHA-256 `579c9917425fe01b583f067d0a1c9702917ccd810584dbac562fcdf9fead51d0`.
   - Existing source archive: `reproduction/reports/paper_completion/source_evidence/arxiv_2504.10783v1.tar.gz`, SHA-256 `18c4140e37eb05abe636dd4999612618f565c120ce57fbe966e39545b68f1986`.
   - Author comments in the source are **draft evidence**, below the published text and tables in authority. They are not recovered executable experiment scripts.
3. **Author implementations:** [CSDecomp](https://github.com/wernerpe/csdecomp), especially `csdecomp/src/planning/cuda_edge_inflation_zero_order.{h,cu}`, `cuda_polytope_builder.cu`, `cuda_set_builder_utils.cu`, `cuda_edit_regions.{h,cu}`, `roadmap.h`, and `roadmap_builder.cpp`; [SCSTrajopt implementation](https://github.com/TobiaMarcucci/scsplanning/tree/d3f885aa2b74a10418de96e61ec2df0bf6ef16b8).
   - Local SCSTrajopt vendor revision: `d3f885aa2b74a10418de96e61ec2df0bf6ef16b8`. `git rev-parse --show-toplevel` identifies the independent vendor repository, its remote is `https://github.com/TobiaMarcucci/scsplanning.git`, and its working tree is clean. The installed Conda package's `direct_url.json` points to this vendor checkout. Installed `polygonal.py` and `biconvex.py` match both vendor files and `git show REV:path` contents: respectively SHA-256 `a8b44ede8e553a87451871dc4f59fc90fb98e26c04adc709ffbfb7d8eb1d1473` and `d1fb6585b77bf8b0eda4d18c3cedaa383a9e15904e2479813283345268c19e27`. No revision was inferred by running Git in `site-packages`.
   - The current CSDecomp release is not the paper-date binary. The existing `reproduction/reports/paper_completion/source_audit.md` records the paper-date public commit `ee94f8145bb32ef6c08b1e575218335b9daf533f` and the later v0.1.2 revision. This audit does not silently equate those versions.

Evidence collection included the following actual commands; these were document/source inspection, not planner execution:

```bash
git rev-parse HEAD
curl -L --fail --connect-timeout 10 --max-time 40 \
  https://www.roboticsproceedings.org/rss21/p045.pdf \
  -o /tmp/werner25-official-rss-audit.pdf
sha256sum /tmp/werner25-official-rss-audit.pdf
/usr/bin/pdfinfo /tmp/werner25-official-rss-audit.pdf
/usr/bin/pdftoppm -f 14 -l 14 -scale-to 2000 -png -singlefile \
  /tmp/werner25-official-rss-audit.pdf /tmp/werner25-official-rss-14
```

The MIT mirror request failed with curl exit 35 (`SSL_ERROR_SYSCALL`); the official proceedings download succeeded subsequently. No hash from a failed or HTML response was substituted for the official PDF hash.

## 2. EI-ZO parameter mapping

The published **FR3 simulation** configuration is Table IV, official PDF p. 14. These algorithm parameters can be applied unchanged to Kinova, but doing so does not make the robot, collision space or input distribution identical.

| Published quantity | Native `EizoOptions` field | Existing `evaluation.json` | Paper-aligned setting | Evidence |
|---|---|---:|---:|---|
| Admissible uncertainty δ | `delta` | 0.01 | **0.005** | Table IV |
| Admissible colliding volume fraction ε | `epsilon` | 0.01 | **0.005** | Table IV |
| Maximum samples to update Np | `num_particles` | 10000 | **10000** | Table IV |
| Maximum faces per iteration Nf | `max_hyperplanes_per_iteration` | 10 | **10** | Table IV |
| Hit-and-run mixing steps | `mixing_steps` | 100 | **60** | Table IV |
| Maximum step back Δmax | `configuration_margin` | 0.1 | **0.01** | Table IV; §IV-C |
| Test threshold τ | `tau` | 0.5 | **0.5** | §IV-B footnote 2, p. 4: typical choice |
| Bisection count Nb | `bisection_steps` | 9 | Explicit implementation choice, e.g. **9** | §IV-C gives a rule of thumb, not an experimental value; native default is 9 |
| Iteration cap Nit | `max_iterations` | Omitted; native default 500 | Explicit choice, with provenance | §IV-C allows a cap; published table has no value. Author arXiv source comment gives **30**, `sections/appendix.tex:138–144` |
| Random seed | `seed` | Set by the runner | Record the actual run seed | No original experimental seed list was recovered |

The source-comment value 30 must be labelled `author_source_comment`, not `published_Table_IV`. Likewise, `Nb=9` must not be presented as a recovered experimental setting. The draft's `offline_voxel_resolution=0.1` conflicts with the published voxel table; it must not override Table V.

Algorithm 1 and §IV-B, pp. 4–5, require an iteration-dependent uncertainty δk = 6δ/(π²k²), test sample count M = ceil(2 log(1/δk)/(ετ²)), drawing max(Np,M) samples, testing the first M samples, projecting colliding samples to the seed segment, bisection, and adding nonredundant separating planes nearest to that segment first. Native `executeEizo` implements this structure. `num_particles` is not the total batch size or the statistical-test sample count.

The initial domain is the robot joint-limit box, an explicitly supported paper choice. `configuration_margin` is a **configuration-space halfspace step back**, not the workspace voxel padding or a guaranteed physical obstacle clearance. The step back is relaxed when required to retain the seed endpoints (§IV-C, equations 11–12; native `maybePlaceFace`).

Stopping before the statistical test accepts voids the stated probabilistic region guarantee (§IV-C, p. 5). Native `CudaEdgeInflator.inflateEdge` presently returns the polytope without an exposed acceptance/iteration-limit status; its local `EizoInfo` is only printed when requested. Record the complete resolved options, cap and API limitation. A returned region, or a numerically validated trajectory, is not by itself evidence that the region's statistical acceptance condition was observed. The trajectory validator also does not certify continuous collision freedom.

## 3. DRM mapping and frozen-input boundary

Table V, official p. 14, applies to the simulation DRM construction. Appendix B, p. 13, defines offline voxel side length s and its circumscribing-sphere radius sqrt(3)s/2.

| Quantity | Native/config field | Existing value | Published simulation mapping |
|---|---|---:|---:|
| Maximum task-space distance | `max_task_space_distance_between_nodes` | 0.85 | **10** |
| Maximum configuration distance | `max_configuration_distance_between_nodes` | 4.0 | **10** |
| Maximum neighbours | `max_neighbors` passed to graph construction | 20 | **10** |
| Offline voxel side length s | `offline_voxel_resolution` | 0.04 | **0.03**, corresponding to s = 0.06 m |
| Lazy online edge-check step | `online_edge_step_size` | 0.01 rad | **0.1 rad** |

The factor of two is essential: `RoadmapOptions` explicitly defines `offline_voxel_resolution` as **half the voxel side length** (`roadmap.h:64–68`); the builder uses a pitch of `2 * offline_voxel_resolution`. Thus 0.04 means an 0.08 m offline grid, and 0.06 would mean 0.12 m. Table V prints metres for configuration distance even though the seven revolute-joint implementation uses the Euclidean norm of joint coordinates; use the numerical threshold 10 in the existing coordinate convention, and do not change joint units to match that printed table label.

New maps must be built for changed construction parameters, with distinct paths and metadata. An old map cannot be relabelled as Table V. The frozen benchmark's online obstacle spheres/voxel representation are distinct from the **offline lookup-grid** parameter; changing the latter does not authorize altering frozen obstacle geometry or collision-pair filters.

For this bounded algorithm reproduction, three independently seeded 6000-node maps form a declared subset of the paper's roadmap scale choices. The paper used four FR3 sizes (3000/6000/9000/12000), ten environment seeds and ten roadmap seeds (§VII-A5, p. 9). Three maps are not that full experimental design. Benchmark map extent, Kinova tool frame, joint sampling margins, random generator and seeds remain explicit adapted choices. Likewise, planning retries, connection-search limits and workspace padding are engineering parameters not recovered from Table V.

## 4. Corridor and recovery implementation contract

Published §VI-A/B, p. 7, and Fig. 5, p. 8, establish the following behavior:

1. Traverse the original PWL path in order. Inflate a segment only when no already constructed region contains it. Keep the relationship between each unique region and the particular segment that seeded it; also keep the coverage assignment for every original path segment.
2. Build a connected region sequence for trajectory optimization. Reusing one region for several contiguous path segments must not create a fresh optimization segment for each identical copy. If reuse is noncontiguous, any sequence simplification must preserve region intersections and original-path coverage; globally deduplicating IDs without checking connectivity is insufficient.
3. Validate the optimized trajectory; assign collision witnesses to the appropriate region(s), refine those regions with the projection/bisection/halfspace steps, then re-optimize.
4. After refinement, test **all original PWL segments** for containment in at least one remaining region. Only the actual seed segment is protected by the construction; any additionally covered segment can become uncovered. Inflate uncovered segments and reconstruct a valid sequence before solving again.

At the audited baseline commit, `csdecomp_baseline.py` appends one region per original edge even when that region was already found in the coverage search. It then associates each copy with that edge during editing. This is a redundant-sequence adaptation, not the compact single-path construction intended by the paper. Its later containment repair/reinflation logic must be redesigned together with region-to-seed tracking; merely deleting duplicate entries breaks the native editor's one-seed-per-region interface.

**Native return-value discrepancy:** `cuda_edit_regions.h` and Python binding documentation describe the second `EditRegionsCuda` return value as segments requiring reinflation. Actual `cuda_edit_regions.cu` returns `(edited_regions, (projections, optimized_collisions))`. These are bisection-search witness data, not the original uncovered path segments. Compute coverage explicitly in the caller; do not pass that second return value to reinflation as though the documentation were correct.

The original runner hardcodes edit margin 0.01, limits collision witnesses to 128 and recovery rounds to 20. Margin 0.01 agrees with Table IV, but using a shared profile value avoids an undocumented mismatch with inflation. The finite caps and witness subsampling are benchmark engineering choices; the paper's recovery description gathers the found collisions per set and repeats the process. Any caps must remain visible and exhaustion must be saved as failure/unfinished work. A bounded endpoint halfspace correction is a numerical adaptation and must not be used to restore an arbitrarily removed additional segment.

The original DRM path is assumed collision-free by the algorithm. Native coarse edge checks do not prove this assumption. A benchmark-level check before inflation can expose invalid seeds and retain an honest frontend failure rather than silently replacing fixed start/goal inputs. Even that finite check is not a continuous certificate.

## 5. LSCS, SCSTO and omitted variants

- **LSCS:** The paper solves minimum Euclidean PWL length through the SCS as an SOCP with **MOSEK** (§VII-A4, p. 9). The installed author `scsplanning.polygonal.get_points` constructs the same standard endpoint/region/second-order-cone problem using **Clarabel**. This preserves the intended optimization problem but changes the solver implementation and timing; name that difference in results. The official paper's Appendix C equation (19d), p. 14, prints `vi = vi+1` for all i, which would contradict distinct fixed endpoints. The arXiv TeX has the same equation. Do not implement this erroneous extra equality: use the working author implementation in which both endpoints of each path segment belong to its region, thereby enforcing intersection membership at shared knots.
- **SCSTO:** The paper uses SCSTrajopt's alternating convex optimizations with **Clarabel**, including velocity/acceleration limits and stationary endpoints (§VII-A2/A4, p. 9). The installed `biconvex` calls the author's polygonal initialization, `FixedPoints`, and `FixedVelocities`; this is the appropriate author algorithm, not a substitute generic optimizer. The local tolerance 0.01 matches the inspected package default. Degree 6 and the exact tolerance are not specified as experiment values in the EI-ZO paper; record them as implementation settings. Fixed benchmark limits of ±1 rad/s and ±1 rad/s² match the paper's **hardware** description, but the simulation passage does not identify the numerical bounds. They remain declared Kinova benchmark limits, not recovered FR3 simulation limits.
- **LGCS is out of scope:** The paper separately inflates **two DRM paths** and solves a GCS mixed-integer shortest-path problem with Gurobi (§VI-C and §VII-A4). That is not implemented or evaluated by this single-path algorithm reproduction. Neither LSCS nor one-path SCSTO results establish reproduction of the LGCS row.
- **NTO is not silently aligned:** Paper FR3 distance NTO used SNOPT, time NTO used IPOPT, with Nc=150. The existing generic runner uses IPOPT and an enriched minimum-control-point initialization. Those are recorded adaptations. This audit does not establish equivalent NTO tuning or reproduce the paper's reported relative speed/reliability.

The separately executed small-program probes in [environment.json](environment.json), inspected for this audit, report actual successful solves with Clarabel, SNOPT and IPOPT. MOSEK is disabled in the current environment; Gurobi is not compiled into the installed Drake binary. Thus SNOPT is available: retaining IPOPT for the existing distance baseline is an implementation/profile choice, not an unavailable-solver limitation. These probes establish solver usability on the probe problem, not motion-planning success.

## 6. Claims that remain adapted or unsupported

The frozen inputs use a seven-joint **Kinova**, the existing gripper/table/equipment and collision filters, fixed joint-space start/goal pairs, and benchmark obstacle representations. The paper's simulation uses **FR3**, 33 robot spheres, 20 spherical obstacles, 1187 total/527 self-collision pairs, and a random right-side configuration to a left-side task-space pose resolved through IK (§VII-A1, p. 8). The original FR3 sphere placements, complete input list and runner were not recovered by the earlier documented asset audit. Parameter alignment cannot remove these model/distribution differences. The frozen joint goals intentionally bypass the paper's task-space/IK stage.

The paper's KUKA iiwa 7 R800, three RealSense D415 cameras, 35,000-node DRM and 15 hardware trajectories are a separate experiment. Its Table VI distances and Nf=20 must not be mixed into the FR3 simulation profile. No hardware reproduction is claimed here.

The existing 60-second end-to-end budget, process startup/model loading, independent validator, contact band, 0.005 rad validation step and sample limits are benchmark policies. Paper component timings and success rates have different boundaries and denominators. Report both end-to-end failures and any frontend-conditioned diagnostic statistics with their denominators; do not compare a cold full-budget time directly with a published optimization-component time.

This audit supports a precisely named **RSS2025 single-path algorithm, Table-IV/V parameters, adapted Kinova inputs** configuration. It does not establish any new success rate, runtime, recovery rate, path cost, continuous-collision guarantee, or match to the paper's numerical tables. Those claims require the actual saved run artifacts and their independent validation.
