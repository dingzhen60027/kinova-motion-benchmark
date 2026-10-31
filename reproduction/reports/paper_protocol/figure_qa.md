# Figure verification

All three final PDFs were rendered with system Poppler at 120 dpi and inspected
panel by panel and as complete figures. The exports also include editable SVG,
300 dpi PNG and 600 dpi LZW TIFF. These are report figures at their native sizes;
the wide status grid is an appendix-style display, not a journal-sized submission.

Source preflight: **20 PASS, 0 WARN, 0 FAIL**. PDF text audits found minimum glyph
sizes 5.6 pt, 6.5 pt and 7.0 pt respectively, with no glyph below 5 pt.

| Panel | Evidence and statistical treatment | Visual/source check |
|---|---|---|
| Results a | All 400 Forest requests per method; observed validated counts | All labels and bars visible; zero baseline; no hidden failures |
| Results b | All 400 FR3 requests per method | Same scale and full denominator; reconstructed model named |
| Results c | Every observed Forest method time; failures included | Positive log axis; n=391 for SCS and 400 for NTO; no invented zero time for unattempted cases |
| Results d | Every observed FR3 method time; failures included | n=151 for SCS and 195 for NTO; 12 caught exceptions use flagged wall-time fallback; no clipping |
| Results e | All common validated cost pairs, raw observations + median/IQR | n=120/120/178; no iid uncertainty claim; axis and labels clear |
| Results f | Same paired rule for FR3 | n=80/80/109; all outliers displayed; no axis truncation of observations |
| Status grid, 10 panels | Every requested case and all five methods in both environments | Each panel has 10 environment rows × 40 roadmap settings; planning absence, invalid seeds, licenses and errors have separate colors; full legend visible |
| FR3 replay | Lexicographically first case, e00_r00_n03000 | Actual 33-sphere start/goal fit and 20 obstacles; saved TCP curves for LSCS/SCSTO/NTO time; invalid NTO marked red and dashed |

Observed proportions/CDFs show the full finite dataset, so no error bars or
significance tests are implied. Paired cost glyphs show median and IQR, rather
than a confidence interval. The CSV summaries separately provide sample SD and
2000 environment-cluster bootstrap resamples; only observed clusters enter a
method's timing interval. No biological replicates or train/test split apply.

The geometry panel uses at most 300 evenly spaced saved samples per curve for
illustration; the full validation arrays remain unchanged. Sphere surfaces and
points are rasterized inside editable vector figures; labels remain text. The
view shows the scene above the tabletop. No photo, external figure, selective
image adjustment or outcome-based case selection was used.

Source CSV/JSON hashes and selected replay paths are recorded in
`reproduction/figures/paper_protocol/figure_sources.json`. Full raw NPZ hashes
are in the two environment inventories. CDF omissions are only unattempted
methods lacking a numeric runtime; paired cost omissions are exactly the
non-common-validated cases, with counts disclosed in the panels and source CSVs.
