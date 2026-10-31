# Figure audit

Both figures were generated from the saved 400-case Forest run and visually
inspected at full-image scale after the final render on 2026-10-25. The final
PDFs were also rasterized using system Poppler and those renders inspected.

| Figure/panel | Evidence check | Result |
|---|---|---|
| Forest A | First case by fixed ID ordering; all 15 obstacles; saved DRM, LSCS, SCSTO and failed NTO candidate | Pass; invalid candidate labelled |
| Forest B | All 400 requested cases in every denominator | Pass; 9 invalid seeds retained |
| Forest C/D | All 391 timed attempts per method, including solver/validation failures | Pass; no timing truncation; log axis labelled |
| Case statuses | 5 methods × 10 environments × 10 roadmap seeds × 4 sizes | Pass; 2000 cells and failure states retained |
| Style | Legible labels, no clipped legends, editable text and restrained colors | Pass on PNG inspection |
| Source preflight | Automated source checks | 20 PASS, 0 WARN, 0 FAIL |
| PDF text | Minimum 5 pt requirement | Forest minimum 6 pt; status minimum 7 pt; none below threshold |
| Export | PNG, editable SVG/PDF, 600 dpi LZW TIFF | All generated |

The plots show observed proportions and empirical distributions, without iid
error bars or significance claims. Timing excludes validation and offline/setup
costs, as stated in the report. The figures do not claim the original FR3 or
hardware experiments were reproduced.

Sources: [figure_sources.json](../figures/figure_sources.json),
[figure contract](figure_contract.json), and the final JSON audits in `../logs/`.
