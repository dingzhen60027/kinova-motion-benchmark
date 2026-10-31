# Figure verification

All four final PDFs were rendered with existing `/usr/bin/pdftoppm` and visually inspected alongside PNG exports. Text, axes, legends, failure styles and the complete case grids are legible. Source checks: 20 PASS, 0 WARN, 0 FAIL. All PDF text audits pass the 5 pt floor. Exact glyph minima, hashes, commands and renderer failures are in `figure_qa.json`.

The first visual review exposed a replaced FR3 trajectory legend; the final figure retains both legends. Rare geometry disagreements are labeled explicitly. The timing figure states the 55 observed wall-time fallbacks and zero case timeouts.

The bundled Poppler binary failed because it requires GLIBC 2.38; the existing Ubuntu system binary rendered all PDFs successfully. No dependencies were installed or changed.
