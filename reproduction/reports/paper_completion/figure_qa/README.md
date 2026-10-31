# Final figure verification

All four final PDFs were rendered with Ubuntu's `/usr/bin/pdftoppm` at 150 dpi and every panel was visually inspected. Titles, legends, units, status colors, sample counts and axes are legible without clipping. The replay legend was added after the first inspection; the final replay explicitly identifies start, IK goal and obstacles. Failed NTO curves remain dashed and labeled. The first frozen FR3 case has no path and is shown as such.

The initial PATH-selected bundled Poppler failed because it requires GLIBC 2.38; the unchanged Ubuntu 22.04 system Poppler succeeded. Both attempts are preserved in `commands.json`; final commands and PDF hashes are in `final_commands.json`. No system packages were changed.

| Figure | Smallest PDF text (pt) | Text runs below 5 pt | Visual review |
|---|---:|---:|---|
| paper_results | 5.6 | 0 | inspected |
| paper_case_statuses | 6.5 | 0 | inspected |
| fr3_scene_replay | 6.5 | 0 | inspected |
| fr3_model_diagnostic | 7.0 | 0 | inspected |

Source preflight: 20 PASS, 0 WARN, 0 FAIL. Quantitative panels use all 400 requested cases or explicitly counted paired subsets. The status panels include every failed/missing plan. Replay filters appended nonfinite-time collision witness rows and records that exclusion in the figure source metadata. The geometry diagnostic plots all 31 saved scan positions; zero penetration is clipped and is not a zero-clearance claim. Figures explicitly label the reconstructed FR3 model and do not establish original-paper reproduction.

Final exports include editable PDF and SVG, PNG at 300 dpi, and LZW TIFF at 600 dpi. The two-scene replay uses the declared wider report canvas, not a journal production width.
