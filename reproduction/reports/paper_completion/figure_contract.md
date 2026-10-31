# Figure contract

The revised runs quantify an explicitly reconstructed protocol; the figures must
not imply reproduction of the authors' original FR3 geometry or headline gains.

- Backend: existing Python/matplotlib pipeline, saved Python preference.
- Adaptation: structural reuse of the earlier local all-case figures; old outputs
  remain unchanged. Field names and units retain their meanings. New per-map IK
  requires loading the actual case goal rather than a scene-wide goal.
- Result grid: all requested cases in the success denominator; all measured
  method runtimes including failures in the ECDF; cost differences only on
  paired independently validated cases. Median/IQR and every available pair are
  shown. These are dependent roadmap/environment seeds, not 400 IID samples.
- Status grid: all 400 cases per family and all five methods. Separate geometry,
  solver, license, missing-plan and program failures. No performance uncertainty
  is appropriate for this census of discrete states.
- Replay: first case by ID in each family, regardless of success; draw saved
  actual inputs and trajectories, label invalid curves and missing plans. The
  two-panel report illustration uses a wider 10.2-inch canvas; it is not a
  journal-sized production figure. FR3 geometry is the disclosed sphere fit.
- Model diagnostic: every one of the 31 measured joint-7 scan positions at fixed
  other joints. Plot measured sphere penetration and independently computed
  official link-5/hand convex-hull separation in separate panels. Zero clipped
  penetration does not mean zero clearance. Home is a separately identified
  configuration; no population uncertainty or continuous-space claim.
- Exports: editable PDF/SVG, PNG 300 dpi and LZW TIFF 600 dpi; sans serif, at least
  5 pt PDF text. Native report widths are retained; no journal production claim.
- QA: source validation, PDF glyph scan, render every final PDF, visually inspect
  every panel, save source hashes and final figure notes.
