# Figure contract

The figures describe measured outcomes on a constructed benchmark and the geometric fidelity of its 33-sphere model. They do not establish the original paper speedup or physical robot performance.

Python/matplotlib is the existing and saved backend. Reuse of v4 plotting code is structural: identical methods/case schema and declared transformations, new source files and geometry diagnostics. The v4 figures remain unchanged.

- Results grid: all requested cases, counts and full denominators; all recorded timings including failures; common independently validated cost pairs with all observations, median and IQR. Rate bars enumerate this fixed case grid and are not population estimates with iid confidence intervals.
- Status matrix: every requested case and method, including upstream failures, unattempted methods and interruptions.
- Scene replay: lexicographically first case, selected before filtering outcomes. Only finite saved trajectory times; appended witness rows are excluded with exact counts recorded. Displayed TCP positions come from actual saved joint samples; spheres and obstacles use frozen inputs.
- Geometry: each of 11 links compares measured maximum exterior distance on fixed sphere-surface probes; the second panel retains all1184 preregistered configurations per condition, including hull-only collisions. Convex-hull penetration alone does not prove original nonconvex mesh collision.

PDF/SVG/PNG/TIFF exports retain editable vector text, minimum5pt glyphs and restrained method colors. Source hashes and exclusions are saved. All panels require visual inspection and PDF text checks after rendering; no generated experimental values are permitted.
