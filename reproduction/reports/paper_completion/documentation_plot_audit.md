# Documentation and plot-data consistency review

This is a text/code review of the root README, reproduction README, protocol README, report generator and figure data-selection code. It did not execute plotting, planning, CUDA or a solver. Python AST parsing of `report_completion.py` and `paper_completion/plot.py` completed successfully.

## Confirmed protocol distinctions

- Current NTO retains DRM nodes and duplicates the time spline's endpoint controls. The v3 minimum-eight-control resampling is mentioned only as a removed historical choice.
- Current Forest builds native offline collision maps and injects obstacles online; the old static-Forest backend is not described as current.
- Eq. (19d) is discussed as equality of adjacent path **nodes**, not equality of velocities. Its literal interpretation conflicts with distinct fixed endpoints; the implementation's continuous PWL interpretation remains a declared ambiguity.
- FR3 counts are explicit: 33 total spheres, 32 active spheres, 211 static pairs and 640 dynamic pairs, totaling 851. These are not the paper's reported 1187 pairs.
- CUDA CFR, solver success, their conjunction, and strict independently validated success remain distinct. The figure's three outcome bars are explicitly solver success, solver-plus-CUDA-free and strict success; unconditional CUDA CFR is reported separately in the tables/CSV.
- All native DRM successes remain in conditional-rate denominators. The local strict seed-validation gate skips LSCS/LGCS/SCSTO on invalid native paths while still allowing NTO warm starts; it is explicitly distinguished from an established author acceptance rule.
- The original 17.1× speedup is not claimed reproduced. Historical protocols and results remain separate and unchanged.

## Corrections made

1. The historical-core report branch no longer treats existence of `paperdate_core_result.json` as proof of a completed native build. It reads `compiled` and `status` explicitly. At review time the actual record is `awaiting_build_window`, with `compiled=false`; READMEs state that native compilation and probes are pending. The root task should update that current-status sentence if later execution changes it.
2. TO+CS wording no longer claims all file IO was excluded. Actual construction timing includes alternate-roadmap serialization/loading, alternate-path validation and recovery-coverage record writes. Final-candidate validation and surrounding pre-solve/replay output are outside the ordinary TO+CS timers; exception wall-time fallbacks are separate.
3. Replay plotting excludes rows with non-finite saved time from trajectory polylines. Such rows are appended collision witnesses, not additional path edges. Their exact excluded row count is recorded in each replay's `trajectory_selection`; original failure labels are unchanged.
4. FR3 replay geometry is loaded from the measured run's frozen source snapshot and checked against its manifest hash. Forest uses its frozen configuration. Manifests, case/scene inputs, trajectory arrays and result records are hashed in figure provenance. The displayed table top comes from the measured FR3 configuration.
5. Each plotted method must have exactly the manifest's case IDs, preventing missing/duplicate CSV rows from silently appearing as successful grid cells. Outcome labels use the actual requested-case denominator. Paired costs still require both methods to be strictly validated; displayed intervals are medians/IQR, not iid confidence intervals. Runtime CDFs retain all recorded finite timings, including failures, with observed sample counts.

No rendering or visual-layout validation was performed in this review. Those remain required when the root task generates the actual final figures.
