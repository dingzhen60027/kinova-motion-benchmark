# Independent read-only smoke audit

Audited `benchmarks/generic_motion_v1/results/rss2025_smoke_v1` at frozen planning commit `e1abf004b4347dff75b15a8868e14b732d3bf539`. This was a CPU inspection of saved GPU-run artifacts, not a new planning or collision experiment. Runtime source was not modified.

All **20 receipts / 525 sealed files** matched their SHA-256 entries. All **6 successful candidates** were reconstructed: sampled positions and times were bit-identical to saved arrays, optimized segment count equaled active region count, and active mappings partitioned the original DRM edges without gaps. Every stored anchor and shared waypoint satisfied its saved halfspaces.

| Case | Objective | Raw edges / active regions | Cost | Saved samples | Recovery |
|---|---|---:|---:|---:|---:|
| smoke-bottleneck-000-p00 | minimum_distance | 4 / 4 | 7.202973631 rad | 1447 | 3 |
| smoke-bottleneck-000-p00 | minimum_time | 4 / 4 | 5.872210014 s | 2242 | 0 |
| smoke-route_choice-000-p01 | minimum_distance | 3 / 3 | 7.791139617 rad | 1563 | 0 |
| smoke-route_choice-000-p01 | minimum_time | 3 / 3 | 7.303303279 s | 2484 | 0 |
| smoke-open_control-000-p01 | minimum_distance | 1 / 1 | 5.957935187 rad | 1193 | 0 |
| smoke-open_control-000-p01 | minimum_time | 1 / 1 | 4.934426204 s | 1912 | 0 |

All six final trajectories have zero saved native and independent collision detections. Their minimum saved sampled clearances are positive. These are finite-resolution numerical results.

## Independent polynomial checks

SciPy BPoly independently evaluated the three time trajectories and their derivatives on 10,001 points per segment. This is separate from the validator’s Bernstein implementation; it corroborates the saved full-segment derivative-control bounds.

| Case | Maximum sampled speed | Maximum sampled acceleration | Endpoint speed | C0/C1 junction residual |
|---|---:|---:|---:|---:|
| smoke-bottleneck-000-p00 | 0.999999999903 | 1 | 3.997e-16 | 2.721e-10 |
| smoke-route_choice-000-p01 | 1.00000000001 | 1.00000000013 | 9.241e-13 | 5.412e-10 |
| smoke-open_control-000-p01 | 0.933401637689 | 0.999999999959 | 1.193e-12 | 0 |

The tiny values above 1 are below the frozen 1e−5 numerical tolerance. Positions also agree with the saved trajectory to floating-point precision; exact maxima are in the JSON.

## Actual recovery and rejected seed evidence

`smoke-bottleneck-000-p00` minimum-distance required three actual native region edits. Saved native/independent collision counts agree in every round: **86 → 49 → 38 → 0**. Each call used original seed edges `[0, 1, 2, 3]`, with start/end arrays exactly matching the fixed DRM path. All collision witnesses were assigned to active region 1. Native debug arrays have shapes 7×86, 7×49 and 7×38. All 12 handoff checks passed with zero additional offset correction; shared-waypoint checks passed after every edit. The final cost was 7.202973631 rad.

`smoke-open_control-000-p00` was correctly rejected before inflation for both objectives. Native DRM returned success, but the common dense seed check found one collision at sample index 664 of 802 in both native and independent checks. The minimum saved clearance was −0.000239026931 m. Both records preserve native success, mark the accepted frontend false, identify `drm_path_validation` as the failed stage, and contain no optimized candidate.

## Coverage limits

- No successful smoke case actually reused a region across multiple consecutive DRM edges. The new compact-corridor branch has CPU regression evidence, but this GPU smoke does not exercise it.
- The three recovery iterations retained edited regions; none triggered original-span reinflation. That branch likewise remains covered by CPU regression, not this GPU smoke.
- Collision arrays were inspected, not recalculated. Numerical samples are not a continuous collision certificate.
- Cold and warm timings retain their declared meanings; warm is a portion of the same execution, not an independent hot-repeat experiment.

Machine-readable evidence with all inspected paths, source hashes, receipt hashes, array counts and numerical checks: [independent_smoke_audit.json](independent_smoke_audit.json).

No obvious candidate, mapping, seed-gate, recovery, hash-integrity, or metric inconsistency was found in these inspected smoke records.
