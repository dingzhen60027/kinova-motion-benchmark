# Independent LGCS evidence audit

Audit passed: **True**. Planned LGCS jobs: 100; audited attempts: 100.

Run: `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/rss2025_formal_v1_main_s002_bottleneck`
Runtime commit: `c8adbd8cef44db25b41559e8fbf2b38e120b624a`

## Recorded outcomes

```json
{
  "latest_status": {
    "success_validated_numerical": 65,
    "unsupported": 15,
    "frontend_failed": 17,
    "infrastructure_error": 3
  },
  "all_attempt_status": {
    "success_validated_numerical": 65,
    "unsupported": 15,
    "frontend_failed": 17,
    "infrastructure_error": 3
  },
  "latest_alternate_status": {
    "no_eligible_interior_region": 51,
    "found": 28,
    "native_no_alternate_path": 1
  },
  "accepted_second_routes": 28,
  "recovery_rounds": 2,
  "recovered_successes": 2,
  "license_limit_failures": 15,
  "unsupported_attempts": 15,
  "native_process_aborts": 3,
  "selected_segments_checked": 154,
  "interpolated_samples_checked": 73800,
  "accepted_pool_original_edges_checked": 281
}
```

## Scope and limits

- No GPU, planner, solver or runtime validator was called.
- Membership and interpolation are independently recomputed from saved arrays; this is not a continuous collision proof.
- Actual collision labels are preserved evidence from the original run and are not recomputed in this CPU audit.
- Saved native/independent labels, signed clearances, ambiguity counts and passed flags are checked for consistency; every success also requires solver success and both frontend gates.
- Same-robot/scene checks use immutable case/model hashes and frozen call sites. The native alternate-roadmap binary is hash-checked; its serializer is not independently decoded.
- Raw cut pools may temporarily lose original path coverage; complete original-edge coverage is independently checked for every subsequent optimizer input pool.
- A final failed solve may retain prior-round top-level validation; this audit labels its scope and never counts it as final success.
