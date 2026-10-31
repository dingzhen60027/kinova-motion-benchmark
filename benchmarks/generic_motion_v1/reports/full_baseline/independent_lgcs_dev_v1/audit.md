# Independent LGCS evidence audit

Audit passed: **True**. Planned LGCS jobs: 180; audited attempts: 180.

Run: `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/rss2025_full_dev_v1`
Runtime commit: `72984027030175b7bca78e2dcf592b38f0c1864b`

## Recorded outcomes

```json
{
  "latest_status": {
    "frontend_failed": 69,
    "success_validated_numerical": 94,
    "unsupported": 16,
    "infrastructure_error": 1
  },
  "all_attempt_status": {
    "frontend_failed": 69,
    "success_validated_numerical": 94,
    "unsupported": 16,
    "infrastructure_error": 1
  },
  "latest_alternate_status": {
    "found": 33,
    "no_eligible_interior_region": 72,
    "native_no_alternate_path": 5
  },
  "accepted_second_routes": 33,
  "recovery_rounds": 20,
  "recovered_successes": 9,
  "license_limit_failures": 16,
  "unsupported_attempts": 16,
  "native_process_aborts": 1,
  "selected_segments_checked": 296,
  "interpolated_samples_checked": 145223,
  "accepted_pool_original_edges_checked": 476
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
