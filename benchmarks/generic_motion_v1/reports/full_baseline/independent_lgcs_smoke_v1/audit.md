# Independent LGCS evidence audit

Audit passed: **True**. Planned LGCS jobs: 10; audited attempts: 10.

Run: `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/rss2025_full_smoke_v1`
Runtime commit: `72984027030175b7bca78e2dcf592b38f0c1864b`

## Recorded outcomes

```json
{
  "latest_status": {
    "frontend_failed": 5,
    "invalid_endpoint": 2,
    "unsupported": 2,
    "success_validated_numerical": 1
  },
  "all_attempt_status": {
    "frontend_failed": 5,
    "invalid_endpoint": 2,
    "unsupported": 2,
    "success_validated_numerical": 1
  },
  "latest_alternate_status": {
    "found": 2,
    "no_eligible_interior_region": 1
  },
  "accepted_second_routes": 2,
  "recovery_rounds": 0,
  "recovered_successes": 0,
  "license_limit_failures": 2,
  "unsupported_attempts": 2,
  "native_process_aborts": 0,
  "selected_segments_checked": 1,
  "interpolated_samples_checked": 1193,
  "accepted_pool_original_edges_checked": 16
}
```

## Scope and limits

- No GPU, planner, solver or runtime validator was called.
- Membership and interpolation are independently recomputed from saved arrays; this is not a continuous collision proof.
- Actual collision labels are preserved evidence from the original run and are not recomputed in this CPU audit.
- Same-robot/scene checks use immutable case/model hashes and frozen call sites. The native alternate-roadmap binary is hash-checked; its serializer is not independently decoded.
- Raw cut pools may temporarily lose original path coverage; complete original-edge coverage is independently checked for every subsequent optimizer input pool.
- A final failed solve may retain prior-round top-level validation; this audit labels its scope and never counts it as final success.
