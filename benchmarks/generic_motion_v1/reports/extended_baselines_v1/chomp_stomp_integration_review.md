# Independent extended-method integration review

The reviewed source has no unresolved blocker for the declared smoke/dev study. Four integration issues were found, reported before the study freeze, and corrected by the owning agents. This was a read-only review: the reviewer did not modify runtime code during this pass. The exact reviewed source hashes and 18 checks are recorded in `chomp_stomp_integration_review.json`.

## Findings corrected before freeze

| ID | Severity | Finding and evidence | Verified correction |
|---|---|---|---|
| E1 | High | TrajOpt's endpoint guard used `1e-7` while the common benchmark tolerance is `1e-5`. A read-only scan of 110 previously validated dev frontends found 70 whose native float32 endpoint rounding exceeded `1e-7`; maximum error was `1.1757522377919827e-7 rad`. Those legitimate inputs would have been incorrectly rejected. | The adapter uses `cfg.endpoint_tolerance_rad`, saves endpoint error/tolerance, and preserves the supplied endpoint values. The regression test accepts a legitimate float32 roundtrip and still rejects a real endpoint change. |
| E2 | Medium | A changed accepted TrajOpt iterate could be returned at its internal time cap, while an equivalent iterate at the native SQP/penalty iteration cap was always rejected. | The adapter applies one usable-incumbent rule, records `solver_converged`, `native_status` and `termination`, and requires the common final validator. A returned incumbent is not described as solver convergence. |
| E3 | Medium | The shared TrajOpt dependency probe ignored a configured build directory. Missing dependencies could be misclassified later as a solver failure. | The probe resolves the configured build directory and raises `ImportError` before optimization when unavailable. |
| E4 | Low | An empty list was silently accepted as empty CHOMP/STOMP settings. | Overrides must be dictionaries. |

`chomp_stomp_integration_endpoint_scan.json` preserves every path hash and measured endpoint residual behind E1. The scan read historical raw paths solely to diagnose a numeric interface mismatch; it did not rerun planners or change tasks or algorithm parameters.

## Algorithm and safety-contract checks

STOMP executes the byte-identical official `ros-industrial/stomp` core, including probability computation, rollout update, control cost and stopping logic. Its task callbacks supply canonical geometry, deterministic correlated noise and boundary-preserving filtering. CHOMP executes a covariance-metric descent on its documented discrete C-space line integral; the independent whole-objective gradient test verifies the implemented derivative. It is explicitly distinguished from the original workspace body-point cost.

TrajOpt executes the author's trust-region SQP and BPMPD numerical code. Inspection of the author's `optimizers.cpp` confirms that iteration callbacks receive the accepted `x_`, while trial QP states reside separately in `new_x`; the interrupt-incumbent path therefore does not accidentally publish a rejected trial. The original initializer is excluded from that timeout-return path. The later transport repair loops over partial POSIX pipe reads/writes and bounds them by a monotonic deadline; the exact transport patch is separately hashed, while the SQP and QP numerical routines remain unchanged.

All three local methods use the same validated DRM path. Added knots retain every original corner. They use canonical primitive dimensions, collision filters and full-size online spheres. No method-specific collision robot is introduced. Their native objectives remain separate from Euclidean PWL length. Their capabilities are explicitly restricted to the geometric track; no physical duration is fabricated.

The root runner saves the returned candidate and then applies the existing independent/CUDA validator. A positive native return alone cannot become a benchmark success. A five-second validation reserve, optimizer callbacks, the outer process-group watchdog and the final deadline check prevent a late result from silently exceeding the declared budget. Native termination, final validation and convergence are separately inspectable. Runtime snapshots include recursively vendored C++ headers, licenses and the author TrajOpt source archive.

## Actual checks performed

The following targeted run completed with **103 tests passed**, one existing Drake deprecation warning, in **1.34 seconds**:

```bash
bash local/python.sh -m pytest \
  benchmarks/generic_motion_v1/tests/test_chomp_stomp.py \
  benchmarks/generic_motion_v1/tests/test_trajopt_baseline.py \
  benchmarks/generic_motion_v1/tests/test_extended_integration.py \
  benchmarks/generic_motion_v1/tests/test_extended_metrics.py -q
```

The log is `chomp_stomp_integration_tests.log`. This is CPU numerical/native and mocked integration coverage, not a claim that the full development matrix has completed. The main study still needs its separately frozen execution, common validation, receipts, aggregation and replay inspection.

Finite collision sampling remains a numerical check, not a continuous-safety proof. TrajOpt uses sampled segment constraints rather than the author's OpenRAVE swept-volume checker. These limits are part of the declared adapters and do not justify changing the frozen robot, inputs or success criteria after observing outcomes.
