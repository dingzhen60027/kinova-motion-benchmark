# Forest reconstruction: saved results

Inputs and unspecified settings are declared in the frozen manifest. These are actual measurements, not the original authors' records.

| Method | Requested | DRM eligible | Solver successes | Independently validated | Mean TO+CS [ms] | Mean validated cost |
|---|---:|---:|---:|---:|---:|---:|
| LSCS | 400 | 391 | 391 | 391 | 2.459 | 10.721 |
| LGCS | 400 | 391 | 391 | 391 | 72.536 | 10.706 |
| SCSTO | 400 | 391 | 391 | 391 | 33.724 | 10.737 |
| NTO_distance | 400 | 391 | 346 | 82 | 34.893 | 10.638 |
| NTO_time | 400 | 391 | 361 | 337 | 238.237 | 10.004 |

Case states: {'completed': 391, 'invalid_seed_path': 9}. Native DRM reported 400/400 solutions; exact seed-path validation accepted 391/400.

All failures and absent outputs remain in methods.csv. Quality comparisons use common independently validated successes in paired_comparisons.csv. Counts from diagnostics or preliminary runs are not pooled.

The original FR3 model/seeds and KUKA hardware recordings are unavailable; this report must not be described as an exact reproduction of the complete paper or its 17.1x claim.
