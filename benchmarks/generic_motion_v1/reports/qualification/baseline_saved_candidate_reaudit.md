# Historical successful-candidate contract re-audit

CPU contract re-audit of every sealed historical successful candidate; unchanged original files. Collision arrays are reused only at bit-identical q samples. No fresh collision checking or GPU planning was performed. SciPy independent position/derivative checks do not certify continuous collision freedom.

| Run | Successful candidates | Passed | Rejected |
|---|---:|---:|---:|
| smoke_v3_fixed | 8 | 8 | 0 |
| dev_eizo_v2 | 198 | 198 | 0 |
| dev_paired_v2 | 24 | 24 | 0 |

Elapsed CPU audit wall time: 2.367 s.

Full per-attempt hashes and numerical checks: [baseline_saved_candidate_reaudit.json](baseline_saved_candidate_reaudit.json).
